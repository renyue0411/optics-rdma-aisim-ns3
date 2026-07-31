#include "rdma-transport.h"

#include "ns3/simulator.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/qbb-net-device.h"

#include <algorithm>
#include <iostream>
#include <limits>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("RdmaTransport");
NS_OBJECT_ENSURE_REGISTERED(RdmaTransport);

TypeId
RdmaTransport::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::RdmaTransport")
            .SetParent<Object>()
            .AddConstructor<RdmaTransport>();

    return tid;
}

RdmaTransport::RdmaTransport()
    : m_node(NULL),
      m_rdma(NULL),
      m_mode(MODE_DEFAULT),
      m_enabled(false),
      m_gateEnabled(false),
      m_wrChunkBytes(16 * 1024),
      m_maxOutstandingBytes(64 * 1024),
      m_safeRateBps(30000000000ULL),
      m_tailGuardNs(80000),
      m_minPostBytes(8 * 1024),
      m_minSafeRateBps(8000000000ULL),
      m_maxSafeRateBps(30000000000ULL),
      m_minTailGuardNs(80000),
      m_maxTailGuardNs(800000),
      m_minOutstandingBytes(32 * 1024),
      m_maxOutstandingCeilingBytes(256 * 1024),
      m_adaptPeriodNs(30000000),
      m_lastAdaptNs(0),
      m_stableAdaptPeriods(0),
      m_recoverySinceLastAdapt(false),
      m_ackProgressSinceLastAdapt(false),
      m_backlogSinceLastAdapt(false),
      m_autoBandwidthConfig(true),
      m_realDeploymentMode(false),
      m_bottleneckRateBps(0),
      m_switchingGuardNs(0),
      m_maxObservedRttNs(10000),
      m_activeQpHint(1),
      m_postLogSampleLimit(0)
{
}

void
RdmaTransport::SetNode(Ptr<Node> node)
{
    m_node = node;
}

void
RdmaTransport::SetRdmaHw(Ptr<RdmaHw> rdma)
{
    m_rdma = rdma;
}

void
RdmaTransport::SetMode(uint32_t mode)
{
    NS_ASSERT_MSG(mode <= MODE_USERSPACE,
                  "RDMA transport mode must be 0(default), 1(RNIC), or 2(userspace)");

    m_mode = static_cast<GateMode>(mode);
    m_enabled = (m_mode == MODE_USERSPACE);

    if (m_rdma == NULL)
    {
        return;
    }

    if (m_mode == MODE_RNIC)
    {
        m_rdma->SetRnicGateCallbacks(
            MakeCallback(&RdmaTransport::RnicGateAllowsQp, this),
            MakeCallback(&RdmaTransport::GetNextRnicGateTime, this));
    }
    else
    {
        m_rdma->ClearRnicGateCallbacks();
    }
}

RdmaTransport::GateMode
RdmaTransport::GetMode() const
{
    return m_mode;
}

void
RdmaTransport::SetEnabled(bool enabled)
{
    SetMode(enabled ? MODE_USERSPACE : MODE_DEFAULT);
}

void
RdmaTransport::Configure(
    uint64_t wrChunkBytes,
    uint64_t maxOutstandingBytes)
{
    NS_ASSERT_MSG(
        wrChunkBytes > 0,
        "userspace WR chunk must be positive");

    NS_ASSERT_MSG(
        maxOutstandingBytes >= wrChunkBytes,
        "maximum outstanding bytes must be >= WR chunk");

    m_wrChunkBytes = wrChunkBytes;
    m_maxOutstandingBytes = maxOutstandingBytes;

    m_minOutstandingBytes = std::min<uint64_t>(
        32 * 1024,
        m_maxOutstandingBytes);

    m_maxOutstandingCeilingBytes = std::max<uint64_t>(
        m_maxOutstandingBytes,
        256 * 1024);

    ApplyBandwidthNormalizedConfig("configure");
}


void
RdmaTransport::ConfigureBandwidthNormalized(
    uint64_t bottleneckRateBps,
    uint64_t switchingGuardNs,
    uint64_t maxRttNs,
    uint32_t activeQpHint,
    bool realDeploymentMode)
{
    m_autoBandwidthConfig = true;
    m_realDeploymentMode = realDeploymentMode;

    if (bottleneckRateBps > 0)
    {
        m_bottleneckRateBps = bottleneckRateBps;
    }

    if (switchingGuardNs > 0)
    {
        m_switchingGuardNs = switchingGuardNs;
    }

    if (maxRttNs > 0)
    {
        m_maxObservedRttNs = maxRttNs;
    }

    m_activeQpHint = std::max<uint32_t>(1, activeQpHint);

    ApplyBandwidthNormalizedConfig("explicit_config");
}

uint64_t
RdmaTransport::ClampValue(
    uint64_t value,
    uint64_t minValue,
    uint64_t maxValue) const
{
    if (maxValue < minValue)
    {
        maxValue = minValue;
    }

    if (value < minValue)
    {
        return minValue;
    }

    if (value > maxValue)
    {
        return maxValue;
    }

    return value;
}

uint64_t
RdmaTransport::RoundUpPowerOfTwo(uint64_t value) const
{
    if (value <= 1)
    {
        return 1;
    }

    value--;
    value |= value >> 1;
    value |= value >> 2;
    value |= value >> 4;
    value |= value >> 8;
    value |= value >> 16;
    value |= value >> 32;
    value++;

    return value;
}

uint64_t
RdmaTransport::GetLocalBottleneckRateBps() const
{
    if (m_node == NULL)
    {
        return 0;
    }

    uint64_t bestRate = std::numeric_limits<uint64_t>::max();

    for (uint32_t i = 0; i < m_node->GetNDevices(); ++i)
    {
        Ptr<QbbNetDevice> dev =
            DynamicCast<QbbNetDevice>(m_node->GetDevice(i));

        if (dev == 0)
        {
            continue;
        }

        uint64_t rate = dev->GetDataRate().GetBitRate();
        if (rate > 0)
        {
            bestRate = std::min(bestRate, rate);
        }
    }

    if (bestRate == std::numeric_limits<uint64_t>::max())
    {
        return 0;
    }

    return bestRate;
}

uint64_t
RdmaTransport::InferSwitchingGuardNs() const
{
    uint64_t minSlotNs = std::numeric_limits<uint64_t>::max();
    for (std::map<uint32_t, GateTable>::const_iterator tableIt =
             m_gateTables.begin();
         tableIt != m_gateTables.end();
         ++tableIt)
    {
        for (uint32_t i = 0; i < tableIt->second.slots.size(); ++i)
        {
            const RdmaTransport::GateSlotEntry& slot =
                tableIt->second.slots[i];
            if (slot.endOffsetNs > slot.startOffsetNs)
            {
                minSlotNs = std::min(
                    minSlotNs,
                    slot.endOffsetNs - slot.startOffsetNs);
            }
        }
    }

    if (minSlotNs == std::numeric_limits<uint64_t>::max() ||
        minSlotNs > 1000000ULL)
    {
        return 10000ULL;
    }
    return std::max<uint64_t>(10000ULL, minSlotNs);
}

void
RdmaTransport::ApplyBandwidthNormalizedConfig(const char* reason)
{
    if (!m_autoBandwidthConfig)
    {
        return;
    }

    uint64_t bottleneckRateBps = m_bottleneckRateBps;
    if (bottleneckRateBps == 0)
    {
        bottleneckRateBps = GetLocalBottleneckRateBps();
    }

    if (bottleneckRateBps == 0)
    {
        return;
    }

    m_bottleneckRateBps = bottleneckRateBps;

    uint64_t effectiveSwitchingGuardNs = m_switchingGuardNs;

    if (effectiveSwitchingGuardNs == 0 && !m_gateTables.empty())
    {
        m_switchingGuardNs = InferSwitchingGuardNs();
        effectiveSwitchingGuardNs = m_switchingGuardNs;
    }

    if (effectiveSwitchingGuardNs == 0)
    {
        effectiveSwitchingGuardNs = 10000ULL;
    }

    if (m_maxObservedRttNs == 0)
    {
        m_maxObservedRttNs = 10000ULL;
    }

    uint32_t activeQps = std::max<uint32_t>(1, m_activeQpHint);

    uint64_t safeFactorPermille =
        m_realDeploymentMode ? 300ULL : 350ULL;

    uint64_t minFactorPermille = 200ULL;
    uint64_t maxFactorPermille = 900ULL;

    uint64_t oldSafeRateBps = m_safeRateBps;
    uint64_t oldTailGuardNs = m_tailGuardNs;
    uint64_t oldMaxOutstandingBytes = m_maxOutstandingBytes;
    uint64_t oldWrChunkBytes = m_wrChunkBytes;
    uint64_t oldMinPostBytes = m_minPostBytes;

    long double rateBase =
        static_cast<long double>(bottleneckRateBps) /
        static_cast<long double>(activeQps);

    uint64_t targetSafeRateBps =
        static_cast<uint64_t>(
            rateBase *
            static_cast<long double>(safeFactorPermille) /
            1000.0L);

    m_minSafeRateBps =
        static_cast<uint64_t>(
            rateBase *
            static_cast<long double>(minFactorPermille) /
            1000.0L);

    m_maxSafeRateBps =
        static_cast<uint64_t>(
            rateBase *
            static_cast<long double>(maxFactorPermille) /
            1000.0L);

    m_minSafeRateBps = std::max<uint64_t>(
        m_minSafeRateBps,
        1000000000ULL);

    m_maxSafeRateBps = std::max<uint64_t>(
        m_maxSafeRateBps,
        m_minSafeRateBps);

    m_safeRateBps = ClampValue(
        targetSafeRateBps,
        m_minSafeRateBps,
        m_maxSafeRateBps);

    uint64_t bdpBytes =
        static_cast<uint64_t>(
            static_cast<long double>(m_safeRateBps) *
            static_cast<long double>(m_maxObservedRttNs) /
            8.0L /
            1000000000.0L);

    uint64_t outstandingTarget = std::max<uint64_t>(
        16 * 1024,
        static_cast<uint64_t>(
            static_cast<long double>(bdpBytes) * 0.5L));

    uint64_t outstandingBytes = RoundUpPowerOfTwo(outstandingTarget);

    m_minOutstandingBytes = std::min<uint64_t>(
        32 * 1024,
        outstandingBytes);

    m_maxOutstandingBytes = ClampValue(
        outstandingBytes,
        m_minOutstandingBytes,
        4 * 1024 * 1024);

    uint64_t outstandingCeiling = RoundUpPowerOfTwo(
        std::max<uint64_t>(
            m_maxOutstandingBytes,
            m_maxOutstandingBytes * 4));

    m_maxOutstandingCeilingBytes = ClampValue(
        outstandingCeiling,
        m_maxOutstandingBytes,
        4 * 1024 * 1024);

    m_wrChunkBytes = std::min<uint64_t>(
        16 * 1024,
        m_maxOutstandingBytes);

    m_minPostBytes = 4 * 1024;

    uint64_t drainNs =
        static_cast<uint64_t>(
            static_cast<long double>(m_maxOutstandingBytes) *
            8.0L *
            1000000000.0L /
            static_cast<long double>(m_safeRateBps));

    uint64_t marginNs =
        m_realDeploymentMode ? 500000ULL : 500000ULL;

    m_minTailGuardNs =
        m_realDeploymentMode ? 500000ULL : 500000ULL;

    m_maxTailGuardNs =
        m_realDeploymentMode ? 3000000ULL : 3000000ULL;

    uint64_t targetTailGuardNs =
        effectiveSwitchingGuardNs +
        m_maxObservedRttNs +
        drainNs +
        marginNs;

    m_tailGuardNs = ClampValue(
        targetTailGuardNs,
        m_minTailGuardNs,
        m_maxTailGuardNs);

    bool changed =
        oldSafeRateBps != m_safeRateBps ||
        oldTailGuardNs != m_tailGuardNs ||
        oldMaxOutstandingBytes != m_maxOutstandingBytes ||
        oldWrChunkBytes != m_wrChunkBytes ||
        oldMinPostBytes != m_minPostBytes;

    // The transport object is initialized for every RDMA mode, but its
    // bandwidth/admission diagnostics are meaningful only in userspace mode.
    if (changed && m_enabled)
    {
        std::cout
            << "[USERSPACE BW INIT]"
            << " reason=" << (reason != 0 ? reason : "unknown")
            << " gate_tables=" << m_gateTables.size()
            << " bottleneckRateBps=" << bottleneckRateBps
            << " activeQpHint=" << activeQps
            << " safeFactorPermille=" << safeFactorPermille
            << " minSafeRateBps=" << m_minSafeRateBps
            << " safeRateBps=" << m_safeRateBps
            << " maxSafeRateBps=" << m_maxSafeRateBps
            << " maxObservedRttNs=" << m_maxObservedRttNs
            << " switchingGuardNs=" << effectiveSwitchingGuardNs
            << " maxOutstandingBytes=" << m_maxOutstandingBytes
            << " maxOutstandingCeilingBytes=" << m_maxOutstandingCeilingBytes
            << " wrChunkBytes=" << m_wrChunkBytes
            << " minPostBytes=" << m_minPostBytes
            << " tailGuardNs=" << m_tailGuardNs
            << " realMode=" << (m_realDeploymentMode ? 1 : 0)
            << std::endl;
    }
}

void
RdmaTransport::InstallGateTable(
    uint32_t rnicId,
    uint64_t epochStartNs,
    uint64_t periodNs,
    const std::vector<RdmaTransport::GateSlotEntry>& slots)
{
    NS_ASSERT_MSG(periodNs > 0,
                  "userspace injection period must be positive");

    GateTable table;
    table.epochStartNs = epochStartNs;
    table.periodNs = periodNs;
    table.slots = slots;
    m_gateTables[rnicId] = table;
    m_gateEnabled = !m_gateTables.empty();
    m_adaptPeriodNs = periodNs;
    m_lastAdaptNs =
        static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());

    ApplyBandwidthNormalizedConfig("enable_gate");

    std::cout << "[RDMA TRANSPORT GATE INSTALLED]"
              << " mode=" << static_cast<uint32_t>(m_mode)
              << " rnic_port=" << rnicId
              << " epochNs=" << epochStartNs
              << " periodNs=" << periodNs
              << " slots=" << slots.size()
              << " tables=" << m_gateTables.size()
              << std::endl;
}

void
RdmaTransport::ClearGateTables()
{
    m_gateEnabled = false;
    m_gateTables.clear();

    for (std::map<uint64_t, EventId>::iterator it =
             m_wakeEvents.begin();
         it != m_wakeEvents.end();
         ++it)
    {
        if (it->second.IsRunning())
        {
            it->second.Cancel();
        }
    }
    m_wakeEvents.clear();
}

bool
RdmaTransport::RnicGateAllowsQp(Ptr<RdmaQueuePair> qp) const
{
    if (m_mode != MODE_RNIC)
    {
        return true;
    }
    return Allows(qp, Simulator::Now());
}

Time
RdmaTransport::GetNextRnicGateTime(Ptr<RdmaQueuePair> qp) const
{
    if (m_mode != MODE_RNIC)
    {
        return Simulator::Now();
    }
    return GetNextAllowedTime(qp, Simulator::Now());
}

RdmaTransport::GateLookupResult
RdmaTransport::LookupGate(
    Ptr<RdmaQueuePair> qp,
    Time now) const
{
    GateLookupResult result;
    result.currentWindowEnd = now;
    result.nextAllowedTime = Simulator::GetMaximumSimulationTime();

    if (!m_gateEnabled || qp == NULL || !qp->m_hasBoundRnicPort)
    {
        result.bypass = true;
        result.allowed = true;
        result.currentWindowEnd = Simulator::GetMaximumSimulationTime();
        result.nextAllowedTime = now;
        return result;
    }

    std::map<uint32_t, GateTable>::const_iterator tableIt =
        m_gateTables.find(qp->m_boundRnicPort);
    if (tableIt == m_gateTables.end() || tableIt->second.periodNs == 0)
    {
        return result;
    }

    const GateTable& table = tableIt->second;
    const uint32_t dstNodeId = qp->GetDest();
    const uint32_t wordIndex = dstNodeId / 64;
    const uint32_t bitIndex = dstNodeId % 64;
    const uint64_t nowNs = static_cast<uint64_t>(now.GetNanoSeconds());
    const uint64_t offsetNs = nowNs >= table.epochStartNs
        ? (nowNs - table.epochStartNs) % table.periodNs
        : (table.periodNs -
           ((table.epochStartNs - nowNs) % table.periodNs)) % table.periodNs;
    const uint64_t periodBaseNs = nowNs - offsetNs;
    uint64_t bestDeltaNs = std::numeric_limits<uint64_t>::max();

    for (uint32_t i = 0; i < table.slots.size(); ++i)
    {
        const GateSlotEntry& slot = table.slots[i];
        if (wordIndex >= slot.dstRnicBitmapWords.size() ||
            (slot.dstRnicBitmapWords[wordIndex] & (1ULL << bitIndex)) == 0)
        {
            continue;
        }

        if (offsetNs >= slot.startOffsetNs && offsetNs < slot.endOffsetNs)
        {
            result.allowed = true;
            result.currentWindowEnd = NanoSeconds(periodBaseNs + slot.endOffsetNs);
            result.nextAllowedTime = now;
            return result;
        }

        const uint64_t deltaNs = offsetNs < slot.startOffsetNs
            ? slot.startOffsetNs - offsetNs
            : table.periodNs - offsetNs + slot.startOffsetNs;
        bestDeltaNs = std::min(bestDeltaNs, deltaNs);
    }

    if (bestDeltaNs != std::numeric_limits<uint64_t>::max())
    {
        result.nextAllowedTime = now + NanoSeconds(bestDeltaNs);
    }

    return result;
}

bool
RdmaTransport::Allows(
    Ptr<RdmaQueuePair> qp,
    Time now) const
{
    return LookupGate(qp, now).allowed;
}

Time
RdmaTransport::GetNextAllowedTime(
    Ptr<RdmaQueuePair> qp,
    Time now) const
{
    return LookupGate(qp, now).nextAllowedTime;
}

uint64_t
RdmaTransport::GetSafeBudgetBytes(
    const GateLookupResult& gate,
    Time now) const
{
    if (gate.bypass)
    {
        return m_maxOutstandingBytes;
    }

    if (!gate.allowed || gate.currentWindowEnd <= now)
    {
        return 0;
    }

    const uint64_t nowNs =
        static_cast<uint64_t>(now.GetNanoSeconds());
    const uint64_t endNs =
        static_cast<uint64_t>(gate.currentWindowEnd.GetNanoSeconds());

    if (endNs <= nowNs + m_tailGuardNs)
    {
        return 0;
    }

    const uint64_t usableNs = endNs - nowNs - m_tailGuardNs;

    // bytes = bps * ns / 8 / 1e9
    return (m_safeRateBps * usableNs) / 8000000000ULL;
}

uint64_t
RdmaTransport::GetAdaptivePeriodNs() const
{
    return m_adaptPeriodNs > 0
        ? m_adaptPeriodNs
        : 30000000ULL;
}

void
RdmaTransport::MaybeAdapt(Ptr<RdmaQueuePair> qp)
{
    if (!m_enabled)
    {
        return;
    }

    uint64_t nowNs =
        static_cast<uint64_t>(
            Simulator::Now().GetNanoSeconds());

    uint64_t periodNs = GetAdaptivePeriodNs();

    bool urgent = m_recoverySinceLastAdapt;

    // Recovery is a safety signal and should react quickly.  Loss-free
    // increase is evaluated only once per OCS period to avoid oscillation.
    if (!urgent &&
        m_lastAdaptNs > 0 &&
        nowNs < m_lastAdaptNs + periodNs)
    {
        return;
    }

    // Avoid repeated multiplicative decrease for a burst of NACKs that belong
    // to the same loss episode.
    if (urgent &&
        m_lastAdaptNs > 0 &&
        nowNs < m_lastAdaptNs + 1000000ULL)
    {
        return;
    }

    uint64_t oldSafeRateBps = m_safeRateBps;
    uint64_t oldTailGuardNs = m_tailGuardNs;
    uint64_t oldMaxOutstandingBytes = m_maxOutstandingBytes;

    const char* action = "hold";

    if (m_recoverySinceLastAdapt)
    {
        // Multiplicative decrease on recovery/retransmission signal.
        m_safeRateBps = std::max<uint64_t>(
            m_minSafeRateBps,
            (m_safeRateBps * 8) / 10);

        m_tailGuardNs = std::min<uint64_t>(
            m_maxTailGuardNs,
            m_tailGuardNs + 25000ULL);

        m_maxOutstandingBytes = std::max<uint64_t>(
            m_minOutstandingBytes,
            (m_maxOutstandingBytes * 8) / 10);

        if (m_maxOutstandingBytes < m_wrChunkBytes)
        {
            m_maxOutstandingBytes = m_wrChunkBytes;
        }

        m_stableAdaptPeriods = 0;
        action = "decrease";
    }
    else if (false &&
             m_backlogSinceLastAdapt &&
             m_ackProgressSinceLastAdapt)
    {
        // Additive/slow increase when the flow is backlogged and the previous
        // period had no recovery.  This improves utilization without relying on
        // AI/RL.
        m_stableAdaptPeriods++;

        if (m_stableAdaptPeriods >= 2)
        {
            m_safeRateBps = std::min<uint64_t>(
                m_maxSafeRateBps,
                (m_safeRateBps * 105) / 100);

            if (m_tailGuardNs > m_minTailGuardNs)
            {
                uint64_t dec = 5000ULL;
                m_tailGuardNs =
                    m_tailGuardNs > m_minTailGuardNs + dec
                        ? m_tailGuardNs - dec
                        : m_minTailGuardNs;
            }

            if ((m_stableAdaptPeriods % 3) == 0)
            {
                m_maxOutstandingBytes = std::min<uint64_t>(
                    m_maxOutstandingCeilingBytes,
                    m_maxOutstandingBytes + 8 * 1024);
            }

            action = "increase";
        }
    }
    else
    {
        m_stableAdaptPeriods = 0;
    }

    if (oldSafeRateBps != m_safeRateBps ||
        oldTailGuardNs != m_tailGuardNs ||
        oldMaxOutstandingBytes != m_maxOutstandingBytes)
    {
        uint32_t srcNodeId =
            m_node != NULL
                ? m_node->GetId()
                : 0;

        uint32_t dstNodeId = 0;
        uint64_t unposted = 0;
        uint64_t outstanding = 0;

        if (qp != NULL)
        {
            dstNodeId = qp->GetDest();
            unposted = qp->GetUnpostedBytes();
            outstanding = qp->GetPostedOutstandingBytes();
        }

        std::cout
            << "[USERSPACE ADAPT]"
            << " t=" << nowNs
            << " rnic=" << srcNodeId
            << " action=" << action
            << " dst=" << dstNodeId
            << " oldSafeRateBps=" << oldSafeRateBps
            << " newSafeRateBps=" << m_safeRateBps
            << " oldTailGuardNs=" << oldTailGuardNs
            << " newTailGuardNs=" << m_tailGuardNs
            << " oldMaxOutstandingBytes=" << oldMaxOutstandingBytes
            << " newMaxOutstandingBytes=" << m_maxOutstandingBytes
            << " recovery=" << (m_recoverySinceLastAdapt ? 1 : 0)
            << " ackProgress=" << (m_ackProgressSinceLastAdapt ? 1 : 0)
            << " backlog=" << (m_backlogSinceLastAdapt ? 1 : 0)
            << " stablePeriods=" << m_stableAdaptPeriods
            << " unposted=" << unposted
            << " outstanding=" << outstanding
            << std::endl;
    }

    m_lastAdaptNs = nowNs;
    m_recoverySinceLastAdapt = false;
    m_ackProgressSinceLastAdapt = false;
    m_backlogSinceLastAdapt = false;
}

void
RdmaTransport::ScheduleNextWake(
    Ptr<RdmaQueuePair> qp,
    Time wakeTime)
{
    if (qp == NULL ||
        wakeTime ==
            Simulator::GetMaximumSimulationTime())
    {
        return;
    }

    Time now = Simulator::Now();

    if (wakeTime <= now)
    {
        return;
    }

    uint64_t key =
        RdmaHw::GetQpKey(
            qp->dip.Get(),
            qp->sport,
            qp->m_pg);

    std::map<uint64_t, EventId>::iterator it =
        m_wakeEvents.find(key);

    if (it != m_wakeEvents.end() &&
        it->second.IsRunning())
    {
        return;
    }

    m_wakeEvents[key] =
        Simulator::Schedule(
            wakeTime - now,
            &RdmaTransport::TrySubmit,
            this,
            qp);
}

uint64_t
RdmaTransport::GetPostStatsKey(
    Ptr<RdmaQueuePair> qp) const
{
    NS_ASSERT(qp != NULL);

    return RdmaHw::GetQpKey(
        qp->dip.Get(),
        qp->sport,
        qp->m_pg);
}

void
RdmaTransport::RecordPost(
    Ptr<RdmaQueuePair> qp,
    uint64_t bytes,
    uint64_t safeBudget,
    Time windowEnd)
{
    if (qp == NULL)
    {
        return;
    }

    uint64_t key = GetPostStatsKey(qp);
    PostStats& st = m_postStats[key];

    uint64_t nowNs =
        static_cast<uint64_t>(
            Simulator::Now().GetNanoSeconds());

    if (st.postCount == 0)
    {
        st.firstPostTimeNs = nowNs;
        st.minBytes = bytes;
        st.maxBytes = bytes;
    }
    else
    {
        st.minBytes = std::min(st.minBytes, bytes);
        st.maxBytes = std::max(st.maxBytes, bytes);
    }

    st.postCount++;
    st.totalBytes += bytes;
    st.lastPostTimeNs = nowNs;

    if (safeBudget > 0 && bytes >= safeBudget)
    {
        st.safeBudgetLimitedCount++;
    }

    if (st.sampleCount < m_postLogSampleLimit)
    {
        uint32_t srcNodeId =
            m_node != NULL
                ? m_node->GetId()
                : 0;

        uint32_t dstNodeId =
            qp->GetDest();

        std::cout
            << "[USERSPACE WR POST SAMPLE]"
            << " t=" << nowNs
            << " src=" << srcNodeId
            << " dst=" << dstNodeId
            << " sport=" << qp->sport
            << " dport=" << qp->dport
            << " pg=" << qp->m_pg
            << " bytes=" << bytes
            << " postedLimit=" << qp->GetPostedLimit()
            << " outstanding=" << qp->GetPostedOutstandingBytes()
            << " safeBudget=" << safeBudget
            << " windowEnd=" << windowEnd.GetNanoSeconds()
            << " sample=" << (st.sampleCount + 1)
            << " sample_limit=" << m_postLogSampleLimit
            << std::endl;

        st.sampleCount++;
    }
}

void
RdmaTransport::FlushPostSummary(
    Ptr<RdmaQueuePair> qp)
{
    if (qp == NULL)
    {
        return;
    }

    uint64_t key = GetPostStatsKey(qp);

    std::map<uint64_t, PostStats>::iterator it =
        m_postStats.find(key);

    if (it == m_postStats.end() ||
        it->second.postCount == 0)
    {
        return;
    }

    PostStats st = it->second;
    m_postStats.erase(it);

    uint32_t srcNodeId =
        m_node != NULL
            ? m_node->GetId()
            : 0;

    uint32_t dstNodeId =
        qp->GetDest();

    uint64_t avgBytes =
        st.postCount > 0
            ? st.totalBytes / st.postCount
            : 0;

    std::cout
        << "[USERSPACE WR SUMMARY]"
        << " t=" << Simulator::Now().GetNanoSeconds()
        << " src=" << srcNodeId
        << " dst=" << dstNodeId
        << " sport=" << qp->sport
        << " dport=" << qp->dport
        << " pg=" << qp->m_pg
        << " posts=" << st.postCount
        << " total_bytes=" << st.totalBytes
        << " min_bytes=" << st.minBytes
        << " max_bytes=" << st.maxBytes
        << " avg_bytes=" << avgBytes
        << " first_post=" << st.firstPostTimeNs
        << " last_post=" << st.lastPostTimeNs
        << " safe_budget_limited=" << st.safeBudgetLimitedCount
        << " sample_limit=" << m_postLogSampleLimit
        << std::endl;
}

void
RdmaTransport::TrySubmit(
    Ptr<RdmaQueuePair> qp)
{
    if (!m_enabled ||
        qp == NULL ||
        m_rdma == NULL ||
        qp->IsFinished() ||
        qp->GetUnpostedBytes() == 0)
    {
        return;
    }

    if (qp->GetUnpostedBytes() > 0)
    {
        m_backlogSinceLastAdapt = true;
    }

    uint64_t outstanding = qp->GetPostedOutstandingBytes();

    while (qp->GetUnpostedBytes() > 0)
    {
        const Time loopNow = Simulator::Now();
        const GateLookupResult gate = LookupGate(qp, loopNow);

        if (!gate.allowed)
        {
            ScheduleNextWake(qp, gate.nextAllowedTime);
            break;
        }

        const uint64_t safeBudget = GetSafeBudgetBytes(gate, loopNow);

        if (safeBudget < m_minPostBytes)
        {
            m_backlogSinceLastAdapt = true;

            if (gate.currentWindowEnd > loopNow &&
                gate.currentWindowEnd != Simulator::GetMaximumSimulationTime())
            {
                const GateLookupResult nextGate =
                    LookupGate(qp, gate.currentWindowEnd + NanoSeconds(1));
                ScheduleNextWake(qp, nextGate.nextAllowedTime);
            }

            break;
        }

        const uint64_t outstandingHeadroom =
            m_maxOutstandingBytes > outstanding
                ? m_maxOutstandingBytes - outstanding
                : 0;

        if (outstandingHeadroom < m_minPostBytes)
        {
            m_backlogSinceLastAdapt = true;
            break;
        }

        const uint64_t unpostedBytes = qp->GetUnpostedBytes();

        uint64_t nextWrBytes = std::min(m_wrChunkBytes, unpostedBytes);
        nextWrBytes = std::min(nextWrBytes, outstandingHeadroom);
        nextWrBytes = std::min(nextWrBytes, safeBudget);

        const bool isFinalTail = nextWrBytes == unpostedBytes;

        if (nextWrBytes < m_minPostBytes && !isFinalTail)
        {
            break;
        }

        m_rdma->PostWork(qp, nextWrBytes);
        outstanding += nextWrBytes;

        // Simulator time does not advance inside PostWork(), so the lookup
        // result obtained for this admission decision remains valid.
        RecordPost(
            qp,
            nextWrBytes,
            safeBudget,
            gate.currentWindowEnd);
    }

    MaybeAdapt(qp);
}

void
RdmaTransport::RegisterQp(
    Ptr<RdmaQueuePair> qp)
{
    NS_ASSERT(qp != NULL);

    if (qp->m_baseRtt > m_maxObservedRttNs)
    {
        m_maxObservedRttNs = qp->m_baseRtt;
        ApplyBandwidthNormalizedConfig("qp_base_rtt");
    }

    TrySubmit(qp);
}

void
RdmaTransport::NotifyAckProgress(
    Ptr<RdmaQueuePair> qp)
{
    if (!m_enabled)
    {
        return;
    }

    if (qp != NULL)
    {
        m_ackProgressSinceLastAdapt = true;

        if (qp->GetUnpostedBytes() > 0)
        {
            m_backlogSinceLastAdapt = true;
        }
    }

    MaybeAdapt(qp);

    if (qp != NULL && qp->IsFinished())
    {
        FlushPostSummary(qp);
        return;
    }

    TrySubmit(qp);

    if (qp != NULL && qp->IsFinished())
    {
        FlushPostSummary(qp);
    }
}

void
RdmaTransport::NotifyRecover(
    Ptr<RdmaQueuePair> qp)
{
    if (!m_enabled)
    {
        return;
    }

    m_recoverySinceLastAdapt = true;

    if (qp != NULL &&
        (qp->GetUnpostedBytes() > 0 ||
         qp->GetPostedOutstandingBytes() > 0))
    {
        m_backlogSinceLastAdapt = true;
    }

    MaybeAdapt(qp);
}

} // namespace ns3