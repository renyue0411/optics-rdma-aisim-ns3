#include "rdma-transport.h"

#include "ns3/simulator.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/qbb-net-device.h"

#include <algorithm>
#include <iostream>
#include <limits>

// MODE2_CQE_SEMANTICS_V1: Mode-2 completion path models signaled WR + CQE.
// MODE2_DEFAULT_PIPELINE_V1: default Mode 2 can bypass OCS admission and pipeline WRs.
// MODE2_INJECTION_WINDOW_PIPELINE_V1: OCS window admission preserves multi-WR pipelining.
// MODE2_OPTIMIZED_GUARD_V1: stable-end + CQE-jitter userspace safety boundary.
// MODE2_PER_PORT_TIMING_V1: isolate Mode-2 rate/CQE state by breakout RNIC port.
// MODE2_PER_PORT_AGGREGATE_ADMISSION_V1: account all in-flight WR bytes per breakout port.
// MODE2_PORT_QP_LOGGING_V1: separate configured QP hint from runtime per-port QP count.
// MODE1_CONTINUATION_ACK_RECOVERY_V1: bounded next-window DATA probing.

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
      m_userspaceAdmissionMode(USERSPACE_OCS_WINDOWED),
      m_wrChunkBytes(16 * 1024),
      m_maxOutstandingBytes(64 * 1024),
      m_safeRateBps(30000000000ULL),
      m_tailGuardNs(80000),
      m_minPostBytes(8 * 1024),
      m_userspaceSoftwareGuardNs(5000),
      m_minSafeRateBps(8000000000ULL),
      m_maxSafeRateBps(30000000000ULL),
      m_minTailGuardNs(80000),
      m_maxTailGuardNs(800000),
      m_minOutstandingBytes(32 * 1024),
      m_maxOutstandingCeilingBytes(256 * 1024),
      m_adaptPeriodNs(30000000),
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

    // RDMA_TRANSPORT_MODE=2 directly means userspace Injection Window.
    // No additional userspace sub-mode configuration is required.
    if (m_mode == MODE_USERSPACE)
    {
        m_userspaceAdmissionMode = USERSPACE_OCS_WINDOWED;
    }

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
RdmaTransport::SetUserspaceAdmissionMode(uint32_t mode)
{
    NS_ASSERT_MSG(
        mode <= USERSPACE_OCS_WINDOWED,
        "userspace admission mode must be 0(default pipeline) or 1(OCS windowed)");

    m_userspaceAdmissionMode =
        static_cast<UserspaceAdmissionMode>(mode);

    if (m_userspaceAdmissionMode == USERSPACE_DEFAULT_PIPELINE)
    {
        // A wake event belongs to OCS-window admission.  Do not leave one
        // armed when returning to the continuously reachable baseline.
        for (std::map<uint64_t, EventId>::iterator it = m_wakeEvents.begin();
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
    else
    {
        ApplyBandwidthNormalizedConfig("set_ocs_windowed");
    }

    std::cout
        << "[USERSPACE ADMISSION MODE]"
        << " mode=" << mode
        << " policy="
        << (m_userspaceAdmissionMode == USERSPACE_DEFAULT_PIPELINE
                ? "default_pipeline"
                : "ocs_windowed")
        << std::endl;
}

RdmaTransport::UserspaceAdmissionMode
RdmaTransport::GetUserspaceAdmissionMode() const
{
    return m_userspaceAdmissionMode;
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

    if (m_userspaceAdmissionMode == USERSPACE_OCS_WINDOWED)
    {
        ApplyBandwidthNormalizedConfig("configure");
        return;
    }

    const uint64_t maxOutstandingWr =
        (m_maxOutstandingBytes + m_wrChunkBytes - 1) /
        m_wrChunkBytes;

    std::cout
        << "[USERSPACE DEFAULT CONFIG]"
        << " reason=configure"
        << " wrChunkBytes=" << m_wrChunkBytes
        << " maxOutstandingBytes=" << m_maxOutstandingBytes
        << " maxOutstandingWr=" << maxOutstandingWr
        << " signaling=every_wr"
        << " gate_lookup=disabled"
        << " safe_rate=disabled"
        << " tail_guard=disabled"
        << std::endl;
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

    if (m_userspaceAdmissionMode == USERSPACE_OCS_WINDOWED)
    {
        ApplyBandwidthNormalizedConfig("explicit_config");
    }
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
    if (m_maxObservedRttNs == 0)
    {
        m_maxObservedRttNs = 10000ULL;
    }

    const uint32_t activeQps = std::max<uint32_t>(1, m_activeQpHint);
    const uint64_t safeFactorPermille =
        m_realDeploymentMode ? 800ULL : 900ULL;
    const uint64_t minFactorPermille =
        m_realDeploymentMode ? 400ULL : 500ULL;
    const uint64_t maxFactorPermille = 950ULL;

    const uint64_t oldSafeRateBps = m_safeRateBps;
    const uint64_t oldTailGuardNs = m_tailGuardNs;
    const uint64_t oldMaxOutstandingBytes = m_maxOutstandingBytes;
    const uint64_t oldWrChunkBytes = m_wrChunkBytes;
    const uint64_t oldMinPostBytes = m_minPostBytes;

    const long double rateBase =
        static_cast<long double>(bottleneckRateBps) /
        static_cast<long double>(activeQps);

    const uint64_t targetSafeRateBps =
        static_cast<uint64_t>(
            rateBase * static_cast<long double>(safeFactorPermille) / 1000.0L);
    m_minSafeRateBps = std::max<uint64_t>(
        static_cast<uint64_t>(
            rateBase * static_cast<long double>(minFactorPermille) / 1000.0L),
        1000000000ULL);
    m_maxSafeRateBps = std::max<uint64_t>(
        static_cast<uint64_t>(
            rateBase * static_cast<long double>(maxFactorPermille) / 1000.0L),
        m_minSafeRateBps);
    m_safeRateBps = ClampValue(
        targetSafeRateBps,
        m_minSafeRateBps,
        m_maxSafeRateBps);

    // Configure() owns the fixed per-QP SQ/WR pipeline capacity.
    m_minOutstandingBytes = std::min<uint64_t>(
        m_minOutstandingBytes,
        m_maxOutstandingBytes);
    m_maxOutstandingCeilingBytes = std::max<uint64_t>(
        m_maxOutstandingCeilingBytes,
        m_maxOutstandingBytes);
    m_minPostBytes = std::min<uint64_t>(4 * 1024, m_wrChunkBytes);

    m_userspaceSoftwareGuardNs =
        m_realDeploymentMode ? 50000ULL : 5000ULL;
    m_minTailGuardNs =
        m_realDeploymentMode ? 50000ULL : 10000ULL;
    m_maxTailGuardNs =
        m_realDeploymentMode ? 5000000ULL : 1000000ULL;

    // This is only a template for diagnostics before any QP has been bound.
    // Runtime admission computes RTT_q + 4*RTTVAR_port + software guard.
    uint64_t templateGuardNs = m_maxObservedRttNs;
    if (templateGuardNs <=
        std::numeric_limits<uint64_t>::max() - m_userspaceSoftwareGuardNs)
    {
        templateGuardNs += m_userspaceSoftwareGuardNs;
    }
    else
    {
        templateGuardNs = std::numeric_limits<uint64_t>::max();
    }
    m_tailGuardNs = ClampValue(
        templateGuardNs,
        m_minTailGuardNs,
        m_maxTailGuardNs);

    const bool changed =
        oldSafeRateBps != m_safeRateBps ||
        oldTailGuardNs != m_tailGuardNs ||
        oldMaxOutstandingBytes != m_maxOutstandingBytes ||
        oldWrChunkBytes != m_wrChunkBytes ||
        oldMinPostBytes != m_minPostBytes;

    if (changed && m_enabled)
    {
        std::cout
            << "[USERSPACE BW INIT]"
            << " reason=" << (reason != NULL ? reason : "unknown")
            << " gate_tables=" << m_gateTables.size()
            << " bottleneckRateBps=" << bottleneckRateBps
            << " configuredActiveQpHint=" << activeQps
            << " safeFactorPermille=" << safeFactorPermille
            << " minSafeRateBps=" << m_minSafeRateBps
            << " safeRateBps=" << m_safeRateBps
            << " maxSafeRateBps=" << m_maxSafeRateBps
            << " maxObservedRttNs=" << m_maxObservedRttNs
            << " configuredSwitchingGuardNs=" << m_switchingGuardNs
            << " switchingGuardAppliedNs=0"
            << " stableWindowEnd=1"
            << " stateScope=transport_template"
            << " cqeSrttNs=0"
            << " cqeRttvarNs=0"
            << " softwareGuardNs=" << m_userspaceSoftwareGuardNs
            << " maxOutstandingBytes=" << m_maxOutstandingBytes
            << " maxOutstandingCeilingBytes=" << m_maxOutstandingCeilingBytes
            << " wrChunkBytes=" << m_wrChunkBytes
            << " pipelineCapacityFixed=1"
            << " minPostBytes=" << m_minPostBytes
            << " tailGuardNs=" << m_tailGuardNs
            << " guardModel=qp_rtt_plus_4xport_cqe_var_plus_software"
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

    if (m_userspaceAdmissionMode == USERSPACE_OCS_WINDOWED)
    {
        ApplyBandwidthNormalizedConfig("enable_gate");
    }

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
    m_userspacePortStates.clear();
    m_userspaceRegisteredQps.clear();

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

    const Time now = Simulator::Now();
    const GateLookupResult gate = LookupGate(qp, now);
    if (qp != NULL)
    {
        // A permit is consumed only by GetNxtPacket().  Recompute it for each
        // scheduler eligibility check so a delayed ACK can cancel the bypass.
        qp->m_ackRecoveryPermit = false;
    }

    if (gate.bypass)
    {
        return true;
    }
    if (!gate.allowed)
    {
        return false;
    }

    bool deadlineAllows = true;
    uint64_t reserveNs = 0;
    uint64_t candidateNs = 0;
    uint64_t windowEndNs =
        static_cast<uint64_t>(gate.currentWindowEnd.GetNanoSeconds());

    if (m_rdma != NULL && m_rdma->IsRnicDeadlineEnabled())
    {
        const Time candidateTx = std::max(now, qp->m_nextAvail);
        reserveNs = m_rdma->GetRnicDeadlineReserveNs(qp);
        candidateNs =
            static_cast<uint64_t>(candidateTx.GetNanoSeconds());

        deadlineAllows =
            candidateNs <= windowEndNs &&
            reserveNs <= windowEndNs - candidateNs;

        qp->m_deadlineCheckCount++;
        if (deadlineAllows)
        {
            qp->m_deadlineAllowedCheckCount++;
        }
        else
        {
            qp->m_deadlineBlockedCheckCount++;

            // Count and log only once per physical window. The scheduler may
            // ask the same QP repeatedly after the ACK-safe cutoff.
            if (qp->m_deadlineLastBlockedWindowEndNs != windowEndNs)
            {
                qp->m_deadlineLastBlockedWindowEndNs = windowEndNs;
                qp->m_deadlineBlockEventCount++;

                const uint64_t cutoffNs =
                    reserveNs <= windowEndNs ? windowEndNs - reserveNs : 0;
                const Time nextWindow = GetNextPhysicalWindowStart(qp, gate);
                const int64_t nextWindowSignedNs = nextWindow.GetNanoSeconds();
                const uint64_t nextWindowNs =
                    nextWindowSignedNs > 0
                        ? static_cast<uint64_t>(nextWindowSignedNs)
                        : 0;

                std::cout
                    << "[RNIC DEADLINE BLOCK]"
                    << " t_ns=" << now.GetNanoSeconds()
                    << " node="
                    << (m_node != NULL
                            ? static_cast<int64_t>(m_node->GetId())
                            : -1)
                    << " src=" << qp->m_src
                    << " dst=" << qp->m_dest
                    << " sport=" << qp->sport
                    << " plane="
                    << (qp->m_hasBoundRnicPort
                            ? static_cast<int64_t>(qp->m_boundPlaneId)
                            : -1)
                    << " rnic_port="
                    << (qp->m_hasBoundRnicPort
                            ? static_cast<int64_t>(qp->m_boundRnicPort)
                            : -1)
                    << " candidate_tx_ns=" << candidateNs
                    << " stable_end_ns=" << windowEndNs
                    << " cutoff_ns=" << cutoffNs
                    << " reserve_ns=" << reserveNs
                    << " next_window_start_ns=" << nextWindowNs
                    << " samples=" << qp->m_deadlineValidSamples
                    << " srtt_ns=" << qp->m_deadlineSrttNs
                    << " rttvar_ns=" << qp->m_deadlineRttVarNs
                    << " block_events=" << qp->m_deadlineBlockEventCount
                    << std::endl;
            }

            ArmRnicAckRecovery(qp, gate, "deadline");
            return false;
        }
    }

    if (!qp->IsWinBound())
    {
        qp->m_deadlineLastAllowedWindowStartNs =
            static_cast<uint64_t>(gate.currentWindowStart.GetNanoSeconds());
        qp->m_deadlineLastAllowedWindowEndNs = windowEndNs;
        return true;
    }

    if (CanSendRnicContinuationProbe(qp, gate, now))
    {
        qp->m_deadlineLastAllowedWindowStartNs =
            static_cast<uint64_t>(gate.currentWindowStart.GetNanoSeconds());
        qp->m_deadlineLastAllowedWindowEndNs = windowEndNs;
        return true;
    }

    // The transport window is full.  Do not spin inside the current window;
    // schedule exactly one bounded continuation opportunity at the next
    // physical window for this destination.
    ArmRnicAckRecovery(qp, gate, "window_bound");
    return false;
}

Time
RdmaTransport::GetNextRnicGateTime(Ptr<RdmaQueuePair> qp) const
{
    if (m_mode != MODE_RNIC)
    {
        return Simulator::Now();
    }

    const Time now = Simulator::Now();
    const GateLookupResult gate = LookupGate(qp, now);
    if (gate.bypass || !gate.allowed)
    {
        return gate.nextAllowedTime;
    }

    if (m_rdma != NULL && m_rdma->IsRnicDeadlineEnabled())
    {
        const Time candidateTx = std::max(now, qp->m_nextAvail);
        const uint64_t reserveNs = m_rdma->GetRnicDeadlineReserveNs(qp);
        const uint64_t candidateNs =
            static_cast<uint64_t>(candidateTx.GetNanoSeconds());
        const uint64_t windowEndNs =
            static_cast<uint64_t>(gate.currentWindowEnd.GetNanoSeconds());

        if (!(candidateNs <= windowEndNs &&
              reserveNs <= windowEndNs - candidateNs))
        {
            return GetNextPhysicalWindowStart(qp, gate);
        }
    }

    if (qp != NULL && qp->IsWinBound())
    {
        if (m_rdma != NULL && m_rdma->IsRnicAckRecoveryEnabled() &&
            qp->m_ackRecoveryActive &&
            qp->m_ackRecoveryNextWindowStartNs >
                static_cast<uint64_t>(now.GetNanoSeconds()))
        {
            return NanoSeconds(qp->m_ackRecoveryNextWindowStartNs);
        }
        // No timer is required for an ordinary transport-window stall.  A
        // cumulative ACK/NACK will call TriggerTransmit().
        return Simulator::GetMaximumSimulationTime();
    }

    return now;
}

Time
RdmaTransport::GetNextPhysicalWindowStart(
    Ptr<RdmaQueuePair> qp,
    const GateLookupResult& gate) const
{
    if (gate.bypass)
    {
        return Simulator::Now();
    }
    return LookupGate(qp, gate.currentWindowEnd).nextAllowedTime;
}

void
RdmaTransport::ArmRnicAckRecovery(
    Ptr<RdmaQueuePair> qp,
    const GateLookupResult& gate,
    const char* reason) const
{
    if (m_rdma == NULL || !m_rdma->IsRnicAckRecoveryEnabled() ||
        qp == NULL || gate.bypass || !gate.allowed ||
        qp->snd_una >= qp->snd_nxt)
    {
        return;
    }

    const uint64_t windowEndNs =
        static_cast<uint64_t>(gate.currentWindowEnd.GetNanoSeconds());

    if (qp->GetBytesLeft() == 0)
    {
        if (qp->m_ackRecoveryLastArmWindowEndNs != windowEndNs)
        {
            qp->m_ackRecoveryLastArmWindowEndNs = windowEndNs;
            qp->m_ackRecoveryNoContinuationCount++;
            std::cout << "[RNIC ACK RECOVERY NO CONTINUATION]"
                      << " t_ns=" << Simulator::Now().GetNanoSeconds()
                      << " node="
                      << (m_node != NULL
                              ? static_cast<int64_t>(m_node->GetId())
                              : -1)
                      << " src=" << qp->m_src
                      << " dst=" << qp->m_dest
                      << " sport=" << qp->sport
                      << " snd_una=" << qp->snd_una
                      << " snd_nxt=" << qp->snd_nxt
                      << " reason=" << reason
                      << std::endl;
        }
        return;
    }

    if (qp->m_ackRecoveryAttempts >=
        m_rdma->GetRnicAckRecoveryMaxAttempts())
    {
        // Keep the final probe active so a late ACK can still complete it,
        // but schedule no further continuation opportunity.
        if (qp->m_ackRecoveryNextWindowStartNs != 0)
        {
            qp->m_ackRecoveryExhaustedCount++;
            std::cout << "[RNIC ACK RECOVERY EXHAUSTED]"
                      << " t_ns=" << Simulator::Now().GetNanoSeconds()
                      << " node="
                      << (m_node != NULL
                              ? static_cast<int64_t>(m_node->GetId())
                              : -1)
                      << " src=" << qp->m_src
                      << " dst=" << qp->m_dest
                      << " sport=" << qp->sport
                      << " attempts=" << qp->m_ackRecoveryAttempts
                      << " snd_una=" << qp->snd_una
                      << " snd_nxt=" << qp->snd_nxt
                      << std::endl;
        }
        qp->m_ackRecoveryPermit = false;
        qp->m_ackRecoveryNextWindowStartNs = 0;
        return;
    }

    const Time nextWindow = GetNextPhysicalWindowStart(qp, gate);
    if (nextWindow == Simulator::GetMaximumSimulationTime())
    {
        return;
    }

    const int64_t nextSignedNs = nextWindow.GetNanoSeconds();
    if (nextSignedNs < 0)
    {
        return;
    }
    const uint64_t nextWindowNs = static_cast<uint64_t>(nextSignedNs);

    if (qp->m_ackRecoveryLastArmWindowEndNs == windowEndNs &&
        qp->m_ackRecoveryNextWindowStartNs == nextWindowNs)
    {
        return;
    }

    qp->m_ackRecoveryActive = true;
    qp->m_ackRecoveryPermit = false;
    qp->m_ackRecoveryNextWindowStartNs = nextWindowNs;
    qp->m_ackRecoveryLastArmWindowEndNs = windowEndNs;
    qp->m_ackRecoveryArmCount++;

    std::cout << "[RNIC ACK RECOVERY ARM]"
              << " t_ns=" << Simulator::Now().GetNanoSeconds()
              << " node="
              << (m_node != NULL ? static_cast<int64_t>(m_node->GetId()) : -1)
              << " src=" << qp->m_src
              << " dst=" << qp->m_dest
              << " sport=" << qp->sport
              << " plane="
              << (qp->m_hasBoundRnicPort
                      ? static_cast<int64_t>(qp->m_boundPlaneId)
                      : -1)
              << " rnic_port="
              << (qp->m_hasBoundRnicPort
                      ? static_cast<int64_t>(qp->m_boundRnicPort)
                      : -1)
              << " source_window_start_ns="
              << gate.currentWindowStart.GetNanoSeconds()
              << " source_window_end_ns=" << windowEndNs
              << " next_window_start_ns=" << nextWindowNs
              << " snd_una=" << qp->snd_una
              << " snd_nxt=" << qp->snd_nxt
              << " on_the_fly=" << qp->GetOnTheFly()
              << " win=" << qp->GetWin()
              << " reason=" << reason
              << std::endl;
}

bool
RdmaTransport::CanSendRnicContinuationProbe(
    Ptr<RdmaQueuePair> qp,
    const GateLookupResult& gate,
    Time now) const
{
    if (m_rdma == NULL || !m_rdma->IsRnicAckRecoveryEnabled() ||
        qp == NULL || !qp->m_ackRecoveryActive ||
        qp->snd_una >= qp->snd_nxt || qp->GetBytesLeft() == 0)
    {
        return false;
    }

    if (qp->m_ackRecoveryAttempts >=
        m_rdma->GetRnicAckRecoveryMaxAttempts())
    {
        ArmRnicAckRecovery(qp, gate, "attempt_limit");
        return false;
    }

    const uint64_t windowStartNs =
        static_cast<uint64_t>(gate.currentWindowStart.GetNanoSeconds());
    const uint64_t nowNs = static_cast<uint64_t>(now.GetNanoSeconds());

    if (windowStartNs < qp->m_ackRecoveryNextWindowStartNs ||
        nowNs < qp->m_ackRecoveryNextWindowStartNs ||
        qp->m_ackRecoveryLastProbeWindowStartNs == windowStartNs)
    {
        // If this window already carried a probe, arm the next one before
        // returning false so the egress queue sleeps rather than spins.
        if (qp->m_ackRecoveryLastProbeWindowStartNs == windowStartNs)
        {
            ArmRnicAckRecovery(qp, gate, "probe_already_sent");
        }
        return false;
    }

    // Any outstanding timestamp predates this recovery window and would
    // include the schedule gap.  Drop it so the continuation packet can
    // provide a fresh, same-window DATA-to-ACK RTT sample.
    m_rdma->InvalidateRnicDeadlineSample(qp);

    qp->m_ackRecoveryPermit = true;
    qp->m_ackRecoveryPermitWindowStartNs = windowStartNs;
    return true;
}

RdmaTransport::GateLookupResult
RdmaTransport::LookupGate(
    Ptr<RdmaQueuePair> qp,
    Time now) const
{
    GateLookupResult result;
    result.currentWindowStart = now;
    result.currentWindowEnd = now;
    result.nextAllowedTime = Simulator::GetMaximumSimulationTime();

    if (!m_gateEnabled || qp == NULL || !qp->m_hasBoundRnicPort)
    {
        result.bypass = true;
        result.allowed = true;
        result.currentWindowStart = now;
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
            result.currentWindowStart =
                NanoSeconds(periodBaseNs + slot.startOffsetNs);
            result.currentWindowEnd =
                NanoSeconds(periodBaseNs + slot.endOffsetNs);
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
RdmaTransport::GetPortBottleneckRateBps(
    Ptr<RdmaQueuePair> qp) const
{
    uint64_t portRateBps = 0;
    if (m_node != NULL &&
        qp != NULL &&
        qp->m_hasBoundRnicPort &&
        qp->m_boundNicIdx < m_node->GetNDevices())
    {
        Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(
            m_node->GetDevice(qp->m_boundNicIdx));
        if (dev != NULL)
        {
            portRateBps = dev->GetDataRate().GetBitRate();
        }
    }

    uint64_t configuredRateBps = m_bottleneckRateBps;
    if (configuredRateBps == 0)
    {
        configuredRateBps = GetLocalBottleneckRateBps();
    }
    if (portRateBps == 0)
    {
        return configuredRateBps;
    }
    if (configuredRateBps == 0)
    {
        return portRateBps;
    }
    return std::min(portRateBps, configuredRateBps);
}

RdmaTransport::UserspacePortState&
RdmaTransport::GetUserspacePortState(
    Ptr<RdmaQueuePair> qp,
    const char* reason)
{
    NS_ASSERT_MSG(qp != NULL, "Cannot resolve Mode-2 port state for a null QP");
    NS_ASSERT_MSG(qp->m_hasBoundRnicPort,
                  "Mode-2 OCS admission requires a QP bound by the plane scheduler");

    const uint32_t rnicPort = qp->m_boundRnicPort;
    std::map<uint32_t, UserspacePortState>::iterator found =
        m_userspacePortStates.find(rnicPort);
    if (found != m_userspacePortStates.end())
    {
        return found->second;
    }

    UserspacePortState state;
    state.rnicPort = rnicPort;
    state.planeId = qp->m_boundPlaneId;
    state.ifIndex = qp->m_boundNicIdx;
    state.bottleneckRateBps = GetPortBottleneckRateBps(qp);
    if (state.bottleneckRateBps == 0)
    {
        state.bottleneckRateBps = std::max<uint64_t>(
            m_bottleneckRateBps,
            1000000000ULL);
    }

    const uint32_t activeQps = std::max<uint32_t>(1, m_activeQpHint);
    const uint64_t safeFactorPermille =
        m_realDeploymentMode ? 800ULL : 900ULL;
    const uint64_t minFactorPermille =
        m_realDeploymentMode ? 400ULL : 500ULL;
    const uint64_t maxFactorPermille = 950ULL;
    const long double rateBase =
        static_cast<long double>(state.bottleneckRateBps) /
        static_cast<long double>(activeQps);

    state.minSafeRateBps = std::max<uint64_t>(
        static_cast<uint64_t>(
            rateBase * static_cast<long double>(minFactorPermille) / 1000.0L),
        1000000000ULL);
    state.maxSafeRateBps = std::max<uint64_t>(
        static_cast<uint64_t>(
            rateBase * static_cast<long double>(maxFactorPermille) / 1000.0L),
        state.minSafeRateBps);
    state.safeRateBps = ClampValue(
        static_cast<uint64_t>(
            rateBase * static_cast<long double>(safeFactorPermille) / 1000.0L),
        state.minSafeRateBps,
        state.maxSafeRateBps);
    state.lastAdaptNs =
        static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());

    std::pair<std::map<uint32_t, UserspacePortState>::iterator, bool> inserted =
        m_userspacePortStates.insert(std::make_pair(rnicPort, state));
    UserspacePortState& portState = inserted.first->second;

    std::cout
        << "[USERSPACE PORT INIT]"
        << " reason=" << (reason != NULL ? reason : "unknown")
        << " node="
        << (m_node != NULL ? static_cast<int64_t>(m_node->GetId()) : -1)
        << " rnic_port=" << portState.rnicPort
        << " plane=" << portState.planeId
        << " ifindex=" << portState.ifIndex
        << " bottleneckRateBps=" << portState.bottleneckRateBps
        << " configuredActiveQpHint=" << activeQps
        << " safeFactorPermille=" << safeFactorPermille
        << " minSafeRateBps=" << portState.minSafeRateBps
        << " safeRateBps=" << portState.safeRateBps
        << " maxSafeRateBps=" << portState.maxSafeRateBps
        << " baseRttNs=" << qp->m_baseRtt
        << " softwareGuardNs=" << m_userspaceSoftwareGuardNs
        << " aggregateOutstandingBytes="
        << portState.aggregateOutstandingBytes
        << " stateScope=per_rnic_port"
        << " switchingGuardAppliedNs=0"
        << std::endl;

    return portState;
}

const RdmaTransport::UserspacePortState*
RdmaTransport::FindUserspacePortState(
    Ptr<RdmaQueuePair> qp) const
{
    if (qp == NULL || !qp->m_hasBoundRnicPort)
    {
        return NULL;
    }
    std::map<uint32_t, UserspacePortState>::const_iterator found =
        m_userspacePortStates.find(qp->m_boundRnicPort);
    return found != m_userspacePortStates.end() ? &found->second : NULL;
}

void
RdmaTransport::SchedulePortQpWakeups(
    Ptr<RdmaQueuePair> triggerQp)
{
    if (triggerQp == NULL || !triggerQp->m_hasBoundRnicPort)
    {
        return;
    }

    const uint32_t rnicPort = triggerQp->m_boundRnicPort;
    const Time wakeTime = Simulator::Now() + NanoSeconds(1);

    for (std::map<uint64_t, Ptr<RdmaQueuePair> >::const_iterator it =
             m_userspaceRegisteredQps.begin();
         it != m_userspaceRegisteredQps.end();
         ++it)
    {
        Ptr<RdmaQueuePair> candidate = it->second;
        if (candidate == NULL ||
            candidate->IsFinished() ||
            candidate->GetUnpostedBytes() == 0 ||
            !candidate->m_hasBoundRnicPort ||
            candidate->m_boundRnicPort != rnicPort)
        {
            continue;
        }

        ScheduleNextWake(candidate, wakeTime);
    }
}

uint64_t
RdmaTransport::GetUserspaceTailGuardNs(
    Ptr<RdmaQueuePair> qp,
    const UserspacePortState& portState) const
{
    const uint64_t baseRttNs =
        qp != NULL && qp->m_baseRtt > 0
            ? qp->m_baseRtt
            : std::max<uint64_t>(m_maxObservedRttNs, 10000ULL);
    const uint64_t variationGuardNs =
        portState.cqeLatencyRttvarNs >
                std::numeric_limits<uint64_t>::max() / 4
            ? std::numeric_limits<uint64_t>::max()
            : 4 * portState.cqeLatencyRttvarNs;

    uint64_t guardNs = baseRttNs;
    if (guardNs <= std::numeric_limits<uint64_t>::max() - variationGuardNs)
    {
        guardNs += variationGuardNs;
    }
    else
    {
        guardNs = std::numeric_limits<uint64_t>::max();
    }
    if (guardNs <=
        std::numeric_limits<uint64_t>::max() - m_userspaceSoftwareGuardNs)
    {
        guardNs += m_userspaceSoftwareGuardNs;
    }
    else
    {
        guardNs = std::numeric_limits<uint64_t>::max();
    }
    return ClampValue(guardNs, m_minTailGuardNs, m_maxTailGuardNs);
}

uint64_t
RdmaTransport::GetSafeBudgetBytes(
    Ptr<RdmaQueuePair> qp,
    const GateLookupResult& gate,
    Time now)
{
    if (gate.bypass)
    {
        return m_maxOutstandingBytes;
    }
    if (!gate.allowed || gate.currentWindowEnd <= now)
    {
        return 0;
    }

    UserspacePortState& portState =
        GetUserspacePortState(qp, "safe_budget");
    const uint64_t tailGuardNs =
        GetUserspaceTailGuardNs(qp, portState);
    const uint64_t nowNs = static_cast<uint64_t>(now.GetNanoSeconds());
    const uint64_t endNs =
        static_cast<uint64_t>(gate.currentWindowEnd.GetNanoSeconds());
    if (endNs <= nowNs + tailGuardNs)
    {
        return 0;
    }

    const uint64_t usableNs = endNs - nowNs - tailGuardNs;
    const long double budgetBytes =
        static_cast<long double>(portState.safeRateBps) *
        static_cast<long double>(usableNs) / 8000000000.0L;
    if (budgetBytes >=
        static_cast<long double>(std::numeric_limits<uint64_t>::max()))
    {
        return std::numeric_limits<uint64_t>::max();
    }
    return static_cast<uint64_t>(budgetBytes);
}

void
RdmaTransport::UpdateCqeTimingModel(
    Ptr<RdmaQueuePair> qp,
    uint64_t completionLatencyNs)
{
    if (qp == NULL || completionLatencyNs == 0)
    {
        return;
    }

    UserspacePortState& portState =
        GetUserspacePortState(qp, "cqe");
    const uint64_t oldTailGuardNs =
        GetUserspaceTailGuardNs(qp, portState);

    if (portState.cqeTimingSamples == 0)
    {
        portState.cqeLatencySrttNs = completionLatencyNs;
        portState.cqeLatencyRttvarNs = 0;
    }
    else
    {
        const uint64_t deltaNs =
            completionLatencyNs >= portState.cqeLatencySrttNs
                ? completionLatencyNs - portState.cqeLatencySrttNs
                : portState.cqeLatencySrttNs - completionLatencyNs;
        portState.cqeLatencyRttvarNs =
            (3 * portState.cqeLatencyRttvarNs + deltaNs) / 4;
        portState.cqeLatencySrttNs =
            (7 * portState.cqeLatencySrttNs + completionLatencyNs) / 8;
    }

    ++portState.cqeTimingSamples;
    const uint64_t newTailGuardNs =
        GetUserspaceTailGuardNs(qp, portState);
    const bool powerOfTwoSample =
        (portState.cqeTimingSamples & (portState.cqeTimingSamples - 1)) == 0;

    if (m_enabled &&
        (portState.cqeTimingSamples <= 4 ||
         (powerOfTwoSample && oldTailGuardNs != newTailGuardNs)))
    {
        std::cout
            << "[USERSPACE CQE GUARD]"
            << " t_ns=" << Simulator::Now().GetNanoSeconds()
            << " node="
            << (m_node != NULL ? static_cast<int64_t>(m_node->GetId()) : -1)
            << " rnic_port=" << portState.rnicPort
            << " plane=" << portState.planeId
            << " ifindex=" << portState.ifIndex
            << " dst=" << qp->GetDest()
            << " sport=" << qp->sport
            << " samples=" << portState.cqeTimingSamples
            << " sampleLatencyNs=" << completionLatencyNs
            << " cqeSrttNs=" << portState.cqeLatencySrttNs
            << " cqeRttvarNs=" << portState.cqeLatencyRttvarNs
            << " baseRttNs=" << qp->m_baseRtt
            << " softwareGuardNs=" << m_userspaceSoftwareGuardNs
            << " oldTailGuardNs=" << oldTailGuardNs
            << " newTailGuardNs=" << newTailGuardNs
            << " stateScope=per_rnic_port"
            << " switchingGuardAppliedNs=0"
            << std::endl;
    }
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
    if (!m_enabled ||
        m_userspaceAdmissionMode != USERSPACE_OCS_WINDOWED ||
        qp == NULL)
    {
        return;
    }

    UserspacePortState& portState =
        GetUserspacePortState(qp, "adapt");
    const uint64_t nowNs =
        static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
    const uint64_t periodNs = GetAdaptivePeriodNs();
    const bool urgent = portState.recoverySinceLastAdapt;

    if (!urgent &&
        portState.lastAdaptNs > 0 &&
        nowNs < portState.lastAdaptNs + periodNs)
    {
        return;
    }
    if (urgent &&
        portState.lastAdaptNs > 0 &&
        nowNs < portState.lastAdaptNs + 1000000ULL)
    {
        return;
    }

    const uint64_t oldSafeRateBps = portState.safeRateBps;
    const uint64_t oldTailGuardNs =
        GetUserspaceTailGuardNs(qp, portState);
    const char* action = "hold";

    if (portState.recoverySinceLastAdapt)
    {
        portState.safeRateBps = std::max<uint64_t>(
            portState.minSafeRateBps,
            (portState.safeRateBps * 8) / 10);
        portState.cqeLatencyRttvarNs = std::min<uint64_t>(
            m_maxTailGuardNs / 4,
            portState.cqeLatencyRttvarNs + 6250ULL);
        portState.stableAdaptPeriods = 0;
        action = "decrease";
    }
    else if (false &&
             portState.backlogSinceLastAdapt &&
             portState.wrCompletionSinceLastAdapt)
    {
        ++portState.stableAdaptPeriods;
        if (portState.stableAdaptPeriods >= 2)
        {
            portState.safeRateBps = std::min<uint64_t>(
                portState.maxSafeRateBps,
                (portState.safeRateBps * 105) / 100);
            action = "increase";
        }
    }
    else
    {
        portState.stableAdaptPeriods = 0;
    }

    const uint64_t newTailGuardNs =
        GetUserspaceTailGuardNs(qp, portState);
    if (oldSafeRateBps != portState.safeRateBps ||
        oldTailGuardNs != newTailGuardNs)
    {
        std::cout
            << "[USERSPACE ADAPT]"
            << " t=" << nowNs
            << " node="
            << (m_node != NULL ? static_cast<int64_t>(m_node->GetId()) : -1)
            << " rnic_port=" << portState.rnicPort
            << " plane=" << portState.planeId
            << " ifindex=" << portState.ifIndex
            << " action=" << action
            << " dst=" << qp->GetDest()
            << " oldSafeRateBps=" << oldSafeRateBps
            << " newSafeRateBps=" << portState.safeRateBps
            << " oldTailGuardNs=" << oldTailGuardNs
            << " newTailGuardNs=" << newTailGuardNs
            << " recovery=" << (portState.recoverySinceLastAdapt ? 1 : 0)
            << " wrCompletion=" << (portState.wrCompletionSinceLastAdapt ? 1 : 0)
            << " backlog=" << (portState.backlogSinceLastAdapt ? 1 : 0)
            << " stablePeriods=" << portState.stableAdaptPeriods
            << " unposted=" << qp->GetUnpostedBytes()
            << " outstanding=" << GetUserspaceOutstandingBytes(qp)
            << " stateScope=per_rnic_port"
            << std::endl;
    }

    portState.lastAdaptNs = nowNs;
    portState.recoverySinceLastAdapt = false;
    portState.wrCompletionSinceLastAdapt = false;
    portState.backlogSinceLastAdapt = false;
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


RdmaTransport::UserspaceQpState&
RdmaTransport::GetUserspaceQpState(Ptr<RdmaQueuePair> qp)
{
    NS_ASSERT(qp != NULL);
    return m_userspaceQpStates[GetPostStatsKey(qp)];
}

uint64_t
RdmaTransport::GetUserspaceOutstandingBytes(Ptr<RdmaQueuePair> qp) const
{
    if (qp == NULL)
    {
        return 0;
    }

    const uint64_t key =
        RdmaHw::GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg);
    std::map<uint64_t, UserspaceQpState>::const_iterator it =
        m_userspaceQpStates.find(key);
    return it != m_userspaceQpStates.end()
        ? it->second.outstandingBytes
        : 0;
}

void
RdmaTransport::RecordPost(
    Ptr<RdmaQueuePair> qp,
    uint64_t bytes,
    uint64_t windowBudget,
    uint64_t admissionBudget,
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

    if (admissionBudget > 0 && bytes >= admissionBudget)
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
            << " qp_outstanding_bytes="
            << GetUserspaceOutstandingBytes(qp);

        const UserspacePortState* portState = FindUserspacePortState(qp);
        std::cout
            << " port_outstanding_bytes="
            << (portState != NULL
                    ? portState->aggregateOutstandingBytes
                    : GetUserspaceOutstandingBytes(qp))
            << " window_budget_bytes=" << windowBudget
            << " admission_budget_bytes=" << admissionBudget
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

    const uint64_t key = GetPostStatsKey(qp);
    std::map<uint64_t, PostStats>::iterator it = m_postStats.find(key);
    if (it == m_postStats.end() || it->second.postCount == 0)
    {
        return;
    }

    const PostStats st = it->second;
    m_postStats.erase(it);
    const uint32_t srcNodeId = m_node != NULL ? m_node->GetId() : 0;
    const uint64_t avgBytes =
        st.postCount > 0 ? st.totalBytes / st.postCount : 0;

    std::cout
        << "[USERSPACE WR SUMMARY]"
        << " t=" << Simulator::Now().GetNanoSeconds()
        << " src=" << srcNodeId
        << " dst=" << qp->GetDest()
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
        << " cqe_count=" << st.completionCount
        << " completed_bytes=" << st.completedBytes
        << " avg_completion_latency_ns="
        << (st.completionCount > 0
                ? st.totalCompletionLatencyNs / st.completionCount
                : 0)
        << " min_completion_latency_ns=" << st.minCompletionLatencyNs
        << " max_completion_latency_ns=" << st.maxCompletionLatencyNs
        << " last_completed_wr_id=" << st.lastCompletedWrId
        << " userspace_outstanding_bytes="
        << GetUserspaceOutstandingBytes(qp);

    const UserspacePortState* portState = FindUserspacePortState(qp);
    if (portState != NULL)
    {
        std::cout
            << " rnic_port=" << portState->rnicPort
            << " plane=" << portState->planeId
            << " ifindex=" << portState->ifIndex
            << " safe_rate_bps=" << portState->safeRateBps
            << " tail_guard_ns=" << GetUserspaceTailGuardNs(qp, *portState)
            << " cqe_timing_samples=" << portState->cqeTimingSamples
            << " cqe_srtt_ns=" << portState->cqeLatencySrttNs
            << " cqe_rttvar_ns=" << portState->cqeLatencyRttvarNs
            << " port_outstanding_bytes="
            << portState->aggregateOutstandingBytes
            << " registered_qp_count=" << portState->registeredQpCount
            << " aggregate_admission_block_events="
            << portState->aggregateAdmissionBlockEvents
            << " state_scope=per_rnic_port";
    }
    else
    {
        std::cout
            << " rnic_port=-1"
            << " plane=-1"
            << " ifindex=-1"
            << " safe_rate_bps=" << m_safeRateBps
            << " tail_guard_ns=" << m_tailGuardNs
            << " cqe_timing_samples=0"
            << " cqe_srtt_ns=0"
            << " cqe_rttvar_ns=0"
            << " state_scope=transport_template";
    }

    std::cout
        << " switching_guard_applied_ns=0"
        << " guard_model=qp_rtt_plus_4xport_cqe_var_plus_software"
        << " completion_semantic=signaled_wr_busy_poll"
        << " sample_limit=" << m_postLogSampleLimit
        << std::endl;

    m_userspaceQpStates.erase(key);
}


void
RdmaTransport::TrySubmit(
    Ptr<RdmaQueuePair> qp)
{
    if (m_userspaceAdmissionMode == USERSPACE_DEFAULT_PIPELINE)
    {
        TrySubmitDefaultPipeline(qp);
        return;
    }

    TrySubmitOcsWindowed(qp);
}

void
RdmaTransport::TrySubmitDefaultPipeline(
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

    UserspaceQpState& state = GetUserspaceQpState(qp);

    while (qp->GetUnpostedBytes() > 0)
    {
        const uint64_t outstandingHeadroom =
            m_maxOutstandingBytes > state.outstandingBytes
                ? m_maxOutstandingBytes - state.outstandingBytes
                : 0;

        if (outstandingHeadroom == 0)
        {
            break;
        }

        const uint64_t unpostedBytes = qp->GetUnpostedBytes();
        uint64_t nextWrBytes = std::min(m_wrChunkBytes, unpostedBytes);
        nextWrBytes = std::min(nextWrBytes, outstandingHeadroom);

        if (nextWrBytes == 0)
        {
            break;
        }

        // One Mode-2 chunk equals one signaled WR.  Unlike OCS-windowed
        // admission, the continuously reachable baseline may keep several WRs
        // outstanding and relies only on CQE credit recovery.
        m_rdma->PostWork(qp, nextWrBytes);
        state.postedBytes += nextWrBytes;
        state.outstandingBytes += nextWrBytes;

        RecordPost(
            qp,
            nextWrBytes,
            0,
            0,
            Simulator::GetMaximumSimulationTime());
    }
}

void
RdmaTransport::TrySubmitOcsWindowed(
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

    UserspacePortState& portState =
        GetUserspacePortState(qp, "submit");
    if (qp->GetUnpostedBytes() > 0)
    {
        portState.backlogSinceLastAdapt = true;
    }

    UserspaceQpState& state = GetUserspaceQpState(qp);
    uint64_t outstanding = state.outstandingBytes;

    while (qp->GetUnpostedBytes() > 0)
    {
        const Time loopNow = Simulator::Now();
        const GateLookupResult gate = LookupGate(qp, loopNow);

        if (!gate.allowed)
        {
            ScheduleNextWake(qp, gate.nextAllowedTime);
            break;
        }

        const uint64_t safeBudget =
            GetSafeBudgetBytes(qp, gate, loopNow);
        if (safeBudget < m_minPostBytes)
        {
            portState.backlogSinceLastAdapt = true;
            if (gate.currentWindowEnd > loopNow &&
                gate.currentWindowEnd != Simulator::GetMaximumSimulationTime())
            {
                const GateLookupResult nextGate =
                    LookupGate(qp, gate.currentWindowEnd + NanoSeconds(1));
                ScheduleNextWake(qp, nextGate.nextAllowedTime);
            }
            break;
        }

        const uint64_t portOutstanding =
            portState.aggregateOutstandingBytes;
        const uint64_t admissionBudget =
            safeBudget > portOutstanding
                ? safeBudget - portOutstanding
                : 0;
        if (admissionBudget < m_minPostBytes)
        {
            portState.backlogSinceLastAdapt = true;

            const uint64_t windowEndNs = static_cast<uint64_t>(
                gate.currentWindowEnd.GetNanoSeconds());
            if (state.lastAggregateBlockWindowEndNs != windowEndNs)
            {
                state.lastAggregateBlockWindowEndNs = windowEndNs;
                portState.aggregateAdmissionBlockEvents++;

                std::cout
                    << "[USERSPACE PORT ADMISSION BLOCK]"
                    << " t_ns=" << loopNow.GetNanoSeconds()
                    << " node="
                    << (m_node != NULL
                            ? static_cast<int64_t>(m_node->GetId())
                            : -1)
                    << " dst=" << qp->GetDest()
                    << " sport=" << qp->sport
                    << " pg=" << qp->m_pg
                    << " rnic_port=" << portState.rnicPort
                    << " plane=" << portState.planeId
                    << " ifindex=" << portState.ifIndex
                    << " qp_outstanding_bytes=" << outstanding
                    << " port_outstanding_bytes=" << portOutstanding
                    << " window_budget_bytes=" << safeBudget
                    << " admission_budget_bytes=" << admissionBudget
                    << " window_end_ns=" << windowEndNs
                    << " stateScope=per_rnic_port"
                    << std::endl;
            }
            break;
        }

        const uint64_t outstandingHeadroom =
            m_maxOutstandingBytes > outstanding
                ? m_maxOutstandingBytes - outstanding
                : 0;
        if (outstandingHeadroom < m_minPostBytes)
        {
            portState.backlogSinceLastAdapt = true;
            break;
        }

        const uint64_t unpostedBytes = qp->GetUnpostedBytes();
        uint64_t nextWrBytes = std::min(m_wrChunkBytes, unpostedBytes);
        nextWrBytes = std::min(nextWrBytes, outstandingHeadroom);
        nextWrBytes = std::min(nextWrBytes, admissionBudget);
        const bool isFinalTail = nextWrBytes == unpostedBytes;
        if (nextWrBytes < m_minPostBytes && !isFinalTail)
        {
            break;
        }

        NS_ASSERT_MSG(
            portState.aggregateOutstandingBytes <=
                std::numeric_limits<uint64_t>::max() - nextWrBytes,
            "Mode-2 per-port outstanding ledger overflow");

        m_rdma->PostWork(qp, nextWrBytes);
        state.postedBytes += nextWrBytes;
        state.outstandingBytes += nextWrBytes;
        portState.aggregateOutstandingBytes += nextWrBytes;
        outstanding = state.outstandingBytes;
        RecordPost(
            qp,
            nextWrBytes,
            safeBudget,
            admissionBudget,
            gate.currentWindowEnd);
    }

    MaybeAdapt(qp);
}


void
RdmaTransport::RegisterQp(
    Ptr<RdmaQueuePair> qp)
{
    NS_ASSERT(qp != NULL);
    GetUserspaceQpState(qp);

    const uint64_t key = GetPostStatsKey(qp);
    const bool newlyRegistered =
        m_userspaceRegisteredQps.insert(std::make_pair(key, qp)).second;

    if (m_userspaceAdmissionMode == USERSPACE_OCS_WINDOWED)
    {
        UserspacePortState& portState =
            GetUserspacePortState(qp, "register_qp");
        if (newlyRegistered)
        {
            portState.registeredQpCount++;

            std::cout
                << "[USERSPACE PORT QP REGISTERED]"
                << " t_ns=" << Simulator::Now().GetNanoSeconds()
                << " node="
                << (m_node != NULL
                        ? static_cast<int64_t>(m_node->GetId())
                        : -1)
                << " rnic_port=" << portState.rnicPort
                << " plane=" << portState.planeId
                << " ifindex=" << portState.ifIndex
                << " configuredActiveQpHint="
                << std::max<uint32_t>(1, m_activeQpHint)
                << " registeredQpCount="
                << portState.registeredQpCount
                << " aggregateOutstandingBytes="
                << portState.aggregateOutstandingBytes
                << " stateScope=per_rnic_port"
                << std::endl;
        }
    }

    TrySubmit(qp);
}


void
RdmaTransport::NotifyWrCompletion(
    Ptr<RdmaQueuePair> qp,
    uint64_t wrId,
    uint64_t bytes,
    uint64_t postTimeNs,
    uint64_t completionTimeNs)
{
    if (!m_enabled || qp == NULL)
    {
        return;
    }

    const uint64_t key = GetPostStatsKey(qp);
    UserspaceQpState& state = m_userspaceQpStates[key];
    NS_ASSERT_MSG(
        state.outstandingBytes >= bytes,
        "CQE completed more bytes than the userspace QP ledger has outstanding");

    UserspacePortState* portState = NULL;
    if (m_userspaceAdmissionMode == USERSPACE_OCS_WINDOWED)
    {
        portState = &GetUserspacePortState(qp, "completion");
        NS_ASSERT_MSG(
            portState->aggregateOutstandingBytes >= bytes,
            "CQE completed more bytes than the userspace port ledger has outstanding");
    }

    state.outstandingBytes -= bytes;
    state.completedBytes += bytes;
    if (portState != NULL)
    {
        portState->aggregateOutstandingBytes -= bytes;
    }

    PostStats& stats = m_postStats[key];
    const uint64_t latencyNs = completionTimeNs >= postTimeNs
        ? completionTimeNs - postTimeNs
        : 0;

    if (m_userspaceAdmissionMode == USERSPACE_OCS_WINDOWED)
    {
        UpdateCqeTimingModel(qp, latencyNs);
    }

    stats.completionCount++;
    stats.completedBytes += bytes;
    stats.totalCompletionLatencyNs += latencyNs;
    stats.lastCompletedWrId = wrId;

    if (stats.completionCount == 1)
    {
        stats.minCompletionLatencyNs = latencyNs;
        stats.maxCompletionLatencyNs = latencyNs;
        std::cout
            << "[USERSPACE CQE]"
            << " t_ns=" << completionTimeNs
            << " src="
            << (m_node != NULL ? static_cast<int64_t>(m_node->GetId()) : -1)
            << " dst=" << qp->GetDest()
            << " sport=" << qp->sport
            << " dport=" << qp->dport
            << " pg=" << qp->m_pg
            << " wr_id=" << wrId
            << " bytes=" << bytes
            << " post_time_ns=" << postTimeNs
            << " completion_latency_ns=" << latencyNs
            << " qp_outstanding_bytes=" << state.outstandingBytes
            << " port_outstanding_bytes="
            << (portState != NULL
                    ? portState->aggregateOutstandingBytes
                    : state.outstandingBytes)
            << " semantic=signaled_wr_busy_poll"
            << std::endl;
    }
    else
    {
        stats.minCompletionLatencyNs =
            std::min(stats.minCompletionLatencyNs, latencyNs);
        stats.maxCompletionLatencyNs =
            std::max(stats.maxCompletionLatencyNs, latencyNs);
    }

    if (portState != NULL)
    {
        portState->wrCompletionSinceLastAdapt = true;
        if (qp->GetUnpostedBytes() > 0)
        {
            portState->backlogSinceLastAdapt = true;
        }

        MaybeAdapt(qp);
        SchedulePortQpWakeups(qp);
    }

    // Models immediate observation by a dedicated CQ busy-poll thread.
    // Final summary/cleanup is deferred to NotifyQpComplete(), because one
    // cumulative ACK may complete several signaled WRs at the same timestamp.
    if (!qp->IsFinished() &&
        m_userspaceAdmissionMode == USERSPACE_DEFAULT_PIPELINE)
    {
        TrySubmit(qp);
    }
}

void
RdmaTransport::NotifyQpComplete(Ptr<RdmaQueuePair> qp)
{
    if (!m_enabled || qp == NULL)
    {
        return;
    }

    const uint64_t outstanding = GetUserspaceOutstandingBytes(qp);
    NS_ASSERT_MSG(
        outstanding == 0,
        "QP completed before all signaled WR completions reached userspace");

    UserspacePortState* portState = NULL;
    if (m_userspaceAdmissionMode == USERSPACE_OCS_WINDOWED &&
        qp->m_hasBoundRnicPort)
    {
        portState = &GetUserspacePortState(qp, "qp_complete");
    }

    FlushPostSummary(qp);

    const uint64_t key = GetPostStatsKey(qp);
    m_userspaceQpStates.erase(key);
    if (m_userspaceRegisteredQps.erase(key) > 0 &&
        portState != NULL &&
        portState->registeredQpCount > 0)
    {
        portState->registeredQpCount--;

        std::cout
            << "[USERSPACE PORT QP UNREGISTERED]"
            << " t_ns=" << Simulator::Now().GetNanoSeconds()
            << " node="
            << (m_node != NULL
                    ? static_cast<int64_t>(m_node->GetId())
                    : -1)
            << " rnic_port=" << portState->rnicPort
            << " plane=" << portState->planeId
            << " ifindex=" << portState->ifIndex
            << " registeredQpCount="
            << portState->registeredQpCount
            << " aggregateOutstandingBytes="
            << portState->aggregateOutstandingBytes
            << " stateScope=per_rnic_port"
            << std::endl;
    }
}

} // namespace ns3