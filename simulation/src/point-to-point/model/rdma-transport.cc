#include "rdma-transport.h"

#include "ns3/simulator.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/qbb-net-device.h"

#include <algorithm>
#include <iostream>
#include <limits>

// MODE2_CQE_SEMANTICS_V1: Mode-2 completion path models signaled WR + CQE.
// MODE2_VERBS_COMPLETION_V2: userspace operation completion is owned by final signaled-WR CQE.
// MODE2_VERBS_ERROR_CQE_V1: userspace failure is observed only through CQE status.
// MODE2_DEFAULT_PIPELINE_V1: default Mode 2 can bypass OCS admission and pipeline WRs.
// MODE2_INJECTION_WINDOW_PIPELINE_V1: OCS window admission preserves multi-WR pipelining.
// MODE2_OPTIMIZED_GUARD_V1: stable-end + CQE-jitter userspace safety boundary.
// MODE2_PER_PORT_TIMING_V1: isolate Mode-2 rate/CQE state by breakout RNIC port.
// MODE2_PER_PORT_AGGREGATE_ADMISSION_V1: account all in-flight WR bytes per breakout port.
// MODE2_PORT_QP_LOGGING_V1: separate configured QP hint from runtime per-port QP count.
// MODE1_CONTINUATION_ACK_RECOVERY_V1: bounded next-window DATA probing.
// MODE1_FINAL_ACK_RECOVERY_V1: receiver-window flush plus final-tail probing.

namespace ns3 {

static const char*
RdmaWrCqeStatusName(uint32_t status)
{
    switch (status)
    {
    case RDMA_WR_CQE_SUCCESS:
        return "SUCCESS";
    case RDMA_WR_CQE_RETRY_EXC_ERR:
        return "RETRY_EXC_ERR";
    case RDMA_WR_CQE_WR_FLUSH_ERR:
        return "WR_FLUSH_ERR";
    default:
        return "UNKNOWN";
    }
}

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
      m_rnicAckFlushEnabled(false),
      m_rnicAckFlushGuardNs(10000),
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
      m_userspaceInitialRttNs(20000),
      m_userspaceMinTailGuardOverrideNs(0),
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
    if (m_rdma != NULL && m_mode == MODE_RNIC)
    {
        m_rdma->SetRnicGateCallbacks(
            MakeCallback(&RdmaTransport::RnicGateAllowsQp, this),
            MakeCallback(&RdmaTransport::GetNextRnicGateTime, this));
    }
    RefreshRnicAckFlushEvents();
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

    if (m_rdma != NULL)
    {
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
    RefreshRnicAckFlushEvents();
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
RdmaTransport::ConfigureRnicAckFlush(bool enabled, uint64_t guardNs)
{
    m_rnicAckFlushEnabled = enabled;
    m_rnicAckFlushGuardNs = guardNs;

    std::cout << "[RNIC ACK FLUSH CONFIG]"
              << " node="
              << (m_node != NULL
                      ? static_cast<int64_t>(m_node->GetId())
                      : -1)
              << " enabled=" << (m_rnicAckFlushEnabled ? 1 : 0)
              << " configured_guard_ns=" << m_rnicAckFlushGuardNs
              << " policy=per_port_static"
              << std::endl;

    RefreshRnicAckFlushEvents();
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
        // Compatibility path for callers that already supply an RTT bootstrap
        // value.  Mode-2 never reads qp->m_baseRtt for admission.
        m_userspaceInitialRttNs = maxRttNs;
    }

    m_activeQpHint = std::max<uint32_t>(1, activeQpHint);

    if (m_userspaceAdmissionMode == USERSPACE_OCS_WINDOWED)
    {
        ApplyBandwidthNormalizedConfig("explicit_config");
    }
}

void
RdmaTransport::ConfigureUserspaceInitialRtt(uint64_t initialRttNs)
{
    m_userspaceInitialRttNs = std::max<uint64_t>(initialRttNs, 1ULL);

    std::cout
        << "[USERSPACE TIMING CONFIG]"
        << " node="
        << (m_node != NULL ? static_cast<int64_t>(m_node->GetId()) : -1)
        << " initial_rtt_ns=" << m_userspaceInitialRttNs
        << " source=configured_userspace_bootstrap"
        << " online_estimator=cqe_completion_srtt_rttvar"
        << " rnics_internal_rtt_visible=0"
        << std::endl;

}

void
RdmaTransport::ConfigureUserspaceMinTailGuard(uint64_t minTailGuardNs)
{
    // Zero keeps the deployment-mode default. A positive value is an explicit
    // userspace policy knob and does not expose RNIC-internal timing state.
    m_userspaceMinTailGuardOverrideNs = minTailGuardNs;

    std::cout
        << "[USERSPACE GUARD CONFIG]"
        << " node="
        << (m_node != NULL ? static_cast<int64_t>(m_node->GetId()) : -1)
        << " requested_min_tail_guard_ns="
        << m_userspaceMinTailGuardOverrideNs
        << " source="
        << (m_userspaceMinTailGuardOverrideNs > 0
                ? "configured_override"
                : "deployment_mode_default")
        << std::endl;
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
    if (m_userspaceInitialRttNs == 0)
    {
        m_userspaceInitialRttNs = 20000ULL;
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
    m_maxTailGuardNs =
        m_realDeploymentMode ? 5000000ULL : 1000000ULL;
    const uint64_t modeDefaultMinTailGuardNs =
        m_realDeploymentMode ? 50000ULL : 10000ULL;
    m_minTailGuardNs =
        m_userspaceMinTailGuardOverrideNs > 0
            ? std::min<uint64_t>(
                  m_userspaceMinTailGuardOverrideNs,
                  m_maxTailGuardNs)
            : modeDefaultMinTailGuardNs;

    // This is only a template for diagnostics before any QP has been bound.
    // Runtime admission computes RTT_q + 4*RTTVAR_port + software guard.
    uint64_t templateGuardNs = m_userspaceInitialRttNs;
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
            << " initialRttEstimateNs=" << m_userspaceInitialRttNs
            << " initialRttSource=configured_userspace_bootstrap"
            << " configuredSwitchingGuardNs=" << m_switchingGuardNs
            << " switchingGuardAppliedNs=0"
            << " stableWindowEnd=1"
            << " stateScope=transport_template"
            << " cqeSrttNs=0"
            << " cqeRttvarNs=0"
            << " softwareGuardNs=" << m_userspaceSoftwareGuardNs
            << " minTailGuardNs=" << m_minTailGuardNs
            << " minTailGuardSource="
            << (m_userspaceMinTailGuardOverrideNs > 0
                    ? "configured_override"
                    : "deployment_mode_default")
            << " maxOutstandingBytes=" << m_maxOutstandingBytes
            << " maxOutstandingCeilingBytes=" << m_maxOutstandingCeilingBytes
            << " wrChunkBytes=" << m_wrChunkBytes
            << " pipelineCapacityFixed=1"
            << " minPostBytes=" << m_minPostBytes
            << " tailGuardNs=" << m_tailGuardNs
            << " guardModel=cqe_srtt_or_initial_plus_4xport_cqe_var_plus_software"
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

    RefreshRnicAckFlushEvents();
}

void
RdmaTransport::ClearGateTables()
{
    CancelRnicAckFlushEvents();
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

uint64_t
RdmaTransport::MakeRnicAckFlushEventKey(
    uint32_t rnicPort,
    uint32_t slotIndex)
{
    return (static_cast<uint64_t>(rnicPort) << 32) | slotIndex;
}

uint64_t
RdmaTransport::GetRnicAckFlushGuardNs(
    uint32_t rnicPort,
    uint64_t windowLengthNs) const
{
    (void)rnicPort;
    if (windowLengthNs == 0)
    {
        return 0;
    }

    // This is the single extension point for a future per-port adaptive guard.
    // The static configuration remains the deterministic fallback/initial value.
    const uint64_t configuredGuardNs =
        std::max<uint64_t>(1, m_rnicAckFlushGuardNs);
    return std::min<uint64_t>(configuredGuardNs, windowLengthNs);
}

void
RdmaTransport::CancelRnicAckFlushEvents()
{
    for (std::map<uint64_t, EventId>::iterator it =
             m_rnicAckFlushEvents.begin();
         it != m_rnicAckFlushEvents.end();
         ++it)
    {
        if (it->second.IsRunning())
        {
            it->second.Cancel();
        }
    }
    m_rnicAckFlushEvents.clear();
}

void
RdmaTransport::RefreshRnicAckFlushEvents()
{
    CancelRnicAckFlushEvents();
    if (!m_rnicAckFlushEnabled ||
        m_mode != MODE_RNIC ||
        !m_gateEnabled ||
        m_rdma == NULL)
    {
        return;
    }

    for (std::map<uint32_t, GateTable>::const_iterator tableIt =
             m_gateTables.begin();
         tableIt != m_gateTables.end();
         ++tableIt)
    {
        for (uint32_t slotIndex = 0;
             slotIndex < tableIt->second.slots.size();
             ++slotIndex)
        {
            ScheduleRnicAckFlushEvent(tableIt->first, slotIndex);
        }
    }
}

void
RdmaTransport::ScheduleRnicAckFlushEvent(
    uint32_t rnicPort,
    uint32_t slotIndex)
{
    std::map<uint32_t, GateTable>::const_iterator tableIt =
        m_gateTables.find(rnicPort);
    if (!m_rnicAckFlushEnabled ||
        m_mode != MODE_RNIC ||
        m_rdma == NULL ||
        tableIt == m_gateTables.end() ||
        tableIt->second.periodNs == 0 ||
        slotIndex >= tableIt->second.slots.size())
    {
        return;
    }

    const GateTable& table = tableIt->second;
    const GateSlotEntry& slot = table.slots[slotIndex];
    if (slot.endOffsetNs <= slot.startOffsetNs ||
        slot.endOffsetNs > table.periodNs ||
        slot.dstRnicBitmapWords.empty())
    {
        return;
    }

    const uint64_t windowLengthNs =
        slot.endOffsetNs - slot.startOffsetNs;
    const uint64_t guardNs =
        GetRnicAckFlushGuardNs(rnicPort, windowLengthNs);
    const uint64_t flushOffsetNs = slot.endOffsetNs - guardNs;
    const uint64_t nowNs =
        static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());

    uint64_t targetNs = table.epochStartNs + flushOffsetNs;
    if (targetNs <= nowNs)
    {
        const uint64_t periodsElapsed =
            (nowNs - targetNs) / table.periodNs + 1;
        targetNs += periodsElapsed * table.periodNs;
    }

    const uint64_t periodBaseNs = targetNs - flushOffsetNs;
    const uint64_t windowStartNs =
        periodBaseNs + slot.startOffsetNs;
    const uint64_t windowEndNs =
        periodBaseNs + slot.endOffsetNs;
    const uint64_t key =
        MakeRnicAckFlushEventKey(rnicPort, slotIndex);

    m_rnicAckFlushEvents[key] = Simulator::Schedule(
        NanoSeconds(targetNs - nowNs),
        &RdmaTransport::RunRnicAckFlushEvent,
        this,
        rnicPort,
        slotIndex,
        windowStartNs,
        windowEndNs);
}

void
RdmaTransport::RunRnicAckFlushEvent(
    uint32_t rnicPort,
    uint32_t slotIndex,
    uint64_t windowStartNs,
    uint64_t windowEndNs)
{
    std::map<uint32_t, GateTable>::const_iterator tableIt =
        m_gateTables.find(rnicPort);
    if (m_rnicAckFlushEnabled &&
        m_mode == MODE_RNIC &&
        m_rdma != NULL &&
        tableIt != m_gateTables.end() &&
        slotIndex < tableIt->second.slots.size())
    {
        const GateSlotEntry& slot = tableIt->second.slots[slotIndex];
        m_rdma->FlushRnicPortAcks(
            rnicPort,
            slot.dstRnicBitmapWords,
            windowStartNs,
            windowEndNs);
    }

    ScheduleRnicAckFlushEvent(rnicPort, slotIndex);
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
        // A permit is consumed only by GetNxtPacket(). Recompute it for each
        // scheduler eligibility check so a delayed ACK can cancel the bypass.
        qp->m_ackRecoveryPermit = false;
        qp->m_ackRecoveryPermitType = RNIC_ACK_PROBE_NONE;
    }

    if (gate.bypass)
    {
        return true;
    }
    if (qp == NULL)
    {
        return false;
    }
    if (!gate.allowed)
    {
        const bool finalOutstanding =
            qp->GetBytesLeft() == 0 &&
            qp->snd_una < qp->m_highestSentSeq;
        const bool continuationBlocked =
            qp->GetBytesLeft() > 0 &&
            qp->IsWinBound() &&
            qp->snd_una < qp->snd_nxt &&
            qp->snd_nxt >= qp->m_highestSentSeq;

        // Recovery may first become visible just after a window closes (for
        // example after the final DATA finishes serialization). Arm the
        // immediately upcoming reachable window instead of waiting one extra
        // optical period before creating the permit.
        if (finalOutstanding || continuationBlocked)
        {
            ArmRnicAckRecovery(
                qp,
                gate,
                finalOutstanding
                    ? "final_tail_next_reachable"
                    : "window_bound_next_reachable");
        }
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

    const bool finalOutstanding =
        qp->GetBytesLeft() == 0 &&
        qp->snd_una < qp->m_highestSentSeq;
    if (finalOutstanding)
    {
        if (CanSendRnicAckProbe(qp, gate, now))
        {
            qp->m_deadlineLastAllowedWindowStartNs =
                static_cast<uint64_t>(gate.currentWindowStart.GetNanoSeconds());
            qp->m_deadlineLastAllowedWindowEndNs = windowEndNs;
            return true;
        }
        ArmRnicAckRecovery(qp, gate, "final_tail");
        return false;
    }

    if (!qp->IsWinBound())
    {
        qp->m_deadlineLastAllowedWindowStartNs =
            static_cast<uint64_t>(gate.currentWindowStart.GetNanoSeconds());
        qp->m_deadlineLastAllowedWindowEndNs = windowEndNs;
        return true;
    }

    if (CanSendRnicAckProbe(qp, gate, now))
    {
        qp->m_deadlineLastAllowedWindowStartNs =
            static_cast<uint64_t>(gate.currentWindowStart.GetNanoSeconds());
        qp->m_deadlineLastAllowedWindowEndNs = windowEndNs;
        return true;
    }

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

    if (qp != NULL &&
        m_rdma != NULL &&
        m_rdma->IsRnicAckRecoveryEnabled() &&
        qp->m_ackRecoveryActive &&
        qp->m_ackRecoveryNextWindowStartNs >
            static_cast<uint64_t>(now.GetNanoSeconds()))
    {
        return NanoSeconds(qp->m_ackRecoveryNextWindowStartNs);
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
        return Simulator::GetMaximumSimulationTime();
    }

    const bool finalOutstanding =
        qp != NULL &&
        qp->GetBytesLeft() == 0 &&
        qp->snd_una < qp->m_highestSentSeq;
    if (finalOutstanding)
    {
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
    if (!gate.allowed)
    {
        return gate.nextAllowedTime;
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
        qp == NULL || gate.bypass)
    {
        return;
    }

    const bool hasContinuation =
        qp->GetBytesLeft() > 0 &&
        qp->snd_una < qp->snd_nxt &&
        qp->snd_nxt >= qp->m_highestSentSeq;
    const bool hasFinalOutstanding =
        qp->GetBytesLeft() == 0 && qp->snd_una < qp->m_highestSentSeq;
    if (!hasContinuation && !hasFinalOutstanding)
    {
        return;
    }

    const RnicAckRecoveryProbeType desiredType =
        hasFinalOutstanding
            ? RNIC_ACK_PROBE_TAIL
            : RNIC_ACK_PROBE_CONTINUATION;

    // Continuation probing retains the original bounded-attempt policy. Tail
    // probing is one segment per reachable window and remains live until ACK,
    // NACK, or partial cumulative progress resolves the final outstanding data.
    if (desiredType == RNIC_ACK_PROBE_CONTINUATION &&
        qp->m_ackRecoveryAttempts >=
            m_rdma->GetRnicAckRecoveryMaxAttempts())
    {
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
                      << " probe_type=continuation"
                      << " attempts=" << qp->m_ackRecoveryAttempts
                      << " snd_una=" << qp->snd_una
                      << " snd_nxt=" << qp->snd_nxt
                      << std::endl;
        }
        qp->m_ackRecoveryPermit = false;
        qp->m_ackRecoveryPermitType = RNIC_ACK_PROBE_NONE;
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
    const uint64_t sourceWindowEndNs =
        gate.allowed
            ? static_cast<uint64_t>(gate.currentWindowEnd.GetNanoSeconds())
            : 0;
    // Reuse the existing field as a stable arm marker. Outside a reachable
    // window, the upcoming window start is stable whereas Simulator::Now()
    // is not, so repeated scheduler checks still coalesce into one arm.
    const uint64_t armMarkerNs =
        gate.allowed ? sourceWindowEndNs : nextWindowNs;

    if (qp->m_ackRecoveryLastArmWindowEndNs == armMarkerNs &&
        qp->m_ackRecoveryNextWindowStartNs == nextWindowNs)
    {
        return;
    }

    qp->m_ackRecoveryActive = true;
    qp->m_ackRecoveryPermit = false;
    qp->m_ackRecoveryPermitType = RNIC_ACK_PROBE_NONE;
    qp->m_ackRecoveryNextWindowStartNs = nextWindowNs;
    qp->m_ackRecoveryLastArmWindowEndNs = armMarkerNs;
    qp->m_ackRecoveryArmCount++;

    std::cout << "[RNIC ACK RECOVERY ARM]"
              << " t_ns=" << Simulator::Now().GetNanoSeconds()
              << " node="
              << (m_node != NULL ? static_cast<int64_t>(m_node->GetId()) : -1)
              << " src=" << qp->m_src
              << " dst=" << qp->m_dest
              << " sport=" << qp->sport
              << " probe_type="
              << (desiredType == RNIC_ACK_PROBE_TAIL
                      ? "tail"
                      : "continuation")
              << " plane="
              << (qp->m_hasBoundRnicPort
                      ? static_cast<int64_t>(qp->m_boundPlaneId)
                      : -1)
              << " rnic_port="
              << (qp->m_hasBoundRnicPort
                      ? static_cast<int64_t>(qp->m_boundRnicPort)
                      : -1)
              << " source_window_allowed=" << (gate.allowed ? 1 : 0)
              << " source_window_start_ns="
              << (gate.allowed
                      ? gate.currentWindowStart.GetNanoSeconds()
                      : -1)
              << " source_window_end_ns=" << sourceWindowEndNs
              << " next_window_start_ns=" << nextWindowNs
              << " snd_una=" << qp->snd_una
              << " snd_nxt=" << qp->snd_nxt
              << " highest_sent=" << qp->m_highestSentSeq
              << " on_the_fly=" << qp->GetOnTheFly()
              << " win=" << qp->GetWin()
              << " reason=" << reason
              << std::endl;
}

bool
RdmaTransport::CanSendRnicAckProbe(
    Ptr<RdmaQueuePair> qp,
    const GateLookupResult& gate,
    Time now) const
{
    if (m_rdma == NULL || !m_rdma->IsRnicAckRecoveryEnabled() ||
        qp == NULL || !qp->m_ackRecoveryActive)
    {
        return false;
    }

    const bool hasContinuation =
        qp->GetBytesLeft() > 0 &&
        qp->snd_una < qp->snd_nxt &&
        qp->snd_nxt >= qp->m_highestSentSeq;
    const bool hasFinalOutstanding =
        qp->GetBytesLeft() == 0 && qp->snd_una < qp->m_highestSentSeq;
    if (!hasContinuation && !hasFinalOutstanding)
    {
        return false;
    }

    const RnicAckRecoveryProbeType desiredType =
        hasFinalOutstanding
            ? RNIC_ACK_PROBE_TAIL
            : RNIC_ACK_PROBE_CONTINUATION;
    if (desiredType == RNIC_ACK_PROBE_CONTINUATION &&
        qp->m_ackRecoveryAttempts >=
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
        if (qp->m_ackRecoveryLastProbeWindowStartNs == windowStartNs)
        {
            ArmRnicAckRecovery(qp, gate, "probe_already_sent");
        }
        return false;
    }

    m_rdma->InvalidateRnicDeadlineSample(qp);
    qp->m_ackRecoveryPermit = true;
    qp->m_ackRecoveryPermitType = desiredType;
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
        << " initialRttEstimateNs=" << m_userspaceInitialRttNs
        << " timingSource=configured_userspace_bootstrap"
        << " softwareGuardNs=" << m_userspaceSoftwareGuardNs
        << " minTailGuardNs=" << m_minTailGuardNs
        << " minTailGuardSource="
        << (m_userspaceMinTailGuardOverrideNs > 0
                ? "configured_override"
                : "deployment_mode_default")
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
    // A real rdma-core userspace process cannot inspect RNIC snd_una/PSN or
    // a hidden per-QP path RTT.  Bootstrap from explicit configuration, then
    // use only completion timing observable at the CQ polling boundary.
    const uint64_t timingEstimateNs =
        portState.cqeTimingSamples > 0 && portState.cqeLatencySrttNs > 0
            ? portState.cqeLatencySrttNs
            : std::max<uint64_t>(m_userspaceInitialRttNs, 1ULL);
    const uint64_t variationGuardNs =
        portState.cqeLatencyRttvarNs >
                std::numeric_limits<uint64_t>::max() / 4
            ? std::numeric_limits<uint64_t>::max()
            : 4 * portState.cqeLatencyRttvarNs;

    uint64_t guardNs = timingEstimateNs;
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

    const uint64_t baselineGuardNs =
        ClampValue(guardNs, m_minTailGuardNs, m_maxTailGuardNs);
    const uint64_t anomalyPenaltyNs =
        std::min<uint64_t>(
            portState.cqeAnomalyPenaltyNs,
            m_maxTailGuardNs);
    return std::max<uint64_t>(baselineGuardNs, anomalyPenaltyNs);
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

    // A commodity RNIC may internally retry for a long time and still return
    // a SUCCESS CQE. Userspace cannot observe the retry itself, only the
    // post_send-to-CQE latency. Isolate extreme SUCCESS latency from the
    // baseline SRTT/RTTVAR and express it as a temporary safety penalty.
    const uint64_t baselineEstimateNs =
        portState.cqeTimingSamples > 0 && portState.cqeLatencySrttNs > 0
            ? portState.cqeLatencySrttNs
            : std::max<uint64_t>(m_userspaceInitialRttNs, 1ULL);
    const uint64_t scaledBaselineNs =
        baselineEstimateNs >
                std::numeric_limits<uint64_t>::max() / 8
            ? std::numeric_limits<uint64_t>::max()
            : 8 * baselineEstimateNs;
    const uint64_t anomalyThresholdNs =
        std::max<uint64_t>(m_maxTailGuardNs, scaledBaselineNs);
    const bool anomalySample =
        completionLatencyNs > anomalyThresholdNs;

    if (anomalySample)
    {
        portState.cqeAnomalyPenaltyNs = m_maxTailGuardNs;
        ++portState.cqeAnomalyCount;
        portState.cqeAnomalySinceLastAdapt = true;
        portState.stableAdaptPeriods = 0;
    }
    else if (portState.cqeTimingSamples == 0)
    {
        portState.cqeLatencySrttNs = completionLatencyNs;
        portState.cqeLatencyRttvarNs = 0;
        ++portState.cqeTimingSamples;
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
        ++portState.cqeTimingSamples;
    }

    const uint64_t newTailGuardNs =
        GetUserspaceTailGuardNs(qp, portState);
    const bool powerOfTwoSample =
        portState.cqeTimingSamples > 0 &&
        (portState.cqeTimingSamples & (portState.cqeTimingSamples - 1)) == 0;
    const uint64_t timingEstimateNs =
        portState.cqeTimingSamples > 0 && portState.cqeLatencySrttNs > 0
            ? portState.cqeLatencySrttNs
            : baselineEstimateNs;

    if (m_enabled &&
        (anomalySample ||
         portState.cqeTimingSamples <= 4 ||
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
            << " sampleClass=" << (anomalySample ? "anomaly" : "baseline")
            << " anomalyThresholdNs=" << anomalyThresholdNs
            << " anomalyPenaltyNs=" << portState.cqeAnomalyPenaltyNs
            << " anomalyCount=" << portState.cqeAnomalyCount
            << " cqeSrttNs=" << portState.cqeLatencySrttNs
            << " cqeRttvarNs=" << portState.cqeLatencyRttvarNs
            << " timingEstimateNs=" << timingEstimateNs
            << " timingSource="
            << (anomalySample
                    ? "cqe_completion_anomaly_penalty"
                    : "cqe_completion_srtt")
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
    const uint64_t oldAnomalyPenaltyNs =
        portState.cqeAnomalyPenaltyNs;
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
    else if (portState.cqeAnomalySinceLastAdapt)
    {
        // Keep the full anomaly penalty for at least one adaptation epoch.
        portState.stableAdaptPeriods = 0;
        action = "timing_anomaly_hold";
    }
    else if (portState.cqeAnomalyPenaltyNs > 0 &&
             portState.backlogSinceLastAdapt &&
             portState.wrCompletionSinceLastAdapt)
    {
        // Decay once per stable adaptation epoch, not once per CQE.
        uint64_t decayedPenaltyNs =
            portState.cqeAnomalyPenaltyNs / 2;

        UserspacePortState baselineState = portState;
        baselineState.cqeAnomalyPenaltyNs = 0;
        const uint64_t baselineGuardNs =
            GetUserspaceTailGuardNs(qp, baselineState);

        if (decayedPenaltyNs <= baselineGuardNs)
        {
            decayedPenaltyNs = 0;
        }

        portState.cqeAnomalyPenaltyNs = decayedPenaltyNs;
        portState.stableAdaptPeriods = 0;
        action = decayedPenaltyNs == 0
            ? "timing_penalty_clear"
            : "timing_penalty_decay";
    }
    else if (portState.cqeAnomalyPenaltyNs == 0 &&
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
        oldAnomalyPenaltyNs != portState.cqeAnomalyPenaltyNs ||
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
            << " oldAnomalyPenaltyNs=" << oldAnomalyPenaltyNs
            << " newAnomalyPenaltyNs="
            << portState.cqeAnomalyPenaltyNs
            << " anomalyCount=" << portState.cqeAnomalyCount
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
    portState.cqeAnomalySinceLastAdapt = false;
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

    std::map<uint64_t, UserspaceQpState>::const_iterator stateIt =
        m_userspaceQpStates.find(key);
    if (stateIt != m_userspaceQpStates.end() &&
        stateIt->second.operationFailureNotified)
    {
        return;
    }

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
        << " error_cqe_count=" << st.errorCompletionCount
        << " retry_exceeded_cqe_count=" << st.retryExceededCount
        << " wr_flush_error_cqe_count=" << st.wrFlushErrorCount
        << " failed_wr_id=" << st.failedWrId
        << " first_error_status=" << RdmaWrCqeStatusName(st.firstErrorStatus)
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
            << " cqe_anomaly_penalty_ns="
            << portState->cqeAnomalyPenaltyNs
            << " cqe_anomaly_count="
            << portState->cqeAnomalyCount
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
        << " guard_model=max_cqe_baseline_and_anomaly_penalty"
        << " completion_semantic=signaled_wr_busy_poll"
        << " sample_limit=" << m_postLogSampleLimit
        << std::endl;

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
        qp->GetUnpostedBytes() == 0)
    {
        return;
    }

    UserspaceQpState& state = GetUserspaceQpState(qp);
    if (state.operationFailureNotified)
    {
        return;
    }

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
        qp->GetUnpostedBytes() == 0)
    {
        return;
    }

    UserspaceQpState& state = GetUserspaceQpState(qp);
    if (state.operationFailureNotified)
    {
        return;
    }

    UserspacePortState& portState =
        GetUserspacePortState(qp, "submit");

    // A terminal CQE may leave recovery feedback pending when the short
    // adaptation debounce suppresses the immediate MaybeAdapt() call.  Never
    // admit a new WR with stale per-port safety parameters: consume pending
    // recovery before computing the OCS safe budget.  If the debounce has not
    // expired yet, defer this QP until it can be applied.
    if (portState.recoverySinceLastAdapt)
    {
        MaybeAdapt(qp);
        if (portState.recoverySinceLastAdapt)
        {
            const uint64_t nowNs = static_cast<uint64_t>(
                Simulator::Now().GetNanoSeconds());
            const uint64_t earliestAdaptNs =
                portState.lastAdaptNs + 1000000ULL;
            const uint64_t wakeNs = std::max<uint64_t>(
                nowNs + 1ULL,
                earliestAdaptNs);

            portState.backlogSinceLastAdapt = true;
            std::cout
                << "[USERSPACE RECOVERY ADMISSION BARRIER]"
                << " t_ns=" << nowNs
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
                << " wake_ns=" << wakeNs
                << " reason=pending_recovery_before_admission"
                << " stateScope=per_rnic_port"
                << std::endl;

            ScheduleNextWake(qp, NanoSeconds(wakeNs));
            return;
        }
    }

    if (qp->GetUnpostedBytes() > 0)
    {
        portState.backlogSinceLastAdapt = true;
    }

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


RdmaTransport::UserspaceCompletionResult
RdmaTransport::NotifyWrCompletion(
    Ptr<RdmaQueuePair> qp,
    uint64_t wrId,
    uint64_t bytes,
    uint64_t postTimeNs,
    uint64_t completionTimeNs,
    uint32_t cqeStatus)
{
    if (!m_enabled || qp == NULL)
    {
        return USERSPACE_COMPLETION_NONE;
    }

    NS_ASSERT_MSG(
        cqeStatus == RDMA_WR_CQE_SUCCESS ||
            cqeStatus == RDMA_WR_CQE_RETRY_EXC_ERR ||
            cqeStatus == RDMA_WR_CQE_WR_FLUSH_ERR,
        "Mode-2 received an unknown simulated verbs CQE status");

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

    const uint64_t latencyNs = completionTimeNs >= postTimeNs
        ? completionTimeNs - postTimeNs
        : 0;
    PostStats& stats = m_postStats[key];

    // Error CQEs are terminal from userspace's perspective.  Userspace never
    // inspects snd_una/PSN/retry state; it only consumes the status returned by
    // the simulated ibv_poll_cq() boundary.
    if (cqeStatus != RDMA_WR_CQE_SUCCESS)
    {
        NS_ASSERT_MSG(
            !state.operationCompleteNotified,
            "Mode-2 received an error CQE after successful operation completion");

        state.outstandingBytes -= bytes;
        if (portState != NULL)
        {
            portState->aggregateOutstandingBytes -= bytes;
            portState->recoverySinceLastAdapt = true;
        }

        stats.errorCompletionCount++;
        if (cqeStatus == RDMA_WR_CQE_RETRY_EXC_ERR)
        {
            stats.retryExceededCount++;
        }
        else if (cqeStatus == RDMA_WR_CQE_WR_FLUSH_ERR)
        {
            stats.wrFlushErrorCount++;
        }

        if (stats.failedWrId == 0)
        {
            stats.failedWrId = wrId;
            stats.firstErrorStatus = cqeStatus;
        }

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
            << " status=" << RdmaWrCqeStatusName(cqeStatus)
            << " qp_outstanding_bytes=" << state.outstandingBytes
            << " port_outstanding_bytes="
            << (portState != NULL
                    ? portState->aggregateOutstandingBytes
                    : state.outstandingBytes)
            << " semantic=signaled_wr_error_cqe"
            << std::endl;

        UserspaceCompletionResult result = USERSPACE_COMPLETION_NONE;
        if (!state.operationFailureNotified)
        {
            state.operationFailureNotified = true;
            state.failedWrId = wrId;
            state.firstErrorStatus = cqeStatus;

            std::map<uint64_t, EventId>::iterator wakeIt =
                m_wakeEvents.find(key);
            if (wakeIt != m_wakeEvents.end())
            {
                if (wakeIt->second.IsRunning())
                {
                    wakeIt->second.Cancel();
                }
                m_wakeEvents.erase(wakeIt);
            }

            std::cout
                << "[USERSPACE OP FAILED]"
                << " t_ns=" << completionTimeNs
                << " src="
                << (m_node != NULL
                        ? static_cast<int64_t>(m_node->GetId())
                        : -1)
                << " dst=" << qp->GetDest()
                << " sport=" << qp->sport
                << " dport=" << qp->dport
                << " pg=" << qp->m_pg
                << " failed_wr_id=" << wrId
                << " status=" << RdmaWrCqeStatusName(cqeStatus)
                << " posted_bytes=" << state.postedBytes
                << " completed_bytes=" << state.completedBytes
                << " outstanding_bytes=" << state.outstandingBytes
                << " unposted_bytes=" << qp->GetUnpostedBytes()
                << " semantic=ibv_poll_cq_error"
                << std::endl;

            result = USERSPACE_COMPLETION_ERROR;
        }

        if (portState != NULL)
        {
            MaybeAdapt(qp);
            SchedulePortQpWakeups(qp);
        }

        // The RNIC emits RETRY_EXC_ERR for the failed WR and WR_FLUSH_ERR for
        // every later posted WR before teardown.  Emit one final summary after
        // userspace has reaped that entire error-CQE set.
        if (state.outstandingBytes == 0)
        {
            FlushPostSummary(qp);
        }
        return result;
    }

    NS_ASSERT_MSG(
        !state.operationFailureNotified,
        "Mode-2 received a success CQE after operation failure");

    state.outstandingBytes -= bytes;
    state.completedBytes += bytes;
    if (portState != NULL)
    {
        portState->aggregateOutstandingBytes -= bytes;
    }

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
            << " status=SUCCESS"
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
    // Userspace admission may only depend on posted-WR/CQE state; it must not
    // inspect RNIC-internal ACK progress such as snd_una/IsFinished().
    if (qp->GetUnpostedBytes() > 0 &&
        m_userspaceAdmissionMode == USERSPACE_DEFAULT_PIPELINE)
    {
        TrySubmit(qp);
    }

    NS_ASSERT_MSG(
        state.completedBytes <= state.postedBytes,
        "Mode-2 CQE ledger completed more bytes than userspace posted");
    NS_ASSERT_MSG(
        state.postedBytes <= qp->m_size,
        "Mode-2 userspace posted more bytes than the operation size");

    const bool allBytesPosted =
        qp->GetUnpostedBytes() == 0 && state.postedBytes == qp->m_size;
    const bool allCqesObserved =
        state.outstandingBytes == 0 && state.completedBytes == state.postedBytes;

    if (allBytesPosted && allCqesObserved && !state.operationCompleteNotified)
    {
        state.operationCompleteNotified = true;

        std::cout
            << "[USERSPACE OP COMPLETE]"
            << " t_ns=" << completionTimeNs
            << " src="
            << (m_node != NULL
                    ? static_cast<int64_t>(m_node->GetId())
                    : -1)
            << " dst=" << qp->GetDest()
            << " sport=" << qp->sport
            << " dport=" << qp->dport
            << " pg=" << qp->m_pg
            << " completed_bytes=" << state.completedBytes
            << " outstanding_bytes=" << state.outstandingBytes
            << " final_wr_id=" << wrId
            << " semantic=final_signaled_wr_cqe"
            << std::endl;

        // Emit userspace statistics at the same boundary visible to a real
        // ibv_poll_cq() loop. State itself stays alive until RNIC teardown so
        // NotifyQpComplete() can validate that hardware did not complete early.
        FlushPostSummary(qp);
        return USERSPACE_COMPLETION_SUCCESS;
    }

    return USERSPACE_COMPLETION_NONE;
}

void
RdmaTransport::NotifyQpComplete(Ptr<RdmaQueuePair> qp)
{
    if (!m_enabled || qp == NULL)
    {
        return;
    }

    const uint64_t key = GetPostStatsKey(qp);
    std::map<uint64_t, UserspaceQpState>::const_iterator stateIt =
        m_userspaceQpStates.find(key);
    NS_ASSERT_MSG(
        stateIt != m_userspaceQpStates.end(),
        "Mode-2 RNIC teardown arrived without a userspace QP ledger");

    const UserspaceQpState& state = stateIt->second;
    const bool success = state.operationCompleteNotified;
    const bool failed = state.operationFailureNotified;
    NS_ASSERT_MSG(
        success != failed,
        "RNIC QP teardown requires exactly one userspace terminal CQE outcome");

    if (success)
    {
        NS_ASSERT_MSG(
            state.outstandingBytes == 0 && state.completedBytes == qp->m_size,
            "RNIC QP teardown disagrees with the Mode-2 successful CQE ledger");
    }
    else
    {
        NS_ASSERT_MSG(
            state.outstandingBytes == 0,
            "RNIC QP error teardown occurred before userspace reaped all error CQEs");
        NS_ASSERT_MSG(
            state.firstErrorStatus != RDMA_WR_CQE_SUCCESS,
            "Mode-2 failed QP is missing a verbs-style error CQE status");
    }

    std::cout
        << "[USERSPACE RNIC TEARDOWN]"
        << " t_ns=" << Simulator::Now().GetNanoSeconds()
        << " src="
        << (m_node != NULL
                ? static_cast<int64_t>(m_node->GetId())
                : -1)
        << " dst=" << qp->GetDest()
        << " sport=" << qp->sport
        << " dport=" << qp->dport
        << " pg=" << qp->m_pg
        << " completed_bytes=" << state.completedBytes
        << " outstanding_bytes=" << state.outstandingBytes
        << " outcome=" << (success ? "success" : "error")
        << " failed_wr_id=" << state.failedWrId
        << " first_error_status=" << RdmaWrCqeStatusName(state.firstErrorStatus)
        << " userspace_complete_observed="
        << (state.operationCompleteNotified ? 1 : 0)
        << " userspace_failure_observed="
        << (state.operationFailureNotified ? 1 : 0)
        << " semantic=rnic_cleanup_after_terminal_cqes"
        << std::endl;

    UserspacePortState* portState = NULL;
    if (m_userspaceAdmissionMode == USERSPACE_OCS_WINDOWED &&
        qp->m_hasBoundRnicPort)
    {
        portState = &GetUserspacePortState(qp, "qp_complete");
    }

    // Normally already emitted at the final success CQE or final error/flush
    // CQE; harmless no-op if no post stats remain.
    FlushPostSummary(qp);

    std::map<uint64_t, EventId>::iterator wakeIt = m_wakeEvents.find(key);
    if (wakeIt != m_wakeEvents.end())
    {
        if (wakeIt->second.IsRunning())
        {
            wakeIt->second.Cancel();
        }
        m_wakeEvents.erase(wakeIt);
    }

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