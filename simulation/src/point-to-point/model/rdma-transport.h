#ifndef RDMA_TRANSPORT_H
#define RDMA_TRANSPORT_H

#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/event-id.h"
#include "ns3/rdma-hw.h"
#include "ns3/rdma-queue-pair.h"

#include <map>
#include <vector>
#include <stdint.h>

// MODE2_CQE_SEMANTICS_V1: userspace observes WR completions, not transport ACKs.
// MODE2_VERBS_COMPLETION_V2: userspace operation completion is owned by final signaled-WR CQE.
// MODE2_VERBS_ERROR_CQE_V1: userspace failure is observed only through CQE status.
// MODE2_INJECTION_WINDOW_PIPELINE_V1: OCS admission preserves multi-WR pipeline capacity.
// MODE2_OPTIMIZED_GUARD_V1: stable-end + CQE-jitter userspace safety boundary.
// MODE2_PER_PORT_TIMING_V1: isolate Mode-2 rate/CQE state by breakout RNIC port.
// MODE2_PER_PORT_AGGREGATE_ADMISSION_V1: account all in-flight WR bytes per breakout port.
// MODE2_PORT_QP_LOGGING_V1: separate configured QP hint from runtime per-port QP count.
// MODE2_DEFAULT_PIPELINE_V1: default Mode 2 uses a fixed-depth WR/CQE pipeline.
// MODE1_CONTINUATION_ACK_RECOVERY_V1: bounded next-window DATA probing.
// MODE1_FINAL_ACK_RECOVERY_V1: receiver-window flush plus final-tail probing.

namespace ns3 {

class RdmaTransport : public Object
{
public:
    static TypeId GetTypeId(void);

    RdmaTransport();

    void SetNode(Ptr<Node> node);
    void SetRdmaHw(Ptr<RdmaHw> rdma);

    enum GateMode
    {
        MODE_DEFAULT = 0,
        MODE_RNIC = 1,
        MODE_USERSPACE = 2
    };

    enum UserspaceAdmissionMode
    {
        USERSPACE_DEFAULT_PIPELINE = 0,
        USERSPACE_OCS_WINDOWED = 1
    };

    enum UserspaceCompletionResult
    {
        USERSPACE_COMPLETION_NONE = 0,
        USERSPACE_COMPLETION_SUCCESS = 1,
        USERSPACE_COMPLETION_ERROR = 2
    };

    struct GateSlotEntry
    {
        uint64_t startOffsetNs;
        uint64_t endOffsetNs;
        std::vector<uint64_t> dstRnicBitmapWords;
    };

    void SetMode(uint32_t mode);
    GateMode GetMode() const;

    void SetUserspaceAdmissionMode(uint32_t mode);
    UserspaceAdmissionMode GetUserspaceAdmissionMode() const;

    void SetEnabled(bool enabled); // compatibility helper; SetMode() is preferred

    void Configure(
        uint64_t wrChunkBytes,
        uint64_t maxOutstandingBytes);

    void ConfigureRnicAckFlush(
        bool enabled,
        uint64_t guardNs);

    // Bandwidth-normalized initialization for mode-2 userspace admission.
    // If bottleneckRateBps/switchingGuardNs/maxRttNs are zero, the transport
    // infers conservative defaults from the local RNIC and installed gate table.
    void ConfigureBandwidthNormalized(
        uint64_t bottleneckRateBps,
        uint64_t switchingGuardNs,
        uint64_t maxRttNs,
        uint32_t activeQpHint,
        bool realDeploymentMode);

    // Mode-2 userspace cannot read RNIC/path RTT state.  This configurable
    // bootstrap estimate is used only until success CQEs provide an observable
    // completion-latency SRTT/RTTVAR.
    void ConfigureUserspaceInitialRtt(uint64_t initialRttNs);

    // Optional userspace policy override for the minimum OCS tail guard.
    // Zero preserves the deployment-mode default; positive values are explicit.
    void ConfigureUserspaceMinTailGuard(uint64_t minTailGuardNs);

    void InstallGateTable(
        uint32_t rnicId,
        uint64_t epochStartNs,
        uint64_t periodNs,
        const std::vector<GateSlotEntry>& slots);

    // Mode-1 callbacks invoked from the RdmaHw egress scheduling point.
    bool RnicGateAllowsQp(Ptr<RdmaQueuePair> qp) const;
    Time GetNextRnicGateTime(Ptr<RdmaQueuePair> qp) const;

    void ClearGateTables();

    void RegisterQp(Ptr<RdmaQueuePair> qp);

    // Completion corresponding to one signaled WR/CQE. Status is the only
    // reliability/error information visible above the simulated verbs boundary.
    // The return value is terminal exactly once for success or failure.
    UserspaceCompletionResult NotifyWrCompletion(
        Ptr<RdmaQueuePair> qp,
        uint64_t wrId,
        uint64_t bytes,
        uint64_t postTimeNs,
        uint64_t completionTimeNs,
        uint32_t cqeStatus);

    // Final QP completion is delivered after all completed WR boundaries have
    // generated CQE-equivalent notifications.
    void NotifyQpComplete(Ptr<RdmaQueuePair> qp);

private:
    struct GateLookupResult
    {
        bool bypass;
        bool allowed;
        Time currentWindowStart;
        Time currentWindowEnd;
        Time nextAllowedTime;

        GateLookupResult()
            : bypass(false),
              allowed(false),
              currentWindowStart(Time(0)),
              currentWindowEnd(Time(0)),
              nextAllowedTime(Time(0))
        {
        }
    };

    struct UserspacePortState
    {
        uint32_t rnicPort;
        uint32_t planeId;
        uint32_t ifIndex;
        uint64_t bottleneckRateBps;
        uint64_t minSafeRateBps;
        uint64_t safeRateBps;
        uint64_t maxSafeRateBps;
        uint64_t cqeLatencySrttNs;
        uint64_t cqeLatencyRttvarNs;
        uint64_t cqeTimingSamples;
        uint64_t cqeAnomalyPenaltyNs;
        uint64_t cqeAnomalyCount;
        uint64_t aggregateOutstandingBytes;
        uint64_t aggregateAdmissionBlockEvents;
        uint32_t registeredQpCount;
        uint64_t lastAdaptNs;
        uint32_t stableAdaptPeriods;
        bool cqeAnomalySinceLastAdapt;
        bool recoverySinceLastAdapt;
        bool wrCompletionSinceLastAdapt;
        bool backlogSinceLastAdapt;

        UserspacePortState()
            : rnicPort(0),
              planeId(0),
              ifIndex(0),
              bottleneckRateBps(0),
              minSafeRateBps(0),
              safeRateBps(0),
              maxSafeRateBps(0),
              cqeLatencySrttNs(0),
              cqeLatencyRttvarNs(0),
              cqeTimingSamples(0),
              cqeAnomalyPenaltyNs(0),
              cqeAnomalyCount(0),
              aggregateOutstandingBytes(0),
              aggregateAdmissionBlockEvents(0),
              registeredQpCount(0),
              lastAdaptNs(0),
              stableAdaptPeriods(0),
              cqeAnomalySinceLastAdapt(false),
              recoverySinceLastAdapt(false),
              wrCompletionSinceLastAdapt(false),
              backlogSinceLastAdapt(false)
        {
        }
    };

    GateLookupResult LookupGate(
        Ptr<RdmaQueuePair> qp,
        Time now) const;

    Time GetNextPhysicalWindowStart(
        Ptr<RdmaQueuePair> qp,
        const GateLookupResult& gate) const;

    void ArmRnicAckRecovery(
        Ptr<RdmaQueuePair> qp,
        const GateLookupResult& gate,
        const char* reason) const;

    bool CanSendRnicAckProbe(
        Ptr<RdmaQueuePair> qp,
        const GateLookupResult& gate,
        Time now) const;

    void RefreshRnicAckFlushEvents();
    void CancelRnicAckFlushEvents();
    void ScheduleRnicAckFlushEvent(
        uint32_t rnicPort,
        uint32_t slotIndex);
    void RunRnicAckFlushEvent(
        uint32_t rnicPort,
        uint32_t slotIndex,
        uint64_t windowStartNs,
        uint64_t windowEndNs);
    uint64_t GetRnicAckFlushGuardNs(
        uint32_t rnicPort,
        uint64_t windowLengthNs) const;
    static uint64_t MakeRnicAckFlushEventKey(
        uint32_t rnicPort,
        uint32_t slotIndex);

    bool Allows(
        Ptr<RdmaQueuePair> qp,
        Time now) const;

    Time GetNextAllowedTime(
        Ptr<RdmaQueuePair> qp,
        Time now) const;

    void TrySubmit(Ptr<RdmaQueuePair> qp);

    void TrySubmitDefaultPipeline(Ptr<RdmaQueuePair> qp);

    void TrySubmitOcsWindowed(Ptr<RdmaQueuePair> qp);

    void ScheduleNextWake(
        Ptr<RdmaQueuePair> qp,
        Time wakeTime);

    uint64_t GetSafeBudgetBytes(
        Ptr<RdmaQueuePair> qp,
        const GateLookupResult& gate,
        Time now);

    UserspacePortState& GetUserspacePortState(
        Ptr<RdmaQueuePair> qp,
        const char* reason);

    const UserspacePortState* FindUserspacePortState(
        Ptr<RdmaQueuePair> qp) const;

    void SchedulePortQpWakeups(
        Ptr<RdmaQueuePair> triggerQp);

    uint64_t GetPortBottleneckRateBps(
        Ptr<RdmaQueuePair> qp) const;

    uint64_t GetUserspaceTailGuardNs(
        Ptr<RdmaQueuePair> qp,
        const UserspacePortState& portState) const;

    void UpdateCqeTimingModel(
        Ptr<RdmaQueuePair> qp,
        uint64_t completionLatencyNs);

    void MaybeAdapt(Ptr<RdmaQueuePair> qp);

    uint64_t GetAdaptivePeriodNs() const;

    uint64_t ClampValue(
        uint64_t value,
        uint64_t minValue,
        uint64_t maxValue) const;

    uint64_t RoundUpPowerOfTwo(uint64_t value) const;

    uint64_t GetLocalBottleneckRateBps() const;

    uint64_t InferSwitchingGuardNs() const;

    void ApplyBandwidthNormalizedConfig(const char* reason);

    uint64_t GetPostStatsKey(
        Ptr<RdmaQueuePair> qp) const;

    void RecordPost(
        Ptr<RdmaQueuePair> qp,
        uint64_t bytes,
        uint64_t windowBudget,
        uint64_t admissionBudget,
        Time windowEnd);

    void FlushPostSummary(
        Ptr<RdmaQueuePair> qp);

    struct UserspaceQpState
    {
        uint64_t postedBytes;
        uint64_t completedBytes;
        uint64_t outstandingBytes;
        uint64_t lastAggregateBlockWindowEndNs;
        bool operationCompleteNotified;
        bool operationFailureNotified;
        uint64_t failedWrId;
        uint32_t firstErrorStatus;

        UserspaceQpState()
            : postedBytes(0),
              completedBytes(0),
              outstandingBytes(0),
              lastAggregateBlockWindowEndNs(0),
              operationCompleteNotified(false),
              operationFailureNotified(false),
              failedWrId(0),
              firstErrorStatus(RDMA_WR_CQE_SUCCESS)
        {
        }
    };

    UserspaceQpState& GetUserspaceQpState(Ptr<RdmaQueuePair> qp);
    uint64_t GetUserspaceOutstandingBytes(Ptr<RdmaQueuePair> qp) const;

private:
    Ptr<Node> m_node;
    Ptr<RdmaHw> m_rdma;

    GateMode m_mode;
    bool m_enabled; // Mode-2 userspace transport active only in MODE_USERSPACE
    bool m_gateEnabled;
    bool m_rnicAckFlushEnabled;
    uint64_t m_rnicAckFlushGuardNs;
    std::map<uint64_t, EventId> m_rnicAckFlushEvents;
    UserspaceAdmissionMode m_userspaceAdmissionMode;

    struct GateTable
    {
        uint64_t epochStartNs;
        uint64_t periodNs;
        std::vector<GateSlotEntry> slots;
    };

    std::map<uint32_t, GateTable> m_gateTables;

    uint64_t m_wrChunkBytes;
    uint64_t m_maxOutstandingBytes;

    std::map<uint64_t, EventId> m_wakeEvents;

    uint64_t m_safeRateBps;
    uint64_t m_tailGuardNs;
    uint64_t m_minPostBytes;

    // Shared userspace execution margin. Link-rate and CQE timing state are
    // maintained per breakout RNIC port in m_userspacePortStates.
    uint64_t m_userspaceSoftwareGuardNs;

    // Deterministic adaptive admission state.
    // These are runtime parameters for mode-2 userspace WR admission.
    uint64_t m_minSafeRateBps;
    uint64_t m_maxSafeRateBps;
    uint64_t m_minTailGuardNs;
    uint64_t m_maxTailGuardNs;
    uint64_t m_minOutstandingBytes;
    uint64_t m_maxOutstandingCeilingBytes;
    uint64_t m_adaptPeriodNs;

    // Bandwidth-normalized initialization state.  These fields avoid
    // 50G-specific constants and allow the same code to scale to 100G/200G/400G.
    bool m_autoBandwidthConfig;
    bool m_realDeploymentMode;
    uint64_t m_bottleneckRateBps;
    uint64_t m_switchingGuardNs;
    uint64_t m_userspaceInitialRttNs;
    uint64_t m_userspaceMinTailGuardOverrideNs;
    uint32_t m_activeQpHint;

    struct PostStats
    {
        uint64_t postCount;
        uint64_t totalBytes;
        uint64_t minBytes;
        uint64_t maxBytes;
        uint64_t firstPostTimeNs;
        uint64_t lastPostTimeNs;
        uint64_t safeBudgetLimitedCount;
        uint64_t completionCount;
        uint64_t completedBytes;
        uint64_t totalCompletionLatencyNs;
        uint64_t minCompletionLatencyNs;
        uint64_t maxCompletionLatencyNs;
        uint64_t lastCompletedWrId;
        uint64_t errorCompletionCount;
        uint64_t retryExceededCount;
        uint64_t wrFlushErrorCount;
        uint64_t failedWrId;
        uint32_t firstErrorStatus;
        uint32_t sampleCount;

        PostStats()
            : postCount(0),
              totalBytes(0),
              minBytes(0),
              maxBytes(0),
              firstPostTimeNs(0),
              lastPostTimeNs(0),
              safeBudgetLimitedCount(0),
              completionCount(0),
              completedBytes(0),
              totalCompletionLatencyNs(0),
              minCompletionLatencyNs(0),
              maxCompletionLatencyNs(0),
              lastCompletedWrId(0),
              errorCompletionCount(0),
              retryExceededCount(0),
              wrFlushErrorCount(0),
              failedWrId(0),
              firstErrorStatus(RDMA_WR_CQE_SUCCESS),
              sampleCount(0)
        {
        }
    };

    std::map<uint64_t, PostStats> m_postStats;
    std::map<uint64_t, UserspaceQpState> m_userspaceQpStates;
    std::map<uint64_t, Ptr<RdmaQueuePair> > m_userspaceRegisteredQps;
    std::map<uint32_t, UserspacePortState> m_userspacePortStates;
    uint32_t m_postLogSampleLimit;
};

} // namespace ns3

#endif