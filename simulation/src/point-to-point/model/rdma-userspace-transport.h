#ifndef RDMA_USERSPACE_TRANSPORT_H
#define RDMA_USERSPACE_TRANSPORT_H

#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/event-id.h"
#include "ns3/rdma-hw.h"
#include "ns3/rdma-queue-pair.h"

#include <map>
#include <vector>
#include <stdint.h>

namespace ns3 {

class RdmaUserspaceTransport : public Object
{
public:
    static TypeId GetTypeId(void);

    RdmaUserspaceTransport();

    void SetNode(Ptr<Node> node);
    void SetRdmaHw(Ptr<RdmaHw> rdma);

    void SetEnabled(bool enabled);

    void Configure(
        uint64_t wrChunkBytes,
        uint64_t maxOutstandingBytes);

    // Bandwidth-normalized initialization for mode-2 userspace admission.
    // If bottleneckRateBps/switchingGuardNs/maxRttNs are zero, the transport
    // infers conservative defaults from the local RNIC and installed gate table.
    void ConfigureBandwidthNormalized(
        uint64_t bottleneckRateBps,
        uint64_t switchingGuardNs,
        uint64_t maxRttNs,
        uint32_t activeQpHint,
        bool realDeploymentMode);

    void EnableInjectionGate(
        uint32_t rnicId,
        uint64_t epochStartNs,
        uint64_t periodNs,
        const std::vector<RdmaHw::RnicGateSlotEntry>& slots);

    void DisableInjectionGate();

    void RegisterQp(Ptr<RdmaQueuePair> qp);
    void NotifyAckProgress(Ptr<RdmaQueuePair> qp);
    void NotifyRecover(Ptr<RdmaQueuePair> qp);

private:
    bool Allows(
        Ptr<RdmaQueuePair> qp,
        Time now) const;

    Time GetNextAllowedTime(
        Ptr<RdmaQueuePair> qp,
        Time now) const;

    void TrySubmit(Ptr<RdmaQueuePair> qp);

    void ScheduleNextWake(
        Ptr<RdmaQueuePair> qp,
        Time wakeTime);

    Time GetCurrentWindowEndTime(
        Ptr<RdmaQueuePair> qp,
        Time now) const;

    uint64_t GetSafeBudgetBytes(
        Ptr<RdmaQueuePair> qp,
        Time now) const;

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
        uint64_t safeBudget,
        Time windowEnd);

    void FlushPostSummary(
        Ptr<RdmaQueuePair> qp);

private:
    Ptr<Node> m_node;
    Ptr<RdmaHw> m_rdma;

    bool m_enabled;
    bool m_gateEnabled;

    uint32_t m_rnicId;
    uint64_t m_epochStartNs;
    uint64_t m_periodNs;

    uint64_t m_wrChunkBytes;
    uint64_t m_maxOutstandingBytes;

    std::vector<RdmaHw::RnicGateSlotEntry> m_slots;
    std::map<uint64_t, EventId> m_wakeEvents;

    uint64_t m_safeRateBps;
    uint64_t m_tailGuardNs;
    uint64_t m_minPostBytes;

    // Deterministic adaptive admission state.
    // These are runtime parameters for mode-2 userspace WR admission.
    uint64_t m_minSafeRateBps;
    uint64_t m_maxSafeRateBps;
    uint64_t m_minTailGuardNs;
    uint64_t m_maxTailGuardNs;
    uint64_t m_minOutstandingBytes;
    uint64_t m_maxOutstandingCeilingBytes;
    uint64_t m_adaptPeriodNs;
    uint64_t m_lastAdaptNs;
    uint32_t m_stableAdaptPeriods;
    bool m_recoverySinceLastAdapt;
    bool m_ackProgressSinceLastAdapt;
    bool m_backlogSinceLastAdapt;

    // Bandwidth-normalized initialization state.  These fields avoid
    // 50G-specific constants and allow the same code to scale to 100G/200G/400G.
    bool m_autoBandwidthConfig;
    bool m_realDeploymentMode;
    uint64_t m_bottleneckRateBps;
    uint64_t m_switchingGuardNs;
    uint64_t m_maxObservedRttNs;
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
        uint32_t sampleCount;

        PostStats()
            : postCount(0),
              totalBytes(0),
              minBytes(0),
              maxBytes(0),
              firstPostTimeNs(0),
              lastPostTimeNs(0),
              safeBudgetLimitedCount(0),
              sampleCount(0)
        {
        }
    };

    std::map<uint64_t, PostStats> m_postStats;
    uint32_t m_postLogSampleLimit;
};

} // namespace ns3

#endif