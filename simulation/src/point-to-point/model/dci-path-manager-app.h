// SPDX-License-Identifier: GPL-2.0-only
#ifndef DCI_PATH_MANAGER_APP_H
#define DCI_PATH_MANAGER_APP_H
#include "ns3/application.h"
#include "ns3/callback.h"
#include <tuple>
#include "ns3/event-id.h"
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/random-variable-stream.h"
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace ns3 {
class QbbNetDevice;
class Packet;

// Private control UDP payload, never an extension of a business/RoCE packet.
class DciPathControlHeader : public Header {
public:
  enum Kind : uint8_t { PROBE = 1, REPLY = 2 };
  static constexpr uint32_t SIZE = 40;
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator i) const override;
  uint32_t Deserialize(Buffer::Iterator i) override;
  void Print(std::ostream& os) const override;
  bool IsValid() const { return m_valid; }
  Kind kind = PROBE;
  uint32_t pathId = 0;
  uint64_t session = 0, sequence = 0, sentNs = 0;
private:
  bool m_valid = true;
};

// V1: startup route snapshot, direct parallel WAN links, no business scheduler.
// IDs are local. Replies echo the initiator's ID; peer IDs need not match.
class DciPathManagerApp : public Application {
public:
  enum Status { UNVERIFIED, UP, DOWN, STOPPED };
  struct Settings {
    Time probeInterval = MilliSeconds(1);
    Time timeout = MilliSeconds(3);
    Time processingDelay = MicroSeconds(5); // fixed per-message delay, not a CPU model
    bool logPackets = false;
    bool probeEnabled = true; // Diagnostic passive mode: install service without originating probes.
  };
  struct Snapshot {
    uint32_t pathId = 0, interface = 0;
    Ipv4Address localAddress, peerAddress;
    Status status = UNVERIFIED;
    bool hasRtt = false;
    Time latestRtt, lastReply;
    uint64_t samples = 0, txPackets = 0, rxPackets = 0;
    uint64_t txBytes = 0, rxBytes = 0, timeouts = 0, enqueueDrops = 0;
    uint64_t staleReplies = 0;
    // Local port statistics, independent of probe counters.
    bool hasLocalEgressStats = false;
    uint64_t localSampleTimeNs = 0, linkRateBps = 0;
    uint64_t egressQueueBytes = 0, egressTxCompletedBytes = 0;
    // DCI_BUSINESS_SNAPSHOT_V1: historical feedback, separate from live port stats.
    uint64_t snapshotTimeNs = 0;
    bool hasBusinessFeedback = false;
    uint64_t businessFeedbackSamples = 0, businessCeSamples = 0;
    uint64_t businessLocalQueueNs = 0, businessDataFeedbackNs = 0;
    uint64_t businessSampleTxNs = 0, businessFeedbackReceivedNs = 0;
    bool IsUsable() const { return status == UP; }
  };
  static constexpr uint32_t MAX_PATHS = 4096, MAX_PENDING_WORK = 4096;
  static TypeId GetTypeId();
  static const char* StatusName(Status status);
  void Configure(const Settings& settings);
  // Read all legal candidates from SwitchNode::GetRoutingTable(), before ECMP.
  // Optional allow-list restricts candidates to interfaces designated as WAN.
  uint32_t AddPeer(Ipv4Address local, Ipv4Address peer,
                   const std::set<uint32_t>& wanInterfaces = {});
  std::vector<Snapshot> GetSnapshot() const;
  uint32_t GetOutstanding() const;
  uint32_t GetPeakOutstanding() const { return m_peakOutstanding; }
  void PrintStats() const;
  void AssignStream(int64_t stream);
  void StartService();
  void StopService();
protected:
  void StartApplication() override;
  void StopApplication() override;
  void DoDispose() override;
private:
  // DCI_BUSINESS_SAMPLING_V1: local records; no fields are added to DATA.
  struct BusinessKey {
    uint32_t sip = 0, dip = 0, frameBytes = 0;
    uint16_t sport = 0, dport = 0, pg = 0, ipid = 0;
    uint64_t seq = 0;
    bool operator<(const BusinessKey& b) const {
      return std::tie(sip, dip, sport, dport, pg, ipid, seq, frameBytes) <
             std::tie(b.sip, b.dip, b.sport, b.dport, b.pg, b.ipid, b.seq, b.frameBytes);
    }
  };
  struct BusinessRecord {
    uint64_t enqueuedNs = 0, txNs = 0, generation = 0;
    bool transmitted = false, ambiguous = false, retired = false;
    EventId deadline;
  };
  struct BusinessStats {
    uint64_t candidates = 0, started = 0, completed = 0, ceSamples = 0;
    uint64_t timeouts = 0, noTx = 0, txFailures = 0, capped = 0, ambiguous = 0;
    uint64_t reportsTx = 0, reportsRx = 0, reportBytes = 0;
    uint64_t reportsSuppressed = 0, reportDrops = 0, unmatched = 0;
    uint64_t nextReportNs = 0, localQueueNs = 0, feedbackNs = 0;
    uint64_t lastSampleNs = 0, lastFeedbackNs = 0;
    uint32_t pending = 0, peakPending = 0;
  };
  static constexpr uint32_t BUSINESS_MAX_PENDING = 1024;
  static constexpr uint32_t BUSINESS_MAX_PER_PATH = 64;
  using BusinessTraceCallback = Callback<void, Ptr<const Packet>>;
  using BusinessRecordKey = std::pair<uint32_t, BusinessKey>;
  void StartBusinessSampling();
  void StopBusinessSampling();
  static void BusinessTrace(DciPathManagerApp* app, uint32_t path,
                            uint8_t stage, Ptr<const Packet> packet);
  bool GetBusinessKey(Ptr<const Packet> packet, BusinessKey& key,
                       uint8_t& ecn) const;
  bool IsBusinessSample(const BusinessKey& key) const;
  void ObserveBusiness(uint32_t path, uint8_t stage, Ptr<const Packet> packet);
  void ExpireBusiness(uint32_t path, BusinessKey key, uint64_t generation);
  void ReportBusiness(uint64_t workId, uint32_t path, BusinessKey key, uint8_t ecn);
  bool ReceiveBusiness(uint32_t path, Ptr<Packet> payload, uint32_t frameBytes);
  void PrintBusinessStats() const;
  bool m_businessEnabled = false;
  uint32_t m_businessEvery = 1024;
  uint64_t m_businessReportGapNs = 100000, m_businessGeneration = 0;
  uint64_t m_businessReportSequence = 0;
  std::map<BusinessRecordKey, BusinessRecord> m_businessRecords;
  std::map<uint32_t, BusinessStats> m_businessStats;
  std::map<std::pair<uint32_t, uint8_t>, BusinessTraceCallback> m_businessHooks;

  struct Path {
    Snapshot view;
    Ptr<QbbNetDevice> device;
    bool pending = false;
    DciPathControlHeader request;
    EventId deadline;
    uint64_t nextSequence = 0;
  };
  void Tick();
  void Request(uint32_t id);
  void Expire(uint32_t id, uint64_t session, uint64_t sequence);
  bool Receive(Ptr<Packet> packet, uint32_t interface);
  void Process(uint64_t workId, uint32_t id, DciPathControlHeader header);
  bool Send(uint32_t id, const DciPathControlHeader& header);
  void Change(Path& path, Status next);
  Settings m_settings;
  std::map<uint32_t, Path> m_paths;
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> m_lookup;
  std::map<uint32_t, Ptr<QbbNetDevice>> m_devices;
  std::map<uint64_t, EventId> m_work;
  EventId m_tick;
  Ptr<UniformRandomVariable> m_random;
  uint64_t m_session = 0, m_workId = 0, m_workDrops = 0;
  uint32_t m_outstanding = 0, m_peakOutstanding = 0, m_peakWork = 0;
  bool m_running = false;
};
} // namespace ns3
#endif
