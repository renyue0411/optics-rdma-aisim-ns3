// SPDX-License-Identifier: GPL-2.0-only
#include "dci-path-manager-app.h"
#include "qbb-net-device.h"
#include "switch-node.h"
#include "ns3/abort.h"
#include "ns3/custom-header.h"
#include "ns3/ipv4-header.h"
#include "ns3/integer.h"
#include "ns3/object-factory.h"
#include "ns3/ppp-header.h"
#include "ns3/simulator.h"
#include "ns3/udp-header.h"
#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace ns3 {
NS_OBJECT_ENSURE_REGISTERED(DciPathControlHeader);
NS_OBJECT_ENSURE_REGISTERED(DciPathManagerApp);
TypeId DciPathControlHeader::GetTypeId() {
  static TypeId tid = TypeId("ns3::DciPathControlHeader").SetParent<Header>()
    .AddConstructor<DciPathControlHeader>();
  return tid;
}
TypeId DciPathControlHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t DciPathControlHeader::GetSerializedSize() const { return SIZE; }
void DciPathControlHeader::Serialize(Buffer::Iterator i) const {
  i.WriteHtonU32(0x4443504d); // DCPM
  i.WriteU8(1); i.WriteU8(kind); i.WriteHtonU16(0);
  i.WriteHtonU32(pathId); i.WriteHtonU32(0);
  i.WriteHtonU64(session); i.WriteHtonU64(sequence); i.WriteHtonU64(sentNs);
}
uint32_t DciPathControlHeader::Deserialize(Buffer::Iterator i) {
  const uint32_t magic = i.ReadNtohU32();
  const uint8_t version = i.ReadU8(), type = i.ReadU8();
  const uint16_t reserved = i.ReadNtohU16();
  pathId = i.ReadNtohU32();
  const uint32_t reserved2 = i.ReadNtohU32();
  session = i.ReadNtohU64(); sequence = i.ReadNtohU64(); sentNs = i.ReadNtohU64();
  kind = static_cast<Kind>(type);
  m_valid = magic == 0x4443504d && version == 1 && !reserved && !reserved2 &&
    (type == PROBE || type == REPLY) && session != 0 && sequence != 0;
  return SIZE;
}
void DciPathControlHeader::Print(std::ostream& os) const {
  os << "kind=" << unsigned(kind) << " path=" << pathId << " seq=" << sequence;
}
TypeId DciPathManagerApp::GetTypeId() {
  static TypeId tid = TypeId("ns3::DciPathManagerApp").SetParent<Application>()
    .AddConstructor<DciPathManagerApp>();
  return tid;
}
const char* DciPathManagerApp::StatusName(Status status) {
  static const char* names[] = {"UNVERIFIED", "UP", "DOWN", "STOPPED"};
  return names[status];
}
void DciPathManagerApp::Configure(const Settings& settings) {
  NS_ABORT_MSG_IF(m_running || settings.probeInterval <= Seconds(0) ||
    settings.timeout <= Seconds(0) || settings.processingDelay < Seconds(0),
    "Invalid DCI path service settings");
  m_settings = settings;
}
void DciPathManagerApp::AssignStream(int64_t stream) {
  NS_ABORT_MSG_IF(m_running, "Assign DCI RNG before service start");
  NS_ABORT_MSG_IF(stream < 0, "DCI session RNG requires an explicit stream");
  if (!m_random) {
    // Set the attribute at construction, before a default Stream=-1 can consume
    // an automatic index shared with the legacy ECN random-variable code.
    ObjectFactory factory;
    factory.SetTypeId(UniformRandomVariable::GetTypeId());
    factory.Set("Stream", IntegerValue(stream));
    m_random = factory.Create<UniformRandomVariable>();
  } else {
    m_random->SetStream(stream);
  }
}
uint32_t DciPathManagerApp::AddPeer(Ipv4Address local, Ipv4Address peer,
    const std::set<uint32_t>& wanInterfaces) {
  auto node = DynamicCast<SwitchNode>(GetNode());
  NS_ABORT_MSG_IF(m_running || !node || local == Ipv4Address::GetAny() ||
    peer == Ipv4Address::GetAny() || local == peer, "Invalid DCI peer");
  const auto& routes = node->GetRoutingTable();
  const auto route = routes.find(peer.Get());
  if (route == routes.end()) return 0;
  std::set<int> candidates(route->second.begin(), route->second.end());
  uint32_t added = 0;
  for (int port : candidates) {
    if (port < 0 || (!wanInterfaces.empty() && !wanInterfaces.count(port))) continue;
    NS_ABORT_MSG_IF(uint32_t(port) >= node->GetNDevices(), "Invalid port in DCI route");
    auto device = DynamicCast<QbbNetDevice>(node->GetDevice(port));
    if (!device) continue;
    const auto key = std::make_pair(uint32_t(port), peer.Get());
    if (m_lookup.count(key)) continue;
    NS_ABORT_MSG_IF(m_paths.size() >= MAX_PATHS, "Too many DCI candidates");
    const uint32_t id = m_paths.size();
    Path path;
    path.view.pathId = id; path.view.interface = port;
    path.view.localAddress = local; path.view.peerAddress = peer;
    path.device = device;
    m_paths.emplace(id, path); m_lookup.emplace(key, id); ++added;
    std::cout << "DCI_PM_CANDIDATE node=" << node->GetId() << " peer=" << peer
      << " path=" << id << " interface=" << port << " source=route_snapshot\n";
  }
  return added;
}
void DciPathManagerApp::StartApplication() { StartService(); }
void DciPathManagerApp::StopApplication() { StopService(); }
void DciPathManagerApp::DoDispose() {
  StopService(); m_paths.clear(); m_lookup.clear();
  Application::DoDispose();
}
void DciPathManagerApp::StartService() {
  NS_ABORT_MSG_IF(m_running || !DynamicCast<SwitchNode>(GetNode()), "Invalid DCI service start");
  if (!m_random) AssignStream(1900000 + GetNode()->GetId());
  do {
    m_session = (uint64_t(m_random->GetInteger(0, 0xffffffffU)) << 32) |
      m_random->GetInteger(0, 0xffffffffU);
  } while (!m_session);
  for (auto& item : m_paths) {
    auto& p = item.second;
    if (!m_devices.count(p.view.interface)) {
      NS_ABORT_MSG_IF(p.device->HasDciControlReceiver(), "DCI interface already has a service");
      p.device->SetDciControlReceiver(MakeCallback(&DciPathManagerApp::Receive, this));
      m_devices.emplace(p.view.interface, p.device);
    }
    p.pending = false; p.nextSequence = 0;
    p.view.hasRtt = false; p.view.latestRtt = Seconds(0);
    p.view.lastReply = Seconds(0); p.view.status = UNVERIFIED;
    std::cout << "DCI_PM_STATE node=" << GetNode()->GetId() << " path=" << item.first
      << " time_ns=" << Simulator::Now().GetNanoSeconds() << " to=UNVERIFIED\n";
  }
  m_running = true;
  StartBusinessSampling();
  if (m_settings.probeEnabled) Tick();
}
void DciPathManagerApp::StopService() {
  if (!m_running) return;
  m_running = false;
  StopBusinessSampling();
  Simulator::Cancel(m_tick);
  for (const auto& work : m_work) Simulator::Cancel(work.second);
  m_work.clear();
  for (auto& item : m_paths) {
    Simulator::Cancel(item.second.deadline); item.second.pending = false;
    Change(item.second, STOPPED);
  }
  for (auto& d : m_devices)
    d.second->SetDciControlReceiver(Callback<bool, Ptr<Packet>, uint32_t>());
  m_devices.clear();
  m_outstanding = 0;
}
void DciPathManagerApp::Change(Path& path, Status next) {
  if (path.view.status == next) return;
  std::cout << "DCI_PM_STATE node=" << GetNode()->GetId() << " path=" << path.view.pathId
    << " time_ns=" << Simulator::Now().GetNanoSeconds()
    << " from=" << StatusName(path.view.status) << " to=" << StatusName(next) << '\n';
  path.view.status = next;
}
void DciPathManagerApp::Tick() {
  if (!m_running) return;
  for (auto& item : m_paths) if (!item.second.pending) Request(item.first);
  m_tick = Simulator::Schedule(m_settings.probeInterval, &DciPathManagerApp::Tick, this);
}
void DciPathManagerApp::Request(uint32_t id) {
  auto& p = m_paths.at(id);
  NS_ABORT_MSG_IF(p.nextSequence == std::numeric_limits<uint64_t>::max(), "Probe sequence exhausted");
  DciPathControlHeader h;
  h.pathId = id; h.session = m_session; h.sequence = ++p.nextSequence;
  h.sentNs = Simulator::Now().GetNanoSeconds();
  p.request = h; p.pending = true;
  m_peakOutstanding = std::max(m_peakOutstanding, ++m_outstanding);
  p.deadline = Simulator::Schedule(m_settings.timeout,
    &DciPathManagerApp::Expire, this, id, h.session, h.sequence);
  Send(id, h);
}
bool DciPathManagerApp::Send(uint32_t id, const DciPathControlHeader& h) {
  if (!m_running) return false;
  auto& p = m_paths.at(id);
  auto packet = Create<Packet>(); packet->AddHeader(h);
  UdpHeader udp;
  udp.SetSourcePort(CustomHeader::DCI_PATH_CONTROL_PORT);
  udp.SetDestinationPort(CustomHeader::DCI_PATH_CONTROL_PORT);
  packet->AddHeader(udp);
  Ipv4Header ip;
  ip.SetSource(p.view.localAddress); ip.SetDestination(p.view.peerAddress);
  ip.SetProtocol(17); ip.SetPayloadSize(packet->GetSize()); ip.SetTtl(64);
  packet->AddHeader(ip);
  PppHeader ppp; ppp.SetProtocol(0x0021); packet->AddHeader(ppp);
  CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
  packet->PeekHeader(ch);
  const auto bytes = packet->GetSize();
  // Real q0 enqueue and real Qbb serialization. No synthetic arrival event.
  if (!p.device->IsLinkUp() || !p.device->SwitchSend(0, packet, ch)) {
    ++p.view.enqueueDrops; return false;
  }
  ++p.view.txPackets; p.view.txBytes += bytes;
  if (m_settings.logPackets)
    std::cout << "DCI_PM_TX node=" << GetNode()->GetId() << " path=" << id
      << " kind=" << unsigned(h.kind) << " seq=" << h.sequence << " bytes=" << bytes
      << " time_ns=" << Simulator::Now().GetNanoSeconds() << '\n';
  return true;
}
bool DciPathManagerApp::Receive(Ptr<Packet> original, uint32_t interface) {
  if (!m_running || original->GetSize() < PppHeader::GetStaticSize() + 20 + 8 + DciPathControlHeader::SIZE)
    return true;
  auto packet = original->Copy();
  PppHeader ppp; Ipv4Header ip; UdpHeader udp;
  packet->RemoveHeader(ppp); packet->RemoveHeader(ip); packet->RemoveHeader(udp);
  const auto entry = m_lookup.find(std::make_pair(interface, ip.GetSource().Get()));
  if (entry == m_lookup.end()) return true;
  auto& p = m_paths.at(entry->second);
  if (ip.GetDestination() != p.view.localAddress) return true;
  if (ReceiveBusiness(entry->second, packet, original->GetSize())) return true;
  if (packet->GetSize() != DciPathControlHeader::SIZE) return true;
  DciPathControlHeader h; packet->RemoveHeader(h);
  if (!h.IsValid()) return true;
  ++p.view.rxPackets; p.view.rxBytes += original->GetSize();
  if (m_work.size() >= MAX_PENDING_WORK) { ++m_workDrops; return true; }
  NS_ABORT_MSG_IF(m_workId == std::numeric_limits<uint64_t>::max(), "Work ID exhausted");
  const auto workId = ++m_workId;
  m_work.emplace(workId, Simulator::Schedule(m_settings.processingDelay,
    &DciPathManagerApp::Process, this, workId, entry->second, h));
  m_peakWork = std::max(m_peakWork, uint32_t(m_work.size()));
  return true;
}
void DciPathManagerApp::Process(uint64_t workId, uint32_t id, DciPathControlHeader h) {
  m_work.erase(workId);
  if (!m_running) return;
  auto& p = m_paths.at(id);
  if (h.kind == DciPathControlHeader::PROBE) {
    h.kind = DciPathControlHeader::REPLY;
    Send(id, h); // echo initiator ID/session/sequence over the receiving interface
    return;
  }
  if (!p.pending || h.session != m_session || h.pathId != id ||
    h.sequence != p.request.sequence || h.sentNs != p.request.sentNs ||
    Simulator::Now() - NanoSeconds(h.sentNs) >= m_settings.timeout) {
    ++p.view.staleReplies; return;
  }
  Simulator::Cancel(p.deadline); p.pending = false; --m_outstanding;
  p.view.latestRtt = Simulator::Now() - NanoSeconds(h.sentNs);
  p.view.hasRtt = true; ++p.view.samples;
  p.view.lastReply = Simulator::Now(); Change(p, UP);
}
void DciPathManagerApp::Expire(uint32_t id, uint64_t session, uint64_t sequence) {
  if (!m_running) return;
  auto& p = m_paths.at(id);
  if (!p.pending || p.request.session != session || p.request.sequence != sequence) return;
  p.pending = false; --m_outstanding; ++p.view.timeouts; Change(p, DOWN);
}
std::vector<DciPathManagerApp::Snapshot> DciPathManagerApp::GetSnapshot() const {
  std::vector<Snapshot> result;
  for (const auto& item : m_paths) {
    const auto& path = item.second;
    Snapshot view = path.view;
    view.snapshotTimeNs = Simulator::Now().GetNanoSeconds();
    const auto business = m_businessStats.find(item.first);
    if (business != m_businessStats.end() && business->second.completed > 0) {
      const auto& stats = business->second;
      view.hasBusinessFeedback = true;
      view.businessFeedbackSamples = stats.completed;
      view.businessCeSamples = stats.ceSamples;
      view.businessLocalQueueNs = stats.localQueueNs;
      view.businessDataFeedbackNs = stats.feedbackNs;
      view.businessSampleTxNs = stats.lastSampleNs;
      view.businessFeedbackReceivedNs = stats.lastFeedbackNs;
    }
    if (path.device) {
      auto queue = path.device->GetQueue();
      if (queue) {
        view.hasLocalEgressStats = true;
        view.localSampleTimeNs = Simulator::Now().GetNanoSeconds();
        view.linkRateBps = path.device->GetDataRate().GetBitRate();
        view.egressQueueBytes = queue->GetNBytesTotal();
        view.egressTxCompletedBytes = path.device->GetLocalTxCompletedBytes();
      }
    }
    result.push_back(view);
  }
  return result;
}
uint32_t DciPathManagerApp::GetOutstanding() const {
  return m_outstanding;
}
void DciPathManagerApp::PrintStats() const {
  PrintBusinessStats();
  for (const auto& s : GetSnapshot())
    std::cout << "DCI_PM_STATS node=" << GetNode()->GetId() << " peer=" << s.peerAddress
      << " path=" << s.pathId << " interface=" << s.interface
      << " state=" << StatusName(s.status) << " has_rtt=" << s.hasRtt
      << " control_rtt_ns=" << s.latestRtt.GetNanoSeconds() << " samples=" << s.samples
      << " tx_packets=" << s.txPackets << " rx_packets=" << s.rxPackets
      << " tx_bytes=" << s.txBytes << " rx_bytes=" << s.rxBytes
      << " timeouts=" << s.timeouts << " enqueue_drops=" << s.enqueueDrops
      << " stale_replies=" << s.staleReplies
      << " snapshot_time_ns=" << s.snapshotTimeNs
      << " business_feedback_valid=" << s.hasBusinessFeedback
      << " business_feedback_samples=" << s.businessFeedbackSamples
      << " business_ce_samples=" << s.businessCeSamples
      << " business_local_queue_ns=" << s.businessLocalQueueNs
      << " business_data_feedback_ns=" << s.businessDataFeedbackNs
      << " business_sample_tx_ns=" << s.businessSampleTxNs
      << " business_feedback_received_ns=" << s.businessFeedbackReceivedNs
      << " local_stats_valid=" << s.hasLocalEgressStats
      << " local_sample_time_ns=" << s.localSampleTimeNs
      << " link_rate_bps=" << s.linkRateBps
      << " egress_queue_bytes=" << s.egressQueueBytes
      << " egress_tx_completed_bytes=" << s.egressTxCompletedBytes << '\n';
  std::cout << "DCI_PM_RESOURCE node=" << GetNode()->GetId() << " paths=" << m_paths.size()
    << " outstanding=" << GetOutstanding() << " peak_outstanding=" << m_peakOutstanding
    << " pending_work=" << m_work.size() << " peak_work=" << m_peakWork
    << " work_drops=" << m_workDrops << " snapshot_bytes=" << m_paths.size() * sizeof(Snapshot)
    << " memory_scope=raw_snapshot_only\n";
}
// DCI_BUSINESS_SAMPLING_V1. Data is observed through existing device traces.
// No Packet UID, PacketTag, remote object, or shared sender/receiver state is used.
namespace {
const char* BusinessTraceName(uint8_t stage) {
  static const char* names[] = {"MacTx", "PhyTxBegin", "MacRx", "PhyTxDrop"};
  return names[stage];
}
uint32_t BusinessOption(const char* name, uint32_t fallback, uint32_t maximum) {
  const char* raw = std::getenv(name);
  if (!raw) return fallback;
  uint64_t value = 0;
  NS_ABORT_MSG_IF(!*raw, "Empty business sampling option: " << name);
  for (const char* c = raw; *c; ++c) {
    NS_ABORT_MSG_IF(*c < '0' || *c > '9', "Invalid business sampling option: " << name);
    value = value * 10 + (*c - '0');
    NS_ABORT_MSG_IF(value > maximum, "Business sampling option too large: " << name);
  }
  return static_cast<uint32_t>(value);
}
void BusinessPut(uint8_t*& p, uint64_t value, unsigned bytes) {
  for (unsigned i = bytes; i != 0; --i) *p++ = uint8_t(value >> ((i - 1) * 8));
}
uint64_t BusinessGet(const uint8_t*& p, unsigned bytes) {
  uint64_t value = 0;
  for (unsigned i = 0; i < bytes; ++i) value = (value << 8) | *p++;
  return value;
}
constexpr uint32_t BUSINESS_REPORT_SIZE = 56;
}
void DciPathManagerApp::StartBusinessSampling() {
  m_businessEnabled = BusinessOption("SIMAI_PM_BUSINESS_SAMPLE", 0, 1) != 0;
  if (!m_businessEnabled) return;
  m_businessStats.clear();
  m_businessEvery = BusinessOption("SIMAI_PM_BUSINESS_EVERY", 1024, 1048576);
  m_businessReportGapNs = uint64_t(BusinessOption(
      "SIMAI_PM_BUSINESS_REPORT_US", 100, 1000000)) * 1000;
  NS_ABORT_MSG_IF(!m_businessEvery || !m_businessReportGapNs,
                  "Business sampling interval and report gap must be positive");
  // V1 attribution is explicitly restricted to direct links, one peer per port.
  std::set<uint32_t> ports;
  for (const auto& item : m_paths) {
    const auto& path = item.second;
    NS_ABORT_MSG_IF(!ports.insert(path.view.interface).second,
                    "Business sampling V1 requires one direct peer/path per interface");
    m_businessStats.emplace(item.first, BusinessStats());
    for (uint8_t stage = 0; stage != 4; ++stage) {
      auto cb = MakeBoundCallback(&DciPathManagerApp::BusinessTrace,
                                   this, item.first, stage);
      NS_ABORT_MSG_IF(!path.device->TraceConnectWithoutContext(BusinessTraceName(stage), cb),
                      "Missing DCI business trace: " << BusinessTraceName(stage));
      m_businessHooks.emplace(std::make_pair(item.first, stage), cb);
    }
  }
  std::cout << "DCI_PM_BUSINESS_CONFIG node=" << GetNode()->GetId()
    << " enabled=1 feedback_enable="
    << BusinessOption("SIMAI_PM_BUSINESS_FEEDBACK", 1, 1)
    << " sample_every=" << m_businessEvery
    << " report_gap_ns=" << m_businessReportGapNs
    << " max_pending=" << BUSINESS_MAX_PENDING
    << " max_per_path=" << BUSINESS_MAX_PER_PATH
    << " attribution=direct_interface return_path=same_link control_queue=0"
    << " timing=local_queue_plus_data_feedback no_clock_sync_required=1\n";
}
void DciPathManagerApp::StopBusinessSampling() {
  for (const auto& h : m_businessHooks) {
    const auto found = m_paths.find(h.first.first);
    if (found != m_paths.end())
      found->second.device->TraceDisconnectWithoutContext(BusinessTraceName(h.first.second), h.second);
  }
  m_businessHooks.clear();
  for (auto& r : m_businessRecords) Simulator::Cancel(r.second.deadline);
  m_businessRecords.clear();
  for (auto& s : m_businessStats) s.second.pending = 0;
  m_businessEnabled = false;
}
void DciPathManagerApp::BusinessTrace(DciPathManagerApp* app, uint32_t path,
                                     uint8_t stage, Ptr<const Packet> packet) {
  app->ObserveBusiness(path, stage, packet);
}
bool DciPathManagerApp::GetBusinessKey(Ptr<const Packet> packet,
                                       BusinessKey& key, uint8_t& ecn) const {
  if (packet->GetSize() <= CustomHeader::GetStaticWholeHeaderSize()) return false;
  CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
  ch.getInt = 0;
  packet->PeekHeader(ch);
  // This simulator's ordinary RDMA DATA format, not arbitrary UDP/control traffic.
  if (ch.l3Prot != 0x11 || ch.IsDciPathControl() || ch.udp.dport != 100 || ch.udp.pg >= 8)
    return false;
  key.sip = ch.sip; key.dip = ch.dip;
  key.sport = ch.udp.sport; key.dport = ch.udp.dport;
  key.pg = ch.udp.pg; key.ipid = ch.ipid;
  key.seq = ch.udp.seq; key.frameBytes = packet->GetSize();
  ecn = ch.GetIpv4EcnBits();
  return true;
}
bool DciPathManagerApp::IsBusinessSample(const BusinessKey& key) const {
  // Canonical hash excludes mutable ECN/TTL/INT fields. Both gateways agree.
  const uint64_t values[] = {key.sip, key.dip, key.sport, key.dport,
                             key.pg, key.ipid, key.seq, key.frameBytes};
  uint64_t h = 14695981039346656037ULL;
  for (uint64_t value : values)
    for (unsigned i = 0; i != 8; ++i) {
      h ^= uint8_t(value >> (i * 8)); h *= 1099511628211ULL;
    }
  h ^= h >> 30; h *= 0xbf58476d1ce4e5b9ULL;
  h ^= h >> 27; h *= 0x94d049bb133111ebULL; h ^= h >> 31;
  return h % m_businessEvery == 0;
}
void DciPathManagerApp::ObserveBusiness(uint32_t path, uint8_t stage,
                                        Ptr<const Packet> packet) {
  if (!m_businessEnabled || !m_running) return;
  BusinessKey key;
  uint8_t ecn = 0;
  if (!GetBusinessKey(packet, key, ecn) || !IsBusinessSample(key)) return;
  auto& stats = m_businessStats.at(path);
  const uint64_t now = Simulator::Now().GetNanoSeconds();
  const BusinessRecordKey recordKey(path, key);
  if (stage == 0) { // MacTx precedes the actual enqueue attempt.
    ++stats.candidates;
    auto existing = m_businessRecords.find(recordKey);
    if (existing != m_businessRecords.end()) {
      if (!existing->second.retired) existing->second.ambiguous = true;
      ++stats.ambiguous; return;
    }
    if (m_businessRecords.size() >= BUSINESS_MAX_PENDING || stats.pending >= BUSINESS_MAX_PER_PATH) {
      ++stats.capped; return;
    }
    NS_ABORT_MSG_IF(m_businessGeneration == std::numeric_limits<uint64_t>::max(),
                    "Business sampling generation exhausted");
    BusinessRecord record;
    record.enqueuedNs = now; record.generation = ++m_businessGeneration;
    record.deadline = Simulator::Schedule(m_settings.timeout,
      &DciPathManagerApp::ExpireBusiness, this, path, key, record.generation);
    m_businessRecords.emplace(recordKey, record);
    stats.peakPending = std::max(stats.peakPending, ++stats.pending);
  } else if (stage == 1) { // Real serialization start, not queue removal accounting.
    auto found = m_businessRecords.find(recordKey);
    if (found == m_businessRecords.end() || found->second.retired) return;
    if (found->second.transmitted) {
      found->second.ambiguous = true; ++stats.ambiguous; return;
    }
    found->second.transmitted = true; found->second.txNs = now; ++stats.started;
  } else if (stage == 2) { // RX after link/error checks, before forwarding onward.
    if (now < stats.nextReportNs || m_work.size() >= MAX_PENDING_WORK) {
      ++stats.reportsSuppressed; return;
    }
    stats.nextReportNs = now + m_businessReportGapNs;
    NS_ABORT_MSG_IF(m_workId == std::numeric_limits<uint64_t>::max(), "Business work ID exhausted");
    const auto workId = ++m_workId;
    m_work.emplace(workId, Simulator::Schedule(m_settings.processingDelay,
      &DciPathManagerApp::ReportBusiness, this, workId, path, key, ecn));
    m_peakWork = std::max(m_peakWork, uint32_t(m_work.size()));
  } else if (stage == 3) {
    auto found = m_businessRecords.find(recordKey);
    if (found == m_businessRecords.end() || found->second.retired) return;
    Simulator::Cancel(found->second.deadline);
    m_businessRecords.erase(found); --stats.pending; ++stats.txFailures;
  }
}
void DciPathManagerApp::ExpireBusiness(uint32_t path, BusinessKey key, uint64_t generation) {
  auto found = m_businessRecords.find(BusinessRecordKey(path, key));
  if (found == m_businessRecords.end() || found->second.generation != generation) return;
  if (found->second.retired) { m_businessRecords.erase(found); return; }
  auto& stats = m_businessStats.at(path);
  if (found->second.transmitted) ++stats.timeouts;
  else ++stats.noTx;
  --stats.pending; m_businessRecords.erase(found);
  // No reply is a measurement timeout, never proof of DATA loss/path failure.
}
void DciPathManagerApp::ReportBusiness(uint64_t workId, uint32_t path,
                                       BusinessKey key, uint8_t ecn) {
  m_work.erase(workId);
  if (!m_running || !m_businessEnabled) return;
  if (!BusinessOption("SIMAI_PM_BUSINESS_FEEDBACK", 1, 1)) return;
  NS_ABORT_MSG_IF(m_businessReportSequence == std::numeric_limits<uint64_t>::max(),
                  "Business report sequence exhausted");
  std::array<uint8_t, BUSINESS_REPORT_SIZE> bytes{};
  uint8_t* out = bytes.data();
  BusinessPut(out, 0x44434246, 4); // DCBF, separate from the unchanged DCPM probes.
  BusinessPut(out, 1, 1); BusinessPut(out, ecn, 1); BusinessPut(out, 0, 2);
  BusinessPut(out, key.sip, 4); BusinessPut(out, key.dip, 4);
  BusinessPut(out, key.sport, 2); BusinessPut(out, key.dport, 2);
  BusinessPut(out, key.pg, 2); BusinessPut(out, key.ipid, 2);
  BusinessPut(out, key.seq, 8); BusinessPut(out, key.frameBytes, 4);
  BusinessPut(out, 0, 4); BusinessPut(out, m_session, 8);
  BusinessPut(out, ++m_businessReportSequence, 8);
  auto& p = m_paths.at(path);
  auto& stats = m_businessStats.at(path);
  auto packet = Create<Packet>(bytes.data(), bytes.size());
  UdpHeader udp;
  udp.SetSourcePort(CustomHeader::DCI_PATH_CONTROL_PORT);
  udp.SetDestinationPort(CustomHeader::DCI_PATH_CONTROL_PORT); packet->AddHeader(udp);
  Ipv4Header ip;
  ip.SetSource(p.view.localAddress); ip.SetDestination(p.view.peerAddress);
  ip.SetProtocol(17); ip.SetPayloadSize(packet->GetSize()); ip.SetTtl(64); packet->AddHeader(ip);
  PppHeader ppp; ppp.SetProtocol(0x0021); packet->AddHeader(ppp);
  CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
  packet->PeekHeader(ch);
  if (!p.device->IsLinkUp() || !p.device->SwitchSend(0, packet, ch)) {
    ++stats.reportDrops; ++p.view.enqueueDrops; return;
  }
  ++stats.reportsTx; stats.reportBytes += packet->GetSize();
  ++p.view.txPackets; p.view.txBytes += packet->GetSize();
}
bool DciPathManagerApp::ReceiveBusiness(uint32_t path, Ptr<Packet> payload,
                                        uint32_t frameBytes) {
  if (payload->GetSize() != BUSINESS_REPORT_SIZE) return false;
  if (!m_businessEnabled) return true;
  std::array<uint8_t, BUSINESS_REPORT_SIZE> bytes{};
  payload->CopyData(bytes.data(), bytes.size());
  const uint8_t* in = bytes.data();
  const auto magic = BusinessGet(in, 4), version = BusinessGet(in, 1);
  const auto ecn = BusinessGet(in, 1), reserved = BusinessGet(in, 2);
  BusinessKey key;
  key.sip = BusinessGet(in, 4); key.dip = BusinessGet(in, 4);
  key.sport = BusinessGet(in, 2); key.dport = BusinessGet(in, 2);
  key.pg = BusinessGet(in, 2); key.ipid = BusinessGet(in, 2);
  key.seq = BusinessGet(in, 8); key.frameBytes = BusinessGet(in, 4);
  const auto reserved2 = BusinessGet(in, 4), reporter = BusinessGet(in, 8);
  const auto reportSequence = BusinessGet(in, 8);
  if (magic != 0x44434246 || version != 1 || ecn > 3 || reserved || reserved2 ||
      !reporter || !reportSequence || key.pg >= 8 || key.dport != 100 ||
      key.frameBytes <= CustomHeader::GetStaticWholeHeaderSize()) return true;
  auto& p = m_paths.at(path);
  auto& stats = m_businessStats.at(path);
  ++stats.reportsRx; ++p.view.rxPackets; p.view.rxBytes += frameBytes;
  auto found = m_businessRecords.find(BusinessRecordKey(path, key));
  if (found == m_businessRecords.end() || found->second.retired) {
    ++stats.unmatched; return true;
  }
  const auto record = found->second;
  const uint64_t now = Simulator::Now().GetNanoSeconds();
  // Keep a bounded tombstone until the original timeout: duplicate feedback
  // or a repeated wire identity must not complete a newly opened sample.
  found->second.retired = true; --stats.pending;
  if (!record.transmitted || record.ambiguous || now < record.txNs ||
      now - record.enqueuedNs >= uint64_t(m_settings.timeout.GetNanoSeconds())) {
    ++stats.unmatched; return true;
  }
  ++stats.completed; stats.ceSamples += ecn == 3;
  stats.localQueueNs = record.txNs - record.enqueuedNs;
  stats.feedbackNs = now - record.txNs;
  stats.lastSampleNs = record.txNs; stats.lastFeedbackNs = now;
  std::cout << "DCI_PM_BUSINESS_SAMPLE node=" << GetNode()->GetId()
    << " path=" << path << " interface=" << p.view.interface
    << " sip=" << Ipv4Address(key.sip) << " dip=" << Ipv4Address(key.dip)
    << " sport=" << key.sport << " dport=" << key.dport << " pg=" << key.pg
    << " seq=" << key.seq << " ipid=" << key.ipid << " frame_bytes=" << key.frameBytes
    << " enqueue_ns=" << record.enqueuedNs << " tx_ns=" << record.txNs
    << " feedback_ns=" << now << " local_queue_ns=" << stats.localQueueNs
    << " data_feedback_ns=" << stats.feedbackNs
    << " total_ns=" << now - record.enqueuedNs << " ecn=" << ecn
    << " reporter_session=" << reporter << " report_seq=" << reportSequence << '\n';
  return true;
}
void DciPathManagerApp::PrintBusinessStats() const {
  if (!m_businessEnabled) return;
  for (const auto& item : m_businessStats) {
    const auto& s = item.second;
    std::cout << "DCI_PM_BUSINESS_STATS node=" << GetNode()->GetId()
      << " path=" << item.first << " interface=" << m_paths.at(item.first).view.interface
      << " candidates=" << s.candidates << " started=" << s.started
      << " completed=" << s.completed << " ce_samples=" << s.ceSamples
      << " pending=" << s.pending << " peak_pending=" << s.peakPending
      << " measurement_timeouts=" << s.timeouts << " no_tx=" << s.noTx
      << " tx_failures=" << s.txFailures << " capped=" << s.capped
      << " ambiguous=" << s.ambiguous << " reports_tx=" << s.reportsTx
      << " reports_rx=" << s.reportsRx << " report_tx_bytes=" << s.reportBytes
      << " report_suppressed=" << s.reportsSuppressed << " report_drops=" << s.reportDrops
      << " unmatched=" << s.unmatched << " local_queue_ns=" << s.localQueueNs
      << " data_feedback_ns=" << s.feedbackNs << " sample_tx_ns=" << s.lastSampleNs
      << " last_feedback_ns=" << s.lastFeedbackNs << '\n';
  }
  std::cout << "DCI_PM_BUSINESS_RESOURCE node=" << GetNode()->GetId()
    << " records=" << m_businessRecords.size() << " max_records=" << BUSINESS_MAX_PENDING
    << " record_value_bytes=" << sizeof(BusinessRecordKey) + sizeof(BusinessRecord)
    << " memory_scope=raw_record_excludes_map_and_event_overhead\n";
}

} // namespace ns3
