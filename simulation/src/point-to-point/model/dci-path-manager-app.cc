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
  if (m_settings.probeEnabled) Tick();
}
void DciPathManagerApp::StopService() {
  if (!m_running) return;
  m_running = false;
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
  if (ip.GetDestination() != p.view.localAddress ||
    packet->GetSize() != DciPathControlHeader::SIZE) return true;
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
  for (const auto& item : m_paths) result.push_back(item.second.view);
  return result;
}
uint32_t DciPathManagerApp::GetOutstanding() const {
  return m_outstanding;
}
void DciPathManagerApp::PrintStats() const {
  for (const auto& s : GetSnapshot())
    std::cout << "DCI_PM_STATS node=" << GetNode()->GetId() << " peer=" << s.peerAddress
      << " path=" << s.pathId << " interface=" << s.interface
      << " state=" << StatusName(s.status) << " has_rtt=" << s.hasRtt
      << " control_rtt_ns=" << s.latestRtt.GetNanoSeconds() << " samples=" << s.samples
      << " tx_packets=" << s.txPackets << " rx_packets=" << s.rxPackets
      << " tx_bytes=" << s.txBytes << " rx_bytes=" << s.rxBytes
      << " timeouts=" << s.timeouts << " enqueue_drops=" << s.enqueueDrops
      << " stale_replies=" << s.staleReplies << '\n';
  std::cout << "DCI_PM_RESOURCE node=" << GetNode()->GetId() << " paths=" << m_paths.size()
    << " outstanding=" << GetOutstanding() << " peak_outstanding=" << m_peakOutstanding
    << " pending_work=" << m_work.size() << " peak_work=" << m_peakWork
    << " work_drops=" << m_workDrops << " snapshot_bytes=" << m_paths.size() * sizeof(Snapshot)
    << " memory_scope=raw_snapshot_only\n";
}
} // namespace ns3
