// SPDX-License-Identifier: GPL-2.0-only
#include "dci-tunnel.h"
#include "ns3/abort.h"
#include "ns3/custom-header.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-header.h"
#include "ns3/node.h"
#include "ns3/ppp-header.h"
#include "ns3/simulator.h"
#include "ns3/simple-drop-tail-queue.h"
#include "ns3/udp-header.h"
#include "ns3/udp-socket-factory.h"
#include <algorithm>
#include <iostream>
#include <utility>

namespace ns3 {
NS_OBJECT_ENSURE_REGISTERED(DciAccessDevice);
NS_OBJECT_ENSURE_REGISTERED(DciTunnelGateway);
namespace {
constexpr uint16_t PORT = 49000;
constexpr uint32_t HEADER = DciTunnelHeader::SIZE;
uint32_t OuterIpUdpBytes()
{
  static const uint32_t bytes = Ipv4Header().GetSerializedSize() + UdpHeader().GetSerializedSize();
  return bytes;
}
uint32_t OuterWireBytes()
{
  // This repository extends PppHeader to 14 B, rather than stock ns-3's 2 B.
  static const uint32_t bytes = OuterIpUdpBytes() + PppHeader().GetSerializedSize();
  return bytes;
}
// The inner packet already contains its own PPP/IP/RDMA headers.
uint64_t Cost(Ptr<const Packet> p) { return uint64_t(p->GetSize()) + HEADER + OuterWireBytes(); }
void IgnoreBytes(uint32_t, uint64_t) {}
int IgnoreSent(Ptr<Packet>, CustomHeader&) { return 0; }
}
TypeId DciAccessDevice::GetTypeId()
{
  static TypeId tid = TypeId("ns3::DciAccessDevice").SetParent<QbbNetDevice>()
    .AddConstructor<DciAccessDevice>();
  return tid;
}
void DciAccessDevice::Receive(Ptr<Packet> packet)
{
  NS_ABORT_MSG_IF(ingress.IsNull(), "Gateway access port is not attached");
  ingress(packet);
}
TypeId DciTunnelGateway::GetTypeId()
{
  static TypeId tid = TypeId("ns3::DciTunnelGateway").SetParent<Application>()
    .AddConstructor<DciTunnelGateway>();
  return tid;
}
void DciTunnelGateway::Configure(Ptr<DciAccessDevice> access, Policy policy,
    uint64_t bufferBytes, bool ecn, uint64_t ecnThresholdBytes)
{
  NS_ABORT_MSG_IF(m_running || !access || bufferBytes < 4096 || ecnThresholdBytes == 0,
                  "Invalid gateway configuration");
  m_access = access;
  m_policy = policy;
  m_limit = bufferBytes;
  m_ecn = ecn;
  m_threshold = ecnThresholdBytes;
  m_manager = CreateObject<DciPathManager>();
  if (!access->m_rdmaEQ) access->m_rdmaEQ = CreateObject<RdmaEgressQueue>();
  access->m_rdmaEQ->m_qpGrp = CreateObject<RdmaQueuePairGroup>();
  access->m_rdmaUpdateTxBytes = MakeCallback(&IgnoreBytes);
  access->m_rdmaSentCb = MakeCallback(&IgnoreSent);
  access->ingress = MakeCallback(&DciTunnelGateway::Ingress, this);
}
void DciTunnelGateway::AddPath(Ptr<NetDevice> device, Ipv4Address local,
    Ipv4Address peer, DataRate rate)
{
  NS_ABORT_MSG_IF(m_running || m_paths.size() >= 255 || !device ||
                  rate.GetBitRate() == 0, "Invalid candidate path");
  Path path;
  path.device = device;
  path.local = local;
  path.peer = peer;
  path.rate = rate;
  m_paths.push_back(path);
  m_manager->AddPath();
}
uint8_t DciTunnelGateway::GetInnerTos(Ptr<const Packet> packet)
{
  auto copy = packet->Copy();
  PppHeader ppp;
  Ipv4Header ip;
  copy->RemoveHeader(ppp);
  copy->PeekHeader(ip);
  return ip.GetTos();
}
void DciTunnelGateway::SetInnerEcn(Ptr<Packet> packet, uint8_t ecn)
{
  PppHeader ppp;
  Ipv4Header ip;
  packet->RemoveHeader(ppp);
  packet->RemoveHeader(ip);
  ip.SetEcn(static_cast<Ipv4Header::EcnType>(ecn & 3));
  packet->AddHeader(ip);
  packet->AddHeader(ppp);
}
bool DciTunnelGateway::ReadKey(Ptr<const Packet> packet, Key& key, uint8_t& tos)
{
  if (packet->GetSize() < 24) return false;
  CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
  ch.getInt = 1;
  packet->PeekHeader(ch);
  uint32_t src = ch.sip, dst = ch.dip;
  uint16_t sport, dport, pg;
  if (ch.l3Prot == 0x11)
  { sport = ch.udp.sport; dport = ch.udp.dport; pg = ch.udp.pg; }
  else if (ch.l3Prot == 0xFC || ch.l3Prot == 0xFD)
  { sport = ch.ack.sport; dport = ch.ack.dport; pg = ch.ack.pg; }
  else if (ch.l3Prot == 0xFF)
  { sport = ch.cnp.fid; dport = 0; pg = ch.cnp.qIndex; }
  else return false; // Link-local PFC is deliberately not tunnelled.
  if (src > dst) { std::swap(src, dst); std::swap(sport, dport); }
  key = std::make_tuple(src, dst, sport, dport, pg);
  tos = ch.m_tos;
  return true;
}
void DciTunnelGateway::ConfigurePaths(const DciPathManager::Settings& settings)
{
  NS_ABORT_MSG_IF(!m_manager, "Call Configure before ConfigurePaths");
  m_manager->Configure(settings);
}
uint32_t DciTunnelGateway::Choose(const Key& key)
{
  std::vector<uint32_t> eligible;
  for (uint32_t id = 0; id < m_paths.size(); ++id)
    if (m_manager->CanSend(id)) eligible.push_back(id);
  // If validation has not completed, bind and hold this QP until its path is active.
  if (eligible.empty())
    for (uint32_t id = 0; id < m_paths.size(); ++id) eligible.push_back(id);
  if (m_policy == ROUND_ROBIN) return eligible[m_cursor++ % eligible.size()];
  if (m_policy == MIN_RTT)
  {
    uint32_t best = eligible.front();
    for (uint32_t id : eligible)
      if (m_manager->HasSample(id) && (!m_manager->HasSample(best) ||
          m_manager->GetRtt(id) < m_manager->GetRtt(best))) best = id;
    return best;
  }
  uint64_t hash = std::get<0>(key);
  hash = hash * 1099511628211ULL ^ std::get<1>(key);
  hash = hash * 1099511628211ULL ^ std::get<2>(key);
  hash = hash * 1099511628211ULL ^ std::get<3>(key);
  hash = hash * 1099511628211ULL ^ std::get<4>(key);
  return eligible[hash % eligible.size()];
}
void DciTunnelGateway::StartApplication()
{
  NS_ABORT_MSG_IF(!m_access || m_paths.empty(), "Gateway has no access port/path");
  m_running = true;
  for (uint32_t i = 0; i < m_paths.size(); ++i)
  {
    auto& path = m_paths[i];
    path.socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    path.socket->BindToNetDevice(path.device);
    path.socket->SetIpRecvTos(true);
    NS_ABORT_MSG_IF(path.socket->Bind(InetSocketAddress(path.local, PORT + i)) < 0,
                    "Cannot bind a WAN tunnel socket");
    path.socket->SetRecvCallback(MakeCallback(&DciTunnelGateway::Receive, this));
  }
  m_manager->Start(GetNode()->GetId(), MakeCallback(&DciTunnelGateway::SendControl, this),
                   MakeCallback(&DciTunnelGateway::Kick, this));
}
void DciTunnelGateway::StopApplication()
{
  m_running = false;
  if (m_manager) m_manager->Stop();
  for (auto& path : m_paths)
  {
    Simulator::Cancel(path.event);
    if (path.socket) path.socket->Close();
  }
}
void DciTunnelGateway::Ingress(Ptr<Packet> packet)
{
  Key key;
  uint8_t tos;
  if (!m_running || !ReadKey(packet, key, tos)) { ++m_invalid; return; }
  auto it = m_flows.find(key);
  if (it == m_flows.end())
  {
    Flow flow;
    flow.path = Choose(key);
    it = m_flows.emplace(key, flow).first;
    std::cout << "DCI_BIND node=" << GetNode()->GetId() << " sport=" << std::get<2>(key)
      << " dport=" << std::get<3>(key) << " pg=" << std::get<4>(key)
      << " path=" << flow.path << " reason=new_qp\n";
  }
  Flow& flow = it->second;
  Path& path = m_paths[flow.path];
  if (packet->GetSize() + HEADER + OuterIpUdpBytes() > path.device->GetMtu())
  { ++m_invalid; return; } // No outer IP fragmentation in this prototype.
  if (m_queued + Cost(packet) > m_limit) { ++m_ingressDrops; return; }
  ++flow.packets;
  flow.bytes += packet->GetSize();
  if (flow.queue.empty()) path.active.push_back(key);
  flow.queue.push_back(packet->Copy());
  path.queuedBytes += Cost(packet);
  m_queued += Cost(packet);
  m_peak = std::max(m_peak, m_queued);
  if (!path.event.IsRunning()) path.event = Simulator::ScheduleNow(&DciTunnelGateway::Drain, this, flow.path);
}
void DciTunnelGateway::Kick(uint32_t id)
{
  if (m_running && !m_paths[id].event.IsRunning())
    m_paths[id].event = Simulator::ScheduleNow(&DciTunnelGateway::Drain, this, id);
}
void DciTunnelGateway::SendControl(uint32_t id, DciTunnelHeader header)
{
  auto& path = m_paths.at(id);
  if (!m_running || path.control.size() >= 4096)
  {
    ++path.controlDrops;
    m_manager->ControlDropped(id, header);
    return;
  }
  Control control; control.header = header; control.queuedAt = Simulator::Now();
  path.control.push_back(control);
  Kick(id);
}
void DciTunnelGateway::Drain(uint32_t id)
{
  auto& path = m_paths.at(id);
  if (!m_running) return;
  Ptr<Packet> payload;
  uint64_t cost;
  uint8_t tos = 0;
  uint32_t innerSize = 0;
  bool control = !path.control.empty();
  DciTunnelHeader header;
  if (control)
  {
    auto item = path.control.front();
    path.control.pop_front();
    header = item.header;
    if (header.kind == DciTunnelHeader::ACK)
      header.delayNs += (Simulator::Now() - item.queuedAt).GetNanoSeconds();
    payload = Create<Packet>();
    payload->AddHeader(header);
    cost = payload->GetSize() + OuterWireBytes();
  }
  else
  {
    if (path.active.empty() || !m_manager->CanSend(id)) return;
    Key key = path.active.front();
    path.active.pop_front();
    auto& flow = m_flows.at(key);
    auto inner = flow.queue.front();
    flow.queue.pop_front();
    if (!flow.queue.empty()) path.active.push_back(key);
    tos = GetInnerTos(inner);
    if (m_ecn && path.queuedBytes >= m_threshold && ((tos & 3) == 1 || (tos & 3) == 2))
    { tos = (tos & 0xfc) | 3; ++path.markedPackets; }
    innerSize = inner->GetSize();
    header = m_manager->DataSent(id, innerSize);
    payload = inner->Copy();
    payload->RemoveAllPacketTags();
    payload->RemoveAllByteTags();
    payload->AddHeader(header);
    cost = Cost(inner);
    path.queuedBytes -= cost;
    m_queued -= cost;
  }
  InetSocketAddress peer(path.peer, PORT + id);
  peer.SetTos(tos);
  int sent = path.socket->SendTo(payload, 0, peer);
  if (sent != static_cast<int>(payload->GetSize()))
  {
    ++m_sendErrors;
    if (control) m_manager->ControlDropped(id, header);
  }
  else if (control)
  {
    ++path.controlTx;
    m_manager->ControlSent(id, header);
  }
  else { ++path.txPackets; path.txInnerBytes += innerSize; }
  // Both control and data consume WAN line capacity. Hold the next send slot
  // even when empty, so a new arrival cannot bypass this serialization time.
  Time interval = path.rate.CalculateBytesTxTime(cost);
  path.event = Simulator::Schedule(std::max(interval, NanoSeconds(1)),
                                   &DciTunnelGateway::Drain, this, id);
}
void DciTunnelGateway::Receive(Ptr<Socket> socket)
{
  uint32_t id = 0;
  while (id < m_paths.size() && m_paths[id].socket != socket) ++id;
  NS_ABORT_MSG_IF(id == m_paths.size(), "Unknown tunnel socket");
  auto& path = m_paths[id];
  Address from;
  while (Ptr<Packet> packet = socket->RecvFrom(from))
  {
    if (packet->GetSize() < HEADER || !InetSocketAddress::IsMatchingType(from) ||
        InetSocketAddress::ConvertFrom(from).GetIpv4() != path.peer ||
        InetSocketAddress::ConvertFrom(from).GetPort() != PORT + id)
    { ++m_invalid; continue; }
    DciTunnelHeader header;
    packet->RemoveHeader(header);
    if (!header.IsValid() || header.path != id || header.number == 0)
    { ++m_invalid; continue; }
    if (header.kind != DciTunnelHeader::DATA)
    {
      if (packet->GetSize() != 0 || !m_manager->ControlReceived(id, header)) ++m_invalid;
      else ++path.controlRx;
      continue;
    }
    if (header.value != packet->GetSize() || packet->GetSize() < 24)
    { ++m_invalid; continue; }
    SocketIpTosTag tag;
    if (!packet->PeekPacketTag(tag)) { ++m_invalid; continue; }
    const uint8_t outerEcn = tag.GetTos() & 3;
    packet->RemoveAllPacketTags();
    packet->RemoveAllByteTags();
    Key key;
    uint8_t innerTos;
    if (!ReadKey(packet, key, innerTos)) { ++m_invalid; continue; }
    if (outerEcn == 3)
    {
      if ((innerTos & 3) == 0) { ++m_invalid; continue; }
      SetInnerEcn(packet, 3);
    }
    auto it = m_flows.find(key);
    if (it == m_flows.end())
    {
      Flow reverse; reverse.path = id;
      m_flows.emplace(key, reverse);
    }
    else NS_ABORT_MSG_IF(it->second.path != id, "A QP changed path in fixed-QP v1");
    // This ACK confirms receipt at THIS gateway, before local host delivery.
    // It does not complete an RDMA operation and does not trigger outer retransmit.
    m_manager->DataReceived(id, header);
    ++path.rxPackets;
    path.rxInnerBytes += packet->GetSize();
    if (uint64_t(m_access->m_rdmaEQ->m_ackQ->GetNBytes()) + packet->GetSize() > m_limit)
    { ++m_egressDrops; continue; }
    m_access->RdmaEnqueueHighPrioQ(packet);
    m_access->TriggerTransmit();
  }
}
void DciTunnelGateway::WritePathCsv(std::ostream& os) const
{ m_manager->WriteCsv(os); }
void DciTunnelGateway::PrintStats() const
{
  m_manager->PrintStats();
  std::cout << "DCI_GATEWAY node=" << GetNode()->GetId() << " flows=" << m_flows.size()
    << " pending_peak_bytes=" << m_peak << " pending_bytes=" << m_queued
    << " ingress_drops=" << m_ingressDrops << " egress_drops=" << m_egressDrops
    << " invalid_packets=" << m_invalid << " send_errors=" << m_sendErrors << '\n';
  for (uint32_t i = 0; i < m_paths.size(); ++i)
  {
    const auto& p = m_paths[i];
    std::cout << "DCI_PATH node=" << GetNode()->GetId() << " path=" << i
      << " tx_packets=" << p.txPackets << " rx_packets=" << p.rxPackets
      << " tx_inner_bytes=" << p.txInnerBytes << " rx_inner_bytes=" << p.rxInnerBytes
      << " control_tx=" << p.controlTx << " control_rx=" << p.controlRx
      << " control_drops=" << p.controlDrops
      << " ecn_marks=" << p.markedPackets << " queued_bytes=" << p.queuedBytes << '\n';
  }
}
} // namespace ns3
