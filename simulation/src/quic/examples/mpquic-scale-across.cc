// SPDX-License-Identifier: GPL-2.0-only
// One MPQUIC connection over independent, abstracted inter-DC routes.
// This transport experiment does not model RDMA verbs, RNIC execution or TLS cost.
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/quic-module.h"

#include <iomanip>
#include <iostream>
#include <set>
#include <vector>

using namespace ns3;

namespace {
struct PathStats
{
  uint64_t txBytes{0};
  uint64_t txPackets{0};
  uint64_t rxBytes{0};
  uint64_t drops{0};
  std::set<uint64_t> connectionIds;
};
std::vector<PathStats> pathStats;
uint64_t deliveredBytes = 0;
uint64_t expectedBytes = 0;
Time completedAt;
bool completed = false;

void OnTx(uint32_t path, Ptr<const Packet> original)
{
  auto& stats = pathStats.at(path);
  stats.txBytes += original->GetSize();
  ++stats.txPackets;
  Ptr<Packet> packet = original->Copy();
  PppHeader ppp;
  Ipv4Header ip;
  UdpHeader udp;
  QuicHeader quic;
  packet->RemoveHeader(ppp);
  packet->RemoveHeader(ip);
  // Only the first IPv4 fragment begins with UDP and QUIC headers.
  if (ip.GetProtocol() == 17 && ip.GetFragmentOffset() == 0)
  {
    packet->RemoveHeader(udp);
    packet->RemoveHeader(quic);
    if (quic.HasConnectionId())
    {
      stats.connectionIds.insert(quic.GetConnectionId());
    }
  }
}

void OnRx(uint32_t path, Ptr<const Packet> packet)
{
  pathStats.at(path).rxBytes += packet->GetSize();
}

void OnDrop(uint32_t path, Ptr<const Packet>)
{
  ++pathStats.at(path).drops;
}

void OnDelivered(Ptr<const Packet> packet, const Address&)
{
  deliveredBytes += packet->GetSize();
  if (!completed && deliveredBytes >= expectedBytes)
  {
    completedAt = Simulator::Now();
    completed = true;
    // Leave time for transport ACKs; the reported FCT ends at application delivery.
  }
}
} // namespace

int main(int argc, char** argv)
{
  uint32_t paths = 2;
  std::string rate0 = "1Gbps", rate1 = "1Gbps";
  double rtt0Ms = 10, rtt1Ms = 20, loss = 0, stop = 3.0;
  uint64_t bytes = 16 * 1024 * 1024;
  uint32_t bufferBytes = 64 * 1024 * 1024;
  uint32_t scheduler = 0, cc = 0, seed = 1;
  bool pcap = false, requireAllPaths = false;
  CommandLine cmd(__FILE__);
  cmd.AddValue("paths", "Number of independent DCI routes (1 or 2)", paths);
  cmd.AddValue("rate0", "Capacity of route 0", rate0);
  cmd.AddValue("rate1", "Capacity of route 1", rate1);
  cmd.AddValue("rtt0Ms", "Route 0 propagation RTT in milliseconds", rtt0Ms);
  cmd.AddValue("rtt1Ms", "Route 1 propagation RTT in milliseconds", rtt1Ms);
  cmd.AddValue("loss", "Independent forward data packet loss probability on each route", loss);
  cmd.AddValue("bytes", "Application bytes in a single stream and connection", bytes);
  cmd.AddValue("bufferBytes", "Socket/stream buffer capacity, not modeled NIC SRAM", bufferBytes);
  cmd.AddValue("scheduler", "0=round-robin, 1=minRTT, 2=BLEST, 3=ECF, 4=Peekaboo", scheduler);
  cmd.AddValue("cc", "0=uncoupled QUIC NewReno, 1=OLIA", cc);
  cmd.AddValue("stop", "Simulation deadline in seconds", stop);
  cmd.AddValue("seed", "Random seed", seed);
  cmd.AddValue("pcap", "Write per-interface packet captures in the current directory", pcap);
  cmd.AddValue("requireAllPaths", "Also require more than 10000 wire bytes on every route", requireAllPaths);
  cmd.Parse(argc, argv);
  NS_ABORT_MSG_IF(paths < 1 || paths > 2, "paths must be 1 or 2");
  NS_ABORT_MSG_IF(scheduler > 4 || cc > 1, "Unsupported scheduler or congestion control");
  NS_ABORT_MSG_IF(paths == 1 && scheduler > 1, "Use round-robin or minRTT for the single-path baseline");
  NS_ABORT_MSG_IF(bytes == 0 || bufferBytes < 4096 || stop <= 0.1,
                  "Require positive bytes, bufferBytes >= 4096 and stop > 0.1");
  NS_ABORT_MSG_IF(rtt0Ms <= 0 || rtt1Ms <= 0 || loss < 0 || loss >= 1,
                  "Require positive RTT and loss in [0,1)");
  RngSeedManager::SetSeed(seed);
  expectedBytes = bytes;
  pathStats.resize(paths);
  Config::SetDefault("ns3::QuicSocketBase::EnableMultipath", BooleanValue(paths > 1));
  Config::SetDefault("ns3::QuicSocketBase::CcType", IntegerValue(cc));
  // Leave room for this model's QUIC, UDP and IPv4 headers within a 1500-byte MTU.
  Config::SetDefault("ns3::QuicSocketBase::MaxPacketSize", UintegerValue(1400));
  Config::SetDefault("ns3::QuicL4Protocol::SocketType",
                     TypeIdValue(cc == 0 ? QuicCongestionOps::GetTypeId() : MpQuicCongestionOps::GetTypeId()));
  Config::SetDefault("ns3::MpQuicScheduler::SchedulerType", IntegerValue(scheduler));
  Config::SetDefault("ns3::QuicSocketBase::SocketSndBufSize", UintegerValue(bufferBytes));
  Config::SetDefault("ns3::QuicSocketBase::SocketRcvBufSize", UintegerValue(bufferBytes));
  Config::SetDefault("ns3::QuicStreamBase::StreamSndBufSize", UintegerValue(bufferBytes));
  Config::SetDefault("ns3::QuicStreamBase::StreamRcvBufSize", UintegerValue(bufferBytes));

  NodeContainer endpoints;
  endpoints.Create(2);
  QuicHelper quic;
  quic.InstallQuic(endpoints);
  Ipv4Address primaryPeer;
  for (uint32_t path = 0; path < paths; ++path)
  {
    // Each channel abstracts one complete DCI route; its Delay is one-way.
    PointToPointHelper route;
    route.SetDeviceAttribute("DataRate", StringValue(path == 0 ? rate0 : rate1));
    route.SetChannelAttribute("Delay", TimeValue(Seconds((path == 0 ? rtt0Ms : rtt1Ms) / 2000.0)));
    route.SetQueue("ns3::DropTailQueue<Packet>", "MaxSize", QueueSizeValue(QueueSize("10000p")));
    NetDeviceContainer devices = route.Install(endpoints);
    Ipv4AddressHelper addressing;
    const std::string subnet = "10." + std::to_string(path + 1) + ".0.0";
    addressing.SetBase(subnet.c_str(), "255.255.255.0");
    auto interfaces = addressing.Assign(devices);
    if (path == 0) primaryPeer = interfaces.GetAddress(1);
    auto error = CreateObject<RateErrorModel>();
    error->SetAttribute("ErrorUnit", EnumValue(RateErrorModel::ERROR_UNIT_PACKET));
    error->SetAttribute("ErrorRate", DoubleValue(loss));
    error->AssignStreams(path + 1);
    devices.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(error));
    devices.Get(0)->TraceConnectWithoutContext("PhyTxBegin", MakeBoundCallback(&OnTx, path));
    devices.Get(1)->TraceConnectWithoutContext("PhyRxEnd", MakeBoundCallback(&OnRx, path));
    devices.Get(1)->TraceConnectWithoutContext("PhyRxDrop", MakeBoundCallback(&OnDrop, path));
    if (pcap) route.EnablePcap("mpquic-scale-across", devices);
  }

  constexpr uint16_t port = 4433;
  PacketSinkHelper receiver("ns3::QuicSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
  auto sinks = receiver.Install(endpoints.Get(1));
  sinks.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&OnDelivered));
  sinks.Start(Seconds(0));
  sinks.Stop(Seconds(stop));
  MpquicBulkSendHelper sender("ns3::QuicSocketFactory", InetSocketAddress(primaryPeer, port));
  sender.SetAttribute("MaxBytes", UintegerValue(bytes));
  sender.SetAttribute("SendSize", UintegerValue(4096));
  auto sources = sender.Install(endpoints.Get(0));
  sources.Start(Seconds(0.1));
  sources.Stop(Seconds(stop));
  Simulator::Stop(Seconds(stop));
  Simulator::Run();

  std::set<uint64_t> allConnectionIds;
  bool usedAllPaths = true;
  for (uint32_t path = 0; path < paths; ++path)
  {
    const auto& stats = pathStats[path];
    allConnectionIds.insert(stats.connectionIds.begin(), stats.connectionIds.end());
    // More than setup traffic must traverse each route for this smoke experiment.
    usedAllPaths = usedAllPaths && stats.txBytes > 10000;
    std::cout << "DCI_PATH path=" << path << " tx_bytes=" << stats.txBytes
              << " rx_bytes=" << stats.rxBytes << " tx_packets=" << stats.txPackets
              << " drops=" << stats.drops << " connection_ids=" << stats.connectionIds.size() << '\n';
  }
  const double fct = completed ? completedAt.GetSeconds() - 0.1 : 0;
  const bool success = completed && deliveredBytes == bytes &&
                       (!requireAllPaths || usedAllPaths) && allConnectionIds.size() == 1;
  std::cout << std::setprecision(9) << "DCI_RESULT success=" << success
            << " paths=" << paths << " delivered_bytes=" << deliveredBytes
            << " expected_bytes=" << bytes << " connection_ids=" << allConnectionIds.size()
            << " used_all_paths=" << usedAllPaths
            << " fct_s=" << fct << " goodput_gbps=" << (fct > 0 ? bytes * 8.0 / fct / 1e9 : 0)
            << " scheduler=" << scheduler << " cc=" << cc << '\n';
  Simulator::Destroy();
  return success ? 0 : 1;
}
