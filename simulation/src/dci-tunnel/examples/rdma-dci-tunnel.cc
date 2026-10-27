// SPDX-License-Identifier: GPL-2.0-only
// RDMA/RC over UDP with MPQUIC-inspired path validation and feedback.
#include "ns3/core-module.h"
#include "ns3/dci-tunnel.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;
namespace {
struct Flow
{
  uint32_t src = 0, dst = 3;
  uint16_t pg = 3, sport = 0, dport = 0;
  uint64_t size = 0, receivedBytes = 0;
  double start = 0;
  bool received = false, acknowledged = false;
  Time receivedAt, acknowledgedAt;
  Ptr<RdmaQueuePair> qp;
};
std::vector<Flow> flows;
Ipv4Address sourceIp, destIp;
void IgnoreApp() {}
struct Observer
{
  Ptr<RdmaHw> hw;
  bool isReceiver = false;
  int Receive(Ptr<QbbNetDevice> dev, Ptr<Packet> p, CustomHeader& ch)
  {
    // Compatibility with THIS repository: ReceiveUdp treats every nonzero
    // ECN value as congestion. Normalize ECT to zero; leave CE as 3.
    if (ch.l3Prot == 0x11 && (ch.m_tos & 3) != 3 && (ch.m_tos & 3) != 0)
    {
      DciTunnelGateway::SetInnerEcn(p, 0);
      p->PeekHeader(ch);
    }
    hw->Receive(dev, p, ch);
    if (isReceiver)
      for (auto& f : flows)
      {
        auto rx = hw->GetRxQp(destIp.Get(), sourceIp.Get(), f.dport, f.sport, f.pg, false);
        if (!rx) continue;
        f.receivedBytes = rx->ReceiverNextExpectedSeq;
        if (!f.received && f.receivedBytes >= f.size)
        { f.received = true; f.receivedAt = Simulator::Now(); }
      }
    return 0;
  }
};
Ptr<Packet> EctPacket(Ptr<RdmaHw> hw, Ptr<RdmaQueuePair> qp)
{
  auto p = hw->GetNxtPacket(qp);
  // Original GetNxtPacket sets Not-ECT. Opt this simulated DCQCN source into
  // ECT(0), paired with Observer::Receive above. No RDMA core file is edited.
  DciTunnelGateway::SetInnerEcn(p, 2);
  return p;
}
void AckComplete(Ptr<RdmaQueuePair> qp)
{
  for (auto& f : flows)
    if (f.sport == qp->sport && !f.acknowledged)
    { f.acknowledged = true; f.acknowledgedAt = Simulator::Now(); }
}
NetDeviceContainer Access(Ptr<Node> host, Ptr<Node> gateway,
    const std::string& rate, double delayUs)
{
  auto h = CreateObject<QbbNetDevice>();
  auto g = CreateObject<DciAccessDevice>();
  // Set these explicitly, including the constructor's RDMA queue, so this
  // example does not depend on nullable PointerValue attribute defaults.
  h->m_rdmaEQ = CreateObject<RdmaEgressQueue>();
  g->m_rdmaEQ = CreateObject<RdmaEgressQueue>();
  h->SetQueue(CreateObject<BEgressQueue>());
  g->SetQueue(CreateObject<BEgressQueue>());
  h->SetAddress(Mac48Address::Allocate());
  g->SetAddress(Mac48Address::Allocate());
  h->SetAttribute("DataRate", StringValue(rate));
  g->SetAttribute("DataRate", StringValue(rate));
  host->AddDevice(h);
  gateway->AddDevice(g);
  auto channel = CreateObject<QbbChannel>();
  channel->SetAttribute("Delay", TimeValue(Seconds(delayUs * 1e-6)));
  h->Attach(channel);
  g->Attach(channel);
  NetDeviceContainer devices;
  devices.Add(h);
  devices.Add(g);
  return devices;
}
Ptr<RdmaDriver> Rdma(Ptr<Node> host, Ptr<QbbNetDevice> dev,
    Ipv4Address local, Ipv4Address peer, uint32_t mtu, double rtoMs)
{
  auto ip = host->GetObject<Ipv4>();
  uint32_t index = ip->AddInterface(dev);
  ip->AddAddress(index, Ipv4InterfaceAddress(local, Ipv4Mask("255.255.255.0")));
  ip->SetUp(index);
  auto hw = CreateObject<RdmaHw>();
  hw->m_gpus_per_server = 1;
  hw->SetAttribute("Mtu", UintegerValue(mtu));
  // This attribute is a BYTE threshold. ACK every packet including a short
  // final payload, avoiding an unrelated tail-ACK timeout in the smoke case.
  hw->SetAttribute("L2AckInterval", UintegerValue(1));
  hw->SetAttribute("CcMode", UintegerValue(1));
  hw->ConfigureRcAckRetry(RdmaHw::RC_RETRY_VANILLA, static_cast<uint64_t>(rtoMs * 1e6), 7);
  auto driver = CreateObject<RdmaDriver>();
  driver->SetNode(host);
  driver->SetRdmaHw(hw);
  host->AggregateObject(driver);
  driver->Init();
  hw->m_rtTable[peer.Get()].push_back(dev->GetIfIndex());
  dev->m_rdmaEQ->m_rdmaGetNxtPkt = MakeBoundCallback(&EctPacket, hw);
  return driver;
}
uint64_t forwardWire[2] = {}, reverseWire[2] = {}, wanDrops[2] = {};
void Forward(uint32_t id, Ptr<const Packet> p) { forwardWire[id] += p->GetSize(); }
void Reverse(uint32_t id, Ptr<const Packet> p) { reverseWire[id] += p->GetSize(); }
void WanDrop(uint32_t id, Ptr<const Packet>) { ++wanDrops[id]; }
}

int main(int argc, char** argv)
{
  std::string flowFile, resultFile, accessRate = "400Gbps", rate0 = "100Gbps", rate1 = "200Gbps";
  double accessDelayUs = 2, rtt0Ms = 10, rtt1Ms = 20, stop = 3, rtoMs = 100, loss = 0;
  uint32_t paths = 2, policy = 0, mtu = 1000, seed = 1, windowBytes = 64 * 1024 * 1024;
  uint64_t bufferBytes = 64 * 1024 * 1024, ecnThresholdBytes = 512 * 1024;
  bool ecn = true;
  std::string pathResultFile;
  double probeIntervalMs = 10, initialPathTimeoutMs = 200, maxTunnelAckDelayUs = 50;
  uint32_t pathFailureThreshold = 3, tunnelAckEvery = 16, rttSampleEvery = 64;
  double outageStart = -1, outageEnd = -1;
  uint32_t outagePath = 0;
  CommandLine cmd(__FILE__);
  cmd.AddValue("flowFile", "Six-column RDMA flow file with count header", flowFile);
  cmd.AddValue("resultFile", "Optional per-flow CSV", resultFile);
  cmd.AddValue("pathResultFile", "Optional final per-gateway/path CSV", pathResultFile);
  cmd.AddValue("probeIntervalMs", "Interval after each probe attempt", probeIntervalMs);
  cmd.AddValue("initialPathTimeoutMs", "Probe timeout before RTT is measured", initialPathTimeoutMs);
  cmd.AddValue("pathFailureThreshold", "Consecutive unanswered attempts before FAILED", pathFailureThreshold);
  cmd.AddValue("tunnelAckEvery", "ACK after this many tunnel DATA packets, <=32", tunnelAckEvery);
  cmd.AddValue("maxTunnelAckDelayUs", "Maximum receiver ACK batching delay", maxTunnelAckDelayUs);
  cmd.AddValue("rttSampleEvery", "Retain one DATA send-time sample per N packets", rttSampleEvery);
  cmd.AddValue("outageStart", "Optional WAN receive blackhole start, -1=disabled", outageStart);
  cmd.AddValue("outageEnd", "Optional WAN receive blackhole end", outageEnd);
  cmd.AddValue("outagePath", "Path to blackhole in both directions", outagePath);
  cmd.AddValue("accessRate", "Both local Qbb link capacities", accessRate);
  cmd.AddValue("accessDelayUs", "Each local link one-way delay", accessDelayUs);
  cmd.AddValue("rate0", "WAN path 0 capacity", rate0);
  cmd.AddValue("rate1", "WAN path 1 capacity", rate1);
  cmd.AddValue("rtt0Ms", "WAN path 0 propagation RTT", rtt0Ms);
  cmd.AddValue("rtt1Ms", "WAN path 1 propagation RTT", rtt1Ms);
  cmd.AddValue("paths", "Use one or two paths", paths);
  cmd.AddValue("policy", "0=QP round robin, 1=QP hash, 2=minimum measured smoothed RTT", policy);
  cmd.AddValue("mtu", "Inner RDMA payload bytes", mtu);
  cmd.AddValue("windowBytes", "Fixed inner byte window; 0=unlimited", windowBytes);
  cmd.AddValue("bufferBytes", "Pending WAN queue budget and separate local egress limit", bufferBytes);
  cmd.AddValue("ecn", "Mark congestion at gateway WAN egress queue", ecn);
  cmd.AddValue("ecnThresholdBytes", "Per-path pending-byte CE threshold", ecnThresholdBytes);
  cmd.AddValue("rtoMs", "Inner RC ACK timeout", rtoMs);
  cmd.AddValue("loss", "Forward WAN packet loss probability", loss);
  cmd.AddValue("stop", "Simulation deadline", stop);
  cmd.AddValue("seed", "Random seed", seed);
  cmd.Parse(argc, argv);
  NS_ABORT_MSG_IF(paths < 1 || paths > 2 || policy > 2 || mtu < 128 || mtu > 1000 ||
    bufferBytes < 4096 || ecnThresholdBytes == 0 || rtoMs <= 0 || stop <= 0 ||
    accessDelayUs < 0 || rtt0Ms <= 0 || rtt1Ms <= 0 || loss < 0 || loss >= 1,
    "Invalid parameters");
  for (double value : {accessDelayUs, rtt0Ms, rtt1Ms, stop, rtoMs, loss,
                       probeIntervalMs, initialPathTimeoutMs, maxTunnelAckDelayUs,
                       outageStart, outageEnd})
    NS_ABORT_MSG_IF(!std::isfinite(value), "Non-finite parameter");
  bool outage = outageStart >= 0;
  NS_ABORT_MSG_IF(probeIntervalMs <= 0 || initialPathTimeoutMs <= 0 ||
    maxTunnelAckDelayUs <= 0 || pathFailureThreshold == 0 || tunnelAckEvery == 0 ||
    tunnelAckEvery > 32 || rttSampleEvery == 0 ||
    (!outage && (outageStart != -1 || outageEnd != -1)) ||
    (outage && (outagePath >= paths || outageEnd <= outageStart || outageEnd >= stop)),
    "Invalid path maintenance/outage parameters");
  std::ifstream input(flowFile);
  uint32_t count = 0;
  NS_ABORT_MSG_IF(!(input >> count) || count == 0 || count > 1024, "Invalid flow count/file");
  flows.resize(count);
  for (uint32_t i = 0; i < count; ++i)
  {
    auto& f = flows[i];
    uint32_t pg, dport;
    NS_ABORT_MSG_IF(!(input >> f.src >> f.dst >> pg >> dport >> f.size >> f.start), "Invalid flow row");
    NS_ABORT_MSG_IF(f.src != 0 || f.dst != 3 || pg == 0 || pg >= 8 || dport == 0 || dport > 65535 ||
      f.size == 0 || f.size > 1024ULL * 1024 * 1024 || f.start <= 0.001 || f.start >= stop,
      "v1 supports A(0)-to-B(3) flows with priority 1..7, <=1GiB per flow");
    f.pg = pg; f.dport = dport; f.sport = 10000 + i;
  }
  std::string extra;
  NS_ABORT_MSG_IF(input >> extra, "Trailing data in normalized flow file");
  RngSeedManager::SetSeed(seed);
  NodeContainer nodes;
  nodes.Create(4);
  InternetStackHelper internet;
  internet.Install(nodes);
  NodeContainer gateways(nodes.Get(1), nodes.Get(2));
  auto left = Access(nodes.Get(0), nodes.Get(1), accessRate, accessDelayUs);
  auto right = Access(nodes.Get(3), nodes.Get(2), accessRate, accessDelayUs);
  Ptr<DciTunnelGateway> gw[2];
  DciPathManager::Settings pathSettings;
  pathSettings.probeInterval = Seconds(probeIntervalMs / 1000);
  pathSettings.initialTimeout = Seconds(initialPathTimeoutMs / 1000);
  pathSettings.maxAckDelay = Seconds(maxTunnelAckDelayUs / 1e6);
  pathSettings.failureThreshold = pathFailureThreshold;
  pathSettings.ackEvery = tunnelAckEvery;
  pathSettings.sampleEvery = rttSampleEvery;
  for (uint32_t side = 0; side < 2; ++side)
  {
    gw[side] = CreateObject<DciTunnelGateway>();
    gateways.Get(side)->AddApplication(gw[side]);
    gw[side]->Configure(DynamicCast<DciAccessDevice>(side == 0 ? left.Get(1) : right.Get(1)),
      static_cast<DciTunnelGateway::Policy>(policy), bufferBytes, ecn, ecnThresholdBytes);
    gw[side]->ConfigurePaths(pathSettings);
    gw[side]->SetStartTime(Seconds(0));
    gw[side]->SetStopTime(Seconds(stop + 1));
  }
  for (uint32_t path = 0; path < paths; ++path)
  {
    PointToPointHelper route;
    DataRate rate(path == 0 ? rate0 : rate1);
    double rtt = path == 0 ? rtt0Ms : rtt1Ms;
    route.SetDeviceAttribute("DataRate", DataRateValue(rate));
    route.SetChannelAttribute("Delay", TimeValue(Seconds(rtt / 2000)));
    route.SetQueue("ns3::DropTailQueue<Packet>", "MaxSize", QueueSizeValue(QueueSize("64p")));
    auto devices = route.Install(gateways);
    Ipv4AddressHelper address;
    std::string subnet = "10." + std::to_string(path + 1) + ".0.0";
    address.SetBase(subnet.c_str(), "255.255.255.0");
    auto interfaces = address.Assign(devices);
    gw[0]->AddPath(devices.Get(0), interfaces.GetAddress(0), interfaces.GetAddress(1), rate);
    gw[1]->AddPath(devices.Get(1), interfaces.GetAddress(1), interfaces.GetAddress(0), rate);
    auto error = CreateObject<RateErrorModel>();
    error->SetAttribute("ErrorUnit", EnumValue(RateErrorModel::ERROR_UNIT_PACKET));
    error->SetAttribute("ErrorRate", DoubleValue(loss));
    error->AssignStreams(path + 1);
    devices.Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(error));
    auto reverseError = CreateObject<RateErrorModel>();
    reverseError->SetAttribute("ErrorUnit", EnumValue(RateErrorModel::ERROR_UNIT_PACKET));
    reverseError->SetAttribute("ErrorRate", DoubleValue(0));
    reverseError->AssignStreams(path + 100);
    devices.Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(reverseError));
    if (outage && path == outagePath)
    {
      Simulator::Schedule(Seconds(outageStart), [error, reverseError, path]() {
        error->SetAttribute("ErrorRate", DoubleValue(1));
        reverseError->SetAttribute("ErrorRate", DoubleValue(1));
        std::cout << "DCI_OUTAGE time_s=" << Simulator::Now().GetSeconds()
          << " path=" << path << " blackhole=1\n";
      });
      Simulator::Schedule(Seconds(outageEnd), [error, reverseError, path, loss]() {
        error->SetAttribute("ErrorRate", DoubleValue(loss));
        reverseError->SetAttribute("ErrorRate", DoubleValue(0));
        std::cout << "DCI_OUTAGE time_s=" << Simulator::Now().GetSeconds()
          << " path=" << path << " blackhole=0\n";
      });
    }
    devices.Get(0)->TraceConnectWithoutContext("PhyTxBegin", MakeBoundCallback(&Forward, path));
    devices.Get(1)->TraceConnectWithoutContext("PhyTxBegin", MakeBoundCallback(&Reverse, path));
    devices.Get(1)->TraceConnectWithoutContext("PhyRxDrop", MakeBoundCallback(&WanDrop, path));
  }
  sourceIp = Ipv4Address(0x0b000001 + (nodes.Get(0)->GetId() << 8));
  destIp = Ipv4Address(0x0b000001 + (nodes.Get(3)->GetId() << 8));
  auto sender = Rdma(nodes.Get(0), DynamicCast<QbbNetDevice>(left.Get(0)), sourceIp, destIp, mtu, rtoMs);
  auto receiver = Rdma(nodes.Get(3), DynamicCast<QbbNetDevice>(right.Get(0)), destIp, sourceIp, mtu, rtoMs);
  Observer observers[2];
  observers[0].hw = sender->m_rdma;
  observers[1].hw = receiver->m_rdma;
  observers[1].isReceiver = true;
  DynamicCast<QbbNetDevice>(left.Get(0))->m_rdmaReceiveCb = MakeCallback(&Observer::Receive, &observers[0]);
  DynamicCast<QbbNetDevice>(right.Get(0))->m_rdmaReceiveCb = MakeCallback(&Observer::Receive, &observers[1]);
  sender->TraceConnectWithoutContext("QpComplete", MakeCallback(&AckComplete));
  for (uint32_t i = 0; i < count; ++i)
    Simulator::Schedule(Seconds(flows[i].start), [&, i]() {
      auto& f = flows[i];
      sender->AddQueuePair(0, 3, i, f.size, f.pg, sourceIp, destIp, f.sport, f.dport,
        windowBytes, static_cast<uint64_t>((std::max(rtt0Ms, rtt1Ms) + accessDelayUs / 250) * 1e6),
        MakeCallback(&IgnoreApp), MakeCallback(&IgnoreApp));
      f.qp = sender->m_rdma->GetQp(destIp.Get(), f.sport, f.pg);
    });
  Simulator::Stop(Seconds(stop));
  Simulator::Run();
  bool success = !gw[0]->HasProtocolErrors() && !gw[1]->HasProtocolErrors();
  std::ofstream csv;
  if (!resultFile.empty())
  {
    csv.open(resultFile);
    NS_ABORT_MSG_IF(!csv, "Cannot open resultFile");
    csv << "flow_id,success,bytes,received_bytes,receiver_fct_s,sender_ack_fct_s,retrans_bytes,timeouts\n";
  }
  for (uint32_t i = 0; i < count; ++i)
  {
    const auto& f = flows[i];
    bool ok = f.received && f.acknowledged && f.receivedBytes == f.size && f.qp && !f.qp->m_qpError;
    success = success && ok;
    double rxFct = f.received ? f.receivedAt.GetSeconds() - f.start : 0;
    double ackFct = f.acknowledged ? f.acknowledgedAt.GetSeconds() - f.start : 0;
    uint64_t retrans = f.qp ? f.qp->m_retransBytes : 0;
    uint64_t timeouts = f.qp ? f.qp->m_timeoutCount : 0;
    std::cout << std::setprecision(10) << "DCI_FLOW id=" << i << " success=" << ok
      << " bytes=" << f.size << " received_bytes=" << f.receivedBytes
      << " receiver_fct_s=" << rxFct << " sender_ack_fct_s=" << ackFct
      << " retrans_bytes=" << retrans << " timeouts=" << timeouts << '\n';
    if (csv) csv << std::setprecision(12) << i << ',' << ok << ',' << f.size << ',' << f.receivedBytes
      << ',' << rxFct << ',' << ackFct << ',' << retrans << ',' << timeouts << '\n';
  }
  gw[0]->PrintStats(); gw[1]->PrintStats();
  if (!pathResultFile.empty())
  {
    std::ofstream pathCsv(pathResultFile);
    NS_ABORT_MSG_IF(!pathCsv, "Cannot open pathResultFile");
    pathCsv << "node,path,state,validated,latest_rtt_ms,min_rtt_ms,srtt_ms,rttvar_ms,"
      "data_rtt_samples,probe_rtt_samples,ack_tx,ack_rx,probe_timeouts,"
      "expired_samples,evicted_samples\n";
    pathCsv << std::setprecision(12);
    gw[0]->WritePathCsv(pathCsv); gw[1]->WritePathCsv(pathCsv);
  }
  for (uint32_t path = 0; path < paths; ++path)
    std::cout << "DCI_WIRE path=" << path << " forward_bytes=" << forwardWire[path]
      << " reverse_bytes=" << reverseWire[path] << " drops=" << wanDrops[path] << '\n';
  std::cout << "DCI_RESULT success=" << success << " flows=" << count << " paths=" << paths
    << " policy=" << policy << " early_ack=0 outer_retransmission=0" << '\n';
  Simulator::Destroy();
  return success ? 0 : 1;
}
