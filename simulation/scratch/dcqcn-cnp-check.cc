// Targeted NP/RP acceptance: controlled DATA observations, real Qbb return packets.
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rdma-hw.h"
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>
using namespace ns3;
namespace {
void Need(bool ok, const char* why) { NS_ABORT_MSG_IF(!ok, why); }
void Noop(Ptr<RdmaQueuePair>) {}
struct ReceiveProgress {
  uint64_t calls = 0;
  std::map<uint16_t,uint64_t> prefixes;
  void Observe(Ptr<RdmaRxQueuePair> q, uint64_t prefix) {
    Need(prefix == q->ReceiverNextExpectedSeq && prefix > prefixes[q->dport],
         "RX progress is not a strictly advancing local contiguous prefix");
    prefixes[q->dport] = prefix; ++calls;
  }
};
struct ReturnWire {
  Ptr<RdmaHw> sender;
  uint64_t cnps = 0, acks = 0, flags = 0, bytes = 0, enqueued = 0;
  std::vector<int64_t> txTimes;
  std::vector<uint16_t> ports;
  std::vector<int64_t> enqueueTimes;
  std::vector<uint16_t> enqueuePorts;
  void Enqueue(Ptr<const Packet> p, uint32_t queue) {
    CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
    ch.brief = 0; p->PeekHeader(ch);
    if (ch.l3Prot == 0xFF) {
      Need(queue == 0, "CNP must use the RNIC high priority return queue");
      ++enqueued;
      enqueueTimes.push_back(Simulator::Now().GetNanoSeconds());
      enqueuePorts.push_back(ch.cnp.fid);
    }
  }
  void Tx(Ptr<const Packet> p) {
    CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
    ch.brief = 0; p->PeekHeader(ch);
    if (ch.l3Prot == 0xFF) {
      bytes += p->GetSize();
      txTimes.push_back(Simulator::Now().GetNanoSeconds());
      ports.push_back(ch.cnp.fid);
      Need(p->GetSize() == 64, "wrong modeled CNP frame size");
    }
  }
  int Rx(Ptr<QbbNetDevice>, Ptr<Packet> p, CustomHeader& ch) {
    if (ch.l3Prot == 0xFF) {
      ++cnps;
      return sender->ReceiveCnp(p, ch);
    }
    if (ch.l3Prot == 0xFC || ch.l3Prot == 0xFD) {
      ++acks;
      if (ch.ack.flags & 1) ++flags;
      // DATA is controlled injection, not sent by the sender's queue; do not ACK
      // fake sent ranges. The actual collective exercises full ACK/SACK handling.
    }
    return 0;
  }
};
}
int main(int argc, char** argv) {
  std::string scenario = "burst";
  uint32_t mode = 1, intervalUs = 50, delayUs = 5;
  CommandLine cmd;
  cmd.AddValue("scenario", "burst, ect, ooo, two_qps, pg1, cancel, late", scenario);
  cmd.AddValue("mode", "0 legacy, 1 independent", mode);
  cmd.AddValue("intervalUs", "NP CNP spacing", intervalUs);
  cmd.AddValue("delayUs", "Actual return-link propagation delay", delayUs);
  cmd.Parse(argc, argv);
  Need(mode <= 1 && intervalUs >= 2 && intervalUs <= 1000 && delayUs >= 1 && delayUs <= 1000,
       "invalid fixture option");
  Need(scenario == "burst" || scenario == "ect" || scenario == "ooo" ||
       scenario == "two_qps" || scenario == "pg1" || scenario == "cancel" ||
       scenario == "late", "unknown scenario");
  setenv("SIMAI_CNP_MODE", std::to_string(mode).c_str(), 1);
  setenv("SIMAI_CNP_INTERVAL_US", std::to_string(intervalUs).c_str(), 1);
  (void)RdmaHw::GetTypeId();
  Config::SetDefault("ns3::RdmaHw::CcMode", UintegerValue(1));
  Config::SetDefault("ns3::RdmaHw::L2BackToZero", BooleanValue(false));
  NodeContainer nodes; nodes.Create(2);
  InternetStackHelper internet; internet.Install(nodes);
  QbbHelper link;
  link.SetDeviceAttribute("DataRate", StringValue("100Gbps"));
  link.SetChannelAttribute("Delay", TimeValue(MicroSeconds(delayUs)));
  auto devs = link.Install(nodes);
  std::vector<Ptr<RdmaHw>> hw;
  for (uint32_t i = 0; i < 2; ++i) {
    auto h = CreateObject<RdmaHw>();
    h->SetNode(nodes.Get(i));
    for (uint32_t j = 0; j < nodes.Get(i)->GetNDevices(); ++j) {
      auto d = DynamicCast<QbbNetDevice>(nodes.Get(i)->GetDevice(j));
      h->m_nic.push_back(RdmaInterfaceMgr(d));
      h->m_nic.back().qpGrp = CreateObject<RdmaQueuePairGroup>();
    }
    h->Setup(MakeCallback(&Noop), MakeCallback(&Noop));
    h->ConfigureIrn(1000000, 100000000, 200000000);
    hw.push_back(h);
  }
  const Ipv4Address source("10.0.0.1"), sink("10.0.1.1");
  // Exercise actual RNIC DATA construction, without registering/sending these QPs.
  const uint32_t savedCcMode = hw[0]->m_cc_mode;
  for (uint32_t cc : {0U, 1U}) for (uint32_t nvls : {0U, 1U}) {
    hw[0]->m_cc_mode = cc;
    auto q = CreateObject<RdmaQueuePair>(3, source, sink, 20000, 100);
    q->SetSize(2000); q->SetPostedLimit(2000); q->nvls_enable = nvls;
    auto packet = hw[0]->GetNxtPacket(q);
    CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
    Need(packet->PeekHeader(ch) != 0, "cannot parse RNIC-generated DATA");
    Need(ch.GetIpv4EcnBits() == (cc == 1 ? 2 : 0), "wrong source ECN capability");
    Need((ch.m_tos & 0xfc) == (nvls ? 4 : 0), "source ECN changed NVLS DSCP");
    Need(ch.udp.seq == 0 && ch.udp.pg == 3 && ch.udp.sport == 20000,
         "source ECN changed RDMA DATA fields");
  }
  hw[0]->m_cc_mode = savedCcMode;
  std::cout << "SOURCE_ECN_CHECK_OK cc_modes=0,1 nvls=0,1 data_packets=4\n";
  auto senderDev = DynamicCast<QbbNetDevice>(devs.Get(0));
  auto rxDev = DynamicCast<QbbNetDevice>(devs.Get(1));
  hw[1]->m_rtTable[source.Get()].push_back(rxDev->GetIfIndex());
  hw[0]->m_rtTable[sink.Get()].push_back(senderDev->GetIfIndex());
  ReturnWire wire; wire.sender = hw[0];
  ReceiveProgress receiveProgress;
  hw[1]->SetRxProgressCallback(MakeCallback(&ReceiveProgress::Observe, &receiveProgress));
  senderDev->m_rdmaReceiveCb = MakeCallback(&ReturnWire::Rx, &wire);
  Need(rxDev->TraceConnectWithoutContext("PhyTxBegin", MakeCallback(&ReturnWire::Tx, &wire)),
       "missing actual wire TX trace");
  Need(rxDev->TraceConnectWithoutContext("QbbEnqueue", MakeCallback(&ReturnWire::Enqueue, &wire)),
       "missing queue trace");
  uint16_t pg = scenario == "pg1" ? 1 : 3;
  for (uint16_t port : {10000,10001}) {
    auto q = CreateObject<RdmaQueuePair>(pg, source, sink, port, 100);
    q->SetSize(1ULL << 40); q->SetPostedLimit(0);
    q->m_rate = DataRate("100Gbps"); q->m_max_rate = q->m_rate;
    q->mlx.m_targetRate = q->m_rate;
    const uint64_t key = hw[0]->GetQpKey(sink.Get(), port, pg);
    hw[0]->m_qpMap[key] = q;
  }
  auto observe = [=](uint64_t whenUs, uint64_t seq, uint8_t ecn, uint16_t port) {
    Simulator::Schedule(MicroSeconds(whenUs), [=]() {
      CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
      ch.pppProto = 0x0021; ch.l3Prot = 0x11;
      ch.sip = source.Get(); ch.dip = sink.Get(); ch.m_tos = ecn;
      ch.udp.sport = port; ch.udp.dport = 100; ch.udp.pg = pg;
      ch.udp.seq = seq; ch.udp.payload_size = 1000;
      auto p = Create<Packet>(1000); p->AddHeader(ch);
      hw[1]->ReceiveUdp(rxDev, p, ch);
    });
  };
  if (scenario == "ect") {
    observe(10, 0, 1, 10000); observe(11, 1000, 2, 10000);
  } else if (scenario == "ooo") {
    observe(10, 1000, 0, 10000); observe(11, 0, 0, 10000);
  } else if (scenario == "two_qps") {
    observe(10, 0, 3, 10000); observe(10, 0, 3, 10001);
    observe(11, 1000, 3, 10000); observe(11, 1000, 3, 10001);
  } else if (scenario == "cancel") {
    observe(10, 0, 3, 10000); observe(11, 1000, 3, 10000);
    Simulator::Schedule(MicroSeconds(12), [=]() { hw[1]->DeleteRxQp(source.Get(), pg, 10000); });
  } else if (scenario == "late") {
    observe(10, 0, 3, 10000);
    // Erase the QP after local CNP creation, before it crosses the return link.
    Simulator::Schedule(MicroSeconds(10) + NanoSeconds(1), [=]() {
      hw[0]->m_qpMap.erase(hw[0]->GetQpKey(sink.Get(), 10000, pg));
    });
  } else {
    observe(10, 0, 3, 10000); observe(11, 1000, 3, 10000);
    observe(12, 2000, 3, 10000);
    observe(11 + intervalUs, 3000, 3, 10000);
  }
  Simulator::Stop(MicroSeconds(100 + intervalUs * 3 + delayUs));
  Simulator::Run();
  Need(receiveProgress.calls > 0, "receiver-local progress hook never fired");
  if (scenario == "ooo") Need(receiveProgress.calls == 1,
                              "out-of-order arrival incorrectly advanced RX prefix");
  std::cout << "RX_PROGRESS_CHECK_OK callbacks=" << receiveProgress.calls
            << " contiguous_only=1 local_receiver_state=1\n";
  const bool silent = scenario == "ect" || scenario == "ooo";
  const uint64_t expected = !mode || silent ? 0 : scenario == "two_qps" ? 4 :
    (scenario == "cancel" || scenario == "late") ? 1 : 3;
  Need(wire.cnps == expected && hw[1]->m_cnpTx == expected &&
       wire.txTimes.size() == expected && wire.enqueued == expected &&
       wire.bytes == expected * 64, "CNP enqueue/actual wire/receive counts differ");
  Need(!mode || wire.flags == 0, "independent mode duplicated feedback on ACK/NACK");
  if (silent) Need(wire.flags == 0, "ECT or unmarked DATA set an ACK congestion flag");
  Need(wire.acks > 0, "reliability ACK/SACK traffic disappeared");
  if (mode && scenario != "late") Need(hw[0]->m_cnpRx == expected, "CNP did not enter RP");
  if (mode && scenario == "late") Need(hw[0]->m_cnpUnknownRx == 1, "late CNP not safely ignored");
  std::map<uint16_t,int64_t> last;
  for (uint32_t i = 0; i < wire.enqueueTimes.size(); ++i) {
    auto found = last.find(wire.enqueuePorts[i]);
    if (found != last.end())
      Need(wire.enqueueTimes[i] - found->second >= intervalUs * 1000,
           "per-QP NP generation spacing violated at Qbb enqueue");
    last[wire.enqueuePorts[i]] = wire.enqueueTimes[i];
  }
  if (mode && expected && scenario != "late")
    Need(hw[0]->m_cnpRateDecreases > 0, "independent CNP did not reduce QP rate");
  if (silent) Need(hw[0]->m_cnpRateDecreases == 0, "ECT or pure reordering triggered DCQCN");
  if (!mode && !silent) Need(wire.flags > 0, "legacy ACK feedback changed");
  for (auto h : hw) { h->PrintCnpFeedbackStats(); h->Dispose(); }
  std::cout << "CHECK_OK scenario=" << scenario << " mode=" << mode
            << " interval_us=" << intervalUs << " delay_us=" << delayUs
            << " actual_cnp_packets=" << wire.cnps << " actual_ack_packets=" << wire.acks
            << " ack_feedback_flags=" << wire.flags
            << " actual_qbb_tx_verified=1 ce_injection_at_receiver=1\n";
  Simulator::Destroy();
}
