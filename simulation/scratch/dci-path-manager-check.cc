// SPDX-License-Identifier: GPL-2.0-only
// Standalone acceptance experiment: real Qbb queues/serialization/propagation.
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
using namespace ns3;
namespace {
void Require(bool ok, const std::string& text) {
  if (!ok) throw std::runtime_error(text);
}
struct WireCounts {
  uint64_t tx = 0, bytes = 0, dequeued = 0;
  void Tx(Ptr<const Packet> p) { ++tx; bytes += p->GetSize(); }
  void Dequeue(Ptr<const Packet> p, uint32_t q) {
    Require(q == 0, "probe did not use the high-priority queue"); ++dequeued;
  }
};
// Regression guard: existing DATA wire fields including 64-bit sequence survive.
void CheckDataHeader() {
  CustomHeader a(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
  a.pppProto = 0x0021; a.l3Prot = 0x11; a.sip = 1; a.dip = 2;
  a.udp.sport = 10000; a.udp.dport = 20000;
  a.udp.pg = 3; a.udp.seq = (uint64_t(1) << 40) + 17; a.udp.payload_size = 100;
  auto p = Create<Packet>(100); p->AddHeader(a);
  CustomHeader b(a.headerType); b.brief = 0;
  Require(p->PeekHeader(b) == a.GetSerializedSize(), "DATA header size changed");
  Require(!b.IsDciPathControl() && b.udp.seq == a.udp.seq && b.udp.pg == 3 &&
    b.udp.sport == 10000 && b.udp.dport == 20000, "DATA header regression");
}
}
int RunCheck(int argc, char** argv) {
  uint32_t paths = 2, peers = 1, timeoutUs = 10000, intervalUs = 1000;
  uint32_t serviceUs = 5, ccMode = 1;
  bool logPackets = false;
  std::string scenario = "normal", rate = "400Gbps";
  CommandLine cmd;
  cmd.AddValue("paths", "WAN candidates per remote gateway (1..8 in this fixture)", paths);
  cmd.AddValue("peers", "Remote gateways (1..8)", peers);
  cmd.AddValue("scenario", "normal, failure, late, skew, restart", scenario);
  cmd.AddValue("rate", "Actual Qbb link data rate", rate);
  cmd.AddValue("intervalUs", "Probe tick interval in microseconds", intervalUs);
  cmd.AddValue("timeoutUs", "Fixed timeout in microseconds", timeoutUs);
  cmd.AddValue("serviceUs", "Fixed processing delay at each end", serviceUs);
  cmd.AddValue("ccMode", "Existing switch CC mode, also exercises INT bypass", ccMode);
  cmd.AddValue("logPackets", "Log each accepted control transmission", logPackets);
  cmd.Parse(argc, argv);
  Require(paths >= 1 && paths <= 8 && peers >= 1 && peers <= 8, "fixture dimensions out of range");
  Require(scenario == "normal" || scenario == "failure" || scenario == "late" ||
    scenario == "skew" || scenario == "restart", "unknown scenario");
  Require(intervalUs && timeoutUs && serviceUs <= 100, "invalid fixture timers");
  CheckDataHeader();
  auto root = CreateObject<SwitchNode>();
  std::vector<Ptr<SwitchNode>> remotes;
  for (uint32_t j = 0; j < peers; ++j) remotes.push_back(CreateObject<SwitchNode>());
  std::vector<Ptr<DciPathManagerApp>> apps;
  std::vector<WireCounts> counts(peers + 1);
  DciPathManagerApp::Settings settings;
  settings.probeInterval = MicroSeconds(intervalUs); settings.timeout = MicroSeconds(timeoutUs);
  settings.processingDelay = MicroSeconds(serviceUs); settings.logPackets = logPackets;
  auto install = [&](Ptr<SwitchNode> node, Ipv4Address controlIp) {
    node->SetAttribute("CcMode", UintegerValue(ccMode));
    InternetStackHelper internet; internet.Install(node);
    node->GetObject<Ipv4>()->AddAddress(0, Ipv4InterfaceAddress(controlIp, Ipv4Mask("255.255.255.255")));
    // Loopback plus a padding device: WAN indices are not hard-coded 0/1.
    PointToPointHelper padding; padding.Install(node, CreateObject<Node>());
    auto app = CreateObject<DciPathManagerApp>();
    node->AddApplication(app); app->Configure(settings);
    app->SetStartTime(Seconds(0)); app->SetStopTime(MilliSeconds(61)); apps.push_back(app);
    return app;
  };
  Ipv4Address rootIp("10.255.0.1");
  auto rootApp = install(root, rootIp);
  for (uint32_t j = 0; j < peers; ++j)
    install(remotes[j], Ipv4Address((std::string("10.255.") + std::to_string(j + 1) + ".2").c_str()));
  std::vector<Ptr<RateErrorModel>> errors;
  const DataRate linkRate(rate);
  const uint32_t frameBytes = PppHeader::GetStaticSize() + 20 + 8 + DciPathControlHeader::SIZE;
  std::vector<Time> expectedRtts;
  for (uint32_t j = 0; j < peers; ++j) {
    Ipv4Address peerIp((std::string("10.255.") + std::to_string(j + 1) + ".2").c_str());
    std::set<uint32_t> firstOnly;
    for (uint32_t i = 0; i < paths; ++i) {
      uint32_t delayUs = 245 + 500 * i;
      if (scenario == "late" && j == 0 && i == 0) delayUs = timeoutUs + 245;
      QbbHelper helper;
      helper.SetDeviceAttribute("DataRate", DataRateValue(linkRate));
      helper.SetChannelAttribute("Delay", TimeValue(MicroSeconds(delayUs)));
      auto devices = helper.Install(root, remotes[j]);
      root->AddTableEntry(peerIp, devices.Get(0)->GetIfIndex());
      if (!i) firstOnly.insert(devices.Get(0)->GetIfIndex());
      remotes[j]->AddTableEntry(rootIp, devices.Get(1)->GetIfIndex());
      // Duplicate route entries must not duplicate managed candidates.
      if (!i) root->AddTableEntry(peerIp, devices.Get(0)->GetIfIndex());
      for (uint32_t end = 0; end < 2; ++end) {
        auto device = DynamicCast<QbbNetDevice>(devices.Get(end));
        auto& trace = counts[end ? j + 1 : 0];
        Require(device->TraceConnectWithoutContext("PhyTxBegin", MakeCallback(&WireCounts::Tx, &trace)), "no TX trace");
        Require(device->TraceConnectWithoutContext("QbbDequeue", MakeCallback(&WireCounts::Dequeue, &trace)), "no queue trace");
        if (scenario == "failure" && j == 0 && i == 0) {
          auto error = CreateObject<RateErrorModel>();
          error->SetUnit(RateErrorModel::ERROR_UNIT_PACKET); error->SetRate(0);
          device->SetAttribute("ReceiveErrorModel", PointerValue(error)); errors.push_back(error);
        }
      }
      expectedRtts.push_back(MicroSeconds(2 * delayUs + 2 * serviceUs) +
        linkRate.CalculateBytesTxTime(frameBytes) * 2);
    }
    // The manager reads routes; it never queries the peer's node or channel.
    Require(rootApp->AddPeer(rootIp, peerIp, firstOnly) == 1, "WAN allow-list failed");
    Require(rootApp->AddPeer(rootIp, peerIp) == paths - 1, "wrong candidate count");
    Require(rootApp->AddPeer(rootIp, peerIp) == 0, "peer registration not idempotent");
    Require(apps[j + 1]->AddPeer(peerIp, rootIp) == paths, "wrong reverse candidate count");
  }
  Require(rootApp->AddPeer(rootIp, Ipv4Address("192.0.2.99")) == 0, "invented an unrouted candidate");
  if (scenario == "skew")
    for (uint32_t j = 1; j < apps.size(); ++j) apps[j]->SetStartTime(MilliSeconds(5));
  auto checkUp = [=]() {
    auto views = rootApp->GetSnapshot();
    Require(views.size() == peers * paths, "lost a candidate");
    for (uint32_t i = 0; i < views.size(); ++i) {
      if (scenario == "late" && i == 0) continue;
      Require(views[i].IsUsable(), "candidate not UP at acceptance checkpoint");
      Require(views[i].latestRtt == expectedRtts[i], "RTT does not match real link serialization/propagation/service delay");
    }
  };
  Simulator::Schedule(MilliSeconds(18), checkUp);
  if (scenario == "failure") {
    Simulator::Schedule(MilliSeconds(20), [=]() {
      std::cout << "DCI_PM_FAULT time_ms=20 action=blackhole link_stays_up=1\n";
      for (auto error : errors) error->SetRate(1);
    });
    Simulator::Schedule(MilliSeconds(34), [=]() {
      const auto views = rootApp->GetSnapshot();
      Require(views[0].status == DciPathManagerApp::DOWN, "blackholed path was not detected");
      for (uint32_t i = 1; i < views.size(); ++i) Require(views[i].IsUsable(), "unrelated path went down");
      std::cout << "CHECK_DOWN_OK time_ms=34\n";
    });
    Simulator::Schedule(MilliSeconds(40), [=]() {
      for (auto error : errors) error->SetRate(0);
      std::cout << "DCI_PM_FAULT time_ms=40 action=restore\n";
    });
  }
  if (scenario == "restart") {
    Simulator::Schedule(MilliSeconds(20), [=]() { apps[1]->StopService(); });
    Simulator::Schedule(MilliSeconds(34), [=]() {
      auto views = rootApp->GetSnapshot();
      for (uint32_t i = 0; i < paths; ++i)
        Require(views[i].status == DciPathManagerApp::DOWN, "stopped peer was not detected");
      std::cout << "CHECK_PEER_DOWN_OK time_ms=34\n";
    });
    Simulator::Schedule(MilliSeconds(40), [=]() { apps[1]->StartService(); });
  }
  Simulator::Schedule(MilliSeconds(55), checkUp);
  bool passed = false;
  Simulator::Schedule(MilliSeconds(60), [&]() {
    if (scenario == "late") {
      const auto bad = rootApp->GetSnapshot()[0];
      Require(bad.status == DciPathManagerApp::DOWN && !bad.samples && bad.staleReplies,
        "late replies incorrectly validated a timed-out path");
    }
    for (uint32_t i = 0; i < apps.size(); ++i) {
      apps[i]->PrintStats();
      uint64_t tx = 0, bytes = 0;
      for (const auto& view : apps[i]->GetSnapshot()) {
        tx += view.txPackets; bytes += view.txBytes;
        Require(view.txBytes == view.txPackets * frameBytes, "control byte accounting incorrect");
      }
      Require(tx == counts[i].tx && tx == counts[i].dequeued && bytes == counts[i].bytes,
        "accepted controls did not traverse the Qbb q0/device TX path");
      Require(apps[i]->GetPeakOutstanding() <= apps[i]->GetSnapshot().size(), "unbounded outstanding requests");
    }
    passed = true;
  });
  Simulator::Stop(MilliSeconds(60) + NanoSeconds(1));
  Simulator::Run(); Require(passed, "acceptance did not complete"); Simulator::Destroy();
  std::cout << "CHECK_OK scenario=" << scenario << " paths_per_peer=" << paths
    << " peers=" << peers << " root_candidates=" << paths * peers
    << " frame_bytes=" << frameBytes << " actual_qbb_tx_verified=1 high_priority_verified=1\n";
  return 0;
}
int main(int argc, char** argv) {
  try { return RunCheck(argc, argv); }
  catch (const std::exception& e) {
    std::cerr << "CHECK_FAILED " << e.what() << '\n';
    Simulator::Destroy(); return 1;
  }
}
