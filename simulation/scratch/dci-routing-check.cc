// Exercise the production selector with real ns-3 nodes, devices and RNGs.
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "../src/point-to-point/model/dci-spray.h"
#include <array>
#include <iostream>
#include <stdexcept>
using namespace ns3;
void Check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
int main(int argc, char **argv) {
  const std::string mode = argc > 1 ? argv[1] : "spray_rr";
  const int count = argc > 2 ? std::stoi(argv[2]) : 2;
  const std::string variant = argc > 3 ? argv[3] : "normal";
  NodeContainer padding;
  if (variant == "legacy") padding.Create(6);
  auto router = CreateObject<SwitchNode>();
  auto peer = CreateObject<Node>();
  NodeContainer nodes;
  nodes.Add(router);
  nodes.Add(peer);
  PointToPointHelper helper;
  std::vector<int> hops;
  for (int i = 0; i < count * 2; ++i) {
    auto devices = helper.Install(nodes);
    if (i % 2 == 0) hops.push_back(devices.Get(0)->GetIfIndex());
  }
  const int ecmp = hops.back();
  if (variant != "legacy")
    SwitchNode::ConfigureDciRouting(mode, variant == "excluded" ? "999" : "0");
  if (variant == "duplicate") hops.push_back(hops.front());
  if (variant == "empty") hops.clear();
  CustomHeader ch{};
  ch.l3Prot = 0x11; ch.sip = 1; ch.dip = 2;
  ch.udp.sport = 10000; ch.udp.dport = 20000; ch.udp.pg = 3;
  std::vector<int> samples(count, 0);
  uint64_t digest = 0;
  const int total = 20000;
  for (int i = 0; i < total; ++i) {
    // Interleaved QPs must not alias with a global round-robin counter.
    for (int qp = 0; qp < 8; ++qp) {
      ch.udp.sport = 10000 + qp;
      const int port = DciSprayV1::Choose(PeekPointer(router), ch, hops, ecmp, 4096);
      auto pos = std::find(hops.begin(), hops.end(), port);
      Check(pos != hops.end(), "selected a non-candidate port");
      if (variant == "excluded" || mode == "ecmp") Check(port == ecmp, "ECMP/scope changed");
      else if (mode == "spray_rr") Check(port == hops[(count - 1 + i) % count], "QP rotation incorrect");
      if (qp == 0) { samples[pos - hops.begin()]++; digest = digest * 131 + port; }
    }
  }
  // Control packets must bypass the selector without changing DATA statistics.
  const auto before = DciSprayV1::Get().routers.size();
  ch.l3Prot = 0xFC;
  Check(DciSprayV1::Choose(PeekPointer(router), ch, hops, ecmp, 64) == ecmp, "control path changed");
  Check(DciSprayV1::Get().routers.size() == before, "control packet entered telemetry");
  if (variant != "excluded") {
    uint64_t packets = 0;
    for (const auto &p : DciSprayV1::Get().routers.at(router->GetId()).ports) packets += p.second.packets;
    Check(packets == total * 8, "DATA packet accounting mismatch");
    if (mode == "qp_fixed") {
      std::vector<std::vector<int>> assigned(2, std::vector<int>(count, 0));
      const auto &flows = DciSprayV1::Get().routers.at(router->GetId()).flowPorts;
      Check(flows.size() == 8, "missing fixed QPs");
      for (const auto &flow : flows) {
        Check(flow.second.size() == 1, "QP DATA used multiple paths");
        const int channel = (std::get<2>(flow.first) - 10000) / 4;
        const auto pos = std::find(hops.begin(), hops.end(), *flow.second.begin());
        Check(pos != hops.end(), "fixed path is not a candidate");
        ++assigned.at(channel).at(pos - hops.begin());
      }
      if (4 % count == 0) for (const auto &channel : assigned)
        for (int n : channel)
          Check(n == 4 / count, "channel QPs are not evenly assigned");
    }
    if (mode == "spray_random") for (auto n : samples)
      Check(n > total / count * 0.85 && n < total / count * 1.15, "random allocation is biased");
    // Route candidate changes must be revalidated and never select a stale port.
    ch.l3Prot = 0x11;
    std::vector<int> single{hops.front()};
    Check(DciSprayV1::Choose(PeekPointer(router), ch, single, single.front(), 4096) == single.front(), "single path failed");
  }
  std::cout << "CHECK_OK mode=" << mode << " candidates=" << count << " digest=" << digest << std::endl;
  Simulator::Destroy();
}
