// SPDX-License-Identifier: GPL-2.0-only
// DCI_SPRAY_V1: DATA-only round-robin over the four direct 6<->7 WAN links.
#ifndef SIMAI_DCI_SPRAY_V1_H
#define SIMAI_DCI_SPRAY_V1_H
#include "switch-node.h"
#include "ns3/abort.h"
#include "ns3/channel.h"
#include "ns3/simulator.h"
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace ns3 {
namespace DciSprayV1 {
using Flow = std::tuple<uint32_t, uint32_t, uint16_t, uint16_t, uint16_t>;
struct PortStats { uint64_t packets = 0; uint64_t bytes = 0; };
struct Router {
  bool configured = false;
  std::set<uint32_t> checkedDestinations;
  std::map<Flow, uint64_t> next;
  std::map<int, PortStats> ports;
  std::map<Flow, std::set<int>> flowPorts;
};
struct State {
  std::string mode = "ecmp";
  bool enabled = false;
  bool printed = false;
  std::map<uint32_t, Router> routers;
  State() {
    enabled = std::getenv("SIMAI_DCI_ROUTING") != nullptr ||
              std::getenv("SIMAI_DCI_EXPERIMENT_MS") != nullptr;
    if (const char *value = std::getenv("SIMAI_DCI_ROUTING")) mode = value;
    NS_ABORT_MSG_IF(mode != "ecmp" && mode != "spray_rr",
                    "SIMAI_DCI_ROUTING must be ecmp or spray_rr");
  }
};
inline State &Get() { static State state; return state; }

inline int Choose(SwitchNode *node, const CustomHeader &ch,
                  const std::vector<int> &hops, int ecmpPort, uint32_t bytes) {
  const uint32_t id = node->GetId();
  if ((id != 6 && id != 7) || ch.l3Prot != 0x11 || hops.size() <= 1)
    return ecmpPort;
  State &state = Get();
  if (!state.enabled) return ecmpPort;
  Router &router = state.routers[id];
  if (router.checkedDestinations.insert(ch.dip).second) {
    std::vector<int> sorted(hops);
    std::sort(sorted.begin(), sorted.end());
    const std::vector<int> expected{3, 4, 5, 6};
    NS_ABORT_MSG_IF(sorted != expected,
                    "DCI spray benchmark requires four distinct WAN ports 3,4,5,6");
    const uint32_t peer = id == 6 ? 7 : 6;
    for (int port : sorted) {
      NS_ABORT_MSG_IF(static_cast<uint32_t>(port) >= node->GetNDevices(), "Invalid WAN interface");
      Ptr<Channel> channel = node->GetDevice(port)->GetChannel();
      NS_ABORT_MSG_IF(channel == nullptr || channel->GetNDevices() != 2,
                      "WAN interface must be a point-to-point channel");
      bool foundPeer = false;
      for (std::size_t i = 0; i < channel->GetNDevices(); ++i) {
        if (channel->GetDevice(i)->GetNode()->GetId() == peer) foundPeer = true;
      }
      NS_ABORT_MSG_IF(!foundPeer, "WAN interface does not connect routers 6 and 7");
    }
  }
  if (!router.configured) {
    router.configured = true;
    for (int port = 3; port <= 6; ++port) router.ports[port];
    std::cout << "DCI_ROUTING_CONFIG node=" << id << " mode=" << state.mode
              << " peer=" << (id == 6 ? 7 : 6)
              << " ports=3,4,5,6 data_only=1 control=ecmp rr_scope=qp initial_phase=ecmp"
              << std::endl;
  }
  const Flow flow(ch.sip, ch.dip, ch.udp.sport, ch.udp.dport, ch.udp.pg);
  int selected = ecmpPort;
  if (state.mode == "spray_rr") {
    // Each QP cycles independently. A shared destination counter could alias
    // with an equally sized round-robin QP schedule and keep each QP on one
    // port. ECMP only seeds the initial phase; every following packet rotates.
    auto counter = router.next.emplace(flow, static_cast<uint64_t>(ecmpPort - 3));
    selected = 3 + static_cast<int>(counter.first->second++ % 4);
  }
  router.ports[selected].packets++;
  router.ports[selected].bytes += bytes;
  router.flowPorts[flow].insert(selected);
  return selected;
}

inline void Print() {
  State &state = Get();
  if (state.printed) return;
  state.printed = true;
  for (const auto &entry : state.routers) {
    const Router &r = entry.second;
    uint64_t packets = 0, bytes = 0, single = 0, multi = 0;
    for (const auto &port : r.ports) {
      packets += port.second.packets;
      bytes += port.second.bytes;
    }
    for (const auto &flow : r.flowPorts) {
      if (flow.second.size() == 1) ++single;
      else if (flow.second.size() > 1) ++multi;
    }
    std::cout << "DCI_ROUTING_STATS node=" << entry.first << " mode=" << state.mode
              << " selected_packets=" << packets << " selected_bytes=" << bytes
              << " wan_flows=" << r.flowPorts.size()
              << " single_path_flows=" << single << " multipath_flows=" << multi
              << " time_ns=" << Simulator::Now().GetNanoSeconds() << std::endl;
    for (const auto &port : r.ports) {
      std::cout << "DCI_ROUTING_PORT node=" << entry.first << " port=" << port.first
                << " selected_packets=" << port.second.packets
                << " selected_bytes=" << port.second.bytes << std::endl;
    }
  }
}
} // namespace DciSprayV1
} // namespace ns3
#endif
