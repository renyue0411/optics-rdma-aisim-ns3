// SPDX-License-Identifier: GPL-2.0-only
// DATA-only DCI routing over the destination routing table candidate ports.
#ifndef SIMAI_DCI_SPRAY_V1_H
#define SIMAI_DCI_SPRAY_V1_H
#include "switch-node.h"
#include "ns3/abort.h"
#include "ns3/channel.h"
#include "ns3/simulator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include <limits>
#include <sstream>
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
  std::set<std::vector<int>> checkedPorts;
  Ptr<UniformRandomVariable> random;
  std::map<Flow, uint64_t> next;
  std::map<int, PortStats> ports;
  std::map<Flow, std::set<int>> flowPorts;
};
struct State {
  std::string mode = "ecmp";
  bool enabled = false;
  bool printed = false;
  std::set<uint32_t> nodes{6, 7};
  std::map<uint32_t, Router> routers;
  State() {
    enabled = std::getenv("SIMAI_DCI_ROUTING") != nullptr ||
              std::getenv("SIMAI_DCI_EXPERIMENT_MS") != nullptr;
    if (const char *value = std::getenv("SIMAI_DCI_ROUTING")) mode = value;
    NS_ABORT_MSG_IF(mode != "ecmp" && mode != "spray_rr" && mode != "spray_random",
                    "SIMAI_DCI_ROUTING must be ecmp, spray_rr or spray_random");
  }
};
inline State &Get() { static State state; return state; }

// Config-file settings override the legacy environment mode. With no config
// keys the old SIMAI_DCI_ROUTING / SIMAI_DCI_EXPERIMENT_MS behavior is retained.
inline void Configure(const std::string &mode, const std::string &nodes) {
  State &state = Get();
  NS_ABORT_MSG_IF(!state.routers.empty(), "Configure DCI routing before traffic starts");
  NS_ABORT_MSG_IF(mode != "ecmp" && mode != "spray_rr" && mode != "spray_random",
                  "DCI_ROUTING_MODE must be ecmp, spray_rr or spray_random");
  std::set<uint32_t> selected;
  std::istringstream input(nodes);
  std::string token;
  NS_ABORT_MSG_IF(nodes.empty() || nodes.back() == ',', "Invalid DCI_ROUTING_NODES");
  while (std::getline(input, token, ',')) {
    NS_ABORT_MSG_IF(token.empty() || token.find_first_not_of("0123456789") != std::string::npos,
                    "DCI_ROUTING_NODES must be comma-separated distinct node IDs");
    uint64_t value = 0;
    for (char c : token) {
      value = value * 10 + (c - '0');
      NS_ABORT_MSG_IF(value > std::numeric_limits<uint32_t>::max(), "DCI node ID overflow");
    }
    NS_ABORT_MSG_IF(!selected.insert(static_cast<uint32_t>(value)).second,
                    "Duplicate DCI_ROUTING_NODES entry");
  }
  state.mode = mode;
  state.nodes = selected;
  state.enabled = true;
}

inline int Choose(SwitchNode *node, const CustomHeader &ch,
                  const std::vector<int> &hops, int ecmpPort, uint32_t bytes) {
  // ACK/NACK/CNP always retain the caller's existing ECMP selection.
  if (ch.l3Prot != 0x11) return ecmpPort;
  State &state = Get();
  const uint32_t id = node->GetId();
  if (!state.enabled || state.nodes.count(id) == 0) return ecmpPort;
  NS_ABORT_MSG_IF(hops.empty(), "DCI routing requires a nonempty candidate list");
  Router &router = state.routers[id];
  if (router.checkedPorts.insert(hops).second) {
    std::set<int> unique;
    for (int port : hops) {
      NS_ABORT_MSG_IF(port < 0 || static_cast<uint32_t>(port) >= node->GetNDevices(),
                      "Invalid DCI candidate interface");
      NS_ABORT_MSG_IF(!unique.insert(port).second, "Duplicate DCI candidate interface");
      Ptr<Channel> channel = node->GetDevice(port)->GetChannel();
      NS_ABORT_MSG_IF(channel == nullptr || channel->GetNDevices() != 2,
                      "DCI candidate must be a point-to-point interface");
      router.ports[port];
    }
    std::cout << "DCI_ROUTING_CONFIG node=" << id << " mode=" << state.mode << " ports=";
    for (size_t i = 0; i < hops.size(); ++i) std::cout << (i ? "," : "") << hops[i];
    std::cout << " candidate_count=" << hops.size()
              << " data_only=1 control=ecmp rr_scope=qp initial_phase=ecmp"
              << " rng_stream=" << (1700000LL + id)
              << " rng_seed=" << RngSeedManager::GetSeed()
              << " rng_run=" << RngSeedManager::GetRun() << std::endl;
  }
  const auto initial = std::find(hops.begin(), hops.end(), ecmpPort);
  NS_ABORT_MSG_IF(initial == hops.end(), "ECMP port is not a DCI candidate");
  const Flow flow(ch.sip, ch.dip, ch.udp.sport, ch.udp.dport, ch.udp.pg);
  int selected = ecmpPort;
  if (hops.size() > 1 && state.mode == "spray_rr") {
    auto counter = router.next.emplace(flow, static_cast<uint64_t>(initial - hops.begin()));
    selected = hops[counter.first->second % hops.size()];
    counter.first->second = (counter.first->second % hops.size() + 1) % hops.size();
  } else if (hops.size() > 1 && state.mode == "spray_random") {
    if (router.random == nullptr) {
      router.random = CreateObject<UniformRandomVariable>();
      // Stable per-router stream, independent of object creation order.
      router.random->SetStream(1700000LL + id);
    }
    selected = hops[router.random->GetInteger(0, static_cast<uint32_t>(hops.size() - 1))];
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
