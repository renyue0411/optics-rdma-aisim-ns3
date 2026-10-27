// SPDX-License-Identifier: GPL-2.0-only
#ifndef DCI_TUNNEL_H
#define DCI_TUNNEL_H
#include "ns3/application.h"
#include "dci-path-manager.h"
#include "ns3/data-rate.h"
#include "ns3/event-id.h"
#include "ns3/ipv4-address.h"
#include "ns3/qbb-net-device.h"
#include "ns3/socket.h"
#include <deque>
#include <map>
#include <tuple>
#include <vector>

namespace ns3 {
// The gateway port intercepts transit packets before Qbb's host-address check.
class DciAccessDevice : public QbbNetDevice
{
public:
  static TypeId GetTypeId();
  Callback<void, Ptr<Packet>> ingress;
  void Receive(Ptr<Packet> packet) override;
};

class DciTunnelGateway : public Application
{
public:
  static TypeId GetTypeId();
  // Path choice happens once per canonical QP key, not once per packet.
  enum Policy { ROUND_ROBIN = 0, HASH = 1, MIN_RTT = 2 };
  using Key = std::tuple<uint32_t, uint32_t, uint16_t, uint16_t, uint16_t>;
  struct Flow
  {
    uint32_t path = 0;
    uint64_t packets = 0, bytes = 0;
    std::deque<Ptr<Packet>> queue;
  };
  struct Control
  {
    DciTunnelHeader header;
    Time queuedAt;
  };
  struct Path
  {
    Ptr<NetDevice> device;
    Ptr<Socket> socket;
    Ipv4Address local, peer;
    DataRate rate;
    EventId event;
    std::deque<Control> control;
    uint64_t controlTx = 0, controlRx = 0, controlDrops = 0;
    std::deque<Key> active;
    uint64_t queuedBytes = 0, txPackets = 0, rxPackets = 0;
    uint64_t txInnerBytes = 0, rxInnerBytes = 0, markedPackets = 0;
  };
  void Configure(Ptr<DciAccessDevice> access, Policy policy, uint64_t bufferBytes,
                 bool ecn, uint64_t ecnThresholdBytes);
  void AddPath(Ptr<NetDevice> device, Ipv4Address local, Ipv4Address peer,
               DataRate rate);
  void ConfigurePaths(const DciPathManager::Settings& settings);
  void Ingress(Ptr<Packet> packet);
  void WritePathCsv(std::ostream& os) const;
  void PrintStats() const;
  bool HasProtocolErrors() const { return m_invalid != 0 || m_sendErrors != 0; }
  static bool ReadKey(Ptr<const Packet> packet, Key& key, uint8_t& tos);
  // Helpers modify only the inner IPv4 ECN bits, preserving other headers.
  static uint8_t GetInnerTos(Ptr<const Packet> packet);
  static void SetInnerEcn(Ptr<Packet> packet, uint8_t ecn);
private:
  void StartApplication() override;
  void StopApplication() override;
  void Receive(Ptr<Socket> socket);
  void SendControl(uint32_t id, DciTunnelHeader header);
  void Kick(uint32_t id);
  void Drain(uint32_t path);
  uint32_t Choose(const Key& key);
  Ptr<DciPathManager> m_manager;
  Ptr<DciAccessDevice> m_access;
  Policy m_policy = ROUND_ROBIN;
  bool m_running = false, m_ecn = true;
  uint32_t m_cursor = 0;
  uint64_t m_limit = 0, m_threshold = 0, m_queued = 0, m_peak = 0;
  uint64_t m_ingressDrops = 0, m_egressDrops = 0, m_invalid = 0, m_sendErrors = 0;
  std::map<Key, Flow> m_flows;
  std::vector<Path> m_paths;
};
} // namespace ns3
#endif
