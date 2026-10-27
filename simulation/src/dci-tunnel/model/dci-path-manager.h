// SPDX-License-Identifier: GPL-2.0-only
#ifndef DCI_PATH_MANAGER_H
#define DCI_PATH_MANAGER_H
#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/header.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rtt-estimator.h"
#include <cstdint>
#include <map>
#include <vector>

namespace ns3 {
// This is a private simulation wire format, not a QUIC frame format.
class DciTunnelHeader : public Header
{
public:
  enum Kind : uint8_t { DATA = 1, ACK = 2, CHALLENGE = 3, RESPONSE = 4 };
  static constexpr uint32_t SIZE = 32;
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override;
  uint32_t GetSerializedSize() const override;
  void Serialize(Buffer::Iterator i) const override;
  uint32_t Deserialize(Buffer::Iterator i) override;
  void Print(std::ostream& os) const override;
  bool IsValid() const { return m_valid; }
  Kind kind = DATA;
  uint16_t path = 0;
  uint64_t number = 0, value = 0, delayNs = 0;
private:
  bool m_valid = true;
};

class DciPathManager : public Object
{
public:
  enum Status { VALIDATING, ACTIVE, SUSPECT, FAILED };
  struct Settings
  {
    Time probeInterval = MilliSeconds(10);
    Time initialTimeout = MilliSeconds(200);
    Time maxAckDelay = MicroSeconds(50);
    uint32_t failureThreshold = 3, ackEvery = 16, sampleEvery = 64;
  };
  struct State
  {
    Status status = VALIDATING;
    bool validated = false, probePending = false, probeOnWire = false;
    uint32_t missed = 0, ackCount = 0;
    uint64_t nextData = 0, lastAcked = 0, nextProbe = 0, token = 0;
    uint64_t largestRx = 0, receiveMask = 0;
    Time probeSent, nextProbeAt, lastConfirmed, largestRxAt;
    Ptr<RttMeanDeviation> rtt;
    Time minRtt, latestRtt;
    std::map<uint64_t, Time> samples;
    EventId ackEvent;
    uint64_t dataTx = 0, dataRx = 0, ackTx = 0, ackRx = 0;
    uint64_t probeTx = 0, probeRx = 0, probeTimeouts = 0;
    uint64_t dataRttSamples = 0, probeRttSamples = 0;
    uint64_t expiredSamples = 0, evictedSamples = 0;
  };
  static TypeId GetTypeId();
  void Configure(const Settings& settings);
  void AddPath();
  void Start(uint32_t node, Callback<void, uint32_t, DciTunnelHeader> send,
             Callback<void, uint32_t> changed);
  void Stop();
  bool CanSend(uint32_t id) const;
  bool HasSample(uint32_t id) const;
  Time GetRtt(uint32_t id) const;
  DciTunnelHeader DataSent(uint32_t id, uint32_t innerSize);
  void ControlSent(uint32_t id, const DciTunnelHeader& header);
  void ControlDropped(uint32_t id, const DciTunnelHeader& header);
  void DataReceived(uint32_t id, const DciTunnelHeader& header);
  bool ControlReceived(uint32_t id, const DciTunnelHeader& header);
  void PrintStats() const;
  void WriteCsv(std::ostream& os) const;
private:
  static const char* Name(Status status);
  void Tick();
  void EmitAck(uint32_t id);
  void Measure(State& state, Time raw, Time ackDelay, bool probe);
  void Confirm(uint32_t id);
  void Change(uint32_t id, Status status);
  Time Timeout(const State& state) const;
  Settings m_settings;
  bool m_running = false;
  uint32_t m_node = 0;
  EventId m_tick;
  Ptr<UniformRandomVariable> m_random;
  std::vector<State> m_states;
  Callback<void, uint32_t, DciTunnelHeader> m_send;
  Callback<void, uint32_t> m_changed;
};
} // namespace ns3
#endif
