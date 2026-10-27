// SPDX-License-Identifier: GPL-2.0-only
#include "dci-path-manager.h"
#include "ns3/abort.h"
#include "ns3/simulator.h"
#include <algorithm>
#include <iostream>
#include <limits>

namespace ns3 {
NS_OBJECT_ENSURE_REGISTERED(DciTunnelHeader);
NS_OBJECT_ENSURE_REGISTERED(DciPathManager);
TypeId DciTunnelHeader::GetTypeId()
{
  static TypeId tid = TypeId("ns3::DciTunnelHeader").SetParent<Header>()
    .AddConstructor<DciTunnelHeader>();
  return tid;
}
TypeId DciTunnelHeader::GetInstanceTypeId() const { return GetTypeId(); }
uint32_t DciTunnelHeader::GetSerializedSize() const { return SIZE; }
void DciTunnelHeader::Serialize(Buffer::Iterator i) const
{
  i.WriteHtonU16(0xdc12); i.WriteU8(1); i.WriteU8(kind);
  i.WriteHtonU16(path); i.WriteHtonU16(0);
  i.WriteHtonU64(number); i.WriteHtonU64(value); i.WriteHtonU64(delayNs);
}
uint32_t DciTunnelHeader::Deserialize(Buffer::Iterator i)
{
  uint16_t magic = i.ReadNtohU16();
  uint8_t version = i.ReadU8(), type = i.ReadU8();
  path = i.ReadNtohU16();
  uint16_t reserved = i.ReadNtohU16();
  number = i.ReadNtohU64(); value = i.ReadNtohU64(); delayNs = i.ReadNtohU64();
  m_valid = magic == 0xdc12 && version == 1 && reserved == 0 &&
            type >= DATA && type <= RESPONSE;
  kind = static_cast<Kind>(type);
  return SIZE;
}
void DciTunnelHeader::Print(std::ostream& os) const
{ os << "kind=" << unsigned(kind) << " path=" << path << " number=" << number; }
TypeId DciPathManager::GetTypeId()
{
  static TypeId tid = TypeId("ns3::DciPathManager").SetParent<Object>()
    .AddConstructor<DciPathManager>();
  return tid;
}
void DciPathManager::Configure(const Settings& settings)
{
  NS_ABORT_MSG_IF(m_running || settings.probeInterval <= Seconds(0) ||
    settings.initialTimeout <= Seconds(0) || settings.maxAckDelay <= Seconds(0) ||
    settings.failureThreshold == 0 || settings.ackEvery == 0 || settings.ackEvery > 32 ||
    settings.sampleEvery == 0, "Invalid path-manager settings");
  m_settings = settings;
}
void DciPathManager::AddPath()
{
  NS_ABORT_MSG_IF(m_running, "Cannot register a path after Start");
  State state;
  state.rtt = CreateObject<RttMeanDeviation>();
  m_states.push_back(state);
}
void DciPathManager::Start(uint32_t node,
    Callback<void, uint32_t, DciTunnelHeader> send, Callback<void, uint32_t> changed)
{
  NS_ABORT_MSG_IF(m_running || m_states.empty() || send.IsNull() || changed.IsNull(),
    "Invalid path-manager Start");
  m_running = true; m_node = node; m_send = send; m_changed = changed;
  m_random = CreateObject<UniformRandomVariable>();
  m_random->SetStream(10000 + node);
  for (uint32_t id = 0; id < m_states.size(); ++id)
    std::cout << "DCI_STATE node=" << m_node << " path=" << id
      << " time_s=" << Simulator::Now().GetSeconds() << " to=VALIDATING\n";
  Tick();
}
void DciPathManager::Stop()
{
  m_running = false;
  Simulator::Cancel(m_tick);
  for (auto& state : m_states) Simulator::Cancel(state.ackEvent);
}
bool DciPathManager::CanSend(uint32_t id) const
{ return m_states.at(id).validated && m_states.at(id).status == ACTIVE; }
bool DciPathManager::HasSample(uint32_t id) const
{ return m_states.at(id).rtt->GetNSamples() != 0; }
Time DciPathManager::GetRtt(uint32_t id) const
{ return m_states.at(id).rtt->GetEstimate(); }
const char* DciPathManager::Name(Status status)
{
  static const char* names[] = {"VALIDATING", "ACTIVE", "SUSPECT", "FAILED"};
  return names[status];
}
void DciPathManager::Change(uint32_t id, Status status)
{
  auto& state = m_states.at(id);
  if (state.status == status) return;
  std::cout << "DCI_STATE node=" << m_node << " path=" << id
    << " time_s=" << Simulator::Now().GetSeconds() << " from=" << Name(state.status)
    << " to=" << Name(status) << " missed=" << state.missed << '\n';
  state.status = status;
  m_changed(id);
}
void DciPathManager::Confirm(uint32_t id)
{
  auto& state = m_states.at(id);
  state.lastConfirmed = Simulator::Now(); state.missed = 0;
  if (state.validated) Change(id, ACTIVE);
}
Time DciPathManager::Timeout(const State& state) const
{
  if (state.rtt->GetNSamples() == 0) return m_settings.initialTimeout;
  // A QUIC-style probe timeout, with an explicit 20 ms floor for this model.
  int64_t ns = state.rtt->GetEstimate().GetNanoSeconds() +
    std::max<int64_t>(4 * state.rtt->GetVariation().GetNanoSeconds(), 1000000) +
    m_settings.maxAckDelay.GetNanoSeconds();
  return NanoSeconds(std::max<int64_t>(20000000, ns));
}
void DciPathManager::Measure(State& state, Time raw, Time delay, bool probe)
{
  if (raw <= Seconds(0)) return;
  state.latestRtt = raw;
  if (state.minRtt == Seconds(0) || raw < state.minRtt) state.minRtt = raw;
  Time adjusted = raw;
  // Do not subtract more ACK delay than can be explained above min RTT.
  delay = std::min(delay, m_settings.maxAckDelay);
  if (raw - state.minRtt >= delay) adjusted -= delay;
  state.rtt->Measurement(adjusted);
  if (probe) ++state.probeRttSamples; else ++state.dataRttSamples;
}
DciTunnelHeader DciPathManager::DataSent(uint32_t id, uint32_t size)
{
  auto& state = m_states.at(id);
  NS_ABORT_MSG_IF(!CanSend(id), "DATA scheduled on an unvalidated/inactive path");
  NS_ABORT_MSG_IF(state.nextData == std::numeric_limits<uint64_t>::max(), "Packet number wrapped");
  DciTunnelHeader h; h.path = id; h.kind = DciTunnelHeader::DATA;
  h.number = ++state.nextData; h.value = size;
  ++state.dataTx;
  // Bound bookkeeping at large BDP; ACKs still cover every received DATA PN.
  if (h.number == 1 || h.number % m_settings.sampleEvery == 0)
  {
    if (state.samples.size() >= 65536)
    { state.samples.erase(state.samples.begin()); ++state.evictedSamples; }
    state.samples.emplace(h.number, Simulator::Now());
  }
  return h;
}
void DciPathManager::ControlSent(uint32_t id, const DciTunnelHeader& h)
{
  auto& state = m_states.at(id);
  if (h.kind == DciTunnelHeader::ACK) ++state.ackTx;
  if (h.kind == DciTunnelHeader::CHALLENGE && state.probePending &&
      h.number == state.nextProbe && h.value == state.token)
  { state.probeSent = Simulator::Now(); state.probeOnWire = true; ++state.probeTx; }
}
void DciPathManager::ControlDropped(uint32_t id, const DciTunnelHeader& h)
{
  auto& state = m_states.at(id);
  if (h.kind == DciTunnelHeader::CHALLENGE && state.probePending &&
      h.number == state.nextProbe && h.value == state.token)
  {
    state.probePending = false; state.probeOnWire = false;
    state.nextProbeAt = Simulator::Now() + m_settings.probeInterval;
  }
}
void DciPathManager::Tick()
{
  if (!m_running) return;
  Time now = Simulator::Now();
  for (uint32_t id = 0; id < m_states.size(); ++id)
  {
    auto& state = m_states[id];
    Time timeout = Timeout(state);
    if (state.probePending && state.probeOnWire && now - state.probeSent >= timeout)
    {
      state.probePending = false; state.probeOnWire = false; ++state.probeTimeouts;
      if (!(state.validated && state.lastConfirmed >= state.probeSent))
      {
        ++state.missed;
        Change(id, state.missed >= m_settings.failureThreshold ? FAILED : SUSPECT);
      }
      state.nextProbeAt = now + m_settings.probeInterval;
    }
    if (!state.probePending && now >= state.nextProbeAt)
    {
      state.probePending = true; state.probeOnWire = false;
      state.token = (uint64_t(m_random->GetInteger(0, 0xffffffffU)) << 32) |
                     m_random->GetInteger(0, 0xffffffffU);
      DciTunnelHeader h; h.kind = DciTunnelHeader::CHALLENGE; h.path = id;
      h.number = ++state.nextProbe; h.value = state.token;
      m_send(id, h);
    }
    Time retention = NanoSeconds(std::max<int64_t>(1000000000,
                                                  8 * timeout.GetNanoSeconds()));
    while (!state.samples.empty() && now - state.samples.begin()->second > retention)
    { state.samples.erase(state.samples.begin()); ++state.expiredSamples; }
  }
  m_tick = Simulator::Schedule(MilliSeconds(1), &DciPathManager::Tick, this);
}
void DciPathManager::EmitAck(uint32_t id)
{
  if (!m_running) return;
  auto& state = m_states.at(id);
  if (state.ackCount == 0) return;
  DciTunnelHeader h; h.kind = DciTunnelHeader::ACK; h.path = id;
  h.number = state.largestRx; h.value = state.receiveMask;
  h.delayNs = (Simulator::Now() - state.largestRxAt).GetNanoSeconds();
  state.ackCount = 0;
  m_send(id, h);
}
void DciPathManager::DataReceived(uint32_t id, const DciTunnelHeader& h)
{
  auto& state = m_states.at(id);
  ++state.dataRx;
  if (h.number > state.largestRx)
  {
    uint64_t delta = h.number - state.largestRx;
    state.receiveMask = delta >= 64 ? 1 : (state.receiveMask << delta) | 1;
    state.largestRx = h.number; state.largestRxAt = Simulator::Now();
  }
  else
  {
    uint64_t delta = state.largestRx - h.number;
    if (delta >= 64)
    {
      // An old reordered datagram needs its own ACK; it is not in the bitmap.
      DciTunnelHeader old; old.kind = DciTunnelHeader::ACK; old.path = id;
      old.number = h.number; old.value = 1;
      m_send(id, old); return;
    }
    state.receiveMask |= uint64_t(1) << delta;
  }
  if (++state.ackCount >= m_settings.ackEvery)
  { Simulator::Cancel(state.ackEvent); EmitAck(id); }
  else if (!state.ackEvent.IsRunning())
    state.ackEvent = Simulator::Schedule(m_settings.maxAckDelay,
                                        &DciPathManager::EmitAck, this, id);
}
bool DciPathManager::ControlReceived(uint32_t id, const DciTunnelHeader& h)
{
  auto& state = m_states.at(id);
  if (h.number == 0) return false;
  if (h.kind == DciTunnelHeader::CHALLENGE)
  {
    ++state.probeRx;
    DciTunnelHeader reply = h; reply.kind = DciTunnelHeader::RESPONSE; reply.delayNs = 0;
    m_send(id, reply); return true;
  }
  if (h.kind == DciTunnelHeader::RESPONSE)
  {
    // Late responses to an expired attempt do not validate a different attempt.
    if (!state.probePending || !state.probeOnWire ||
        h.number != state.nextProbe || h.value != state.token) return true;
    Measure(state, Simulator::Now() - state.probeSent, Seconds(0), true);
    state.probePending = false; state.probeOnWire = false; state.validated = true;
    state.nextProbeAt = Simulator::Now() + m_settings.probeInterval;
    Confirm(id); return true;
  }
  if (h.kind != DciTunnelHeader::ACK || h.number > state.nextData || (h.value & 1) == 0)
    return false;
  ++state.ackRx;
  bool advanced = h.number > state.lastAcked;
  for (uint32_t bit = 0; bit < 64 && bit < h.number; ++bit)
  {
    if ((h.value & (uint64_t(1) << bit)) == 0) continue;
    auto it = state.samples.find(h.number - bit);
    if (it == state.samples.end()) continue;
    if (bit == 0 && advanced)
      Measure(state, Simulator::Now() - it->second,
        NanoSeconds(static_cast<int64_t>(std::min<uint64_t>(h.delayNs,
          m_settings.maxAckDelay.GetNanoSeconds()))), false);
    state.samples.erase(it);
  }
  if (advanced) { state.lastAcked = h.number; Confirm(id); }
  return true;
}
void DciPathManager::PrintStats() const
{
  for (uint32_t id = 0; id < m_states.size(); ++id)
  {
    const auto& s = m_states[id];
    std::cout << "DCI_PATH_STATE node=" << m_node << " path=" << id
      << " state=" << Name(s.status) << " validated=" << s.validated
      << " latest_rtt_ms=" << s.latestRtt.GetSeconds() * 1000
      << " min_rtt_ms=" << s.minRtt.GetSeconds() * 1000
      << " srtt_ms=" << (s.rtt->GetNSamples() ? s.rtt->GetEstimate().GetSeconds() * 1000 : 0)
      << " rttvar_ms=" << (s.rtt->GetNSamples() ? s.rtt->GetVariation().GetSeconds() * 1000 : 0)
      << " data_rtt_samples=" << s.dataRttSamples << " probe_rtt_samples=" << s.probeRttSamples
      << " ack_tx=" << s.ackTx << " ack_rx=" << s.ackRx
      << " probe_timeouts=" << s.probeTimeouts << " expired_samples=" << s.expiredSamples
      << " evicted_samples=" << s.evictedSamples << " pending_samples=" << s.samples.size() << '\n';
  }
}
void DciPathManager::WriteCsv(std::ostream& os) const
{
  for (uint32_t id = 0; id < m_states.size(); ++id)
  {
    const auto& s = m_states[id];
    os << m_node << ',' << id << ',' << Name(s.status) << ',' << s.validated << ','
      << s.latestRtt.GetSeconds() * 1000 << ',' << s.minRtt.GetSeconds() * 1000 << ','
      << (s.rtt->GetNSamples() ? s.rtt->GetEstimate().GetSeconds() * 1000 : 0) << ','
      << (s.rtt->GetNSamples() ? s.rtt->GetVariation().GetSeconds() * 1000 : 0) << ','
      << s.dataRttSamples << ',' << s.probeRttSamples << ',' << s.ackTx << ',' << s.ackRx << ','
      << s.probeTimeouts << ',' << s.expiredSamples << ',' << s.evictedSamples << '\n';
  }
}
} // namespace ns3
