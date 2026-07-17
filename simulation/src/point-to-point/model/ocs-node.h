#ifndef OCS_NODE_H
#define OCS_NODE_H

#include <stdint.h>
#include <map>
#include <utility>
#include <vector>
#include <unordered_map>

#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/nstime.h"
#include "ns3/custom-header.h"
#include "ns3/qbb-net-device.h"

namespace ns3 {

/**
 * OpenOptics-style OCS model.
 *
 * This class models an OCS as a time-sliced transparent circuit fabric:
 *
 *   match(in_port, current_slice) -> connected_out_port
 *
 * Each scheduled port pair represents an optical circuit. When a stable
 * circuit exists, packets are delivered to the peer device attached to the
 * paired OCS port. The OCS does not perform packet-level output scheduling,
 * buffering, queueing, or output-port arbitration.
 *
 * OCS drops are limited to circuit-level unavailability:
 *   - switching blackout window
 *   - no circuit for the ingress port in the current slice
 *
 * For backward compatibility, it can also run in Phase-1 static mode:
 *
 *   match(in_port) -> connected_out_port
 */
class OcsNode : public Node
{
public:
  static TypeId GetTypeId(void);

  OcsNode();

  /** Remove all static circuits. */
  void ClearCircuits();

  /** Install a unidirectional static circuit. */
  void SetCircuit(uint32_t inPort, uint32_t outPort);

  /** Install a bidirectional static circuit. */
  void SetBidirectionalCircuit(uint32_t portA, uint32_t portB);

  /**
   * Compatibility wrapper for the existing OCS_MAP_FILE loader.
   *
   * This disables schedule mode and installs a static circuit map.
   */
  void SetInitialMapping(const std::vector<std::pair<uint32_t, uint32_t> >& mapping);

  /**
   * Configure local time-sliced OCS schedule.
   *
   * epochStart:     time when slice 0 starts
   * sliceDuration:  total duration of each slice
   * numSlices:      number of slices in one schedule period
   * switchingTime:  blackout interval at the end of each slice
   */
  void ConfigureSchedule(Time epochStart,
                         Time sliceDuration,
                         uint32_t numSlices,
                         Time switchingTime);

  /** Remove all scheduled circuits. */
  void ClearSchedule();

  /**
   * Install a unidirectional scheduled circuit:
   *
   *   match(inPort, slice) -> outPort
   */
  void AddScheduleEntry(uint32_t inPort,
                        uint32_t slice,
                        uint32_t outPort);

  /** Install a bidirectional scheduled circuit in a given slice. */
  void AddBidirectionalScheduleEntry(uint32_t portA,
                                     uint32_t portB,
                                     uint32_t slice);

  /** Query current slice based on Simulator::Now(). */
  uint32_t GetCurrentSlice() const;

  /** Return whether schedule mode is enabled. */
  bool IsScheduleEnabled() const;

  /** Return whether current time is inside the switching blackout window. */
  bool IsInSwitchingTime() const;

  /** Query whether an input port currently has an active circuit. */
  bool HasCircuit(uint32_t inPort) const;

  /**
   * Lookup output port. This automatically uses either static mode or
   * schedule mode depending on m_scheduleEnabled.
   */
  bool LookupOutPort(uint32_t inPort, uint32_t& outPort) const;

  /** Lookup output port under the current time-sliced schedule. */
  bool LookupScheduledOutPort(uint32_t inPort, uint32_t& outPort) const;

  /** Main OCS data-plane entry. */
  bool SwitchReceiveFromDevice(Ptr<NetDevice> device,
                               Ptr<Packet> packet,
                               CustomHeader &ch) override;

  /** OCS has no MMU/PFC/ECN state. */
  void SwitchNotifyDequeue(uint32_t ifIndex,
                           uint32_t qIndex,
                           Ptr<Packet> p) override;

  /** Print OCS forwarding/drop counters. */
  void DumpStats() const;

private:
  enum OcsForwardResult
  {
    OCS_FORWARD_OK = 0,
    OCS_FORWARD_BAD_DEVICE
  };

  static uint64_t MakeScheduleKey(uint32_t inPort, uint32_t slice);

  /**
   * Deliver a packet through a transparent optical circuit.
   *
   * outPort is an OCS-side ifIndex. The function finds the peer device
   * attached to that OCS port's channel and directly invokes peer Receive().
   * This intentionally avoids OCS output-device TransmitStart() and therefore
   * avoids artificial OCS outPort-busy drops.
   */
  OcsForwardResult ForwardOverCircuit(uint32_t outPort,
                                      Ptr<Packet> packet,
                                      CustomHeader &ch);

private:
  // Static circuit map:
  //   inPort -> outPort
  std::unordered_map<uint32_t, uint32_t> m_circuitMap;

  // Time-sliced schedule:
  //   (inPort, slice) -> outPort
  bool m_scheduleEnabled;
  uint32_t m_numSlices;
  Time m_epochStart;
  Time m_sliceDuration;
  Time m_switchingTime;
  std::unordered_map<uint64_t, uint32_t> m_schedule;

  // Basic stats.
  uint64_t m_forwardedPackets;
  uint64_t m_forwardedBytes;

  uint64_t m_dropNoCircuit;
  uint64_t m_dropSwitching;
  uint64_t m_dropBadPort;
  uint64_t m_dropBadDevice;

  std::map<uint32_t, uint64_t> m_forwardedPacketsByOutPort;
  std::map<uint32_t, uint64_t> m_forwardedBytesByOutPort;
};

} // namespace ns3

#endif // OCS_NODE_H
