/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef TIME_FLOW_TABLE_H
#define TIME_FLOW_TABLE_H

#include <map>
#include <stdint.h>

namespace ns3 {

/**
 * Switch-side time-flow table.
 *
 * Mode 1 keeps the existing forwarding decision and only selects the
 * departure calendar slot:
 *
 *   (destination IP, selected out-if, arrival slot) -> send slot
 *
 * Mode 2 is reserved for route-and-gate and is intentionally not implemented
 * in the first integration.
 */
class TimeFlowTable
{
public:
  enum Mode
  {
    DISABLED = 0,
    FORWARD_THEN_GATE = 1,
    ROUTE_AND_GATE = 2
  };

  TimeFlowTable ();

  void SetMode (Mode mode);
  Mode GetMode () const;

  void Clear ();

  void AddGateEntry (uint32_t dstIp,
                     uint32_t selectedOutIf,
                     uint32_t arrivalSlot,
                     uint32_t sendSlot);

  bool LookupGate (uint32_t dstIp,
                   uint32_t selectedOutIf,
                   uint32_t arrivalSlot,
                   uint32_t &sendSlot) const;

  uint32_t GetGateEntryCount () const;

private:
  struct GateKey
  {
    uint32_t dstIp;
    uint32_t selectedOutIf;
    uint32_t arrivalSlot;

    bool operator< (const GateKey &other) const;
  };

  Mode m_mode;
  std::map<GateKey, uint32_t> m_gateEntries;
};

} // namespace ns3

#endif // TIME_FLOW_TABLE_H
