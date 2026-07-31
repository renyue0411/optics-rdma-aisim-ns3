/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/time-flow-table.h"
#include "ns3/assert.h"

namespace ns3 {

TimeFlowTable::TimeFlowTable ()
  : m_mode (DISABLED)
{
}

void
TimeFlowTable::SetMode (Mode mode)
{
  NS_ASSERT_MSG (mode == DISABLED ||
                 mode == FORWARD_THEN_GATE ||
                 mode == ROUTE_AND_GATE,
                 "Invalid switch time-flow mode");
  m_mode = mode;
}

TimeFlowTable::Mode
TimeFlowTable::GetMode () const
{
  return m_mode;
}

void
TimeFlowTable::Clear ()
{
  m_gateEntries.clear ();
}

bool
TimeFlowTable::GateKey::operator< (const GateKey &other) const
{
  if (dstIp != other.dstIp)
    {
      return dstIp < other.dstIp;
    }
  if (selectedOutIf != other.selectedOutIf)
    {
      return selectedOutIf < other.selectedOutIf;
    }
  return arrivalSlot < other.arrivalSlot;
}

void
TimeFlowTable::AddGateEntry (uint32_t dstIp,
                             uint32_t selectedOutIf,
                             uint32_t arrivalSlot,
                             uint32_t sendSlot)
{
  GateKey key;
  key.dstIp = dstIp;
  key.selectedOutIf = selectedOutIf;
  key.arrivalSlot = arrivalSlot;
  m_gateEntries[key] = sendSlot;
}

bool
TimeFlowTable::LookupGate (uint32_t dstIp,
                           uint32_t selectedOutIf,
                           uint32_t arrivalSlot,
                           uint32_t &sendSlot) const
{
  GateKey key;
  key.dstIp = dstIp;
  key.selectedOutIf = selectedOutIf;
  key.arrivalSlot = arrivalSlot;

  std::map<GateKey, uint32_t>::const_iterator it = m_gateEntries.find (key);
  if (it == m_gateEntries.end ())
    {
      return false;
    }

  sendSlot = it->second;
  return true;
}

uint32_t
TimeFlowTable::GetGateEntryCount () const
{
  return static_cast<uint32_t> (m_gateEntries.size ());
}

} // namespace ns3
