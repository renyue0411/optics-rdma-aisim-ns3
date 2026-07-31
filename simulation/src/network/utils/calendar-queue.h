/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef CALENDAR_QUEUE_H
#define CALENDAR_QUEUE_H

#include <deque>
#include <vector>
#include <stdint.h>

#include "ns3/assert.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"

namespace ns3 {

/**
 * Internal time-bucket queue used by BEgressQueue.
 *
 * CalendarQueue owns no independent capacity or MMU state.  BEgressQueue
 * remains the single egress admission/accounting point and aggregates bytes
 * across all calendar slots for each priority group.
 */
class CalendarQueue
{
public:
  CalendarQueue ()
    : m_numSlots (0),
      m_numQueues (0)
  {
  }

  void Configure (uint32_t numSlots, uint32_t numQueues)
  {
    NS_ASSERT_MSG (numSlots > 0, "CalendarQueue requires at least one slot");
    NS_ASSERT_MSG (numQueues > 0, "CalendarQueue requires at least one priority queue");

    m_numSlots = numSlots;
    m_numQueues = numQueues;
    m_queues.assign (numSlots,
                     std::vector<std::deque<Ptr<Packet> > > (numQueues));
    m_bytes.assign (numSlots, std::vector<uint32_t> (numQueues, 0));
  }

  bool IsConfigured () const
  {
    return m_numSlots > 0 && m_numQueues > 0;
  }

  uint32_t GetNumSlots () const
  {
    return m_numSlots;
  }

  bool Enqueue (uint32_t slot, uint32_t qIndex, Ptr<Packet> packet)
  {
    CheckIndex (slot, qIndex);
    NS_ASSERT_MSG (packet != 0, "Cannot enqueue a null packet");
    m_queues[slot][qIndex].push_back (packet);
    m_bytes[slot][qIndex] += packet->GetSize ();
    return true;
  }

  Ptr<const Packet> Peek (uint32_t slot, uint32_t qIndex) const
  {
    CheckIndex (slot, qIndex);
    if (m_queues[slot][qIndex].empty ())
      {
        return 0;
      }
    return m_queues[slot][qIndex].front ();
  }

  Ptr<Packet> Dequeue (uint32_t slot, uint32_t qIndex)
  {
    CheckIndex (slot, qIndex);
    if (m_queues[slot][qIndex].empty ())
      {
        return 0;
      }

    Ptr<Packet> packet = m_queues[slot][qIndex].front ();
    m_queues[slot][qIndex].pop_front ();
    NS_ASSERT_MSG (m_bytes[slot][qIndex] >= packet->GetSize (),
                   "CalendarQueue byte accounting underflow");
    m_bytes[slot][qIndex] -= packet->GetSize ();
    return packet;
  }

  Ptr<Packet> DequeueAny (uint32_t &slot, uint32_t &qIndex)
  {
    for (uint32_t s = 0; s < m_numSlots; ++s)
      {
        for (uint32_t q = 0; q < m_numQueues; ++q)
          {
            if (!m_queues[s][q].empty ())
              {
                slot = s;
                qIndex = q;
                return Dequeue (s, q);
              }
          }
      }
    return 0;
  }

  uint32_t GetNBytes (uint32_t slot, uint32_t qIndex) const
  {
    CheckIndex (slot, qIndex);
    return m_bytes[slot][qIndex];
  }

  uint32_t GetNBytesByPriority (uint32_t qIndex) const
  {
    NS_ASSERT_MSG (qIndex < m_numQueues, "CalendarQueue priority out of range");
    uint32_t total = 0;
    for (uint32_t s = 0; s < m_numSlots; ++s)
      {
        total += m_bytes[s][qIndex];
      }
    return total;
  }

  uint32_t GetNBytesTotal () const
  {
    uint32_t total = 0;
    for (uint32_t s = 0; s < m_numSlots; ++s)
      {
        for (uint32_t q = 0; q < m_numQueues; ++q)
          {
            total += m_bytes[s][q];
          }
      }
    return total;
  }

private:
  void CheckIndex (uint32_t slot, uint32_t qIndex) const
  {
    NS_ASSERT_MSG (IsConfigured (), "CalendarQueue is not configured");
    NS_ASSERT_MSG (slot < m_numSlots, "CalendarQueue slot out of range");
    NS_ASSERT_MSG (qIndex < m_numQueues, "CalendarQueue priority out of range");
  }

  uint32_t m_numSlots;
  uint32_t m_numQueues;
  std::vector<std::vector<std::deque<Ptr<Packet> > > > m_queues;
  std::vector<std::vector<uint32_t> > m_bytes;
};

} // namespace ns3

#endif // CALENDAR_QUEUE_H
