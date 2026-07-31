/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
* Copyright (c) 2006 Georgia Tech Research Corporation, INRIA
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation;
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
#include <iostream>
#include <stdio.h>
#include "ns3/log.h"
#include "ns3/enum.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/simulator.h"
#include "drop-tail-queue.h"
#include "broadcom-egress-queue.h"

NS_LOG_COMPONENT_DEFINE("BEgressQueue");

namespace ns3 {

	NS_OBJECT_ENSURE_REGISTERED(BEgressQueue);

	TypeId BEgressQueue::GetTypeId(void)
	{
		static TypeId tid = TypeId("ns3::BEgressQueue")
			.SetParent<PacketQueue>()
			.AddConstructor<BEgressQueue>()
			.AddAttribute("MaxBytes",
				"The maximum number of bytes accepted by this BEgressQueue.",
				DoubleValue(1000.0 * 1024 * 1024),
				MakeDoubleAccessor(&BEgressQueue::m_maxBytes),
				MakeDoubleChecker<double>())
			.AddTraceSource ("BeqEnqueue", "Enqueue a packet in the BEgressQueue. Multiple queue",
					MakeTraceSourceAccessor (&BEgressQueue::m_traceBeqEnqueue),
					"ns3::BEgressQueue::TracedCallback")
			.AddTraceSource ("BeqDequeue", "Dequeue a packet in the BEgressQueue. Multiple queue",
					MakeTraceSourceAccessor (&BEgressQueue::m_traceBeqDequeue),
					"ns3::BEgressQueue::TracedCallback")
			;

		return tid;
	}

	BEgressQueue::BEgressQueue() 
		: PacketQueue(),
		m_calendarEnabled(false),
		m_activeCalendarSlot(-1),
		NS_LOG_TEMPLATE_DEFINE ("BEgressQueue")
	{
		NS_LOG_FUNCTION_NOARGS();
		m_bytesInQueueTotal = 0;
		m_rrlast = 0;
		m_qlast = 0;
		m_nBytes = 0;
		m_nTotalReceivedBytes = 0;
		m_nPackets = 0;
		m_nTotalReceivedPackets = 0;
		m_nTotalDroppedBytes = 0;
		m_nTotalDroppedPackets = 0;
		for (uint32_t i = 0; i < fCnt; i++)
		{
			m_bytesInQueue[i] = 0;
			m_queues.push_back(CreateObject<SimpleDropTailQueue>());
			// m_queues.push_back(CreateObject<PacketQueue>());
		}
	}

	BEgressQueue::~BEgressQueue()
	{
		NS_LOG_FUNCTION_NOARGS();
	}

	void
		BEgressQueue::ConfigureCalendar(uint32_t numSlots)
	{
		NS_ASSERT_MSG(m_bytesInQueueTotal == 0,
		              "Calendar configuration must be installed before packets are queued");
		m_calendarQueue.Configure(numSlots, qCnt);
		m_calendarEnabled = true;
		m_activeCalendarSlot = -1;
	}

	bool
		BEgressQueue::IsCalendarEnabled() const
	{
		return m_calendarEnabled;
	}

	uint32_t
		BEgressQueue::GetCalendarNumSlots() const
	{
		return m_calendarQueue.IsConfigured() ? m_calendarQueue.GetNumSlots() : 0;
	}

	void
		BEgressQueue::SetActiveCalendarSlot(int32_t slot)
	{
		if (slot >= 0)
		{
			NS_ASSERT_MSG(m_calendarEnabled, "Cannot activate an unconfigured CalendarQueue");
			NS_ASSERT_MSG(static_cast<uint32_t>(slot) < m_calendarQueue.GetNumSlots(),
			              "Calendar slot out of range");
		}
		m_activeCalendarSlot = slot;
	}

	int32_t
		BEgressQueue::GetActiveCalendarSlot() const
	{
		return m_activeCalendarSlot;
	}

	bool
		BEgressQueue::DoEnqueue(Ptr<Packet> p, uint32_t qIndex)
	{
		NS_LOG_FUNCTION(this << p);
		NS_ASSERT_MSG(qIndex < fCnt, "BEgressQueue priority out of range");

		if (m_bytesInQueueTotal + p->GetSize() < m_maxBytes)  //infinite queue
		{
			m_queues[qIndex]->Enqueue(p);
			m_bytesInQueueTotal += p->GetSize();
			m_bytesInQueue[qIndex] += p->GetSize();
		}
		else
		{
			return false;
		}
		return true;
	}

	bool
		BEgressQueue::DoEnqueueCalendar(Ptr<Packet> p, uint32_t qIndex, uint32_t sendSlot)
	{
		NS_LOG_FUNCTION(this << p << qIndex << sendSlot);
		NS_ASSERT_MSG(m_calendarEnabled, "Calendar enqueue on an unconfigured queue");
		NS_ASSERT_MSG(qIndex < qCnt, "Calendar priority out of range");
		NS_ASSERT_MSG(sendSlot < m_calendarQueue.GetNumSlots(), "Calendar send slot out of range");

		if (m_bytesInQueueTotal + p->GetSize() < m_maxBytes)
		{
			m_calendarQueue.Enqueue(sendSlot, qIndex, p);
			m_bytesInQueueTotal += p->GetSize();
			m_bytesInQueue[qIndex] += p->GetSize();
			return true;
		}
		return false;
	}

	bool
		BEgressQueue::SelectNextQueue(bool paused[], uint32_t &qIndex, bool &fromCalendar) const
	{
		fromCalendar = false;

		// Link-local priority-0 traffic (notably PFC) remains immediately eligible.
		if (m_queues[0]->GetNPackets() > 0)
		{
			qIndex = 0;
			return true;
		}

		// Routed priority-0 packets may be time-gated but retain strict priority.
		if (m_calendarEnabled && m_activeCalendarSlot >= 0 &&
		    m_calendarQueue.Peek(static_cast<uint32_t>(m_activeCalendarSlot), 0) != 0)
		{
			qIndex = 0;
			fromCalendar = true;
			return true;
		}

		// Round-robin across data PGs 1..7.
		for (uint32_t step = 1; step < qCnt; ++step)
		{
			uint32_t candidate = 1 + ((m_rrlast + step - 1) % (qCnt - 1));
			if (paused[candidate])
			{
				continue;
			}

			if (m_queues[candidate]->GetNPackets() > 0)
			{
				qIndex = candidate;
				return true;
			}

			if (m_calendarEnabled && m_activeCalendarSlot >= 0 &&
			    m_calendarQueue.Peek(static_cast<uint32_t>(m_activeCalendarSlot), candidate) != 0)
			{
				qIndex = candidate;
				fromCalendar = true;
				return true;
			}
		}

		return false;
	}

	Ptr<const Packet>
		BEgressQueue::PeekRR(bool paused[], uint32_t &qIndex, bool &fromCalendar) const
	{
		if (!SelectNextQueue(paused, qIndex, fromCalendar))
		{
			return 0;
		}
		if (fromCalendar)
		{
			return m_calendarQueue.Peek(static_cast<uint32_t>(m_activeCalendarSlot), qIndex);
		}
		return m_queues[qIndex]->Peek();
	}

	Ptr<Packet>
		BEgressQueue::DoDequeueRR(bool paused[]) //this is for switch only
	{
		NS_LOG_FUNCTION(this);

		if (m_bytesInQueueTotal == 0)
		{
			NS_LOG_LOGIC("Queue empty");
			return 0;
		}

		uint32_t qIndex = 0;
		bool fromCalendar = false;
		if (!SelectNextQueue(paused, qIndex, fromCalendar))
		{
			NS_LOG_LOGIC("Nothing can be sent");
			return 0;
		}

		Ptr<Packet> p;
		if (fromCalendar)
		{
			p = m_calendarQueue.Dequeue(static_cast<uint32_t>(m_activeCalendarSlot), qIndex);
		}
		else
		{
			p = m_queues[qIndex]->Dequeue();
		}

		NS_ASSERT_MSG(p != 0, "Selected BEgressQueue entry disappeared");
		m_traceBeqDequeue(p, qIndex);
		m_bytesInQueueTotal -= p->GetSize();
		m_bytesInQueue[qIndex] -= p->GetSize();
		if (qIndex != 0)
		{
			m_rrlast = qIndex;
		}
		m_qlast = qIndex;
		return p;
	}

	bool
		BEgressQueue::Enqueue(Ptr<Packet> p, uint32_t qIndex)
	{
		NS_LOG_FUNCTION(this << p);
		//
		// If DoEnqueue fails, Queue::Drop is called by the subclass
		//
		bool retval = DoEnqueue(p, qIndex);
		if (retval)
		{
			NS_LOG_LOGIC("m_traceEnqueue (p)");
			m_traceEnqueue(p);
			m_traceBeqEnqueue(p, qIndex);

			uint32_t size = p->GetSize();
			m_nBytes += size;
			m_nTotalReceivedBytes += size;

			m_nPackets++;
			m_nTotalReceivedPackets++;
		}
		return retval;
	}

	bool
		BEgressQueue::EnqueueCalendar(Ptr<Packet> p, uint32_t qIndex, uint32_t sendSlot)
	{
		NS_LOG_FUNCTION(this << p << qIndex << sendSlot);
		bool retval = DoEnqueueCalendar(p, qIndex, sendSlot);
		if (retval)
		{
			m_traceEnqueue(p);
			m_traceBeqEnqueue(p, qIndex);
			uint32_t size = p->GetSize();
			m_nBytes += size;
			m_nTotalReceivedBytes += size;
			m_nPackets++;
			m_nTotalReceivedPackets++;
		}
		return retval;
	}

	Ptr<Packet>
		BEgressQueue::DequeueRR(bool paused[])
	{
		NS_LOG_FUNCTION(this);
		Ptr<Packet> packet = DoDequeueRR(paused);
		if (packet != 0)
		{
			NS_ASSERT(m_nBytes >= packet->GetSize());
			NS_ASSERT(m_nPackets > 0);
			m_nBytes -= packet->GetSize();
			m_nPackets--;
			NS_LOG_LOGIC("m_traceDequeue (packet)");
			m_traceDequeue(packet);
		}
		return packet;
	}

	bool
		BEgressQueue::DoEnqueue(Ptr<Packet> p)	//for compatiability
	{
		std::cout << "Warning: Call Broadcom queues without priority\n";
		uint32_t qIndex = 0;
		NS_LOG_FUNCTION(this << p);
		if (m_bytesInQueueTotal + p->GetSize() < m_maxBytes)
		{
			m_queues[qIndex]->Enqueue(p);
			m_bytesInQueueTotal += p->GetSize();
			m_bytesInQueue[qIndex] += p->GetSize();
		}
		else
		{
			return false;

		}
		return true;
	}


	Ptr<Packet>
		BEgressQueue::DoDequeue(void)
	{
		NS_ASSERT_MSG(false, "BEgressQueue::DoDequeue not implemented");
		return 0;
	}


	Ptr<const Packet>
		BEgressQueue::DoPeek(void) const	//DoPeek doesn't work for multiple queues!!
	{
		std::cout << "Warning: Call Broadcom queues without priority\n";
		NS_LOG_FUNCTION(this);
		if (m_bytesInQueueTotal == 0)
		{
			NS_LOG_LOGIC("Queue empty");
			return 0;
		}
		NS_LOG_LOGIC("Number bytes " << m_bytesInQueue);
		return m_queues[0]->Peek();
	}

	uint32_t
		BEgressQueue::GetNBytes(uint32_t qIndex) const
	{
		return m_bytesInQueue[qIndex];
	}


	uint32_t
		BEgressQueue::GetNBytesTotal() const
	{
		return m_bytesInQueueTotal;
	}

	uint32_t
		BEgressQueue::GetLastQueue()
	{
		return m_qlast;
	}

}
