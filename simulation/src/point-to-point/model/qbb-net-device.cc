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
*
* Author: Yuliang Li <yuliangli@g.harvard.com>
*/

#define __STDC_LIMIT_MACROS 1
#include <stdint.h>
#include <stdio.h>
#include "ns3/qbb-net-device.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/data-rate.h"
#include "ns3/object-vector.h"
#include "ns3/pause-header.h"
#include "ns3/simple-drop-tail-queue.h"
#include "ns3/red-queue.h"
#include "ns3/assert.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-header.h"
#include "ns3/simulator.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/qbb-channel.h"
#include "ns3/random-variable.h"
#include "ns3/flow-id-tag.h"
#include "ns3/qbb-header.h"
#include "ns3/error-model.h"
#include "ns3/cn-header.h"
#include "ns3/ppp-header.h"
#include "ns3/udp-header.h"
#include "ns3/seq-ts-header.h"
#include "ns3/pointer.h"
#include "ns3/custom-header.h"
#include <iostream>
// MODE1_CONTINUATION_ACK_RECOVERY_V1: gate may grant one bounded window bypass.
// MODE1_FINAL_ACK_RECOVERY_V1: a final-outstanding QP may emit one tail probe.
NS_LOG_COMPONENT_DEFINE("QbbNetDevice");

namespace ns3 {
	
	uint32_t RdmaEgressQueue::ack_q_idx = 3;
	// RdmaEgressQueue
	TypeId RdmaEgressQueue::GetTypeId (void)
	{
		static TypeId tid = TypeId ("ns3::RdmaEgressQueue")
			.SetParent<Object> ()
			.AddTraceSource ("RdmaEnqueue", "Enqueue a packet in the RdmaEgressQueue.",
					MakeTraceSourceAccessor (&RdmaEgressQueue::m_traceRdmaEnqueue),
					"ns3::RdmaEgressQueue::RdmaEnqueue")
			.AddTraceSource ("RdmaDequeue", "Dequeue a packet in the RdmaEgressQueue.",
					MakeTraceSourceAccessor (&RdmaEgressQueue::m_traceRdmaDequeue),
					"ns3::RdmaEgressQueue::RdmaDequeue")
			;
		return tid;
	}

	RdmaEgressQueue::RdmaEgressQueue(){
		m_rrlast = 0;
		m_qlast = 0;
		m_ackQ = CreateObject<SimpleDropTailQueue>();
		//m_ackQ = CreateObject<RedQueue>();
		m_ackQ->SetAttribute("MaxBytes", UintegerValue(0xffffffff)); // queue limit is on a higher level, not here
		m_nextGateWake = Simulator::GetMaximumSimulationTime();
	}

	Ptr<Packet> RdmaEgressQueue::DequeueQindex(int qIndex){
		if (qIndex == -1){ // high prio
			Ptr<Packet> p = m_ackQ->Dequeue();
			m_qlast = -1;
			m_traceRdmaDequeue(p, 0);
			return p;
		}
		if (qIndex >= 0){ // qp
			Ptr<Packet> p = m_rdmaGetNxtPkt(m_qpGrp->Get(qIndex));
			m_rrlast = qIndex;
			m_qlast = qIndex;
			m_traceRdmaDequeue(p, m_qpGrp->Get(qIndex)->m_pg);
			return p;
		}
		return 0;
	}
	int RdmaEgressQueue::GetNextQindex(bool paused[]){
		m_nextGateWake = Simulator::GetMaximumSimulationTime();
		bool found = false;
		uint32_t qIndex;
		if (!paused[ack_q_idx] && m_ackQ->GetNPackets() > 0)
			return -1;

		// no pkt in highest priority queue, do rr for each qp
		int res = -1024;
		uint32_t fcount = m_qpGrp->GetN();
		uint32_t min_finish_id = 0xffffffff;
		for (qIndex = 1; qIndex <= fcount; qIndex++){
			// iterate start behind the m_rrlast qp
			uint32_t idx = (qIndex + m_rrlast) % fcount;
			Ptr<RdmaQueuePair> qp = m_qpGrp->Get(idx);
			if (qp->m_qpError){
				min_finish_id = idx < min_finish_id ? idx : min_finish_id;
				continue;
			}
			if(qp->GetBytesLeft()<=0){
				int sender_node = qp->GetSrc();
				int receiver_node = qp->GetDest();
				int tag = qp->GetTag();
				int t_count = qp->GetInitialSize();
				// qp transmission finished
			}
			const bool hasNewData = qp->GetBytesLeft() > 0;
			const bool hasFinalOutstanding =
				!hasNewData &&
				!m_rdmaGateAllowQp.IsNull() &&
				qp->snd_una < qp->m_highestSentSeq;
			if (!paused[qp->m_pg] && (hasNewData || hasFinalOutstanding)){
				if (m_qpGrp->Get(idx)->m_nextAvail.GetTimeStep() > Simulator::Now().GetTimeStep()) //not available now
					continue;

				// Consult the Mode-1 gate even when the ordinary transport window
				// is full or all new DATA has already been sent. The gate may grant
				// one continuation packet or one oldest-unacknowledged tail probe.
				if (!m_rdmaGateAllowQp.IsNull() && !m_rdmaGateAllowQp(qp)){
					if (!m_rdmaGateNextTime.IsNull()){
						Time t = m_rdmaGateNextTime(qp);
						if (t > Simulator::Now() && t < m_nextGateWake){
							m_nextGateWake = t;
						}
					}
					continue;
				}

				const bool tailPermit =
					qp->m_ackRecoveryPermit &&
					qp->m_ackRecoveryPermitType == RNIC_ACK_PROBE_TAIL;
				if (!hasNewData && !tailPermit){
					continue;
				}
				if (hasNewData && qp->IsWinBound() && !qp->m_ackRecoveryPermit){
					continue;
				}

				res = idx;
				break;
			}else if (qp->IsFinished()){
				min_finish_id = idx < min_finish_id ? idx : min_finish_id;
			}
		}

		// clear the finished qp
		if (min_finish_id < 0xffffffff){
			int nxt = min_finish_id;
			auto &qps = m_qpGrp->m_qps;
			for (int i = min_finish_id + 1; i < fcount; i++) if (!qps[i]->IsFinished() && !qps[i]->m_qpError){
				if (i == res) // update res to the idx after removing finished qp
					res = nxt;
				qps[nxt] = qps[i];
				nxt++;
			}
			qps.resize(nxt);
		}
		return res;
	}

	
	Time RdmaEgressQueue::GetNextGateWake(void) const{
		return m_nextGateWake;
	}

	int RdmaEgressQueue::GetLastQueue(){
		return m_qlast;
	}

	uint32_t RdmaEgressQueue::GetNBytes(uint32_t qIndex){
		NS_ASSERT_MSG(qIndex < m_qpGrp->GetN(), "RdmaEgressQueue::GetNBytes: qIndex >= m_qpGrp->GetN()");
		return m_qpGrp->Get(qIndex)->GetBytesLeft();
	}

	uint32_t RdmaEgressQueue::GetFlowCount(void){
		return m_qpGrp->GetN();
	}

	Ptr<RdmaQueuePair> RdmaEgressQueue::GetQp(uint32_t i){
		return m_qpGrp->Get(i);
	}
 
	void RdmaEgressQueue::RecoverQueue(uint32_t i){
		NS_ASSERT_MSG(i < m_qpGrp->GetN(), "RdmaEgressQueue::RecoverQueue: qIndex >= m_qpGrp->GetN()");
		m_qpGrp->Get(i)->snd_nxt = m_qpGrp->Get(i)->snd_una;
	}

	void RdmaEgressQueue::EnqueueHighPrioQ(Ptr<Packet> p){
		m_traceRdmaEnqueue(p, 0);
		m_ackQ->Enqueue(p);
	}

	void RdmaEgressQueue::CleanHighPrio(TracedCallback<Ptr<const Packet>, uint32_t> dropCb){
		while (m_ackQ->GetNPackets() > 0){
			Ptr<Packet> p = m_ackQ->Dequeue();
			dropCb(p, 0);
		}
	}

	/******************
	 * QbbNetDevice
	 *****************/
	NS_OBJECT_ENSURE_REGISTERED(QbbNetDevice);

	TypeId
		QbbNetDevice::GetTypeId(void)
	{
		static TypeId tid = TypeId("ns3::QbbNetDevice")
			.SetParent<PointToPointNetDevice>()
			.AddConstructor<QbbNetDevice>()
			.AddAttribute("QbbEnabled",
				"Enable the generation of PAUSE packet.",
				BooleanValue(true),
				MakeBooleanAccessor(&QbbNetDevice::m_qbbEnabled),
				MakeBooleanChecker())
			.AddAttribute("QcnEnabled",
				"Enable the generation of PAUSE packet.",
				BooleanValue(false),
				MakeBooleanAccessor(&QbbNetDevice::m_qcnEnabled),
				MakeBooleanChecker())
			.AddAttribute("DynamicThreshold",
				"Enable dynamic threshold.",
				BooleanValue(false),
				MakeBooleanAccessor(&QbbNetDevice::m_dynamicth),
				MakeBooleanChecker())
			.AddAttribute("PauseTime",
				"Number of microseconds to pause upon congestion",
				UintegerValue(5),
				MakeUintegerAccessor(&QbbNetDevice::m_pausetime),
				MakeUintegerChecker<uint32_t>())
			.AddAttribute ("TxBeQueue", 
					"A queue to use as the transmit queue in the device.",
					PointerValue (),
					MakePointerAccessor (&QbbNetDevice::m_queue),
					MakePointerChecker<PacketQueue> ())
			.AddAttribute ("RdmaEgressQueue", 
					"A queue to use as the transmit queue in the device.",
					PointerValue (),
					MakePointerAccessor (&QbbNetDevice::m_rdmaEQ),
					MakePointerChecker<Object> ())
			.AddAttribute ("NVLS_enable", 
					"enable NVLS",
					UintegerValue (0),
					MakeUintegerAccessor (&QbbNetDevice::nvls_enable),
					MakeUintegerChecker<uint32_t>())
			.AddTraceSource ("QbbEnqueue", "Enqueue a packet in the QbbNetDevice.",
					MakeTraceSourceAccessor (&QbbNetDevice::m_traceEnqueue),
					"ns3::QbbEnabled::QbbEnqueue")
			.AddTraceSource ("QbbDequeue", "Dequeue a packet in the QbbNetDevice.",
					MakeTraceSourceAccessor (&QbbNetDevice::m_traceDequeue),
					"ns3::QbbEnabled::QbbDequeue")
			.AddTraceSource ("QbbDrop", "Drop a packet in the QbbNetDevice.",
					MakeTraceSourceAccessor (&QbbNetDevice::m_traceDrop),
					"ns3::QbbEnabled::QbbDrop")
			.AddTraceSource ("RdmaQpDequeue", "A qp dequeue a packet.",
					MakeTraceSourceAccessor (&QbbNetDevice::m_traceQpDequeue),
					"ns3::QbbEnabled::Dequeue")
			.AddTraceSource ("QbbPfc", "get a PFC packet. 0: resume, 1: pause",
					MakeTraceSourceAccessor (&QbbNetDevice::m_tracePfc),
					"ns3::QbbEnabled::QbbPfc")
			;

		return tid;
	}

	QbbNetDevice::QbbNetDevice()
	  : m_calendarEnabled(false),
	    m_calendarEpochStart(Seconds(0)),
	    m_calendarSliceDuration(Seconds(0)),
	    m_calendarSwitchingTime(Seconds(0)),
	    m_calendarIngressLinkDelay(Seconds(0)),
	    m_calendarNumSlots(0)
	{
		NS_LOG_FUNCTION(this);
		m_ecn_source = new std::vector<ECNAccount>;
		for (uint32_t i = 0; i < qCnt; i++){
			m_paused[i] = false;
		}

		m_rdmaEQ = CreateObject<RdmaEgressQueue>();
	}

	QbbNetDevice::~QbbNetDevice()
	{
		NS_LOG_FUNCTION(this);
	}

	void
		QbbNetDevice::DoDispose()
	{
		NS_LOG_FUNCTION(this);

		// std::cout << "Do QbbNetDevice::DoDispose() function " << std::endl;

		if (!m_calendarEvent.IsExpired())
		{
			Simulator::Cancel(m_calendarEvent);
		}
		PointToPointNetDevice::DoDispose();
	}

	void
		QbbNetDevice::TransmitComplete(void)
	{
		// if(m_node->GetId() == 0) std::cout << "transmit complete at tick: " <<  Simulator::Now().GetNanoSeconds() << std::endl;
		NS_LOG_FUNCTION(this);
		NS_ASSERT_MSG(m_txMachineState == BUSY, "Must be BUSY if transmitting");
		m_txMachineState = READY;
		NS_ASSERT_MSG(m_currentPkt != 0, "QbbNetDevice::TransmitComplete(): m_currentPkt zero");
		m_phyTxEndTrace(m_currentPkt);
		m_currentPkt = 0;
		DequeueAndTransmit();
	}

	void QbbNetDevice::SwitchAsHostTransmitComplete(void) {
		NS_LOG_FUNCTION(this);
		NS_ASSERT_MSG(m_txMachineState == BUSY, "Must be BUSY if transmitting");
		m_txMachineState = READY;
		NS_ASSERT_MSG(m_currentPkt != 0, "QbbNetDevice::TransmitComplete(): m_currentPkt zero");
		m_phyTxEndTrace(m_currentPkt);
		m_currentPkt = 0;
		SwitchAsHostSend();
	}

	void
		QbbNetDevice::DequeueAndTransmit(void)
	{
		NS_LOG_FUNCTION(this);
		// if(m_node->GetId() == 0) std::cout << "QP start send at tick: " << Simulator::Now().GetNanoSeconds() << std::endl;
		if (!m_linkUp) return; // if link is down, return
		if (m_txMachineState == BUSY) return;	// Quit if channel busy
		Ptr<Packet> p;
		if (m_node->GetNodeType() == 0 || (m_node->GetNodeType() == 2 && nvls_enable == 1)){
			int qIndex = m_rdmaEQ->GetNextQindex(m_paused);
			if (qIndex != -1024){
				if (qIndex == -1){ // high prio
					p = m_rdmaEQ->DequeueQindex(qIndex);
					
					m_traceDequeue(p, 0);
					// update statistics for monitor
					m_rdmaUpdateTxBytes(m_ifIndex, p->GetSize());
					TransmitStart(p);
					return;
				}
				// a qp dequeue a packet
				Ptr<RdmaQueuePair> lastQp = m_rdmaEQ->GetQp(qIndex);
				p = m_rdmaEQ->DequeueQindex(qIndex);
				// update statistics for monitor
				m_rdmaUpdateTxBytes(m_ifIndex, p->GetSize());
				// transmit
				m_traceQpDequeue(p, lastQp);
				TransmitStart(p);

				// update for the next avail time
				m_rdmaPktSent(lastQp, p, m_tInterframeGap);
			}else { // no packet to send
				NS_LOG_INFO("PAUSE/gate prohibits send at node " << m_node->GetId());
				Time t = Simulator::GetMaximumSimulationTime();
				Time now = Simulator::Now();
				for (uint32_t i = 0; i < m_rdmaEQ->GetFlowCount(); i++){
					Ptr<RdmaQueuePair> qp = m_rdmaEQ->GetQp(i);
					if (qp->m_nextAvail > now){
						t = Min(qp->m_nextAvail, t);
					}
				}
				Time gateWake = m_rdmaEQ->GetNextGateWake();
				if (gateWake > now){
					t = Min(gateWake, t);
				}
				if (t < Simulator::GetMaximumSimulationTime() && t > now &&
					(m_nextSend.IsExpired() || t < Time(m_nextSend.GetTs()))){
					if (!m_nextSend.IsExpired()){
						Simulator::Cancel(m_nextSend);
					}
					m_nextSend = Simulator::Schedule(t - now, &QbbNetDevice::DequeueAndTransmit, this);
				}
			}
			return;
		}else{   //switch data plane
			SwitchDequeueAndTransmit();
		}
		return;
	}

	void QbbNetDevice::SwitchAsHostSend(void) {
		NS_LOG_FUNCTION(this);
		if (!m_linkUp) return; // if link is down, return
		if (m_txMachineState == BUSY) return;	// Quit if channel busy
		Ptr<Packet> p;
		int qIndex = m_rdmaEQ->GetNextQindex(m_paused);
		if (qIndex != -1024){
			if (qIndex == -1){ // high prio
				p = m_rdmaEQ->DequeueQindex(qIndex);
				m_traceDequeue(p, 0);
				// update statistics for monitor
				m_rdmaUpdateTxBytes(m_ifIndex, p->GetSize());
				SwitchAsHostTransmitStart(p);
				return;
			}
			// a qp dequeue a packet
			Ptr<RdmaQueuePair> lastQp = m_rdmaEQ->GetQp(qIndex);
			NS_ASSERT_MSG(lastQp->nvls_enable == 1 && m_node->GetNodeType() == 2, "Switch as host send must with NVLS ON!");
			p = m_rdmaEQ->DequeueQindex(qIndex);
			// update statistics for monitor
			m_rdmaUpdateTxBytes(m_ifIndex, p->GetSize());
			// transmit
			m_traceQpDequeue(p, lastQp);
			SwitchAsHostTransmitStart(p);

			// update for the next avail time
			m_rdmaPktSent(lastQp, p, m_tInterframeGap);
		}else { // no packet to send
			NS_LOG_INFO("PAUSE prohibits send at node " << m_node->GetId());
			Time t = Simulator::GetMaximumSimulationTime();
			Time now = Simulator::Now();
			for (uint32_t i = 0; i < m_rdmaEQ->GetFlowCount(); i++){
				Ptr<RdmaQueuePair> qp = m_rdmaEQ->GetQp(i);
				if (qp->m_nextAvail > now){
					t = Min(qp->m_nextAvail, t);
				}
			}
			Time gateWake = m_rdmaEQ->GetNextGateWake();
			if (gateWake > now){
				t = Min(gateWake, t);
			}
			if (m_nextSend.IsExpired() && t < Simulator::GetMaximumSimulationTime() && t > now){
				m_nextSend = Simulator::Schedule(t - now, &QbbNetDevice::SwitchAsHostSend, this);
			}
		}
		return;
	}

	void
	QbbNetDevice::ConfigureCalendar(Time epochStart,
	                                Time sliceDuration,
	                                Time switchingTime,
	                                uint32_t numSlots,
	                                Time ingressLinkDelay)
	{
		NS_ASSERT_MSG(m_queue != 0, "QbbNetDevice queue must exist before calendar configuration");
		NS_ASSERT_MSG(numSlots > 0, "Calendar requires at least one slot");
		NS_ASSERT_MSG(sliceDuration.GetTimeStep() > 0, "Calendar slice duration must be positive");
		NS_ASSERT_MSG(switchingTime < sliceDuration,
		              "Calendar switching time must be smaller than slice duration");

		if (m_calendarEnabled)
		{
			NS_ASSERT_MSG(m_calendarEpochStart == epochStart &&
			              m_calendarSliceDuration == sliceDuration &&
			              m_calendarSwitchingTime == switchingTime &&
			              m_calendarIngressLinkDelay == ingressLinkDelay &&
			              m_calendarNumSlots == numSlots,
			              "Conflicting calendar configuration on one egress port");
			return;
		}

		m_calendarEnabled = true;
		m_calendarEpochStart = epochStart;
		m_calendarSliceDuration = sliceDuration;
		m_calendarSwitchingTime = switchingTime;
		m_calendarIngressLinkDelay = ingressLinkDelay;
		m_calendarNumSlots = numSlots;
		m_queue->ConfigureCalendar(numSlots);
		RefreshCalendarState(false);
	}

	bool
	QbbNetDevice::IsCalendarEnabled() const
	{
		return m_calendarEnabled;
	}

	uint32_t
	QbbNetDevice::GetCalendarLookupSlot(uint32_t packetBytes) const
	{
		NS_ASSERT_MSG(m_calendarEnabled, "Calendar lookup requested on an unconfigured port");
		NS_ASSERT_MSG(m_calendarNumSlots > 0, "Calendar has no slots");

		Time now = Simulator::Now();
		if (now < m_calendarEpochStart)
		{
			return 0;
		}

		uint64_t sliceTicks = m_calendarSliceDuration.GetTimeStep();
		uint64_t elapsed = (now - m_calendarEpochStart).GetTimeStep();
		uint64_t offset = elapsed % sliceTicks;
		uint32_t slot = static_cast<uint32_t>((elapsed / sliceTicks) % m_calendarNumSlots);
		uint64_t activeTicks = sliceTicks - m_calendarSwitchingTime.GetTimeStep();
		Time required = m_bps.CalculateBytesTxTime(packetBytes) +
		                m_calendarIngressLinkDelay + TimeStep(1);
		NS_ASSERT_MSG(required.GetTimeStep() <= activeTicks,
		              "A packet cannot reach the OCS inside one active calendar window");

		if (offset < activeTicks &&
		    required.GetTimeStep() <= activeTicks - offset)
		{
			return slot;
		}

		return (slot + 1) % m_calendarNumSlots;
	}

	void
	QbbNetDevice::RefreshCalendarState(bool triggerTransmit)
	{
		if (!m_calendarEnabled)
		{
			return;
		}

		if (!m_calendarEvent.IsExpired())
		{
			Simulator::Cancel(m_calendarEvent);
		}

		Time now = Simulator::Now();
		int32_t activeSlot = -1;
		Time nextBoundary;

		if (now < m_calendarEpochStart)
		{
			nextBoundary = m_calendarEpochStart;
		}
		else
		{
			uint64_t sliceTicks = m_calendarSliceDuration.GetTimeStep();
			uint64_t elapsed = (now - m_calendarEpochStart).GetTimeStep();
			uint64_t offset = elapsed % sliceTicks;
			uint32_t slot = static_cast<uint32_t>((elapsed / sliceTicks) % m_calendarNumSlots);
			uint64_t activeTicks = sliceTicks - m_calendarSwitchingTime.GetTimeStep();

			if (offset < activeTicks)
			{
				activeSlot = static_cast<int32_t>(slot);
				nextBoundary = now + TimeStep(activeTicks - offset);
			}
			else
			{
				nextBoundary = now + TimeStep(sliceTicks - offset);
			}
		}

		int32_t oldSlot = m_queue->GetActiveCalendarSlot();
		m_queue->SetActiveCalendarSlot(activeSlot);

		if (nextBoundary > now)
		{
			m_calendarEvent = Simulator::Schedule(nextBoundary - now,
			                                      &QbbNetDevice::HandleCalendarBoundary,
			                                      this);
		}

		if (triggerTransmit && activeSlot >= 0 && activeSlot != oldSlot)
		{
			SwitchDequeueAndTransmit();
		}
	}

	void
	QbbNetDevice::HandleCalendarBoundary()
	{
		RefreshCalendarState(true);
	}

	bool
	QbbNetDevice::CalendarPacketFits(Ptr<const Packet> packet) const
	{
		if (!m_calendarEnabled || m_queue->GetActiveCalendarSlot() < 0)
		{
			return false;
		}

		Time now = Simulator::Now();
		if (now < m_calendarEpochStart)
		{
			return false;
		}

		uint64_t sliceTicks = m_calendarSliceDuration.GetTimeStep();
		uint64_t elapsed = (now - m_calendarEpochStart).GetTimeStep();
		uint64_t offset = elapsed % sliceTicks;
		uint64_t activeTicks = sliceTicks - m_calendarSwitchingTime.GetTimeStep();
		if (offset >= activeTicks)
		{
			return false;
		}

		Time required = m_bps.CalculateBytesTxTime(packet->GetSize()) +
		                m_calendarIngressLinkDelay + TimeStep(1);
		return required.GetTimeStep() <= activeTicks - offset;
	}

	void QbbNetDevice::SwitchDequeueAndTransmit(void) {
		NS_LOG_FUNCTION(this);
		if (!m_linkUp) return;
		if (m_txMachineState == BUSY) return;

		if (m_calendarEnabled)
		{
			RefreshCalendarState(false);
		}

		uint32_t peekQ = 0;
		bool fromCalendar = false;
		Ptr<const Packet> next = m_queue->PeekRR(m_paused, peekQ, fromCalendar);
		if (next == 0)
		{
			NS_LOG_INFO("PAUSE/time gate prohibits send at node " << m_node->GetId());
			return;
		}

		if (fromCalendar && !CalendarPacketFits(next))
		{
			return;
		}

		Ptr<Packet> p = m_queue->DequeueRR(m_paused);
		NS_ASSERT_MSG(p != 0, "Peeked switch packet could not be dequeued");
		m_snifferTrace(p);
		m_promiscSnifferTrace(p);

		FlowIdTag t;
		uint32_t qIndex = m_queue->GetLastQueue();
		m_node->SwitchNotifyDequeue(m_ifIndex, qIndex, p);
		p->RemovePacketTag(t);
		m_traceDequeue(p, qIndex);
		TransmitStart(p);
	}

	void
		QbbNetDevice::Resume(unsigned qIndex)
	{
		NS_LOG_FUNCTION(this << qIndex);
		NS_ASSERT_MSG(m_paused[qIndex], "Must be PAUSEd");
		m_paused[qIndex] = false;
		NS_LOG_INFO("Node " << m_node->GetId() << " dev " << m_ifIndex << " queue " << qIndex <<
			" resumed at " << Simulator::Now().GetSeconds());
		if (m_node->GetNodeType() == 1)
		{
			SwitchDequeueAndTransmit();
			return;
		}
		Ptr<RdmaQueuePair> lastQp = m_rdmaEQ->GetQp(qIndex);
		if(lastQp->nvls_enable == 1 && m_node->GetNodeType() == 2) SwitchAsHostSend(); 
		else DequeueAndTransmit();
	}

	void
		QbbNetDevice::Receive(Ptr<Packet> packet)
	{
		NS_LOG_FUNCTION(this << packet);
		if (!m_linkUp){
			m_traceDrop(packet, 0);
			return;
		}

		if (m_receiveErrorModel && m_receiveErrorModel->IsCorrupt(packet))
		{
			// 
			// If we have an error model and it indicates that it is time to lose a
			// corrupted packet, don't forward this packet up, let it go.
			//
			m_phyRxDropTrace(packet);
			return;
		}

		m_macRxTrace(packet);
		CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
		ch.getInt = 1; // parse INT header
		packet->PeekHeader(ch);
		if (ch.l3Prot == 0xFE){ // PFC
			if (!m_qbbEnabled) return;
			unsigned qIndex = ch.pfc.qIndex;
			if (ch.pfc.time > 0){
				m_tracePfc(1);
				m_paused[qIndex] = true;
			}else{
				m_tracePfc(0);
				Resume(qIndex);
			}
		}else { // non-PFC packets (data, ACK, NACK, CNP...)
			uint32_t sip = ch.sip;
			uint32_t sid = (sip >> 8) & 0xffff;
			uint32_t dip = ch.dip;
			uint32_t did = (dip >> 8) & 0xffff;

			// A host NIC must never consume a packet addressed to another node.
			// This matters for transparent OCS fabrics: during a different
			// circuit slice, an unaware sender's packet may physically arrive
			// at the wrong host. The optical fabric should still deliver the
			// signal, but the wrong host must silently discard the packet rather
			// than create RDMA RX state and generate ACK/NACK feedback.
			if (m_node->GetNodeType() == 0 &&
			    did != m_node->GetId()){
				m_traceDrop(packet, 0);
				return;
			}

			if (m_node->GetNodeType() > 0 && ch.m_tos != 4 && did != m_node->GetId()){ // switch
				// std::cout << "id: " << m_node->GetId() << " switch receive from " << sid << std::endl;
				packet->AddPacketTag(FlowIdTag(m_ifIndex));
				m_node->SwitchReceiveFromDevice(this, packet, ch);
			}else { // NIC
				// send to RdmaHw
				// std::cout << "id: " << m_node->GetId() << " NIC receive from " << sid << std::endl;
				if (ch.l3Prot == 0xFC) {
				}
				int ret = m_rdmaReceiveCb(this, packet, ch);
				// TODO we may based on the ret do something
			}
		}
		return;
	}

	bool QbbNetDevice::Send(Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber)
	{
		NS_ASSERT_MSG(false, "QbbNetDevice::Send not implemented yet\n");
		return false;
	}

	bool QbbNetDevice::SwitchSend (uint32_t qIndex, Ptr<Packet> packet, CustomHeader &ch){
		m_macTxTrace(packet);
		m_traceEnqueue(packet, qIndex);
		bool queued = m_queue->Enqueue(packet, qIndex);
		if (queued)
		{
			SwitchDequeueAndTransmit();
		}
		return queued;
	}

	bool QbbNetDevice::SwitchSend (uint32_t qIndex,
	                              Ptr<Packet> packet,
	                              CustomHeader &ch,
	                              uint32_t sendSlot){
		NS_ASSERT_MSG(m_calendarEnabled, "Calendar SwitchSend on an unconfigured port");
		m_macTxTrace(packet);
		m_traceEnqueue(packet, qIndex);
		bool queued = m_queue->EnqueueCalendar(packet, qIndex, sendSlot);
		if (queued)
		{
			SwitchDequeueAndTransmit();
		}
		return queued;
	}

	void QbbNetDevice::SendPfc(uint32_t qIndex, uint32_t type){
		Ptr<Packet> p = Create<Packet>(0);
		PauseHeader pauseh((type == 0 ? m_pausetime : 0), m_queue->GetNBytes(qIndex), qIndex);
		p->AddHeader(pauseh);
		Ipv4Header ipv4h;  // Prepare IPv4 header
		ipv4h.SetProtocol(0xFE);
		ipv4h.SetSource(m_node->GetObject<Ipv4>()->GetAddress(m_ifIndex, 0).GetLocal());
		ipv4h.SetDestination(Ipv4Address("255.255.255.255"));
		ipv4h.SetPayloadSize(p->GetSize());
		ipv4h.SetTtl(1);
		ipv4h.SetIdentification(UniformVariable(0, 65536).GetValue());
		p->AddHeader(ipv4h);
		AddHeader(p, 0x800);
		CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
		p->PeekHeader(ch);
		SwitchSend(0, p, ch);
	}

	Ptr<Packet> QbbNetDevice::NICSendPfc(uint32_t qIndex, uint32_t type) {
	Ptr<Packet> p = Create<Packet>(0);
	PauseHeader pauseh((type == 0 ? m_pausetime : 0), m_queue->GetNBytes(qIndex),
						qIndex);
	// std::cout << "m_pausetime " << pauseh.GetTime() << " " << pauseh.GetQIndex() <<std::endl;
	p->AddHeader(pauseh);
	Ipv4Header ipv4h;  // Prepare IPv4 header
	ipv4h.SetProtocol(0xFE);
	//ipv4h.SetProtocol(L3ProtType::kPFC);
	ipv4h.SetSource(
		m_node->GetObject<Ipv4>()->GetAddress(m_ifIndex, 0).GetLocal());
	ipv4h.SetDestination(Ipv4Address("255.255.255.255"));
	ipv4h.SetPayloadSize(p->GetSize());
	ipv4h.SetTtl(1);
	ipv4h.SetIdentification(UniformVariable(0, 65536).GetValue());
	p->AddHeader(ipv4h);
	AddHeader(p, 0x800);
	return p;
	}

	bool
		QbbNetDevice::Attach(Ptr<QbbChannel> ch)
	{
		NS_LOG_FUNCTION(this << &ch);
		m_channel = ch;
		m_channel->Attach(this);
		NotifyLinkUp();
		return true;
	}
    void QbbNetDevice::SendCallback(Ptr<Packet> packet) {
        CustomHeader ch(
                CustomHeader::L2_Header | CustomHeader::L3_Header |
                CustomHeader::L4_Header);
        packet->PeekHeader(ch);
        m_rdmaSentCb(packet, ch);
    }
	bool
		QbbNetDevice::TransmitStart(Ptr<Packet> p)
	{
		NS_LOG_FUNCTION(this << p);
		NS_LOG_LOGIC("UID is " << p->GetUid() << ")");
		//
		// This function is called to start the process of transmitting a packet.
		// We need to tell the channel that we've started wiggling the wire and
		// schedule an event that will be executed when the transmission is complete.
		//
		NS_ASSERT_MSG(m_txMachineState == READY, "Must be READY to transmit");
		if(m_txMachineState == READY){
			//std:://cout<<"must be ready to transmit\n";
		}
		m_txMachineState = BUSY;
		m_currentPkt = p;
		m_phyTxBeginTrace(m_currentPkt);
		Time txTime = m_bps.CalculateBytesTxTime(p->GetSize());
        //添加当前qp所要发送的最后一个packet txtime后回调 根据mtu
        //根据qpindex
        // 添加一个回调
		if(m_rdmaEQ!=nullptr&&m_rdmaEQ->m_qpGrp!=nullptr && m_node->GetNodeType() == 0){
			// int qIndex = m_rdmaEQ->GetNextQindex(m_paused);
			// if(qIndex != -1024) {
				// Ptr<RdmaQueuePair> lastQp = m_rdmaEQ->GetQp(qIndex);
				// std::cout<<" net: "<<this<<" QPindex: "<<qIndex<<" GetBytesLeft "<<lastQp->GetBytesLeft()<<" p->GetSize() "<<p->GetSize()<<std::endl;
				// std::cout<<" net: "<<this<<" p->GetSize() "<<p->GetSize()<<std::endl;
				// if(9000>=lastQp->GetBytesLeft()){
				if(p->GetSize()<9000&&p->GetSize()>60){	//增加判断当前packet是否是ack报文的逻辑。
				// if(lastQp->IsFinished()){s
					// Simulator::Schedule(txTime,&sendfinsh,this);
					CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
					// ch.getInt = 1; // parse INT header
					p->PeekHeader(ch);
					// std::cout<<" p->GetSize()>=lastQp->GetBytesLeft() "<<std::endl;
					Simulator::Schedule(txTime,&QbbNetDevice::SendCallback,this,p);
				}
			// }
        }
		Time txCompleteTime = txTime + m_tInterframeGap;
		NS_LOG_LOGIC("Schedule TransmitCompleteEvent in " << txCompleteTime.GetSeconds() << "sec");
		Simulator::Schedule(txCompleteTime, &QbbNetDevice::TransmitComplete, this);

		bool result = m_channel->TransmitStart(p, this, txTime);

		if (result == false)
		{
			m_phyTxDropTrace(p);
		}
		return result;
	}

	bool QbbNetDevice::SwitchAsHostTransmitStart(Ptr<Packet> p) {
		NS_LOG_FUNCTION(this << p);
		NS_LOG_LOGIC("UID is " << p->GetUid() << ")");
		//
		// This function is called to start the process of transmitting a packet.
		// We need to tell the channel that we've started wiggling the wire and
		// schedule an event that will be executed when the transmission is complete.
		//
		NS_ASSERT_MSG(m_txMachineState == READY, "Must be READY to transmit");
		if(m_txMachineState == READY){
			//std:://cout<<"must be ready to transmit\n";
		}
		m_txMachineState = BUSY;
		m_currentPkt = p;
		m_phyTxBeginTrace(m_currentPkt);
		Time txTime = m_bps.CalculateBytesTxTime(p->GetSize());
		if(m_rdmaEQ!=nullptr&&m_rdmaEQ->m_qpGrp!=nullptr && m_node->GetNodeType() == 2){
			// int qIndex = m_rdmaEQ->GetNextQindex(m_paused);
			// if(qIndex != -1024) {
				// Ptr<RdmaQueuePair> lastQp = m_rdmaEQ->GetQp(qIndex);
				// std::cout<<" net: "<<this<<" QPindex: "<<qIndex<<" GetBytesLeft "<<lastQp->GetBytesLeft()<<" p->GetSize() "<<p->GetSize()<<std::endl;
				// std::cout<<" net: "<<this<<" p->GetSize() "<<p->GetSize()<<std::endl;
				// if(9000>=lastQp->GetBytesLeft()){
				if(p->GetSize()<9000&&p->GetSize()>60){	//增加判断当前packet是否是ack报文的逻辑。
				// if(lastQp->IsFinished()){s
					// Simulator::Schedule(txTime,&sendfinsh,this);
					CustomHeader ch(CustomHeader::L2_Header | CustomHeader::L3_Header | CustomHeader::L4_Header);
					// ch.getInt = 1; // parse INT header
					p->PeekHeader(ch);
					// std::cout<<" p->GetSize()>=lastQp->GetBytesLeft() "<<std::endl;
					Simulator::Schedule(txTime,&QbbNetDevice::SendCallback,this,p);
				}
			// }
        }
		Time txCompleteTime = txTime + m_tInterframeGap;
		// std::cout << "txCompleteTime: " << txCompleteTime << std::endl;
		NS_LOG_LOGIC("Schedule TransmitCompleteEvent in " << txCompleteTime.GetSeconds() << "sec");
		Simulator::Schedule(txCompleteTime, &QbbNetDevice::SwitchAsHostTransmitComplete, this);

		bool result = m_channel->TransmitStart(p, this, txTime);

		if (result == false)
		{
			m_phyTxDropTrace(p);
		}
		return result;
	}

	Ptr<Channel>
		QbbNetDevice::GetChannel(void) const
	{
		return m_channel;
	}

   bool QbbNetDevice::IsQbb(void) const{
	   return true;
   }

   void QbbNetDevice::NewQp(Ptr<RdmaQueuePair> qp){
	   qp->m_nextAvail = Simulator::Now();
	   if(qp->nvls_enable == 1 && m_node->GetNodeType() == 2) SwitchAsHostSend();
	   else DequeueAndTransmit();
   }
   void QbbNetDevice::ReassignedQp(Ptr<RdmaQueuePair> qp){
	   DequeueAndTransmit();
   }
   void QbbNetDevice::TriggerTransmit(){
	   DequeueAndTransmit();
   }

	void QbbNetDevice::SetQueue(Ptr<BEgressQueue> q){
		NS_LOG_FUNCTION(this << q);
		m_queue = q;
	}

	Ptr<BEgressQueue> QbbNetDevice::GetQueue(){
		return m_queue;
	}

	Ptr<RdmaEgressQueue> QbbNetDevice::GetRdmaQueue(){
		return m_rdmaEQ;
	}

	void QbbNetDevice::RdmaEnqueueHighPrioQ(Ptr<Packet> p){
		m_traceEnqueue(p, 0);
		m_rdmaEQ->EnqueueHighPrioQ(p);
	}

	void QbbNetDevice::TakeDown(){
		// TODO: delete packets in the queue, set link down
		if (m_node->GetNodeType() == 0){
			// clean the high prio queue
			m_rdmaEQ->CleanHighPrio(m_traceDrop);
			// notify driver/RdmaHw that this link is down
			m_rdmaLinkDownCb(this);
		}else { // switch
			// clean the queue
			for (uint32_t i = 0; i < qCnt; i++)
				m_paused[i] = false;
			while (1){
				Ptr<Packet> p = m_queue->DequeueRR(m_paused);
				if (p == 0)
					 break;
				m_traceDrop(p, m_queue->GetLastQueue());
			}
			// TODO: Notify switch that this link is down
		}
		m_linkUp = false;
	}

	void QbbNetDevice::UpdateNextAvail(Time t){
		Time now = Simulator::Now();
		if (m_nextSend.IsExpired() || t < Time(m_nextSend.GetTs())){
			if (!m_nextSend.IsExpired()){
				Simulator::Cancel(m_nextSend);
			}
			Time delta = t < now ? Time(0) : t - now;
			m_nextSend = Simulator::Schedule(delta, &QbbNetDevice::DequeueAndTransmit, this);
		}
	}
} // namespace ns3
