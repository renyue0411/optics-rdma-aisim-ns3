#include "rdma-driver.h"

// MODE2_CQE_SEMANTICS_V1: bind userspace to WR completions, not ACK progress.
// MODE2_VERBS_COMPLETION_V2: userspace operation completion is owned by final signaled-WR CQE.
// MODE2_VERBS_ERROR_CQE_V1: Mode 2 reports CQE failure separately from success.

namespace ns3 {

/***********************
 * RdmaDriver
 **********************/
TypeId RdmaDriver::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaDriver")
		.SetParent<Object> ()
		.AddTraceSource ("QpComplete", "A qp completes.",
				MakeTraceSourceAccessor (&RdmaDriver::m_traceQpComplete),
				"ns3::RdmaDriver::QpComplete")
		.AddTraceSource ("QpError", "A userspace-visible QP operation fails.",
				MakeTraceSourceAccessor (&RdmaDriver::m_traceQpError),
				"ns3::RdmaDriver::QpError")
		.AddTraceSource(
				"SendComplete",
				"A qp Send completes.",
				MakeTraceSourceAccessor(&RdmaDriver::m_traceSendComplete),
				"ns3::RdmaDriver::SendComplete");
	return tid;
}

RdmaDriver::RdmaDriver(){
	m_injectionMode = INJECTION_DEFAULT;
}

void RdmaDriver::Init(void){
	Ptr<Ipv4> ipv4 = m_node->GetObject<Ipv4> ();
	#if 0
	m_rdma->m_nic.resize(ipv4->GetNInterfaces());
	for (uint32_t i = 0; i < m_rdma->m_nic.size(); i++){
		m_rdma->m_nic[i] = CreateObject<RdmaQueuePairGroup>();
		// share the queue pair group with NIC
		if (ipv4->GetNetDevice(i)->IsQbb()){
			DynamicCast<QbbNetDevice>(ipv4->GetNetDevice(i))->m_rdmaEQ->m_qpGrp = m_rdma->m_nic[i];
		}
	}
	#endif
	for (uint32_t i = 0; i < m_node->GetNDevices(); i++){
		Ptr<QbbNetDevice> dev = NULL;
		if (m_node->GetDevice(i)->IsQbb())
			dev = DynamicCast<QbbNetDevice>(m_node->GetDevice(i));
		m_rdma->m_nic.push_back(RdmaInterfaceMgr(dev));
		m_rdma->m_nic.back().qpGrp = CreateObject<RdmaQueuePairGroup>();
	}
	#if 0
	for (uint32_t i = 0; i < ipv4->GetNInterfaces (); i++){
		if (ipv4->GetNetDevice(i)->IsQbb() && ipv4->IsUp(i)){
			Ptr<QbbNetDevice> dev = DynamicCast<QbbNetDevice>(ipv4->GetNetDevice(i));
			// add a new RdmaInterfaceMgr for this device
			m_rdma->m_nic.push_back(RdmaInterfaceMgr(dev));
			m_rdma->m_nic.back().qpGrp = CreateObject<RdmaQueuePairGroup>();
		}
	}
	#endif
	// RdmaHw do setup
	m_rdma->SetNode(m_node);
    m_rdma->Setup(MakeCallback(&RdmaDriver::QpComplete, this),MakeCallback(&RdmaDriver::SendComplete, this));
	m_transport = CreateObject<RdmaTransport>();
	m_transport->SetNode(m_node);
	m_transport->SetRdmaHw(m_rdma);
	// Equivalent to a dedicated userspace CQ busy-poll loop. Keep the
	// callback at the driver boundary so Mode 2 can translate the final CQE
	// into the userspace-visible operation-complete event without exposing
	// RNIC ACK/PSN state to RdmaTransport.
	m_rdma->SetWrCompletionCallback(
		MakeCallback(&RdmaDriver::WrComplete, this));
}

void RdmaDriver::SetNode(Ptr<Node> node){
	m_node = node;
}

void RdmaDriver::SetRdmaHw(Ptr<RdmaHw> rdma){
	m_rdma = rdma;
}

void RdmaDriver::AddQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address sip, Ipv4Address dip, uint16_t sport, uint16_t dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent){
	if (m_injectionMode == INJECTION_USERSPACE){
		Ptr<RdmaQueuePair> qp = m_rdma->CreateQueuePair(src, dest, tag, size, pg, sip, dip, sport, dport, win, baseRtt, notifyAppFinish, notifyAppSent, 0);
		m_transport->RegisterQp(qp);
		return;
	}
	m_rdma->AddQueuePair(src, dest, tag, size, pg, sip, dip, sport, dport, win, baseRtt, notifyAppFinish, notifyAppSent);
}


void RdmaDriver::EnbaleNVLS() {
	m_rdma->enable_nvls();
}

void RdmaDriver::DisableNVLS() {
	m_rdma->disable_nvls();
}

void RdmaDriver::QpComplete(Ptr<RdmaQueuePair> q)
{
    if (m_transport != NULL)
    {
        m_transport->NotifyQpComplete(q);
    }

    // Mode 2 exposes completion at the verbs boundary: the final signaled WR
    // CQE, not the later RNIC-internal QP teardown callback. Modes 0/1 retain
    // the legacy QP-complete trace semantics.
    if (m_injectionMode != INJECTION_USERSPACE)
    {
        m_traceQpComplete(q);
    }
}

void RdmaDriver::WrComplete(Ptr<RdmaQueuePair> q,
                            uint64_t wrId,
                            uint64_t bytes,
                            uint64_t postTimeNs,
                            uint64_t completionTimeNs,
                            uint32_t cqeStatus)
{
    if (m_transport == NULL)
    {
        return;
    }

    const RdmaTransport::UserspaceCompletionResult result =
        m_transport->NotifyWrCompletion(
            q, wrId, bytes, postTimeNs, completionTimeNs, cqeStatus);
    if (m_injectionMode != INJECTION_USERSPACE)
    {
        return;
    }

    if (result == RdmaTransport::USERSPACE_COMPLETION_SUCCESS)
    {
        // Preserve the existing success trace/callback contract while making
        // its Mode-2 meaning match ibv_poll_cq(): the operation is complete
        // when the final signaled WR CQE has been observed by userspace.
        m_traceQpComplete(q);
    }
    else if (result == RdmaTransport::USERSPACE_COMPLETION_ERROR)
    {
        // Do not reuse the success trace for failures.  A real verbs consumer
        // distinguishes these outcomes using wc.status.
        m_traceQpError(q, cqeStatus);
    }
}

void RdmaDriver::SendComplete(Ptr<RdmaQueuePair> q){
    m_traceSendComplete(q);
}

void RdmaDriver::SetInjectionMode(uint32_t mode){
	NS_ASSERT_MSG(mode <= INJECTION_USERSPACE, "injection mode must be 0, 1, or 2");
	m_injectionMode = mode;

	if (m_transport != NULL){
		m_transport->SetMode(mode);
	}
}

void RdmaDriver::ConfigureTransport(uint64_t wrChunkBytes, uint64_t maxOutstandingBytes){
	NS_ASSERT(m_transport != NULL);
	m_transport->Configure(wrChunkBytes, maxOutstandingBytes);
}

Ptr<RdmaTransport> RdmaDriver::GetTransport() const{
	return m_transport;
}

} // namespace ns3
