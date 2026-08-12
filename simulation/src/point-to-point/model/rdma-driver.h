#ifndef RDMA_DRIVER_H
#define RDMA_DRIVER_H

#include <ns3/node.h>
#include <ns3/qbb-net-device.h>
#include <ns3/rdma.h>
#include <ns3/rdma-queue-pair.h>
#include <ns3/rdma-hw.h>
#include <vector>
#include <unordered_map>
#include <ns3/rdma-transport.h>

// MODE2_VERBS_COMPLETION_V2: userspace operation completion is owned by final signaled-WR CQE.
// MODE2_VERBS_ERROR_CQE_V1: expose a separate userspace-visible QP error trace.

namespace ns3 {

class RdmaDriver : public Object {
public:
	Ptr<Node> m_node;
	Ptr<RdmaHw> m_rdma;

	// trace
	TracedCallback<Ptr<RdmaQueuePair> > m_traceQpComplete;
	TracedCallback<Ptr<RdmaQueuePair>, uint32_t> m_traceQpError;
    TracedCallback<Ptr<RdmaQueuePair> > m_traceSendComplete;

    static TypeId GetTypeId (void);
	RdmaDriver();

	// This function init the m_nic according to the NetDevice
	// So this must be called after all NICs are installed
	void Init(void);

	// Set Node
	void SetNode(Ptr<Node> node);

	// Set RdmaHw
	void SetRdmaHw(Ptr<RdmaHw> rdma);

	// add a queue pair
	void AddQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent);

	// enable NVLS
	void EnbaleNVLS();
	void DisableNVLS();
	
	// RNIC teardown callback. In Mode 2, userspace-visible completion is
	// emitted earlier from the final signaled-WR CQE.
	void QpComplete(Ptr<RdmaQueuePair> q);
    void WrComplete(Ptr<RdmaQueuePair> q,
                    uint64_t wrId,
                    uint64_t bytes,
                    uint64_t postTimeNs,
                    uint64_t completionTimeNs,
                    uint32_t cqeStatus);
    void SendComplete(Ptr<RdmaQueuePair> q);

	enum InjectionMode{
		INJECTION_DEFAULT = 0,
		INJECTION_RNIC = 1,
		INJECTION_USERSPACE = 2
	};

	uint32_t m_injectionMode;
	Ptr<RdmaTransport> m_transport;

	void SetInjectionMode(uint32_t mode);
	void ConfigureTransport(uint64_t wrChunkBytes, uint64_t maxOutstandingBytes);
	Ptr<RdmaTransport> GetTransport() const;

};

} // namespace ns3

#endif /* RDMA_DRIVER_H */
