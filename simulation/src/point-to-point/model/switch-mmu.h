#ifndef SWITCH_MMU_H
#define SWITCH_MMU_H

#include <unordered_map>
#include "ns3/random-variable-stream.h"
#include <ns3/node.h>
#include <ns3/abort.h>

namespace ns3 {

class Packet;

class SwitchMmu: public Object{
public:
	static const uint32_t pCnt = 1025;	// Number of ports used
	static const uint32_t qCnt = 8;	// Number of queues/priorities used

	static TypeId GetTypeId (void);

	SwitchMmu(void);
        void SetEcnStream(int64_t stream);
	// DCI_WAN_POOL_V1: WAN ingress shares a finite lossy pool; LAN PFC is retained.
	bool m_dciWanPool = false;
	bool m_dciWanPort[pCnt] = {};
	uint64_t m_dciDataUsed = 0, m_dciDataPeak = 0, m_dciAdmissionDrops = 0;
	void ConfigDciWanPort(uint32_t port) {
		NS_ABORT_MSG_IF(m_dciDataUsed != 0 || port == 0 || port >= pCnt,
		                "WAN pool must be configured before DATA arrives");
		m_dciWanPool = true; m_dciWanPort[port] = true; headroom[port] = 0;
	}
	uint64_t DciSharedCapacity() const {
		const uint64_t reserved = uint64_t(total_hdrm) + total_rsrv;
		return reserved < buffer_size ? uint64_t(buffer_size) - reserved : 0;
	}


	bool CheckIngressAdmission(uint32_t port, uint32_t qIndex, uint32_t psize);
	bool CheckEgressAdmission(uint32_t port, uint32_t qIndex, uint32_t psize);
	void UpdateIngressAdmission(uint32_t port, uint32_t qIndex, uint32_t psize);
	void UpdateEgressAdmission(uint32_t port, uint32_t qIndex, uint32_t psize);
	void RemoveFromIngressAdmission(uint32_t port, uint32_t qIndex, uint32_t psize);
	void RemoveFromEgressAdmission(uint32_t port, uint32_t qIndex, uint32_t psize);

	bool CheckShouldPause(uint32_t port, uint32_t qIndex);
	bool CheckShouldResume(uint32_t port, uint32_t qIndex);
	void SetPause(uint32_t port, uint32_t qIndex);
	void SetResume(uint32_t port, uint32_t qIndex);
	//void GetPauseClasses(uint32_t port, uint32_t qIndex);
	//bool GetResumeClasses(uint32_t port, uint32_t qIndex);

	uint32_t GetPfcThreshold(uint32_t port);
	uint32_t GetSharedUsed(uint32_t port, uint32_t qIndex);

	bool ShouldSendCN(uint32_t ifindex, uint32_t qIndex);

	void ConfigEcn(uint32_t port, uint32_t _kmin, uint32_t _kmax, double _pmax);
	void ConfigHdrm(uint32_t port, uint32_t size);
	void ConfigNPort(uint32_t n_port);
	void ConfigBufferSize(uint32_t size);

	// config
	uint32_t node_id;
	uint32_t buffer_size;
	uint32_t pfc_a_shift[pCnt];
	uint32_t reserve;
	uint32_t headroom[pCnt];
	uint32_t resume_offset;
	uint32_t kmin[pCnt], kmax[pCnt];
	double pmax[pCnt];
	uint32_t total_hdrm;
	uint32_t total_rsrv;

	// runtime
	uint32_t shared_used_bytes;
	uint32_t hdrm_bytes[pCnt][qCnt];
	uint32_t ingress_bytes[pCnt][qCnt];
	uint32_t paused[pCnt][qCnt];
	uint64_t egress_bytes[pCnt][qCnt];
private:
        // DCI_ECN_STREAM_V1
        Ptr<UniformRandomVariable> m_ecnRandom;
};

} /* namespace ns3 */

#endif /* SWITCH_MMU_H */

