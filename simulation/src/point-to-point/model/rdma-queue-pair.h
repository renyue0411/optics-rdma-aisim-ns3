#ifndef RDMA_QUEUE_PAIR_H
#define RDMA_QUEUE_PAIR_H

#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/ipv4-address.h>
#include <ns3/data-rate.h>
#include <ns3/event-id.h>
#include <ns3/custom-header.h>
#include <ns3/int-header.h>
#include <vector>

// MODE1_CONTINUATION_ACK_RECOVERY_V1: bounded next-window DATA probing.
// MODE1_FINAL_ACK_RECOVERY_V1: receiver-window flush plus final-tail probing.
// MODE2_RNIC_RC_RETRY_V1: commodity-RNIC-style ACK timeout/retry state.
// MODE2_VERBS_ERROR_CQE_V1: expose RNIC failures only as CQE status to userspace.

namespace ns3 {

enum RdmaWrCqeStatus {
	RDMA_WR_CQE_SUCCESS = 0,
	RDMA_WR_CQE_RETRY_EXC_ERR = 1,
	RDMA_WR_CQE_WR_FLUSH_ERR = 2
};

enum RnicAckRecoveryProbeType {
	RNIC_ACK_PROBE_NONE = 0,
	RNIC_ACK_PROBE_CONTINUATION = 1,
	RNIC_ACK_PROBE_TAIL = 2
};

class RdmaQueuePair : public Object {
public:
	Time startTime;
	Ipv4Address sip, dip;
	uint16_t sport, dport;
	uint64_t m_size, m_init_size, m_tag;
	uint64_t m_postedLimit; // userspace-visible posted byte limit
	uint32_t m_src, m_dest;
	uint64_t snd_nxt, snd_una; // next seq to send, the highest unacked seq
	uint64_t m_highestSentSeq;
	uint64_t m_retransPackets;
	uint64_t m_retransBytes;
	uint64_t m_nackCount;
	uint64_t m_timeoutCount;

	// Mode-2 commodity-RNIC RC reliability state. Userspace never reads these
	// fields; it observes only the resulting WR CQEs. The timer is intentionally
	// schedule-unaware, matching a stock RNIC that does not know the OCS table.
	EventId m_rcAckTimeoutEvent;
	uint32_t m_rcRetryAttempts;
	uint64_t m_rcRetryAttemptCount;
	uint64_t m_rcRetryExhaustedCount;
	bool m_rcRetryExhausted;
	bool m_qpError;
	uint32_t m_qpErrorStatus;

	// Mode-1 ACK RTT estimator. One first-transmission sample is tracked per QP.
	bool m_deadlineSampleOutstanding;
	uint64_t m_deadlineSampleSeq;
	uint64_t m_deadlineSampleTxNs;
	uint64_t m_deadlineSampleWindowStartNs;
	uint64_t m_deadlineSampleWindowEndNs;
	uint64_t m_deadlineLastAllowedWindowStartNs;
	uint64_t m_deadlineLastAllowedWindowEndNs;
	uint64_t m_deadlineRejectedCrossWindowSamples;
	uint64_t m_deadlineSrttNs;
	uint64_t m_deadlineRttVarNs;
	uint32_t m_deadlineValidSamples;
	uint64_t m_deadlineCheckCount;
	uint64_t m_deadlineAllowedCheckCount;
	uint64_t m_deadlineBlockedCheckCount;
	uint64_t m_deadlineBlockEventCount;
	uint64_t m_deadlineLastBlockedWindowEndNs;
	uint64_t m_deadlineLastPacketSeq;
	uint64_t m_deadlineLastPacketEndSeq;
	bool m_deadlineLastPacketWasRetransmission;

	// Mode-1 schedule-aware ACK recovery.  These fields are RNIC-local QP
	// context; no packet buffering or switch-side state is introduced.
	bool m_ackRecoveryActive;
	bool m_ackRecoveryPermit;
	bool m_ackRecoveryProbeInFlight;
	RnicAckRecoveryProbeType m_ackRecoveryPermitType;
	RnicAckRecoveryProbeType m_ackRecoveryInFlightType;
	uint64_t m_ackRecoveryNextWindowStartNs;
	uint64_t m_ackRecoveryLastArmWindowEndNs;
	uint64_t m_ackRecoveryPermitWindowStartNs;
	uint64_t m_ackRecoveryLastProbeWindowStartNs;
	uint64_t m_ackRecoveryProbeStartSeq;
	uint64_t m_ackRecoveryProbeEndSeq;
	uint64_t m_ackRecoveryLastAckProgressNs;
	uint64_t m_ackRecoveryLastAckProgressSeq;
	uint32_t m_ackRecoveryAttempts;
	uint64_t m_ackRecoveryArmCount;
	uint64_t m_ackRecoveryProbeCount;
	uint64_t m_ackRecoveryTailProbeCount;
	uint64_t m_ackRecoveryPartialAckCount;
	uint64_t m_ackRecoverySuccessCount;
	uint64_t m_ackRecoveryDelayedAckCount;
	uint64_t m_ackRecoveryNackCount;
	uint64_t m_ackRecoveryExhaustedCount;
	uint64_t m_ackRecoveryNoContinuationCount;

	uint16_t m_pg;
	uint16_t m_ipid;
	uint32_t m_win; // bound of on-the-fly packets
	uint64_t m_baseRtt; // base RTT of this qp
	bool m_hasBoundRnicPort; // true after the QP is pinned to one scale-out interface
	uint32_t m_boundRnicPort; // internally encoded stable RNIC port id
	uint32_t m_boundNicIdx; // ns-3 QbbNetDevice/interface index
	uint32_t m_boundPhysicalNicId; // topology NIC id, local to the GPU/NPU node
	uint32_t m_boundPlaneId; // global scale-out plane id
	DataRate m_max_rate; // max rate
	bool m_var_win; // variable window size
	Time m_nextAvail;	//< Soonest time of next send
	uint32_t wp; // current window of packets
	uint32_t lastPktSize;
	Callback<void> m_notifyAppFinish;
	Callback<void> m_notifyAppSent;
	/******************************
	 * runtime states
	 *****************************/
	uint32_t nvls_enable;
	DataRate m_rate;	//< Current rate
	struct {
		DataRate m_targetRate;	//< Target rate
		EventId m_eventUpdateAlpha;
		double m_alpha;
		bool m_alpha_cnp_arrived; // indicate if CNP arrived in the last slot
		bool m_first_cnp; // indicate if the current CNP is the first CNP
		EventId m_eventDecreaseRate;
		bool m_decrease_cnp_arrived; // indicate if CNP arrived in the last slot
		uint32_t m_rpTimeStage;
		EventId m_rpTimer;
	} mlx;
	struct {
		uint64_t m_lastUpdateSeq;
		DataRate m_curRate;
		IntHop hop[IntHeader::maxHop];
		uint32_t keep[IntHeader::maxHop];
		uint32_t m_incStage;
		double m_lastGap;
		double u;
		struct {
			double u;
			DataRate Rc;
			uint32_t incStage;
		}hopState[IntHeader::maxHop];
	} hp;
	struct{
		uint64_t m_lastUpdateSeq;
		DataRate m_curRate;
		uint32_t m_incStage;
		uint64_t lastRtt;
		double rttDiff;
	} tmly;
	struct{
		uint64_t m_lastUpdateSeq;
		uint32_t m_caState;
		uint64_t m_highSeq; // when to exit cwr
		double m_alpha;
		uint32_t m_ecnCnt;
		uint32_t m_batchSizeOfAlpha;
	} dctcp;
	struct{
		uint64_t m_lastUpdateSeq;
		DataRate m_curRate;
		uint32_t m_incStage;
	}hpccPint;

	/***********
	 * methods
	 **********/
	static TypeId GetTypeId (void);
	RdmaQueuePair(uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport);
	void SetSize(uint64_t size);
	void SetPostedLimit(uint64_t limit);
	void AddPostedBytes(uint64_t bytes);
	uint64_t GetPostedLimit() const;
	uint64_t GetUnpostedBytes() const;
	uint64_t GetPostedOutstandingBytes() const;
	void SetWin(uint32_t win);
	void SetBaseRtt(uint64_t baseRtt);
	void SetVarWin(bool v);
	void SetAppNotifyCallback(Callback<void> notifyAppFinish);
	void SetAppSentCallback(Callback<void> notifyAppSent);

	uint64_t GetBytesLeft();
	uint64_t GetInitialSize();
	uint32_t GetSrc() const;
	uint32_t GetDest() const;
	uint64_t GetTag();
	void SetTag(uint64_t tag);void SetSrc(uint32_t src);void SetDest(uint32_t dest);void SetInitialSize(uint64_t size);
	uint32_t GetHash(void);
	void Acknowledge(uint64_t ack);
	uint64_t GetOnTheFly();
	bool IsWinBound();
	uint64_t GetWin(); // window size calculated from m_rate
	bool IsFinished();
	uint64_t HpGetCurWin(); // window size calculated from hp.m_curRate, used by HPCC
};

class RdmaRxQueuePair : public Object { // Rx side queue pair
public:
	struct ECNAccount{
		uint16_t qIndex;
		uint8_t ecnbits;
		uint16_t qfb;
		uint16_t total;

		ECNAccount() { memset(this, 0, sizeof(ECNAccount));}
	};
	ECNAccount m_ecn_source;
	uint32_t sip, dip;
	uint16_t sport, dport;
	uint16_t m_ipid;
	uint64_t ReceiverNextExpectedSeq;
	Time m_nackTimer;
	uint64_t m_lastAckGeneratedSeq;
	bool m_ackDirty;
	IntHeader m_lastAckIntHeader;
	uint8_t m_lastAckTos;
	bool m_ackCnpPending;
	bool m_hasAckMetadata;
	uint64_t m_ackGeneratedCount;
	uint64_t m_ackFlushCount;
	uint64_t m_duplicateAckCount;
	uint32_t m_lastNACK;
	bool m_hasBoundRnicPort; // same-plane ACK/NACK affinity is active
	uint32_t m_boundRnicPort;
	uint32_t m_boundNicIdx;
	uint32_t m_boundPhysicalNicId;
	uint32_t m_boundPlaneId;
	EventId QcnTimerEvent; // if destroy this rxQp, remember to cancel this timer

	static TypeId GetTypeId (void);
	RdmaRxQueuePair();
	uint32_t GetHash(void);
};

class RdmaQueuePairGroup : public Object {
public:
	std::vector<Ptr<RdmaQueuePair> > m_qps;
	//std::vector<Ptr<RdmaRxQueuePair> > m_rxQps;

	static TypeId GetTypeId (void);
	RdmaQueuePairGroup(void);
	uint32_t GetN(void);
	Ptr<RdmaQueuePair> Get(uint32_t idx);
	Ptr<RdmaQueuePair> operator[](uint32_t idx);
	void AddQp(Ptr<RdmaQueuePair> qp);
	//void AddRxQp(Ptr<RdmaRxQueuePair> rxQp);
	void Clear(void);
};

}

#endif /* RDMA_QUEUE_PAIR_H */
