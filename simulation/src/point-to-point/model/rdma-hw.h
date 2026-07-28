#ifndef RDMA_HW_H
#define RDMA_HW_H

#include <ns3/rdma.h>
#include <ns3/rdma-queue-pair.h>
#include <ns3/node.h>
#include <ns3/custom-header.h>
#include "qbb-net-device.h"
#include <unordered_map>
#include <set>
#include "pint.h"

namespace ns3 {

struct RdmaInterfaceMgr{
	// The QbbDevice dev contains many qps in qpGrp
	Ptr<QbbNetDevice> dev;
	Ptr<RdmaQueuePairGroup> qpGrp;

	RdmaInterfaceMgr() : dev(NULL), qpGrp(NULL) {}
	RdmaInterfaceMgr(Ptr<QbbNetDevice> _dev){
		dev = _dev;
	}
};

class RdmaHw : public Object {
public:

	static TypeId GetTypeId (void);
	RdmaHw();

	Ptr<Node> m_node;
	DataRate m_minRate;		//< Min sending rate
	uint32_t m_mtu;
	uint32_t m_cc_mode;
	double m_nack_interval;
	uint32_t m_chunk;
	uint32_t m_ack_interval;
	bool m_backto0;
	bool m_var_win, m_fast_react;
	bool m_rateBound;
	uint32_t m_total_pause_times; 
	uint32_t m_paused_times;
	std::vector<RdmaInterfaceMgr> m_nic; // list of running nic controlled by this RdmaHw
	std::unordered_map<uint64_t, Ptr<RdmaQueuePair> > m_qpMap; // mapping from uint64_t to qp
	std::unordered_map<uint64_t, Ptr<RdmaRxQueuePair> > m_rxQpMap; // mapping from uint64_t to rx qp
	std::unordered_map<uint32_t, std::vector<int> > m_rtTable; // map from ip address (u32) to possible ECMP port (index of dev)
	std::unordered_map<uint32_t, std::vector<int> > m_rtTable_nxthop_nvswitch;
	struct RnicInterfaceIdentity {
		uint32_t rnicPortId;
		uint32_t physicalNicId;
		uint32_t planeId;

		RnicInterfaceIdentity()
			: rnicPortId(0), physicalNicId(0), planeId(0) {}
		RnicInterfaceIdentity(uint32_t portId, uint32_t nicId, uint32_t plane)
			: rnicPortId(portId), physicalNicId(nicId), planeId(plane) {}
	};

	std::unordered_map<uint32_t, uint32_t> m_rnicPortToNicIdx; // stable RNIC port -> QbbNetDevice index
	std::unordered_map<uint32_t, uint32_t> m_nicIdxToRnicPort; // reverse mapping used when binding a QP
	std::unordered_map<uint32_t, RnicInterfaceIdentity> m_nicIdxToRnicIdentity;
	std::unordered_map<uint64_t, uint32_t> m_nicPlaneToNicIdx;

	enum ScaleOutPlaneScheduler {
		SCALE_OUT_HASH = 0,
		SCALE_OUT_ROUND_ROBIN = 1,
		SCALE_OUT_LEAST_QP = 2
	};
	uint32_t m_scaleOutPlaneScheduler;
	std::unordered_map<uint32_t, uint32_t> m_scaleOutRrCursor;

	typedef Callback<bool, Ptr<RdmaQueuePair> > RnicGateAllowCallback;
	typedef Callback<Time, Ptr<RdmaQueuePair> > RnicGateNextTimeCallback;
	RnicGateAllowCallback m_rnicGateAllowCallback;
	RnicGateNextTimeCallback m_rnicGateNextTimeCallback;
	uint32_t m_gpus_per_server; // uesed for routing; if src and dst in the same server, then communicate by nvswitch.
	uint32_t nvls_enable;
	std::set<uint32_t> nvswitch_set;

	// qp complete callback
	typedef Callback<void, Ptr<RdmaQueuePair> > QpCompleteCallback;
	QpCompleteCallback m_qpCompleteCallback;
    typedef Callback<void, Ptr<RdmaQueuePair> > SendCompleteCallback;
    SendCompleteCallback m_sendCompleteCallback;
	typedef Callback<void, Ptr<RdmaQueuePair> > QpProgressCallback;
	typedef Callback<void, Ptr<RdmaQueuePair> > QpRecoverCallback;
	QpProgressCallback m_qpProgressCallback;
	QpRecoverCallback m_qpRecoverCallback;

    // for monitor
	std::vector<uint64_t> tx_bytes; // <port_id, tx_bytes>
	std::unordered_map<uint64_t, uint32_t> qp_cnp; // key of qp ---> received cnp number
	std::vector<uint64_t> last_tx_bytes; // last sampling value <port_id, tx_bytes>
	std::unordered_map<uint64_t, uint32_t> last_qp_cnp; // last sampling value key of qp ---> received cnp number
	std::unordered_map<uint64_t, uint64_t> last_qp_rate; // last sampling value key of qp ---> sending rate
	void UpdateTxBytes(uint32_t port_id, uint64_t bytes);
	void PrintHostBW(FILE* bw_output, uint32_t bw_mon_interval);
	void PrintQPRate(FILE* rate_output);
	void PrintQPCnpNumber(FILE* cnp_output);

	// nvls
	void enable_nvls();
	void disable_nvls();
	void add_nvswitch(uint32_t nvswitch_id);

	void SetNode(Ptr<Node> node);
	static uint32_t EncodeRnicPortId(uint32_t physicalNicId, uint32_t planeId);
	static uint64_t MakeNicPlaneKey(uint32_t physicalNicId, uint32_t planeId);
	void RegisterRnicPort(uint32_t portId, uint32_t nicIdx); // legacy compatibility
	void RegisterRnicInterface(uint32_t portId,
	                           uint32_t physicalNicId,
	                           uint32_t planeId,
	                           uint32_t nicIdx);
	bool GetRnicPortId(uint32_t nicIdx, uint32_t &portId) const;
	bool GetRnicInterfaceIdentity(uint32_t nicIdx, RnicInterfaceIdentity &identity) const;
	bool GetNicIdxForNicPlane(uint32_t physicalNicId, uint32_t planeId, uint32_t &nicIdx) const;
	void Setup(QpCompleteCallback cb,SendCompleteCallback send_cb);
	void SetQpProgressCallback(QpProgressCallback cb);
	void SetQpRecoverCallback(QpRecoverCallback cb);
	void SetRnicGateCallbacks(RnicGateAllowCallback allowCb,
	                          RnicGateNextTimeCallback nextTimeCb);
	void ClearRnicGateCallbacks();
	bool RnicGateAllowsQp(Ptr<RdmaQueuePair> qp) const;
	Time GetNextRnicGateTime(Ptr<RdmaQueuePair> qp) const; // setup shared data and callbacks with the QbbNetDevice
	static uint64_t GetQpKey(uint32_t dip, uint16_t sport, uint16_t pg); // get the lookup key for m_qpMap
	Ptr<RdmaQueuePair> GetQp(uint32_t dip, uint16_t sport, uint16_t pg); // get the qp
	uint32_t GetNicIdxOfQp(Ptr<RdmaQueuePair> qp); // get the NIC index of the qp
	uint32_t SelectScaleOutNic(Ptr<RdmaQueuePair> qp, const std::vector<int> &candidates);
	uint32_t GetNicIdxForDevice(Ptr<QbbNetDevice> dev) const;
	void BindTxQpToNic(Ptr<RdmaQueuePair> qp, uint32_t nicIdx);
	void BindRxQpToIngress(Ptr<RdmaRxQueuePair> qp, uint32_t nicIdx);
	Ptr<RdmaQueuePair> CreateQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent, uint64_t initialPostedBytes);
	void PostWork(Ptr<RdmaQueuePair> qp, uint64_t bytes);
	void AddQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent); // add a new qp (new send)
	void DeleteQueuePair(Ptr<RdmaQueuePair> qp);

	Ptr<RdmaRxQueuePair> GetRxQp(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport, uint16_t pg, bool create); // get a rxQp
	uint32_t GetNicIdxOfRxQp(Ptr<RdmaRxQueuePair> q); // get the NIC index of the rxQp
	void DeleteRxQp(uint32_t dip, uint16_t pg, uint16_t dport);

	int ReceiveUdp(Ptr<QbbNetDevice> ingressDev, Ptr<Packet> p, CustomHeader &ch);
	int ReceiveCnp(Ptr<Packet> p, CustomHeader &ch);
	int ReceiveAck(Ptr<Packet> p, CustomHeader &ch); // handle both ACK and NACK
	int Receive(Ptr<QbbNetDevice> ingressDev, Ptr<Packet> p, CustomHeader &ch); // callback used by the receiving QbbNetDevice

	void PCIePause(uint32_t nic_idx, uint32_t qIndex);
	void PCIeResume(uint32_t nic_idx, uint32_t qIndex);
	void EnablePause();
	bool enable_pcie_pause; 

	void CheckandSendQCN(Ptr<RdmaRxQueuePair> q);
	int ReceiverCheckSeq(uint64_t seq, Ptr<RdmaRxQueuePair> q, uint32_t size);
	void AddHeader (Ptr<Packet> p, uint16_t protocolNumber);
	static uint16_t EtherToPpp (uint16_t protocol);

	void RecoverQueue(Ptr<RdmaQueuePair> qp);
	void QpComplete(Ptr<RdmaQueuePair> qp);
	void SetLinkDown(Ptr<QbbNetDevice> dev);

    int SendPacketComplete(Ptr<Packet> p, CustomHeader &ch);
    void SendComplete(Ptr<RdmaQueuePair> qp);

    // call this function after the NIC is setup
	void AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx, bool is_nvswitch);
	void ClearTable();
	void RedistributeQp();

	Ptr<Packet> GetNxtPacket(Ptr<RdmaQueuePair> qp); // get next packet to send, inc snd_nxt
	void PktSent(Ptr<RdmaQueuePair> qp, Ptr<Packet> pkt, Time interframeGap);
	void UpdateNextAvail(Ptr<RdmaQueuePair> qp, Time interframeGap, uint32_t pkt_size);
	void ChangeRate(Ptr<RdmaQueuePair> qp, DataRate new_rate);
	/******************************
	 * Mellanox's version of DCQCN
	 *****************************/
	double m_g; //feedback weight
	double m_rateOnFirstCNP; // the fraction of line rate to set on first CNP
	bool m_EcnClampTgtRate;
	double m_rpgTimeReset;
	double m_rateDecreaseInterval;
	uint32_t m_rpgThreshold;
	double m_alpha_resume_interval;
	DataRate m_rai;		//< Rate of additive increase
	DataRate m_rhai;		//< Rate of hyper-additive increase

	// the Mellanox's version of alpha update:
	// every fixed time slot, update alpha.
	void UpdateAlphaMlx(Ptr<RdmaQueuePair> q);
	void ScheduleUpdateAlphaMlx(Ptr<RdmaQueuePair> q);

	// Mellanox's version of CNP receive
	void cnp_received_mlx(Ptr<RdmaQueuePair> q);

	// Mellanox's version of rate decrease
	// It checks every m_rateDecreaseInterval if CNP arrived (m_decrease_cnp_arrived).
	// If so, decrease rate, and reset all rate increase related things
	void CheckRateDecreaseMlx(Ptr<RdmaQueuePair> q);
	void ScheduleDecreaseRateMlx(Ptr<RdmaQueuePair> q, uint32_t delta);

	// Mellanox's version of rate increase
	void RateIncEventTimerMlx(Ptr<RdmaQueuePair> q);
	void RateIncEventMlx(Ptr<RdmaQueuePair> q);
	void FastRecoveryMlx(Ptr<RdmaQueuePair> q);
	void ActiveIncreaseMlx(Ptr<RdmaQueuePair> q);
	void HyperIncreaseMlx(Ptr<RdmaQueuePair> q);

	/***********************
	 * High Precision CC
	 ***********************/
	double m_targetUtil;
	double m_utilHigh;
	uint32_t m_miThresh;
	bool m_multipleRate;
	bool m_sampleFeedback; // only react to feedback every RTT, or qlen > 0
	void HandleAckHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);
	void UpdateRateHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react);
	void UpdateRateHpTest(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react);
	void FastReactHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);

	/**********************
	 * TIMELY
	 *********************/
	double m_tmly_alpha, m_tmly_beta;
	uint64_t m_tmly_TLow, m_tmly_THigh, m_tmly_minRtt;
	void HandleAckTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);
	void UpdateRateTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool us);
	void FastReactTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);

	/**********************
	 * DCTCP
	 *********************/
	DataRate m_dctcp_rai;
	void HandleAckDctcp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);

	/*********************
	 * HPCC-PINT
	 ********************/
	uint32_t pint_smpl_thresh;
	void SetPintSmplThresh(double p);
	void HandleAckHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch);
	void UpdateRateHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react);
};

} /* namespace ns3 */

#endif /* RDMA_HW_H */
