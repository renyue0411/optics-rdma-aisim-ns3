#include <ns3/simulator.h>
#include <ns3/simple-seq-ts-header.h>
#include <ns3/udp-header.h>
#include <ns3/ipv4-header.h>
#include "ns3/ppp-header.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/data-rate.h"
#include "ns3/pointer.h"
#include "rdma-hw.h"
#include "ppp-header.h"
#include "qbb-header.h"
#include "cn-header.h"
#ifdef NS3_MTP
#include "ns3/mtp-interface.h"
#endif
#include <algorithm>
#include <iostream>	// debug
#include <limits>

// MODE2_CQE_SEMANTICS_V1: complete WR boundaries generate CQE callbacks.

namespace ns3{

const uint64_t RdmaHw::FLOW_RX_BUCKET_NS;
std::map<RdmaHw::FlowRxTraceKey, uint64_t> RdmaHw::m_flowRxBytes;
std::set<RdmaHw::FlowRxTraceKey> RdmaHw::m_flowRxScheduled;

bool
RdmaHw::FlowRxTraceKey::operator< (const FlowRxTraceKey &other) const
{
	if (bucketStartNs != other.bucketStartNs) return bucketStartNs < other.bucketStartNs;
	if (src != other.src) return src < other.src;
	if (dst != other.dst) return dst < other.dst;
	if (sport != other.sport) return sport < other.sport;
	if (dport != other.dport) return dport < other.dport;
	return pg < other.pg;
}

void
RdmaHw::RecordFlowRxBytes(uint32_t src,
                          uint32_t dst,
                          uint16_t sport,
                          uint16_t dport,
                          uint16_t pg,
                          uint32_t bytes)
{
	const uint64_t nowNs = static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
	FlowRxTraceKey key;
	key.bucketStartNs = (nowNs / FLOW_RX_BUCKET_NS) * FLOW_RX_BUCKET_NS;
	key.src = src;
	key.dst = dst;
	key.sport = sport;
	key.dport = dport;
	key.pg = pg;
	m_flowRxBytes[key] += bytes;

	if (m_flowRxScheduled.insert(key).second)
	{
		const uint64_t bucketEndNs = key.bucketStartNs + FLOW_RX_BUCKET_NS;
		const uint64_t delayNs = bucketEndNs > nowNs ? bucketEndNs - nowNs : 1;
		Simulator::Schedule(NanoSeconds(delayNs), &RdmaHw::FlushFlowRxBucket, key);
	}
}

void
RdmaHw::FlushFlowRxBucket(FlowRxTraceKey key)
{
	std::map<FlowRxTraceKey, uint64_t>::iterator it = m_flowRxBytes.find(key);
	if (it != m_flowRxBytes.end())
	{
		std::cout << "[FLOW_RX_BYTES]"
		          << " t=" << key.bucketStartNs
		          << " bucket_ns=" << FLOW_RX_BUCKET_NS
		          << " src=" << key.src
		          << " dst=" << key.dst
		          << " sport=" << key.sport
		          << " dport=" << key.dport
		          << " pg=" << key.pg
		          << " bytes=" << it->second
		          << std::endl;
		m_flowRxBytes.erase(it);
	}
	m_flowRxScheduled.erase(key);
}

TypeId RdmaHw::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaHw")
		.SetParent<Object> ()
		.AddAttribute("MinRate",
				"Minimum rate of a throttled flow",
				DataRateValue(DataRate("100Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_minRate),
				MakeDataRateChecker())
		.AddAttribute("Mtu",
				"Mtu.",
				UintegerValue(1000),
				MakeUintegerAccessor(&RdmaHw::m_mtu),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute ("CcMode",
				"which mode of DCQCN is running",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_cc_mode),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("NACKGenerationInterval",
				"The NACK Generation interval",
				DoubleValue(500.0),
				MakeDoubleAccessor(&RdmaHw::m_nack_interval),
				MakeDoubleChecker<double>())
		.AddAttribute("L2ChunkSize",
				"Layer 2 chunk size. Disable chunk mode if equals to 0.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_chunk),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("L2AckInterval",
				"Layer 2 Ack intervals. Disable ack if equals to 0.",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_ack_interval),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("L2BackToZero",
				"Layer 2 go back to zero transmission.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_backto0),
				MakeBooleanChecker())
		.AddAttribute("EwmaGain",
				"Control gain parameter which determines the level of rate decrease",
				DoubleValue(1.0 / 16),
				MakeDoubleAccessor(&RdmaHw::m_g),
				MakeDoubleChecker<double>())
		.AddAttribute ("RateOnFirstCnp",
				"the fraction of rate on first CNP",
				DoubleValue(1.0),
				MakeDoubleAccessor(&RdmaHw::m_rateOnFirstCNP),
				MakeDoubleChecker<double> ())
		.AddAttribute("ClampTargetRate",
				"Clamp target rate.",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_EcnClampTgtRate),
				MakeBooleanChecker())
		.AddAttribute("RPTimer",
				"The rate increase timer at RP in microseconds",
				DoubleValue(1500.0),
				MakeDoubleAccessor(&RdmaHw::m_rpgTimeReset),
				MakeDoubleChecker<double>())
		.AddAttribute("RateDecreaseInterval",
				"The interval of rate decrease check",
				DoubleValue(4.0),
				MakeDoubleAccessor(&RdmaHw::m_rateDecreaseInterval),
				MakeDoubleChecker<double>())
		.AddAttribute("FastRecoveryTimes",
				"The rate increase timer at RP",
				UintegerValue(5),
				MakeUintegerAccessor(&RdmaHw::m_rpgThreshold),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("AlphaResumInterval",
				"The interval of resuming alpha",
				DoubleValue(55.0),
				MakeDoubleAccessor(&RdmaHw::m_alpha_resume_interval),
				MakeDoubleChecker<double>())
		.AddAttribute("RateAI",
				"Rate increment unit in AI period",
				DataRateValue(DataRate("5Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_rai),
				MakeDataRateChecker())
		.AddAttribute("RateHAI",
				"Rate increment unit in hyperactive AI period",
				DataRateValue(DataRate("50Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_rhai),
				MakeDataRateChecker())
		.AddAttribute("VarWin",
				"Use variable window size or not",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_var_win),
				MakeBooleanChecker())
		.AddAttribute("FastReact",
				"Fast React to congestion feedback",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_fast_react),
				MakeBooleanChecker())
		.AddAttribute("MiThresh",
				"Threshold of number of consecutive AI before MI",
				UintegerValue(5),
				MakeUintegerAccessor(&RdmaHw::m_miThresh),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("TargetUtil",
				"The Target Utilization of the bottleneck bandwidth, by default 95%",
				DoubleValue(0.95),
				MakeDoubleAccessor(&RdmaHw::m_targetUtil),
				MakeDoubleChecker<double>())
		.AddAttribute("UtilHigh",
				"The upper bound of Target Utilization of the bottleneck bandwidth, by default 98%",
				DoubleValue(0.98),
				MakeDoubleAccessor(&RdmaHw::m_utilHigh),
				MakeDoubleChecker<double>())
		.AddAttribute("RateBound",
				"Bound packet sending by rate, for test only",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_rateBound),
				MakeBooleanChecker())
		.AddAttribute("MultiRate",
				"Maintain multiple rates in HPCC",
				BooleanValue(true),
				MakeBooleanAccessor(&RdmaHw::m_multipleRate),
				MakeBooleanChecker())
		.AddAttribute("SampleFeedback",
				"Whether sample feedback or not",
				BooleanValue(false),
				MakeBooleanAccessor(&RdmaHw::m_sampleFeedback),
				MakeBooleanChecker())
		.AddAttribute("TimelyAlpha",
				"Alpha of TIMELY",
				DoubleValue(0.875),
				MakeDoubleAccessor(&RdmaHw::m_tmly_alpha),
				MakeDoubleChecker<double>())
		.AddAttribute("TimelyBeta",
				"Beta of TIMELY",
				DoubleValue(0.8),
				MakeDoubleAccessor(&RdmaHw::m_tmly_beta),
				MakeDoubleChecker<double>())
		.AddAttribute("TimelyTLow",
				"TLow of TIMELY (ns)",
				UintegerValue(50000),
				MakeUintegerAccessor(&RdmaHw::m_tmly_TLow),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("TimelyTHigh",
				"THigh of TIMELY (ns)",
				UintegerValue(500000),
				MakeUintegerAccessor(&RdmaHw::m_tmly_THigh),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("TimelyMinRtt",
				"MinRtt of TIMELY (ns)",
				UintegerValue(20000),
				MakeUintegerAccessor(&RdmaHw::m_tmly_minRtt),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("DctcpRateAI",
				"DCTCP's Rate increment unit in AI period",
				DataRateValue(DataRate("1000Mb/s")),
				MakeDataRateAccessor(&RdmaHw::m_dctcp_rai),
				MakeDataRateChecker())
		.AddAttribute("PintSmplThresh",
				"PINT's sampling threshold in rand()%65536",
				UintegerValue(65536),
				MakeUintegerAccessor(&RdmaHw::pint_smpl_thresh),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("GPUsPerServer",
				"the number of gpus in a server, used for routing",
				UintegerValue(1),
				MakeUintegerAccessor(&RdmaHw::m_gpus_per_server),
				MakeUintegerChecker<uint32_t>())
		.AddAttribute("ScaleOutPlaneScheduler",
				"Scale-out plane scheduler: 0=hash, 1=round-robin, 2=least-QP, 3=time-hash",
				UintegerValue(SCALE_OUT_HASH),
				MakeUintegerAccessor(&RdmaHw::m_scaleOutPlaneScheduler),
				MakeUintegerChecker<uint32_t>(SCALE_OUT_HASH, SCALE_OUT_TIME_HASH))
		.AddAttribute("TotalPauseTimes",
				"The number of pause times to simulate PFC pause due to PCIe",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::m_total_pause_times),
				MakeUintegerChecker<uint64_t>())
		.AddAttribute("NVLS_enable",
				"NVLS enable info",
				UintegerValue(0),
				MakeUintegerAccessor(&RdmaHw::nvls_enable),
				MakeUintegerChecker<uint32_t>());
		;
	return tid;
}

RdmaHw::RdmaHw(){
	m_scaleOutPlaneScheduler = SCALE_OUT_HASH;
	m_rnicDeadlineEnabled = false;
	m_rnicDeadlinePipelineGuardNs = 0;
	m_rnicDeadlineClockGuardNs = 0;
	m_rnicDeadlineInitialGuardNs = 0;
	m_rnicDeadlineMinRttSamples = 4;
	m_rnicDeadlineRttVarMultiplier = 4;
}

void RdmaHw::enable_nvls() {
	nvls_enable = 1;
}

void RdmaHw::disable_nvls() {
	nvls_enable = 0;
}

void RdmaHw::add_nvswitch(uint32_t nvswitch_id) {
	nvswitch_set.insert(nvswitch_id);
}

void RdmaHw::SetNode(Ptr<Node> node){
	m_node = node;
}

uint32_t RdmaHw::EncodeRnicPortId(uint32_t physicalNicId, uint32_t planeId){
	NS_ASSERT_MSG(physicalNicId <= 0xffff, "physical NIC id exceeds 16-bit topology encoding");
	NS_ASSERT_MSG(planeId <= 0xffff, "plane id exceeds 16-bit topology encoding");
	return (physicalNicId << 16) | planeId;
}

uint64_t RdmaHw::MakeNicPlaneKey(uint32_t physicalNicId, uint32_t planeId){
	return (static_cast<uint64_t>(physicalNicId) << 32) | planeId;
}

void RdmaHw::RegisterRnicPort(uint32_t portId, uint32_t nicIdx){
	// Legacy integer topology ports are interpreted as NIC 0 / plane <portId>.
	RegisterRnicInterface(portId, 0, portId, nicIdx);
}

void RdmaHw::RegisterRnicInterface(uint32_t portId,
	                                uint32_t physicalNicId,
	                                uint32_t planeId,
	                                uint32_t nicIdx){
	NS_ASSERT_MSG(nicIdx < m_nic.size(), "RNIC interface maps to an invalid NIC index");
	NS_ASSERT_MSG(m_nic[nicIdx].dev != NULL, "RNIC interface must map to a QbbNetDevice");

	auto portIt = m_rnicPortToNicIdx.find(portId);
	if (portIt != m_rnicPortToNicIdx.end()){
		NS_ASSERT_MSG(portIt->second == nicIdx, "Duplicate RNIC port id maps to different NIC indices");
		return;
	}

	auto nicIt = m_nicIdxToRnicPort.find(nicIdx);
	NS_ASSERT_MSG(nicIt == m_nicIdxToRnicPort.end(), "A NIC index is already registered as another RNIC interface");

	uint64_t key = MakeNicPlaneKey(physicalNicId, planeId);
	auto endpointIt = m_nicPlaneToNicIdx.find(key);
	NS_ASSERT_MSG(endpointIt == m_nicPlaneToNicIdx.end(),
	              "Duplicate (physical NIC, plane) identity on one endpoint");

	m_rnicPortToNicIdx[portId] = nicIdx;
	m_nicIdxToRnicPort[nicIdx] = portId;
	m_nicIdxToRnicIdentity[nicIdx] = RnicInterfaceIdentity(portId, physicalNicId, planeId);
	m_nicPlaneToNicIdx[key] = nicIdx;

	std::cout << "[RNIC INTERFACE REGISTERED] node=" << m_node->GetId()
	          << " nic=" << physicalNicId
	          << " plane=" << planeId
	          << " rnic_port=" << portId
	          << " ifindex=" << nicIdx
	          << std::endl;
}

bool RdmaHw::GetRnicPortId(uint32_t nicIdx, uint32_t &portId) const{
	auto it = m_nicIdxToRnicPort.find(nicIdx);
	if (it == m_nicIdxToRnicPort.end()){
		return false;
	}
	portId = it->second;
	return true;
}

bool RdmaHw::GetRnicInterfaceIdentity(uint32_t nicIdx, RnicInterfaceIdentity &identity) const{
	auto it = m_nicIdxToRnicIdentity.find(nicIdx);
	if (it == m_nicIdxToRnicIdentity.end()){
		return false;
	}
	identity = it->second;
	return true;
}

bool RdmaHw::GetNicIdxForNicPlane(uint32_t physicalNicId, uint32_t planeId, uint32_t &nicIdx) const{
	auto it = m_nicPlaneToNicIdx.find(MakeNicPlaneKey(physicalNicId, planeId));
	if (it == m_nicPlaneToNicIdx.end()){
		return false;
	}
	nicIdx = it->second;
	return true;
}

void RdmaHw::ClearTimeHashReachability(){
	m_timeHashReachablePorts.clear();
}

void RdmaHw::AddTimeHashReachability(uint32_t dstNodeId, uint32_t rnicPortId){
	NS_ASSERT_MSG(m_rnicPortToNicIdx.find(rnicPortId) != m_rnicPortToNicIdx.end(),
	              "TIME HASH reachability references an unregistered RNIC port");
	m_timeHashReachablePorts[dstNodeId].insert(rnicPortId);
}

bool RdmaHw::IsTimeHashReachable(uint32_t dstNodeId, uint32_t rnicPortId) const{
	auto dstIt = m_timeHashReachablePorts.find(dstNodeId);
	if (dstIt == m_timeHashReachablePorts.end()){
		return false;
	}
	return dstIt->second.find(rnicPortId) != dstIt->second.end();
}

void RdmaHw::Setup(QpCompleteCallback cb,SendCompleteCallback send_cb){
	tx_bytes.resize(m_nic.size());
	last_tx_bytes.resize(m_nic.size());
	for (uint32_t i = 0; i < m_nic.size(); i++){
		tx_bytes[i] = 0;
		last_tx_bytes[i] = 0;
		Ptr<QbbNetDevice> dev = m_nic[i].dev;
		if (dev == NULL)
			continue;
		// share data with NIC
		dev->m_rdmaEQ->m_qpGrp = m_nic[i].qpGrp;
		// setup callback
		dev->m_rdmaReceiveCb = MakeCallback(&RdmaHw::Receive, this);
        dev->m_rdmaSentCb = MakeCallback(&RdmaHw::SendPacketComplete, this);
        dev->m_rdmaLinkDownCb = MakeCallback(&RdmaHw::SetLinkDown, this);
		dev->m_rdmaPktSent = MakeCallback(&RdmaHw::PktSent, this);
		dev->m_rdmaUpdateTxBytes = MakeCallback(&RdmaHw::UpdateTxBytes, this);
		// config NIC
		dev->m_rdmaEQ->m_rdmaGetNxtPkt = MakeCallback(&RdmaHw::GetNxtPacket, this);
		dev->m_rdmaEQ->m_rdmaGateAllowQp = MakeCallback(&RdmaHw::RnicGateAllowsQp, this);
		dev->m_rdmaEQ->m_rdmaGateNextTime = MakeCallback(&RdmaHw::GetNextRnicGateTime, this);
	}
	// setup qp complete callback
	m_qpCompleteCallback = cb;
    m_sendCompleteCallback = send_cb;
}

void RdmaHw::SetQpProgressCallback(QpProgressCallback cb){
	m_qpProgressCallback = cb;
}

void RdmaHw::SetQpRecoverCallback(QpRecoverCallback cb){
	m_qpRecoverCallback = cb;
}

void RdmaHw::SetWrCompletionCallback(WrCompletionCallback cb){
    m_wrCompletionCallback = cb;
}

void RdmaHw::GenerateWrCompletions(Ptr<RdmaQueuePair> qp){
    if (qp == NULL){
        return;
    }

    const uint64_t key = GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg);
    std::map<uint64_t, std::deque<PostedWrRecord> >::iterator it =
        m_postedWrRecords.find(key);
    if (it == m_postedWrRecords.end()){
        return;
    }

    std::deque<PostedWrRecord>& records = it->second;
    const uint64_t completionTimeNs =
        static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());

    while (!records.empty() && records.front().endSeq <= qp->snd_una){
        const PostedWrRecord record = records.front();
        records.pop_front();
        if (!m_wrCompletionCallback.IsNull()){
            m_wrCompletionCallback(
                qp,
                record.wrId,
                record.bytes,
                record.postTimeNs,
                completionTimeNs);
        }
    }

    if (records.empty()){
        m_postedWrRecords.erase(it);
        if (qp->IsFinished()){
            m_nextWrId.erase(key);
        }
    }
}

void RdmaHw::SetRnicGateCallbacks(RnicGateAllowCallback allowCb,
                                      RnicGateNextTimeCallback nextTimeCb){
	m_rnicGateAllowCallback = allowCb;
	m_rnicGateNextTimeCallback = nextTimeCb;
}

void RdmaHw::ClearRnicGateCallbacks(){
	m_rnicGateAllowCallback = RnicGateAllowCallback();
	m_rnicGateNextTimeCallback = RnicGateNextTimeCallback();
}

void RdmaHw::ConfigureRnicDeadline(bool enabled,
                                      uint64_t pipelineGuardNs,
                                      uint64_t clockGuardNs,
                                      uint64_t initialGuardNs,
                                      uint32_t minRttSamples,
                                      uint32_t rttVarMultiplier){
	m_rnicDeadlineEnabled = enabled;
	m_rnicDeadlinePipelineGuardNs = pipelineGuardNs;
	m_rnicDeadlineClockGuardNs = clockGuardNs;
	m_rnicDeadlineInitialGuardNs = initialGuardNs;
	m_rnicDeadlineMinRttSamples = std::max<uint32_t>(1, minRttSamples);
	m_rnicDeadlineRttVarMultiplier = std::max<uint32_t>(1, rttVarMultiplier);

	if (m_rnicDeadlineEnabled){
		std::cout << "[RNIC DEADLINE CONFIG]"
		          << " node=" << (m_node != NULL ? static_cast<int64_t>(m_node->GetId()) : -1)
		          << " pipeline_guard_ns=" << m_rnicDeadlinePipelineGuardNs
		          << " clock_guard_ns=" << m_rnicDeadlineClockGuardNs
		          << " initial_guard_ns=" << m_rnicDeadlineInitialGuardNs
		          << " min_rtt_samples=" << m_rnicDeadlineMinRttSamples
		          << " rttvar_multiplier=" << m_rnicDeadlineRttVarMultiplier
		          << std::endl;
	}
}

bool RdmaHw::IsRnicDeadlineEnabled() const{
	return m_rnicDeadlineEnabled;
}

uint64_t RdmaHw::GetRnicDeadlineReserveNs(Ptr<RdmaQueuePair> qp) const{
	if (!m_rnicDeadlineEnabled || qp == NULL){
		return 0;
	}

	uint64_t rttBoundNs;
	if (qp->m_deadlineValidSamples < m_rnicDeadlineMinRttSamples){
		rttBoundNs = qp->m_baseRtt + m_rnicDeadlineInitialGuardNs;
	}else{
		const uint64_t multiplier =
			static_cast<uint64_t>(m_rnicDeadlineRttVarMultiplier);
		const uint64_t variationReserve =
			qp->m_deadlineRttVarNs >
				std::numeric_limits<uint64_t>::max() / multiplier
				? std::numeric_limits<uint64_t>::max()
				: qp->m_deadlineRttVarNs * multiplier;
		rttBoundNs = variationReserve >
			std::numeric_limits<uint64_t>::max() - qp->m_deadlineSrttNs
			? std::numeric_limits<uint64_t>::max()
			: qp->m_deadlineSrttNs + variationReserve;
	}

	return rttBoundNs +
		m_rnicDeadlinePipelineGuardNs +
		m_rnicDeadlineClockGuardNs;
}

void RdmaHw::UpdateRnicDeadlineRtt(Ptr<RdmaQueuePair> qp, uint64_t ackSeq){
	if (!m_rnicDeadlineEnabled || qp == NULL ||
		!qp->m_deadlineSampleOutstanding ||
		ackSeq < qp->m_deadlineSampleSeq){
		return;
	}

	const uint64_t nowNs =
		static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
	if (nowNs >= qp->m_deadlineSampleTxNs){
		const uint64_t sampleNs = nowNs - qp->m_deadlineSampleTxNs;
		if (qp->m_deadlineValidSamples == 0){
			qp->m_deadlineSrttNs = sampleNs;
			qp->m_deadlineRttVarNs = sampleNs / 2;
		}else{
			const uint64_t deviation =
				qp->m_deadlineSrttNs > sampleNs
					? qp->m_deadlineSrttNs - sampleNs
					: sampleNs - qp->m_deadlineSrttNs;
			qp->m_deadlineRttVarNs =
				(3 * qp->m_deadlineRttVarNs + deviation) / 4;
			qp->m_deadlineSrttNs =
				(7 * qp->m_deadlineSrttNs + sampleNs) / 8;
		}
		qp->m_deadlineValidSamples++;
	}

	qp->m_deadlineSampleOutstanding = false;
}

void RdmaHw::InvalidateRnicDeadlineSample(Ptr<RdmaQueuePair> qp){
	if (qp != NULL){
		qp->m_deadlineSampleOutstanding = false;
	}
}

bool RdmaHw::RnicGateAllowsQp(Ptr<RdmaQueuePair> qp) const{
	if (m_rnicGateAllowCallback.IsNull()){
		return true;
	}
	return m_rnicGateAllowCallback(qp);
}

Time RdmaHw::GetNextRnicGateTime(Ptr<RdmaQueuePair> qp) const{
	if (m_rnicGateNextTimeCallback.IsNull()){
		return Simulator::Now();
	}
	return m_rnicGateNextTimeCallback(qp);
}

uint32_t RdmaHw::GetNicIdxForDevice(Ptr<QbbNetDevice> dev) const{
	NS_ASSERT_MSG(dev != NULL, "Cannot resolve a null QbbNetDevice");
	uint32_t ifIndex = dev->GetIfIndex();
	if (ifIndex < m_nic.size() && m_nic[ifIndex].dev == dev){
		return ifIndex;
	}
	for (uint32_t i = 0; i < m_nic.size(); ++i){
		if (m_nic[i].dev == dev){
			return i;
		}
	}
	NS_ASSERT_MSG(false, "Ingress QbbNetDevice is not registered in RdmaHw::m_nic");
	return 0;
}

void RdmaHw::BindTxQpToNic(Ptr<RdmaQueuePair> qp, uint32_t nicIdx){
	RnicInterfaceIdentity identity;
	if (!GetRnicInterfaceIdentity(nicIdx, identity)){
		return; // scale-up interface: retain the existing NVSwitch behavior
	}
	qp->m_hasBoundRnicPort = true;
	qp->m_boundRnicPort = identity.rnicPortId;
	qp->m_boundNicIdx = nicIdx;
	qp->m_boundPhysicalNicId = identity.physicalNicId;
	qp->m_boundPlaneId = identity.planeId;

	std::cout << "[QP PLANE BIND] node=" << m_node->GetId()
	          << " src=" << qp->m_src
	          << " dst=" << qp->m_dest
	          << " sport=" << qp->sport
	          << " nic=" << identity.physicalNicId
	          << " plane=" << identity.planeId
	          << " ifindex=" << nicIdx
	          << std::endl;
}

void RdmaHw::BindRxQpToIngress(Ptr<RdmaRxQueuePair> qp, uint32_t nicIdx){
	RnicInterfaceIdentity identity;
	if (!GetRnicInterfaceIdentity(nicIdx, identity)){
		return; // scale-up traffic continues to use the NVSwitch routing table
	}
	if (qp->m_hasBoundRnicPort){
		NS_ASSERT_MSG(qp->m_boundNicIdx == nicIdx,
		              "One RX QP received scale-out data from multiple planes");
		return;
	}
	qp->m_hasBoundRnicPort = true;
	qp->m_boundRnicPort = identity.rnicPortId;
	qp->m_boundNicIdx = nicIdx;
	qp->m_boundPhysicalNicId = identity.physicalNicId;
	qp->m_boundPlaneId = identity.planeId;
}

uint32_t RdmaHw::SelectScaleOutNic(Ptr<RdmaQueuePair> qp, const std::vector<int> &candidates){
	NS_ASSERT_MSG(!candidates.empty(), "Scale-out route has no live candidate interface");

	if (m_scaleOutPlaneScheduler == SCALE_OUT_HASH){
		return candidates[qp->GetHash() % candidates.size()];
	}

	if (m_scaleOutPlaneScheduler == SCALE_OUT_ROUND_ROBIN){
		uint32_t &cursor = m_scaleOutRrCursor[qp->dip.Get()];
		uint32_t offset = m_node != NULL ? m_node->GetId() : 0;
		uint32_t selected = candidates[(cursor + offset) % candidates.size()];
		++cursor;
		return selected;
	}

	if (m_scaleOutPlaneScheduler == SCALE_OUT_LEAST_QP){
		uint32_t selected = candidates[0];
		uint32_t selectedLoad = m_nic[selected].qpGrp->GetN();
		for (uint32_t i = 1; i < candidates.size(); ++i){
			uint32_t candidate = candidates[i];
			uint32_t load = m_nic[candidate].qpGrp->GetN();
			if (load < selectedLoad){
				selected = candidate;
				selectedLoad = load;
			}
		}
		return selected;
	}

	if (m_scaleOutPlaneScheduler == SCALE_OUT_TIME_HASH){
		std::vector<int> reachableCandidates;
		for (uint32_t i = 0; i < candidates.size(); ++i){
			uint32_t candidate = static_cast<uint32_t>(candidates[i]);
			uint32_t rnicPortId = 0;
			NS_ASSERT_MSG(GetRnicPortId(candidate, rnicPortId),
			              "TIME HASH candidate is not a registered scale-out RNIC interface");
			if (IsTimeHashReachable(qp->m_dest, rnicPortId)){
				reachableCandidates.push_back(candidates[i]);
			}
		}

		NS_ASSERT_MSG(!reachableCandidates.empty(),
		              "TIME HASH found no reachable plane for src=" << qp->m_src
		              << " dst=" << qp->m_dest
		              << " sport=" << qp->sport
		              << " candidates=" << candidates.size());

		uint32_t selected =
			static_cast<uint32_t>(reachableCandidates[qp->GetHash() % reachableCandidates.size()]);
		RnicInterfaceIdentity identity;
		NS_ASSERT_MSG(GetRnicInterfaceIdentity(selected, identity),
		              "TIME HASH selected NIC has no RNIC identity");

		std::cout << "[TIME HASH BIND] node=" << m_node->GetId()
		          << " src=" << qp->m_src
		          << " dst=" << qp->m_dest
		          << " sport=" << qp->sport
		          << " reachable_candidates=" << reachableCandidates.size()
		          << " rnic_port=" << identity.rnicPortId
		          << " nic=" << identity.physicalNicId
		          << " plane=" << identity.planeId
		          << " ifindex=" << selected
		          << std::endl;
		return selected;
	}

	NS_ASSERT_MSG(false, "Unknown scale-out plane scheduler");
	return candidates[0];
}

uint32_t RdmaHw::GetNicIdxOfQp(Ptr<RdmaQueuePair> qp){
	if (qp->m_hasBoundRnicPort){
		NS_ASSERT_MSG(qp->m_boundNicIdx < m_nic.size(), "QP is bound to an invalid RNIC NIC index");
		NS_ASSERT_MSG(m_nic[qp->m_boundNicIdx].dev != NULL, "QP is bound to an unavailable RNIC port");
		return qp->m_boundNicIdx;
	}

	uint32_t src = qp->m_src;
	uint32_t dst = qp->m_dest;
	if(src / m_gpus_per_server == dst / m_gpus_per_server || m_rtTable_nxthop_nvswitch.count(qp->dip.Get()) != 0){
		// Scale-up keeps the original NVSwitch hash behavior.
		auto &v = m_rtTable_nxthop_nvswitch[qp->dip.Get()];
		if (!v.empty()){
			return v[qp->GetHash() % v.size()];
		}
		NS_ASSERT_MSG(false, "Scale-up route has no live NIC");
	}else{
		// Scale-out uses a pluggable plane scheduler and is then pinned per QP.
		auto &v = m_rtTable[qp->dip.Get()];
		uint32_t nicIdx = SelectScaleOutNic(qp, v);
		BindTxQpToNic(qp, nicIdx);
		return nicIdx;
	}
	return 0;
}
uint64_t RdmaHw::GetQpKey(uint32_t dip, uint16_t sport, uint16_t pg){
	return ((uint64_t)dip << 32) | ((uint64_t)sport << 16) | (uint64_t)pg;
}
Ptr<RdmaQueuePair> RdmaHw::GetQp(uint32_t dip, uint16_t sport, uint16_t pg){
	uint64_t key = GetQpKey(dip, sport, pg);
	auto it = m_qpMap.find(key);
	if (it != m_qpMap.end())
		return it->second;
	return NULL;
}

Ptr<RdmaQueuePair> RdmaHw::CreateQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address sip, Ipv4Address dip, uint16_t sport, uint16_t dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent, uint64_t initialPostedBytes){
	NS_ASSERT_MSG(initialPostedBytes <= size, "initial posted bytes exceed flow size");

	Ptr<RdmaQueuePair> qp = CreateObject<RdmaQueuePair>(pg, sip, dip, sport, dport);
	qp->SetSrc(src);
	qp->SetDest(dest);
	qp->SetTag(tag);
	qp->SetSize(size);
	qp->SetInitialSize(size);
	qp->SetPostedLimit(initialPostedBytes);
	qp->SetWin(win);
	qp->SetBaseRtt(baseRtt);
	qp->SetVarWin(m_var_win);
	qp->SetAppNotifyCallback(notifyAppFinish);
	qp->SetAppSentCallback(notifyAppSent);

	uint32_t nic_idx = GetNicIdxOfQp(qp);
	m_nic[nic_idx].qpGrp->AddQp(qp);
	uint64_t key = GetQpKey(dip.Get(), sport, pg);
	m_qpMap[key] = qp;
	qp_cnp[key] = 0;
	last_qp_cnp[key] = 0;
	last_qp_rate[key] = 0;

	DataRate m_bps = m_nic[nic_idx].dev->GetDataRate();
	qp->m_rate = m_bps;
	qp->m_max_rate = m_bps;
	if (m_cc_mode == 1){
		qp->mlx.m_targetRate = m_bps;
	}else if (m_cc_mode == 3){
		qp->hp.m_curRate = m_bps;
		if (m_multipleRate){
			for (uint32_t i = 0; i < IntHeader::maxHop; i++)
				qp->hp.hopState[i].Rc = m_bps;
		}
	}else if (m_cc_mode == 7){
		qp->tmly.m_curRate = m_bps;
	}else if (m_cc_mode == 10){
		qp->hpccPint.m_curRate = m_bps;
	}

	if(nvls_enable == 1) qp->nvls_enable = 1;
	else qp->nvls_enable = 0;

	if (initialPostedBytes > 0){
		m_nic[nic_idx].dev->NewQp(qp);
	}
	return qp;
}

void RdmaHw::PostWork(Ptr<RdmaQueuePair> qp, uint64_t bytes){
	if (qp == NULL || bytes == 0){
		return;
	}

	uint64_t oldLimit = qp->GetPostedLimit();
	qp->AddPostedBytes(bytes);
	uint64_t newLimit = qp->GetPostedLimit();

	if (newLimit == oldLimit){
		return;
	}

    const uint64_t qpKey = GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg);
    PostedWrRecord record;
    record.wrId = ++m_nextWrId[qpKey];
    record.endSeq = newLimit;
    record.bytes = newLimit - oldLimit;
    record.postTimeNs =
        static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
    m_postedWrRecords[qpKey].push_back(record);

	NS_ASSERT(qp->snd_una <= qp->snd_nxt);
	NS_ASSERT(qp->snd_nxt <= qp->GetPostedLimit());
	NS_ASSERT(qp->GetPostedLimit() <= qp->m_size);

	uint32_t nicIdx = GetNicIdxOfQp(qp);
	m_nic[nicIdx].dev->NewQp(qp);
}

void RdmaHw::AddQueuePair(uint32_t src, uint32_t dest, uint64_t tag, uint64_t size, uint16_t pg, Ipv4Address sip, Ipv4Address dip, uint16_t sport, uint16_t dport, uint32_t win, uint64_t baseRtt, Callback<void> notifyAppFinish, Callback<void> notifyAppSent){
	CreateQueuePair(src, dest, tag, size, pg, sip, dip, sport, dport, win, baseRtt, notifyAppFinish, notifyAppSent, size);
}


void RdmaHw::DeleteQueuePair(Ptr<RdmaQueuePair> qp){
	// remove qp from the m_qpMap
	uint64_t key = GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg);
	m_qpMap.erase(key);
	qp_cnp.erase(key);
	last_qp_cnp.erase(key);
	last_qp_rate.erase(key);
}

Ptr<RdmaRxQueuePair> RdmaHw::GetRxQp(uint32_t sip, uint32_t dip, uint16_t sport, uint16_t dport, uint16_t pg, bool create){
    uint64_t key = ((uint64_t)dip << 32) | ((uint64_t)pg << 16) | (uint64_t)dport;
    #ifdef NS3_MTP
    MtpInterface::explicitCriticalSection cs;
    #endif
    auto it = m_rxQpMap.find(key);
    if (it != m_rxQpMap.end()){
        #ifdef NS3_MTP
        cs.ExitSection();
        #endif
        return it->second;
    }
    if (create){
        // create new rx qp
        Ptr<RdmaRxQueuePair> q = CreateObject<RdmaRxQueuePair>();
        // init the qp
        q->sip = sip;
        q->dip = dip;
        q->sport = sport;
        q->dport = dport;
        q->m_ecn_source.qIndex = pg;
        // store in map
        m_rxQpMap[key] = q;
        #ifdef NS3_MTP
        cs.ExitSection();
        #endif
        return q;
    }
    #ifdef NS3_MTP
    cs.ExitSection();
    #endif
    return NULL;
}

uint32_t RdmaHw::GetNicIdxOfRxQp(Ptr<RdmaRxQueuePair> q){
	if (q->m_hasBoundRnicPort){
		NS_ASSERT_MSG(q->m_boundNicIdx < m_nic.size(), "RX QP is bound to an invalid RNIC interface");
		return q->m_boundNicIdx;
	}

	uint32_t remoteNode =
		(Ipv4Address(q->dip).Get() >> 8) & 0xffff;
	bool sameServer = (m_node->GetId() / m_gpus_per_server == remoteNode / m_gpus_per_server);
	if (sameServer || m_rtTable_nxthop_nvswitch.count(q->dip) != 0){
		auto &v = m_rtTable_nxthop_nvswitch[q->dip];
		if (!v.empty()){
			return v[q->GetHash() % v.size()];
		}
	}

	auto it = m_rtTable.find(q->dip);
	if (it != m_rtTable.end() && !it->second.empty()){
		// Legacy fallback for a flow that was not bound from an ingress plane.
		return it->second[q->GetHash() % it->second.size()];
	}

	NS_ASSERT_MSG(false, "RX QP has no reverse route");
	return 0;
}
void RdmaHw::DeleteRxQp(uint32_t dip, uint16_t pg, uint16_t dport){
	uint64_t key = ((uint64_t)dip << 32) | ((uint64_t)pg << 16) | (uint64_t)dport;
	m_rxQpMap.erase(key);
}

int RdmaHw::SendPacketComplete(Ptr<Packet> p, CustomHeader &ch)
{
	uint16_t qIndex = ch.udp.pg;
	uint16_t port = ch.udp.sport;
	uint32_t seq = ch.udp.seq;
	// uint8_t cnp = (ch.flags >> qbbHeader::FLAG_CNP) & 1;
	// int i;
	Ptr<RdmaQueuePair> qp = GetQp(ch.dip, port, qIndex);
	if (qp == NULL)
	{
		return 0;
	}
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	SendComplete(qp);
	return 0;
}

void RdmaHw::SendComplete(Ptr<RdmaQueuePair> qp)
{
	NS_ASSERT(!m_sendCompleteCallback.IsNull());

	m_sendCompleteCallback(qp);
}

int RdmaHw::ReceiveUdp(Ptr<QbbNetDevice> ingressDev, Ptr<Packet> p, CustomHeader &ch){
	uint8_t ecnbits = ch.GetIpv4EcnBits();
	
	uint32_t payload_size = p->GetSize() - ch.GetSerializedSize();
	// TODO find corresponding rx queue pair
	Ptr<RdmaRxQueuePair> rxQp = GetRxQp(ch.dip, ch.sip, ch.udp.dport, ch.udp.sport, ch.udp.pg, true);
	BindRxQpToIngress(rxQp, GetNicIdxForDevice(ingressDev));
	if (ecnbits != 0){
		rxQp->m_ecn_source.ecnbits |= ecnbits;
		rxQp->m_ecn_source.qfb++;
	}
	rxQp->m_ecn_source.total++;
	rxQp->m_milestone_rx = m_ack_interval;

	const bool acceptedNewData = (ch.udp.seq == rxQp->ReceiverNextExpectedSeq);
	int x = ReceiverCheckSeq(ch.udp.seq, rxQp, payload_size);
	if (acceptedNewData)
	{
		const uint32_t srcNode = (ch.sip >> 8) & 0xffff;
		const uint32_t dstNode = (ch.dip >> 8) & 0xffff;
		RecordFlowRxBytes(srcNode,
		                  dstNode,
		                  ch.udp.sport,
		                  ch.udp.dport,
		                  ch.udp.pg,
		                  payload_size);
	}
	if (x == 1 || x == 2){ //generate ACK or NACK
		qbbHeader seqh;
		seqh.SetSeq(rxQp->ReceiverNextExpectedSeq);
		seqh.SetPG(ch.udp.pg);
		seqh.SetSport(ch.udp.dport);
		seqh.SetDport(ch.udp.sport);
		seqh.SetIntHeader(ch.udp.ih);
		if (ecnbits)
			seqh.SetCnp();

		Ptr<Packet> newp = Create<Packet>(std::max(60-14-20-(int)seqh.GetSerializedSize(), 0));
		newp->AddHeader(seqh);

		Ipv4Header head;	// Prepare IPv4 header
		head.SetDestination(Ipv4Address(ch.sip));
		head.SetSource(Ipv4Address(ch.dip));
		head.SetProtocol(x == 1 ? 0xFC : 0xFD); //ack=0xFC nack=0xFD
		head.SetTtl(64);
		head.SetPayloadSize(newp->GetSize());
		head.SetIdentification(rxQp->m_ipid++);
		// GPU receives the packet and generate ACK with NVLS tag
		if(ch.m_tos == 4) head.SetTos(4);

		newp->AddHeader(head);
		AddHeader(newp, 0x800);	// Attach PPP header
		uint32_t sip = ch.sip;
		uint32_t sid = (sip >> 8) & 0xffff;
		uint32_t dip = ch.dip;
		uint32_t did = (dip >> 8) & 0xffff;
		// send
		uint32_t nic_idx = GetNicIdxOfRxQp(rxQp);
		m_nic[nic_idx].dev->RdmaEnqueueHighPrioQ(newp);
		// 发送给目标 NVSwitch 的报文
		if(did == m_node->GetId() && m_node->GetNodeType() == 2 && ch.m_tos == 4) m_nic[nic_idx].dev->SwitchAsHostSend();
		else m_nic[nic_idx].dev->TriggerTransmit();
	}
	return 0;
}

int RdmaHw::ReceiveCnp(Ptr<Packet> p, CustomHeader &ch){
	// QCN on NIC
	// This is a Congestion signal
	// Then, extract data from the congestion packet.
	// We assume, without verify, the packet is destinated to me
	uint32_t qIndex = ch.cnp.qIndex;
	if (qIndex == 1){		//DCTCP
		return 0;
	}
	uint16_t udpport = ch.cnp.fid; // corresponds to the sport
	uint8_t ecnbits = ch.cnp.ecnBits;
	uint16_t qfb = ch.cnp.qfb;
	uint16_t total = ch.cnp.total;

	uint32_t i;
	// get qp
	Ptr<RdmaQueuePair> qp = GetQp(ch.sip, udpport, qIndex);
	if (qp == NULL)
		std::cout << "ERROR: QCN NIC cannot find the flow\n";
	// get nic
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;

	if (qp->m_rate == 0)			//lazy initialization	
	{
		qp->m_rate = dev->GetDataRate();
		if (m_cc_mode == 1){
			qp->mlx.m_targetRate = dev->GetDataRate();
		}else if (m_cc_mode == 3){
			qp->hp.m_curRate = dev->GetDataRate();
			if (m_multipleRate){
				for (uint32_t i = 0; i < IntHeader::maxHop; i++)
					qp->hp.hopState[i].Rc = dev->GetDataRate();
			}
		}else if (m_cc_mode == 7){
			qp->tmly.m_curRate = dev->GetDataRate();
		}else if (m_cc_mode == 10){
			qp->hpccPint.m_curRate = dev->GetDataRate();
		}
	}
	return 0;
}

int RdmaHw::ReceiveAck(Ptr<Packet> p, CustomHeader &ch){
	uint16_t qIndex = ch.ack.pg;
	uint16_t port = ch.ack.dport;
	uint64_t seq = ch.ack.seq;
	uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;


	int i;
	Ptr<RdmaQueuePair> qp = GetQp(ch.sip, port, qIndex);
	if (qp == NULL){
		return 0;
	}

	uint32_t nic_idx = GetNicIdxOfQp(qp);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	if (m_ack_interval == 0)
		std::cout << "ERROR: shouldn't receive ack\n";
	else {
		const uint64_t effectiveAckSeq =
			m_backto0 ? (seq / m_chunk * m_chunk) : seq;
		if (ch.l3Prot == 0xFC){
			UpdateRnicDeadlineRtt(qp, effectiveAckSeq);
		}
		qp->Acknowledge(effectiveAckSeq);
		if (ch.l3Prot == 0xFC){
		GenerateWrCompletions(qp);
	}
		if (qp->IsFinished()){
			QpComplete(qp);
		}
	}
	if (ch.l3Prot == 0xFD) // NACK
	{
		qp->m_nackCount++;
		RecoverQueue(qp);
	}

	// handle cnp
	if (cnp){
		uint64_t key = GetQpKey(qp->dip.Get(), qp->sport, qp->m_pg);
		qp_cnp[key]++; // update for the number of cnp this qp has received
		if (m_cc_mode == 1){ // mlx version
			cnp_received_mlx(qp);
		} 
	}

	if (m_cc_mode == 3){
		HandleAckHp(qp, p, ch);
	}else if (m_cc_mode == 7){
		HandleAckTimely(qp, p, ch);
	}else if (m_cc_mode == 8){
		HandleAckDctcp(qp, p, ch);
	}else if (m_cc_mode == 10){
		HandleAckHpPint(qp, p, ch);
	}
	uint32_t sip = ch.sip;
	uint32_t sid = (sip >> 8) & 0xffff;
	uint32_t dip = ch.dip;
	uint32_t did = (dip >> 8) & 0xffff;
	// ACK may advance the on-the-fly window, allowing more packets to send
	if(did == m_node->GetId() && m_node->GetNodeType() == 2) m_nic[nic_idx].dev->SwitchAsHostSend();
	else m_nic[nic_idx].dev->TriggerTransmit();
	//std:://cout << "ack triggere transmitted\n";
	return 0;
}

int RdmaHw::Receive(Ptr<QbbNetDevice> ingressDev, Ptr<Packet> p, CustomHeader &ch){
	if (ch.l3Prot == 0x11){ // UDP
		ReceiveUdp(ingressDev, p, ch);
	}else if (ch.l3Prot == 0xFF){ // CNP
		ReceiveCnp(p, ch);
	}else if (ch.l3Prot == 0xFD){ // NACK
		ReceiveAck(p, ch);
	}else if (ch.l3Prot == 0xFC){ // ACK
		ReceiveAck(p, ch);
	}
	return 0;
}

int RdmaHw::ReceiverCheckSeq(uint64_t seq, Ptr<RdmaRxQueuePair> q, uint32_t size){
	uint64_t expected = q->ReceiverNextExpectedSeq;
	if (seq == expected){
		q->ReceiverNextExpectedSeq = expected + size;
		if (q->ReceiverNextExpectedSeq >= q->m_milestone_rx){
			q->m_milestone_rx += m_ack_interval;
			return 1; //Generate ACK
		}else if (q->ReceiverNextExpectedSeq % m_chunk == 0){
			return 1;
		}else {
			return 5;
		}
	} else if (seq > expected) {
		// Generate NACK
		if (Simulator::Now() >= q->m_nackTimer || q->m_lastNACK != expected){
			q->m_nackTimer = Simulator::Now() + MicroSeconds(m_nack_interval);
			q->m_lastNACK = expected;
			if (m_backto0){
				q->ReceiverNextExpectedSeq = q->ReceiverNextExpectedSeq / m_chunk*m_chunk;
			}
			return 2;
		}else
			return 4;
	}else {
		// Duplicate. 
		return 3;
	}
}
void RdmaHw::AddHeader (Ptr<Packet> p, uint16_t protocolNumber){
	PppHeader ppp;
	ppp.SetProtocol (EtherToPpp (protocolNumber));
	p->AddHeader (ppp);
}
uint16_t RdmaHw::EtherToPpp (uint16_t proto){
	switch(proto){
		case 0x0800: return 0x0021;   //IPv4
		case 0x86DD: return 0x0057;   //IPv6
		default: NS_ASSERT_MSG (false, "PPP Protocol number not defined!");
	}
	return 0;
}

void RdmaHw::RecoverQueue(Ptr<RdmaQueuePair> qp){
	InvalidateRnicDeadlineSample(qp);
	qp->snd_nxt = qp->snd_una;
}

void RdmaHw::QpComplete(Ptr<RdmaQueuePair> qp){
	NS_ASSERT(!m_qpCompleteCallback.IsNull());
	std::cout << "[RNIC RETRANSMISSION STATS]"
	          << " node=" << m_node->GetId()
	          << " src=" << qp->m_src
	          << " dst=" << qp->m_dest
	          << " sport=" << qp->sport
	          << " dport=" << qp->dport
	          << " pg=" << qp->m_pg
	          << " plane=" << (qp->m_hasBoundRnicPort ? static_cast<int64_t>(qp->m_boundPlaneId) : -1)
	          << " rnic_port=" << (qp->m_hasBoundRnicPort ? static_cast<int64_t>(qp->m_boundRnicPort) : -1)
	          << " retrans_packets=" << qp->m_retransPackets
	          << " retrans_bytes=" << qp->m_retransBytes
	          << " nack_count=" << qp->m_nackCount
	          << " timeout_count=" << qp->m_timeoutCount
	          << " deadline_samples=" << qp->m_deadlineValidSamples
	          << " deadline_srtt_ns=" << qp->m_deadlineSrttNs
	          << " deadline_rttvar_ns=" << qp->m_deadlineRttVarNs
	          << " deadline_reserve_ns=" << GetRnicDeadlineReserveNs(qp)
	          << " deadline_checks=" << qp->m_deadlineCheckCount
	          << " deadline_allowed_checks=" << qp->m_deadlineAllowedCheckCount
	          << " deadline_blocked_checks=" << qp->m_deadlineBlockedCheckCount
	          << " deadline_block_events=" << qp->m_deadlineBlockEventCount
	          << " deadline_last_blocked_window_end_ns="
	          << qp->m_deadlineLastBlockedWindowEndNs
	          << std::endl;
	if (m_cc_mode == 1){
		Simulator::Cancel(qp->mlx.m_eventUpdateAlpha);
		Simulator::Cancel(qp->mlx.m_eventDecreaseRate);
		Simulator::Cancel(qp->mlx.m_rpTimer);
	}

	// This callback will log info
	// It may also delete the rxQp on the receiver
	m_qpCompleteCallback(qp);

	qp->m_notifyAppFinish();

	// delete the qp
	DeleteQueuePair(qp);
}

void RdmaHw::SetLinkDown(Ptr<QbbNetDevice> dev){
	printf("RdmaHw: node:%u a link down\n", m_node->GetId());
}

void RdmaHw::AddTableEntry(Ipv4Address &dstAddr, uint32_t intf_idx, bool is_nvswitch){
	uint32_t dip = dstAddr.Get();
	if(is_nvswitch == false)
		m_rtTable[dip].push_back(intf_idx);
	else {
		m_rtTable_nxthop_nvswitch[dip].push_back(intf_idx);
	}
}

void RdmaHw::ClearTable(){
	m_rtTable.clear();
	m_rtTable_nxthop_nvswitch.clear();
}

void RdmaHw::RedistributeQp(){
	// clear old qpGrp
	for (uint32_t i = 0; i < m_nic.size(); i++){
		if (m_nic[i].dev == NULL)
			continue;
		m_nic[i].qpGrp->Clear();
	}

	// redistribute qp
	for (auto &it : m_qpMap){
		Ptr<RdmaQueuePair> qp = it.second;
		uint32_t nic_idx = GetNicIdxOfQp(qp);
		m_nic[nic_idx].qpGrp->AddQp(qp);
		// Notify Nic
		m_nic[nic_idx].dev->ReassignedQp(qp);
	}
}

Ptr<Packet> RdmaHw::GetNxtPacket(Ptr<RdmaQueuePair> qp){
	uint64_t payload_size = qp->GetBytesLeft();
	if ((uint64_t)m_mtu < payload_size)
		payload_size = m_mtu;
	Ptr<Packet> p = Create<Packet> ((uint32_t)payload_size);
	// add SimpleSeqTsHeader
	SimpleSeqTsHeader seqTs;
	seqTs.SetSeq (qp->snd_nxt);
	seqTs.SetPG (qp->m_pg);
	p->AddHeader (seqTs);
	// add udp header
	UdpHeader udpHeader;
	udpHeader.SetDestinationPort (qp->dport);
	udpHeader.SetSourcePort (qp->sport);
	p->AddHeader (udpHeader);
	// add ipv4 header
	Ipv4Header ipHeader;
	ipHeader.SetSource (qp->sip);
	ipHeader.SetDestination (qp->dip);
	ipHeader.SetProtocol (0x11);
	ipHeader.SetPayloadSize (p->GetSize());
	ipHeader.SetTtl (64);
	// nvls <-> ToS, ToS = 1 -> NVLS enable
	if(qp->nvls_enable == 1) ipHeader.SetTos (4);
	else ipHeader.SetTos (0);
	ipHeader.SetIdentification (qp->m_ipid);
	p->AddHeader(ipHeader);
	// add ppp header
	PppHeader ppp;
	ppp.SetProtocol (0x0021); // EtherToPpp(0x800), see point-to-point-net-device.cc
	p->AddHeader (ppp);

	// Record whether this generated packet is a first transmission. PktSent()
	// uses this metadata at the actual RNIC transmission point.
	const uint64_t packetSeq = qp->snd_nxt;
	qp->m_deadlineLastPacketSeq = packetSeq;
	qp->m_deadlineLastPacketEndSeq = packetSeq + payload_size;
	qp->m_deadlineLastPacketWasRetransmission =
		(packetSeq < qp->m_highestSentSeq);

	// update retransmission accounting before advancing snd_nxt.
	if (packetSeq < qp->m_highestSentSeq)
	{
		qp->m_retransPackets++;
		qp->m_retransBytes += std::min<uint64_t>(payload_size,
		                                              qp->m_highestSentSeq - packetSeq);
	}
	qp->m_highestSentSeq = std::max<uint64_t>(qp->m_highestSentSeq,
	                                          packetSeq + payload_size);

	// update state
	qp->snd_nxt += payload_size;
	// std::cout << "current snd_nxt is: " << qp->snd_nxt << ", the window is: " << qp->m_win << std::endl;
	qp->m_ipid++;

	// return
	return p;
}

void RdmaHw::PktSent(Ptr<RdmaQueuePair> qp, Ptr<Packet> pkt, Time interframeGap){
	qp->lastPktSize = pkt->GetSize();

	if (m_rnicDeadlineEnabled &&
		!qp->m_deadlineSampleOutstanding &&
		!qp->m_deadlineLastPacketWasRetransmission){
		qp->m_deadlineSampleOutstanding = true;
		qp->m_deadlineSampleSeq = qp->m_deadlineLastPacketEndSeq;
		qp->m_deadlineSampleTxNs =
			static_cast<uint64_t>(Simulator::Now().GetNanoSeconds());
	}

	UpdateNextAvail(qp, interframeGap, pkt->GetSize());
}

void RdmaHw::UpdateNextAvail(Ptr<RdmaQueuePair> qp, Time interframeGap, uint32_t pkt_size){
	Time sendingTime;
	if (m_rateBound)
		sendingTime = interframeGap + qp->m_rate.CalculateBytesTxTime(pkt_size);
	else
		sendingTime = interframeGap + qp->m_max_rate.CalculateBytesTxTime(pkt_size);
	qp->m_nextAvail = Simulator::Now() + sendingTime;
}

void RdmaHw::ChangeRate(Ptr<RdmaQueuePair> qp, DataRate new_rate){
	#if 1
	Time sendingTime = qp->m_rate.CalculateBytesTxTime(qp->lastPktSize);
	Time new_sendintTime = new_rate.CalculateBytesTxTime(qp->lastPktSize);
	qp->m_nextAvail = qp->m_nextAvail + new_sendintTime - sendingTime;
	// update nic's next avail event
	uint32_t nic_idx = GetNicIdxOfQp(qp);
	m_nic[nic_idx].dev->UpdateNextAvail(qp->m_nextAvail);
	#endif

	// change to new rate
	qp->m_rate = new_rate;
}
/**
 * when nic send a packet, update the bytes it has sent
*/
void RdmaHw::UpdateTxBytes(uint32_t port_id, uint64_t bytes){
	tx_bytes[port_id] += bytes;
}
/**
 * output format:
 * time, host_id, port_id, bandwidth
*/
void RdmaHw::PrintHostBW(FILE* bw_output, uint32_t bw_mon_interval){
	for(int i = 0; i < m_nic.size(); ++i){
		if(tx_bytes[i] == last_tx_bytes[i]){
			continue;
		}
		double bw = (tx_bytes[i] - last_tx_bytes[i]) * 8 * 1e6 / (bw_mon_interval); // bit/s
		bw = bw*1.0 / 1e9; // Gbps
		fprintf(bw_output, "%lu, %u, %u, %f\n", Simulator::Now().GetTimeStep(), m_node->GetId(), i, bw);
		fflush(bw_output);
		last_tx_bytes[i] = tx_bytes[i];
	}
}
/**
 * output format:
 * time, src, dst, sport, dport, size, rate
*/
void RdmaHw::PrintQPRate(FILE* rate_output){
	std::unordered_map<uint64_t, Ptr<RdmaQueuePair>>::iterator it = m_qpMap.begin();
	for(; it != m_qpMap.end(); it++){
		Ptr<RdmaQueuePair> qp = it->second;
		uint64_t key = it->first;
		if(qp->m_rate.GetBitRate() == last_qp_rate[key]){
			continue;
		}
		fprintf(rate_output, "%lu, %u, %u, %u, %u, %u, %u\n", Simulator::Now().GetTimeStep(), qp->m_src, qp->m_dest, qp->sport, qp->dport, qp->m_size, qp->m_rate.GetBitRate());
		fflush(rate_output);
		last_qp_rate[key] = qp->m_rate.GetBitRate();
	}
}
/**
 * output format:
 * time, src, dst, sport, dport, size, cnp_number
*/
void RdmaHw::PrintQPCnpNumber(FILE* cnp_output){
	std::unordered_map<uint64_t, Ptr<RdmaQueuePair>>::iterator it = m_qpMap.begin();
	for(; it != m_qpMap.end(); it++){
		Ptr<RdmaQueuePair> qp = it->second;
		uint64_t key = it->first;
		if(qp_cnp[key] != last_qp_cnp[key]){
			fprintf(cnp_output, "%lu, %u, %u, %u, %u, %u, %u\n", Simulator::Now().GetTimeStep(), qp->m_src, qp->m_dest, qp->sport, qp->dport, qp->m_size, qp_cnp[key]);
			fflush(cnp_output);
			last_qp_cnp[key] = qp_cnp[key];
		}
	}
}
#define PRINT_LOG 0
/******************************
 * Mellanox's version of DCQCN
 *****************************/
void RdmaHw::UpdateAlphaMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	//printf("%lu alpha update: %08x %08x %u %u %.6lf->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_alpha);
	#endif
	if (q->mlx.m_alpha_cnp_arrived){
		q->mlx.m_alpha = (1 - m_g)*q->mlx.m_alpha + m_g; 	//binary feedback
	}else {
		q->mlx.m_alpha = (1 - m_g)*q->mlx.m_alpha; 	//binary feedback
	}
	#if PRINT_LOG
	//printf("%.6lf\n", q->mlx.m_alpha);
	#endif
	q->mlx.m_alpha_cnp_arrived = false; // clear the CNP_arrived bit
	ScheduleUpdateAlphaMlx(q);
}
void RdmaHw::ScheduleUpdateAlphaMlx(Ptr<RdmaQueuePair> q){
	q->mlx.m_eventUpdateAlpha = Simulator::Schedule(MicroSeconds(m_alpha_resume_interval), &RdmaHw::UpdateAlphaMlx, this, q);
}

void RdmaHw::cnp_received_mlx(Ptr<RdmaQueuePair> q){
	q->mlx.m_alpha_cnp_arrived = true; // set CNP_arrived bit for alpha update
	q->mlx.m_decrease_cnp_arrived = true; // set CNP_arrived bit for rate decrease
	if (q->mlx.m_first_cnp){
		// init alpha
		q->mlx.m_alpha = 1;
		q->mlx.m_alpha_cnp_arrived = false;
		// schedule alpha update
		ScheduleUpdateAlphaMlx(q);
		// schedule rate decrease
		ScheduleDecreaseRateMlx(q, 1); // add 1 ns to make sure rate decrease is after alpha update
		// set rate on first CNP
		q->mlx.m_targetRate = q->m_rate = m_rateOnFirstCNP * q->m_rate;
		q->mlx.m_first_cnp = false;
	}
}

void RdmaHw::CheckRateDecreaseMlx(Ptr<RdmaQueuePair> q){
	ScheduleDecreaseRateMlx(q, 0);
	if (q->mlx.m_decrease_cnp_arrived){
		#if PRINT_LOG
		printf("%lu rate dec: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
		#endif
		bool clamp = true;
		if (!m_EcnClampTgtRate){
			if (q->mlx.m_rpTimeStage == 0)
				clamp = false;
		}
		if (clamp)
			q->mlx.m_targetRate = q->m_rate;
		q->m_rate = std::max(m_minRate, q->m_rate * (1 - q->mlx.m_alpha / 2));
		// reset rate increase related things
		q->mlx.m_rpTimeStage = 0;
		q->mlx.m_decrease_cnp_arrived = false;
		Simulator::Cancel(q->mlx.m_rpTimer);
		q->mlx.m_rpTimer = Simulator::Schedule(MicroSeconds(m_rpgTimeReset), &RdmaHw::RateIncEventTimerMlx, this, q);
		#if PRINT_LOG
		printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
		#endif
	}
}
void RdmaHw::ScheduleDecreaseRateMlx(Ptr<RdmaQueuePair> q, uint32_t delta){
	q->mlx.m_eventDecreaseRate = Simulator::Schedule(MicroSeconds(m_rateDecreaseInterval) + NanoSeconds(delta), &RdmaHw::CheckRateDecreaseMlx, this, q);
}

void RdmaHw::RateIncEventTimerMlx(Ptr<RdmaQueuePair> q){
	q->mlx.m_rpTimer = Simulator::Schedule(MicroSeconds(m_rpgTimeReset), &RdmaHw::RateIncEventTimerMlx, this, q);
	RateIncEventMlx(q);
	q->mlx.m_rpTimeStage++;
}
void RdmaHw::RateIncEventMlx(Ptr<RdmaQueuePair> q){
	// check which increase phase: fast recovery, active increase, hyper increase
	if (q->mlx.m_rpTimeStage < m_rpgThreshold){ // fast recovery
		FastRecoveryMlx(q);
	}else if (q->mlx.m_rpTimeStage == m_rpgThreshold){ // active increase
		ActiveIncreaseMlx(q);
	}else { // hyper increase
		HyperIncreaseMlx(q);
	}
}

void RdmaHw::FastRecoveryMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	printf("%lu fast recovery: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
	q->m_rate = (q->m_rate / 2) + (q->mlx.m_targetRate / 2);
	#if PRINT_LOG
	printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
}
void RdmaHw::ActiveIncreaseMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	printf("%lu active inc: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
	// get NIC
	uint32_t nic_idx = GetNicIdxOfQp(q);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	// increate rate
	q->mlx.m_targetRate += m_rai;
	if (q->mlx.m_targetRate > dev->GetDataRate())
		q->mlx.m_targetRate = dev->GetDataRate();
	q->m_rate = (q->m_rate / 2) + (q->mlx.m_targetRate / 2);
	#if PRINT_LOG
	printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
}
void RdmaHw::HyperIncreaseMlx(Ptr<RdmaQueuePair> q){
	#if PRINT_LOG
	printf("%lu hyper inc: %08x %08x %u %u (%0.3lf %.3lf)->", Simulator::Now().GetTimeStep(), q->sip.Get(), q->dip.Get(), q->sport, q->dport, q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
	// get NIC
	uint32_t nic_idx = GetNicIdxOfQp(q);
	Ptr<QbbNetDevice> dev = m_nic[nic_idx].dev;
	// increate rate
	q->mlx.m_targetRate += m_rhai;
	if (q->mlx.m_targetRate > dev->GetDataRate())
		q->mlx.m_targetRate = dev->GetDataRate();
	q->m_rate = (q->m_rate / 2) + (q->mlx.m_targetRate / 2);
	#if PRINT_LOG
	printf("(%.3lf %.3lf)\n", q->mlx.m_targetRate.GetBitRate() * 1e-9, q->m_rate.GetBitRate() * 1e-9);
	#endif
}

/***********************
 * High Precision CC
 ***********************/
void RdmaHw::HandleAckHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	uint64_t ack_seq = ch.ack.seq;
	// update rate
	if (ack_seq > qp->hp.m_lastUpdateSeq){ // if full RTT feedback is ready, do full update
		UpdateRateHp(qp, p, ch, false);
	}else{ // do fast react
		FastReactHp(qp, p, ch);
	}
}

void RdmaHw::UpdateRateHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react){
	uint64_t next_seq = qp->snd_nxt;
	bool print = !fast_react || true;
	if (qp->hp.m_lastUpdateSeq == 0){ // first RTT
		qp->hp.m_lastUpdateSeq = next_seq;
		// store INT
		IntHeader &ih = ch.ack.ih;
		NS_ASSERT(ih.nhop <= IntHeader::maxHop);
		for (uint32_t i = 0; i < ih.nhop; i++)
			qp->hp.hop[i] = ih.hop[i];
		#if PRINT_LOG
		if (print){
			printf("%lu %s %08x %08x %u %u [%u,%u,%u]", Simulator::Now().GetTimeStep(), fast_react? "fast" : "update", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->hp.m_lastUpdateSeq, ch.ack.seq, next_seq);
			for (uint32_t i = 0; i < ih.nhop; i++)
				printf(" %u %lu %lu", ih.hop[i].GetQlen(), ih.hop[i].GetBytes(), ih.hop[i].GetTime());
			printf("\n");
		}
		#endif
	}else {
		// check packet INT
		IntHeader &ih = ch.ack.ih;
		if (ih.nhop <= IntHeader::maxHop){
			double max_c = 0;
			bool inStable = false;
			#if PRINT_LOG
			if (print)
				printf("%lu %s %08x %08x %u %u [%u,%u,%u]", Simulator::Now().GetTimeStep(), fast_react? "fast" : "update", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->hp.m_lastUpdateSeq, ch.ack.seq, next_seq);
			#endif
			// check each hop
			double U = 0;
			uint64_t dt = 0;
			bool updated[IntHeader::maxHop] = {false}, updated_any = false;
			NS_ASSERT(ih.nhop <= IntHeader::maxHop);
			for (uint32_t i = 0; i < ih.nhop; i++){
				if (m_sampleFeedback){
					if (ih.hop[i].GetQlen() == 0 && fast_react)
						continue;
				}
				updated[i] = updated_any = true;
				#if PRINT_LOG
				if (print)
					printf(" %u(%u) %lu(%lu) %lu(%lu)", ih.hop[i].GetQlen(), qp->hp.hop[i].GetQlen(), ih.hop[i].GetBytes(), qp->hp.hop[i].GetBytes(), ih.hop[i].GetTime(), qp->hp.hop[i].GetTime());
				#endif
				uint64_t tau = ih.hop[i].GetTimeDelta(qp->hp.hop[i]);;
				double duration = tau * 1e-9;
				double txRate = (ih.hop[i].GetBytesDelta(qp->hp.hop[i])) * 8 / duration;
				double u = txRate / ih.hop[i].GetLineRate() + (double)std::min(ih.hop[i].GetQlen(), qp->hp.hop[i].GetQlen()) * qp->m_max_rate.GetBitRate() / ih.hop[i].GetLineRate() /qp->m_win;
				#if PRINT_LOG
				if (print)
					printf(" %.3lf %.3lf", txRate, u);
				#endif
				if (!m_multipleRate){
					// for aggregate (single R)
					if (u > U){
						U = u;
						dt = tau;
					}
				}else {
					// for per hop (per hop R)
					if (tau > qp->m_baseRtt)
						tau = qp->m_baseRtt;
					qp->hp.hopState[i].u = (qp->hp.hopState[i].u * (qp->m_baseRtt - tau) + u * tau) / double(qp->m_baseRtt);
				}
				qp->hp.hop[i] = ih.hop[i];
			}

			DataRate new_rate;
			int32_t new_incStage;
			DataRate new_rate_per_hop[IntHeader::maxHop];
			int32_t new_incStage_per_hop[IntHeader::maxHop];
			if (!m_multipleRate){
				// for aggregate (single R)
				if (updated_any){
					if (dt > qp->m_baseRtt)
						dt = qp->m_baseRtt;
					qp->hp.u = (qp->hp.u * (qp->m_baseRtt - dt) + U * dt) / double(qp->m_baseRtt);
					max_c = qp->hp.u / m_targetUtil;

					if (max_c >= 1 || qp->hp.m_incStage >= m_miThresh){
						new_rate = qp->hp.m_curRate / max_c + m_rai;
						new_incStage = 0;
					}else{
						new_rate = qp->hp.m_curRate + m_rai;
						new_incStage = qp->hp.m_incStage+1;
					}
					if (new_rate < m_minRate)
						new_rate = m_minRate;
					if (new_rate > qp->m_max_rate)
						new_rate = qp->m_max_rate;
					#if PRINT_LOG
					if (print)
						printf(" u=%.6lf U=%.3lf dt=%u max_c=%.3lf", qp->hp.u, U, dt, max_c);
					#endif
					#if PRINT_LOG
					if (print)
						printf(" rate:%.3lf->%.3lf\n", qp->hp.m_curRate.GetBitRate()*1e-9, new_rate.GetBitRate()*1e-9);
					#endif
				}
			}else{
				// for per hop (per hop R)
				new_rate = qp->m_max_rate;
				for (uint32_t i = 0; i < ih.nhop; i++){
					if (updated[i]){
						double c = qp->hp.hopState[i].u / m_targetUtil;
						if (c >= 1 || qp->hp.hopState[i].incStage >= m_miThresh){
							new_rate_per_hop[i] = qp->hp.hopState[i].Rc / c + m_rai;
							new_incStage_per_hop[i] = 0;
						}else{
							new_rate_per_hop[i] = qp->hp.hopState[i].Rc + m_rai;
							new_incStage_per_hop[i] = qp->hp.hopState[i].incStage+1;
						}
						// bound rate
						if (new_rate_per_hop[i] < m_minRate)
							new_rate_per_hop[i] = m_minRate;
						if (new_rate_per_hop[i] > qp->m_max_rate)
							new_rate_per_hop[i] = qp->m_max_rate;
						// find min new_rate
						if (new_rate_per_hop[i] < new_rate)
							new_rate = new_rate_per_hop[i];
						#if PRINT_LOG
						if (print)
							printf(" [%u]u=%.6lf c=%.3lf", i, qp->hp.hopState[i].u, c);
						#endif
						#if PRINT_LOG
						if (print)
							printf(" %.3lf->%.3lf", qp->hp.hopState[i].Rc.GetBitRate()*1e-9, new_rate.GetBitRate()*1e-9);
						#endif
					}else{
						if (qp->hp.hopState[i].Rc < new_rate)
							new_rate = qp->hp.hopState[i].Rc;
					}
				}
				#if PRINT_LOG
				printf("\n");
				#endif
			}
			if (updated_any)
				ChangeRate(qp, new_rate);
			if (!fast_react){
				if (updated_any){
					qp->hp.m_curRate = new_rate;
					qp->hp.m_incStage = new_incStage;
				}
				if (m_multipleRate){
					// for per hop (per hop R)
					for (uint32_t i = 0; i < ih.nhop; i++){
						if (updated[i]){
							qp->hp.hopState[i].Rc = new_rate_per_hop[i];
							qp->hp.hopState[i].incStage = new_incStage_per_hop[i];
						}
					}
				}
			}
		}
		if (!fast_react){
			if (next_seq > qp->hp.m_lastUpdateSeq)
				qp->hp.m_lastUpdateSeq = next_seq; //+ rand() % 2 * m_mtu;
		}
	}
}

void RdmaHw::FastReactHp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	if (m_fast_react)
		UpdateRateHp(qp, p, ch, true);
}

/**********************
 * TIMELY
 *********************/
void RdmaHw::HandleAckTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	uint64_t ack_seq = ch.ack.seq;
	// update rate
	if (ack_seq > qp->tmly.m_lastUpdateSeq){ // if full RTT feedback is ready, do full update
		UpdateRateTimely(qp, p, ch, false);
	}else{ // do fast react
		FastReactTimely(qp, p, ch);
	}
}
void RdmaHw::UpdateRateTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool us){
	uint64_t next_seq = qp->snd_nxt;
	uint64_t rtt = Simulator::Now().GetTimeStep() - ch.ack.ih.ts;
	bool print = !us;
	if (qp->tmly.m_lastUpdateSeq != 0){ // not first RTT
		int64_t new_rtt_diff = (int64_t)rtt - (int64_t)qp->tmly.lastRtt;
		double rtt_diff = (1 - m_tmly_alpha) * qp->tmly.rttDiff + m_tmly_alpha * new_rtt_diff;
		double gradient = rtt_diff / m_tmly_minRtt;
		bool inc = false;
		double c = 0;
		#if PRINT_LOG
		if (print)
			printf("%lu node:%u rtt:%lu rttDiff:%.0lf gradient:%.3lf rate:%.3lf", Simulator::Now().GetTimeStep(), m_node->GetId(), rtt, rtt_diff, gradient, qp->tmly.m_curRate.GetBitRate() * 1e-9);
		#endif
		if (rtt < m_tmly_TLow){
			inc = true;
		}else if (rtt > m_tmly_THigh){
			c = 1 - m_tmly_beta * (1 - (double)m_tmly_THigh / rtt);
			inc = false;
		}else if (gradient <= 0){
			inc = true;
		}else{
			c = 1 - m_tmly_beta * gradient;
			if (c < 0)
				c = 0;
			inc = false;
		}
		if (inc){
			if (qp->tmly.m_incStage < 5){
				qp->m_rate = qp->tmly.m_curRate + m_rai;
			}else{
				qp->m_rate = qp->tmly.m_curRate + m_rhai;
			}
			if (qp->m_rate > qp->m_max_rate)
				qp->m_rate = qp->m_max_rate;
			if (!us){
				qp->tmly.m_curRate = qp->m_rate;
				qp->tmly.m_incStage++;
				qp->tmly.rttDiff = rtt_diff;
			}
		}else{
			qp->m_rate = std::max(m_minRate, qp->tmly.m_curRate * c);
			if (!us){
				qp->tmly.m_curRate = qp->m_rate;
				qp->tmly.m_incStage = 0;
				qp->tmly.rttDiff = rtt_diff;
			}
		}
		#if PRINT_LOG
		if (print){
			printf(" %c %.3lf\n", inc? '^':'v', qp->m_rate.GetBitRate() * 1e-9);
		}
		#endif
	}
	if (!us && next_seq > qp->tmly.m_lastUpdateSeq){
		qp->tmly.m_lastUpdateSeq = next_seq;
		// update
		qp->tmly.lastRtt = rtt;
	}
}
void RdmaHw::FastReactTimely(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
}

/**********************
 * DCTCP
 *********************/
void RdmaHw::HandleAckDctcp(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
	uint64_t ack_seq = ch.ack.seq;
	uint8_t cnp = (ch.ack.flags >> qbbHeader::FLAG_CNP) & 1;
	bool new_batch = false;

	// update alpha
	qp->dctcp.m_ecnCnt += (cnp > 0);
	if (ack_seq > qp->dctcp.m_lastUpdateSeq){ // if full RTT feedback is ready, do alpha update
		#if PRINT_LOG
		printf("%lu %s %08x %08x %u %u [%u,%u,%u] %.3lf->", Simulator::Now().GetTimeStep(), "alpha", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->dctcp.m_lastUpdateSeq, ch.ack.seq, qp->snd_nxt, qp->dctcp.m_alpha);
		#endif
		new_batch = true;
		if (qp->dctcp.m_lastUpdateSeq == 0){ // first RTT
			qp->dctcp.m_lastUpdateSeq = qp->snd_nxt;
			qp->dctcp.m_batchSizeOfAlpha = qp->snd_nxt / m_mtu + 1;
		}else {
			double frac = std::min(1.0, double(qp->dctcp.m_ecnCnt) / qp->dctcp.m_batchSizeOfAlpha);
			qp->dctcp.m_alpha = (1 - m_g) * qp->dctcp.m_alpha + m_g * frac;
			qp->dctcp.m_lastUpdateSeq = qp->snd_nxt;
			qp->dctcp.m_ecnCnt = 0;
			qp->dctcp.m_batchSizeOfAlpha = (qp->snd_nxt - ack_seq) / m_mtu + 1;
			#if PRINT_LOG
			printf("%.3lf F:%.3lf", qp->dctcp.m_alpha, frac);
			#endif
		}
		#if PRINT_LOG
		printf("\n");
		#endif
	}

	// check cwr exit
	if (qp->dctcp.m_caState == 1){
		if (ack_seq > qp->dctcp.m_highSeq)
			qp->dctcp.m_caState = 0;
	}

	// check if need to reduce rate: ECN and not in CWR
	if (cnp && qp->dctcp.m_caState == 0){
		#if PRINT_LOG
		printf("%lu %s %08x %08x %u %u %.3lf->", Simulator::Now().GetTimeStep(), "rate", qp->sip.Get(), qp->dip.Get(), qp->sport, qp->dport, qp->m_rate.GetBitRate()*1e-9);
		#endif
		qp->m_rate = std::max(m_minRate, qp->m_rate * (1 - qp->dctcp.m_alpha / 2));
		#if PRINT_LOG
		printf("%.3lf\n", qp->m_rate.GetBitRate() * 1e-9);
		#endif
		qp->dctcp.m_caState = 1;
		qp->dctcp.m_highSeq = qp->snd_nxt;
	}

	// additive inc
	if (qp->dctcp.m_caState == 0 && new_batch)
		qp->m_rate = std::min(qp->m_max_rate, qp->m_rate + m_dctcp_rai);
}

/*********************
 * HPCC-PINT
 ********************/
void RdmaHw::SetPintSmplThresh(double p){
       pint_smpl_thresh = (uint32_t)(65536 * p);
}
void RdmaHw::HandleAckHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch){
       uint64_t ack_seq = ch.ack.seq;
       if (rand() % 65536 >= pint_smpl_thresh)
               return;
       // update rate
       if (ack_seq > qp->hpccPint.m_lastUpdateSeq){ // if full RTT feedback is ready, do full update
               UpdateRateHpPint(qp, p, ch, false);
       }else{ // do fast react
               UpdateRateHpPint(qp, p, ch, true);
       }
}

void RdmaHw::UpdateRateHpPint(Ptr<RdmaQueuePair> qp, Ptr<Packet> p, CustomHeader &ch, bool fast_react){
       uint64_t next_seq = qp->snd_nxt;
       if (qp->hpccPint.m_lastUpdateSeq == 0){ // first RTT
               qp->hpccPint.m_lastUpdateSeq = next_seq;
       }else {
               // check packet INT
               IntHeader &ih = ch.ack.ih;
               double U = Pint::decode_u(ih.GetPower());

               DataRate new_rate;
               int32_t new_incStage;
               double max_c = U / m_targetUtil;

               if (max_c >= 1 || qp->hpccPint.m_incStage >= m_miThresh){
                       new_rate = qp->hpccPint.m_curRate / max_c + m_rai;
                       new_incStage = 0;
               }else{
                       new_rate = qp->hpccPint.m_curRate + m_rai;
                       new_incStage = qp->hpccPint.m_incStage+1;
               }
               if (new_rate < m_minRate)
                       new_rate = m_minRate;
               if (new_rate > qp->m_max_rate)
                       new_rate = qp->m_max_rate;
               ChangeRate(qp, new_rate);
               if (!fast_react){
                       qp->hpccPint.m_curRate = new_rate;
                       qp->hpccPint.m_incStage = new_incStage;
               }
               if (!fast_react){
                       if (next_seq > qp->hpccPint.m_lastUpdateSeq)
                               qp->hpccPint.m_lastUpdateSeq = next_seq; //+ rand() % 2 * m_mtu;
               }
       }
}

}
