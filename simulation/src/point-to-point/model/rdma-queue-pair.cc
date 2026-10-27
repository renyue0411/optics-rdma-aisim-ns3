#include <ns3/hash.h>
#include <ns3/log.h>
#include <algorithm>
NS_LOG_COMPONENT_DEFINE("RdmaQueuePairIrn");
#include <ns3/uinteger.h>
#include <ns3/seq-ts-header.h>
#include <ns3/udp-header.h>
#include <ns3/ipv4-header.h>
#include <ns3/simulator.h>
#include "ns3/ppp-header.h"
#include "rdma-queue-pair.h"

// MODE1_CONTINUATION_ACK_RECOVERY_V1: bounded next-window DATA probing.
// COMMON_RC_RELIABILITY_V1: RNIC-local RC timeout/retry/error state shared
// across the transport modes.
// MODE2_VERBS_ERROR_CQE_V1: Mode-2 terminal errors map to verbs-style CQEs.

namespace ns3 {
IrnSackManager::IrnSackManager() {}

IrnSackManager::IrnSackManager(int flow_id) { socketId = flow_id; }

std::ostream& operator<<(std::ostream& os, const IrnSackManager& im) {
    auto it = im.m_data.begin();
    for (; it != im.m_data.end(); ++it) {
        uint64_t blockBegin = it->first;             // inclusive
        uint64_t blockEnd = it->first + it->second;  // exclusive
        os << "[" << blockBegin << "-" << blockEnd << "] ";
    }
    return os;
}

// put blocks
void IrnSackManager::sack(uint64_t seq, uint64_t sz) {
    if (!sz) return;
    NS_LOG_LOGIC("Flow " << socketId << " : Inserting Block " << seq << "-" << (seq + sz));
    uint64_t seqEnd = seq + sz;  // exclusive

    auto it = m_data.begin();
    for (; it != m_data.end(); ++it) {
        uint64_t blockBegin = it->first;             // inclusive
        uint64_t blockEnd = it->first + it->second;  // exclusive

        if (blockBegin <= seq && seqEnd <= blockEnd) {
            // seq-seqEnd is included inside block-blockEnd
            return;
        } else if (seq < blockBegin && blockEnd < seqEnd) {
            // block-blockEnd is included inside Endseq-seqEnd
            // first segment : seq - blockBegin
            // second segment : blockEnd - seqEnd
            m_data.insert(it, std::pair<uint64_t, uint64_t>(seq, blockBegin - seq));
            NS_LOG_LOGIC("Flow " << socketId << " : Inserting Seg " << seq << "-" << blockBegin);
            seq = blockEnd;
            sz = seqEnd - blockEnd;
            seqEnd = seq + sz;
        } else if (seq < blockBegin && seqEnd <= blockBegin) {
            // seq-seqEnd is mutually exclusive to block-blockEnd, and smaller than block-blockEnd
            m_data.insert(it, std::pair<uint64_t, uint64_t>(seq, sz));
            NS_LOG_LOGIC("Flow " << socketId << " : Inserting Seg (Mutex)" << seq << "-"
                                 << (seq + sz));
            sz = 0;
            break;
        } else if (blockBegin <= seq && seq <= blockEnd && blockEnd < seqEnd) {
            // front part of seq-seqEnd is overlapped
            // new segment : blockEnd - seqEnd
            seq = blockEnd;
            sz = seqEnd - blockEnd;
        } else if (seq < blockBegin && blockBegin <= seqEnd && seqEnd <= blockEnd) {
            // rear part of seq-seqEnd is overlapped
            // new segment : seq - blockBegin
            m_data.insert(it, std::pair<uint64_t, uint64_t>(seq, blockBegin - seq));
            sz = 0;
            break;
        } else {
            NS_ASSERT(blockEnd <= seq);
        }
    }
    if (sz) {
        NS_LOG_LOGIC("Flow " << socketId << " : Inserting Seg (rem) " << seq << "-" << (seq + sz));
        m_data.insert(it, std::pair<uint64_t, uint64_t>(seq, sz));
    }
    NS_ASSERT(m_data.size() > 0);

    // Sanity check : check duplicate, empty blocks
    // merge neighboring blocks
    auto it_prev = m_data.begin();
    for (it = m_data.begin(); it != m_data.end();) {
        if (it == it_prev) {
            ++it;
            continue;
        }
        NS_ASSERT(it_prev->first + it_prev->second <= it->first);
        NS_ASSERT(it->second > 0);
        if (it_prev->first + it_prev->second == it->first) {
            // merge neighboring block
            NS_LOG_LOGIC("Flow " << socketId << " : Merging Block " << it_prev->first << "-"
                                 << (it_prev->first + it_prev->second) << " and " << it->first
                                 << "-" << (it->first + it->second));
            it_prev->second += it->second;
            it = m_data.erase(it);
        } else {
            it_prev = it;
            ++it;
        }
    }

    NS_LOG_LOGIC("Flow " << socketId << " : Blocks " << *this);
}

// put into return number of blocks removed
size_t IrnSackManager::discardUpTo(uint64_t cumAck) {
    auto it = m_data.begin();
    size_t erase_len = 0;
    for (; it != m_data.end();) {
        if (it->first + it->second <= cumAck) {
            NS_LOG_LOGIC("Flow " << socketId << " : Removing under " << cumAck
                                 << " - Removed Whole " << it->first << "-"
                                 << (it->first + it->second));
            erase_len += it->second;
            it = m_data.erase(it);
        } else if (it->first < cumAck) {  // do we need equal here? Maybe not
            NS_LOG_LOGIC("Flow " << socketId << " : Removing under " << cumAck
                                 << " - Removed Part  " << it->first << "-" << (cumAck)
                                 << " of Entire part " << it->first << "-"
                                 << (it->first + it->second));
            erase_len += cumAck - it->first;
            it->second = it->first + it->second - cumAck;
            it->first = cumAck;
            NS_ASSERT(it->second != 0);
            break;
        } else {
            break;
        }
    }
    return erase_len;
}

bool IrnSackManager::IsEmpty() { return !m_data.size(); }

bool IrnSackManager::blockExists(uint64_t seq, uint64_t size) {
    // query if block exists inside SACK table
    auto it = m_data.begin();
    for (; it != m_data.end(); ++it) {
        if (it->first <= seq && seq + size <= it->first + it->second) {
            return true;
        }
    }
    return false;
}
bool IrnSackManager::peekFrontBlock(uint64_t* pseq, uint64_t* psize) {
    NS_ASSERT(pseq);
    NS_ASSERT(psize);

    if (!m_data.size()) {
        *pseq = 0;
        *psize = 0;
        return false;
    }

    auto it = m_data.begin();
    *pseq = it->first;
    *psize = it->second;
    return true;
}

size_t IrnSackManager::getSackBufferOverhead() {
    size_t overhead = 0;
    auto it = m_data.begin();
    for (; it != m_data.end(); ++it) {
        overhead += it->second;  // Bytes
    }
    return overhead;
}



/**************************
 * RdmaQueuePair
 *************************/
TypeId RdmaQueuePair::GetTypeId (void)
{
static TypeId tid = TypeId ("ns3::RdmaQueuePair")
		.SetParent<Object> ()
		;
	return tid;
}

RdmaQueuePair::RdmaQueuePair(uint16_t pg, Ipv4Address _sip, Ipv4Address _dip, uint16_t _sport, uint16_t _dport){
	startTime = Simulator::Now();
	sip = _sip;
	dip = _dip;
	sport = _sport;
	dport = _dport;
	m_size = 0;
	m_init_size = 0;
	m_postedLimit = 0;
	m_src = -1;
	m_dest = -1;
	m_tag = -1;
	snd_nxt = snd_una = 0;
	m_highestSentSeq = 0;
	m_retransPackets = 0;
	m_retransBytes = 0;
	m_nackCount = 0;
	m_timeoutCount = 0;
	m_rcRetryAttempts = 0;
	m_rcRetryAttemptCount = 0;
	m_rcTimeoutDeferredCount = 0;
	m_rcRetryExhaustedCount = 0;
	m_rcRetryExhausted = false;
	m_qpError = false;
	m_qpErrorStatus = RDMA_WR_CQE_SUCCESS;
	m_deadlineSampleOutstanding = false;
	m_deadlineSampleSeq = 0;
	m_deadlineSampleTxNs = 0;
	m_deadlineSampleWindowStartNs = 0;
	m_deadlineSampleWindowEndNs = 0;
	m_deadlineLastAllowedWindowStartNs = 0;
	m_deadlineLastAllowedWindowEndNs = 0;
	m_deadlineRejectedCrossWindowSamples = 0;
	m_deadlineSrttNs = 0;
	m_deadlineRttVarNs = 0;
	m_deadlineValidSamples = 0;
	m_deadlineCheckCount = 0;
	m_deadlineAllowedCheckCount = 0;
	m_deadlineBlockedCheckCount = 0;
	m_deadlineBlockEventCount = 0;
	m_deadlineLastBlockedWindowEndNs = 0;
	m_deadlineLastPacketSeq = 0;
	m_deadlineLastPacketEndSeq = 0;
	m_deadlineLastPacketWasRetransmission = false;
	m_ackRecoveryActive = false;
	m_ackRecoveryPermit = false;
	m_ackRecoveryProbeInFlight = false;
	m_ackRecoveryPermitType = RNIC_ACK_PROBE_NONE;
	m_ackRecoveryInFlightType = RNIC_ACK_PROBE_NONE;
	m_ackRecoveryNextWindowStartNs = 0;
	m_ackRecoveryLastArmWindowEndNs = 0;
	m_ackRecoveryPermitWindowStartNs = 0;
	m_ackRecoveryLastProbeWindowStartNs = 0;
	m_ackRecoveryProbeStartSeq = 0;
	m_ackRecoveryProbeEndSeq = 0;
	m_ackRecoveryLastAckProgressNs = 0;
	m_ackRecoveryLastAckProgressSeq = 0;
	m_ackRecoveryAttempts = 0;
	m_ackRecoveryArmCount = 0;
	m_ackRecoveryProbeCount = 0;
	m_ackRecoveryTailProbeCount = 0;
	m_ackRecoveryPartialAckCount = 0;
	m_ackRecoverySuccessCount = 0;
	m_ackRecoveryDelayedAckCount = 0;
	m_ackRecoveryNackCount = 0;
	m_ackRecoveryExhaustedCount = 0;
	m_ackRecoveryNoContinuationCount = 0;
	m_pg = pg;
	m_ipid = 0;
	m_win = 0;
	m_baseRtt = 0;
	m_hasBoundRnicPort = false;
	m_boundRnicPort = 0;
	m_boundNicIdx = 0;
	m_boundPhysicalNicId = 0;
	m_boundPlaneId = 0;
	m_max_rate = 0;
	m_var_win = false;
	m_rate = 0;
	m_nextAvail = Time(0);
	mlx.m_alpha = 1;
	mlx.m_alpha_cnp_arrived = false;
	mlx.m_first_cnp = true;
	mlx.m_decrease_cnp_arrived = false;
	mlx.m_rpTimeStage = 0;
	hp.m_lastUpdateSeq = 0;
	for (uint32_t i = 0; i < sizeof(hp.keep) / sizeof(hp.keep[0]); i++)
		hp.keep[i] = 0;
	hp.m_incStage = 0;
	hp.m_lastGap = 0;
	hp.u = 1;
	for (uint32_t i = 0; i < IntHeader::maxHop; i++){
		hp.hopState[i].u = 1;
		hp.hopState[i].incStage = 0;
	}

	tmly.m_lastUpdateSeq = 0;
	tmly.m_incStage = 0;
	tmly.lastRtt = 0;
	tmly.rttDiff = 0;

	dctcp.m_lastUpdateSeq = 0;
	dctcp.m_caState = 0;
	dctcp.m_highSeq = 0;
	dctcp.m_alpha = 1;
	dctcp.m_ecnCnt = 0;
	dctcp.m_batchSizeOfAlpha = 0;

	hpccPint.m_lastUpdateSeq = 0;
	hpccPint.m_incStage = 0;
}

void RdmaQueuePair::SetSize(uint64_t size){
	m_size = size;
	m_postedLimit = size;
}

void RdmaQueuePair::SetSrc(uint32_t src){
	m_src = src;
}

void RdmaQueuePair::SetDest(uint32_t dest){
	m_dest = dest;
}

uint32_t RdmaQueuePair::GetSrc() const{
	return m_src;
}

uint32_t RdmaQueuePair::GetDest() const{
	return m_dest;
}

void RdmaQueuePair::SetTag(uint64_t tag){
	m_tag = tag;
}

uint64_t RdmaQueuePair::GetTag(){
	return m_tag;
}

void RdmaQueuePair::SetInitialSize(uint64_t size){
	m_init_size = size;
}

uint64_t RdmaQueuePair::GetInitialSize(){
	return m_init_size;
}


void RdmaQueuePair::SetPostedLimit(uint64_t limit){
	NS_ASSERT_MSG(limit <= m_size, "posted limit exceeds qp size");
	m_postedLimit = limit;
	if (snd_nxt > m_postedLimit){
		snd_nxt = m_postedLimit;
	}
	if (snd_una > m_postedLimit){
		snd_una = m_postedLimit;
	}
}

void RdmaQueuePair::AddPostedBytes(uint64_t bytes){
	NS_ASSERT_MSG(m_postedLimit <= m_size, "posted limit exceeds qp size");
	uint64_t remaining = m_size - m_postedLimit;
	uint64_t admitted = bytes < remaining ? bytes : remaining;
	m_postedLimit += admitted;
}

uint64_t RdmaQueuePair::GetPostedLimit() const{
	return m_postedLimit;
}

uint64_t RdmaQueuePair::GetUnpostedBytes() const{
	return m_size > m_postedLimit ? m_size - m_postedLimit : 0;
}

uint64_t RdmaQueuePair::GetPostedOutstandingBytes() const{
	return m_postedLimit > snd_una ? m_postedLimit - snd_una : 0;
}

void RdmaQueuePair::SetWin(uint32_t win){
	m_win = win;
	// std::cout << "set win: " << m_win << std::endl;
}

void RdmaQueuePair::SetBaseRtt(uint64_t baseRtt){
	m_baseRtt = baseRtt;
}

void RdmaQueuePair::SetVarWin(bool v){
	m_var_win = v;
}

void RdmaQueuePair::SetAppNotifyCallback(Callback<void> notifyAppFinish){
	m_notifyAppFinish = notifyAppFinish;
}

void RdmaQueuePair::SetAppSentCallback(Callback<void> notifyAppSent){
	m_notifyAppSent = notifyAppSent;
}


// IRN_RECOVERY_V11: independent recovery scan and fresh-data frontier.
void RdmaQueuePair::StartIrnRecovery(){
	m_irnRecovery = true;
	m_irnRecoveryEnd = m_highestSentSeq;
	m_irnRetransmitNext = snd_una;
	m_irnForceFirst = true;
	snd_nxt = snd_una;
}

uint64_t RdmaQueuePair::GetBytesLeft(){
	if (m_irnEnabled){
		m_irnSack.discardUpTo(snd_una);
		// Unconfirmed tail packets are not automatically lost. Without a
		// recovery candidate, resume fresh DATA at the high-water mark.
		snd_nxt = m_highestSentSeq;
		if (m_irnRecovery){
			m_irnRetransmitNext = std::max(m_irnRetransmitNext, snd_una);
			m_irnRetransmitNext = m_irnSack.AdvancePast(m_irnRetransmitNext);
			// The first retransmission is permitted by NACK/timeout. Later
			// candidates require a higher-sequence selectively ACKed packet.
			// Keep the scan position when resuming new DATA: a later SACK
			// may reveal another hole behind the fresh-data frontier.
			if (m_irnRetransmitNext < m_highestSentSeq &&
			    (m_irnForceFirst || m_irnRetransmitNext < m_irnHighestSackEnd)){
				snd_nxt = m_irnRetransmitNext;
			}
		}
	}
	return m_postedLimit >= snd_nxt ? m_postedLimit - snd_nxt : 0;
}

uint32_t RdmaQueuePair::GetHash(void){
	union{
		struct {
			uint32_t sip, dip;
			uint16_t sport, dport;
		};
		char c[12];
	} buf;
	buf.sip = sip.Get();
	buf.dip = dip.Get();
	buf.sport = sport;
	buf.dport = dport;
	return Hash32(buf.c, 12);
}

void RdmaQueuePair::Acknowledge(uint64_t ack){
	if (ack > snd_una){
		snd_una = ack;

		// RecoverQueue() may rewind snd_nxt while ACKs for already
		// transmitted packets are still in flight. A cumulative ACK
		// can then advance snd_una beyond snd_nxt. Keep the send
		// cursor out of already acknowledged sequence space.
		if (snd_nxt < snd_una){
			snd_nxt = snd_una;
		}
	}
}

uint64_t RdmaQueuePair::GetOnTheFly(){
	return snd_nxt - snd_una;
}

bool RdmaQueuePair::IsWinBound(){
	if (m_irnEnabled){
		// Re-select after fresh SACK feedback, even if snd_nxt last pointed
		// at new DATA. Recovery must remain possible at a full BDP cap.
		GetBytesLeft();
		return snd_nxt >= m_highestSentSeq &&
		       m_highestSentSeq - snd_una >= m_irnBdpBytes;
	}
	uint64_t w = GetWin();
	return w != 0 && GetOnTheFly() >= w;
}

uint64_t RdmaQueuePair::GetWin(){
	if (m_win == 0)
		return 0;
	uint64_t w;
	if (m_var_win){
		w = m_win * m_rate.GetBitRate() / m_max_rate.GetBitRate();
		if (w == 0)
			w = 1; // must > 0
	}else{
		w = m_win;
	}
	return w;
}

uint64_t RdmaQueuePair::HpGetCurWin(){
	if (m_win == 0)
		return 0;
	uint64_t w;
	if (m_var_win){
		w = m_win * hp.m_curRate.GetBitRate() / m_max_rate.GetBitRate();
		if (w == 0)
			w = 1; // must > 0
	}else{
		w = m_win;
	}
	return w;
}

bool RdmaQueuePair::IsFinished(){
	return snd_una >= m_size;
}

/*********************
 * RdmaRxQueuePair
 ********************/
TypeId RdmaRxQueuePair::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaRxQueuePair")
		.SetParent<Object> ()
		;
	return tid;
}

RdmaRxQueuePair::RdmaRxQueuePair(){
	sip = dip = sport = dport = 0;
	m_ipid = 0;
	ReceiverNextExpectedSeq = 0;
	m_nackTimer = Time(0);
	m_lastAckGeneratedSeq = 0;
	m_ackDirty = false;
	m_lastAckTos = 0;
	m_ackCnpPending = false;
	m_hasAckMetadata = false;
	m_ackGeneratedCount = 0;
	m_ackFlushCount = 0;
	m_duplicateAckCount = 0;
	m_lastNACK = 0;
	m_hasBoundRnicPort = false;
	m_boundRnicPort = 0;
	m_boundNicIdx = 0;
	m_boundPhysicalNicId = 0;
	m_boundPlaneId = 0;
}

uint32_t RdmaRxQueuePair::GetHash(void){
	union{
		struct {
			uint32_t sip, dip;
			uint16_t sport, dport;
		};
		char c[12];
	} buf;
	buf.sip = sip;
	buf.dip = dip;
	buf.sport = sport;
	buf.dport = dport;
	return Hash32(buf.c, 12);
}

/*********************
 * RdmaQueuePairGroup
 ********************/
TypeId RdmaQueuePairGroup::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::RdmaQueuePairGroup")
		.SetParent<Object> ()
		;
	return tid;
}

RdmaQueuePairGroup::RdmaQueuePairGroup(void){
}

uint32_t RdmaQueuePairGroup::GetN(void){
	return m_qps.size();
}

Ptr<RdmaQueuePair> RdmaQueuePairGroup::Get(uint32_t idx){
	return m_qps[idx];
}

Ptr<RdmaQueuePair> RdmaQueuePairGroup::operator[](uint32_t idx){
	return m_qps[idx];
}

void RdmaQueuePairGroup::AddQp(Ptr<RdmaQueuePair> qp){
	m_qps.push_back(qp);
}

#if 0
void RdmaQueuePairGroup::AddRxQp(Ptr<RdmaRxQueuePair> rxQp){
	m_rxQps.push_back(rxQp);
}
#endif

void RdmaQueuePairGroup::Clear(void){
	m_qps.clear();
}

}
