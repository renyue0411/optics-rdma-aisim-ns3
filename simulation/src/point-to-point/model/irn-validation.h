// SPDX-License-Identifier: GPL-2.0-only
// IRN_VALIDATION_V1: opt-in, single-process fault injection and observation.
// This file is a test harness, not an IRN transport algorithm.
#ifndef SIMAI_IRN_VALIDATION_V1_H
#define SIMAI_IRN_VALIDATION_V1_H

#include "rdma-hw.h"
#include "ns3/abort.h"
#include "ns3/simulator.h"
#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <string>

namespace ns3 {
namespace IrnValidationV1 {

struct State {
  std::string mode;
  uint64_t delayNs = 5000;
  bool injected = false;
  bool released = false;
  bool reported = false;
  bool holeFilled = false;
  bool ackFilled = false;
  uint64_t targetSeq = 0;
  uint64_t targetEnd = 0;
  uint64_t oooPackets = 0;
  uint64_t sackForHole = 0;
  uint64_t targetRetrans = 0;
  uint64_t otherRetrans = 0;
  uint64_t knownSackRetrans = 0;

  State() {
    const char *value = std::getenv("SIMAI_IRN_TEST_FAULT");
    if (value == nullptr || std::string(value).empty() ||
        std::string(value) == "none") return;
    mode = value;
    NS_ABORT_MSG_IF(mode != "delay" && mode != "drop",
                    "SIMAI_IRN_TEST_FAULT must be none, delay, or drop");
    if (const char *delay = std::getenv("SIMAI_IRN_TEST_DELAY_NS")) {
      char *end = nullptr;
      errno = 0;
      const unsigned long long parsed = std::strtoull(delay, &end, 10);
      NS_ABORT_MSG_IF(delay[0] < '0' || delay[0] > '9' || errno != 0 ||
                          end == delay || *end != '\0' || parsed == 0 ||
                          parsed > 1000000000ULL,
                      "SIMAI_IRN_TEST_DELAY_NS must be 1..1000000000");
      delayNs = parsed;
    }
  }
};

inline State &Get() {
  static State state;
  return state;
}

inline bool MatchData(const CustomHeader &ch) {
  return ((ch.sip >> 8) & 0xffff) == 0 &&
         ((ch.dip >> 8) & 0xffff) == 3 &&
         ch.udp.sport == 10000 && ch.udp.dport == 100 && ch.udp.pg == 3;
}

inline bool MatchQp(Ptr<RdmaQueuePair> qp) {
  return qp->m_src == 0 && qp->m_dest == 3 &&
         qp->sport == 10000 && qp->dport == 100 && qp->m_pg == 3;
}

inline void Release(Ptr<RdmaHw> hw, Ptr<QbbNetDevice> dev,
                    Ptr<Packet> packet, CustomHeader header) {
  State &s = Get();
  s.released = true;
  std::cout << "IRN_TEST_RELEASE time_ns=" << Simulator::Now().GetNanoSeconds()
            << " seq=" << header.udp.seq << std::endl;
  // injected was set before scheduling, so this re-entry cannot inject twice.
  hw->ReceiveUdp(dev, packet, header);
}

inline bool Inject(RdmaHw *hw, Ptr<QbbNetDevice> dev,
                   Ptr<Packet> packet, CustomHeader &header) {
  State &s = Get();
  if (s.mode.empty()) return false;
  NS_ABORT_MSG_IF(!hw->m_irnEnabled,
                  "IRN validation requires SIMAI_IRN_ENABLE=1");
  if (s.injected || !MatchData(header) || header.udp.seq != hw->m_mtu)
    return false;
  const uint32_t headerBytes = header.GetSerializedSize();
  NS_ABORT_MSG_IF(packet->GetSize() <= headerBytes,
                  "IRN validation matched a DATA packet without payload");
  const uint32_t bytes = packet->GetSize() - headerBytes;
  s.injected = true;
  s.targetSeq = header.udp.seq;
  s.targetEnd = s.targetSeq + bytes;
  std::cout << "IRN_TEST_INJECT mode=" << s.mode
            << " time_ns=" << Simulator::Now().GetNanoSeconds()
            << " src=0 dst=3 sport=10000 dport=100 pg=3"
            << " seq=" << s.targetSeq << " bytes=" << bytes
            << " delay_ns=" << (s.mode == "delay" ? s.delayNs : 0)
            << std::endl;
  if (s.mode == "delay") {
    Simulator::Schedule(NanoSeconds(s.delayNs), &Release, Ptr<RdmaHw>(hw),
                        dev, packet->Copy(), header);
  }
  // Injection is at the destination RNIC receive entry, after the link.
  // Neither a switch queue nor the configured WAN delay is modified.
  return true;
}

inline void ObserveRx(const CustomHeader &header, Ptr<RdmaRxQueuePair> qp,
                      uint64_t expectedBefore, uint64_t newUsefulBytes) {
  State &s = Get();
  if (!s.injected || !MatchData(header)) return;
  if (header.udp.seq > expectedBefore && newUsefulBytes != 0) {
    if (++s.oooPackets == 1) {
      std::cout << "IRN_TEST_OOO time_ns=" << Simulator::Now().GetNanoSeconds()
                << " expected=" << expectedBefore
                << " received_seq=" << header.udp.seq << std::endl;
    }
  }
  if (!s.holeFilled && expectedBefore <= s.targetSeq &&
      qp->ReceiverNextExpectedSeq >= s.targetEnd) {
    s.holeFilled = true;
    std::cout << "IRN_TEST_HOLE_FILLED time_ns="
              << Simulator::Now().GetNanoSeconds()
              << " expected_before=" << expectedBefore
              << " expected_after=" << qp->ReceiverNextExpectedSeq
              << std::endl;
  }
}

inline void ObserveAck(Ptr<RdmaQueuePair> qp, const CustomHeader &header) {
  State &s = Get();
  if (!s.injected || !MatchQp(qp)) return;
  if ((header.ack.flags & 2) && header.ack.irnSackBytes != 0 &&
      header.ack.seq <= s.targetSeq && header.ack.irnSackSeq >= s.targetEnd) {
    if (++s.sackForHole == 1) {
      std::cout << "IRN_TEST_SACK time_ns=" << Simulator::Now().GetNanoSeconds()
                << " cum=" << header.ack.seq
                << " sack_seq=" << header.ack.irnSackSeq
                << " sack_bytes=" << header.ack.irnSackBytes << std::endl;
    }
  }
  if (!s.ackFilled && qp->snd_una >= s.targetEnd) {
    s.ackFilled = true;
    std::cout << "IRN_TEST_ACK_FILLED time_ns="
              << Simulator::Now().GetNanoSeconds()
              << " snd_una=" << qp->snd_una << std::endl;
  }
}

inline void ObserveTx(Ptr<RdmaQueuePair> qp, uint64_t seq, uint64_t bytes) {
  State &s = Get();
  if (!s.injected || !MatchQp(qp) || seq >= qp->m_highestSentSeq) return;
  // Non-destructive query: verify the sender never retransmits an interval
  // already fully SACKed in its own scoreboard at this instant.
  if (qp->m_irnSack.AdvancePast(seq) >= seq + bytes) ++s.knownSackRetrans;
  if (seq == s.targetSeq) {
    if (++s.targetRetrans == 1) {
      std::cout << "IRN_TEST_RETX time_ns=" << Simulator::Now().GetNanoSeconds()
                << " seq=" << seq << " bytes=" << bytes << std::endl;
    }
  } else {
    ++s.otherRetrans;
  }
}

inline void Report(RdmaHw *hw) {
  State &s = Get();
  if (s.mode.empty() || hw->m_node->GetId() != 0 || s.reported) return;
  s.reported = true;
  std::cout << "IRN_TEST_SUMMARY mode=" << s.mode
            << " injected=" << s.injected << " released=" << s.released
            << " ooo_packets=" << s.oooPackets
            << " hole_filled=" << s.holeFilled
            << " sack_for_hole=" << s.sackForHole
            << " ack_filled=" << s.ackFilled
            << " target_retrans_packets=" << s.targetRetrans
            << " other_retrans_packets=" << s.otherRetrans
            << " known_sack_retrans_packets=" << s.knownSackRetrans
            << std::endl;
}

} // namespace IrnValidationV1
} // namespace ns3
#endif
