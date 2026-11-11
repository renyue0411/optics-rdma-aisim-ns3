// Contracts for opt-in timeout replay; actual packets are tested in AllReduce runs.
#include "ns3/core-module.h"
#include "ns3/rdma-hw.h"
#include <iostream>
using namespace ns3;

Ptr<RdmaQueuePair> MakeQp(bool replay) {
  auto q = CreateObject<RdmaQueuePair>(3, Ipv4Address("10.0.0.1"),
                                      Ipv4Address("10.0.0.2"), 10000, 100);
  q->SetSize(1ULL << 40);
  q->SetPostedLimit(10000);
  q->m_irnEnabled = true;
  q->m_irnTimeoutReplay = replay;
  q->m_irnBdpBytes = 7000;
  q->snd_una = 1000;
  q->m_highestSentSeq = 8000;
  q->m_irnSack.sack(3000, 2000);
  return q;
}

int main() {
  for (bool replay : {false, true}) {
    auto q = MakeQp(replay);
    q->StartIrnRecovery();
    q->m_irnTimeoutSweep = q->m_irnTimeoutReplay; // timeout entry contract
    NS_ABORT_MSG_IF(q->m_irnRecoveryEnd != 8000 || q->GetBytesLeft() != 9000 ||
                    q->snd_nxt != 1000 || q->IsWinBound(), "first retry at full BDP cap");
    q->m_irnRetransmitNext = 2000;
    q->m_irnForceFirst = false; // model first retransmission commit
    q->GetBytesLeft();
    NS_ABORT_MSG_IF(q->snd_nxt != (replay ? 2000U : 8000U), "legacy/replay distinction");
    NS_ABORT_MSG_IF(q->IsWinBound() == replay, "replay must work at a full BDP cap");
    if (replay) {
      q->m_irnRetransmitNext = 3000;
      q->GetBytesLeft();
      NS_ABORT_MSG_IF(q->snd_nxt != 5000 || q->m_irnSack.RangeCount() != 1,
                      "known SACKs must be skipped and retained");
      q->GetBytesLeft();
      NS_ABORT_MSG_IF(q->snd_nxt != 5000 || q->m_irnSack.RangeCount() != 1,
                      "selection is nondestructive");
      q->m_irnRetransmitNext = 8000;
      q->GetBytesLeft();
      NS_ABORT_MSG_IF(q->snd_nxt != 8000 || !q->IsWinBound(),
                      "new DATA still obeys BDP cap");
      q->m_highestSentSeq = 9000; // fresh DATA sent after the captured frontier
      q->GetBytesLeft();
      NS_ABORT_MSG_IF(q->snd_nxt != 9000 || q->m_irnRecoveryEnd != 8000,
                      "timeout replay cannot extend into later-sent data");
      q->SetPostedLimit(9000);
      NS_ABORT_MSG_IF(q->GetBytesLeft() != 0,
                      "QP lifetime capacity is not permission to send unposted bytes");
      q->StartIrnRecovery(); // ordinary SACK entry resets the timeout-only policy
      q->m_irnForceFirst = false;
      q->m_irnRetransmitNext = 2000;
      q->GetBytesLeft();
      NS_ABORT_MSG_IF(q->m_irnTimeoutSweep || q->snd_nxt != 9000,
                      "ordinary recovery cannot inherit timeout sweep permission");
    }
  }
  auto hw = CreateObject<RdmaHw>();
  hw->ConfigureIrn(8000, 1000000, 2000000); // validates runtime option too
  std::cout << "IRN_TIMEOUT_REPLAY_CHECK success=1 legacy=1 sack_skip=1"
               " bounded_frontier=1 posted_limit=1 bdp_gate=1 normal_recovery=1\n";
  Simulator::Destroy();
}
