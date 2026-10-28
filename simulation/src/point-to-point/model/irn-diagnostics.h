// SPDX-License-Identifier: GPL-2.0-only
// Observational, single-process/single-thread diagnostics. No protocol changes.
#ifndef SIMAI_IRN_DIAGNOSTICS_V1_H
#define SIMAI_IRN_DIAGNOSTICS_V1_H
#include "rdma-queue-pair.h"
#include "ns3/abort.h"
#include "ns3/simulator.h"
#include <cstdlib>
#include <cerrno>
#include <iostream>
#include <map>
#include <sstream>
#include <tuple>
#include <vector>
namespace ns3 { namespace IrnDiagnosticsV1 {
using Flow = std::tuple<uint32_t,uint32_t,uint16_t,uint16_t,uint16_t>;
inline uint64_t Number(const char *key, uint64_t fallback) {
  const char *p = std::getenv(key); if (!p) return fallback;
  char *end = nullptr; errno = 0;
  auto v = std::strtoull(p, &end, 10);
  NS_ABORT_MSG_IF(!*p || *p < '0' || *p > '9' || errno || *end, "Invalid " << key);
  return v;
}
struct Copy {
  uint64_t seq=0, bytes=0, attempt=0, generated=0, sourceTx=0;
  bool tx=false, reached=false, accepted=false, dropped=false, windowDrop=false;
  bool rxHadDataAtGen=false, originalAcceptedAtGen=false, knownSack=false, belowUna=false;
};
struct Segment { uint64_t originalUid=0, copies=0; bool hasOriginal=false; };
struct State {
  bool enabled=false, selected=false, reported=false;
  uint64_t src=0,dst=3,sport=0,maxEvents=20000,events=0,suppressed=0;
  Flow flow;
  std::map<uint64_t,Copy> copies;
  std::map<uint64_t,Segment> segments;
  IrnSackManager received; // experimental oracle, never read by transport
  uint64_t gapEntries=0, timeoutEntries=0;
  State() {
    auto flag=Number("SIMAI_IRN_DIAG",0);
    NS_ABORT_MSG_IF(flag>1,"SIMAI_IRN_DIAG must be 0 or 1"); enabled=flag;
    if (!enabled) return;
    src=Number("SIMAI_IRN_DIAG_SRC",0); dst=Number("SIMAI_IRN_DIAG_DST",3);
    sport=Number("SIMAI_IRN_DIAG_SPORT",0); maxEvents=Number("SIMAI_IRN_DIAG_MAX_EVENTS",20000);
    NS_ABORT_MSG_IF(src>65535 || dst>65535 || sport>65535,"Diagnostic filter out of range");
  }
};
inline State &Get() { static State s; return s; }
inline uint64_t Now() { return Simulator::Now().GetNanoSeconds(); }
inline Flow Key(Ptr<RdmaQueuePair> q) { return Flow(q->sip.Get(),q->dip.Get(),q->sport,q->dport,q->m_pg); }
inline bool Match(Ptr<RdmaQueuePair> q) { auto &s=Get(); return s.enabled && s.selected && Key(q)==s.flow; }
inline void Emit(const char *event,const std::string &fields) {
  auto &s=Get(); if (s.events>=s.maxEvents) {++s.suppressed; return;}
  ++s.events; std::cout << "IRN_DIAG_" << event << " time_ns=" << Now() << " " << fields << std::endl;
}
inline void Generate(Ptr<RdmaQueuePair> q,Ptr<const Packet> p,uint64_t seq,uint64_t bytes,bool retx) {
  auto &s=Get(); if (!s.enabled || !q->m_irnEnabled) return;
  if (!s.selected) {
    if (q->m_src!=s.src || q->m_dest!=s.dst || (s.sport && q->sport!=s.sport)) return;
    s.flow=Key(q); s.selected=true;
    std::cout << "IRN_DIAG_CONFIG src=" << q->m_src << " dst=" << q->m_dest
              << " sport=" << q->sport << " dport=" << q->dport << " pg=" << q->m_pg
              << " max_events=" << s.maxEvents << " scope=one_qp observation_only=1" << std::endl;
  }
  if (!Match(q)) return;
  auto &segment=s.segments[seq];
  Copy c; c.seq=seq;c.bytes=bytes;c.generated=Now();c.attempt=segment.copies++;
  NS_ABORT_MSG_IF(retx && !segment.hasOriginal,"Diagnostics missed original DATA generation");
  NS_ABORT_MSG_IF(!retx && segment.hasOriginal,"Repeated original sequence allocation");
  if (!retx) { segment.originalUid=p->GetUid();segment.hasOriginal=true; }
  c.rxHadDataAtGen=s.received.AdvancePast(seq)>=seq+bytes;
  c.originalAcceptedAtGen=retx && s.copies.at(segment.originalUid).accepted;
  c.knownSack=q->m_irnSack.AdvancePast(seq)>=seq+bytes;
  c.belowUna=seq<q->snd_una;
  NS_ABORT_MSG_IF(s.copies.count(p->GetUid()),"Repeated generated packet UID");
  s.copies.emplace(p->GetUid(),c);
  std::ostringstream out;
  out << "uid=" << p->GetUid() << " seq=" << seq << " bytes=" << bytes << " attempt=" << c.attempt
      << " retx=" << retx << " una=" << q->snd_una << " high=" << q->m_highestSentSeq
      << " scan=" << q->m_irnRetransmitNext << " sack_high=" << q->m_irnHighestSackEnd
      << " recovery=" << q->m_irnRecovery << " known_sack=" << c.knownSack
      << " below_una=" << c.belowUna << " rx_has_data=" << c.rxHadDataAtGen;
  Emit("GEN",out.str());
}
inline void LinkTx(Ptr<const Packet> p,uint32_t node,uint32_t port,bool success) {
  auto &s=Get(); if (!s.enabled || !s.selected) return;
  auto it=s.copies.find(p->GetUid());if(it==s.copies.end())return;
  auto &c=it->second;
  if(node==s.src && success) { c.tx=true; c.sourceTx=Now(); }
  if(!success)c.dropped=true;
  std::ostringstream out;out<<"uid="<<p->GetUid()<<" seq="<<c.seq<<" attempt="<<c.attempt
      <<" node="<<node<<" port="<<port<<" success="<<success;
  Emit("LINK_TX",out.str());
}
inline void Drop(Ptr<const Packet> p,uint32_t node,const char *reason) {
  auto &s=Get();if(!s.enabled)return;
  auto it=s.copies.find(p->GetUid());if(it==s.copies.end())return;
  it->second.dropped=true;
  std::ostringstream out;out<<"uid="<<p->GetUid()<<" seq="<<it->second.seq
      <<" attempt="<<it->second.attempt<<" node="<<node<<" reason="<<reason;
  Emit("DROP",out.str());
}
inline void Receive(Ptr<const Packet> p,uint64_t before,uint64_t after,uint64_t useful,bool windowDrop) {
  auto &s=Get();if(!s.enabled)return;
  auto it=s.copies.find(p->GetUid());if(it==s.copies.end())return;
  auto &c=it->second;c.reached=true;c.windowDrop=windowDrop;c.accepted=!windowDrop;
  if(!windowDrop)s.received.sack(c.seq,c.bytes);
  std::ostringstream out;out<<"uid="<<p->GetUid()<<" seq="<<c.seq<<" attempt="<<c.attempt
    <<" expected_before="<<before<<" expected_after="<<after<<" useful="<<useful<<" window_drop="<<windowDrop;
  Emit("RX",out.str());
}
inline void Ack(Ptr<RdmaQueuePair> q,const CustomHeader &h) {
  if(!Match(q))return;
  std::ostringstream out;out<<"cum="<<h.ack.seq<<" sack_seq="<<h.ack.irnSackSeq
    <<" sack_bytes="<<h.ack.irnSackBytes<<" una_before="<<q->snd_una
    <<" recovery_before="<<q->m_irnRecovery<<" recovery_end="<<q->m_irnRecoveryEnd;
  Emit("ACK",out.str());
}
inline void Recovery(Ptr<RdmaQueuePair> q,const char *reason) {
  if(!Match(q))return;
  auto &s=Get();if(std::string(reason)=="gap")++s.gapEntries;else ++s.timeoutEntries;
  std::ostringstream out;out<<"reason="<<reason<<" una="<<q->snd_una
    <<" high="<<q->m_highestSentSeq<<" sack_high="<<q->m_irnHighestSackEnd;
  Emit("RECOVERY",out.str());
}
inline void Counters(Ptr<RdmaQueuePair> q) {
  if(!Match(q))return;
  std::cout<<"IRN_DIAG_COUNTER retrans_packets="<<q->m_retransPackets
    <<" retrans_bytes="<<q->m_retransBytes<<" nacks="<<q->m_nackCount
    <<" timeouts="<<q->m_timeoutCount<<" una="<<q->snd_una
    <<" high="<<q->m_highestSentSeq<<std::endl;
}
inline void Report(uint32_t node) {
  auto &s=Get();if(!s.enabled || node!=s.src || s.reported)return;s.reported=true;
  uint64_t retx=0,bytes=0,tx=0,known=0,below=0,alreadyRx=0,origBefore=0,origLater=0,
    origDrop=0,origWindow=0,unknown=0,originals=0,originalTx=0;
  for(const auto &entry:s.copies) {
    const auto &c=entry.second;
    if(!c.attempt){++originals;originalTx+=c.tx;continue;}
    ++retx;bytes+=c.bytes;tx+=c.tx;known+=c.knownSack;below+=c.belowUna;alreadyRx+=c.rxHadDataAtGen;
    const auto &o=s.copies.at(s.segments.at(c.seq).originalUid);
    if(c.originalAcceptedAtGen)++origBefore;
    else if(o.accepted)++origLater;
    else if(o.windowDrop)++origWindow;
    else if(o.dropped)++origDrop;
    else ++unknown;
  }
  std::cout<<"IRN_DIAG_SUMMARY selected="<<s.selected<<" src="<<s.src<<" dst="<<s.dst
    <<" sport="<<(s.selected?std::get<2>(s.flow):0)<<" original_generated="<<originals
    <<" original_source_tx="<<originalTx<<" retx_generated="<<retx<<" retx_bytes="<<bytes
    <<" retx_source_tx="<<tx<<" retx_known_sack="<<known<<" retx_below_una="<<below
    <<" retx_data_already_at_rx="<<alreadyRx<<" retx_original_accepted_before_gen="<<origBefore
    <<" retx_original_accepted_after_gen="<<origLater<<" retx_original_observed_drop="<<origDrop
    <<" retx_original_rx_window_drop="<<origWindow<<" retx_original_unresolved="<<unknown
    <<" recovery_gap="<<s.gapEntries<<" recovery_timeout="<<s.timeoutEntries
    <<" events="<<s.events<<" suppressed="<<s.suppressed<<" scope=one_qp"<<std::endl;
}
} }
#endif
