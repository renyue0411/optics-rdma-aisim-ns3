// Contracts for finite shared DATA storage, LAN PFC retention and WAN PFC disable.
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "dci-wan-profile.h"
#include <cstdlib>
#include <iostream>
using namespace ns3;
namespace {
void Need(bool ok,const char* text){NS_ABORT_MSG_IF(!ok,text);}
void Configure(Ptr<SwitchMmu> m){
  for(uint32_t p=1;p<=3;++p){m->ConfigHdrm(p,4096);m->pfc_a_shift[p]=3;m->ConfigEcn(p,1,2,1);}
  m->ConfigNPort(3);m->ConfigBufferSize(65536);
}
void CheckPool(){
  auto m=CreateObject<SwitchMmu>();Configure(m);
  Need(!m->m_dciWanPool,"legacy model must remain default");
  Need(!m->CheckIngressAdmission(1,3,20000),"legacy per-ingress admission unexpectedly changed");
  m->ConfigDciWanPort(1);m->ConfigDciWanPort(2);m->ConfigNPort(3);
  Need(m->total_hdrm==4096 && m->total_rsrv==4096 && m->DciSharedCapacity()==57344,
       "WAN headroom/reserve leaked into LAN reservation");
  for(int i=0;i<14;++i){
    uint32_t port=i%2+1;
    Need(m->CheckIngressAdmission(port,3,4096),"WAN admission still limited by PFC dynamic threshold");
    m->UpdateIngressAdmission(port,3,4096);m->UpdateEgressAdmission(2,3,4096);
  }
  Need(!m->CheckIngressAdmission(1,3,1),"WAN consumed reserved LAN space");
  Need(!m->CheckShouldPause(1,3) && !m->CheckShouldResume(1,3),"WAN MMU generated pause state");
  for(int i=0;i<2;++i){
    Need(m->CheckIngressAdmission(3,3,4096),"LAN reserve/headroom unavailable");
    m->UpdateIngressAdmission(3,3,4096);m->UpdateEgressAdmission(2,3,4096);
  }
  Need(m->m_dciDataUsed==65536 && m->m_dciDataPeak==65536,"DATA capacity/peak mismatch");
  Need(!m->CheckIngressAdmission(3,3,1),"global DATA capacity not enforced");
  Need(m->CheckShouldPause(3,3),"LAN PFC decision was disabled");
  for(int i=0;i<14;++i){uint32_t p=i%2+1;m->RemoveFromIngressAdmission(p,3,4096);m->RemoveFromEgressAdmission(2,3,4096);}
  for(int i=0;i<2;++i){m->RemoveFromIngressAdmission(3,3,4096);m->RemoveFromEgressAdmission(2,3,4096);}
  Need(m->m_dciDataUsed==0 && m->shared_used_bytes==0 && m->egress_bytes[2][3]==0 &&
       m->ingress_bytes[1][3]==0 && m->ingress_bytes[2][3]==0 && m->ingress_bytes[3][3]==0 &&
       m->hdrm_bytes[3][3]==0,"DATA accounting did not drain to zero");
  m->egress_bytes[1][3]=3000;
  Need(m->ShouldSendCN(1,3) && !m->ShouldSendCN(1,0),"ECN threshold/control exclusion failed");
  m->egress_bytes[1][3]=0;
  Need(!m->ShouldSendCN(1,3),"ECN marked empty DATA queue");
  std::cout<<"POOL_CHECK_OK capacity_bytes=65536 wan_shared_bytes=57344 drained=1 bounded=1 lan_pfc=1\n";
}
struct Wire{
  uint64_t wan=0,lan=0;
  uint32_t ecnMask=0,nvlsRx=0;
  void Wan(Ptr<const Packet> p){
    CustomHeader c(CustomHeader::L2_Header|CustomHeader::L3_Header|CustomHeader::L4_Header);
    p->PeekHeader(c); if(c.l3Prot==0xFE)++wan;
    if(c.l3Prot==0x11 && c.udp.sport>=21000 && c.udp.sport<21008){
      const uint32_t i=c.udp.sport-21000;
      const uint8_t expected=(i<4?0xa0:4) | (i%4==0?0:3);
      Need(c.m_tos==expected,"switch ECN changed Not-ECT, DSCP, or failed to mark ECT");
      Need(c.udp.seq==0 && c.udp.pg==3,"switch ECN corrupted DATA header");
      Need(!(ecnMask & (1U<<i)),"ECN fixture transmitted duplicate DATA");
      ecnMask |= 1U<<i;
    }
  }
  int Receive(Ptr<QbbNetDevice>,Ptr<Packet>,CustomHeader& c){
    Need(c.l3Prot==0x11 && c.udp.sport>=21004 && c.udp.sport<21008,
         "non-NVLS packet was delivered to the local RDMA callback");
    ++nvlsRx; return 0;
  }
  void Lan(Ptr<const Packet> p){CustomHeader c(CustomHeader::L2_Header|CustomHeader::L3_Header|CustomHeader::L4_Header);p->PeekHeader(c);if(c.l3Prot==0xFE)++lan;}
};
}
int main(){
  CheckPool();
  setenv("SIMAI_WAN_PROFILE","1",1);
  setenv("SIMAI_DCI_BUFFER_MIB","160",1);
  setenv("SIMAI_WAN_ECN_KMIN_KB","1000",1);
  setenv("SIMAI_WAN_ECN_KMAX_KB","20000",1);
  setenv("SIMAI_WAN_ECN_PMAX","0.15",1);
  auto a=CreateObject<SwitchNode>(),b=CreateObject<SwitchNode>();
  NodeContainer nodes;nodes.Add(a);nodes.Add(b);nodes.Create(2);
  InternetStackHelper internet;internet.Install(nodes);
  QbbHelper link;link.SetDeviceAttribute("DataRate",StringValue("100Gbps"));
  link.SetChannelAttribute("Delay",TimeValue(MicroSeconds(250)));
  auto wan=link.Install(a,b);
  link.SetChannelAttribute("Delay",TimeValue(MicroSeconds(1)));
  auto lanA=link.Install(a,nodes.Get(2)),lanB=link.Install(b,nodes.Get(3));
  Ipv4AddressHelper ips;ips.SetBase("10.0.0.0","255.255.255.0");ips.Assign(wan);
  ips.SetBase("10.0.1.0","255.255.255.0");ips.Assign(lanA);
  ips.SetBase("10.0.2.0","255.255.255.0");ips.Assign(lanB);
  for(auto sw:{a,b}){
    for(uint32_t p=1;p<sw->GetNDevices();++p){sw->m_mmu->ConfigHdrm(p,4096);sw->m_mmu->pfc_a_shift[p]=3;sw->m_mmu->ConfigEcn(p,1,2,1);}
    sw->m_mmu->ConfigNPort(sw->GetNDevices()-1);sw->m_mmu->ConfigBufferSize(4*1048576);
    DciWanProfile::Apply(sw,{1});
  }
  auto source=DynamicCast<QbbNetDevice>(wan.Get(0)),sink=DynamicCast<QbbNetDevice>(wan.Get(1));
  auto local=DynamicCast<QbbNetDevice>(lanA.Get(0));
  Need(!source->IsPfcEnabled() && !sink->IsPfcEnabled() && local->IsPfcEnabled(),"PFC scope wrong");
  Need(a->m_mmu->headroom[1]==0 && a->m_mmu->headroom[2]==4096 &&
       a->m_mmu->buffer_size==160*1048576 && a->m_mmu->kmin[1]==1000000 &&
       a->m_mmu->kmax[1]==20000000,"WAN configuration not applied");
  Wire wire;
  sink->m_rdmaReceiveCb=MakeCallback(&Wire::Receive,&wire);
  source->TraceConnectWithoutContext("PhyTxBegin",MakeCallback(&Wire::Wan,&wire));
  local->TraceConnectWithoutContext("PhyTxBegin",MakeCallback(&Wire::Lan,&wire));
  Simulator::Schedule(MicroSeconds(1),[=](){source->SendPfc(3,0);local->SendPfc(3,0);});
  Simulator::Schedule(MicroSeconds(2),[=](){
    // Force one foreign PAUSE onto the WAN link; disabled receive must ignore it.
    auto p=Create<Packet>();PauseHeader pause(1000,0,3);p->AddHeader(pause);
    Ipv4Header ip;ip.SetProtocol(0xFE);ip.SetSource(Ipv4Address("10.0.0.1"));
    ip.SetDestination(Ipv4Address("255.255.255.255"));ip.SetTtl(1);ip.SetPayloadSize(p->GetSize());p->AddHeader(ip);
    PppHeader shim;shim.SetProtocol(0x0021);p->AddHeader(shim);
    CustomHeader c(CustomHeader::L2_Header|CustomHeader::L3_Header|CustomHeader::L4_Header);p->PeekHeader(c);
    source->SwitchSend(0,p,c);
  });
  Simulator::Schedule(MicroSeconds(400),[=](){
    // A warm-up frame keeps the link busy while all test frames are enqueued.
    // A trailing frame leaves a positive DATA backlog for every tested dequeue.
    Ipv4Address destination("11.0.99.1");
    a->AddTableEntry(destination,source->GetIfIndex());
    a->m_mmu->ConfigEcn(source->GetIfIndex(),0,0,1);
    for(int i=-1;i<=8;++i){
      const bool tested=i>=0 && i<8;
      CustomHeader c(CustomHeader::L2_Header|CustomHeader::L3_Header|CustomHeader::L4_Header);
      c.pppProto=0x0021;c.l3Prot=0x11;
      c.sip=Ipv4Address("11.0.2.1").Get();c.dip=destination.Get();
      c.m_tos=tested?((i<4?0xa0:4)|(i%4)):2;
      c.udp.sport=tested?21000+i:22000;c.udp.dport=100;c.udp.pg=3;c.udp.seq=0;
      c.udp.payload_size=CustomHeader::GetUdpHeaderSize()+1000;
      c.m_payloadSize=c.udp.payload_size;
      auto p=Create<Packet>(1000);p->AddHeader(c);
      p->AddPacketTag(FlowIdTag(local->GetIfIndex()));
      a->SwitchReceiveFromDevice(local,p,c);
    }
  });
  Simulator::Stop(MilliSeconds(1));Simulator::Run();
  Need(wire.ecnMask==255 && wire.nvlsRx==4,
       "ECN wire cases missing or NVLS forwarding depends on ECN bits");
  Need(a->m_mmu->m_dciDataUsed==0 && b->m_mmu->m_dciDataUsed==0,
       "ECN fixture left DATA admission accounting outstanding");
  std::cout<<"ECN_WIRE_CHECK_OK codepoints=0,1,2,3 dscp_preserved=1"
             " data_frames=8 nvls_received=4 actual_qbb_tx_verified=1\n";
  Need(source->m_dciPfcTx==0 && source->m_dciPfcTxSuppressed==1 && wire.wan==1,
       "disabled WAN transmitted a generated PFC");
  Need(sink->m_dciPfcRx==1 && sink->m_dciPfcIgnored==1 && !sink->IsPfcPaused(3),
       "disabled WAN obeyed a foreign PAUSE");
  Need(local->m_dciPfcTx==1 && wire.lan==1,"LAN PFC no longer transmits");
  DciWanProfile::Dump();
  std::cout<<"WAN_PROFILE_CHECK_OK pool_bound=1 reserved_protection=1 accounting_drain=1"
             " wan_pfc_tx_disabled=1 wan_pfc_rx_ignored=1 lan_pfc_preserved=1 actual_qbb_tx_verified=1\n";
  Simulator::Destroy();
}
