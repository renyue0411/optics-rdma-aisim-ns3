#include "ocs-node.h"

#include <iostream>

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/assert.h"
#include "ns3/channel.h"
#include "ns3/net-device.h"
#include "ns3/flow-id-tag.h"
#include "ns3/qbb-channel.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OcsNode");
NS_OBJECT_ENSURE_REGISTERED(OcsNode);

TypeId
OcsNode::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::OcsNode")
    .SetParent<Node>()
    .AddConstructor<OcsNode>();
  return tid;
}

OcsNode::OcsNode()
  : m_scheduleEnabled(false),
    m_numSlices(0),
    m_epochStart(Seconds(0)),
    m_sliceDuration(Seconds(0)),
    m_switchingTime(Seconds(0)),
    m_forwardedPackets(0),
    m_forwardedBytes(0),
    m_dropNoCircuit(0),
    m_dropSwitching(0),
    m_dropBadPort(0),
    m_dropBadDevice(0)
{
  // Existing backend: 0 = host, 1 = EPS switch, 2 = NVSwitch.
  // Reserve 3 for the optical circuit switch.
  m_node_type = 3;
}

uint64_t
OcsNode::MakeScheduleKey(uint32_t inPort, uint32_t slice)
{
  return (static_cast<uint64_t>(slice) << 32) |
         static_cast<uint64_t>(inPort);
}

void
OcsNode::ClearCircuits()
{
  m_circuitMap.clear();

  std::cout << "[OCS CLEAR]"
            << " t=" << Simulator::Now().GetTimeStep()
            << " node=" << GetId()
            << std::endl;
}

void
OcsNode::SetCircuit(uint32_t inPort, uint32_t outPort)
{
  NS_ASSERT_MSG(inPort != outPort, "OCS self-circuit is not allowed");
  NS_ASSERT_MSG(inPort < GetNDevices(), "OCS input port out of range");
  NS_ASSERT_MSG(outPort < GetNDevices(), "OCS output port out of range");

  std::unordered_map<uint32_t, uint32_t>::iterator old = m_circuitMap.find(inPort);
  if (old != m_circuitMap.end())
    {
      std::cout << "[OCS WARN OVERWRITE]"
                << " t=" << Simulator::Now().GetTimeStep()
                << " node=" << GetId()
                << " inPort=" << inPort
                << " oldOut=" << old->second
                << " newOut=" << outPort
                << std::endl;
    }

  m_circuitMap[inPort] = outPort;

  std::cout << "[OCS CIRCUIT]"
            << " t=" << Simulator::Now().GetTimeStep()
            << " node=" << GetId()
            << " " << inPort << "->" << outPort
            << std::endl;
}

void
OcsNode::SetBidirectionalCircuit(uint32_t portA, uint32_t portB)
{
  SetCircuit(portA, portB);
  SetCircuit(portB, portA);
}

void
OcsNode::SetInitialMapping(const std::vector<std::pair<uint32_t, uint32_t> >& mapping)
{
  // Backward-compatible static mode.
  m_scheduleEnabled = false;
  m_schedule.clear();
  m_numSlices = 0;
  m_epochStart = Seconds(0);
  m_sliceDuration = Seconds(0);
  m_switchingTime = Seconds(0);

  ClearCircuits();

  for (uint32_t i = 0; i < mapping.size(); ++i)
    {
      SetCircuit(mapping[i].first, mapping[i].second);
    }

  std::cout << "[OCS STATIC MAP INSTALLED]"
            << " t=" << Simulator::Now().GetTimeStep()
            << " node=" << GetId()
            << " entries=" << mapping.size()
            << std::endl;
}

void
OcsNode::ConfigureSchedule(Time epochStart,
                           Time sliceDuration,
                           uint32_t numSlices,
                           Time switchingTime)
{
  NS_ASSERT_MSG(numSlices > 0, "OCS schedule must have at least one slice");
  NS_ASSERT_MSG(sliceDuration.GetTimeStep() > 0,
                "OCS slice duration must be positive");
  NS_ASSERT_MSG(switchingTime.GetTimeStep() < sliceDuration.GetTimeStep(),
                "OCS switching time must be smaller than slice duration");

  m_epochStart = epochStart;
  m_sliceDuration = sliceDuration;
  m_numSlices = numSlices;
  m_switchingTime = switchingTime;
  m_scheduleEnabled = true;

  // Schedule mode and static mode are mutually exclusive.
  m_circuitMap.clear();

  std::cout << "[OCS SCHEDULE CONFIG]"
            << " t=" << Simulator::Now().GetTimeStep()
            << " node=" << GetId()
            << " epoch_start=" << m_epochStart.GetTimeStep()
            << " slice_duration=" << m_sliceDuration.GetTimeStep()
            << " switching_time=" << m_switchingTime.GetTimeStep()
            << " num_slices=" << m_numSlices
            << std::endl;
}

void
OcsNode::ClearSchedule()
{
  m_schedule.clear();

  std::cout << "[OCS SCHEDULE CLEAR]"
            << " t=" << Simulator::Now().GetTimeStep()
            << " node=" << GetId()
            << std::endl;
}

void
OcsNode::AddScheduleEntry(uint32_t inPort,
                          uint32_t slice,
                          uint32_t outPort)
{
  NS_ASSERT_MSG(inPort != outPort, "OCS self-circuit is not allowed");
  NS_ASSERT_MSG(inPort < GetNDevices(), "OCS input port out of range");
  NS_ASSERT_MSG(outPort < GetNDevices(), "OCS output port out of range");
  NS_ASSERT_MSG(m_scheduleEnabled, "ConfigureSchedule() must be called first");
  NS_ASSERT_MSG(m_numSlices > 0, "OCS schedule must have at least one slice");
  NS_ASSERT_MSG(slice < m_numSlices, "OCS schedule slice out of range");

  uint64_t key = MakeScheduleKey(inPort, slice);

  std::unordered_map<uint64_t, uint32_t>::iterator old = m_schedule.find(key);
  if (old != m_schedule.end())
    {
      std::cout << "[OCS SCHEDULE WARN OVERWRITE]"
                << " t=" << Simulator::Now().GetTimeStep()
                << " node=" << GetId()
                << " slice=" << slice
                << " inPort=" << inPort
                << " oldOut=" << old->second
                << " newOut=" << outPort
                << std::endl;
    }

  m_schedule[key] = outPort;

  std::cout << "[OCS SCHEDULE ENTRY]"
            << " t=" << Simulator::Now().GetTimeStep()
            << " node=" << GetId()
            << " slice=" << slice
            << " " << inPort << "->" << outPort
            << std::endl;
}

void
OcsNode::AddBidirectionalScheduleEntry(uint32_t portA,
                                       uint32_t portB,
                                       uint32_t slice)
{
  AddScheduleEntry(portA, slice, portB);
  AddScheduleEntry(portB, slice, portA);
}

uint32_t
OcsNode::GetCurrentSlice() const
{
  if (!m_scheduleEnabled || m_numSlices == 0)
    {
      return 0;
    }

  Time now = Simulator::Now();

  if (now < m_epochStart)
    {
      return 0;
    }

  uint64_t elapsed = (now - m_epochStart).GetTimeStep();
  uint64_t sliceTicks = m_sliceDuration.GetTimeStep();

  if (sliceTicks == 0)
    {
      return 0;
    }

  return static_cast<uint32_t>((elapsed / sliceTicks) % m_numSlices);
}

bool
OcsNode::IsScheduleEnabled() const
{
  return m_scheduleEnabled;
}

bool
OcsNode::IsInSwitchingTime() const
{
  if (!m_scheduleEnabled || m_switchingTime.IsZero())
    {
      return false;
    }

  Time now = Simulator::Now();

  if (now < m_epochStart)
    {
      return false;
    }

  uint64_t elapsed = (now - m_epochStart).GetTimeStep();
  uint64_t sliceTicks = m_sliceDuration.GetTimeStep();
  uint64_t switchingTicks = m_switchingTime.GetTimeStep();

  if (sliceTicks == 0 || switchingTicks == 0)
    {
      return false;
    }

  uint64_t offset = elapsed % sliceTicks;

  return offset >= (sliceTicks - switchingTicks);
}

bool
OcsNode::HasCircuit(uint32_t inPort) const
{
  uint32_t outPort = 0;
  return LookupOutPort(inPort, outPort);
}

bool
OcsNode::LookupScheduledOutPort(uint32_t inPort, uint32_t& outPort) const
{
  if (!m_scheduleEnabled)
    {
      return false;
    }

  uint32_t slice = GetCurrentSlice();
  uint64_t key = MakeScheduleKey(inPort, slice);

  std::unordered_map<uint64_t, uint32_t>::const_iterator it = m_schedule.find(key);
  if (it == m_schedule.end())
    {
      return false;
    }

  outPort = it->second;
  return true;
}

bool
OcsNode::LookupOutPort(uint32_t inPort, uint32_t& outPort) const
{
  if (m_scheduleEnabled)
    {
      return LookupScheduledOutPort(inPort, outPort);
    }

  std::unordered_map<uint32_t, uint32_t>::const_iterator it = m_circuitMap.find(inPort);
  if (it == m_circuitMap.end())
    {
      return false;
    }

  outPort = it->second;
  return true;
}

OcsNode::OcsForwardResult
OcsNode::ForwardOverCircuit(uint32_t outPort,
                            Ptr<Packet> packet,
                            CustomHeader &ch)
{
  Ptr<NetDevice> ocsOutDev = GetDevice(outPort);
  if (ocsOutDev == 0)
    {
      return OCS_FORWARD_BAD_DEVICE;
    }

  Ptr<Channel> channel = ocsOutDev->GetChannel();
  if (channel == 0)
    {
      return OCS_FORWARD_BAD_DEVICE;
    }

  Ptr<NetDevice> peerDev = 0;
  for (uint32_t i = 0; i < channel->GetNDevices(); ++i)
    {
      Ptr<NetDevice> dev = channel->GetDevice(i);
      if (dev != ocsOutDev)
        {
          peerDev = dev;
          break;
        }
    }

  if (peerDev == 0)
    {
      return OCS_FORWARD_BAD_DEVICE;
    }

  // QbbNetDevice::Receive() adds a FlowIdTag
  // before entering OcsNode::SwitchReceiveFromDevice(). Since transparent
  // circuit forwarding directly invokes the peer Receive(), remove the OCS
  // ingress tag first to avoid duplicate PacketTagList assertions at the peer.
  FlowIdTag t;
  packet->RemovePacketTag(t);

  Time propagationDelay = NanoSeconds(0);

  Ptr<QbbChannel> qbbChannel = DynamicCast<QbbChannel>(channel);
  if (qbbChannel != 0)
    {
      propagationDelay = qbbChannel->GetDelay();
    }

  (void)ch;

  Ptr<QbbNetDevice> qbbPeer = DynamicCast<QbbNetDevice>(peerDev);
  if (qbbPeer != 0)
    {
      Simulator::Schedule(propagationDelay,
                          &QbbNetDevice::Receive,
                          qbbPeer,
                          packet);
      return OCS_FORWARD_OK;
    }

  return OCS_FORWARD_BAD_DEVICE;
}

bool
OcsNode::SwitchReceiveFromDevice(Ptr<NetDevice> device,
                                 Ptr<Packet> packet,
                                 CustomHeader &ch)
{
  uint32_t inPort = device->GetIfIndex();

  if (m_scheduleEnabled && IsInSwitchingTime())
    {
      m_dropSwitching++;

      std::cout << "[OCS DROP SWITCHING]"
                << " t=" << Simulator::Now().GetTimeStep()
                << " node=" << GetId()
                << " inPort=" << inPort
                << " slice=" << GetCurrentSlice()
                << " drop_switching=" << m_dropSwitching
                << std::endl;

      return true;
    }

  uint32_t outPort = 0;
  if (!LookupOutPort(inPort, outPort))
    {
      m_dropNoCircuit++;

      std::cout << "[OCS DROP NO_CIRCUIT]"
                << " t=" << Simulator::Now().GetTimeStep()
                << " node=" << GetId()
                << " inPort=" << inPort;

      if (m_scheduleEnabled)
        {
          std::cout << " slice=" << GetCurrentSlice();
        }

      std::cout << " drop_no_circuit=" << m_dropNoCircuit
                << std::endl;

      return true;
    }

  if (outPort >= GetNDevices())
    {
      m_dropBadPort++;

      std::cout << "[OCS DROP BAD_PORT]"
                << " t=" << Simulator::Now().GetTimeStep()
                << " node=" << GetId()
                << " inPort=" << inPort
                << " outPort=" << outPort
                << " nDevices=" << GetNDevices()
                << " drop_bad_port=" << m_dropBadPort
                << std::endl;

      return true;
    }

  uint32_t pktSize = packet->GetSize();

  OcsForwardResult result = ForwardOverCircuit(outPort, packet, ch);

  if (result == OCS_FORWARD_BAD_DEVICE)
    {
      m_dropBadDevice++;

      std::cout << "[OCS DROP BAD_DEV]"
                << " t=" << Simulator::Now().GetTimeStep()
                << " node=" << GetId()
                << " inPort=" << inPort
                << " outPort=" << outPort
                << " drop_bad_device=" << m_dropBadDevice
                << std::endl;

      return true;
    }

  m_forwardedPackets++;
  m_forwardedBytes += pktSize;
  m_forwardedPacketsByOutPort[outPort]++;
  m_forwardedBytesByOutPort[outPort] += pktSize;

  return true;
}

void
OcsNode::SwitchNotifyDequeue(uint32_t ifIndex,
                             uint32_t qIndex,
                             Ptr<Packet> p)
{
  // Transparent OCS has no MMU/PFC/ECN state.
  // This callback is intentionally empty.
}

void
OcsNode::DumpStats() const
{
  std::cout << "[OCS STATS]"
            << " node=" << GetId()
            << " schedule_enabled=" << (m_scheduleEnabled ? 1 : 0)
            << " current_slice=" << GetCurrentSlice()
            << " forwarded_packets=" << m_forwardedPackets
            << " forwarded_bytes=" << m_forwardedBytes
            << " drop_no_circuit=" << m_dropNoCircuit
            << " drop_switching=" << m_dropSwitching
            << " drop_bad_port=" << m_dropBadPort
            << " drop_bad_device=" << m_dropBadDevice
            << std::endl;

  for (std::map<uint32_t, uint64_t>::const_iterator it = m_forwardedBytesByOutPort.begin();
       it != m_forwardedBytesByOutPort.end();
       ++it)
    {
      uint32_t outPort = it->first;
      uint64_t bytes = it->second;

      std::map<uint32_t, uint64_t>::const_iterator pktIt =
        m_forwardedPacketsByOutPort.find(outPort);

      uint64_t packets = 0;
      if (pktIt != m_forwardedPacketsByOutPort.end())
        {
          packets = pktIt->second;
        }

      std::cout << "[OCS PORT STATS]"
                << " node=" << GetId()
                << " outPort=" << outPort
                << " packets=" << packets
                << " bytes=" << bytes
                << std::endl;
    }
}

} // namespace ns3
