/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/tdm-controller.h"
#include "ns3/ocs-node.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/abort.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>
#include <queue>
#include <functional>
#include "ns3/rdma-driver.h"
#include "ns3/rdma-hw.h"
#include "ns3/rdma-transport.h"
#include "ns3/switch-node.h"
#include "ns3/qbb-net-device.h"
#include "ns3/time-flow-table.h"

namespace ns3 {

static uint64_t
GcdUint64 (uint64_t a, uint64_t b)
{
  while (b != 0)
    {
      uint64_t t = b;
      b = a % b;
      a = t;
    }
  return a;
}

static uint64_t
LcmUint64 (uint64_t a, uint64_t b)
{
  if (a == 0 || b == 0)
    {
      return 0;
    }
  return a / GcdUint64 (a, b) * b;
}

static std::string
FormatDestinationBitmap (const std::vector<uint64_t> &bitmapWords)
{
  std::ostringstream os;
  bool first = true;
  for (uint32_t wordIndex = 0; wordIndex < bitmapWords.size (); ++wordIndex)
    {
      uint64_t word = bitmapWords[wordIndex];
      for (uint32_t bitIndex = 0; bitIndex < 64; ++bitIndex)
        {
          if ((word & (1ULL << bitIndex)) == 0)
            {
              continue;
            }
          if (!first)
            {
              os << ",";
            }
          os << (wordIndex * 64 + bitIndex);
          first = false;
        }
    }
  return first ? "-" : os.str ();
}

class SimpleDsu
{
public:
  explicit SimpleDsu (uint32_t n)
  {
    m_parent.resize (n);
    for (uint32_t i = 0; i < n; ++i)
      {
        m_parent[i] = i;
      }
  }

  uint32_t Find (uint32_t x)
  {
    if (m_parent[x] != x)
      {
        m_parent[x] = Find (m_parent[x]);
      }
    return m_parent[x];
  }

  void Unite (uint32_t a, uint32_t b)
  {
    uint32_t ra = Find (a);
    uint32_t rb = Find (b);

    if (ra != rb)
      {
        m_parent[rb] = ra;
      }
  }

private:
  std::vector<uint32_t> m_parent;
};

NS_LOG_COMPONENT_DEFINE ("TdmController");
NS_OBJECT_ENSURE_REGISTERED (TdmController);

TypeId
TdmController::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TdmController")
    .SetParent<Object> ()
    .SetGroupName ("PointToPoint")
    .AddConstructor<TdmController> ();
  return tid;
}

TdmController::TdmController ()
{
}

TdmController::~TdmController ()
{
}

void
TdmController::SetNodeContainer (NodeContainer nodes)
{
  m_nodes = nodes;
}

void
TdmController::AddOcsNode (uint32_t nodeId)
{
  NS_ASSERT_MSG (nodeId < m_nodes.GetN (), "OCS node id out of range");

  Ptr<OcsNode> ocs = DynamicCast<OcsNode> (m_nodes.Get (nodeId));
  NS_ASSERT_MSG (ocs != 0, "AddOcsNode points to a non-OCS node");

  m_ocsNodeIds.insert (nodeId);
}

void
TdmController::AddTimeFlowSwitch (uint32_t nodeId)
{
  NS_ASSERT_MSG (nodeId < m_nodes.GetN (), "Time-flow switch node id out of range");
  Ptr<SwitchNode> sw = DynamicCast<SwitchNode> (m_nodes.Get (nodeId));
  NS_ASSERT_MSG (sw != 0, "AddTimeFlowSwitch points to a non-EPS node");
  m_timeFlowSwitchIds.insert (nodeId);
}

void
TdmController::AddPortBinding (uint32_t nodeId,
                                   uint32_t logicalPort,
                                   uint32_t ifIndex,
                                   uint32_t peerNodeId,
                                   uint32_t peerLogicalPort,
                                   uint64_t linkDelayNs,
                                   uint64_t linkBandwidthBps)
{
  PortBinding binding;
  binding.ifIndex = ifIndex;
  binding.peerNodeId = peerNodeId;
  binding.peerLogicalPort = peerLogicalPort;
  binding.linkDelayNs = linkDelayNs;
  binding.linkBandwidthBps = linkBandwidthBps;

  NS_ASSERT_MSG (m_portBindings[nodeId].find (logicalPort) ==
                   m_portBindings[nodeId].end (),
                 "Duplicate logical port binding");

  m_portBindings[nodeId][logicalPort] = binding;
}

bool
TdmController::IsOcsNode (uint32_t nodeId) const
{
  return m_ocsNodeIds.find (nodeId) != m_ocsNodeIds.end ();
}

uint32_t
TdmController::ResolveLogicalPortToIf (uint32_t nodeId,
                                           uint32_t logicalPort) const
{
  std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
    m_portBindings.find (nodeId);

  NS_ASSERT_MSG (nodeIt != m_portBindings.end (),
                 "No port binding found for node");

  std::map<uint32_t, PortBinding>::const_iterator portIt =
    nodeIt->second.find (logicalPort);

  NS_ASSERT_MSG (portIt != nodeIt->second.end (),
                 "Logical port not found");

  return portIt->second.ifIndex;
}

void
TdmController::LoadAndInstallOcsMap (const std::string &filename)
{
  if (filename.empty ())
    {
      std::cout << "[OCS MAP] no map file configured" << std::endl;
      return;
    }

  std::ifstream fin (filename.c_str ());
  if (!fin.is_open ())
    {
      std::cout << "Cannot open OCS_MAP_FILE: " << filename << std::endl;
      return;
    }

  std::map<uint32_t, std::vector<std::pair<uint32_t, uint32_t> > > ocsMappings;

  std::string line;
  uint32_t lineNo = 0;

  while (std::getline (fin, line))
    {
      lineNo++;

      size_t commentPos = line.find ('#');
      if (commentPos != std::string::npos)
        {
          line = line.substr (0, commentPos);
        }

      std::stringstream ss (line);

      uint32_t ocsId;
      uint32_t logicalPortA;
      uint32_t logicalPortB;

      if (!(ss >> ocsId >> logicalPortA >> logicalPortB))
        {
          continue;
        }

      std::string extra;
      if (ss >> extra)
        {
          std::cout << "[WARN] Extra field in OCS_MAP_FILE at line "
                    << lineNo << ": " << extra << std::endl;
        }

      NS_ASSERT_MSG (ocsId < m_nodes.GetN (), "OCS map points to invalid node id");
      NS_ASSERT_MSG (IsOcsNode (ocsId), "OCS_MAP_FILE points to a non-OCS node");

      uint32_t actualIfA = ResolveLogicalPortToIf (ocsId, logicalPortA);
      uint32_t actualIfB = ResolveLogicalPortToIf (ocsId, logicalPortB);

      std::cout << "[LOAD OCS MAP] OCS " << ocsId
                << " logical " << logicalPortA
                << "(if " << actualIfA << ")"
                << " <-> "
                << "logical " << logicalPortB
                << "(if " << actualIfB << ")"
                << std::endl;

      ocsMappings[ocsId].push_back (std::make_pair (actualIfA, actualIfB));
      ocsMappings[ocsId].push_back (std::make_pair (actualIfB, actualIfA));
    }

  fin.close ();

  for (std::map<uint32_t, std::vector<std::pair<uint32_t, uint32_t> > >::iterator it =
         ocsMappings.begin ();
       it != ocsMappings.end ();
       ++it)
    {
      uint32_t id = it->first;

      Ptr<OcsNode> ocs = DynamicCast<OcsNode> (m_nodes.Get (id));
      NS_ASSERT_MSG (ocs != 0, "OCS map points to a non-OCS node");

      ocs->SetInitialMapping (it->second);
    }

  for (std::set<uint32_t>::iterator it = m_ocsNodeIds.begin ();
       it != m_ocsNodeIds.end ();
       ++it)
    {
      if (ocsMappings.find (*it) == ocsMappings.end ())
        {
          std::cout << "[WARN] OCS " << *it
                    << " has no initial mapping"
                    << std::endl;
        }
    }
}

void
TdmController::LoadAndInstallOcsSchedule (const std::string &filename)
{
  if (filename.empty ())
    {
      std::cout << "[OCS SCHEDULE] no schedule file configured" << std::endl;
      return;
    }

  std::ifstream fin (filename.c_str ());
  if (!fin.is_open ())
    {
      std::cout << "Cannot open OCS_SCHEDULE_FILE: " << filename << std::endl;
      return;
    }

  typedef std::pair<uint32_t, uint32_t> PortPair;
  typedef std::pair<uint32_t, PortPair> SliceCircuit;

  // ocsId -> vector<(slice, (actualIfA, actualIfB))>
  std::map<uint32_t, std::vector<SliceCircuit> > scheduleEntries;

  // ocsId -> slice -> used logical ports
  std::map<uint32_t, std::map<uint32_t, std::set<uint32_t> > > usedLogicalPorts;

  m_ocsScheduleConfigs.clear ();
  m_ocsScheduleEntries.clear ();

  std::string line;
  uint32_t lineNo = 0;

  while (std::getline (fin, line))
    {
      lineNo++;

      size_t commentPos = line.find ('#');
      if (commentPos != std::string::npos)
        {
          line = line.substr (0, commentPos);
        }

      std::stringstream ss (line);
      std::string first;

      if (!(ss >> first))
        {
          continue;
        }

      if (first == "CONFIG" || first == "config")
        {
          uint32_t ocsId;
          OcsScheduleConfig cfg;

          if (!(ss >> ocsId
                   >> cfg.epochStartUs
                   >> cfg.sliceDurationUs
                   >> cfg.switchingTimeUs
                   >> cfg.numSlices))
            {
              NS_ASSERT_MSG (false,
                             "Invalid OCS CONFIG line at line " << lineNo);
            }

          std::string extra;
          if (ss >> extra)
            {
              std::cout << "[WARN] Extra field in OCS CONFIG at line "
                        << lineNo << ": " << extra << std::endl;
            }

          NS_ASSERT_MSG (ocsId < m_nodes.GetN (),
                         "OCS CONFIG points to invalid node id");
          NS_ASSERT_MSG (IsOcsNode (ocsId),
                         "OCS CONFIG points to a non-OCS node");
          NS_ASSERT_MSG (cfg.numSlices > 0,
                         "OCS CONFIG num_slices must be larger than 0");
          NS_ASSERT_MSG (cfg.sliceDurationUs > 0,
                         "OCS CONFIG slice_duration_us must be larger than 0");
          NS_ASSERT_MSG (cfg.switchingTimeUs < cfg.sliceDurationUs,
                         "OCS CONFIG switching_time_us must be smaller than slice_duration_us");

          NS_ASSERT_MSG (m_ocsScheduleConfigs.find (ocsId) ==
                           m_ocsScheduleConfigs.end (),
                         "Duplicate OCS CONFIG line");

          m_ocsScheduleConfigs[ocsId] = cfg;

          std::cout << "[LOAD OCS CONFIG] OCS " << ocsId
                    << " epoch_start_us=" << cfg.epochStartUs
                    << " slice_duration_us=" << cfg.sliceDurationUs
                    << " switching_time_us=" << cfg.switchingTimeUs
                    << " num_slices=" << cfg.numSlices
                    << std::endl;

          continue;
        }

      // Normal schedule entry:
      // ocs_id slice logical_port_a logical_port_b
      std::stringstream lineSs (line);

      uint32_t ocsId;
      uint32_t slice;
      uint32_t logicalPortA;
      uint32_t logicalPortB;

      if (!(lineSs >> ocsId >> slice >> logicalPortA >> logicalPortB))
        {
          NS_ASSERT_MSG (false,
                         "Invalid OCS schedule line at line " << lineNo);
        }

      std::string extra;
      if (lineSs >> extra)
        {
          std::cout << "[WARN] Extra field in OCS_SCHEDULE_FILE at line "
                    << lineNo << ": " << extra << std::endl;
        }

      NS_ASSERT_MSG (ocsId < m_nodes.GetN (),
                     "OCS schedule points to invalid node id");
      NS_ASSERT_MSG (IsOcsNode (ocsId),
                     "OCS_SCHEDULE_FILE points to a non-OCS node");
      NS_ASSERT_MSG (logicalPortA != logicalPortB,
                     "OCS schedule cannot connect a port to itself");

      NS_ASSERT_MSG (usedLogicalPorts[ocsId][slice].find (logicalPortA) ==
                       usedLogicalPorts[ocsId][slice].end (),
                     "OCS schedule reuses logical port A in the same slice");

      NS_ASSERT_MSG (usedLogicalPorts[ocsId][slice].find (logicalPortB) ==
                       usedLogicalPorts[ocsId][slice].end (),
                     "OCS schedule reuses logical port B in the same slice");

      usedLogicalPorts[ocsId][slice].insert (logicalPortA);
      usedLogicalPorts[ocsId][slice].insert (logicalPortB);

      uint32_t actualIfA = ResolveLogicalPortToIf (ocsId, logicalPortA);
      uint32_t actualIfB = ResolveLogicalPortToIf (ocsId, logicalPortB);

      std::cout << "[LOAD OCS SCHEDULE] OCS " << ocsId
                << " slice " << slice
                << " logical " << logicalPortA
                << "(if " << actualIfA << ")"
                << " <-> "
                << "logical " << logicalPortB
                << "(if " << actualIfB << ")"
                << std::endl;

      scheduleEntries[ocsId].push_back (
        std::make_pair (slice, std::make_pair (actualIfA, actualIfB)));

		OcsScheduleEntry entry;
		entry.ocsId = ocsId;
		entry.slice = slice;
		entry.logicalPortA = logicalPortA;
		entry.logicalPortB = logicalPortB;
		entry.actualIfA = actualIfA;
		entry.actualIfB = actualIfB;

		m_ocsScheduleEntries.push_back (entry);
	}

  fin.close ();

  for (std::map<uint32_t, std::vector<SliceCircuit> >::iterator it =
         scheduleEntries.begin ();
       it != scheduleEntries.end ();
       ++it)
    {
      uint32_t id = it->first;

      Ptr<OcsNode> ocs = DynamicCast<OcsNode> (m_nodes.Get (id));
      NS_ASSERT_MSG (ocs != 0, "OCS schedule points to a non-OCS node");

      std::map<uint32_t, OcsScheduleConfig>::iterator cfgIt =
        m_ocsScheduleConfigs.find (id);

      NS_ASSERT_MSG (cfgIt != m_ocsScheduleConfigs.end (),
                     "Missing CONFIG line for scheduled OCS node");

      OcsScheduleConfig cfg = cfgIt->second;

      ocs->ConfigureSchedule (MicroSeconds (cfg.epochStartUs),
                              MicroSeconds (cfg.sliceDurationUs),
                              cfg.numSlices,
                              MicroSeconds (cfg.switchingTimeUs));

      ocs->ClearSchedule ();

      for (uint32_t i = 0; i < it->second.size (); ++i)
        {
          uint32_t slice = it->second[i].first;
          uint32_t actualIfA = it->second[i].second.first;
          uint32_t actualIfB = it->second[i].second.second;

          NS_ASSERT_MSG (slice < cfg.numSlices,
                         "OCS schedule slice index exceeds num_slices");

          ocs->AddBidirectionalScheduleEntry (actualIfA, actualIfB, slice);
        }

      std::cout << "[OCS SCHEDULE INSTALLED] node=" << id
                << " entries=" << it->second.size ()
                << " epoch_start_us=" << cfg.epochStartUs
                << " num_slices=" << cfg.numSlices
                << " slice_duration_us=" << cfg.sliceDurationUs
                << " switching_time_us=" << cfg.switchingTimeUs
                << std::endl;
    }

  for (std::set<uint32_t>::iterator it = m_ocsNodeIds.begin ();
       it != m_ocsNodeIds.end ();
       ++it)
    {
      if (scheduleEntries.find (*it) == scheduleEntries.end ())
        {
          std::cout << "[WARN] OCS " << *it
                    << " has no schedule entries"
                    << std::endl;
        }

      if (m_ocsScheduleConfigs.find (*it) == m_ocsScheduleConfigs.end ())
        {
          std::cout << "[WARN] OCS " << *it
                    << " has no CONFIG line"
                    << std::endl;
        }
    }
}

uint32_t
TdmController::GetDegree (uint32_t nodeId) const
{
  std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator it =
    m_portBindings.find (nodeId);

  if (it == m_portBindings.end ())
    {
      return 0;
    }

  return it->second.size ();
}

bool
TdmController::IsEndpointNode (uint32_t nodeId) const
{
  if (nodeId >= m_nodes.GetN () || IsOcsNode (nodeId))
    {
      return false;
    }

  /*
   * common.h deliberately excludes GPU/NPU <-> NVSwitch scale-up links from
   * this controller.  A node of type 0 appearing here is therefore an RNIC
   * endpoint even when it exposes two or four scale-out plane ports.
   */
  return m_nodes.Get (nodeId)->GetNodeType () == 0;
}

uint32_t
TdmController::GetRnicGroupForNode (uint32_t nodeId) const
{
  std::map<uint32_t, std::vector<uint32_t> >::const_iterator it =
    m_nodeToRnicGroups.find (nodeId);

  NS_ASSERT_MSG (it != m_nodeToRnicGroups.end () && !it->second.empty (),
                 "RNIC node has no assigned RNIC group");
  NS_ASSERT_MSG (it->second.size () == 1,
                 "RNIC node belongs to multiple plane groups; use GetRnicGroupForEndpoint");

  return it->second[0];
}

uint32_t
TdmController::GetRnicGroupForEndpoint (uint32_t nodeId,
                                             uint32_t rnicPortId) const
{
  RnicEndpointKey key = std::make_pair (nodeId, rnicPortId);
  std::map<RnicEndpointKey, uint32_t>::const_iterator it =
    m_endpointToRnicGroup.find (key);

  NS_ASSERT_MSG (it != m_endpointToRnicGroup.end (),
                 "RNIC endpoint has no assigned RNIC group");
  return it->second;
}

void
TdmController::BuildRnicGroups ()
{
  m_endpointToRnicGroup.clear ();
  m_nodeToRnicGroups.clear ();
  m_rnicGroups.clear ();
  m_attachmentNodeToRnicGroup.clear ();

  uint32_t nextGroupId = 0;
  std::map<uint32_t, std::vector<RnicEndpoint> > epsToEndpoints;

  for (std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
         m_portBindings.begin ();
       nodeIt != m_portBindings.end ();
       ++nodeIt)
    {
      uint32_t nodeId = nodeIt->first;
      if (!IsEndpointNode (nodeId))
        {
          continue;
        }

      for (std::map<uint32_t, PortBinding>::const_iterator portIt =
             nodeIt->second.begin ();
           portIt != nodeIt->second.end ();
           ++portIt)
        {
          uint32_t rnicPortId = portIt->first;
          const PortBinding &binding = portIt->second;

          RnicEndpoint endpoint;
          endpoint.nodeId = nodeId;
          endpoint.rnicPortId = rnicPortId;

          RnicEndpointKey endpointKey =
            std::make_pair (endpoint.nodeId, endpoint.rnicPortId);
          NS_ASSERT_MSG (m_endpointToRnicGroup.find (endpointKey) ==
                           m_endpointToRnicGroup.end (),
                         "Duplicate RNIC endpoint while building plane groups");

          if (IsOcsNode (binding.peerNodeId))
            {
              RnicGroup group;
              group.groupId = nextGroupId;
              group.type = RNIC_DIRECT_OCS;
              group.attachmentNode = nodeId;
              group.attachmentLogicalPort = rnicPortId;
              group.endpoints.push_back (endpoint);

              m_rnicGroups[nextGroupId] = group;
              m_endpointToRnicGroup[endpointKey] = nextGroupId;
              m_nodeToRnicGroups[nodeId].push_back (nextGroupId);
              nextGroupId++;
            }
          else
            {
              epsToEndpoints[binding.peerNodeId].push_back (endpoint);
            }
        }
    }

  /*
   * Endpoints attached to one EPS/ToR belong to the same reachability group.
   * In a multiplane topology each plane normally has a distinct EPS node, so
   * the attachment node also identifies the plane-specific fabric component.
   */
  for (std::map<uint32_t, std::vector<RnicEndpoint> >::iterator it =
         epsToEndpoints.begin ();
       it != epsToEndpoints.end ();
       ++it)
    {
      uint32_t attachmentNode = it->first;

      RnicGroup group;
      group.groupId = nextGroupId;
      group.type = EPS_AGGREGATED;
      group.attachmentNode = attachmentNode;
      group.attachmentLogicalPort = 0;
      group.endpoints = it->second;

      m_rnicGroups[nextGroupId] = group;
      m_attachmentNodeToRnicGroup[attachmentNode] = nextGroupId;

      for (uint32_t i = 0; i < group.endpoints.size (); ++i)
        {
          const RnicEndpoint &endpoint = group.endpoints[i];
          RnicEndpointKey endpointKey =
            std::make_pair (endpoint.nodeId, endpoint.rnicPortId);
          m_endpointToRnicGroup[endpointKey] = nextGroupId;
          m_nodeToRnicGroups[endpoint.nodeId].push_back (nextGroupId);
        }

      nextGroupId++;
    }

  std::cout << "[RNIC GROUP BUILD]"
            << " endpoints=" << m_endpointToRnicGroup.size ()
            << " groups=" << m_rnicGroups.size ()
            << " granularity=node+rnic_port"
            << std::endl;
}

uint32_t
TdmController::GetGroupForOcsLogicalPort (uint32_t ocsId,
                                              uint32_t logicalPort) const
{
  std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
    m_portBindings.find (ocsId);

  NS_ASSERT_MSG (nodeIt != m_portBindings.end (),
                 "No port binding found for OCS node");

  std::map<uint32_t, PortBinding>::const_iterator portIt =
    nodeIt->second.find (logicalPort);

  NS_ASSERT_MSG (portIt != nodeIt->second.end (),
                 "OCS logical port has no port binding");

  const PortBinding &binding = portIt->second;
  uint32_t peerNode = binding.peerNodeId;

  /* Direct OCS attachment: use the exact peer RNIC logical port. */
  RnicEndpointKey endpointKey =
    std::make_pair (peerNode, binding.peerLogicalPort);
  std::map<RnicEndpointKey, uint32_t>::const_iterator endpointIt =
    m_endpointToRnicGroup.find (endpointKey);
  if (endpointIt != m_endpointToRnicGroup.end ())
    {
      return endpointIt->second;
    }

  /* EPS/ToR attachment: the plane-specific EPS node maps to one group. */
  std::map<uint32_t, uint32_t>::const_iterator attachmentGroupIt =
    m_attachmentNodeToRnicGroup.find (peerNode);
  if (attachmentGroupIt != m_attachmentNodeToRnicGroup.end ())
    {
      return attachmentGroupIt->second;
    }

  NS_ASSERT_MSG (false,
                 "OCS logical port cannot be mapped to an RNIC endpoint group");
  return 0;
}

void
TdmController::CompileRnicReachabilityWindows ()
{
  m_rnicReachabilityWindows.clear ();

  if (m_ocsScheduleEntries.empty ())
    {
      std::cout << "[RNIC REACHABILITY] no OCS schedule entries; "
                << "RNIC time-sliced gate is not required"
                << std::endl;
      return;
    }

  if (m_rnicGroups.empty ())
    {
      BuildRnicGroups ();
    }

  struct VertexInfo
  {
    bool isOcsPort;
    bool isPacketSwitch;
    uint32_t nodeId;
    uint32_t ocsId;
    uint32_t logicalPort;
  };

  struct WeightedEdge
  {
    uint32_t a;
    uint32_t b;
    uint64_t delayNs;
    uint64_t bandwidthBps;
  };

  uint32_t nextVertex = 0;

  std::map<uint32_t, uint32_t> switchToVertex;
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> ocsPortToVertex;
  std::map<uint32_t, uint32_t> groupToAttachmentVertex;
  std::vector<VertexInfo> vertexInfo;

  /*
   * Create EPS/switch vertices.
   */
  for (std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
         m_portBindings.begin ();
       nodeIt != m_portBindings.end ();
       ++nodeIt)
    {
      uint32_t nodeId = nodeIt->first;

      if (IsOcsNode (nodeId) || IsEndpointNode (nodeId))
        {
          continue;
        }

      switchToVertex[nodeId] = nextVertex++;

      VertexInfo info;
      info.isOcsPort = false;
      info.isPacketSwitch = true;
      info.nodeId = nodeId;
      info.ocsId = std::numeric_limits<uint32_t>::max ();
      info.logicalPort = std::numeric_limits<uint32_t>::max ();
      vertexInfo.push_back (info);
    }

  /*
   * Create OCS port vertices.
   */
  for (std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
         m_portBindings.begin ();
       nodeIt != m_portBindings.end ();
       ++nodeIt)
    {
      uint32_t nodeId = nodeIt->first;

      if (!IsOcsNode (nodeId))
        {
          continue;
        }

      for (std::map<uint32_t, PortBinding>::const_iterator portIt =
             nodeIt->second.begin ();
           portIt != nodeIt->second.end ();
           ++portIt)
        {
          uint32_t logicalPort = portIt->first;
          std::pair<uint32_t, uint32_t> key =
            std::make_pair (nodeId, logicalPort);

          ocsPortToVertex[key] = nextVertex++;

          VertexInfo info;
          info.isOcsPort = true;
          info.isPacketSwitch = false;
          info.nodeId = nodeId;
          info.ocsId = nodeId;
          info.logicalPort = logicalPort;
          vertexInfo.push_back (info);
        }
    }

  NS_ASSERT_MSG (vertexInfo.size () == nextVertex,
                 "internal vertex metadata size mismatch");

  /*
   * Resolve RNIC group attachment vertex.
   */
  for (std::map<uint32_t, RnicGroup>::const_iterator groupIt =
         m_rnicGroups.begin ();
       groupIt != m_rnicGroups.end ();
       ++groupIt)
    {
      uint32_t groupId = groupIt->first;
      const RnicGroup &group = groupIt->second;

      if (group.type == RNIC_DIRECT_OCS)
        {
          NS_ASSERT_MSG (group.endpoints.size () == 1,
                         "RNIC_DIRECT_OCS group should contain one endpoint");
          const RnicEndpoint &endpoint = group.endpoints[0];

          std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
            m_portBindings.find (endpoint.nodeId);
          NS_ASSERT_MSG (nodeIt != m_portBindings.end (),
                         "RNIC_DIRECT_OCS group has no RNIC port binding");

          std::map<uint32_t, PortBinding>::const_iterator portIt =
            nodeIt->second.find (endpoint.rnicPortId);
          NS_ASSERT_MSG (portIt != nodeIt->second.end (),
                         "RNIC_DIRECT_OCS endpoint port is not bound");

          const PortBinding &binding = portIt->second;
          NS_ASSERT_MSG (IsOcsNode (binding.peerNodeId),
                         "RNIC_DIRECT_OCS group is not connected to an OCS");

          std::pair<uint32_t, uint32_t> key =
            std::make_pair (binding.peerNodeId, binding.peerLogicalPort);

          std::map<std::pair<uint32_t, uint32_t>, uint32_t>::const_iterator vIt =
            ocsPortToVertex.find (key);

          NS_ASSERT_MSG (vIt != ocsPortToVertex.end (),
                         "RNIC_DIRECT_OCS attachment OCS port has no vertex");

          groupToAttachmentVertex[groupId] = vIt->second;
        }
      else
        {
          uint32_t switchNode = group.attachmentNode;

          std::map<uint32_t, uint32_t>::const_iterator vIt =
            switchToVertex.find (switchNode);

          NS_ASSERT_MSG (vIt != switchToVertex.end (),
                         "EPS_AGGREGATED attachment switch has no vertex");

          groupToAttachmentVertex[groupId] = vIt->second;
        }
    }

  /*
   * Static fabric edges from physical topology.
   *
   * Endpoint links are not fabric edges because RNIC groups attach to
   * the fabric through groupToAttachmentVertex. Endpoint link delay/rate
   * is still used later to derive the source-side packet arrival offset.
   */
  std::vector<WeightedEdge> staticEdges;

  for (std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
         m_portBindings.begin ();
       nodeIt != m_portBindings.end ();
       ++nodeIt)
    {
      uint32_t nodeId = nodeIt->first;

      for (std::map<uint32_t, PortBinding>::const_iterator portIt =
             nodeIt->second.begin ();
           portIt != nodeIt->second.end ();
           ++portIt)
        {
          uint32_t logicalPort = portIt->first;
          const PortBinding &binding = portIt->second;

          uint32_t peerNode = binding.peerNodeId;
          uint32_t peerLogicalPort = binding.peerLogicalPort;

          if (nodeId > peerNode)
            {
              continue;
            }

          if (IsEndpointNode (nodeId) || IsEndpointNode (peerNode))
            {
              continue;
            }

          uint32_t vA = std::numeric_limits<uint32_t>::max ();
          uint32_t vB = std::numeric_limits<uint32_t>::max ();

          if (IsOcsNode (nodeId))
            {
              std::pair<uint32_t, uint32_t> key =
                std::make_pair (nodeId, logicalPort);
              std::map<std::pair<uint32_t, uint32_t>, uint32_t>::const_iterator it =
                ocsPortToVertex.find (key);
              NS_ASSERT_MSG (it != ocsPortToVertex.end (),
                             "OCS physical-link endpoint has no vertex");
              vA = it->second;
            }
          else
            {
              std::map<uint32_t, uint32_t>::const_iterator it =
                switchToVertex.find (nodeId);
              NS_ASSERT_MSG (it != switchToVertex.end (),
                             "Switch physical-link endpoint has no vertex");
              vA = it->second;
            }

          if (IsOcsNode (peerNode))
            {
              std::pair<uint32_t, uint32_t> key =
                std::make_pair (peerNode, peerLogicalPort);
              std::map<std::pair<uint32_t, uint32_t>, uint32_t>::const_iterator it =
                ocsPortToVertex.find (key);
              NS_ASSERT_MSG (it != ocsPortToVertex.end (),
                             "Peer OCS physical-link endpoint has no vertex");
              vB = it->second;
            }
          else
            {
              std::map<uint32_t, uint32_t>::const_iterator it =
                switchToVertex.find (peerNode);
              NS_ASSERT_MSG (it != switchToVertex.end (),
                             "Peer switch physical-link endpoint has no vertex");
              vB = it->second;
            }

          if (vA != vB)
            {
              WeightedEdge edge;
              edge.a = vA;
              edge.b = vB;
              edge.delayNs = binding.linkDelayNs;
              edge.bandwidthBps = binding.linkBandwidthBps;
              staticEdges.push_back (edge);
            }
        }
    }

  /*
   * Do not pre-eliminate destinations that are already reachable through
   * static EPS edges.  The RNIC gate table is a per-window reachability
   * bitmap over the mixed fabric, not an OCS-only bitmap.  Static EPS
   * reachability must therefore be present in every applicable window;
   * otherwise same-ToR or multi-hop-EPS traffic is incorrectly blocked
   * when the RNIC gate is enabled.
   */

  uint64_t commonPeriodNs = 0;
  for (std::map<uint32_t, OcsScheduleConfig>::const_iterator it =
         m_ocsScheduleConfigs.begin ();
       it != m_ocsScheduleConfigs.end ();
       ++it)
    {
      const OcsScheduleConfig &cfg = it->second;
      uint64_t periodNs =
        static_cast<uint64_t> (cfg.sliceDurationUs) *
        static_cast<uint64_t> (cfg.numSlices) * 1000ULL;

      if (commonPeriodNs == 0)
        {
          commonPeriodNs = periodNs;
        }
      else
        {
          commonPeriodNs = LcmUint64 (commonPeriodNs, periodNs);
        }
    }

  NS_ASSERT_MSG (commonPeriodNs > 0,
                 "Invalid common OCS calendar period");

  std::vector<uint64_t> boundaries;
  boundaries.push_back (0);
  boundaries.push_back (commonPeriodNs);

  for (std::map<uint32_t, OcsScheduleConfig>::const_iterator cfgIt =
         m_ocsScheduleConfigs.begin ();
       cfgIt != m_ocsScheduleConfigs.end ();
       ++cfgIt)
    {
      const OcsScheduleConfig &cfg = cfgIt->second;

      uint64_t epochNs = static_cast<uint64_t> (cfg.epochStartUs) * 1000ULL;
      uint64_t sliceDurationNs = static_cast<uint64_t> (cfg.sliceDurationUs) * 1000ULL;
      uint64_t switchingTimeNs = static_cast<uint64_t> (cfg.switchingTimeUs) * 1000ULL;
      uint64_t periodNs = sliceDurationNs * static_cast<uint64_t> (cfg.numSlices);
      uint64_t repeat = commonPeriodNs / periodNs;

      for (uint64_t k = 0; k < repeat; ++k)
        {
          uint64_t base = epochNs + k * periodNs;

          for (uint32_t s = 0; s < cfg.numSlices; ++s)
            {
              uint64_t sliceStart = base + static_cast<uint64_t> (s) * sliceDurationNs;
              uint64_t stableEnd = base +
                static_cast<uint64_t> (s + 1) * sliceDurationNs - switchingTimeNs;
              uint64_t sliceEnd = base +
                static_cast<uint64_t> (s + 1) * sliceDurationNs;

              boundaries.push_back (sliceStart % commonPeriodNs);
              boundaries.push_back (stableEnd % commonPeriodNs);
              boundaries.push_back (sliceEnd % commonPeriodNs);
            }
        }
    }

  std::sort (boundaries.begin (), boundaries.end ());
  boundaries.erase (std::unique (boundaries.begin (), boundaries.end ()),
                    boundaries.end ());

  for (uint32_t bi = 0; bi + 1 < boundaries.size (); ++bi)
    {
      uint64_t intervalStartNs = boundaries[bi];
      uint64_t intervalEndNs = boundaries[bi + 1];

      if (intervalEndNs <= intervalStartNs)
        {
          continue;
        }

      SimpleDsu dsu (nextVertex);

      for (uint32_t i = 0; i < staticEdges.size (); ++i)
        {
          const WeightedEdge &edge = staticEdges[i];
          dsu.Unite (edge.a, edge.b);
        }

      for (uint32_t i = 0; i < m_ocsScheduleEntries.size (); ++i)
        {
          const OcsScheduleEntry &entry = m_ocsScheduleEntries[i];

          std::map<uint32_t, OcsScheduleConfig>::const_iterator cfgIt =
            m_ocsScheduleConfigs.find (entry.ocsId);
          NS_ASSERT_MSG (cfgIt != m_ocsScheduleConfigs.end (),
                         "Missing OCS schedule config");

          const OcsScheduleConfig &cfg = cfgIt->second;
          uint64_t epochNs = static_cast<uint64_t> (cfg.epochStartUs) * 1000ULL;
          uint64_t sliceDurationNs = static_cast<uint64_t> (cfg.sliceDurationUs) * 1000ULL;
          uint64_t switchingTimeNs = static_cast<uint64_t> (cfg.switchingTimeUs) * 1000ULL;
          uint64_t periodNs = sliceDurationNs * static_cast<uint64_t> (cfg.numSlices);

          uint64_t rel;
          if (intervalStartNs >= epochNs)
            {
              rel = (intervalStartNs - epochNs) % periodNs;
            }
          else
            {
              uint64_t delta = epochNs - intervalStartNs;
              rel = (periodNs - (delta % periodNs)) % periodNs;
            }

          uint32_t activeSlice = static_cast<uint32_t> (rel / sliceDurationNs);
          uint64_t offsetInSlice = rel % sliceDurationNs;

          bool active =
            (entry.slice == activeSlice) &&
            (offsetInSlice < (sliceDurationNs - switchingTimeNs));

          if (!active)
            {
              continue;
            }

          std::pair<uint32_t, uint32_t> keyA =
            std::make_pair (entry.ocsId, entry.logicalPortA);
          std::pair<uint32_t, uint32_t> keyB =
            std::make_pair (entry.ocsId, entry.logicalPortB);

          std::map<std::pair<uint32_t, uint32_t>, uint32_t>::const_iterator itA =
            ocsPortToVertex.find (keyA);
          std::map<std::pair<uint32_t, uint32_t>, uint32_t>::const_iterator itB =
            ocsPortToVertex.find (keyB);

          NS_ASSERT_MSG (itA != ocsPortToVertex.end (),
                         "OCS schedule port A has no graph vertex");
          NS_ASSERT_MSG (itB != ocsPortToVertex.end (),
                         "OCS schedule port B has no graph vertex");

          dsu.Unite (itA->second, itB->second);
        }

      for (std::map<uint32_t, uint32_t>::const_iterator srcIt =
             groupToAttachmentVertex.begin ();
           srcIt != groupToAttachmentVertex.end ();
           ++srcIt)
        {
          uint32_t srcGroup = srcIt->first;
          uint32_t srcVertex = srcIt->second;

          for (std::map<uint32_t, uint32_t>::const_iterator dstIt =
                 groupToAttachmentVertex.begin ();
               dstIt != groupToAttachmentVertex.end ();
               ++dstIt)
            {
              uint32_t dstGroup = dstIt->first;
              uint32_t dstVertex = dstIt->second;

              /*
               * Keep srcGroup == dstGroup and static-EPS-reachable
               * destinations.  They are valid destinations for the RNIC
               * injection table.  The per-RNIC bitmap expansion below will
               * remove only dstRnic == srcRnic, so intra-group traffic such
               * as 6->7 remains injectable.
               */
              if (dsu.Find (srcVertex) != dsu.Find (dstVertex))
                {
                  continue;
                }


              /*
               * The controller exports only the physical path-stable interval.
               * DATA/ACK timing reserves are calculated at the RNIC at run time.
               */

              RnicReachabilityWindow window;
              window.srcGroup = srcGroup;
              window.dstGroup = dstGroup;
              window.ocsId = std::numeric_limits<uint32_t>::max ();
              window.slice = bi;
              window.startOffset = NanoSeconds (intervalStartNs);
              window.endOffset = NanoSeconds (intervalEndNs);
              window.period = NanoSeconds (commonPeriodNs);

              m_rnicReachabilityWindows.push_back (window);
            }
        }
    }
}

void
TdmController::InstallTimeHashReachability () const
{
  typedef std::map<RnicEndpointKey, std::set<uint32_t> > ReachabilityMap;
  ReachabilityMap reachableByEndpoint;

  for (uint32_t i = 0; i < m_rnicReachabilityWindows.size (); ++i)
    {
      const RnicReachabilityWindow &window = m_rnicReachabilityWindows[i];
      std::map<uint32_t, RnicGroup>::const_iterator srcIt =
        m_rnicGroups.find (window.srcGroup);
      std::map<uint32_t, RnicGroup>::const_iterator dstIt =
        m_rnicGroups.find (window.dstGroup);

      NS_ASSERT_MSG (srcIt != m_rnicGroups.end (),
                     "TIME HASH source group is missing");
      NS_ASSERT_MSG (dstIt != m_rnicGroups.end (),
                     "TIME HASH destination group is missing");

      const RnicGroup &srcGroup = srcIt->second;
      const RnicGroup &dstGroup = dstIt->second;

      for (uint32_t sidx = 0; sidx < srcGroup.endpoints.size (); ++sidx)
        {
          const RnicEndpoint &src = srcGroup.endpoints[sidx];
          RnicEndpointKey key = std::make_pair (src.nodeId, src.rnicPortId);

          for (uint32_t didx = 0; didx < dstGroup.endpoints.size (); ++didx)
            {
              uint32_t dstNode = dstGroup.endpoints[didx].nodeId;
              if (dstNode != src.nodeId)
                {
                  reachableByEndpoint[key].insert (dstNode);
                }
            }
        }
    }

  std::set<uint32_t> endpointNodes;
  for (std::map<RnicEndpointKey, uint32_t>::const_iterator endpointIt =
         m_endpointToRnicGroup.begin ();
       endpointIt != m_endpointToRnicGroup.end ();
       ++endpointIt)
    {
      endpointNodes.insert (endpointIt->first.first);
    }

  for (std::set<uint32_t>::const_iterator nodeIt = endpointNodes.begin ();
       nodeIt != endpointNodes.end ();
       ++nodeIt)
    {
      uint32_t nodeId = *nodeIt;
      NS_ASSERT_MSG (nodeId < m_nodes.GetN (),
                     "TIME HASH endpoint node is out of range");
      Ptr<RdmaDriver> driver = m_nodes.Get (nodeId)->GetObject<RdmaDriver> ();
      NS_ASSERT_MSG (driver != 0 && driver->m_rdma != 0,
                     "TIME HASH endpoint has no RdmaHw");
      driver->m_rdma->ClearTimeHashReachability ();
    }

  uint32_t installedEndpointTables = 0;
  uint64_t installedPairPortMappings = 0;

  for (ReachabilityMap::const_iterator it = reachableByEndpoint.begin ();
       it != reachableByEndpoint.end ();
       ++it)
    {
      uint32_t nodeId = it->first.first;
      uint32_t rnicPortId = it->first.second;

      NS_ASSERT_MSG (nodeId < m_nodes.GetN (),
                     "TIME HASH endpoint node is out of range");

      Ptr<RdmaDriver> driver = m_nodes.Get (nodeId)->GetObject<RdmaDriver> ();
      NS_ASSERT_MSG (driver != 0 && driver->m_rdma != 0,
                     "TIME HASH endpoint has no RdmaHw");

      Ptr<RdmaHw> rdmaHw = driver->m_rdma;

      for (std::set<uint32_t>::const_iterator dstIt = it->second.begin ();
           dstIt != it->second.end ();
           ++dstIt)
        {
          rdmaHw->AddTimeHashReachability (*dstIt, rnicPortId);
          ++installedPairPortMappings;
        }

      std::cout << "[TIME HASH REACHABILITY]"
                << " node=" << nodeId
                << " rnic_port=" << rnicPortId
                << " destinations=" << it->second.size ()
                << std::endl;
      ++installedEndpointTables;
    }

  std::cout << "[TIME HASH SUMMARY]"
            << " endpoint_tables=" << installedEndpointTables
            << " pair_port_mappings=" << installedPairPortMappings
            << std::endl;
}

void
TdmController::DumpRnicGroups () const
{
  for (std::map<uint32_t, RnicGroup>::const_iterator it =
         m_rnicGroups.begin ();
       it != m_rnicGroups.end ();
       ++it)
    {
      const RnicGroup &group = it->second;

      std::cout << "[RNIC GROUP] id=" << group.groupId
                << " type="
                << (group.type == RNIC_DIRECT_OCS
                      ? "RNIC_DIRECT_OCS"
                      : "EPS_AGGREGATED")
                << " attachmentNode=" << group.attachmentNode
                << " members=";

      for (uint32_t i = 0; i < group.endpoints.size (); ++i)
        {
          if (i > 0)
            {
              std::cout << ",";
            }
          std::cout << group.endpoints[i].nodeId
                    << ":" << group.endpoints[i].rnicPortId;
        }

      std::cout << std::endl;
    }
}

void
TdmController::DumpRnicReachabilityWindows () const
{
  std::cout
    << "[INJECTION WINDOW SUMMARY]"
    << " endpoints=" << m_endpointToRnicGroup.size ()
    << " rawWindows=" << m_rnicReachabilityWindows.size ()
    << " srcMode=NODE_RNIC_PORT"
    << " dstMode=DST_NODE_BITMAP"
    << std::endl;
}

void
TdmController::InstallRdmaGateTables (uint32_t mode) const
{
  NS_ASSERT_MSG (mode <= 2,
                 "RDMA gate mode must be 0(default), 1(RNIC), or 2(userspace)");
  if (mode == 0)
    {
      return;
    }
  if (mode == 1)
    {
      InstallRnicGateTablesToRdmaHw ();
      return;
    }
  InstallRnicGateTablesToUserspace ();
}

void
TdmController::InstallRnicGateTablesToRdmaHw () const
{
  struct GateSlot
  {
    uint64_t startOffsetNs;
    uint64_t endOffsetNs;
    uint64_t periodNs;
    std::vector<uint64_t> bitmapWords;
  };

  std::vector<RnicEndpointKey> allEndpoints;
  for (std::map<RnicEndpointKey, uint32_t>::const_iterator it =
         m_endpointToRnicGroup.begin ();
       it != m_endpointToRnicGroup.end ();
       ++it)
    {
      allEndpoints.push_back (it->first);
    }
  std::sort (allEndpoints.begin (), allEndpoints.end ());

  uint32_t bitmapWordCount =
    static_cast<uint32_t> ((m_nodes.GetN () + 63) / 64);
  if (bitmapWordCount == 0)
    {
      bitmapWordCount = 1;
    }

  std::map<RnicEndpointKey, std::vector<GateSlot> > tables;

  for (uint32_t i = 0; i < m_rnicReachabilityWindows.size (); ++i)
    {
      const RnicReachabilityWindow &w = m_rnicReachabilityWindows[i];
      const RnicGroup &srcGroup = m_rnicGroups.find (w.srcGroup)->second;
      const RnicGroup &dstGroup = m_rnicGroups.find (w.dstGroup)->second;

      uint64_t startOffsetNs =
        static_cast<uint64_t> (w.startOffset.GetNanoSeconds ());
      uint64_t endOffsetNs =
        static_cast<uint64_t> (w.endOffset.GetNanoSeconds ());
      uint64_t periodNs =
        static_cast<uint64_t> (w.period.GetNanoSeconds ());

      for (uint32_t sidx = 0; sidx < srcGroup.endpoints.size (); ++sidx)
        {
          const RnicEndpoint &src = srcGroup.endpoints[sidx];
          RnicEndpointKey key = std::make_pair (src.nodeId, src.rnicPortId);
          std::vector<GateSlot> &slots = tables[key];


          GateSlot *slot = 0;
          for (uint32_t k = 0; k < slots.size (); ++k)
            {
              if (slots[k].startOffsetNs == startOffsetNs &&
                  slots[k].endOffsetNs == endOffsetNs &&
                  slots[k].periodNs == periodNs)
                {
                  slot = &slots[k];
                  break;
                }
            }

          if (slot == 0)
            {
              GateSlot newSlot;
              newSlot.startOffsetNs = startOffsetNs;
              newSlot.endOffsetNs = endOffsetNs;
              newSlot.periodNs = periodNs;
              newSlot.bitmapWords.assign (bitmapWordCount, 0);
              slots.push_back (newSlot);
              slot = &slots.back ();
            }

          for (uint32_t didx = 0; didx < dstGroup.endpoints.size (); ++didx)
            {
              uint32_t dstNode = dstGroup.endpoints[didx].nodeId;
              if (dstNode == src.nodeId)
                {
                  continue;
                }
              uint32_t wordIndex = dstNode / 64;
              uint32_t bitIndex = dstNode % 64;
              slot->bitmapWords[wordIndex] |= (1ULL << bitIndex);
            }
        }
    }

  /* Normalize overlapping raw windows into disjoint lookup slots. */
  for (std::map<RnicEndpointKey, std::vector<GateSlot> >::iterator it =
         tables.begin ();
       it != tables.end ();
       ++it)
    {
      std::vector<GateSlot> rawSlots = it->second;
      std::vector<uint64_t> boundaries;
      for (uint32_t k = 0; k < rawSlots.size (); ++k)
        {
          if (rawSlots[k].endOffsetNs > rawSlots[k].startOffsetNs)
            {
              boundaries.push_back (rawSlots[k].startOffsetNs);
              boundaries.push_back (rawSlots[k].endOffsetNs);
            }
        }
      std::sort (boundaries.begin (), boundaries.end ());
      boundaries.erase (std::unique (boundaries.begin (), boundaries.end ()),
                        boundaries.end ());

      std::vector<GateSlot> normalized;
      for (uint32_t b = 0; b + 1 < boundaries.size (); ++b)
        {
          GateSlot slot;
          slot.startOffsetNs = boundaries[b];
          slot.endOffsetNs = boundaries[b + 1];
          slot.periodNs = 0;
          slot.bitmapWords.assign (bitmapWordCount, 0);

          for (uint32_t k = 0; k < rawSlots.size (); ++k)
            {
              const GateSlot &raw = rawSlots[k];
              if (raw.startOffsetNs <= slot.startOffsetNs &&
                  slot.endOffsetNs <= raw.endOffsetNs)
                {
                  slot.periodNs = raw.periodNs;
                  for (uint32_t widx = 0; widx < slot.bitmapWords.size (); ++widx)
                    {
                      slot.bitmapWords[widx] |= raw.bitmapWords[widx];
                    }
                }
            }

          bool hasDst = false;
          for (uint32_t widx = 0; widx < slot.bitmapWords.size (); ++widx)
            {
              hasDst = hasDst || slot.bitmapWords[widx] != 0;
            }
          if (!hasDst)
            {
              continue;
            }

          if (!normalized.empty () &&
              normalized.back ().endOffsetNs == slot.startOffsetNs &&
              normalized.back ().periodNs == slot.periodNs &&
              normalized.back ().bitmapWords == slot.bitmapWords)
            {
              normalized.back ().endOffsetNs = slot.endOffsetNs;
            }
          else
            {
              normalized.push_back (slot);
            }
        }
      it->second = normalized;
    }

  uint32_t installedCount = 0;
  uint32_t maxSlots = 0;
  uint64_t summaryPeriodNs = 0;

  for (uint32_t i = 0; i < allEndpoints.size (); ++i)
    {
      const RnicEndpointKey &key = allEndpoints[i];
      std::map<RnicEndpointKey, std::vector<GateSlot> >::const_iterator tableIt =
        tables.find (key);
      if (tableIt == tables.end () || tableIt->second.empty ())
        {
          continue;
        }

      uint32_t nodeId = key.first;
      uint32_t rnicPortId = key.second;
      Ptr<RdmaDriver> rdmaDriver =
        m_nodes.Get (nodeId)->GetObject<RdmaDriver> ();
      if (rdmaDriver == 0 || rdmaDriver->m_rdma == 0)
        {
          std::cout << "[RNIC GATE INSTALL SKIP]"
                    << " node=" << nodeId
                    << " rnic_port=" << rnicPortId
                    << " reason=no_rdma_driver"
                    << std::endl;
          continue;
        }

      Ptr<RdmaTransport> transport = rdmaDriver->GetTransport ();
      if (transport == 0)
        {
          std::cout << "[RNIC GATE INSTALL SKIP]"
                    << " node=" << nodeId
                    << " rnic_port=" << rnicPortId
                    << " reason=no_rdma_transport"
                    << std::endl;
          continue;
        }

      std::vector<RdmaTransport::GateSlotEntry> hwSlots;
      uint64_t periodNs = tableIt->second[0].periodNs;
      for (uint32_t k = 0; k < tableIt->second.size (); ++k)
        {
          RdmaTransport::GateSlotEntry hwSlot;
          hwSlot.startOffsetNs = tableIt->second[k].startOffsetNs;
          hwSlot.endOffsetNs = tableIt->second[k].endOffsetNs;
          hwSlot.dstRnicBitmapWords = tableIt->second[k].bitmapWords;
          hwSlots.push_back (hwSlot);
        }

      for (uint32_t k = 0; k < hwSlots.size (); ++k)
        {
          std::cout << "[INJECTION WINDOW]"
                    << " mode=1"
                    << " layer=rnic"
                    << " node=" << nodeId
                    << " rnic_port=" << rnicPortId
                    << " plane=" << (rnicPortId & 0xffffU)
                    << " epoch_ns=0"
                    << " start_ns=" << hwSlots[k].startOffsetNs
                    << " end_ns=" << hwSlots[k].endOffsetNs
                    << " period_ns=" << periodNs
                    << " destinations="
                    << FormatDestinationBitmap (hwSlots[k].dstRnicBitmapWords)
                    << std::endl;
        }

      transport->InstallGateTable (
          rnicPortId,
          0,
          periodNs,
          hwSlots);

      std::cout << "[RNIC GATE TABLE INSTALLED]"
                << " node=" << nodeId
                << " rnic_port=" << rnicPortId
                << " periodNs=" << periodNs
                << " slots=" << hwSlots.size ()
                << std::endl;
      installedCount++;
      maxSlots = std::max (maxSlots,
                           static_cast<uint32_t> (hwSlots.size ()));
      summaryPeriodNs = periodNs;
    }

  if (installedCount > 0)
    {
      std::cout << "[RNIC GATE INSTALLED SUMMARY]"
                << " endpoint_tables=" << installedCount
                << " periodNs=" << summaryPeriodNs
                << " maxSlotsPerEndpoint=" << maxSlots
                << std::endl;
    }
}

void
TdmController::InstallRnicGateTablesToUserspace () const
{
  struct GateSlot
  {
    uint64_t startOffsetNs;
    uint64_t endOffsetNs;
    uint64_t periodNs;
    std::vector<uint64_t> bitmapWords;
  };

  std::vector<RnicEndpointKey> allEndpoints;
  for (std::map<RnicEndpointKey, uint32_t>::const_iterator it =
         m_endpointToRnicGroup.begin ();
       it != m_endpointToRnicGroup.end ();
       ++it)
    {
      allEndpoints.push_back (it->first);
    }
  std::sort (allEndpoints.begin (), allEndpoints.end ());

  uint32_t bitmapWordCount =
    static_cast<uint32_t> ((m_nodes.GetN () + 63) / 64);
  if (bitmapWordCount == 0)
    {
      bitmapWordCount = 1;
    }

  std::map<RnicEndpointKey, std::vector<GateSlot> > tables;

  for (uint32_t i = 0; i < m_rnicReachabilityWindows.size (); ++i)
    {
      const RnicReachabilityWindow &w = m_rnicReachabilityWindows[i];
      const RnicGroup &srcGroup = m_rnicGroups.find (w.srcGroup)->second;
      const RnicGroup &dstGroup = m_rnicGroups.find (w.dstGroup)->second;

      uint64_t startOffsetNs =
        static_cast<uint64_t> (w.startOffset.GetNanoSeconds ());
      uint64_t endOffsetNs =
        static_cast<uint64_t> (w.endOffset.GetNanoSeconds ());
      uint64_t periodNs =
        static_cast<uint64_t> (w.period.GetNanoSeconds ());

      for (uint32_t sidx = 0; sidx < srcGroup.endpoints.size (); ++sidx)
        {
          const RnicEndpoint &src = srcGroup.endpoints[sidx];
          RnicEndpointKey key = std::make_pair (src.nodeId, src.rnicPortId);
          std::vector<GateSlot> &slots = tables[key];

          GateSlot *slot = 0;
          for (uint32_t k = 0; k < slots.size (); ++k)
            {
              if (slots[k].startOffsetNs == startOffsetNs &&
                  slots[k].endOffsetNs == endOffsetNs &&
                  slots[k].periodNs == periodNs)
                {
                  slot = &slots[k];
                  break;
                }
            }

          if (slot == 0)
            {
              GateSlot newSlot;
              newSlot.startOffsetNs = startOffsetNs;
              newSlot.endOffsetNs = endOffsetNs;
              newSlot.periodNs = periodNs;
              newSlot.bitmapWords.assign (bitmapWordCount, 0);
              slots.push_back (newSlot);
              slot = &slots.back ();
            }

          for (uint32_t didx = 0; didx < dstGroup.endpoints.size (); ++didx)
            {
              uint32_t dstNode = dstGroup.endpoints[didx].nodeId;
              if (dstNode == src.nodeId)
                {
                  continue;
                }
              uint32_t wordIndex = dstNode / 64;
              uint32_t bitIndex = dstNode % 64;
              slot->bitmapWords[wordIndex] |= (1ULL << bitIndex);
            }
        }
    }

  /* Normalize overlapping raw windows into disjoint lookup slots. */
  for (std::map<RnicEndpointKey, std::vector<GateSlot> >::iterator it =
         tables.begin ();
       it != tables.end ();
       ++it)
    {
      std::vector<GateSlot> rawSlots = it->second;
      std::vector<uint64_t> boundaries;
      for (uint32_t k = 0; k < rawSlots.size (); ++k)
        {
          if (rawSlots[k].endOffsetNs > rawSlots[k].startOffsetNs)
            {
              boundaries.push_back (rawSlots[k].startOffsetNs);
              boundaries.push_back (rawSlots[k].endOffsetNs);
            }
        }
      std::sort (boundaries.begin (), boundaries.end ());
      boundaries.erase (std::unique (boundaries.begin (), boundaries.end ()),
                        boundaries.end ());

      std::vector<GateSlot> normalized;
      for (uint32_t b = 0; b + 1 < boundaries.size (); ++b)
        {
          GateSlot slot;
          slot.startOffsetNs = boundaries[b];
          slot.endOffsetNs = boundaries[b + 1];
          slot.periodNs = 0;
          slot.bitmapWords.assign (bitmapWordCount, 0);

          for (uint32_t k = 0; k < rawSlots.size (); ++k)
            {
              const GateSlot &raw = rawSlots[k];
              if (raw.startOffsetNs <= slot.startOffsetNs &&
                  slot.endOffsetNs <= raw.endOffsetNs)
                {
                  slot.periodNs = raw.periodNs;
                  for (uint32_t widx = 0; widx < slot.bitmapWords.size (); ++widx)
                    {
                      slot.bitmapWords[widx] |= raw.bitmapWords[widx];
                    }
                }
            }

          bool hasDst = false;
          for (uint32_t widx = 0; widx < slot.bitmapWords.size (); ++widx)
            {
              hasDst = hasDst || slot.bitmapWords[widx] != 0;
            }
          if (!hasDst)
            {
              continue;
            }

          if (!normalized.empty () &&
              normalized.back ().endOffsetNs == slot.startOffsetNs &&
              normalized.back ().periodNs == slot.periodNs &&
              normalized.back ().bitmapWords == slot.bitmapWords)
            {
              normalized.back ().endOffsetNs = slot.endOffsetNs;
            }
          else
            {
              normalized.push_back (slot);
            }
        }
      it->second = normalized;
    }

  uint32_t installedCount = 0;
  uint32_t maxSlots = 0;
  uint64_t summaryPeriodNs = 0;

  for (uint32_t i = 0; i < allEndpoints.size (); ++i)
    {
      const RnicEndpointKey &key = allEndpoints[i];
      std::map<RnicEndpointKey, std::vector<GateSlot> >::const_iterator tableIt =
        tables.find (key);
      if (tableIt == tables.end () || tableIt->second.empty ())
        {
          continue;
        }

      uint32_t nodeId = key.first;
      uint32_t rnicPortId = key.second;
      Ptr<RdmaDriver> rdmaDriver =
        m_nodes.Get (nodeId)->GetObject<RdmaDriver> ();
      if (rdmaDriver == 0 || rdmaDriver->m_rdma == 0)
        {
          std::cout << "[USERSPACE GATE INSTALL SKIP]"
                    << " node=" << nodeId
                    << " rnic_port=" << rnicPortId
                    << " reason=no_rdma_driver"
                    << std::endl;
          continue;
        }

      std::vector<RdmaTransport::GateSlotEntry> hwSlots;
      uint64_t periodNs = tableIt->second[0].periodNs;
      for (uint32_t k = 0; k < tableIt->second.size (); ++k)
        {
          RdmaTransport::GateSlotEntry hwSlot;
          hwSlot.startOffsetNs = tableIt->second[k].startOffsetNs;
          hwSlot.endOffsetNs = tableIt->second[k].endOffsetNs;
          hwSlot.dstRnicBitmapWords = tableIt->second[k].bitmapWords;
          hwSlots.push_back (hwSlot);
        }

      for (uint32_t k = 0; k < hwSlots.size (); ++k)
        {
          std::cout << "[INJECTION WINDOW]"
                    << " mode=2"
                    << " layer=userspace"
                    << " node=" << nodeId
                    << " rnic_port=" << rnicPortId
                    << " plane=" << (rnicPortId & 0xffffU)
                    << " epoch_ns=0"
                    << " start_ns=" << hwSlots[k].startOffsetNs
                    << " end_ns=" << hwSlots[k].endOffsetNs
                    << " period_ns=" << periodNs
                    << " destinations="
                    << FormatDestinationBitmap (hwSlots[k].dstRnicBitmapWords)
                    << std::endl;
        }

      Ptr<RdmaTransport> transport =
        rdmaDriver->GetTransport ();
      if (transport == 0)
        {
          std::cout << "[USERSPACE GATE INSTALL SKIP]"
                    << " node=" << nodeId
                    << " rnic_port=" << rnicPortId
                    << " reason=no_userspace_transport"
                    << std::endl;
          continue;
        }

      transport->InstallGateTable (
          rnicPortId,
          0,
          periodNs,
          hwSlots);

      std::cout << "[USERSPACE GATE TABLE INSTALLED]"
                << " node=" << nodeId
                << " rnic_port=" << rnicPortId
                << " periodNs=" << periodNs
                << " slots=" << hwSlots.size ()
                << std::endl;
      installedCount++;
      maxSlots = std::max (maxSlots,
                           static_cast<uint32_t> (hwSlots.size ()));
      summaryPeriodNs = periodNs;
    }

  if (installedCount > 0)
    {
      std::cout << "[USERSPACE GATE INSTALLED SUMMARY]"
                << " endpoint_tables=" << installedCount
                << " periodNs=" << summaryPeriodNs
                << " maxSlotsPerEndpoint=" << maxSlots
                << std::endl;
    }
}



void
TdmController::InstallSwitchTimeFlowTables (uint32_t mode)
{
  NS_ASSERT_MSG (mode <= 2, "Invalid switch time-flow mode");

  if (m_timeFlowSwitchIds.empty ())
    {
      return;
    }

  if (mode == TimeFlowTable::ROUTE_AND_GATE)
    {
      NS_ABORT_MSG ("SWITCH_TIME_FLOW_MODE=2 is reserved but not implemented in this version");
    }

  uint32_t totalEntries = 0;
  uint32_t calendarPorts = 0;

  for (std::set<uint32_t>::const_iterator switchIt = m_timeFlowSwitchIds.begin ();
       switchIt != m_timeFlowSwitchIds.end (); ++switchIt)
    {
      uint32_t switchId = *switchIt;
      Ptr<SwitchNode> sw = DynamicCast<SwitchNode> (m_nodes.Get (switchId));
      NS_ASSERT_MSG (sw != 0, "Time-flow node is not a SwitchNode");
      sw->SetTimeFlowCapable (true);
      sw->SetTimeFlowMode (mode);
      sw->GetTimeFlowTable ().Clear ();

      if (mode == TimeFlowTable::DISABLED)
        {
          continue;
        }

      const std::unordered_map<uint32_t, std::vector<int> > &routes =
        sw->GetRoutingTable ();
      for (std::unordered_map<uint32_t, std::vector<int> >::const_iterator routeIt =
             routes.begin (); routeIt != routes.end (); ++routeIt)
        {
          uint32_t dstIp = routeIt->first;
          uint32_t dstNodeId = (dstIp >> 8) & 0xffff;

          for (uint32_t nextIndex = 0; nextIndex < routeIt->second.size (); ++nextIndex)
            {
              int outIfSigned = routeIt->second[nextIndex];
              if (outIfSigned < 0)
                {
                  continue;
                }
              uint32_t outIf = static_cast<uint32_t> (outIfSigned);

              uint32_t switchLogicalPort = 0;
              PortBinding switchBinding;
              if (!FindBindingByIfIndex (switchId,
                                         outIf,
                                         switchLogicalPort,
                                         switchBinding))
                {
                  continue;
                }

              uint32_t ocsId = switchBinding.peerNodeId;
              if (!IsOcsNode (ocsId))
                {
                  continue;
                }

              std::map<uint32_t, OcsScheduleConfig>::const_iterator cfgIt =
                m_ocsScheduleConfigs.find (ocsId);
              NS_ASSERT_MSG (cfgIt != m_ocsScheduleConfigs.end (),
                             "Time-flow EPS port connects to an OCS without a schedule");
              const OcsScheduleConfig &cfg = cfgIt->second;

              Ptr<QbbNetDevice> dev =
                DynamicCast<QbbNetDevice> (sw->GetDevice (outIf));
              NS_ASSERT_MSG (dev != 0, "Time-flow egress is not a QbbNetDevice");
              bool newlyConfigured = !dev->IsCalendarEnabled ();
              dev->ConfigureCalendar (MicroSeconds (cfg.epochStartUs),
                                      MicroSeconds (cfg.sliceDurationUs),
                                      MicroSeconds (cfg.switchingTimeUs),
                                      cfg.numSlices,
                                      NanoSeconds (switchBinding.linkDelayNs));
              if (newlyConfigured)
                {
                  calendarPorts++;
                }

              // The switch-side binding points to the OCS ingress logical port.
              uint32_t inputOcsLogicalPort = switchBinding.peerLogicalPort;

              for (uint32_t arrivalSlot = 0;
                   arrivalSlot < cfg.numSlices;
                   ++arrivalSlot)
                {
                  bool found = false;
                  uint32_t sendSlot = 0;

                  for (uint32_t wait = 0; wait < cfg.numSlices; ++wait)
                    {
                      uint32_t candidateSlot = (arrivalSlot + wait) % cfg.numSlices;
                      uint32_t outputOcsLogicalPort = 0;
                      if (!FindScheduledPeerLogicalPort (ocsId,
                                                         inputOcsLogicalPort,
                                                         candidateSlot,
                                                         outputOcsLogicalPort))
                        {
                          continue;
                        }

                      std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator
                        ocsBindingNode = m_portBindings.find (ocsId);
                      NS_ASSERT_MSG (ocsBindingNode != m_portBindings.end (),
                                     "OCS port bindings are missing");
                      std::map<uint32_t, PortBinding>::const_iterator remotePort =
                        ocsBindingNode->second.find (outputOcsLogicalPort);
                      NS_ASSERT_MSG (remotePort != ocsBindingNode->second.end (),
                                     "Scheduled OCS output logical port is not bound");

                      uint32_t remotePeerNode = remotePort->second.peerNodeId;
                      if (IsReachableWithoutOcs (remotePeerNode,
                                                dstNodeId,
                                                ocsId))
                        {
                          found = true;
                          sendSlot = candidateSlot;
                          break;
                        }
                    }

                  if (found)
                    {
                      sw->GetTimeFlowTable ().AddGateEntry (dstIp,
                                                           outIf,
                                                           arrivalSlot,
                                                           sendSlot);
                      totalEntries++;
                    }
                }
            }
        }
    }

  std::cout << "[SWITCH TIME FLOW INSTALLED]"
            << " mode=" << mode
            << " capable_eps=" << m_timeFlowSwitchIds.size ()
            << " calendar_ports=" << calendarPorts
            << " entries=" << totalEntries
            << std::endl;
}


void
TdmController::ConfigureOcsSchedule (uint32_t ocsId,
                                     uint64_t epochUs,
                                     uint64_t sliceUs,
                                     uint64_t switchUs,
                                     uint32_t numSlices)
{
  NS_ASSERT_MSG (IsOcsNode (ocsId), "Schedule references non-OCS node " << ocsId);
  NS_ASSERT_MSG (ocsId < m_nodes.GetN (), "OCS node id out of range");
  NS_ASSERT_MSG (sliceUs > 0, "SLICE_US must be positive");
  NS_ASSERT_MSG (switchUs < sliceUs, "SWITCH_US must be smaller than SLICE_US");
  NS_ASSERT_MSG (numSlices > 0, "numSlices must be positive");
  NS_ASSERT_MSG (m_configuredOcs.find (ocsId) == m_configuredOcs.end (),
                 "OCS is configured more than once: " << ocsId);

  Ptr<OcsNode> ocs = DynamicCast<OcsNode> (m_nodes.Get (ocsId));
  NS_ASSERT_MSG (ocs != 0, "Node is not an OcsNode: " << ocsId);
  ocs->ConfigureSchedule (MicroSeconds (epochUs),
                          MicroSeconds (sliceUs),
                          numSlices,
                          MicroSeconds (switchUs));
  ocs->ClearSchedule ();

  OcsScheduleConfig cfg;
  cfg.epochStartUs = static_cast<uint32_t> (epochUs);
  cfg.sliceDurationUs = static_cast<uint32_t> (sliceUs);
  cfg.switchingTimeUs = static_cast<uint32_t> (switchUs);
  cfg.numSlices = numSlices;
  m_ocsScheduleConfigs[ocsId] = cfg;
  m_configuredOcs.insert (ocsId);
}

void
TdmController::InstallPair (uint32_t ocsId,
                            uint32_t slice,
                            uint32_t logicalPortA,
                            uint32_t logicalPortB)
{
  NS_ASSERT_MSG (logicalPortA != logicalPortB,
                 "OCS schedule cannot connect a port to itself");
  uint32_t actualIfA = ResolveLogicalPortToIf (ocsId, logicalPortA);
  uint32_t actualIfB = ResolveLogicalPortToIf (ocsId, logicalPortB);
  Ptr<OcsNode> ocs = DynamicCast<OcsNode> (m_nodes.Get (ocsId));
  NS_ASSERT_MSG (ocs != 0, "Node is not an OcsNode: " << ocsId);
  ocs->AddBidirectionalScheduleEntry (actualIfA, actualIfB, slice);

  OcsScheduleEntry entry;
  entry.ocsId = ocsId;
  entry.slice = slice;
  entry.logicalPortA = logicalPortA;
  entry.logicalPortB = logicalPortB;
  entry.actualIfA = actualIfA;
  entry.actualIfB = actualIfB;
  m_ocsScheduleEntries.push_back (entry);
}

bool
TdmController::FindBindingByIfIndex (uint32_t nodeId,
                                     uint32_t ifIndex,
                                     uint32_t &logicalPort,
                                     PortBinding &binding) const
{
  std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
    m_portBindings.find (nodeId);
  if (nodeIt == m_portBindings.end ())
    {
      return false;
    }

  for (std::map<uint32_t, PortBinding>::const_iterator it = nodeIt->second.begin ();
       it != nodeIt->second.end (); ++it)
    {
      if (it->second.ifIndex == ifIndex)
        {
          logicalPort = it->first;
          binding = it->second;
          return true;
        }
    }
  return false;
}

bool
TdmController::FindScheduledPeerLogicalPort (uint32_t ocsId,
                                             uint32_t inputLogicalPort,
                                             uint32_t slice,
                                             uint32_t &outputLogicalPort) const
{
  for (uint32_t i = 0; i < m_ocsScheduleEntries.size (); ++i)
    {
      const OcsScheduleEntry &entry = m_ocsScheduleEntries[i];
      if (entry.ocsId != ocsId || entry.slice != slice)
        {
          continue;
        }
      if (entry.logicalPortA == inputLogicalPort)
        {
          outputLogicalPort = entry.logicalPortB;
          return true;
        }
      if (entry.logicalPortB == inputLogicalPort)
        {
          outputLogicalPort = entry.logicalPortA;
          return true;
        }
    }
  return false;
}

bool
TdmController::IsReachableWithoutOcs (uint32_t startNodeId,
                                      uint32_t dstNodeId,
                                      uint32_t excludedOcsId) const
{
  if (startNodeId == dstNodeId)
    {
      return true;
    }
  if (startNodeId >= m_nodes.GetN () || dstNodeId >= m_nodes.GetN ())
    {
      return false;
    }
  if (IsOcsNode (startNodeId) || IsOcsNode (dstNodeId))
    {
      return false;
    }

  std::queue<uint32_t> pending;
  std::set<uint32_t> visited;
  pending.push (startNodeId);
  visited.insert (startNodeId);

  while (!pending.empty ())
    {
      uint32_t current = pending.front ();
      pending.pop ();

      std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
        m_portBindings.find (current);
      if (nodeIt == m_portBindings.end ())
        {
          continue;
        }

      for (std::map<uint32_t, PortBinding>::const_iterator portIt =
             nodeIt->second.begin (); portIt != nodeIt->second.end (); ++portIt)
        {
          uint32_t peer = portIt->second.peerNodeId;
          if (current == excludedOcsId || peer == excludedOcsId ||
              IsOcsNode (current) || IsOcsNode (peer))
            {
              continue;
            }
          if (peer == dstNodeId)
            {
              return true;
            }
          if (visited.insert (peer).second)
            {
              pending.push (peer);
            }
        }
    }

  return false;
}

void
TdmController::LoadStaticMap (const std::string &filename)
{
  LoadAndInstallOcsMap (filename);
}

void
TdmController::DumpPortBindings (std::ostream &os) const
{
  os << "# node_id logical_port if_index peer_node peer_logical_port delay_ns bandwidth_bps\n";
  for (std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator n =
         m_portBindings.begin (); n != m_portBindings.end (); ++n)
    {
      for (std::map<uint32_t, PortBinding>::const_iterator p =
             n->second.begin (); p != n->second.end (); ++p)
        {
          os << n->first << " " << p->first << " " << p->second.ifIndex << " "
             << p->second.peerNodeId << " " << p->second.peerLogicalPort << " "
             << p->second.linkDelayNs << " " << p->second.linkBandwidthBps << "\n";
        }
    }
}

void
TdmController::DumpExpandedSchedule (std::ostream &os) const
{
  os << "# Expanded OCS schedule (logical ports)\n";
  os << "# CONFIG ocs_id epoch_start_us slice_duration_us switching_time_us num_slices\n";
  for (std::map<uint32_t, OcsScheduleConfig>::const_iterator it =
         m_ocsScheduleConfigs.begin (); it != m_ocsScheduleConfigs.end (); ++it)
    {
      os << "CONFIG " << it->first << " "
         << it->second.epochStartUs << " "
         << it->second.sliceDurationUs << " "
         << it->second.switchingTimeUs << " "
         << it->second.numSlices << "\n";
    }
  os << "\n# ocs_id slice port_a port_b\n";
  for (uint32_t i = 0; i < m_ocsScheduleEntries.size (); ++i)
    {
      const OcsScheduleEntry &e = m_ocsScheduleEntries[i];
      os << e.ocsId << " " << e.slice << " "
         << e.logicalPortA << " " << e.logicalPortB << "\n";
    }
}

std::vector<std::pair<uint32_t, uint32_t> >
TdmController::GenerateRoundRobinPairsForSlice(
    const std::vector<uint32_t>& inputPorts,
    uint32_t slice) const
{
  if (inputPorts.size() < 2)
    {
      NS_ABORT_MSG("ROUND_ROBIN requires at least two ports");
    }

  std::vector<int64_t> ports;
  for (uint32_t i = 0; i < inputPorts.size(); ++i)
    {
      ports.push_back(static_cast<int64_t>(inputPorts[i]));
    }

  const int64_t dummy = -1;
  if (ports.size() % 2 != 0)
    {
      ports.push_back(dummy);
    }

  const uint32_t n = ports.size();
  const uint32_t rounds = n - 1;
  const uint32_t r = slice % rounds;

  std::vector<std::pair<uint32_t, uint32_t> > pairs;

  // Generate the same ordering as the original hand-written 4-port schedule:
  //   slice 0: 0-1, 2-3
  //   slice 1: 0-2, 1-3
  //   slice 2: 0-3, 1-2
  // For odd port counts, a dummy port is appended internally.  Any pair
  // involving the dummy port is skipped, leaving one real port idle.
  const auto addPair = [&pairs, dummy](int64_t a, int64_t b) {
    if (a == dummy || b == dummy)
      {
        return;
      }

    uint32_t pa = static_cast<uint32_t>(a);
    uint32_t pb = static_cast<uint32_t>(b);
    if (pa > pb)
      {
        std::swap(pa, pb);
      }
    pairs.push_back(std::make_pair(pa, pb));
  };

  addPair(ports[0], ports[1 + r]);

  for (uint32_t k = 1; k < n / 2; ++k)
    {
      const uint32_t aIndex = 1 + ((r + k) % (n - 1));
      const uint32_t bIndex = 1 + ((r + n - 1 - k) % (n - 1));
      addPair(ports[aIndex], ports[bIndex]);
    }

  std::sort(pairs.begin(), pairs.end());
  return pairs;
}

void
TdmController::FlushScheduleBlock(const ScheduleBlock& block)
{
  if (!block.active)
    {
      return;
    }

  if (block.ocsIds.empty())
    {
      NS_ABORT_MSG("SCHEDULE block has no OCS list");
    }

  if (block.hasRoundRobin && !block.slices.empty())
    {
      NS_ABORT_MSG("A SCHEDULE block cannot mix ROUND_ROBIN and explicit SLICE lines");
    }

  uint32_t numSlices = 0;

  if (block.hasRoundRobin)
    {
      if (block.rrPorts.size() < 2)
        {
          NS_ABORT_MSG("ROUND_ROBIN requires at least two ports");
        }

      const uint32_t effectivePortCount =
        (block.rrPorts.size() % 2 == 0) ? block.rrPorts.size() : block.rrPorts.size() + 1;
      numSlices = effectivePortCount - 1;

      for (uint32_t oi = 0; oi < block.ocsIds.size(); ++oi)
        {
          ConfigureOcsSchedule(block.ocsIds[oi],
                               block.epochUs,
                               block.sliceUs,
                               block.switchUs,
                               numSlices);

          for (uint32_t s = 0; s < numSlices; ++s)
            {
              std::vector<std::pair<uint32_t, uint32_t> > pairs =
                GenerateRoundRobinPairsForSlice(block.rrPorts, s);
              for (uint32_t i = 0; i < pairs.size(); ++i)
                {
                  InstallPair(block.ocsIds[oi], s, pairs[i].first, pairs[i].second);
                }
            }
        }
      return;
    }

  if (block.slices.empty())
    {
      NS_ABORT_MSG("SCHEDULE block has neither ROUND_ROBIN nor SLICE definitions");
    }

  uint32_t maxSlice = 0;
  std::set<uint32_t> seenSlices;
  for (uint32_t i = 0; i < block.slices.size(); ++i)
    {
      maxSlice = std::max(maxSlice, block.slices[i].slice);
      seenSlices.insert(block.slices[i].slice);

      std::set<uint32_t> usedPorts;
      for (uint32_t j = 0; j < block.slices[i].pairs.size(); ++j)
        {
          const uint32_t a = block.slices[i].pairs[j].first;
          const uint32_t b = block.slices[i].pairs[j].second;
          if (a == b)
            {
              NS_ABORT_MSG("Explicit SLICE pair connects a port to itself");
            }
          if (usedPorts.find(a) != usedPorts.end() ||
              usedPorts.find(b) != usedPorts.end())
            {
              NS_ABORT_MSG("A logical port appears more than once in SLICE "
                           << block.slices[i].slice);
            }
          usedPorts.insert(a);
          usedPorts.insert(b);
        }
    }

  numSlices = maxSlice + 1;
  for (uint32_t s = 0; s < numSlices; ++s)
    {
      if (seenSlices.find(s) == seenSlices.end())
        {
          NS_ABORT_MSG("Explicit SLICE definitions must be contiguous from 0; missing slice "
                       << s);
        }
    }

  for (uint32_t oi = 0; oi < block.ocsIds.size(); ++oi)
    {
      ConfigureOcsSchedule(block.ocsIds[oi],
                           block.epochUs,
                           block.sliceUs,
                           block.switchUs,
                           numSlices);

      for (uint32_t i = 0; i < block.slices.size(); ++i)
        {
          for (uint32_t j = 0; j < block.slices[i].pairs.size(); ++j)
            {
              InstallPair(block.ocsIds[oi],
                          block.slices[i].slice,
                          block.slices[i].pairs[j].first,
                          block.slices[i].pairs[j].second);
            }
        }
    }
}

void
TdmController::LoadCompactSchedule(const std::string& filename)
{
  m_ocsScheduleConfigs.clear ();
  m_ocsScheduleEntries.clear ();
  m_configuredOcs.clear ();
  std::ifstream fin(filename.c_str());
  if (!fin.is_open())
    {
      NS_ABORT_MSG("Cannot open OCS schedule file: " << filename);
    }

  ScheduleBlock current;
  std::string rawLine;
  uint32_t lineNo = 0;

  while (std::getline(fin, rawLine))
    {
      ++lineNo;

      std::string line = rawLine;
      std::string::size_type hash = line.find('#');
      if (hash != std::string::npos)
        {
          line = line.substr(0, hash);
        }
      line = Trim(line);
      if (line.empty())
        {
          continue;
        }

      std::vector<std::string> tokens = SplitWs(line);
      if (tokens.empty())
        {
          continue;
        }

      if (tokens[0] == "SCHEDULE")
        {
          FlushScheduleBlock(current);
          current = ScheduleBlock();
          current.active = true;

          std::map<std::string, std::string> kv = ParseKeyValues(tokens, 1);
          if (kv.find("OCS") == kv.end() ||
              kv.find("EPOCH_US") == kv.end() ||
              kv.find("SLICE_US") == kv.end() ||
              kv.find("SWITCH_US") == kv.end())
            {
              NS_ABORT_MSG("Invalid SCHEDULE line " << lineNo
                           << ": requires OCS, EPOCH_US, SLICE_US, SWITCH_US");
            }

          current.ocsIds = ParseUintList(kv["OCS"]);
          current.epochUs = std::strtoull(kv["EPOCH_US"].c_str(), 0, 10);
          current.sliceUs = std::strtoull(kv["SLICE_US"].c_str(), 0, 10);
          current.switchUs = std::strtoull(kv["SWITCH_US"].c_str(), 0, 10);
          continue;
        }

      if (!current.active)
        {
          NS_ABORT_MSG("Schedule statement before SCHEDULE block at line " << lineNo);
        }

      if (tokens[0] == "ROUND_ROBIN")
        {
          std::map<std::string, std::string> kv = ParseKeyValues(tokens, 1);
          if (kv.find("PORTS") == kv.end())
            {
              NS_ABORT_MSG("ROUND_ROBIN line " << lineNo << " requires PORTS");
            }
          current.hasRoundRobin = true;
          current.rrPorts = ParseUintList(kv["PORTS"]);
          continue;
        }

      if (tokens[0] == "SLICE")
        {
          if (tokens.size() < 3)
            {
              NS_ABORT_MSG("Invalid SLICE line " << lineNo
                           << ": expected SLICE <slice> PAIRS=...");
            }

          ScheduleBlock::SliceDef sd;
          sd.slice = static_cast<uint32_t>(std::strtoul(tokens[1].c_str(), 0, 10));

          std::map<std::string, std::string> kv = ParseKeyValues(tokens, 2);
          if (kv.find("PAIRS") == kv.end())
            {
              NS_ABORT_MSG("SLICE line " << lineNo << " requires PAIRS");
            }
          sd.pairs = ParsePairList(kv["PAIRS"]);
          current.slices.push_back(sd);
          continue;
        }

      NS_ABORT_MSG("Unknown OCS schedule statement at line " << lineNo
                   << ": " << tokens[0]);
    }

  FlushScheduleBlock(current);

  NS_LOG_UNCOND("[OCS SCHEDULE INSTALLED] file=" << filename
                << " configs=" << m_ocsScheduleConfigs.size()
                << " entries=" << m_ocsScheduleEntries.size());
}

std::vector<uint32_t>
TdmController::ParseUintList(const std::string& value) const
{
  std::vector<uint32_t> out;
  std::stringstream ss(value);
  std::string item;

  while (std::getline(ss, item, ','))
    {
      item = Trim(item);
      if (item.empty())
        {
          continue;
        }

      std::string::size_type dash = item.find('-');
      if (dash == std::string::npos)
        {
          out.push_back(static_cast<uint32_t>(std::strtoul(item.c_str(), 0, 10)));
        }
      else
        {
          uint32_t a = static_cast<uint32_t>(
              std::strtoul(item.substr(0, dash).c_str(), 0, 10));
          uint32_t b = static_cast<uint32_t>(
              std::strtoul(item.substr(dash + 1).c_str(), 0, 10));
          if (b < a)
            {
              NS_ABORT_MSG("Invalid decreasing range: " << item);
            }
          for (uint32_t v = a; v <= b; ++v)
            {
              out.push_back(v);
            }
        }
    }

  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());

  if (out.empty())
    {
      NS_ABORT_MSG("Empty uint list: " << value);
    }
  return out;
}

std::vector<std::pair<uint32_t, uint32_t> >
TdmController::ParsePairList(const std::string& value) const
{
  std::vector<std::pair<uint32_t, uint32_t> > pairs;
  std::stringstream ss(value);
  std::string item;

  while (std::getline(ss, item, ','))
    {
      item = Trim(item);
      if (item.empty())
        {
          continue;
        }

      std::string::size_type dash = item.find('-');
      if (dash == std::string::npos)
        {
          NS_ABORT_MSG("Invalid pair expression: " << item);
        }

      uint32_t a = static_cast<uint32_t>(
          std::strtoul(item.substr(0, dash).c_str(), 0, 10));
      uint32_t b = static_cast<uint32_t>(
          std::strtoul(item.substr(dash + 1).c_str(), 0, 10));
      pairs.push_back(std::make_pair(a, b));
    }

  if (pairs.empty())
    {
      NS_ABORT_MSG("Empty pair list: " << value);
    }
  return pairs;
}

std::map<std::string, std::string>
TdmController::ParseKeyValues(const std::vector<std::string>& tokens,
                                       uint32_t firstIndex) const
{
  std::map<std::string, std::string> kv;

  for (uint32_t i = firstIndex; i < tokens.size(); ++i)
    {
      std::string::size_type eq = tokens[i].find('=');
      if (eq == std::string::npos)
        {
          NS_ABORT_MSG("Expected KEY=VALUE token, got: " << tokens[i]);
        }

      std::string key = tokens[i].substr(0, eq);
      std::string value = tokens[i].substr(eq + 1);
      if (key.empty() || value.empty())
        {
          NS_ABORT_MSG("Invalid KEY=VALUE token: " << tokens[i]);
        }

      kv[key] = value;
    }

  return kv;
}

std::string
TdmController::Trim(const std::string& s)
{
  const char* ws = " \t\r\n";
  std::string::size_type b = s.find_first_not_of(ws);
  if (b == std::string::npos)
    {
      return std::string();
    }
  std::string::size_type e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

std::vector<std::string>
TdmController::SplitWs(const std::string& s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string tok;
  while (ss >> tok)
    {
      out.push_back(tok);
    }
  return out;
}

bool
TdmController::StartsWith(const std::string& s,
                                   const std::string& prefix)
{
  return s.size() >= prefix.size() &&
         std::equal(prefix.begin(), prefix.end(), s.begin());
}


} // namespace ns3
