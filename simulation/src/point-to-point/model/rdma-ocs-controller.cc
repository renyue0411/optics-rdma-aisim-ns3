/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/rdma-ocs-controller.h"
#include "ns3/ocs-node.h"
#include "ns3/log.h"
#include "ns3/assert.h"

#include <algorithm>
#include <cmath>
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

static uint64_t
CeilDivUint64 (uint64_t a, uint64_t b)
{
  NS_ASSERT_MSG (b > 0, "division by zero");
  return (a + b - 1) / b;
}

static uint64_t
CalcSerializationNs (uint32_t packetBytes, uint64_t bandwidthBps)
{
  if (bandwidthBps == 0 || packetBytes == 0)
    {
      return 0;
    }

  return CeilDivUint64 (static_cast<uint64_t> (packetBytes) * 8ULL * 1000000000ULL,
                        bandwidthBps);
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

NS_LOG_COMPONENT_DEFINE ("RdmaOcsController");
NS_OBJECT_ENSURE_REGISTERED (RdmaOcsController);

TypeId
RdmaOcsController::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RdmaOcsController")
    .SetParent<Object> ()
    .SetGroupName ("PointToPoint")
    .AddConstructor<RdmaOcsController> ();
  return tid;
}

RdmaOcsController::RdmaOcsController ()
  : m_rnicGatePacketBytes (1200),
    m_rnicGateAckBytes (92),
    m_rnicGateExtraMarginNs (200000),
    m_rnicGateBurstBytes (65536)
{
}

RdmaOcsController::~RdmaOcsController ()
{
}

void
RdmaOcsController::SetNodeContainer (NodeContainer nodes)
{
  m_nodes = nodes;
}

void
RdmaOcsController::SetRnicGatePacketBytes (uint32_t packetBytes)
{
  m_rnicGatePacketBytes = packetBytes;
}

void
RdmaOcsController::SetRnicGateAckBytes (uint32_t packetBytes)
{
  m_rnicGateAckBytes = packetBytes;
}

void
RdmaOcsController::SetRnicGateExtraMarginNs (uint64_t marginNs)
{
  m_rnicGateExtraMarginNs = marginNs;
}

void
RdmaOcsController::SetRnicGateBurstBytes (uint64_t burstBytes)
{
  m_rnicGateBurstBytes = burstBytes;
}

void
RdmaOcsController::AddOcsNode (uint32_t nodeId)
{
  NS_ASSERT_MSG (nodeId < m_nodes.GetN (), "OCS node id out of range");

  Ptr<OcsNode> ocs = DynamicCast<OcsNode> (m_nodes.Get (nodeId));
  NS_ASSERT_MSG (ocs != 0, "AddOcsNode points to a non-OCS node");

  m_ocsNodeIds.insert (nodeId);
}

void
RdmaOcsController::AddPortBinding (uint32_t nodeId,
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
RdmaOcsController::IsOcsNode (uint32_t nodeId) const
{
  return m_ocsNodeIds.find (nodeId) != m_ocsNodeIds.end ();
}

uint32_t
RdmaOcsController::ResolveLogicalPortToIf (uint32_t nodeId,
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
RdmaOcsController::LoadAndInstallOcsMap (const std::string &filename)
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
RdmaOcsController::LoadAndInstallOcsSchedule (const std::string &filename)
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
RdmaOcsController::GetDegree (uint32_t nodeId) const
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
RdmaOcsController::IsEndpointNode (uint32_t nodeId) const
{
  if (IsOcsNode (nodeId))
    {
      return false;
    }

  /*
   * First implementation rule:
   * a non-OCS node with exactly one fabric-facing port is treated as an RNIC endpoint.
   *
   * Current topology:
   *   hosts have degree 1
   *   switches/EPS have degree > 1
   */
  return GetDegree (nodeId) == 1;
}

uint32_t
RdmaOcsController::GetRnicGroupForNode (uint32_t nodeId) const
{
  std::map<uint32_t, uint32_t>::const_iterator it =
    m_nodeToRnicGroup.find (nodeId);

  NS_ASSERT_MSG (it != m_nodeToRnicGroup.end (),
                 "RNIC node has no assigned RNIC group");

  return it->second;
}

void
RdmaOcsController::BuildRnicGroups ()
{
  m_nodeToRnicGroup.clear ();
  m_rnicGroups.clear ();
  m_attachmentNodeToRnicGroup.clear ();

  uint32_t nextGroupId = 0;

  /*
   * Pass 1:
   * RNIC directly connected to OCS.
   * Each RNIC port becomes a group of size 1.
   */
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

      const std::map<uint32_t, PortBinding> &ports = nodeIt->second;
      NS_ASSERT_MSG (ports.size () == 1,
                     "Endpoint node should have exactly one port");

      const PortBinding &binding = ports.begin ()->second;

      if (!IsOcsNode (binding.peerNodeId))
        {
          continue;
        }

      RnicGroup group;
      group.groupId = nextGroupId;
      group.type = RNIC_DIRECT_OCS;
      group.attachmentNode = nodeId;
      group.rnicNodes.push_back (nodeId);

      m_rnicGroups[nextGroupId] = group;
      m_nodeToRnicGroup[nodeId] = nextGroupId;
      m_attachmentNodeToRnicGroup[nodeId] = nextGroupId;

      nextGroupId++;
    }

  /*
   * Pass 2:
   * RNICs connected to the same EPS/ToR are aggregated into one group.
   */
  std::map<uint32_t, std::vector<uint32_t> > epsToRnicNodes;

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

      if (m_nodeToRnicGroup.find (nodeId) != m_nodeToRnicGroup.end ())
        {
          continue;
        }

      const std::map<uint32_t, PortBinding> &ports = nodeIt->second;
      NS_ASSERT_MSG (ports.size () == 1,
                     "Endpoint node should have exactly one port");

      const PortBinding &binding = ports.begin ()->second;

      if (IsOcsNode (binding.peerNodeId))
        {
          continue;
        }

      epsToRnicNodes[binding.peerNodeId].push_back (nodeId);
    }

  for (std::map<uint32_t, std::vector<uint32_t> >::iterator it =
         epsToRnicNodes.begin ();
       it != epsToRnicNodes.end ();
       ++it)
    {
      uint32_t attachmentNode = it->first;

      RnicGroup group;
      group.groupId = nextGroupId;
      group.type = EPS_AGGREGATED;
      group.attachmentNode = attachmentNode;
      group.rnicNodes = it->second;

      m_rnicGroups[nextGroupId] = group;
      m_attachmentNodeToRnicGroup[attachmentNode] = nextGroupId;

      for (uint32_t i = 0; i < group.rnicNodes.size (); ++i)
        {
          m_nodeToRnicGroup[group.rnicNodes[i]] = nextGroupId;
        }

      nextGroupId++;
    }
}

uint32_t
RdmaOcsController::GetGroupForOcsLogicalPort (uint32_t ocsId,
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

  uint32_t peerNode = portIt->second.peerNodeId;

  /*
   * Case 1:
   * OCS port directly connects to an RNIC endpoint.
   */
  std::map<uint32_t, uint32_t>::const_iterator nodeGroupIt =
    m_nodeToRnicGroup.find (peerNode);

  if (nodeGroupIt != m_nodeToRnicGroup.end ())
    {
      return nodeGroupIt->second;
    }

  /*
   * Case 2:
   * OCS port connects to an EPS/ToR attachment node.
   */
  std::map<uint32_t, uint32_t>::const_iterator attachmentGroupIt =
    m_attachmentNodeToRnicGroup.find (peerNode);

  if (attachmentGroupIt != m_attachmentNodeToRnicGroup.end ())
    {
      return attachmentGroupIt->second;
    }

  NS_ASSERT_MSG (false,
                 "OCS logical port cannot be mapped to an RNIC group");

  return 0;
}

void
RdmaOcsController::CompileRnicReachabilityWindows ()
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
          uint32_t rnicNode = group.attachmentNode;

          std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
            m_portBindings.find (rnicNode);

          NS_ASSERT_MSG (nodeIt != m_portBindings.end (),
                         "RNIC_DIRECT_OCS group has no RNIC port binding");
          NS_ASSERT_MSG (nodeIt->second.size () == 1,
                         "RNIC_DIRECT_OCS group should have exactly one port");

          const PortBinding &binding = nodeIt->second.begin ()->second;
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

  auto GetEndpointSourceOffsetNs =
    [&] (const RnicGroup &group, uint32_t packetBytes) -> uint64_t
    {
      uint64_t maxOffsetNs = 0;

      for (uint32_t r = 0; r < group.rnicNodes.size (); ++r)
        {
          uint32_t rnicNode = group.rnicNodes[r];
          std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
            m_portBindings.find (rnicNode);

          NS_ASSERT_MSG (nodeIt != m_portBindings.end (),
                         "endpoint RNIC has no port binding");
          NS_ASSERT_MSG (nodeIt->second.size () == 1,
                         "endpoint RNIC should have one port binding");

          const PortBinding &binding = nodeIt->second.begin ()->second;
          uint64_t serNs = CalcSerializationNs (packetBytes,
                                                binding.linkBandwidthBps);
          uint64_t offsetNs = binding.linkDelayNs + serNs;

          if (offsetNs > maxOffsetNs)
            {
              maxOffsetNs = offsetNs;
            }
        }

      return maxOffsetNs;
    };

  auto GetEndpointDataDeliveryExtraNs =
    [&] (const RnicGroup &group) -> uint64_t
    {
      uint64_t maxExtraNs = 0;

      for (uint32_t r = 0; r < group.rnicNodes.size (); ++r)
        {
          uint32_t rnicNode = group.rnicNodes[r];
          std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
            m_portBindings.find (rnicNode);

          NS_ASSERT_MSG (nodeIt != m_portBindings.end (),
                         "destination RNIC has no port binding");
          NS_ASSERT_MSG (nodeIt->second.size () == 1,
                         "destination RNIC should have one port binding");

          const PortBinding &binding = nodeIt->second.begin ()->second;

          /*
           * A directly-attached OCS is transparent and schedules the packet
           * onto the RNIC-facing channel without a second OCS-side
           * serialization.  An EPS/ToR attachment, however, is a packet
           * switch and its host-facing egress serialization is part of the
           * time before the receiver can generate an RDMA ACK.
           */
          uint64_t extraNs = binding.linkDelayNs;
          if (group.type == EPS_AGGREGATED)
            {
              extraNs += CalcSerializationNs (m_rnicGatePacketBytes,
                                              binding.linkBandwidthBps);
            }

          if (extraNs > maxExtraNs)
            {
              maxExtraNs = extraNs;
            }
        }

      return maxExtraNs;
    };

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

  const uint64_t infinity = std::numeric_limits<uint64_t>::max () / 4;

  auto GetDirectedStaticEdgeWeightNs =
    [&] (uint32_t fromVertex,
         uint64_t delayNs,
         uint64_t bandwidthBps,
         uint32_t packetBytes) -> uint64_t
    {
      uint64_t weightNs = delayNs;

      /*
       * OCS vertices model transparent optical forwarding.  Therefore,
       * leaving an OCS port only contributes the outgoing channel
       * propagation delay.  Packet-switch vertices, such as EPS/ToR
       * switches, must additionally serialize the packet on their
       * selected egress link.  Source/destination RNIC serialization is
       * accounted for separately by the endpoint offset helpers.
       */
      if (vertexInfo[fromVertex].isPacketSwitch)
        {
          weightNs += CalcSerializationNs (packetBytes, bandwidthBps);
        }

      return weightNs;
    };

  for (uint32_t bi = 0; bi + 1 < boundaries.size (); ++bi)
    {
      uint64_t intervalStartNs = boundaries[bi];
      uint64_t intervalEndNs = boundaries[bi + 1];

      if (intervalEndNs <= intervalStartNs)
        {
          continue;
        }

      SimpleDsu dsu (nextVertex);
      std::vector<std::vector<std::pair<uint32_t, uint64_t> > > dataAdj (nextVertex);
      std::vector<std::vector<std::pair<uint32_t, uint64_t> > > ackAdj (nextVertex);

      for (uint32_t i = 0; i < staticEdges.size (); ++i)
        {
          const WeightedEdge &edge = staticEdges[i];
          dsu.Unite (edge.a, edge.b);

          uint64_t dataAB =
            GetDirectedStaticEdgeWeightNs (edge.a, edge.delayNs,
                                           edge.bandwidthBps,
                                           m_rnicGatePacketBytes);
          uint64_t dataBA =
            GetDirectedStaticEdgeWeightNs (edge.b, edge.delayNs,
                                           edge.bandwidthBps,
                                           m_rnicGatePacketBytes);
          uint64_t ackAB =
            GetDirectedStaticEdgeWeightNs (edge.a, edge.delayNs,
                                           edge.bandwidthBps,
                                           m_rnicGateAckBytes);
          uint64_t ackBA =
            GetDirectedStaticEdgeWeightNs (edge.b, edge.delayNs,
                                           edge.bandwidthBps,
                                           m_rnicGateAckBytes);

          dataAdj[edge.a].push_back (std::make_pair (edge.b, dataAB));
          dataAdj[edge.b].push_back (std::make_pair (edge.a, dataBA));
          ackAdj[edge.a].push_back (std::make_pair (edge.b, ackAB));
          ackAdj[edge.b].push_back (std::make_pair (edge.a, ackBA));
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
          dataAdj[itA->second].push_back (std::make_pair (itB->second, 0));
          dataAdj[itB->second].push_back (std::make_pair (itA->second, 0));
          ackAdj[itA->second].push_back (std::make_pair (itB->second, 0));
          ackAdj[itB->second].push_back (std::make_pair (itA->second, 0));
        }

      for (std::map<uint32_t, uint32_t>::const_iterator srcIt =
             groupToAttachmentVertex.begin ();
           srcIt != groupToAttachmentVertex.end ();
           ++srcIt)
        {
          uint32_t srcGroup = srcIt->first;
          uint32_t srcVertex = srcIt->second;

          std::map<uint32_t, RnicGroup>::const_iterator srcGroupIt =
            m_rnicGroups.find (srcGroup);
          NS_ASSERT_MSG (srcGroupIt != m_rnicGroups.end (),
                         "source group not found");
          const RnicGroup &srcGroupObj = srcGroupIt->second;

          uint64_t sourceInjectionOffsetNs =
            GetEndpointSourceOffsetNs (srcGroupObj, m_rnicGatePacketBytes);

          std::vector<uint64_t> dist (nextVertex, infinity);
          std::vector<uint32_t> prev (nextVertex, std::numeric_limits<uint32_t>::max ());
          typedef std::pair<uint64_t, uint32_t> QueueItem;
          std::priority_queue<QueueItem,
                              std::vector<QueueItem>,
                              std::greater<QueueItem> > pq;

          dist[srcVertex] = 0;
          pq.push (std::make_pair (0, srcVertex));

          while (!pq.empty ())
            {
              QueueItem item = pq.top ();
              pq.pop ();

              uint64_t d = item.first;
              uint32_t v = item.second;

              if (d != dist[v])
                {
                  continue;
                }

              for (uint32_t ei = 0; ei < dataAdj[v].size (); ++ei)
                {
                  uint32_t to = dataAdj[v][ei].first;
                  uint64_t w = dataAdj[v][ei].second;
                  if (dist[to] > d + w)
                    {
                      dist[to] = d + w;
                      prev[to] = v;
                      pq.push (std::make_pair (dist[to], to));
                    }
                }
            }

          for (std::map<uint32_t, uint32_t>::const_iterator dstIt =
                 groupToAttachmentVertex.begin ();
               dstIt != groupToAttachmentVertex.end ();
               ++dstIt)
            {
              uint32_t dstGroup = dstIt->first;
              uint32_t dstVertex = dstIt->second;

              std::map<uint32_t, RnicGroup>::const_iterator dstGroupObjIt =
                m_rnicGroups.find (dstGroup);
              NS_ASSERT_MSG (dstGroupObjIt != m_rnicGroups.end (),
                             "destination group not found");
              const RnicGroup &dstGroupObj = dstGroupObjIt->second;

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

              if (dist[dstVertex] == infinity)
                {
                  continue;
                }

              std::vector<uint32_t> pathVertices;
              bool pathComplete = false;
              uint32_t walk = dstVertex;
              while (walk != std::numeric_limits<uint32_t>::max ())
                {
                  pathVertices.push_back (walk);

                  if (walk == srcVertex)
                    {
                      pathComplete = true;
                      break;
                    }
                  walk = prev[walk];
                }

              if (!pathComplete)
                {
                  continue;
                }

              uint64_t maxDeadlineOffsetNs = 0;

              /*
               * Data-safe constraint: the data packet must reach each OCS on
               * the forward path before that OCS enters switching time.
               */
              for (uint32_t pv = 0; pv < pathVertices.size (); ++pv)
                {
                  uint32_t v = pathVertices[pv];
                  if (vertexInfo[v].isOcsPort)
                    {
                      uint64_t dataArrivalOffsetNs = sourceInjectionOffsetNs + dist[v];
                      if (dataArrivalOffsetNs > maxDeadlineOffsetNs)
                        {
                          maxDeadlineOffsetNs = dataArrivalOffsetNs;
                        }
                    }
                }

              /*
               * ACK-safe / completion-safe constraint: after the data packet
               * reaches the destination RNIC, the receiver-side ACK must also
               * reach each OCS on the reverse path before the circuit becomes
               * invalid.  The ACK itself is not separately gated here; instead
               * the sender-side data injection deadline is pulled earlier.
               *
               * The reverse path uses ACK-sized directed edge weights.  Packet
               * switches contribute egress serialization, while OCS ports remain
               * transparent and contribute only channel propagation.
               */
              uint64_t dstDataDeliveryExtraNs =
                GetEndpointDataDeliveryExtraNs (dstGroupObj);
              uint64_t dstAckSourceOffsetNs =
                GetEndpointSourceOffsetNs (dstGroupObj, m_rnicGateAckBytes);
              uint64_t dataForwardToDstNs =
                sourceInjectionOffsetNs + dist[dstVertex] + dstDataDeliveryExtraNs;

              std::vector<uint64_t> reverseDist (nextVertex, infinity);
              typedef std::pair<uint64_t, uint32_t> ReverseQueueItem;
              std::priority_queue<ReverseQueueItem,
                                  std::vector<ReverseQueueItem>,
                                  std::greater<ReverseQueueItem> > reversePq;

              reverseDist[dstVertex] = 0;
              reversePq.push (std::make_pair (0, dstVertex));

              while (!reversePq.empty ())
                {
                  ReverseQueueItem item = reversePq.top ();
                  reversePq.pop ();

                  uint64_t d = item.first;
                  uint32_t v = item.second;

                  if (d != reverseDist[v])
                    {
                      continue;
                    }

                  for (uint32_t ei = 0; ei < ackAdj[v].size (); ++ei)
                    {
                      uint32_t to = ackAdj[v][ei].first;
                      uint64_t w = ackAdj[v][ei].second;
                      if (reverseDist[to] > d + w)
                        {
                          reverseDist[to] = d + w;
                          reversePq.push (std::make_pair (reverseDist[to], to));
                        }
                    }
                }

              for (uint32_t pv = 0; pv < pathVertices.size (); ++pv)
                {
                  uint32_t v = pathVertices[pv];
                  if (vertexInfo[v].isOcsPort && reverseDist[v] != infinity)
                    {
                      uint64_t ackArrivalOffsetNs =
                        dataForwardToDstNs + dstAckSourceOffsetNs + reverseDist[v];
                      if (ackArrivalOffsetNs > maxDeadlineOffsetNs)
                        {
                          maxDeadlineOffsetNs = ackArrivalOffsetNs;
                        }
                    }
                }

              uint64_t latestInjectNs = intervalEndNs;

              if (maxDeadlineOffsetNs > 0)
                {
                  if (intervalEndNs <= maxDeadlineOffsetNs)
                    {
                      continue;
                    }

                  latestInjectNs = intervalEndNs - maxDeadlineOffsetNs;
                }

              if (latestInjectNs <= intervalStartNs)
                {
                  continue;
                }

              RnicReachabilityWindow window;
              window.srcGroup = srcGroup;
              window.dstGroup = dstGroup;
              window.ocsId = std::numeric_limits<uint32_t>::max ();
              window.slice = bi;
              window.startOffset = NanoSeconds (intervalStartNs);
              window.endOffset = NanoSeconds (latestInjectNs);
              window.period = NanoSeconds (commonPeriodNs);

              m_rnicReachabilityWindows.push_back (window);
            }
        }
    }
}

void
RdmaOcsController::DumpRnicGroups () const
{
  for (std::map<uint32_t, RnicGroup>::const_iterator it =
         m_rnicGroups.begin ();
       it != m_rnicGroups.end ();
       ++it)
    {
      const RnicGroup &group = it->second;

      std::cout << "[RNIC GROUP] id=" << group.groupId
                << " type=";

      if (group.type == RNIC_DIRECT_OCS)
        {
          std::cout << "RNIC_DIRECT_OCS";
        }
      else
        {
          std::cout << "EPS_AGGREGATED";
        }

      std::cout << " attachmentNode=" << group.attachmentNode
                << " members=";

      for (uint32_t i = 0; i < group.rnicNodes.size (); ++i)
        {
          if (i > 0)
            {
              std::cout << ",";
            }
          std::cout << group.rnicNodes[i];
        }

      std::cout << std::endl;
    }
}

void
RdmaOcsController::DumpRnicReachabilityWindows () const
{
  struct DumpSlot
  {
    uint64_t startOffsetNs;
    uint64_t endOffsetNs;
    uint64_t periodNs;
    std::vector<uint64_t> bitmapWords;
    std::set<uint32_t> dstRnics;
  };

  std::vector<uint32_t> allRnicNodes;
  for (std::map<uint32_t, uint32_t>::const_iterator it =
         m_nodeToRnicGroup.begin ();
       it != m_nodeToRnicGroup.end ();
       ++it)
    {
      allRnicNodes.push_back (it->first);
    }

  std::sort (allRnicNodes.begin (), allRnicNodes.end ());

  uint32_t bitmapWordCount = static_cast<uint32_t> ((m_nodes.GetN () + 63) / 64);
  if (bitmapWordCount == 0)
    {
      bitmapWordCount = 1;
    }

  std::map<uint32_t, std::vector<DumpSlot> > tables;

  for (uint32_t i = 0; i < m_rnicReachabilityWindows.size (); ++i)
    {
      const RnicReachabilityWindow &w = m_rnicReachabilityWindows[i];

      std::map<uint32_t, RnicGroup>::const_iterator srcGroupIt =
        m_rnicGroups.find (w.srcGroup);
      std::map<uint32_t, RnicGroup>::const_iterator dstGroupIt =
        m_rnicGroups.find (w.dstGroup);

      NS_ASSERT_MSG (srcGroupIt != m_rnicGroups.end (),
                     "RNIC window points to an unknown source group");
      NS_ASSERT_MSG (dstGroupIt != m_rnicGroups.end (),
                     "RNIC window points to an unknown destination group");

      const RnicGroup &srcGroup = srcGroupIt->second;
      const RnicGroup &dstGroup = dstGroupIt->second;

      uint64_t startOffsetNs = static_cast<uint64_t> (w.startOffset.GetNanoSeconds ());
      uint64_t endOffsetNs = static_cast<uint64_t> (w.endOffset.GetNanoSeconds ());
      uint64_t periodNs = static_cast<uint64_t> (w.period.GetNanoSeconds ());

      for (uint32_t s = 0; s < srcGroup.rnicNodes.size (); ++s)
        {
          uint32_t srcRnic = srcGroup.rnicNodes[s];
          std::vector<DumpSlot> &slots = tables[srcRnic];

          DumpSlot *slot = 0;
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
              DumpSlot newSlot;
              newSlot.startOffsetNs = startOffsetNs;
              newSlot.endOffsetNs = endOffsetNs;
              newSlot.periodNs = periodNs;
              newSlot.bitmapWords.assign (bitmapWordCount, 0);
              slots.push_back (newSlot);
              slot = &slots.back ();
            }

          for (uint32_t d = 0; d < dstGroup.rnicNodes.size (); ++d)
            {
              uint32_t dstRnic = dstGroup.rnicNodes[d];

              if (dstRnic == srcRnic)
                {
                  continue;
                }

              uint32_t wordIndex = dstRnic / 64;
              uint32_t bitIndex = dstRnic % 64;

              NS_ASSERT_MSG (wordIndex < slot->bitmapWords.size (),
                             "RNIC bitmap word index out of range");

              slot->bitmapWords[wordIndex] |= (1ULL << bitIndex);
              slot->dstRnics.insert (dstRnic);
            }
        }
    }

  /*
   * Normalize overlapping reachability windows into disjoint time ranges.
   * The gate installed in RdmaHw is keyed by time first and then bitmap, so
   * overlapping slots with different bitmaps can accidentally mask each
   * other depending on lookup order.  For each RNIC, split at every boundary
   * and OR all raw windows that cover the resulting atomic interval.
   */
  for (std::map<uint32_t, std::vector<DumpSlot> >::iterator it =
         tables.begin ();
       it != tables.end ();
       ++it)
    {
      std::vector<DumpSlot> rawSlots = it->second;
      std::vector<uint64_t> boundaries;

      for (uint32_t k = 0; k < rawSlots.size (); ++k)
        {
          if (rawSlots[k].endOffsetNs <= rawSlots[k].startOffsetNs)
            {
              continue;
            }
          boundaries.push_back (rawSlots[k].startOffsetNs);
          boundaries.push_back (rawSlots[k].endOffsetNs);
        }

      std::sort (boundaries.begin (), boundaries.end ());
      boundaries.erase (std::unique (boundaries.begin (), boundaries.end ()),
                        boundaries.end ());

      std::vector<DumpSlot> normalizedSlots;
      for (uint32_t b = 0; b + 1 < boundaries.size (); ++b)
        {
          uint64_t startOffsetNs = boundaries[b];
          uint64_t endOffsetNs = boundaries[b + 1];

          if (endOffsetNs <= startOffsetNs)
            {
              continue;
            }

          DumpSlot slot;
          slot.startOffsetNs = startOffsetNs;
          slot.endOffsetNs = endOffsetNs;
          slot.periodNs = 0;
          slot.bitmapWords.assign (bitmapWordCount, 0);

          for (uint32_t k = 0; k < rawSlots.size (); ++k)
            {
              const DumpSlot &raw = rawSlots[k];
              if (raw.startOffsetNs <= startOffsetNs &&
                  endOffsetNs <= raw.endOffsetNs)
                {
                  slot.periodNs = raw.periodNs;
                  for (uint32_t w = 0; w < slot.bitmapWords.size (); ++w)
                    {
                      slot.bitmapWords[w] |= raw.bitmapWords[w];
                    }
                  slot.dstRnics.insert (raw.dstRnics.begin (),
                                        raw.dstRnics.end ());
                }
            }

          bool hasDst = false;
          for (uint32_t w = 0; w < slot.bitmapWords.size (); ++w)
            {
              if (slot.bitmapWords[w] != 0)
                {
                  hasDst = true;
                  break;
                }
            }

          if (!hasDst)
            {
              continue;
            }

          if (!normalizedSlots.empty () &&
              normalizedSlots.back ().endOffsetNs == slot.startOffsetNs &&
              normalizedSlots.back ().periodNs == slot.periodNs &&
              normalizedSlots.back ().bitmapWords == slot.bitmapWords)
            {
              normalizedSlots.back ().endOffsetNs = slot.endOffsetNs;
              normalizedSlots.back ().dstRnics.insert (slot.dstRnics.begin (),
                                                       slot.dstRnics.end ());
            }
          else
            {
              normalizedSlots.push_back (slot);
            }
        }

      it->second = normalizedSlots;
    }

  for (uint32_t i = 0; i < allRnicNodes.size (); ++i)
    {
      uint32_t rnic = allRnicNodes[i];
      std::map<uint32_t, std::vector<DumpSlot> >::const_iterator tableIt =
        tables.find (rnic);

      uint64_t periodNs = 0;
      uint32_t entries = 0;

      if (tableIt != tables.end () && !tableIt->second.empty ())
        {
          periodNs = tableIt->second[0].periodNs;
          entries = tableIt->second.size ();
        }

      std::cout << "[INJECTION WINDOW TABLE BEGIN] rnic=" << rnic
                << " epochNs=0";

      if (periodNs > 0)
        {
          std::cout << " periodNs=" << periodNs;
        }
      else
        {
          std::cout << " periodNs=NA";
        }

      std::cout << " dstMode=DST_RNIC_BITMAP"
                << " entries=" << entries
                << std::endl;

      if (tableIt != tables.end ())
        {
          const std::vector<DumpSlot> &slots = tableIt->second;

          for (uint32_t k = 0; k < slots.size (); ++k)
            {
              const DumpSlot &slot = slots[k];

              std::cout << "  window=" << k
                        << " injectNs=[" << slot.startOffsetNs
                        << "," << slot.endOffsetNs << ")"
                        << " injectUs=[" << (static_cast<double> (slot.startOffsetNs) / 1000.0)
                        << "," << (static_cast<double> (slot.endOffsetNs) / 1000.0) << ")"
                        << " dstRnics={";

              uint32_t count = 0;
              for (std::set<uint32_t>::const_iterator dstIt =
                     slot.dstRnics.begin ();
                   dstIt != slot.dstRnics.end ();
                   ++dstIt)
                {
                  if (count > 0)
                    {
                      std::cout << ",";
                    }
                  std::cout << *dstIt;
                  count++;
                }

              std::cout << "} bitmapWords={";

              for (uint32_t w = 0; w < slot.bitmapWords.size (); ++w)
                {
                  if (w > 0)
                    {
                      std::cout << ",";
                    }

                  std::cout << "0x"
                            << std::hex
                            << std::setw (16)
                            << std::setfill ('0')
                            << slot.bitmapWords[w]
                            << std::dec
                            << std::setfill (' ');
                }

              std::cout << "}" << std::endl;
            }
        }

      std::cout << "[INJECTION WINDOW TABLE END] rnic=" << rnic
                << std::endl;
    }
}

void
RdmaOcsController::InstallRnicGateTablesToRdmaHw () const
{
  struct GateSlot
  {
    uint64_t startOffsetNs;
    uint64_t endOffsetNs;
    uint64_t periodNs;
    std::vector<uint64_t> bitmapWords;
  };

  std::vector<uint32_t> allRnicNodes;
  for (std::map<uint32_t, uint32_t>::const_iterator it =
         m_nodeToRnicGroup.begin ();
       it != m_nodeToRnicGroup.end ();
       ++it)
    {
      allRnicNodes.push_back (it->first);
    }
  std::sort (allRnicNodes.begin (), allRnicNodes.end ());

  uint32_t bitmapWordCount = static_cast<uint32_t> ((m_nodes.GetN () + 63) / 64);
  if (bitmapWordCount == 0)
    {
      bitmapWordCount = 1;
    }

  std::map<uint32_t, std::vector<GateSlot> > tables;
  std::map<uint32_t, uint64_t> tableMaxExtraGuardNs;
  std::map<uint32_t, uint64_t> tableMinBottleneckBps;

  for (uint32_t i = 0; i < m_rnicReachabilityWindows.size (); ++i)
    {
      const RnicReachabilityWindow &w = m_rnicReachabilityWindows[i];

      std::map<uint32_t, RnicGroup>::const_iterator srcGroupIt =
        m_rnicGroups.find (w.srcGroup);
      std::map<uint32_t, RnicGroup>::const_iterator dstGroupIt =
        m_rnicGroups.find (w.dstGroup);

      NS_ASSERT_MSG (srcGroupIt != m_rnicGroups.end (),
                     "RNIC window points to an unknown source group");
      NS_ASSERT_MSG (dstGroupIt != m_rnicGroups.end (),
                     "RNIC window points to an unknown destination group");

      const RnicGroup &srcGroup = srcGroupIt->second;
      const RnicGroup &dstGroup = dstGroupIt->second;

      uint64_t startOffsetNs = static_cast<uint64_t> (w.startOffset.GetNanoSeconds ());
      uint64_t endOffsetNs = static_cast<uint64_t> (w.endOffset.GetNanoSeconds ());
      uint64_t periodNs = static_cast<uint64_t> (w.period.GetNanoSeconds ());

      /*
       * Bandwidth-aware, deployment-friendly RNIC guard.
       *
       * This intentionally avoids simulator-only information such as exact
       * OCS drop causes or per-packet OCS arrival timestamps.  The guard uses
       * only statically known link rates and a bounded post-gate burst size,
       * plus a calibrated fixed margin for RNIC/device pipeline delay, event
       * scheduling jitter, and clock uncertainty.
       */
      uint64_t bottleneckBps = std::numeric_limits<uint64_t>::max ();
      bool haveBottleneck = false;

      for (uint32_t rn = 0; rn < srcGroup.rnicNodes.size (); ++rn)
        {
          uint32_t srcRnic = srcGroup.rnicNodes[rn];
          std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator
            rnicBindingIt = m_portBindings.find (srcRnic);
          if (rnicBindingIt != m_portBindings.end ())
            {
              for (std::map<uint32_t, PortBinding>::const_iterator pb =
                     rnicBindingIt->second.begin ();
                   pb != rnicBindingIt->second.end ();
                   ++pb)
                {
                  if (pb->second.peerNodeId == srcGroup.attachmentNode &&
                      pb->second.linkBandwidthBps > 0)
                    {
                      bottleneckBps = std::min (bottleneckBps,
                                                pb->second.linkBandwidthBps);
                      haveBottleneck = true;
                    }
                }
            }
        }

      std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator
        attachBindingIt = m_portBindings.find (srcGroup.attachmentNode);
      if (attachBindingIt != m_portBindings.end ())
        {
          for (std::map<uint32_t, PortBinding>::const_iterator pb =
                 attachBindingIt->second.begin ();
               pb != attachBindingIt->second.end ();
               ++pb)
            {
              if (pb->second.peerNodeId == w.ocsId &&
                  pb->second.linkBandwidthBps > 0)
                {
                  bottleneckBps = std::min (bottleneckBps,
                                            pb->second.linkBandwidthBps);
                  haveBottleneck = true;
                }
            }
        }

      uint64_t drainGuardNs = 0;
      if (haveBottleneck && bottleneckBps > 0 && m_rnicGateBurstBytes > 0)
        {
          long double numerator =
            static_cast<long double> (m_rnicGateBurstBytes) * 8.0L * 1000000000.0L;
          drainGuardNs = static_cast<uint64_t>
            (std::ceil (numerator / static_cast<long double> (bottleneckBps)));
        }

      uint64_t rnicHwExtraTailGuardNs = m_rnicGateExtraMarginNs + drainGuardNs;
      if (endOffsetNs <= startOffsetNs + rnicHwExtraTailGuardNs)
        {
          continue;
        }
      endOffsetNs -= rnicHwExtraTailGuardNs;

      for (uint32_t s = 0; s < srcGroup.rnicNodes.size (); ++s)
        {
          uint32_t srcRnic = srcGroup.rnicNodes[s];
          std::vector<GateSlot> &slots = tables[srcRnic];
          tableMaxExtraGuardNs[srcRnic] =
            std::max (tableMaxExtraGuardNs[srcRnic], rnicHwExtraTailGuardNs);
          if (haveBottleneck)
            {
              if (tableMinBottleneckBps.find (srcRnic) == tableMinBottleneckBps.end ())
                {
                  tableMinBottleneckBps[srcRnic] = bottleneckBps;
                }
              else
                {
                  tableMinBottleneckBps[srcRnic] =
                    std::min (tableMinBottleneckBps[srcRnic], bottleneckBps);
                }
            }

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

          for (uint32_t d = 0; d < dstGroup.rnicNodes.size (); ++d)
            {
              uint32_t dstRnic = dstGroup.rnicNodes[d];
              if (dstRnic == srcRnic)
                {
                  continue;
                }

              uint32_t wordIndex = dstRnic / 64;
              uint32_t bitIndex = dstRnic % 64;
              NS_ASSERT_MSG (wordIndex < slot->bitmapWords.size (),
                             "RNIC bitmap word index out of range");
              slot->bitmapWords[wordIndex] |= (1ULL << bitIndex);
            }
        }
    }

  /*
   * Install only disjoint slots.  This mirrors the dump normalization above
   * and makes the hardware gate lookup independent of slot iteration order.
   */
  for (std::map<uint32_t, std::vector<GateSlot> >::iterator it =
         tables.begin ();
       it != tables.end ();
       ++it)
    {
      std::vector<GateSlot> rawSlots = it->second;
      std::vector<uint64_t> boundaries;

      for (uint32_t k = 0; k < rawSlots.size (); ++k)
        {
          if (rawSlots[k].endOffsetNs <= rawSlots[k].startOffsetNs)
            {
              continue;
            }
          boundaries.push_back (rawSlots[k].startOffsetNs);
          boundaries.push_back (rawSlots[k].endOffsetNs);
        }

      std::sort (boundaries.begin (), boundaries.end ());
      boundaries.erase (std::unique (boundaries.begin (), boundaries.end ()),
                        boundaries.end ());

      std::vector<GateSlot> normalizedSlots;
      for (uint32_t b = 0; b + 1 < boundaries.size (); ++b)
        {
          uint64_t startOffsetNs = boundaries[b];
          uint64_t endOffsetNs = boundaries[b + 1];

          if (endOffsetNs <= startOffsetNs)
            {
              continue;
            }

          GateSlot slot;
          slot.startOffsetNs = startOffsetNs;
          slot.endOffsetNs = endOffsetNs;
          slot.periodNs = 0;
          slot.bitmapWords.assign (bitmapWordCount, 0);

          for (uint32_t k = 0; k < rawSlots.size (); ++k)
            {
              const GateSlot &raw = rawSlots[k];
              if (raw.startOffsetNs <= startOffsetNs &&
                  endOffsetNs <= raw.endOffsetNs)
                {
                  slot.periodNs = raw.periodNs;
                  for (uint32_t w = 0; w < slot.bitmapWords.size (); ++w)
                    {
                      slot.bitmapWords[w] |= raw.bitmapWords[w];
                    }
                }
            }

          bool hasDst = false;
          for (uint32_t w = 0; w < slot.bitmapWords.size (); ++w)
            {
              if (slot.bitmapWords[w] != 0)
                {
                  hasDst = true;
                  break;
                }
            }

          if (!hasDst)
            {
              continue;
            }

          if (!normalizedSlots.empty () &&
              normalizedSlots.back ().endOffsetNs == slot.startOffsetNs &&
              normalizedSlots.back ().periodNs == slot.periodNs &&
              normalizedSlots.back ().bitmapWords == slot.bitmapWords)
            {
              normalizedSlots.back ().endOffsetNs = slot.endOffsetNs;
            }
          else
            {
              normalizedSlots.push_back (slot);
            }
        }

      it->second = normalizedSlots;
    }

  for (uint32_t i = 0; i < allRnicNodes.size (); ++i)
    {
      uint32_t rnic = allRnicNodes[i];
      std::map<uint32_t, std::vector<GateSlot> >::const_iterator tableIt =
        tables.find (rnic);
      if (tableIt == tables.end () || tableIt->second.empty ())
        {
          continue;
        }

      Ptr<Node> node = m_nodes.Get (rnic);
      Ptr<RdmaDriver> rdmaDriver =
        node->GetObject<RdmaDriver> ();

      if (rdmaDriver == 0 || rdmaDriver->m_rdma == 0)
        {
          std::cout
            << "[RNIC GATE INSTALL SKIP]"
            << " rnic=" << rnic
            << " reason=no_rdma_hw"
            << std::endl;

          continue;
        }

      std::vector<RdmaHw::RnicGateSlotEntry> hwSlots;
      uint64_t periodNs = tableIt->second[0].periodNs;
      const std::vector<GateSlot> &slots = tableIt->second;
      for (uint32_t k = 0; k < slots.size (); ++k)
        {
          RdmaHw::RnicGateSlotEntry hwSlot;
          hwSlot.startOffsetNs = slots[k].startOffsetNs;
          hwSlot.endOffsetNs = slots[k].endOffsetNs;
          hwSlot.dstRnicBitmapWords = slots[k].bitmapWords;
          hwSlots.push_back (hwSlot);
        }

      uint64_t loggedExtraGuardNs = 0;
      std::map<uint32_t, uint64_t>::const_iterator guardIt =
        tableMaxExtraGuardNs.find (rnic);
      if (guardIt != tableMaxExtraGuardNs.end ())
        {
          loggedExtraGuardNs = guardIt->second;
        }

      uint64_t loggedBottleneckBps = 0;
      std::map<uint32_t, uint64_t>::const_iterator bwIt =
        tableMinBottleneckBps.find (rnic);
      if (bwIt != tableMinBottleneckBps.end ())
        {
          loggedBottleneckBps = bwIt->second;
        }

      uint64_t loggedDrainGuardNs = 0;
      if (loggedExtraGuardNs > m_rnicGateExtraMarginNs)
        {
          loggedDrainGuardNs = loggedExtraGuardNs - m_rnicGateExtraMarginNs;
        }

      std::cout
        << "[RNIC GATE HW EXTRA GUARD]"
        << " rnic=" << rnic
        << " extraTailGuardNs=" << loggedExtraGuardNs
        << " marginNs=" << m_rnicGateExtraMarginNs
        << " burstBytes=" << m_rnicGateBurstBytes
        << " drainGuardNs=" << loggedDrainGuardNs
        << " bottleneckBps=" << loggedBottleneckBps
        << std::endl;

      rdmaDriver->m_rdma->EnableRnicGate(
          rnic,
          0,
          periodNs,
          hwSlots);
    }
}


void
RdmaOcsController::InstallRnicGateTablesToUserspace () const
{
  struct GateSlot
  {
    uint64_t startOffsetNs;
    uint64_t endOffsetNs;
    uint64_t periodNs;
    std::vector<uint64_t> bitmapWords;
  };

  std::vector<uint32_t> allRnicNodes;
  for (std::map<uint32_t, uint32_t>::const_iterator it =
         m_nodeToRnicGroup.begin ();
       it != m_nodeToRnicGroup.end ();
       ++it)
    {
      allRnicNodes.push_back (it->first);
    }
  std::sort (allRnicNodes.begin (), allRnicNodes.end ());

  uint32_t bitmapWordCount = static_cast<uint32_t> ((m_nodes.GetN () + 63) / 64);
  if (bitmapWordCount == 0)
    {
      bitmapWordCount = 1;
    }

  std::map<uint32_t, std::vector<GateSlot> > tables;

  for (uint32_t i = 0; i < m_rnicReachabilityWindows.size (); ++i)
    {
      const RnicReachabilityWindow &w = m_rnicReachabilityWindows[i];

      std::map<uint32_t, RnicGroup>::const_iterator srcGroupIt =
        m_rnicGroups.find (w.srcGroup);
      std::map<uint32_t, RnicGroup>::const_iterator dstGroupIt =
        m_rnicGroups.find (w.dstGroup);

      NS_ASSERT_MSG (srcGroupIt != m_rnicGroups.end (),
                     "RNIC window points to an unknown source group");
      NS_ASSERT_MSG (dstGroupIt != m_rnicGroups.end (),
                     "RNIC window points to an unknown destination group");

      const RnicGroup &srcGroup = srcGroupIt->second;
      const RnicGroup &dstGroup = dstGroupIt->second;

      uint64_t startOffsetNs = static_cast<uint64_t> (w.startOffset.GetNanoSeconds ());
      uint64_t endOffsetNs = static_cast<uint64_t> (w.endOffset.GetNanoSeconds ());
      uint64_t periodNs = static_cast<uint64_t> (w.period.GetNanoSeconds ());

      for (uint32_t s = 0; s < srcGroup.rnicNodes.size (); ++s)
        {
          uint32_t srcRnic = srcGroup.rnicNodes[s];
          std::vector<GateSlot> &slots = tables[srcRnic];

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

          for (uint32_t d = 0; d < dstGroup.rnicNodes.size (); ++d)
            {
              uint32_t dstRnic = dstGroup.rnicNodes[d];
              if (dstRnic == srcRnic)
                {
                  continue;
                }

              uint32_t wordIndex = dstRnic / 64;
              uint32_t bitIndex = dstRnic % 64;
              NS_ASSERT_MSG (wordIndex < slot->bitmapWords.size (),
                             "RNIC bitmap word index out of range");
              slot->bitmapWords[wordIndex] |= (1ULL << bitIndex);
            }
        }
    }

  /*
   * Install only disjoint slots.  This mirrors the dump normalization above
   * and makes the hardware gate lookup independent of slot iteration order.
   */
  for (std::map<uint32_t, std::vector<GateSlot> >::iterator it =
         tables.begin ();
       it != tables.end ();
       ++it)
    {
      std::vector<GateSlot> rawSlots = it->second;
      std::vector<uint64_t> boundaries;

      for (uint32_t k = 0; k < rawSlots.size (); ++k)
        {
          if (rawSlots[k].endOffsetNs <= rawSlots[k].startOffsetNs)
            {
              continue;
            }
          boundaries.push_back (rawSlots[k].startOffsetNs);
          boundaries.push_back (rawSlots[k].endOffsetNs);
        }

      std::sort (boundaries.begin (), boundaries.end ());
      boundaries.erase (std::unique (boundaries.begin (), boundaries.end ()),
                        boundaries.end ());

      std::vector<GateSlot> normalizedSlots;
      for (uint32_t b = 0; b + 1 < boundaries.size (); ++b)
        {
          uint64_t startOffsetNs = boundaries[b];
          uint64_t endOffsetNs = boundaries[b + 1];

          if (endOffsetNs <= startOffsetNs)
            {
              continue;
            }

          GateSlot slot;
          slot.startOffsetNs = startOffsetNs;
          slot.endOffsetNs = endOffsetNs;
          slot.periodNs = 0;
          slot.bitmapWords.assign (bitmapWordCount, 0);

          for (uint32_t k = 0; k < rawSlots.size (); ++k)
            {
              const GateSlot &raw = rawSlots[k];
              if (raw.startOffsetNs <= startOffsetNs &&
                  endOffsetNs <= raw.endOffsetNs)
                {
                  slot.periodNs = raw.periodNs;
                  for (uint32_t w = 0; w < slot.bitmapWords.size (); ++w)
                    {
                      slot.bitmapWords[w] |= raw.bitmapWords[w];
                    }
                }
            }

          bool hasDst = false;
          for (uint32_t w = 0; w < slot.bitmapWords.size (); ++w)
            {
              if (slot.bitmapWords[w] != 0)
                {
                  hasDst = true;
                  break;
                }
            }

          if (!hasDst)
            {
              continue;
            }

          if (!normalizedSlots.empty () &&
              normalizedSlots.back ().endOffsetNs == slot.startOffsetNs &&
              normalizedSlots.back ().periodNs == slot.periodNs &&
              normalizedSlots.back ().bitmapWords == slot.bitmapWords)
            {
              normalizedSlots.back ().endOffsetNs = slot.endOffsetNs;
            }
          else
            {
              normalizedSlots.push_back (slot);
            }
        }

      it->second = normalizedSlots;
    }

  for (uint32_t i = 0; i < allRnicNodes.size (); ++i)
    {
      uint32_t rnic = allRnicNodes[i];
      std::map<uint32_t, std::vector<GateSlot> >::const_iterator tableIt =
        tables.find (rnic);
      if (tableIt == tables.end () || tableIt->second.empty ())
        {
          continue;
        }

      Ptr<Node> node = m_nodes.Get(rnic);

      Ptr<RdmaDriver> rdmaDriver =
          node->GetObject<RdmaDriver>();

      if (rdmaDriver == 0)
      {
          std::cout
              << "[USERSPACE GATE INSTALL SKIP]"
              << " rnic=" << rnic
              << " reason=no_rdma_driver"
              << std::endl;

          continue;
      }

      Ptr<RdmaUserspaceTransport> transport =
        rdmaDriver->GetUserspaceTransport ();

      if (transport == 0)
        {
          std::cout
            << "[USERSPACE GATE INSTALL SKIP]"
            << " rnic=" << rnic
            << " reason=no_userspace_transport"
            << std::endl;

          continue;
        }

      std::vector<RdmaHw::RnicGateSlotEntry> hwSlots;
      uint64_t periodNs = tableIt->second[0].periodNs;
      const std::vector<GateSlot> &slots = tableIt->second;
      for (uint32_t k = 0; k < slots.size (); ++k)
        {
          RdmaHw::RnicGateSlotEntry hwSlot;
          hwSlot.startOffsetNs = slots[k].startOffsetNs;
          hwSlot.endOffsetNs = slots[k].endOffsetNs;
          hwSlot.dstRnicBitmapWords = slots[k].bitmapWords;
          hwSlots.push_back (hwSlot);
        }

      transport->EnableInjectionGate(
        rnic,
        0,
        periodNs,
        hwSlots);
    }
}


} // namespace ns3
