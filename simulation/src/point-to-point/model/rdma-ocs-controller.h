/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef RDMA_OCS_CONTROLLER_H
#define RDMA_OCS_CONTROLLER_H

#include "ns3/object.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"

#include <map>
#include <set>
#include <string>
#include <vector>
#include <stdint.h>

namespace ns3 {
class RdmaHw;
class RdmaUserspaceTransport;



class RdmaOcsController : public Object
{
public:
  static TypeId GetTypeId (void);

  RdmaOcsController ();
  virtual ~RdmaOcsController ();

  void SetNodeContainer (NodeContainer nodes);

  void AddOcsNode (uint32_t nodeId);

  /*
   * The topology file uses stable logical port IDs.
   * ns-3 devices use interface indices.
   *
   * For Commit 1, only nodeId/logicalPort/ifIndex is required.
   * peerNodeId and peerLogicalPort are kept for later reachability compilation.
   */
  void AddPortBinding (uint32_t nodeId,
                       uint32_t logicalPort,
                       uint32_t ifIndex,
                       uint32_t peerNodeId,
                       uint32_t peerLogicalPort,
                       uint64_t linkDelayNs,
                       uint64_t linkBandwidthBps);

  void SetRnicGatePacketBytes (uint32_t packetBytes);
  void SetRnicGateAckBytes (uint32_t packetBytes);
  void SetRnicGateExtraMarginNs (uint64_t marginNs);
  void SetRnicGateBurstBytes (uint64_t burstBytes);

  void LoadAndInstallOcsMap (const std::string &filename);
  void LoadAndInstallOcsSchedule (const std::string &filename);

  void BuildRnicGroups ();
  void CompileRnicReachabilityWindows ();
  void DumpRnicGroups () const;
  void DumpRnicReachabilityWindows () const;
  void InstallRnicGateTablesToRdmaHw () const;
  void InstallRnicGateTablesToUserspace () const;

  uint32_t GetRnicGroupForNode (uint32_t nodeId) const;


private:
  struct PortBinding
  {
    uint32_t ifIndex;
    uint32_t peerNodeId;
    uint32_t peerLogicalPort;
    uint64_t linkDelayNs;
    uint64_t linkBandwidthBps;
  };

  struct OcsScheduleConfig
  {
    uint32_t epochStartUs;
    uint32_t sliceDurationUs;
    uint32_t switchingTimeUs;
    uint32_t numSlices;
  };

  uint32_t ResolveLogicalPortToIf (uint32_t nodeId,
                                   uint32_t logicalPort) const;

  bool IsOcsNode (uint32_t nodeId) const;

    enum RnicGroupType
  {
    RNIC_DIRECT_OCS = 0,
    EPS_AGGREGATED = 1
  };

  struct RnicGroup
  {
    uint32_t groupId;
    RnicGroupType type;
    uint32_t attachmentNode;
    std::vector<uint32_t> rnicNodes;
  };

  struct OcsScheduleEntry
  {
    uint32_t ocsId;
    uint32_t slice;
    uint32_t logicalPortA;
    uint32_t logicalPortB;
    uint32_t actualIfA;
    uint32_t actualIfB;
  };

  struct RnicReachabilityWindow
  {
    uint32_t srcGroup;
    uint32_t dstGroup;
    uint32_t ocsId;
    uint32_t slice;
    Time startOffset;
    Time endOffset;
    Time period;
  };

  bool IsEndpointNode (uint32_t nodeId) const;
  uint32_t GetDegree (uint32_t nodeId) const;
  uint32_t GetGroupForOcsLogicalPort (uint32_t ocsId,
                                      uint32_t logicalPort) const;

private:
  NodeContainer m_nodes;

  std::set<uint32_t> m_ocsNodeIds;

  // nodeId -> logicalPort -> binding
  std::map<uint32_t, std::map<uint32_t, PortBinding> > m_portBindings;

  // ocsId -> timing config
  std::map<uint32_t, OcsScheduleConfig> m_ocsScheduleConfigs;

  std::vector<OcsScheduleEntry> m_ocsScheduleEntries;

  // RNIC nodeId -> groupId
  std::map<uint32_t, uint32_t> m_nodeToRnicGroup;

  // groupId -> group
  std::map<uint32_t, RnicGroup> m_rnicGroups;

  // attachment node -> groupId
  // For EPS_AGGREGATED: attachment node is EPS/ToR node.
  // For RNIC_DIRECT_OCS: attachment node is the RNIC node itself.
  std::map<uint32_t, uint32_t> m_attachmentNodeToRnicGroup;

  uint32_t m_rnicGatePacketBytes;
  uint32_t m_rnicGateAckBytes;
  uint64_t m_rnicGateExtraMarginNs;
  uint64_t m_rnicGateBurstBytes;

  std::vector<RnicReachabilityWindow> m_rnicReachabilityWindows;
};

} // namespace ns3

#endif /* RDMA_OCS_CONTROLLER_H */
