/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef TDM_CONTROLLER_H
#define TDM_CONTROLLER_H

#include "ns3/object.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"

#include <map>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <stdint.h>
#include <ostream>

namespace ns3 {
class RdmaHw;
class RdmaTransport;



class TdmController : public Object
{
public:
  static TypeId GetTypeId (void);

  TdmController ();
  virtual ~TdmController ();

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

  // Centralized OCS/TDM schedule input and data-plane programming.
  void LoadStaticMap (const std::string &filename);
  void LoadCompactSchedule (const std::string &filename);
  void LoadAndInstallOcsMap (const std::string &filename);       // compatibility
  void LoadAndInstallOcsSchedule (const std::string &filename);  // expanded format compatibility
  void DumpPortBindings (std::ostream &os) const;
  void DumpExpandedSchedule (std::ostream &os) const;

  void BuildRnicGroups ();
  void CompileRnicReachabilityWindows ();
  void DumpRnicGroups () const;
  void DumpRnicReachabilityWindows () const;
  void InstallRdmaGateTables (uint32_t mode) const;

  uint32_t GetRnicGroupForNode (uint32_t nodeId) const;
  uint32_t GetRnicGroupForEndpoint (uint32_t nodeId,
                                    uint32_t rnicPortId) const;


private:
  struct ScheduleBlock
  {
    bool active;
    std::vector<uint32_t> ocsIds;
    uint64_t epochUs;
    uint64_t sliceUs;
    uint64_t switchUs;
    bool hasRoundRobin;
    std::vector<uint32_t> rrPorts;

    struct SliceDef
    {
      uint32_t slice;
      std::vector<std::pair<uint32_t, uint32_t> > pairs;
    };

    std::vector<SliceDef> slices;

    ScheduleBlock ()
      : active (false),
        epochUs (0),
        sliceUs (0),
        switchUs (0),
        hasRoundRobin (false)
    {
    }
  };

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

  void ConfigureOcsSchedule (uint32_t ocsId,
                             uint64_t epochUs,
                             uint64_t sliceUs,
                             uint64_t switchUs,
                             uint32_t numSlices);
  void InstallPair (uint32_t ocsId,
                    uint32_t slice,
                    uint32_t logicalPortA,
                    uint32_t logicalPortB);
  std::vector<std::pair<uint32_t, uint32_t> >
  GenerateRoundRobinPairsForSlice (const std::vector<uint32_t> &ports,
                                   uint32_t slice) const;
  void FlushScheduleBlock (const ScheduleBlock &block);
  std::vector<uint32_t> ParseUintList (const std::string &value) const;
  std::vector<std::pair<uint32_t, uint32_t> >
  ParsePairList (const std::string &value) const;
  std::map<std::string, std::string>
  ParseKeyValues (const std::vector<std::string> &tokens,
                  uint32_t firstIndex) const;
  static std::string Trim (const std::string &s);
  static std::vector<std::string> SplitWs (const std::string &s);
  static bool StartsWith (const std::string &s, const std::string &prefix);

  void InstallRnicGateTablesToRdmaHw () const;
  void InstallRnicGateTablesToUserspace () const;

    enum RnicGroupType
  {
    RNIC_DIRECT_OCS = 0,
    EPS_AGGREGATED = 1
  };

  typedef std::pair<uint32_t, uint32_t> RnicEndpointKey;

  struct RnicEndpoint
  {
    uint32_t nodeId;
    uint32_t rnicPortId;
  };

  struct RnicGroup
  {
    uint32_t groupId;
    RnicGroupType type;
    uint32_t attachmentNode;
    uint32_t attachmentLogicalPort;
    std::vector<RnicEndpoint> endpoints;
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
  std::set<uint32_t> m_configuredOcs;

  // (RNIC nodeId, stable RNIC port id) -> groupId.
  // The stable port id is the encoded <nicId, planeId> topology identity.
  std::map<RnicEndpointKey, uint32_t> m_endpointToRnicGroup;

  // RNIC nodeId -> all groups containing one of its scale-out ports.
  std::map<uint32_t, std::vector<uint32_t> > m_nodeToRnicGroups;

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

#endif /* TDM_CONTROLLER_H */
