#ifndef OCS_DATA_PLANE_CONTROLLER_H
#define OCS_DATA_PLANE_CONTROLLER_H

#include "ns3/object.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"

#include <map>
#include <set>
#include <string>
#include <vector>
#include <stdint.h>
#include <ostream>

namespace ns3 {

/**
 * Data-plane-only OCS controller.
 *
 * This class deliberately does not compile RNIC injection windows and does not
 * touch RDMA QPs.  Its only responsibility is to translate stable topology
 * logical ports plus a compact schedule file into OcsNode circuit state.
 *
 * Schedule format:
 *
 *   SCHEDULE OCS=24,25 EPOCH_US=0 SLICE_US=10000 SWITCH_US=10
 *   ROUND_ROBIN PORTS=0-3
 *
 * or explicit:
 *
 *   SCHEDULE OCS=24,25 EPOCH_US=0 SLICE_US=10000 SWITCH_US=10
 *   SLICE 0 PAIRS=0-1,2-3
 *   SLICE 1 PAIRS=0-2,1-3
 *   SLICE 2 PAIRS=0-3,1-2
 *
 * The generated circuit semantics are intentionally identical to OcsNode:
 *   match(actual_ifindex, current_slice) -> connected_actual_ifindex
 */
class OcsDataPlaneController : public Object
{
public:
  static TypeId GetTypeId(void);

  OcsDataPlaneController();

  void SetNodeContainer(NodeContainer nodes);

  void AddOcsNode(uint32_t nodeId);

  void AddPortBinding(uint32_t nodeId,
                      uint32_t logicalPort,
                      uint32_t ifIndex,
                      uint32_t peerNodeId,
                      uint32_t peerLogicalPort,
                      uint64_t linkDelayNs,
                      uint64_t linkBandwidthBps);

  void LoadStaticMap(const std::string& filename);

  void LoadCompactSchedule(const std::string& filename);

  void DumpPortBindings(std::ostream& os) const;

  void DumpExpandedSchedule(std::ostream& os) const;

  /*
   * Batch3a: compile OCS schedule into endpoint-side RNIC injection windows.
   * RnicGroup is an internal compilation map, not a user-visible data-plane table.
   */
  void BuildRnicGroups();
  void CompileRnicReachabilityWindows();

private:
  struct PortBinding
  {
    uint32_t ifIndex;
    uint32_t peerNodeId;
    uint32_t peerLogicalPort;
    uint64_t linkDelayNs;
    uint64_t linkBandwidthBps;
  };

  struct ExpandedEntry
  {
    uint32_t ocsId;
    uint32_t slice;
    uint32_t logicalPortA;
    uint32_t logicalPortB;
    uint32_t actualIfA;
    uint32_t actualIfB;
  };

  struct ExpandedConfig
  {
    uint32_t ocsId;
    uint64_t epochUs;
    uint64_t sliceUs;
    uint64_t switchUs;
    uint32_t numSlices;
  };

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

    ScheduleBlock()
      : active(false),
        epochUs(0),
        sliceUs(0),
        switchUs(0),
        hasRoundRobin(false)
    {
    }
  };

  bool IsOcsNode(uint32_t nodeId) const;

  uint32_t ResolveLogicalPortToIf(uint32_t nodeId,
                                  uint32_t logicalPort) const;

  void InstallPair(uint32_t ocsId,
                   uint32_t slice,
                   uint32_t logicalPortA,
                   uint32_t logicalPortB);

  void ConfigureOcsSchedule(uint32_t ocsId,
                            uint64_t epochUs,
                            uint64_t sliceUs,
                            uint64_t switchUs,
                            uint32_t numSlices);

  std::vector<std::pair<uint32_t, uint32_t> >
  GenerateRoundRobinPairsForSlice(const std::vector<uint32_t>& ports,
                                  uint32_t slice) const;

  void FlushScheduleBlock(const ScheduleBlock& block);

  std::vector<uint32_t> ParseUintList(const std::string& value) const;

  std::vector<std::pair<uint32_t, uint32_t> >
  ParsePairList(const std::string& value) const;

  std::map<std::string, std::string>
  ParseKeyValues(const std::vector<std::string>& tokens,
                 uint32_t firstIndex) const;

  static std::string Trim(const std::string& s);
  static std::vector<std::string> SplitWs(const std::string& s);
  static bool StartsWith(const std::string& s, const std::string& prefix);

private:
  
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

  uint32_t GetDegree(uint32_t nodeId) const;
  bool IsEndpointNode(uint32_t nodeId) const;
  uint32_t GetGroupForOcsLogicalPort(uint32_t ocsId, uint32_t logicalPort) const;
  uint64_t GetEndpointOffsetNs(const RnicGroup& group, uint32_t packetBytes) const;

  // Internal endpoint attachment map.
  std::map<uint32_t, uint32_t> m_nodeToRnicGroup;
  std::map<uint32_t, RnicGroup> m_rnicGroups;
  std::map<uint32_t, uint32_t> m_attachmentNodeToRnicGroup;
  std::vector<RnicReachabilityWindow> m_rnicReachabilityWindows;
  uint32_t m_rnicGatePacketBytes = 1200;
  uint32_t m_rnicGateAckBytes = 92;

  NodeContainer m_nodes;
  std::set<uint32_t> m_ocsNodeIds;

  // nodeId -> logicalPort -> binding
  std::map<uint32_t, std::map<uint32_t, PortBinding> > m_portBindings;

  std::vector<ExpandedConfig> m_expandedConfigs;
  std::vector<ExpandedEntry> m_expandedEntries;

  std::set<uint32_t> m_configuredOcs;
};

} // namespace ns3

#endif // OCS_DATA_PLANE_CONTROLLER_H
