#include <iomanip>
#include "ocs-data-plane-controller.h"

#include "ns3/log.h"
#include "ns3/abort.h"
#include "ns3/simulator.h"
#include "ns3/ocs-node.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ns3 {

static uint64_t
Batch3aCeilDivUint64(uint64_t a, uint64_t b)
{
  NS_ASSERT_MSG(b > 0, "division by zero");
  return (a + b - 1) / b;
}

static uint64_t
Batch3aCalcSerializationNs(uint32_t packetBytes, uint64_t bandwidthBps)
{
  if (packetBytes == 0 || bandwidthBps == 0)
    {
      return 0;
    }
  return Batch3aCeilDivUint64(static_cast<uint64_t>(packetBytes) * 8ULL * 1000000000ULL,
                              bandwidthBps);
}

static void
Batch3aSetBitmap(std::vector<uint64_t>& words, uint32_t nodeId)
{
  uint32_t word = nodeId / 64;
  uint32_t bit = nodeId % 64;
  if (word >= words.size())
    {
      words.resize(word + 1, 0);
    }
  words[word] |= (1ULL << bit);
}

static bool
Batch3aBitmapEmpty(const std::vector<uint64_t>& words)
{
  for (uint32_t i = 0; i < words.size(); ++i)
    {
      if (words[i] != 0)
        {
          return false;
        }
    }
  return true;
}

static std::set<uint32_t>
Batch3aBitmapToSet(const std::vector<uint64_t>& words)
{
  std::set<uint32_t> nodes;
  for (uint32_t w = 0; w < words.size(); ++w)
    {
      uint64_t x = words[w];
      for (uint32_t b = 0; b < 64; ++b)
        {
          if (x & (1ULL << b))
            {
              nodes.insert(w * 64 + b);
            }
        }
    }
  return nodes;
}



NS_LOG_COMPONENT_DEFINE("OcsDataPlaneController");
NS_OBJECT_ENSURE_REGISTERED(OcsDataPlaneController);

TypeId
OcsDataPlaneController::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::OcsDataPlaneController")
    .SetParent<Object>()
    .SetGroupName("PointToPoint")
    .AddConstructor<OcsDataPlaneController>();
  return tid;
}

OcsDataPlaneController::OcsDataPlaneController()
{
}

void
OcsDataPlaneController::SetNodeContainer(NodeContainer nodes)
{
  m_nodes = nodes;
}

void
OcsDataPlaneController::AddOcsNode(uint32_t nodeId)
{
  m_ocsNodeIds.insert(nodeId);
}

bool
OcsDataPlaneController::IsOcsNode(uint32_t nodeId) const
{
  return m_ocsNodeIds.find(nodeId) != m_ocsNodeIds.end();
}

void
OcsDataPlaneController::AddPortBinding(uint32_t nodeId,
                                       uint32_t logicalPort,
                                       uint32_t ifIndex,
                                       uint32_t peerNodeId,
                                       uint32_t peerLogicalPort,
                                       uint64_t linkDelayNs,
                                       uint64_t linkBandwidthBps)
{
  PortBinding b;
  b.ifIndex = ifIndex;
  b.peerNodeId = peerNodeId;
  b.peerLogicalPort = peerLogicalPort;
  b.linkDelayNs = linkDelayNs;
  b.linkBandwidthBps = linkBandwidthBps;

  std::map<uint32_t, PortBinding>& byPort = m_portBindings[nodeId];
  if (byPort.find(logicalPort) != byPort.end())
    {
      NS_ABORT_MSG("Duplicate logical port binding: node="
                   << nodeId << " logicalPort=" << logicalPort);
    }

  byPort[logicalPort] = b;
}

uint32_t
OcsDataPlaneController::ResolveLogicalPortToIf(uint32_t nodeId,
                                               uint32_t logicalPort) const
{
  std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator n =
    m_portBindings.find(nodeId);
  if (n == m_portBindings.end())
    {
      NS_ABORT_MSG("No port binding for node " << nodeId);
    }

  std::map<uint32_t, PortBinding>::const_iterator p = n->second.find(logicalPort);
  if (p == n->second.end())
    {
      NS_ABORT_MSG("No logical port binding: node="
                   << nodeId << " logicalPort=" << logicalPort);
    }

  return p->second.ifIndex;
}

void
OcsDataPlaneController::ConfigureOcsSchedule(uint32_t ocsId,
                                             uint64_t epochUs,
                                             uint64_t sliceUs,
                                             uint64_t switchUs,
                                             uint32_t numSlices)
{
  if (!IsOcsNode(ocsId))
    {
      NS_ABORT_MSG("Schedule references non-OCS node " << ocsId);
    }
  if (ocsId >= m_nodes.GetN())
    {
      NS_ABORT_MSG("OCS node id out of NodeContainer range: " << ocsId);
    }
  if (sliceUs == 0)
    {
      NS_ABORT_MSG("SLICE_US must be positive for OCS " << ocsId);
    }
  if (switchUs >= sliceUs)
    {
      NS_ABORT_MSG("SWITCH_US must be smaller than SLICE_US for OCS " << ocsId);
    }
  if (numSlices == 0)
    {
      NS_ABORT_MSG("numSlices must be positive for OCS " << ocsId);
    }
  if (m_configuredOcs.find(ocsId) != m_configuredOcs.end())
    {
      NS_ABORT_MSG("OCS " << ocsId << " is configured more than once");
    }

  Ptr<OcsNode> ocs = DynamicCast<OcsNode>(m_nodes.Get(ocsId));
  if (ocs == 0)
    {
      NS_ABORT_MSG("Node " << ocsId << " is not an OcsNode");
    }

  ocs->ConfigureSchedule(MicroSeconds(epochUs),
                         MicroSeconds(sliceUs),
                         numSlices,
                         MicroSeconds(switchUs));
  ocs->ClearSchedule();

  ExpandedConfig cfg;
  cfg.ocsId = ocsId;
  cfg.epochUs = epochUs;
  cfg.sliceUs = sliceUs;
  cfg.switchUs = switchUs;
  cfg.numSlices = numSlices;
  m_expandedConfigs.push_back(cfg);

  m_configuredOcs.insert(ocsId);
}

void
OcsDataPlaneController::InstallPair(uint32_t ocsId,
                                    uint32_t slice,
                                    uint32_t logicalPortA,
                                    uint32_t logicalPortB)
{
  if (logicalPortA == logicalPortB)
    {
      NS_ABORT_MSG("OCS pair connects a port to itself: OCS="
                   << ocsId << " slice=" << slice
                   << " port=" << logicalPortA);
    }

  uint32_t ifA = ResolveLogicalPortToIf(ocsId, logicalPortA);
  uint32_t ifB = ResolveLogicalPortToIf(ocsId, logicalPortB);

  Ptr<OcsNode> ocs = DynamicCast<OcsNode>(m_nodes.Get(ocsId));
  if (ocs == 0)
    {
      NS_ABORT_MSG("Node " << ocsId << " is not an OcsNode");
    }

  ocs->AddBidirectionalScheduleEntry(ifA, ifB, slice);

  ExpandedEntry e;
  e.ocsId = ocsId;
  e.slice = slice;
  e.logicalPortA = logicalPortA;
  e.logicalPortB = logicalPortB;
  e.actualIfA = ifA;
  e.actualIfB = ifB;
  m_expandedEntries.push_back(e);
}

std::vector<std::pair<uint32_t, uint32_t> >
OcsDataPlaneController::GenerateRoundRobinPairsForSlice(
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
OcsDataPlaneController::FlushScheduleBlock(const ScheduleBlock& block)
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
OcsDataPlaneController::LoadCompactSchedule(const std::string& filename)
{
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
                << " configs=" << m_expandedConfigs.size()
                << " entries=" << m_expandedEntries.size());
}

void
OcsDataPlaneController::LoadStaticMap(const std::string& filename)
{
  std::ifstream fin(filename.c_str());
  if (!fin.is_open())
    {
      NS_ABORT_MSG("Cannot open OCS map file: " << filename);
    }

  std::map<uint32_t, std::vector<std::pair<uint32_t, uint32_t> > > byOcs;

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

      std::vector<std::string> t = SplitWs(line);
      if (t.size() != 3)
        {
          NS_ABORT_MSG("Invalid OCS map line " << lineNo
                       << ": expected <ocs_id> <port_a> <port_b>");
        }

      uint32_t ocsId = static_cast<uint32_t>(std::strtoul(t[0].c_str(), 0, 10));
      uint32_t a = static_cast<uint32_t>(std::strtoul(t[1].c_str(), 0, 10));
      uint32_t b = static_cast<uint32_t>(std::strtoul(t[2].c_str(), 0, 10));

      uint32_t ifA = ResolveLogicalPortToIf(ocsId, a);
      uint32_t ifB = ResolveLogicalPortToIf(ocsId, b);
      byOcs[ocsId].push_back(std::make_pair(ifA, ifB));
    }

  for (std::map<uint32_t, std::vector<std::pair<uint32_t, uint32_t> > >::const_iterator it =
         byOcs.begin(); it != byOcs.end(); ++it)
    {
      if (!IsOcsNode(it->first))
        {
          NS_ABORT_MSG("Static OCS map references non-OCS node " << it->first);
        }

      Ptr<OcsNode> ocs = DynamicCast<OcsNode>(m_nodes.Get(it->first));
      if (ocs == 0)
        {
          NS_ABORT_MSG("Node " << it->first << " is not an OcsNode");
        }

      ocs->SetInitialMapping(it->second);
    }

  NS_LOG_UNCOND("[OCS MAP INSTALLED] file=" << filename
                << " ocs_count=" << byOcs.size());
}

void
OcsDataPlaneController::DumpPortBindings(std::ostream& os) const
{
  os << "# DEBUG: node_id logical_port if_index peer_node peer_logical_port delay_ns bandwidth_bps\n";
  for (std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator n =
         m_portBindings.begin(); n != m_portBindings.end(); ++n)
    {
      for (std::map<uint32_t, PortBinding>::const_iterator p =
             n->second.begin(); p != n->second.end(); ++p)
        {
          os << n->first << " "
             << p->first << " "
             << p->second.ifIndex << " "
             << p->second.peerNodeId << " "
             << p->second.peerLogicalPort << " "
             << p->second.linkDelayNs << " "
             << p->second.linkBandwidthBps << "\n";
        }
    }
}

void
OcsDataPlaneController::DumpExpandedSchedule(std::ostream& os) const
{
  os << "# Expanded OCS schedule (logical ports)\n";
  os << "# CONFIG ocs_id epoch_start_us slice_duration_us switching_time_us num_slices\n";
  for (uint32_t i = 0; i < m_expandedConfigs.size(); ++i)
    {
      const ExpandedConfig& c = m_expandedConfigs[i];
      os << "CONFIG "
         << c.ocsId << " "
         << c.epochUs << " "
         << c.sliceUs << " "
         << c.switchUs << " "
         << c.numSlices << "\n";
    }

  os << "\n# ocs_id slice port_a port_b\n";
  for (uint32_t i = 0; i < m_expandedEntries.size(); ++i)
    {
      const ExpandedEntry& e = m_expandedEntries[i];
      os << e.ocsId << " "
         << e.slice << " "
         << e.logicalPortA << " "
         << e.logicalPortB << "\n";
    }
}

std::vector<uint32_t>
OcsDataPlaneController::ParseUintList(const std::string& value) const
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
OcsDataPlaneController::ParsePairList(const std::string& value) const
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
OcsDataPlaneController::ParseKeyValues(const std::vector<std::string>& tokens,
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
OcsDataPlaneController::Trim(const std::string& s)
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
OcsDataPlaneController::SplitWs(const std::string& s)
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
OcsDataPlaneController::StartsWith(const std::string& s,
                                   const std::string& prefix)
{
  return s.size() >= prefix.size() &&
         std::equal(prefix.begin(), prefix.end(), s.begin());
}


uint32_t
OcsDataPlaneController::GetDegree(uint32_t nodeId) const
{
  std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator it =
    m_portBindings.find(nodeId);
  if (it == m_portBindings.end())
    {
      return 0;
    }
  return it->second.size();
}

bool
OcsDataPlaneController::IsEndpointNode(uint32_t nodeId) const
{
  if (IsOcsNode(nodeId))
    {
      return false;
    }

  /*
   * Same rule as the legacy RdmaOcsController:
   * a non-OCS node with one fabric-facing port is treated as an RNIC endpoint.
   */
  return GetDegree(nodeId) == 1;
}

uint64_t
OcsDataPlaneController::GetEndpointOffsetNs(const RnicGroup& group,
                                            uint32_t packetBytes) const
{
  uint64_t maxOffsetNs = 0;

  for (uint32_t i = 0; i < group.rnicNodes.size(); ++i)
    {
      uint32_t rnic = group.rnicNodes[i];

      std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
        m_portBindings.find(rnic);

      if (nodeIt == m_portBindings.end() || nodeIt->second.empty())
        {
          continue;
        }

      const PortBinding& binding = nodeIt->second.begin()->second;
      uint64_t offsetNs = binding.linkDelayNs +
        Batch3aCalcSerializationNs(packetBytes, binding.linkBandwidthBps);

      if (offsetNs > maxOffsetNs)
        {
          maxOffsetNs = offsetNs;
        }
    }

  return maxOffsetNs;
}

void
OcsDataPlaneController::BuildRnicGroups()
{
  m_nodeToRnicGroup.clear();
  m_rnicGroups.clear();
  m_attachmentNodeToRnicGroup.clear();

  uint32_t nextGroupId = 0;

  /*
   * Pass 1: RNIC directly connected to an OCS.
   */
  for (std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
         m_portBindings.begin();
       nodeIt != m_portBindings.end();
       ++nodeIt)
    {
      uint32_t nodeId = nodeIt->first;
      if (!IsEndpointNode(nodeId))
        {
          continue;
        }

      const std::map<uint32_t, PortBinding>& ports = nodeIt->second;
      NS_ASSERT_MSG(ports.size() == 1, "Endpoint node should have exactly one port");
      const PortBinding& binding = ports.begin()->second;

      if (!IsOcsNode(binding.peerNodeId))
        {
          continue;
        }

      RnicGroup group;
      group.groupId = nextGroupId;
      group.type = RNIC_DIRECT_OCS;
      group.attachmentNode = nodeId;
      group.rnicNodes.push_back(nodeId);

      m_rnicGroups[nextGroupId] = group;
      m_nodeToRnicGroup[nodeId] = nextGroupId;
      m_attachmentNodeToRnicGroup[nodeId] = nextGroupId;
      nextGroupId++;
    }

  /*
   * Pass 2: RNICs connected to the same EPS/ToR are aggregated.
   */
  std::map<uint32_t, std::vector<uint32_t> > epsToRnicNodes;

  for (std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
         m_portBindings.begin();
       nodeIt != m_portBindings.end();
       ++nodeIt)
    {
      uint32_t nodeId = nodeIt->first;
      if (!IsEndpointNode(nodeId))
        {
          continue;
        }
      if (m_nodeToRnicGroup.find(nodeId) != m_nodeToRnicGroup.end())
        {
          continue;
        }

      const std::map<uint32_t, PortBinding>& ports = nodeIt->second;
      NS_ASSERT_MSG(ports.size() == 1, "Endpoint node should have exactly one port");
      const PortBinding& binding = ports.begin()->second;

      if (IsOcsNode(binding.peerNodeId))
        {
          continue;
        }

      epsToRnicNodes[binding.peerNodeId].push_back(nodeId);
    }

  for (std::map<uint32_t, std::vector<uint32_t> >::iterator it =
         epsToRnicNodes.begin();
       it != epsToRnicNodes.end();
       ++it)
    {
      RnicGroup group;
      group.groupId = nextGroupId;
      group.type = EPS_AGGREGATED;
      group.attachmentNode = it->first;
      group.rnicNodes = it->second;

      std::sort(group.rnicNodes.begin(), group.rnicNodes.end());

      m_rnicGroups[nextGroupId] = group;
      m_attachmentNodeToRnicGroup[group.attachmentNode] = nextGroupId;

      for (uint32_t i = 0; i < group.rnicNodes.size(); ++i)
        {
          m_nodeToRnicGroup[group.rnicNodes[i]] = nextGroupId;
        }

      nextGroupId++;
    }

  std::cout << "[RNIC GROUP MAP COMPILED] groups=" << m_rnicGroups.size()
            << " rnics=" << m_nodeToRnicGroup.size()
            << std::endl;
}

uint32_t
OcsDataPlaneController::GetGroupForOcsLogicalPort(uint32_t ocsId,
                                                  uint32_t logicalPort) const
{
  std::map<uint32_t, std::map<uint32_t, PortBinding> >::const_iterator nodeIt =
    m_portBindings.find(ocsId);
  NS_ASSERT_MSG(nodeIt != m_portBindings.end(), "No port binding found for OCS node");

  std::map<uint32_t, PortBinding>::const_iterator portIt =
    nodeIt->second.find(logicalPort);
  NS_ASSERT_MSG(portIt != nodeIt->second.end(), "OCS logical port has no binding");

  uint32_t peerNode = portIt->second.peerNodeId;

  std::map<uint32_t, uint32_t>::const_iterator nodeGroupIt =
    m_nodeToRnicGroup.find(peerNode);
  if (nodeGroupIt != m_nodeToRnicGroup.end())
    {
      return nodeGroupIt->second;
    }

  std::map<uint32_t, uint32_t>::const_iterator attachmentGroupIt =
    m_attachmentNodeToRnicGroup.find(peerNode);
  if (attachmentGroupIt != m_attachmentNodeToRnicGroup.end())
    {
      return attachmentGroupIt->second;
    }

  NS_ASSERT_MSG(false, "OCS logical port cannot be mapped to an RNIC group");
  return 0;
}

void
OcsDataPlaneController::CompileRnicReachabilityWindows()
{
  m_rnicReachabilityWindows.clear();

  if (m_expandedEntries.empty())
    {
      std::cout << "[RNIC REACHABILITY] no OCS schedule entries; skip"
                << std::endl;
      return;
    }

  if (m_rnicGroups.empty())
    {
      BuildRnicGroups();
    }

  for (uint32_t ci = 0; ci < m_expandedConfigs.size(); ++ci)
    {
      const ExpandedConfig& cfg = m_expandedConfigs[ci];
      uint64_t epochNs = static_cast<uint64_t>(cfg.epochUs) * 1000ULL;
      uint64_t sliceNs = static_cast<uint64_t>(cfg.sliceUs) * 1000ULL;
      uint64_t switchingNs = static_cast<uint64_t>(cfg.switchUs) * 1000ULL;
      uint64_t periodNs = sliceNs * static_cast<uint64_t>(cfg.numSlices);

      if (periodNs == 0 || sliceNs <= switchingNs)
        {
          continue;
        }

      for (uint32_t ei = 0; ei < m_expandedEntries.size(); ++ei)
        {
          const ExpandedEntry& e = m_expandedEntries[ei];
          if (e.ocsId != cfg.ocsId)
            {
              continue;
            }
          if (e.slice >= cfg.numSlices)
            {
              continue;
            }

          uint32_t groupA = GetGroupForOcsLogicalPort(e.ocsId, e.logicalPortA);
          uint32_t groupB = GetGroupForOcsLogicalPort(e.ocsId, e.logicalPortB);
          if (groupA == groupB)
            {
              continue;
            }

          uint64_t sliceStartNs =
            (epochNs + static_cast<uint64_t>(e.slice) * sliceNs) % periodNs;
          uint64_t stableEndNs =
            (epochNs + static_cast<uint64_t>(e.slice + 1) * sliceNs - switchingNs) % periodNs;

          /*
           * Batch3a follows the legacy controller's tail-trimming direction:
           * injection must finish early enough before the next switching window.
           *
           * This first migration computes endpoint/link offsets from the current
           * port bindings and leaves the full graph-based ACK deadline model for
           * the next install phase.
           */
          const RnicGroup& a = m_rnicGroups[groupA];
          const RnicGroup& b = m_rnicGroups[groupB];

          uint64_t trimAB =
            GetEndpointOffsetNs(a, m_rnicGatePacketBytes) +
            GetEndpointOffsetNs(b, m_rnicGatePacketBytes) +
            GetEndpointOffsetNs(b, m_rnicGateAckBytes);

          uint64_t trimBA =
            GetEndpointOffsetNs(b, m_rnicGatePacketBytes) +
            GetEndpointOffsetNs(a, m_rnicGatePacketBytes) +
            GetEndpointOffsetNs(a, m_rnicGateAckBytes);

          if (stableEndNs > sliceStartNs + trimAB)
            {
              RnicReachabilityWindow w;
              w.srcGroup = groupA;
              w.dstGroup = groupB;
              w.ocsId = e.ocsId;
              w.slice = e.slice;
              w.startOffset = NanoSeconds(sliceStartNs);
              w.endOffset = NanoSeconds(stableEndNs - trimAB);
              w.period = NanoSeconds(periodNs);
              m_rnicReachabilityWindows.push_back(w);
            }

          if (stableEndNs > sliceStartNs + trimBA)
            {
              RnicReachabilityWindow w;
              w.srcGroup = groupB;
              w.dstGroup = groupA;
              w.ocsId = e.ocsId;
              w.slice = e.slice;
              w.startOffset = NanoSeconds(sliceStartNs);
              w.endOffset = NanoSeconds(stableEndNs - trimBA);
              w.period = NanoSeconds(periodNs);
              m_rnicReachabilityWindows.push_back(w);
            }
        }
    }

  std::cout << "[RNIC REACHABILITY COMPILED]"
            << " windows=" << m_rnicReachabilityWindows.size()
            << std::endl;
}




} // namespace ns3
