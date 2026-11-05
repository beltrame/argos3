/**
 * @file <argos3/plugins/simulator/media/mesh_medium.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "mesh_medium.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/configuration/argos_exception.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

namespace argos {

   /****************************************/
   /****************************************/

   Real CMeshMedium::LinkRateMbps(Real f_distance, Real f_range) {
      if(!(f_distance >= 0.0 && f_distance <= f_range)) return 0.0;
      return MAX_RATE_MBPS - (MAX_RATE_MBPS - EDGE_RATE_MBPS) * f_distance / f_range;
   }

   /****************************************/
   /****************************************/

   std::vector<CMeshMedium::SRoute> CMeshMedium::ComputeRoutes(const std::vector<SNode>& vec_nodes,
                                                               Real f_range,
                                                               bool b_unlimited) {
      if(!b_unlimited && !(std::isfinite(f_range) && f_range > 0.0)) {
         THROW_ARGOSEXCEPTION("mesh range_m must be finite and positive");
      }
      std::set<std::string> setIds;
      for(const SNode& sNode : vec_nodes) {
         if(!setIds.insert(sNode.Id).second) {
            THROW_ARGOSEXCEPTION("duplicate mesh node id \"" << sNode.Id << "\"");
         }
         if(!std::isfinite(sNode.Position.GetX()) ||
            !std::isfinite(sNode.Position.GetY()) ||
            !std::isfinite(sNode.Position.GetZ())) {
            THROW_ARGOSEXCEPTION("mesh node \"" << sNode.Id << "\" has a non-finite position");
         }
      }
      /* Index order is id order: the output and every tie-break follow it. */
      std::vector<SNode> vecSorted(vec_nodes);
      std::sort(vecSorted.begin(), vecSorted.end(),
                [](const SNode& s_a, const SNode& s_b) { return s_a.Id < s_b.Id; });
      const size_t unNodes = vecSorted.size();
      /* Symmetric link rates; 0 means no link. */
      std::vector<std::vector<Real>> vecLink(unNodes, std::vector<Real>(unNodes, 0.0));
      for(size_t i = 0; i < unNodes; ++i) {
         for(size_t j = i + 1; j < unNodes; ++j) {
            const Real fRate = b_unlimited
               ? MAX_RATE_MBPS
               : LinkRateMbps((vecSorted[i].Position - vecSorted[j].Position).Length(), f_range);
            vecLink[i][j] = vecLink[j][i] = fRate;
         }
      }
      constexpr UInt32 unUnreached = std::numeric_limits<UInt32>::max();
      std::vector<SRoute> vecRoutes;
      vecRoutes.reserve(unNodes * unNodes);
      std::vector<UInt32> vecHops(unNodes);
      std::vector<Real> vecBottleneck(unNodes);
      std::vector<size_t> vecFrontier, vecNext;
      for(size_t unSource = 0; unSource < unNodes; ++unSource) {
         /* Breadth-first hop layers; within a layer each node keeps the
          * greatest bottleneck over its predecessors in the previous layer. A
          * strict `>` keeps the first, lexicographically smallest, predecessor
          * on a tie. */
         std::fill(vecHops.begin(), vecHops.end(), unUnreached);
         std::fill(vecBottleneck.begin(), vecBottleneck.end(), 0.0);
         vecHops[unSource] = 0;
         vecBottleneck[unSource] = std::numeric_limits<Real>::infinity();
         vecFrontier.assign(1, unSource);
         for(UInt32 unLayer = 1; !vecFrontier.empty(); ++unLayer) {
            vecNext.clear();
            for(size_t unNode = 0; unNode < unNodes; ++unNode) {
               if(vecHops[unNode] != unUnreached) continue;
               for(size_t unPrevious : vecFrontier) {
                  if(vecLink[unPrevious][unNode] <= 0.0) continue;
                  const Real fCandidate = std::min(vecBottleneck[unPrevious], vecLink[unPrevious][unNode]);
                  if(vecHops[unNode] == unUnreached || fCandidate > vecBottleneck[unNode]) {
                     vecHops[unNode] = unLayer;
                     vecBottleneck[unNode] = fCandidate;
                  }
               }
               if(vecHops[unNode] == unLayer) vecNext.push_back(unNode);
            }
            vecFrontier.swap(vecNext);
         }
         for(size_t unDestination = 0; unDestination < unNodes; ++unDestination) {
            SRoute sRoute{vecSorted[unSource].Id, vecSorted[unDestination].Id, false, 0, 0.0};
            if(unDestination == unSource) {
               sRoute.Reachable = true;
               sRoute.RateMbps = MAX_RATE_MBPS;
            }
            else if(vecHops[unDestination] != unUnreached) {
               sRoute.Reachable = true;
               sRoute.Hops = vecHops[unDestination];
               sRoute.RateMbps = vecBottleneck[unDestination] / vecHops[unDestination];
            }
            vecRoutes.push_back(sRoute);
         }
      }
      return vecRoutes;
   }

   /****************************************/
   /****************************************/

   void CMeshMedium::Init(TConfigurationNode& t_tree) {
      try {
         CMedium::Init(t_tree);
         std::string strRobots;
         GetNodeAttribute(t_tree, "robots", strRobots);
         m_vecRobotIds.clear();
         std::stringstream cRobots(strRobots);
         for(std::string strId; std::getline(cRobots, strId, ',');) {
            if(!strId.empty()) m_vecRobotIds.push_back(strId);
         }
         if(m_vecRobotIds.empty()) {
            THROW_ARGOSEXCEPTION("no robots listed; set robots=\"id,id,...\"");
         }
         GetNodeAttributeOrDefault(t_tree, "range_m", m_fRange, m_fRange);
         GetNodeAttributeOrDefault(t_tree, "unlimited", m_bUnlimited, m_bUnlimited);
         GetNodeAttributeOrDefault(t_tree, "cap_bandwidth", m_bCapBandwidth, m_bCapBandwidth);
         GetNodeAttributeOrDefault(t_tree, "base_id", m_strBaseId, m_strBaseId);
         GetNodeAttribute(t_tree, "base_position", m_cBasePosition);
         if(!(std::isfinite(m_fRange) && m_fRange > 0.0)) {
            THROW_ARGOSEXCEPTION("range_m must be finite and positive");
         }
         if(!m_bCapBandwidth && !m_bUnlimited) {
            THROW_ARGOSEXCEPTION("cap_bandwidth=\"false\" needs unlimited=\"true\"");
         }
         if(std::find(m_vecRobotIds.begin(), m_vecRobotIds.end(), m_strBaseId) != m_vecRobotIds.end()) {
            THROW_ARGOSEXCEPTION("robot id \"" << m_strBaseId << "\" is reserved for the base station");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the mesh medium", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CMeshMedium::PostSpaceInit() {
      m_vecBodies.clear();
      for(const std::string& strId : m_vecRobotIds) {
         try {
            auto* pcComposable = dynamic_cast<CComposableEntity*>(&GetSpace().GetEntity(strId));
            if(pcComposable == nullptr) {
               THROW_ARGOSEXCEPTION("Entity \"" << strId << "\" is not composable.");
            }
            m_vecBodies.push_back(&pcComposable->GetComponent<CEmbodiedEntity>("body"));
         }
         catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("Error locating mesh robot \"" << strId << "\"", ex);
         }
      }
      m_unTick = 0;
      UpdateRoutes();
   }

   /****************************************/
   /****************************************/

   void CMeshMedium::Reset() {
      /* CSimulator::Reset resets the space, and with it the bodies, first. */
      m_unTick = 0;
      UpdateRoutes();
   }

   /****************************************/
   /****************************************/

   void CMeshMedium::Destroy() {
      m_vecBodies.clear();
      m_vecRoutes.clear();
   }

   /****************************************/
   /****************************************/

   void CMeshMedium::Update() {
      m_unTick = GetSpace().GetSimulationClock();
      UpdateRoutes();
   }

   /****************************************/
   /****************************************/

   void CMeshMedium::UpdateRoutes() {
      std::vector<SNode> vecNodes;
      vecNodes.reserve(m_vecBodies.size() + 1);
      for(size_t i = 0; i < m_vecBodies.size(); ++i) {
         vecNodes.push_back({m_vecRobotIds[i], m_vecBodies[i]->GetOriginAnchor().Position});
      }
      vecNodes.push_back({m_strBaseId, m_cBasePosition});
      m_vecRoutes = ComputeRoutes(vecNodes, m_fRange, m_bUnlimited);
   }

   /****************************************/
   /****************************************/

   REGISTER_MEDIUM(CMeshMedium,
                   "mesh_medium",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "Multi-hop connectivity between a set of robots and a base station.",
                   "Every tick, this medium computes the route between every ordered pair of\n"
                   "the listed robots and a fixed base station: whether it is reachable, its\n"
                   "hop count and its estimated rate. Two nodes are linked when they are at\n"
                   "most range_m apart in 3-D; a link runs at 300 Mbps close up, falling\n"
                   "linearly to 2 Mbps at range_m, and a route's rate is its slowest link\n"
                   "divided by its hop count. Routing takes the fewest hops, then the greatest\n"
                   "bottleneck rate, then the lexicographic order of node ids. The medium\n"
                   "carries no payload: code that delivers messages between robots reads the\n"
                   "routes through CMeshMedium::GetRoutes() and gates delivery itself.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <mesh_medium id=\"mesh\" robots=\"robot_0,robot_1\"\n"
                   "               base_position=\"-18,0,0.15\" />\n\n"
                   "The 'robots' attribute lists the ids of the robots in the mesh. Each must be\n"
                   "a composable entity with a 'body' component. The 'base_position' attribute\n"
                   "places the base station in the arena.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "  range_m        the communication range in meters (default 50)\n"
                   "  unlimited      link every pair at 300 Mbps regardless of range\n"
                   "                 (default false)\n"
                   "  cap_bandwidth  whether consumers should cap each pair at its route rate\n"
                   "                 (default true); 'false' requires unlimited=\"true\". The\n"
                   "                 medium never applies the cap itself.\n"
                   "  base_id        the base station's node id (default base_station); no\n"
                   "                 robot may use it\n",
                   "Usable"
      );

   /****************************************/
   /****************************************/

}
