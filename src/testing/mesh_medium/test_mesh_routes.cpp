/*
 * CMeshMedium::ComputeRoutes on its own: geometry, hops, bottlenecks,
 * tie-breaks and invalid input.
 */

#include <argos3/plugins/simulator/media/mesh_medium.h>
#include <argos3/core/utility/configuration/argos_exception.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace argos;

using SNode = CMeshMedium::SNode;
using SRoute = CMeshMedium::SRoute;

namespace {

int g_nFailures = 0;

/* Loose enough for a build where Real is float (ARGOS_USE_DOUBLE=OFF). */
const Real TOLERANCE = 1e-4;

void Expect(bool b_condition, const std::string& str_what) {
   if(!b_condition) {
      ++g_nFailures;
      std::cerr << "FAILED: " << str_what << "\n";
   }
}

void ExpectNear(Real f_actual, Real f_expected, Real f_tolerance, const std::string& str_what) {
   std::ostringstream cMessage;
   cMessage << str_what << ": " << f_actual << " is not within " << f_tolerance << " of " << f_expected;
   Expect(std::isfinite(f_actual) && std::fabs(f_actual - f_expected) < f_tolerance, cMessage.str());
}

std::vector<SRoute> Routes(const std::vector<SNode>& vec_nodes, Real f_range = 50, bool b_unlimited = false) {
   return CMeshMedium::ComputeRoutes(vec_nodes, f_range, b_unlimited);
}

SRoute Route(const std::vector<SRoute>& vec_routes,
             const std::string& str_source, const std::string& str_destination) {
   for(const SRoute& sRoute : vec_routes) {
      if(sRoute.Source == str_source && sRoute.Destination == str_destination) return sRoute;
   }
   throw std::runtime_error("no route " + str_source + "->" + str_destination);
}

void LinkRate() {
   Expect(CMeshMedium::LinkRateMbps(0, 50) == 300, "link_rate(0,50) == 300");
   Expect(CMeshMedium::LinkRateMbps(25, 50) == 151, "link_rate(25,50) == 151");
   Expect(CMeshMedium::LinkRateMbps(50, 50) == 2, "link_rate(50,50) == 2");
   std::vector<SNode> vecNodes = {{"a", CVector3(0, 0, 0)}, {"b", CVector3(50.001, 0, 0)}};
   Expect(!Route(Routes(vecNodes), "a", "b").Reachable,
          "link_rate: 50.001 m apart is unreachable at range 50");
   vecNodes[1].Position.SetX(50.0);
   Expect(Route(Routes(vecNodes), "a", "b").Reachable,
          "link_rate: exactly 50 m apart is reachable at range 50");
}

void MultiHop() {
   const std::vector<SRoute> vecRoutes = Routes({
      {"base_station", CVector3(0, 0, 0)}, {"A", CVector3(40, 0, 0)}, {"B", CVector3(80, 0, 0)}});
   Expect(vecRoutes.size() == 9, "multi_hop: all ordered pairs, self included");
   const SRoute sRelayed = Route(vecRoutes, "base_station", "B");
   Expect(sRelayed.Reachable, "multi_hop: base->B reachable");
   Expect(sRelayed.Hops == 2, "multi_hop: base->B hops == 2");
   ExpectNear(sRelayed.RateMbps, 30.8, TOLERANCE, "multi_hop: base->B rate");
   const SRoute sSelf = Route(vecRoutes, "A", "A");
   Expect(sSelf.Reachable && sSelf.Hops == 0 && sSelf.RateMbps == 300,
          "multi_hop: self route is 0 hops at 300");
   const SRoute sDirect = Route(vecRoutes, "B", "A");
   Expect(sDirect.Hops == 1, "multi_hop: B->A is one hop");
   ExpectNear(sDirect.RateMbps, 61.6, TOLERANCE, "multi_hop: B->A rate");
}

void LongChain() {
   /* Five relays 45 m apart: base -> r4 is five hops, at the 45 m link's
    * rate divided by five. */
   std::vector<SNode> vecNodes = {{"base_station", CVector3(0, 0, 0)}};
   for(int i = 0; i < 5; ++i) {
      vecNodes.push_back({"r" + std::to_string(i), CVector3(45.0 * (i + 1), 0, 0)});
   }
   const SRoute sRoute = Route(Routes(vecNodes), "base_station", "r4");
   Expect(sRoute.Reachable && sRoute.Hops == 5, "long_chain: base->r4 is five hops");
   ExpectNear(sRoute.RateMbps, CMeshMedium::LinkRateMbps(45, 50) / 5, TOLERANCE, "long_chain: rate");
   /* Removing the middle relay partitions the chain. */
   vecNodes.erase(vecNodes.begin() + 3);
   Expect(!Route(Routes(vecNodes), "base_station", "r4").Reachable,
          "long_chain: a missing relay partitions the mesh");
}

void FewestHops() {
   /* The direct 49 m link is slow (7.96 Mbps); relaying through "r" halfway
    * has a far better bottleneck (154 Mbps, 77 Mbps end to end). Fewer hops
    * still win. */
   const SRoute sRoute = Route(Routes({{"base_station", CVector3(0, 0, 0)},
                                       {"r", CVector3(24.5, 0, 0)},
                                       {"dst", CVector3(49, 0, 0)}}),
                               "base_station", "dst");
   Expect(sRoute.Reachable && sRoute.Hops == 1, "fewest_hops: the direct link wins");
   ExpectNear(sRoute.RateMbps, 7.96, TOLERANCE, "fewest_hops: at the direct link's rate");
}

void VerticalRange() {
   const SRoute sRoute = Route(Routes({{"low", CVector3(0, 0, 0)}, {"high", CVector3(0, 0, 51)}}),
                               "low", "high");
   Expect(!sRoute.Reachable, "vertical_range: 51 m of z is unreachable");
   Expect(sRoute.Hops == 0 && sRoute.RateMbps == 0,
          "vertical_range: unreachable route has no hops and no rate");
}

void Unlimited() {
   for(const SRoute& sRoute : Routes({{"base_station", CVector3(0, 0, 0)},
                                      {"A", CVector3(400, 0, 0)},
                                      {"B", CVector3(0, 900, -50)}},
                                     50, true)) {
      if(sRoute.Source == sRoute.Destination) {
         Expect(sRoute.Reachable && sRoute.Hops == 0 && sRoute.RateMbps == 300,
                "unlimited: self route");
      }
      else {
         Expect(sRoute.Reachable && sRoute.Hops == 1 && sRoute.RateMbps == 300,
                "unlimited: " + sRoute.Source + "->" + sRoute.Destination + " is one hop at 300");
      }
   }
}

void RouteTie() {
   /* Two-hop candidates through "a_far" (off-axis, longer links) and
    * "b_near" (on-axis). "a_far" comes first lexicographically; the greater
    * bottleneck through "b_near" must still win. */
   const SRoute sRoute = Route(Routes({{"base_station", CVector3(0, 0, 0)},
                                       {"a_far", CVector3(40, 10, 0)},
                                       {"b_near", CVector3(40, 0, 0)},
                                       {"dst", CVector3(80, 0, 0)}}),
                               "base_station", "dst");
   Expect(sRoute.Hops == 2, "route_tie: two hops");
   ExpectNear(sRoute.RateMbps, 30.8, TOLERANCE, "route_tie: greater bottleneck wins");
   /* Equal bottlenecks, mirrored relays: the same route from either order. */
   const SRoute sFirst = Route(Routes({{"base_station", CVector3(0, 0, 0)},
                                       {"m", CVector3(40, 10, 0)},
                                       {"n", CVector3(40, -10, 0)},
                                       {"dst", CVector3(80, 0, 0)}}),
                               "base_station", "dst");
   const SRoute sSecond = Route(Routes({{"base_station", CVector3(0, 0, 0)},
                                        {"n", CVector3(40, 10, 0)},
                                        {"m", CVector3(40, -10, 0)},
                                        {"dst", CVector3(80, 0, 0)}}),
                                "base_station", "dst");
   Expect(sFirst == sSecond, "route_tie: equal bottlenecks resolve the same either way");
   ExpectNear(sFirst.RateMbps, CMeshMedium::LinkRateMbps(std::sqrt(1700.0), 50) / 2, TOLERANCE,
              "route_tie: equal bottleneck rate");
}

void PermutationInvariance() {
   const std::vector<SNode> vecOriginal = {
      {"base_station", CVector3(-18, 0, 0.15)}, {"robot_0", CVector3(10, 0, 0)},
      {"robot_1", CVector3(45, 20, 3)}, {"robot_2", CVector3(70, 5, 0)},
      {"robot_3", CVector3(300, 0, 0)}, {"robot_4", CVector3(30, -30, 8)}};
   const std::vector<SNode> vecPermuted = {vecOriginal[4], vecOriginal[2], vecOriginal[5],
                                           vecOriginal[0], vecOriginal[3], vecOriginal[1]};
   const std::vector<SRoute> vecRoutes = Routes(vecOriginal);
   Expect(Routes(vecPermuted) == vecRoutes,
          "permutation_invariance: routes(permuted) == routes(original)");
   Expect(vecRoutes.size() == 36, "permutation_invariance: 6x6 ordered pairs");
   Expect(std::is_sorted(vecRoutes.begin(), vecRoutes.end(),
                         [](const SRoute& s_a, const SRoute& s_b) {
                            return s_a.Source != s_b.Source ? s_a.Source < s_b.Source
                                                            : s_a.Destination < s_b.Destination;
                         }),
          "permutation_invariance: sorted by source then destination");
}

void InvalidInput() {
   auto Throws = [](const std::vector<SNode>& vec_nodes, Real f_range, bool b_unlimited) {
      try {
         CMeshMedium::ComputeRoutes(vec_nodes, f_range, b_unlimited);
      }
      catch(CARGoSException&) {
         return true;
      }
      return false;
   };
   const std::vector<SNode> vecNodes = {{"a", CVector3(0, 0, 0)}, {"b", CVector3(1, 0, 0)}};
   Expect(Throws(vecNodes, 0, false), "invalid: zero range");
   Expect(Throws(vecNodes, -1, false), "invalid: negative range");
   Expect(Throws(vecNodes, std::nan(""), false), "invalid: nan range");
   Expect(!Throws(vecNodes, std::nan(""), true), "invalid: unlimited ignores the range");
   Expect(Throws({{"a", CVector3(0, 0, 0)}, {"a", CVector3(1, 0, 0)}}, 50, false),
          "invalid: duplicate id");
   Expect(Throws({{"a", CVector3(0, 0, 0)}, {"b", CVector3(std::nan(""), 0, 0)}}, 50, false),
          "invalid: non-finite coordinate");
}

}

int main() {
   LinkRate();
   MultiHop();
   LongChain();
   FewestHops();
   VerticalRange();
   Unlimited();
   RouteTie();
   PermutationInvariance();
   InvalidInput();
   if(g_nFailures > 0) {
      std::cerr << g_nFailures << " mesh route check(s) failed\n";
      return 1;
   }
   std::cout << "mesh routes follow geometry, hops and bottlenecks\n";
   return 0;
}
