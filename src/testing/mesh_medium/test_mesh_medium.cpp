/*
 * The mesh medium inside a real experiment.
 *
 *   test_mesh_medium <configuration.argos> limited|unlimited
 *
 * The experiment has two box robots, robot_0 at the origin and robot_1 40 m
 * away, and the base station 40 m behind robot_0. The medium must read its
 * attributes and follow the robots as they move.
 */

#include <argos3/plugins/simulator/media/mesh_medium.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/plugins/dynamic_loading.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace argos;

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

/* By value: the medium replaces its routes on every update. */
CMeshMedium::SRoute Route(const CMeshMedium& c_medium,
                          const std::string& str_source, const std::string& str_destination) {
   for(const CMeshMedium::SRoute& sRoute : c_medium.GetRoutes()) {
      if(sRoute.Source == str_source && sRoute.Destination == str_destination) return sRoute;
   }
   throw std::runtime_error("no route " + str_source + "->" + str_destination);
}

void MoveRobot1(CSimulator& c_simulator, Real f_x) {
   CEmbodiedEntity& cBody =
      dynamic_cast<CComposableEntity&>(c_simulator.GetSpace().GetEntity("robot_1"))
      .GetComponent<CEmbodiedEntity>("body");
   Expect(cBody.MoveTo(CVector3(f_x, 0, 0), cBody.GetOriginAnchor().Orientation),
          "robot_1 moves to x = " + std::to_string(f_x));
}

/* range_m="45", every other attribute at its default. */
void Limited(CSimulator& c_simulator, const CMeshMedium& c_medium) {
   Expect(c_medium.GetRange() == 45, "range_m is read");
   Expect(!c_medium.IsUnlimited() && c_medium.IsBandwidthCapped(), "defaults: limited and capped");
   Expect(c_medium.GetBaseStationId() == "base_station", "default base station id");

   Expect(c_medium.GetTick() == 0, "GetTick() is 0 after load");
   Expect(c_medium.GetRoutes().size() == 9, "three nodes give nine ordered pairs");
   Expect(Route(c_medium, "robot_0", "robot_1").Reachable, "40 m apart: reachable");
   const CMeshMedium::SRoute sRelayed = Route(c_medium, "base_station", "robot_1");
   Expect(sRelayed.Reachable && sRelayed.Hops == 2, "base -> robot_1 relays through robot_0");
   /* (300 - 298 * 40 / 45) / 2 */
   Expect(std::fabs(sRelayed.RateMbps - 17.5556) < TOLERANCE, "base -> robot_1 rate");

   c_simulator.UpdateSpace();
   Expect(c_medium.GetTick() == 1, "GetTick() follows the simulation clock");
   Expect(Route(c_medium, "robot_0", "robot_1").Reachable, "still 40 m apart after a tick");

   MoveRobot1(c_simulator, 60);
   Expect(Route(c_medium, "robot_0", "robot_1").Reachable,
          "GetRoutes() keeps the sampled tick until the medium updates");

   c_simulator.UpdateSpace();
   Expect(c_medium.GetTick() == 2, "GetTick() advances with the move");
   Expect(!Route(c_medium, "robot_0", "robot_1").Reachable,
          "60 m apart: unreachable on the next tick");
   Expect(!Route(c_medium, "base_station", "robot_1").Reachable,
          "base -> robot_1 lost its relay");
   Expect(Route(c_medium, "base_station", "robot_0").Reachable,
          "base -> robot_0 is still direct");

   c_simulator.Reset();
   Expect(c_medium.GetTick() == 0, "Reset() sets GetTick() to 0");
   Expect(Route(c_medium, "robot_0", "robot_1").Reachable,
          "Reset() recomputes routes from the reset positions");
   Expect(Route(c_medium, "base_station", "robot_1").Hops == 2, "Reset() restores the relay");
}

/* unlimited="true" cap_bandwidth="false" base_id="gateway" range_m="30" */
void Unlimited(CSimulator& c_simulator, const CMeshMedium& c_medium) {
   Expect(c_medium.GetRange() == 30, "range_m is read");
   Expect(c_medium.IsUnlimited() && !c_medium.IsBandwidthCapped(), "unlimited and uncapped");
   Expect(c_medium.GetBaseStationId() == "gateway", "base_id is read");

   Expect(c_medium.GetRoutes().size() == 9, "three nodes give nine ordered pairs");
   const CMeshMedium::SRoute sDirect = Route(c_medium, "gateway", "robot_1");
   Expect(sDirect.Reachable && sDirect.Hops == 1 && sDirect.RateMbps == 300,
          "gateway -> robot_1 is one hop at 300 Mbps, 80 m apart");

   MoveRobot1(c_simulator, 90);
   c_simulator.UpdateSpace();
   const CMeshMedium::SRoute sFar = Route(c_medium, "gateway", "robot_1");
   Expect(sFar.Reachable && sFar.Hops == 1 && sFar.RateMbps == 300,
          "gateway -> robot_1 is still one hop at 300 Mbps, 130 m apart");
}

}

int main(int argc, char** argv) {
   const std::string strMode = argc == 3 ? argv[2] : "";
   if(strMode != "limited" && strMode != "unlimited") {
      std::cerr << "usage: test_mesh_medium <configuration.argos> limited|unlimited\n";
      return 2;
   }
   CSimulator& cSimulator = CSimulator::GetInstance();
   try {
      CDynamicLoading::LoadAllLibraries();
      cSimulator.SetExperimentFileName(argv[1]);
      cSimulator.LoadExperiment();
      const auto& cMedium = cSimulator.GetMedium<CMeshMedium>("mesh");
      if(strMode == "limited") {
         Limited(cSimulator, cMedium);
      }
      else {
         Unlimited(cSimulator, cMedium);
      }
      cSimulator.Destroy();
   }
   catch(CARGoSException& ex) {
      std::cerr << "ARGoS error: " << ex.what() << "\n";
      ++g_nFailures;
   }
   catch(std::exception& ex) {
      std::cerr << "error: " << ex.what() << "\n";
      ++g_nFailures;
   }
   if(g_nFailures > 0) {
      std::cerr << g_nFailures << " mesh medium check(s) failed\n";
      return 1;
   }
   std::cout << "mesh medium (" << strMode << ") follows the robots\n";
   return 0;
}
