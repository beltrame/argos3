/**
 * @file <argos3/testing/jolt/mesh/loop_functions.h>
 *
 * Shared loop functions of the Jolt mesh tests. ARGoS accepts one
 * <loop_functions> element per experiment, so the three jobs the tests
 * need live in one class and are switched on independently from the XML.
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#ifndef JOLT_MESH_LOOP_FUNCTIONS_H
#define JOLT_MESH_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

#include <string>
#include <vector>

namespace argos {
   class CEmbodiedEntity;
}

using namespace argos;

/**
 * Checks the Jolt mesh entity from three angles, each optional:
 *
 * - every <ray> child is cast once, on the first step, and the returned
 *   distance is compared with a closed-form value derived from the asset;
 *   expect="none" asserts that the ray leaves the world instead;
 * - with robot="<id>", the pose of that robot is compared at the end of
 *   the experiment with an expected position, with a model of the surface
 *   it should be resting on, or both, and its lateral deviation from the
 *   X axis is tracked over the whole run;
 * - with scan="true", one full lidar-class scan is cast per step and
 *   timed, and the mean time per ray is reported at the end.
 *
 * Attributes:
 *   ray_length          length of the analytic rays (default 100 m)
 *   ray_tolerance       allowed error on an analytic ray (default 1e-3 m)
 *   robot               id of the robot whose pose is checked
 *   position_tolerance  allowed error on the robot position (default 1e-2 m)
 *   expect_x            expected final X of the robot
 *   expect_z            expected final Z of the robot
 *   slope               slope of the surface the robot rests on, rise per
 *                       unit X; the expected Z is then derived from the
 *                       final X instead of being given by expect_z
 *   slope_origin        X at which that surface has Z = 0
 *   lateral_tolerance   largest |Y| the robot may reach during the run
 *   poses_file          file the final pose is written to, in the exact
 *                       %a format, for a determinism comparison
 *   scan                "true" to run the ray throughput measurement
 *   rings               beams in elevation per scan (default 16)
 *   azimuth_steps       beams per revolution (default 900)
 *   elevation_min       lowest beam elevation in degrees (default -15)
 *   elevation_max       highest beam elevation in degrees (default 15)
 *   scan_range          length of the scan rays (default 100 m)
 *   scan_height         height of the scan origin (default 0.75 m)
 *   scan_radius         radius of the circle the scan origin walks
 *                       (default 3 m)
 *   warmup_steps        steps discarded before the timing starts
 *                       (default 2)
 */
class CMeshLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void PostExperiment();

private:

   struct SRayCheck {
      CVector3 Origin;
      CVector3 Direction;
      bool ExpectHit;
      Real Expected;
      std::string Label;
   };

   /** Distance to the closest hit, or a negative value when there is none */
   Real CastRay(const CVector3& c_origin,
                const CVector3& c_direction,
                Real f_length);

   void CheckRays();
   void CheckRobot();
   void WritePoses();
   void RunScan();
   void ReportScan();

   /* Analytic ray checks */
   std::vector<SRayCheck> m_vecRayChecks;
   Real m_fRayLength = 100.0;
   Real m_fRayTolerance = 1.0e-3;

   /* Robot pose checks */
   CEmbodiedEntity* m_pcRobot = nullptr;
   std::string m_strRobot;
   Real m_fPositionTolerance = 1.0e-2;
   bool m_bCheckX = false;
   Real m_fExpectX = 0.0;
   bool m_bCheckZ = false;
   Real m_fExpectZ = 0.0;
   bool m_bCheckSlope = false;
   Real m_fSlope = 0.0;
   Real m_fSlopeOrigin = 0.0;
   bool m_bCheckLateral = false;
   Real m_fLateralTolerance = 0.0;
   Real m_fMaxLateral = 0.0;
   std::string m_strPosesFile;

   /* Ray throughput measurement */
   bool m_bScan = false;
   UInt32 m_unRings = 16;
   UInt32 m_unAzimuthSteps = 900;
   Real m_fElevationMin = -15.0;
   Real m_fElevationMax = 15.0;
   Real m_fScanRange = 100.0;
   Real m_fScanHeight = 0.75;
   Real m_fScanRadius = 3.0;
   UInt32 m_unWarmupSteps = 2;
   std::vector<CVector3> m_vecScanDirections;
   double m_fScanNanoseconds = 0.0;
   size_t m_unScanRays = 0;
   size_t m_unScanHits = 0;

};

#endif
