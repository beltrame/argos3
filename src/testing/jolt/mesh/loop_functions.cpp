/**
 * @file <argos3/testing/jolt/mesh/loop_functions.cpp>
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#include "loop_functions.h"

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/ray3.h>

#include <chrono>
#include <cmath>
#include <cstdio>

/****************************************/
/****************************************/

/* Body radius of the foot-bot, from jolt_footbot_model.cpp. A robot
 * stopped by a vertical surface comes to rest one radius short of it. */
static const Real FOOTBOT_RADIUS = 0.085036758;

/****************************************/
/****************************************/

void CMeshLoopFunctions::Init(TConfigurationNode& t_tree) {
   GetNodeAttributeOrDefault(t_tree, "ray_length", m_fRayLength, m_fRayLength);
   GetNodeAttributeOrDefault(t_tree, "ray_tolerance", m_fRayTolerance,
                             m_fRayTolerance);
   /* Analytic ray checks */
   TConfigurationNodeIterator itRay("ray");
   for(itRay = itRay.begin(&t_tree); itRay != itRay.end(); ++itRay) {
      SRayCheck sCheck;
      GetNodeAttribute(*itRay, "label", sCheck.Label);
      GetNodeAttribute(*itRay, "origin", sCheck.Origin);
      GetNodeAttribute(*itRay, "direction", sCheck.Direction);
      sCheck.Direction.Normalize();
      std::string strExpected;
      GetNodeAttribute(*itRay, "expect", strExpected);
      sCheck.ExpectHit = (strExpected != "none");
      sCheck.Expected = sCheck.ExpectHit ? std::stod(strExpected) : 0.0;
      m_vecRayChecks.push_back(sCheck);
   }
   /* Robot pose checks */
   if(NodeAttributeExists(t_tree, "robot")) {
      GetNodeAttribute(t_tree, "robot", m_strRobot);
      m_pcRobot = &dynamic_cast<CComposableEntity&>(
         GetSpace().GetEntity(m_strRobot)).GetComponent<CEmbodiedEntity>("body");
      GetNodeAttributeOrDefault(t_tree, "position_tolerance",
                                m_fPositionTolerance, m_fPositionTolerance);
      m_bCheckX = NodeAttributeExists(t_tree, "expect_x");
      if(m_bCheckX) {
         GetNodeAttribute(t_tree, "expect_x", m_fExpectX);
      }
      m_bCheckZ = NodeAttributeExists(t_tree, "expect_z");
      if(m_bCheckZ) {
         GetNodeAttribute(t_tree, "expect_z", m_fExpectZ);
      }
      m_bCheckSlope = NodeAttributeExists(t_tree, "slope");
      if(m_bCheckSlope) {
         GetNodeAttribute(t_tree, "slope", m_fSlope);
         GetNodeAttribute(t_tree, "slope_origin", m_fSlopeOrigin);
      }
      m_bCheckLateral = NodeAttributeExists(t_tree, "lateral_tolerance");
      if(m_bCheckLateral) {
         GetNodeAttribute(t_tree, "lateral_tolerance", m_fLateralTolerance);
      }
      GetNodeAttributeOrDefault(t_tree, "poses_file", m_strPosesFile,
                                m_strPosesFile);
   }
   /* Ray throughput measurement */
   GetNodeAttributeOrDefault(t_tree, "scan", m_bScan, m_bScan);
   if(m_bScan) {
      GetNodeAttributeOrDefault(t_tree, "rings", m_unRings, m_unRings);
      GetNodeAttributeOrDefault(t_tree, "azimuth_steps", m_unAzimuthSteps,
                                m_unAzimuthSteps);
      GetNodeAttributeOrDefault(t_tree, "elevation_min", m_fElevationMin,
                                m_fElevationMin);
      GetNodeAttributeOrDefault(t_tree, "elevation_max", m_fElevationMax,
                                m_fElevationMax);
      GetNodeAttributeOrDefault(t_tree, "scan_range", m_fScanRange,
                                m_fScanRange);
      GetNodeAttributeOrDefault(t_tree, "scan_height", m_fScanHeight,
                                m_fScanHeight);
      GetNodeAttributeOrDefault(t_tree, "scan_radius", m_fScanRadius,
                                m_fScanRadius);
      GetNodeAttributeOrDefault(t_tree, "warmup_steps", m_unWarmupSteps,
                                m_unWarmupSteps);
      m_vecScanDirections.reserve(m_unRings * m_unAzimuthSteps);
      for(UInt32 i = 0; i < m_unRings; ++i) {
         Real fElevation = (m_unRings == 1) ?
            m_fElevationMin :
            m_fElevationMin +
            (m_fElevationMax - m_fElevationMin) * i / Real(m_unRings - 1);
         fElevation *= M_PI / 180.0;
         for(UInt32 j = 0; j < m_unAzimuthSteps; ++j) {
            Real fAzimuth = 2.0 * M_PI * j / Real(m_unAzimuthSteps);
            m_vecScanDirections.push_back(
               CVector3(std::cos(fElevation) * std::cos(fAzimuth),
                        std::cos(fElevation) * std::sin(fAzimuth),
                        std::sin(fElevation)));
         }
      }
   }
}

/****************************************/
/****************************************/

Real CMeshLoopFunctions::CastRay(const CVector3& c_origin,
                                 const CVector3& c_direction,
                                 Real f_length) {
   CRay3 cRay(c_origin, c_direction, f_length);
   SEmbodiedEntityIntersectionItem sItem;
   if(!GetClosestEmbodiedEntityIntersectedByRay(sItem, cRay)) {
      return -1.0;
   }
   return sItem.TOnRay * f_length;
}

/****************************************/
/****************************************/

void CMeshLoopFunctions::CheckRays() {
   char pchLine[256];
   UInt32 unFailures = 0;
   Real fWorstError = 0.0;
   std::string strWorstLabel;
   for(const SRayCheck& sCheck : m_vecRayChecks) {
      Real fMeasured = CastRay(sCheck.Origin, sCheck.Direction, m_fRayLength);
      bool bHit = (fMeasured >= 0.0);
      Real fError = 0.0;
      bool bPass;
      if(sCheck.ExpectHit) {
         fError = bHit ? (fMeasured - sCheck.Expected) : 0.0;
         bPass = bHit && (std::fabs(fError) <= m_fRayTolerance);
      }
      else {
         bPass = !bHit;
      }
      if(!bPass) {
         ++unFailures;
      }
      if(std::fabs(fError) > std::fabs(fWorstError)) {
         fWorstError = fError;
         strWorstLabel = sCheck.Label;
      }
      std::snprintf(pchLine, sizeof(pchLine),
                    "[mesh] ray %-16s expected %-14s measured %-14s "
                    "error %+.3e %s",
                    sCheck.Label.c_str(),
                    sCheck.ExpectHit ?
                       std::to_string(double(sCheck.Expected)).c_str() : "none",
                    bHit ?
                       std::to_string(double(fMeasured)).c_str() : "none",
                    double(fError), bPass ? "pass" : "FAIL");
      LOG << pchLine << std::endl;
   }
   std::snprintf(pchLine, sizeof(pchLine),
                 "[mesh] ray checks %zu, failures %u, worst error %.3e m "
                 "on \"%s\", tolerance %.1e m",
                 m_vecRayChecks.size(), unFailures, double(fWorstError),
                 strWorstLabel.c_str(), double(m_fRayTolerance));
   LOG << pchLine << std::endl;
   LOG.Flush();
   if(unFailures > 0) {
      THROW_ARGOSEXCEPTION(unFailures << " of " << m_vecRayChecks.size()
                           << " ray checks are outside the tolerance of "
                           << m_fRayTolerance << " m, the worst by "
                           << fWorstError << " m on \"" << strWorstLabel
                           << "\"");
   }
}

/****************************************/
/****************************************/

void CMeshLoopFunctions::CheckRobot() {
   const CVector3& cPosition = m_pcRobot->GetOriginAnchor().Position;
   char pchLine[256];
   std::snprintf(pchLine, sizeof(pchLine),
                 "[mesh] robot %s final position %.6f, %.6f, %.6f, "
                 "largest |Y| %.3e m",
                 m_strRobot.c_str(), double(cPosition.GetX()),
                 double(cPosition.GetY()), double(cPosition.GetZ()),
                 double(m_fMaxLateral));
   LOG << pchLine << std::endl;
   if(m_bCheckX) {
      Real fError = cPosition.GetX() - m_fExpectX;
      LOG << "[mesh] X error " << fError << " m, tolerance "
          << m_fPositionTolerance << " m" << std::endl;
      if(std::fabs(fError) > m_fPositionTolerance) {
         THROW_ARGOSEXCEPTION("Robot \"" << m_strRobot << "\" stopped at x = "
                              << cPosition.GetX() << ", expected "
                              << m_fExpectX << " within "
                              << m_fPositionTolerance << " m");
      }
   }
   if(m_bCheckZ) {
      Real fError = cPosition.GetZ() - m_fExpectZ;
      LOG << "[mesh] Z error " << fError << " m, tolerance "
          << m_fPositionTolerance << " m" << std::endl;
      if(std::fabs(fError) > m_fPositionTolerance) {
         THROW_ARGOSEXCEPTION("Robot \"" << m_strRobot << "\" is at z = "
                              << cPosition.GetZ() << ", expected "
                              << m_fExpectZ << " within "
                              << m_fPositionTolerance
                              << " m: it is not supported by the mesh");
      }
   }
   if(m_bCheckSlope) {
      /* The robot's Jolt body is a cylinder that cannot pitch, so its flat
       * bottom face touches a plane of slope s at its uphill edge and its
       * origin anchor sits s * radius above the surface height under its
       * centre. */
      Real fExpectedZ = m_fSlope * (cPosition.GetX() - m_fSlopeOrigin) +
                        m_fSlope * FOOTBOT_RADIUS;
      Real fError = cPosition.GetZ() - fExpectedZ;
      LOG << "[mesh] Z on the sloped surface " << cPosition.GetZ()
          << " m, expected " << fExpectedZ << " m, error " << fError
          << " m, tolerance " << m_fPositionTolerance << " m" << std::endl;
      if(std::fabs(fError) > m_fPositionTolerance) {
         THROW_ARGOSEXCEPTION("Robot \"" << m_strRobot << "\" is at z = "
                              << cPosition.GetZ() << " over x = "
                              << cPosition.GetX() << ", but the sloped "
                              "surface is at z = " << fExpectedZ
                              << " there, within " << m_fPositionTolerance
                              << " m");
      }
   }
   if(m_bCheckLateral && m_fMaxLateral > m_fLateralTolerance) {
      THROW_ARGOSEXCEPTION("Robot \"" << m_strRobot << "\" deviated "
                           << m_fMaxLateral << " m sideways while driving "
                           "straight, tolerance " << m_fLateralTolerance
                           << " m: contact normals on the mesh surface are "
                           "wrong");
   }
}

/****************************************/
/****************************************/

void CMeshLoopFunctions::WritePoses() {
   std::FILE* ptFile = std::fopen(m_strPosesFile.c_str(), "w");
   if(ptFile == nullptr) {
      THROW_ARGOSEXCEPTION("Cannot write \"" << m_strPosesFile << "\"");
   }
   const CVector3& cPosition = m_pcRobot->GetOriginAnchor().Position;
   const CQuaternion& cOrientation = m_pcRobot->GetOriginAnchor().Orientation;
   std::fprintf(ptFile, "%s %a %a %a %a %a %a %a\n", m_strRobot.c_str(),
                cPosition.GetX(), cPosition.GetY(), cPosition.GetZ(),
                cOrientation.GetW(), cOrientation.GetX(),
                cOrientation.GetY(), cOrientation.GetZ());
   std::fclose(ptFile);
}

/****************************************/
/****************************************/

void CMeshLoopFunctions::RunScan() {
   Real fAngle = 2.0 * M_PI * (GetSpace().GetSimulationClock() % 64) / 64.0;
   CVector3 cOrigin(m_fScanRadius * std::cos(fAngle),
                    m_fScanRadius * std::sin(fAngle),
                    m_fScanHeight);
   const size_t unCount = m_vecScanDirections.size();
   /* Start the sweep at a different beam every step, so that no two steps
    * repeat the same query sequence */
   const size_t unOffset =
      size_t(GetSpace().GetSimulationClock() * 37) % unCount;
   size_t unHits = 0;
   auto tStart = std::chrono::steady_clock::now();
   for(size_t i = 0; i < unCount; ++i) {
      if(CastRay(cOrigin, m_vecScanDirections[(i + unOffset) % unCount],
                 m_fScanRange) >= 0.0) {
         ++unHits;
      }
   }
   auto tEnd = std::chrono::steady_clock::now();
   if(GetSpace().GetSimulationClock() > m_unWarmupSteps) {
      m_fScanNanoseconds +=
         std::chrono::duration<double, std::nano>(tEnd - tStart).count();
      m_unScanRays += unCount;
      m_unScanHits += unHits;
   }
}

/****************************************/
/****************************************/

void CMeshLoopFunctions::ReportScan() {
   if(m_unScanRays == 0) {
      THROW_ARGOSEXCEPTION("No ray was timed: the experiment is shorter than "
                           "the warm-up of " << m_unWarmupSteps << " steps");
   }
   char pchLine[256];
   double fMicrosecondsPerRay =
      m_fScanNanoseconds / double(m_unScanRays) / 1000.0;
   std::snprintf(pchLine, sizeof(pchLine),
                 "[mesh] %zu rays timed, %.3f hit, %.4f us/ray, %.0f rays/s",
                 m_unScanRays, double(m_unScanHits) / double(m_unScanRays),
                 fMicrosecondsPerRay, 1.0e6 / fMicrosecondsPerRay);
   LOG << pchLine << std::endl;
}

/****************************************/
/****************************************/

void CMeshLoopFunctions::PostStep() {
   /* The mesh never moves, so one pass over the analytic rays is enough.
    * It is done early, while the robots are still where the arena put
    * them and cannot stand in the way of a beam. */
   if(GetSpace().GetSimulationClock() == 1 && !m_vecRayChecks.empty()) {
      CheckRays();
   }
   if(m_pcRobot != nullptr) {
      /* Every scenario starts the robot on the X axis and drives it along
       * it, so |Y| is its lateral deviation */
      m_fMaxLateral = std::max(
         m_fMaxLateral,
         Real(std::fabs(m_pcRobot->GetOriginAnchor().Position.GetY())));
   }
   if(m_bScan) {
      RunScan();
   }
}

/****************************************/
/****************************************/

void CMeshLoopFunctions::PostExperiment() {
   if(m_pcRobot != nullptr) {
      if(!m_strPosesFile.empty()) {
         WritePoses();
      }
      CheckRobot();
   }
   if(m_bScan) {
      ReportScan();
   }
   LOG.Flush();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMeshLoopFunctions, "mesh_loop_functions");
