/**
 * @file <argos3/testing/jolt/mesh/loop_functions.h>
 *
 * Shared loop functions of the Jolt mesh tests. ARGoS accepts one
 * <loop_functions> element per experiment, so the jobs the tests need live in
 * one class and are switched on independently from the XML.
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#ifndef JOLT_MESH_LOOP_FUNCTIONS_H
#define JOLT_MESH_LOOP_FUNCTIONS_H

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_model.h>
#include <Jolt/Physics/PhysicsStepListener.h>
#include <argos3/core/simulator/loop_functions.h>

#include <string>
#include <vector>

namespace argos {
   class CEmbodiedEntity;
}

using namespace argos;

/** Shared checks for the Jolt mesh experiments. */
class CMeshLoopFunctions : public CLoopFunctions, public JPH::PhysicsStepListener {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void PostExperiment();
   void Destroy() override;
   void OnStep(const JPH::PhysicsStepListenerContext& c_context) override;

private:

   struct SRayCheck {
      CVector3 Origin;
      CVector3 Direction;
      bool ExpectHit;
      Real Expected;
      std::string Label;
   };

   Real CastRay(const CVector3& c_origin,
                const CVector3& c_direction,
                Real f_length);

   void CheckRays();
   void CheckRobot();
   void CheckAttitude();
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

   /* Contact-driven terrain attitude checks */
   bool m_bCheckAttitude = false;
   Real m_fAttitudeSlope = 0.0;
   Real m_fAttitudeTolerance = 0.08;
   Real m_fMinimumRise = 0.15;
   Real m_fMinimumYaw = 0.1;
   UInt32 m_unAttitudeWarmup = 10;
   bool m_bHaveAttitudeStart = false;
   CVector3 m_cAttitudeStartPosition;

   /* Wall-contact and replay measurements, sampled each physics substep. */
   bool m_bMotionMetrics = false;
   Real m_fMaximumTilt = 180.0;
   Real m_fMaximumSpeed = 1.0e6;
   Real m_fMaximumRise = 1.0e6;
   Real m_fMaximumUpSpeed = 1.0e6;
   Real m_fPeakUpSpeed = 0.0;
   bool m_bStepCheck = false;
   Real m_fStepReachedTime = -1.0;
   bool m_bResetGroundCheck = false;
   CJoltModel* m_pcMotionModel = nullptr;
   CVector3 m_cInitialAngularVelocity;
   UInt32 m_unResetTick = 0;
   Real m_fPeakTilt = 0.0;
   Real m_fPeakPitch = 0.0;
   Real m_fPeakRoll = 0.0;
   Real m_fPeakRise = 0.0;
   Real m_fPeakSpeed = 0.0;
   Real m_fTravel = 0.0;
   Real m_fMinimumTravel = 0.0;
   CVector3 m_cMotionStart;
   CVector3 m_cPreviousPosition;
   void SampleMotion(const JPH::Body& c_body);
   /* Ballistic drop timing and rebound are independent of the speed ceiling. */
   bool m_bDrop = false;
   Real m_fPhysicsTime = 0.0;
   Real m_fFallStart = -1.0;
   Real m_fLandTime = -1.0;
   Real m_fRebound = 0.0;
   Real m_fPeakHorizontalSpeed = 0.0;
   Real m_fDropStartHeight = 0.0;
   Real m_fDropStartVelocity = 0.0;
   Real m_fDropDistance = 0.0;

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
