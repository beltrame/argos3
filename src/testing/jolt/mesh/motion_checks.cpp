/* Sample real physics substeps, not only 10 Hz controller poses. */
#include "loop_functions.h"
#include <argos3/core/simulator/space/space.h>
#include <algorithm>
#include <cmath>
#include <limits>

void CMeshLoopFunctions::OnStep(const JPH::PhysicsStepListenerContext& c_context) {
   const JPH::Body* pcBody = c_context.mPhysicsSystem->GetBodyLockInterfaceNoLock().TryGetBody(
      m_pcMotionModel->GetBodies()[0].Id);
   SampleMotion(*pcBody);
   m_fPhysicsTime += c_context.mDeltaTime;
}

void CMeshLoopFunctions::SampleMotion(const JPH::Body& c_body) {
   const JPH::RVec3 cOrigin = c_body.GetPosition() - c_body.GetRotation() *
      m_pcMotionModel->GetBodies()[0].AnchorOffsetPosition;
   const CVector3 cPosition = ToARGoS(cOrigin);
   CRadians cYaw, cPitch, cRoll;
   ToARGoS(c_body.GetRotation()).ToEulerAngles(cYaw, cPitch, cRoll);
   const UInt32 unTick = GetSpace().GetSimulationClock();
   /* Sense/control runs after physics: a command issued on tick N first
    * reaches the body on tick N+1. Measure exactly the actuated hold interval. */
   if(m_unInterruptTick && unTick > m_unInterruptTick &&
      unTick <= m_unInterruptTick + m_unInterruptTicks) {
      if(!m_bHaveHoldStart) {
         m_bHaveHoldStart = true;
         m_cHoldStart = cPosition;
         m_cHoldYaw = cYaw;
      }
      m_fHoldHeightError = std::max(m_fHoldHeightError, std::abs(cPosition.GetZ() - m_cHoldStart.GetZ()));
      m_fHoldPlanarMotion = std::max(m_fHoldPlanarMotion,
         std::hypot(cPosition.GetX() - m_cHoldStart.GetX(), cPosition.GetY() - m_cHoldStart.GetY()));
      CRadians cTurn = cYaw - m_cHoldYaw;
      m_fHoldYaw = std::max(m_fHoldYaw, std::abs(cTurn.SignedNormalize().GetValue()));
   }
   if(m_unYawLockTick && unTick > m_unYawLockTick &&
      unTick <= m_unYawLockTick + m_unYawLockTicks)
      m_fPeakLockedYawRate = std::max(m_fPeakLockedYawRate, Real(std::abs(c_body.GetAngularVelocity().GetZ())));
   m_fPeakPitch = std::max(m_fPeakPitch, std::abs(cPitch.GetValue()) * 180.0 / M_PI);
   m_fPeakRoll = std::max(m_fPeakRoll, std::abs(cRoll.GetValue()) * 180.0 / M_PI);
   const float fUp = (c_body.GetRotation() * JPH::Vec3::sAxisZ()).GetZ();
   const Real fTilt = std::acos(std::clamp(double(fUp), -1.0, 1.0)) * 180.0 / M_PI;
   m_fPeakTilt = std::max(m_fPeakTilt, fTilt);
   m_fFinishTilt = fTilt;
   m_fPeakRise = std::max(m_fPeakRise, cPosition.GetZ() - m_cMotionStart.GetZ());
   const CVector3 cDelta = cPosition - m_cPreviousPosition;
   const Real fDt = m_pcMotionModel->GetJoltEngine().GetPhysicsClockTick();
   if(m_bDropAfterRetreat && unTick > m_unDropStartTick && m_fFallStart < 0 &&
      c_body.GetLinearVelocity().GetX() < -0.02f) {
      m_fRetreatDistance += std::max(Real(0), -cDelta.GetX());
      m_fPeakRetreatSpeed = std::max({m_fPeakRetreatSpeed,
         std::hypot(Real(c_body.GetLinearVelocity().GetX()), Real(c_body.GetLinearVelocity().GetY())),
         std::hypot(cDelta.GetX(), cDelta.GetY()) / fDt});
      m_fRetreatHeightError = std::max(m_fRetreatHeightError, std::abs(cPosition.GetZ() - 0.356));
      m_fRetreatYaw = std::max(m_fRetreatYaw, std::abs(cYaw.GetValue()));
   }
   /* Obstacle inserted at 4.5 s; expiry starts recovery at 34.5 s. Allow
    * one second to hit the block, then require a held stance until 44.5 s. */
   if(m_bBlockedRecovery && m_fPhysicsTime >= 35.5 && m_fPhysicsTime < 44.5) {
      if(!m_bHaveBlockedSample) {
         m_bHaveBlockedSample = true;
         m_cBlockedPosition = cPosition;
      }
      m_fBlockedMotion = std::max(m_fBlockedMotion, (cPosition - m_cBlockedPosition).Length());
      m_fBlockedHeightError = std::max(m_fBlockedHeightError, std::abs(cPosition.GetZ() - 0.356));
   }
   if(m_bBlockedRecovery && m_fPhysicsTime >= 35.5 &&
      m_fBlockedReleaseTime < 0 && cPosition.GetZ() < 0.354)
      m_fBlockedReleaseTime = m_fPhysicsTime;
   m_fTravel += cDelta.Length();
   m_cPreviousPosition = cPosition;
   const JPH::Vec3 cVelocity = c_body.GetPointVelocity(cOrigin);
   /* Pose differences also catch teleports, which body velocity alone misses.
    * Duplicate control-boundary samples have zero displacement. */
   if(!std::isfinite(cVelocity.Length()) || !std::isfinite(cPosition.Length()))
      m_fPeakSpeed = std::numeric_limits<Real>::infinity();
   m_fPeakSpeed = std::max({m_fPeakSpeed, Real(cVelocity.Length()), cDelta.Length() / fDt});
   m_fPeakUpSpeed = std::max({m_fPeakUpSpeed, Real(cVelocity.GetZ()), cDelta.GetZ() / fDt});
   if(m_bConeContactCheck && m_fPhysicsTime >= 2.0) {
      if(!m_bHaveConeSettleStart) {
         m_bHaveConeSettleStart = true;
         m_cConeSettleStart = cPosition;
      }
      m_fConeSettleMotion = std::max(m_fConeSettleMotion, (cPosition - m_cConeSettleStart).Length());
      m_fConeSettleSpeed = std::max({m_fConeSettleSpeed, Real(cVelocity.Length()), cDelta.Length() / fDt});
      m_fConeTiltMin = std::min(m_fConeTiltMin, fTilt);
      m_fConeTiltMax = std::max(m_fConeTiltMax, fTilt);
   }
   if(m_bStepCheck && m_fStepReachedTime < 0 &&
      c_body.GetWorldSpaceBounds().mMin.GetX() > 1.01f &&
      std::abs(cPosition.GetZ() - m_fExpectZ) < m_fPositionTolerance)
      m_fStepReachedTime = m_fPhysicsTime;
   m_fPeakHorizontalSpeed = std::max({m_fPeakHorizontalSpeed,
      std::hypot(Real(cVelocity.GetX()), Real(cVelocity.GetY())),
      std::hypot(cDelta.GetX(), cDelta.GetY()) / fDt});
   if(m_bDrop) {
      /* Ledge: wait for full clearance. Pause expiry: sample freefall just
       * after the specified release tick, including its actual initial vz. */
      const bool bReleased = m_bDropAfterRetreat ?
         (unTick > m_unDropStartTick && c_body.GetWorldSpaceBounds().mMax.GetX() < 0.999f &&
          c_body.GetLinearVelocity().GetZ() < -0.01f) :
         (m_unDropStartTick ? unTick > m_unDropStartTick :
          c_body.GetWorldSpaceBounds().mMin.GetX() > 1.001f);
      if(m_fFallStart < 0 && bReleased) {
         m_fFallStart = m_fPhysicsTime;
         m_fDropStartHeight = c_body.GetCenterOfMassPosition().GetZ();
         m_fDropStartVelocity = c_body.GetLinearVelocity().GetZ();
      }
      const Real fBottom = c_body.GetWorldSpaceBounds().mMin.GetZ();
      if(m_fFallStart >= 0 && m_fLandTime < 0 && fBottom < 0.005) {
         m_fLandTime = m_fPhysicsTime;
         m_fDropDistance = m_fDropStartHeight - c_body.GetCenterOfMassPosition().GetZ();
      }
      if(m_fLandTime >= 0) m_fRebound = std::max(m_fRebound, fBottom);
   }
}

void CMeshLoopFunctions::Destroy() {
   if(m_pcMotionModel) {
      m_pcMotionModel->GetJoltEngine().GetSystem().RemoveStepListener(this);
      m_pcMotionModel = nullptr;
   }
}
