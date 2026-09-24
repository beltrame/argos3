/* Sample real physics substeps, not only 10 Hz controller poses. */
#include "loop_functions.h"
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
   m_fPeakPitch = std::max(m_fPeakPitch, std::abs(cPitch.GetValue()) * 180.0 / M_PI);
   m_fPeakRoll = std::max(m_fPeakRoll, std::abs(cRoll.GetValue()) * 180.0 / M_PI);
   const float fUp = (c_body.GetRotation() * JPH::Vec3::sAxisZ()).GetZ();
   m_fPeakTilt = std::max(m_fPeakTilt, std::acos(std::clamp(double(fUp), -1.0, 1.0)) * 180.0 / M_PI);
   m_fPeakRise = std::max(m_fPeakRise, cPosition.GetZ() - m_cMotionStart.GetZ());
   const CVector3 cDelta = cPosition - m_cPreviousPosition;
   const Real fDt = m_pcMotionModel->GetJoltEngine().GetPhysicsClockTick();
   m_fTravel += cDelta.Length();
   m_cPreviousPosition = cPosition;
   const JPH::Vec3 cVelocity = c_body.GetPointVelocity(cOrigin);
   /* Pose differences also catch teleports, which body velocity alone misses.
    * Duplicate control-boundary samples have zero displacement. */
   if(!std::isfinite(cVelocity.Length()) || !std::isfinite(cPosition.Length()))
      m_fPeakSpeed = std::numeric_limits<Real>::infinity();
   m_fPeakSpeed = std::max({m_fPeakSpeed, Real(cVelocity.Length()), cDelta.Length() / fDt});
   m_fPeakUpSpeed = std::max({m_fPeakUpSpeed, Real(cVelocity.GetZ()), cDelta.GetZ() / fDt});
   if(m_bStepCheck && m_fStepReachedTime < 0 &&
      c_body.GetWorldSpaceBounds().mMin.GetX() > 1.01f &&
      std::abs(cPosition.GetZ() - m_fExpectZ) < m_fPositionTolerance)
      m_fStepReachedTime = m_fPhysicsTime;
   m_fPeakHorizontalSpeed = std::max({m_fPeakHorizontalSpeed,
      std::hypot(Real(cVelocity.GetX()), Real(cVelocity.GetY())),
      std::hypot(cDelta.GetX(), cDelta.GetY()) / fDt});
   if(m_bDrop) {
      /* Ledge fixture ends at X=1: start the ballistic clock only after the
       * entire body clears it, not during supported rotation over its edge. */
      if(m_fFallStart < 0 && c_body.GetWorldSpaceBounds().mMin.GetX() > 1.001f) {
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
