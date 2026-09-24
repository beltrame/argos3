/** Collision-checked, velocity-driven leg lift for Spot; never changes pose. */
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_common.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include "jolt_spot_model.h"
#include <algorithm>
#include <cmath>

namespace argos {

   bool CJoltSpotModel::HasStepSupport(JPH::RVec3Arg c_position, float f_reach) const {
      const auto& cSystem = GetJoltEngine().GetSystem();
      JPH::IgnoreSingleBodyFilter cFilter(m_vecBodies[0].Id);
      const JPH::RRayCast cRay(c_position, JPH::Vec3(0, 0, -f_reach));
      /* The simple ray overload includes backfaces: duplicated mesh faces can
       * then return a downward normal for a perfectly walkable floor. */
      JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> cHits;
      cSystem.GetNarrowPhaseQuery().CastRay(cRay, JPH::RayCastSettings(), cHits, {}, {}, cFilter);
      if(!cHits.HadHit()) return false;
      const auto& cHit = cHits.mHit;
      JPH::BodyLockRead cLock(cSystem.GetBodyLockInterface(), cHit.mBodyID);
      return cLock.Succeeded() && cLock.GetBody().IsStatic() &&
         cLock.GetBody().GetWorldSpaceSurfaceNormal(cHit.mSubShapeID2,
            cRay.GetPointOnRay(cHit.mFraction)).GetZ() >= 0.866f;
   }

   void CJoltSpotModel::TryStartStep() {
      /* Reverse commands release assistance; do not start a reverse step. */
      if(m_fCommandLinear < 0.001f) return;
      auto& cSystem = GetJoltEngine().GetSystem();
      auto& cBodies = cSystem.GetBodyInterface();
      const JPH::BodyID cId = m_vecBodies[0].Id;
      if(std::abs(cBodies.GetLinearVelocity(cId).GetZ()) > 0.3f) return;
      JPH::RVec3 cPosition;
      JPH::Quat cRotation;
      cBodies.GetPositionAndRotation(cId, cPosition, cRotation);
      if(!HasStepSupport(cPosition, float(SPOT_HEIGHT) * 0.5f + 0.025f)) return;
      JPH::Vec3 cDirection = cRotation * JPH::Vec3::sAxisX();
      cDirection.SetZ(0);
      cDirection = cDirection.Normalized();
      const auto pcShape = cBodies.GetShape(cId);
      JPH::IgnoreSingleBodyFilter cFilter(cId);
      auto Cast = [&](JPH::RVec3Arg cStart, JPH::Vec3Arg cDelta) {
         JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> cHit;
         const JPH::RShapeCast cSweep(pcShape, JPH::Vec3::sReplicate(1.0f),
            JPH::RMat44::sRotationTranslation(cRotation, cStart), cDelta);
         JPH::ShapeCastSettings cSettings; // Full, unshrunken body for clearance.
         cSystem.GetNarrowPhaseQuery().CastShape(cSweep, cSettings,
            JPH::RVec3::sZero(), cHit, {}, {}, cFilter);
         return cHit;
      };
      const JPH::Vec3 cSkin(0, 0, 0.006f);
      const auto cObstacle = Cast(cPosition + cSkin, cDirection * 0.10f);
      /* Ordinary slopes already have contact traction. A horizontal sweep
       * intersects an uphill ramp even when there is no step to climb. */
      if(!cObstacle.HadHit() ||
         (-cObstacle.mHit.mPenetrationAxis.Normalized()).GetZ() >= STEEP_CONTACT_NORMAL_Z) return;
      const JPH::Vec3 cUp(0, 0, MAX_STEP_HEIGHT + 0.012f);
      if(Cast(cPosition + cSkin, cUp).HadHit()) return;
      const JPH::RVec3 cRaised = cPosition + cSkin + cUp;
      /* Check a complete stance ahead, not just a toe on a thin lip. This also
       * rejects a lip backed by a wall before any lift is started. */
      const JPH::Vec3 cAdvance = cDirection * (float(SPOT_LENGTH) + 0.05f);
      if(Cast(cRaised, cAdvance).HadHit()) return;
      const auto cLanding = Cast(cRaised + cAdvance, -(cUp + cSkin));
      if(!cLanding.HadHit() ||
         cBodies.GetMotionType(cLanding.mHit.mBodyID2) != JPH::EMotionType::Static ||
         (-cLanding.mHit.mPenetrationAxis.Normalized()).GetZ() < 0.866f) return;
      const JPH::RVec3 cTarget = cRaised + cAdvance -
         (cUp + cSkin) * cLanding.mHit.mFraction + cSkin;
      const float fRise = float(cTarget.GetZ() - cPosition.GetZ());
      if(fRise < 0.015f || fRise > MAX_STEP_HEIGHT + 0.008f ||
         !HasStepSupport(cTarget, float(SPOT_HEIGHT) * 0.5f + 0.025f)) return;
      m_cStepTarget = cTarget;
      m_cStepDirection = cDirection;
      m_fStepTimeLeft = cAdvance.Length() / std::abs(m_fCommandLinear) +
         fRise / MAX_LIFT_SPEED + 2.0f;
      m_fStepPauseTimeLeft = STEP_PAUSE_ALLOWANCE;
      m_eStepPhase = EStepPhase::LIFT;
   }

   void CJoltSpotModel::EndStep() {
      m_eStepPhase = EStepPhase::NONE;
      m_bStepPaused = false;
      m_fStepPauseTimeLeft = 0.0f;
      m_fStepCooldown = 0.2f;
      SetDriveVelocity(m_fCommandLinear, m_fCommandAngular);
   }

   void CJoltSpotModel::UpdatePhysics() {
      const float fDt = float(GetJoltEngine().GetPhysicsClockTick());
      m_fStepCooldown = std::max(0.0f, m_fStepCooldown - fDt);
      if(m_eStepPhase == EStepPhase::NONE) return;
      auto& cBodies = GetJoltEngine().GetBodyInterface();
      const JPH::BodyID cId = m_vecBodies[0].Id;
      const JPH::RVec3 cPosition = cBodies.GetPosition(cId);
      JPH::Vec3 cForward = cBodies.GetRotation(cId) * JPH::Vec3::sAxisX();
      cForward.SetZ(0.0f);
      /* Compare planar heading, not body tilt, with the checked corridor. */
      const bool bAligned = cForward.LengthSq() > 1.0e-8f &&
         cForward.Normalized().Dot(m_cStepDirection) >= STEP_ALIGNMENT_COS;
      const bool bPause = m_fCommandLinear < 0.001f || !bAligned;
      /* Turning through 90 degrees is not reversal: only an explicitly
       * negative forward command, absent support, or either timeout aborts. */
      if(m_fStepTimeLeft <= 0 || m_fStepPauseTimeLeft <= 0 || m_fCommandLinear < -0.001f ||
         !HasStepSupport(cPosition, float(SPOT_HEIGHT) * 0.5f + MAX_STEP_HEIGHT + 0.025f)) {
         EndStep();
         return;
      }
      /* Charge the interval being actuated, releasing on the next substep
       * after expiry. Pauses share a cumulative budget, never replenished by
       * realignment, and do not consume the active step time budget. */
      if(bPause) m_fStepPauseTimeLeft -= fDt;
      else m_fStepTimeLeft -= fDt;
      if(bPause && !m_bStepPaused) m_fStepHoldHeight = float(cPosition.GetZ());
      /* Paused stances may yaw. Active lift/advance is heading-locked even
       * if the resumed command still requests yaw. Preserve roll/pitch. */
      JPH::Vec3 cAngular = cBodies.GetAngularVelocity(cId);
      cAngular.SetZ(bPause ? std::clamp(m_fCommandAngular, -MAX_STEP_YAW_RATE, MAX_STEP_YAW_RATE) : 0.0f);
      cBodies.SetAngularVelocity(cId, cAngular);
      m_bStepPaused = bPause;
      const float fHeightError = float((bPause ? m_fStepHoldHeight : m_cStepTarget.GetZ()) - cPosition.GetZ());
      if(!bPause && m_eStepPhase == EStepPhase::LIFT && fHeightError < 0.002f)
         m_eStepPhase = EStepPhase::ADVANCE;
      const float fRemaining = JPH::Vec3(m_cStepTarget - cPosition).Dot(m_cStepDirection);
      if(!bPause && m_eStepPhase == EStepPhase::ADVANCE && fRemaining < 0.003f) {
         EndStep();
         return;
      }
      const bool bAdvance = !bPause && m_eStepPhase == EStepPhase::ADVANCE;
      const float fSpeed = bAdvance ? std::min(std::abs(m_fCommandLinear), fRemaining / fDt) : 0.0f;
      const JPH::Vec3 cPlanar = m_cStepDirection * fSpeed;
      /* A bounded leg actuator counters gravity only while static support is
       * within leg reach. Physics still integrates every pose and resolves all
       * contacts; neither position nor orientation is ever assigned here. */
      const float fGravity = GetJoltEngine().GetSystem().GetGravity().GetZ();
      const float fLift = std::clamp(10.0f * fHeightError - fGravity * fDt,
                                    -MAX_LIFT_SPEED, MAX_LIFT_SPEED);
      SetDriveVelocity(bAdvance ? m_fCommandLinear : 0.0f, 0.0f);
      cBodies.SetLinearVelocity(cId, JPH::Vec3(cPlanar.GetX(), cPlanar.GetY(), fLift));
   }
}
