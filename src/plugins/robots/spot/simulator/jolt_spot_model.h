/**
 * @file <argos3/plugins/robots/spot/simulator/jolt_spot_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_SPOT_MODEL_H
#define JOLT_SPOT_MODEL_H

namespace argos {
   class CJoltSpotModel;
   class CSpotEntity;
   class CWheeledEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_ground_robot_model.h>

namespace argos {

   class CJoltSpotModel : public CJoltGroundRobotModel {

   public:

      CJoltSpotModel(CJoltEngine& c_engine,
                           CSpotEntity& c_entity);

      ~CJoltSpotModel() override;

      void Reset() override;

      /* Set contact-surface drive targets without overriding chassis motion. */
      virtual void UpdateFromEntityStatus() override;
      /* Velocity-limited leg actuation, only during a checked step. */
      void UpdatePhysics() override;

      SContactSurfaceVelocity GetContactSurfaceVelocity(
         const JPH::Body& c_body,
         JPH::Vec3Arg c_support_normal,
         JPH::RVec3Arg c_contact_offset,
         const JPH::ContactPoints& c_contact_points) const override;

      void UpdateAuxiliaryAnchor(SAnchor& s_anchor);

   private:

      enum class EStepPhase { NONE, LIFT, ADVANCE };
      void TryStartStep();
      bool HasStepSupport(JPH::RVec3Arg c_position, float f_reach) const;
      void EndStep();

      EStepPhase m_eStepPhase = EStepPhase::NONE;
      JPH::RVec3 m_cStepTarget = JPH::RVec3::sZero();
      JPH::Vec3 m_cStepDirection = JPH::Vec3::sZero();
      float m_fStepTimeLeft = 0.0f;
      float m_fStepCooldown = 0.0f;
      float m_fCommandLinear = 0.0f;
      float m_fCommandAngular = 0.0f;
      static constexpr float STEEP_CONTACT_NORMAL_Z = 0.76604444f; // cos(40 degrees)
      static constexpr float MAX_STEP_HEIGHT = 0.35f;
      static constexpr float MAX_LIFT_SPEED = 0.5f;

      JPH::Ref<JPH::TwoBodyConstraint> m_pcBalance;
      CSpotEntity& m_cSpotEntity;
      CWheeledEntity& m_cWheeledEntity;

      static const Real SPOT_LENGTH;
      static const Real SPOT_WIDTH;
      static const Real SPOT_HEIGHT;
      static const Real SPOT_MASS;
      static const Real SPOT_TRACK_GAUGE;

   };

}

#endif
