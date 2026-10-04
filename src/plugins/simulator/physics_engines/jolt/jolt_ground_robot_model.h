/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_ground_robot_model.h>
 *
 * Contact-driven approximation for single-body differential-drive robots.
 */
#ifndef JOLT_GROUND_ROBOT_MODEL_H
#define JOLT_GROUND_ROBOT_MODEL_H

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>

namespace argos {

   class CJoltGroundRobotModel : public CJoltSingleBodyObjectModel {
   public:
      CJoltGroundRobotModel(CJoltEngine& c_engine,
                            CComposableEntity& c_entity) :
         CJoltSingleBodyObjectModel(c_engine, c_entity) {}

      void Reset() override {
         m_fLinearVelocity = 0.0f;
         m_fAngularVelocity = 0.0f;
         CJoltSingleBodyObjectModel::Reset();
      }

      SContactSurfaceVelocity GetContactSurfaceVelocity(
         const JPH::Body& c_body,
         JPH::Vec3Arg c_support_normal,
         JPH::RVec3Arg,
         const JPH::ContactPoints&) const override {
         if(m_fLinearVelocity == 0.0f && m_fAngularVelocity == 0.0f) return {};
         const JPH::Quat cRotation = c_body.GetRotation();
         const JPH::Vec3 cUp = cRotation * JPH::Vec3::sAxisZ();
         /* Include rounded leading edges for wheel/track wall climbing.
          * This single-box approximation does not resolve individual wheels. */
         if(c_support_normal.Dot(cUp) <= 0.0f) return {};
         return {cRotation * JPH::Vec3(-m_fLinearVelocity, 0.0f, 0.0f),
                 cUp * -m_fAngularVelocity};
      }

   protected:
      void SetDriveVelocity(Real f_linear, Real f_angular) {
         /* The moving contact surface opposes desired chassis motion. Jolt
          * supplies traction bounded by friction and the contact normal load;
          * no velocity, gravity response, or roll/pitch state is overwritten. */
         m_fLinearVelocity = float(f_linear);
         m_fAngularVelocity = float(f_angular);
      }

   private:
      float m_fLinearVelocity = 0.0f;
      float m_fAngularVelocity = 0.0f;
   };

}

#endif
