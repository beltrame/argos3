/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_MODEL_H
#define JOLT_MODEL_H

namespace argos {
   class CJoltEngine;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_common.h>

#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/physics_engines/jolt/jolt_engine.h>

#include <vector>

namespace argos {

   class CJoltModel : public CPhysicsModel {

   public:

      typedef std::map<std::string, CJoltModel*> TMap;

      /**
       * A Jolt body and its binding to an ARGoS anchor. The body pose
       * relates to the anchor pose through a constant local offset:
       * BodyPose = AnchorPose * Offset.
       */
      struct SBody {
         JPH::BodyID Id;
         SAnchor* Anchor = nullptr;
         JPH::Vec3 AnchorOffsetPosition = JPH::Vec3::sZero();
         JPH::Quat AnchorOffsetRotation = JPH::Quat::sIdentity();
         /* Pose at creation, restored on Reset() */
         JPH::RVec3 StartPosition = JPH::RVec3::sZero();
         JPH::Quat StartRotation = JPH::Quat::sIdentity();
      };

   public:

      CJoltModel(CJoltEngine& c_engine,
                 CComposableEntity& c_entity);

      virtual ~CJoltModel();

      virtual void Reset();

      virtual void UpdateEntityStatus();

      virtual void UpdateFromEntityStatus() {}

      virtual void CalculateBoundingBox();

      virtual bool IsCollidingWithSomething() const;

      inline CJoltEngine& GetJoltEngine() {
         return m_cJoltEngine;
      }

      inline const CJoltEngine& GetJoltEngine() const {
         return m_cJoltEngine;
      }

      inline CComposableEntity& GetComposableEntity() {
         return m_cComposableEntity;
      }

      inline std::vector<SBody>& GetBodies() {
         return m_vecBodies;
      }

   protected:

      /**
       * Creates a Jolt body bound to the given anchor and adds it to
       * the world. The user data is set to this model; the start pose
       * in c_settings is recorded for Reset().
       */
      JPH::BodyID CreateBody(JPH::BodyCreationSettings& c_settings,
                             SAnchor* ps_anchor,
                             const JPH::Vec3& c_anchor_offset_position,
                             const JPH::Quat& c_anchor_offset_rotation);

   protected:

      std::vector<SBody> m_vecBodies;

   private:

      CJoltEngine& m_cJoltEngine;
      CComposableEntity& m_cComposableEntity;

   };

}

#endif
