/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_single_body_object_model.h"

namespace argos {

   /****************************************/
   /****************************************/

   CJoltSingleBodyObjectModel::CJoltSingleBodyObjectModel(CJoltEngine& c_engine,
                                                          CComposableEntity& c_entity) :
      CJoltModel(c_engine, c_entity) {
      m_vecBodies.reserve(1);
   }

   /****************************************/
   /****************************************/

   void CJoltSingleBodyObjectModel::MoveTo(const CVector3& c_position,
                                           const CQuaternion& c_orientation) {
      SBody& sBody = m_vecBodies[0];
      /* BodyPose = AnchorPose * Offset */
      JPH::Quat cRotation = ToJolt(c_orientation) * sBody.AnchorOffsetRotation;
      JPH::Quat cAnchorRotation = ToJolt(c_orientation);
      JPH::RVec3 cPosition =
         ToJolt(c_position) + cAnchorRotation * sBody.AnchorOffsetPosition;
      JPH::BodyInterface& cInterface = GetJoltEngine().GetBodyInterface();
      cInterface.SetPositionAndRotation(sBody.Id, cPosition, cRotation,
                                        JPH::EActivation::Activate);
      if(cInterface.GetMotionType(sBody.Id) != JPH::EMotionType::Static) {
         cInterface.SetLinearAndAngularVelocity(sBody.Id,
                                                JPH::Vec3::sZero(),
                                                JPH::Vec3::sZero());
      }
      /* Synchronize with the entity in the space */
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

}
