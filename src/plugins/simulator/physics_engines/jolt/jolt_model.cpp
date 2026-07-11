/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "jolt_model.h"

namespace argos {

   /****************************************/
   /****************************************/

   CJoltModel::CJoltModel(CJoltEngine& c_engine,
                          CComposableEntity& c_entity) :
      CPhysicsModel(c_engine, c_entity.GetComponent<CEmbodiedEntity>("body")),
      m_cJoltEngine(c_engine),
      m_cComposableEntity(c_entity) {}

   /****************************************/
   /****************************************/

   CJoltModel::~CJoltModel() {
      JPH::BodyInterface& cInterface = m_cJoltEngine.GetBodyInterface();
      for(SBody& s_body : m_vecBodies) {
         cInterface.RemoveBody(s_body.Id);
         cInterface.DestroyBody(s_body.Id);
      }
   }

   /****************************************/
   /****************************************/

   JPH::BodyID CJoltModel::CreateBody(JPH::BodyCreationSettings& c_settings,
                                      SAnchor* ps_anchor,
                                      const JPH::Vec3& c_anchor_offset_position,
                                      const JPH::Quat& c_anchor_offset_rotation) {
      c_settings.mUserData = reinterpret_cast<JPH::uint64>(this);
      JPH::BodyInterface& cInterface = m_cJoltEngine.GetBodyInterface();
      JPH::BodyID cId = cInterface.CreateAndAddBody(
         c_settings,
         c_settings.mMotionType == JPH::EMotionType::Static ?
            JPH::EActivation::DontActivate : JPH::EActivation::Activate);
      if(cId.IsInvalid()) {
         THROW_ARGOSEXCEPTION("Could not create a Jolt body for entity \""
                              << m_cComposableEntity.GetId()
                              << "\"; body limit reached?");
      }
      SBody sBody;
      sBody.Id = cId;
      sBody.Anchor = ps_anchor;
      sBody.AnchorOffsetPosition = c_anchor_offset_position;
      sBody.AnchorOffsetRotation = c_anchor_offset_rotation;
      sBody.StartPosition = c_settings.mPosition;
      sBody.StartRotation = c_settings.mRotation;
      m_vecBodies.push_back(sBody);
      return cId;
   }

   /****************************************/
   /****************************************/

   void CJoltModel::Reset() {
      JPH::BodyInterface& cInterface = m_cJoltEngine.GetBodyInterface();
      for(SBody& s_body : m_vecBodies) {
         cInterface.SetPositionAndRotation(s_body.Id,
                                           s_body.StartPosition,
                                           s_body.StartRotation,
                                           JPH::EActivation::Activate);
         if(cInterface.GetMotionType(s_body.Id) != JPH::EMotionType::Static) {
            cInterface.SetLinearAndAngularVelocity(s_body.Id,
                                                   JPH::Vec3::sZero(),
                                                   JPH::Vec3::sZero());
         }
      }
      UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CJoltModel::UpdateEntityStatus() {
      JPH::BodyInterface& cInterface = m_cJoltEngine.GetBodyInterface();
      for(SBody& s_body : m_vecBodies) {
         if(s_body.Anchor != nullptr) {
            JPH::RVec3 cPosition;
            JPH::Quat cRotation;
            cInterface.GetPositionAndRotation(s_body.Id, cPosition, cRotation);
            /* AnchorPose = BodyPose * Offset^-1 */
            JPH::Quat cAnchorRotation =
               cRotation * s_body.AnchorOffsetRotation.Conjugated();
            JPH::Vec3 cAnchorPosition =
               JPH::Vec3(cPosition) -
               cAnchorRotation * s_body.AnchorOffsetPosition;
            s_body.Anchor->Position = ToARGoS(cAnchorPosition);
            s_body.Anchor->Orientation = ToARGoS(cAnchorRotation);
         }
      }
      /* Updates the AABB and the entity components */
      CPhysicsModel::UpdateEntityStatus();
   }

   /****************************************/
   /****************************************/

   void CJoltModel::CalculateBoundingBox() {
      if(m_vecBodies.empty()) {
         return;
      }
      const JPH::BodyInterface& cInterface =
         m_cJoltEngine.GetSystem().GetBodyInterfaceNoLock();
      JPH::AABox cBounds;
      for(const SBody& s_body : m_vecBodies) {
         cBounds.Encapsulate(
            cInterface.GetTransformedShape(s_body.Id).GetWorldSpaceBounds());
      }
      GetBoundingBox().MinCorner = ToARGoS(cBounds.mMin);
      GetBoundingBox().MaxCorner = ToARGoS(cBounds.mMax);
   }

   /****************************************/
   /****************************************/

   bool CJoltModel::IsCollidingWithSomething() const {
      const JPH::PhysicsSystem& cSystem = m_cJoltEngine.GetSystem();
      const JPH::BodyInterface& cInterface = cSystem.GetBodyInterfaceNoLock();
      /* Ignore contacts among this model's own bodies */
      JPH::IgnoreMultipleBodiesFilter cBodyFilter;
      cBodyFilter.Reserve(JPH::uint(m_vecBodies.size()));
      for(const SBody& s_body : m_vecBodies) {
         cBodyFilter.IgnoreBody(s_body.Id);
      }
      JPH::CollideShapeSettings sSettings;
      for(const SBody& s_body : m_vecBodies) {
         JPH::TransformedShape cShape =
            cInterface.GetTransformedShape(s_body.Id);
         if(cShape.mShape == nullptr) {
            continue;
         }
         JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> cCollector;
         cSystem.GetNarrowPhaseQuery().CollideShape(
            cShape.mShape, JPH::Vec3::sReplicate(1.0f),
            cShape.GetCenterOfMassTransform(),
            sSettings, JPH::RVec3::sZero(), cCollector,
            {}, {}, cBodyFilter);
         for(const JPH::CollideShapeResult& sHit : cCollector.mHits) {
            /* Like dynamics3d, contacts with non-model bodies (e.g.
             * the floor) do not count as collisions */
            if(cInterface.GetUserData(sHit.mBodyID2) != 0) {
               return true;
            }
         }
      }
      return false;
   }

   /****************************************/
   /****************************************/

}
