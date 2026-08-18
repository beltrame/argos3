/**
 * @file <argos3/plugins/robots/bunker-mini/simulator/dynamics2d_bunker_mini_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "dynamics2d_bunker_mini_model.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   const Real CDynamics2DBunkerMiniModel::BUNKER_MINI_LENGTH      = 0.660f;
   const Real CDynamics2DBunkerMiniModel::BUNKER_MINI_WIDTH       = 0.584f;
   const Real CDynamics2DBunkerMiniModel::BUNKER_MINI_HEIGHT      = 0.281f;
   const Real CDynamics2DBunkerMiniModel::BUNKER_MINI_MASS        = 55.0f;
   const Real CDynamics2DBunkerMiniModel::BUNKER_MINI_MAX_FORCE   = 500.0f;
   const Real CDynamics2DBunkerMiniModel::BUNKER_MINI_MAX_TORQUE  = 150.0f;
   const Real CDynamics2DBunkerMiniModel::BUNKER_MINI_TRACK_GAUGE = 0.412f;

   /****************************************/
   /****************************************/

   CDynamics2DBunkerMiniModel::CDynamics2DBunkerMiniModel(CDynamics2DEngine& c_engine,
                                                          CBunkerMiniEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cBunkerMiniEntity(c_entity),
      m_cWheeledEntity(m_cBunkerMiniEntity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      BUNKER_MINI_MAX_FORCE,
                      BUNKER_MINI_MAX_TORQUE,
                      BUNKER_MINI_TRACK_GAUGE,
                      c_entity.GetConfigurationNode()) {
      /* Create the physics body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(BUNKER_MINI_MASS,
                                  cpMomentForBox(BUNKER_MINI_MASS,
                                                 BUNKER_MINI_LENGTH,
                                                 BUNKER_MINI_WIDTH)));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());

      /* Create the collision shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpBoxShapeNew(ptBody,
                                       BUNKER_MINI_LENGTH,
                                       BUNKER_MINI_WIDTH));
      ptShape->e = 0.0;  // Inelastic collision
      ptShape->u = 0.8;  // High traction rubber tracks

      /* Constrain base body to follow diff steering kinematics */
      m_cDiffSteering.AttachTo(ptBody);
      SetBody(ptBody, BUNKER_MINI_HEIGHT);

      /* Register anchor update callbacks */
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("body"),
                           &CDynamics2DBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("left_track"),
                           &CDynamics2DBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("right_track"),
                           &CDynamics2DBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("lidar"),
                           &CDynamics2DBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("camera"),
                           &CDynamics2DBunkerMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("imu"),
                           &CDynamics2DBunkerMiniModel::UpdateAuxiliaryAnchor);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DBunkerMiniModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DBunkerMiniModel::UpdateFromEntityStatus() {
      m_cDiffSteering.SetWheelVelocity(m_cWheeledEntity.GetWheelVelocities()[0],
                                       m_cWheeledEntity.GetWheelVelocities()[1]);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DBunkerMiniModel::UpdateAuxiliaryAnchor(SAnchor& s_anchor) {
      s_anchor.Position = s_anchor.OffsetPosition;
      s_anchor.Position.Rotate(GetEmbodiedEntity().GetOriginAnchor().Orientation);
      s_anchor.Position += GetEmbodiedEntity().GetOriginAnchor().Position;
      s_anchor.Orientation = GetEmbodiedEntity().GetOriginAnchor().Orientation * s_anchor.OffsetOrientation;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CBunkerMiniEntity, CDynamics2DBunkerMiniModel);

}

