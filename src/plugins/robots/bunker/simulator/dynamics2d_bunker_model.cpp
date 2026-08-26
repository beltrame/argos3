/**
 * @file <argos3/plugins/robots/bunker/simulator/dynamics2d_bunker_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "dynamics2d_bunker_model.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   const Real CDynamics2DBunkerModel::BUNKER_LENGTH      = 1.023f;
   const Real CDynamics2DBunkerModel::BUNKER_WIDTH       = 0.778f;
   const Real CDynamics2DBunkerModel::BUNKER_HEIGHT      = 0.380f;
   const Real CDynamics2DBunkerModel::BUNKER_MASS        = 170.0f;
   const Real CDynamics2DBunkerModel::BUNKER_MAX_FORCE   = 1400.0f;
   const Real CDynamics2DBunkerModel::BUNKER_MAX_TORQUE  = 460.0f;
   const Real CDynamics2DBunkerModel::BUNKER_TRACK_GAUGE = 0.620f;

   /****************************************/
   /****************************************/

   CDynamics2DBunkerModel::CDynamics2DBunkerModel(CDynamics2DEngine& c_engine,
                                                          CBunkerEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cBunkerEntity(c_entity),
      m_cWheeledEntity(m_cBunkerEntity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      BUNKER_MAX_FORCE,
                      BUNKER_MAX_TORQUE,
                      BUNKER_TRACK_GAUGE,
                      c_entity.GetConfigurationNode()) {
      /* Create the physics body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(BUNKER_MASS,
                                  cpMomentForBox(BUNKER_MASS,
                                                 BUNKER_LENGTH,
                                                 BUNKER_WIDTH)));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());

      /* Create the collision shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpBoxShapeNew(ptBody,
                                       BUNKER_LENGTH,
                                       BUNKER_WIDTH));
      ptShape->e = 0.0;  // Inelastic collision
      ptShape->u = 0.8;  // High traction rubber tracks

      /* Constrain base body to follow diff steering kinematics */
      m_cDiffSteering.AttachTo(ptBody);
      SetBody(ptBody, BUNKER_HEIGHT);

      /* Register anchor update callbacks */
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("body"),
                           &CDynamics2DBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("left_tracks"),
                           &CDynamics2DBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("right_tracks"),
                           &CDynamics2DBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("lidar"),
                           &CDynamics2DBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("camera"),
                           &CDynamics2DBunkerModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("imu"),
                           &CDynamics2DBunkerModel::UpdateAuxiliaryAnchor);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DBunkerModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DBunkerModel::UpdateFromEntityStatus() {
      m_cDiffSteering.SetWheelVelocity(m_cWheeledEntity.GetWheelVelocities()[0],
                                       m_cWheeledEntity.GetWheelVelocities()[1]);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DBunkerModel::UpdateAuxiliaryAnchor(SAnchor& s_anchor) {
      s_anchor.Position = s_anchor.OffsetPosition;
      s_anchor.Position.Rotate(GetEmbodiedEntity().GetOriginAnchor().Orientation);
      s_anchor.Position += GetEmbodiedEntity().GetOriginAnchor().Position;
      s_anchor.Orientation = GetEmbodiedEntity().GetOriginAnchor().Orientation * s_anchor.OffsetOrientation;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CBunkerEntity, CDynamics2DBunkerModel);

}

