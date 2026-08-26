/**
 * @file <argos3/plugins/robots/scout-mini/simulator/dynamics2d_scout_mini_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "dynamics2d_scout_mini_model.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   const Real CDynamics2DScoutMiniModel::SCOUT_MINI_LENGTH      = 0.612f;
   const Real CDynamics2DScoutMiniModel::SCOUT_MINI_WIDTH       = 0.580f;
   const Real CDynamics2DScoutMiniModel::SCOUT_MINI_HEIGHT      = 0.245f;
   const Real CDynamics2DScoutMiniModel::SCOUT_MINI_MASS        = 25.0f;
   const Real CDynamics2DScoutMiniModel::SCOUT_MINI_MAX_FORCE   = 250.0f;
   const Real CDynamics2DScoutMiniModel::SCOUT_MINI_MAX_TORQUE  = 75.0f;
   const Real CDynamics2DScoutMiniModel::SCOUT_MINI_TRACK_GAUGE = 0.450f;

   /****************************************/
   /****************************************/

   CDynamics2DScoutMiniModel::CDynamics2DScoutMiniModel(CDynamics2DEngine& c_engine,
                                                          CScoutMiniEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cScoutMiniEntity(c_entity),
      m_cWheeledEntity(m_cScoutMiniEntity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      SCOUT_MINI_MAX_FORCE,
                      SCOUT_MINI_MAX_TORQUE,
                      SCOUT_MINI_TRACK_GAUGE,
                      c_entity.GetConfigurationNode()) {
      /* Create the physics body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(SCOUT_MINI_MASS,
                                  cpMomentForBox(SCOUT_MINI_MASS,
                                                 SCOUT_MINI_LENGTH,
                                                 SCOUT_MINI_WIDTH)));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());

      /* Create the collision shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpBoxShapeNew(ptBody,
                                       SCOUT_MINI_LENGTH,
                                       SCOUT_MINI_WIDTH));
      ptShape->e = 0.0;  // Inelastic collision
      ptShape->u = 0.8;  // High traction rubber tracks

      /* Constrain base body to follow diff steering kinematics */
      m_cDiffSteering.AttachTo(ptBody);
      SetBody(ptBody, SCOUT_MINI_HEIGHT);

      /* Register anchor update callbacks */
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("body"),
                           &CDynamics2DScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("left_wheels"),
                           &CDynamics2DScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("right_wheels"),
                           &CDynamics2DScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("lidar"),
                           &CDynamics2DScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("camera"),
                           &CDynamics2DScoutMiniModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("imu"),
                           &CDynamics2DScoutMiniModel::UpdateAuxiliaryAnchor);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DScoutMiniModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DScoutMiniModel::UpdateFromEntityStatus() {
      m_cDiffSteering.SetWheelVelocity(m_cWheeledEntity.GetWheelVelocities()[0],
                                       m_cWheeledEntity.GetWheelVelocities()[1]);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DScoutMiniModel::UpdateAuxiliaryAnchor(SAnchor& s_anchor) {
      s_anchor.Position = s_anchor.OffsetPosition;
      s_anchor.Position.Rotate(GetEmbodiedEntity().GetOriginAnchor().Orientation);
      s_anchor.Position += GetEmbodiedEntity().GetOriginAnchor().Position;
      s_anchor.Orientation = GetEmbodiedEntity().GetOriginAnchor().Orientation * s_anchor.OffsetOrientation;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CScoutMiniEntity, CDynamics2DScoutMiniModel);

}

