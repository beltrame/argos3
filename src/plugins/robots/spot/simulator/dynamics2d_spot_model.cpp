/**
 * @file <argos3/plugins/robots/spot/simulator/dynamics2d_spot_model.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "dynamics2d_spot_model.h"
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   const Real CDynamics2DSpotModel::SPOT_LENGTH      = 1.100f;
   const Real CDynamics2DSpotModel::SPOT_WIDTH       = 0.500f;
   const Real CDynamics2DSpotModel::SPOT_HEIGHT      = 0.620f;
   const Real CDynamics2DSpotModel::SPOT_MASS        = 32.7f;
   const Real CDynamics2DSpotModel::SPOT_MAX_FORCE   = 350.0f;
   const Real CDynamics2DSpotModel::SPOT_MAX_TORQUE  = 120.0f;
   const Real CDynamics2DSpotModel::SPOT_TRACK_GAUGE = 0.500f;

   /****************************************/
   /****************************************/

   CDynamics2DSpotModel::CDynamics2DSpotModel(CDynamics2DEngine& c_engine,
                                                          CSpotEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cSpotEntity(c_entity),
      m_cWheeledEntity(m_cSpotEntity.GetWheeledEntity()),
      m_cDiffSteering(c_engine,
                      SPOT_MAX_FORCE,
                      SPOT_MAX_TORQUE,
                      SPOT_TRACK_GAUGE,
                      c_entity.GetConfigurationNode()) {
      /* Create the physics body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(SPOT_MASS,
                                  cpMomentForBox(SPOT_MASS,
                                                 SPOT_LENGTH,
                                                 SPOT_WIDTH)));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());

      /* Create the collision shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpBoxShapeNew(ptBody,
                                       SPOT_LENGTH,
                                       SPOT_WIDTH));
      ptShape->e = 0.0;  // Inelastic collision
      ptShape->u = 0.8;  // High traction rubber tracks

      /* Constrain base body to follow diff steering kinematics */
      m_cDiffSteering.AttachTo(ptBody);
      SetBody(ptBody, SPOT_HEIGHT);

      /* Register anchor update callbacks */
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("body"),
                           &CDynamics2DSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("left_legs"),
                           &CDynamics2DSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("right_legs"),
                           &CDynamics2DSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("lidar"),
                           &CDynamics2DSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("camera"),
                           &CDynamics2DSpotModel::UpdateAuxiliaryAnchor);
      RegisterAnchorMethod(c_entity.GetEmbodiedEntity().GetAnchor("imu"),
                           &CDynamics2DSpotModel::UpdateAuxiliaryAnchor);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DSpotModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DSpotModel::UpdateFromEntityStatus() {
      m_cDiffSteering.SetWheelVelocity(m_cWheeledEntity.GetWheelVelocities()[0],
                                       m_cWheeledEntity.GetWheelVelocities()[1]);
   }

   /****************************************/
   /****************************************/

   void CDynamics2DSpotModel::UpdateAuxiliaryAnchor(SAnchor& s_anchor) {
      s_anchor.Position = s_anchor.OffsetPosition;
      s_anchor.Position.Rotate(GetEmbodiedEntity().GetOriginAnchor().Orientation);
      s_anchor.Position += GetEmbodiedEntity().GetOriginAnchor().Position;
      s_anchor.Orientation = GetEmbodiedEntity().GetOriginAnchor().Orientation * s_anchor.OffsetOrientation;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CSpotEntity, CDynamics2DSpotModel);

}

