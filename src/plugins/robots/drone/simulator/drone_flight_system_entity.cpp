/**
 * @file <argos3/plugins/robots/drone/simulator/drone_flight_system_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "drone_flight_system_entity.h"

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/simulator/space/space.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDroneFlightSystemEntity::CDroneFlightSystemEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent),
      m_cPositionReading(CVector3::ZERO),
      m_cOrientationReading(CVector3::ZERO),
      m_cVelocityReading(CVector3::ZERO),
      m_cAngularVelocityReading(CVector3::ZERO),
      m_cTargetPosition(CVector3::ZERO),
      m_fTargetYawAngle(0.0) {}



   /****************************************/
   /****************************************/

   CDroneFlightSystemEntity::CDroneFlightSystemEntity(CComposableEntity* pc_parent,
                                                      const std::string& str_id) :
      CEntity(pc_parent, str_id),
      m_cPositionReading(CVector3::ZERO),
      m_cOrientationReading(CVector3::ZERO),
      m_cVelocityReading(CVector3::ZERO),
      m_cAngularVelocityReading(CVector3::ZERO),
      m_cTargetPosition(CVector3::ZERO),
      m_fTargetYawAngle(0.0) {}

   /****************************************/
   /****************************************/
    
   void CDroneFlightSystemEntity::Reset() {
      m_cPositionReading = CVector3::ZERO;
      m_cOrientationReading = CVector3::ZERO;
      m_cVelocityReading = CVector3::ZERO;
      m_cAngularVelocityReading = CVector3::ZERO;
      m_cTargetPosition = CVector3::ZERO;
      m_fTargetYawAngle = CRadians::ZERO;
      m_bArmed = m_bArmedAtStart;
   }

   /****************************************/
   /****************************************/

   void CDroneFlightSystemEntity::Configure(TConfigurationNode& t_tree) {
      GetNodeAttributeOrDefault(t_tree, "max_xy_velocity", m_fMaxXYVelocity, m_fMaxXYVelocity);
      GetNodeAttributeOrDefault(t_tree, "max_tilt", m_fMaxTilt, m_fMaxTilt);
      GetNodeAttributeOrDefault(t_tree, "armed", m_bArmedAtStart, m_bArmedAtStart);
      if(m_fMaxXYVelocity <= 0.0) {
         THROW_ARGOSEXCEPTION("flight_system max_xy_velocity must be > 0, got " << m_fMaxXYVelocity);
      }
      if(m_fMaxTilt <= 0.0 || m_fMaxTilt >= 1.5) {
         THROW_ARGOSEXCEPTION("flight_system max_tilt must be in (0, 1.5) rad, got " << m_fMaxTilt);
      }
      m_bArmed = m_bArmedAtStart;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CDroneFlightSystemEntity);

   /****************************************/
   /****************************************/

}
   
