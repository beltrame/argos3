#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_imu_sensor.h>

#include <cmath>

/****************************************/
/****************************************/

void CDroneImuLoopFunctions::Init(TConfigurationNode& t_tree) {
   m_pcDrone = &dynamic_cast<CDroneEntity&>(GetSpace().GetEntity("drone"));
}

/****************************************/
/****************************************/

void CDroneImuLoopFunctions::PostStep() {
   /* The drone must stay upright the whole flight */
   const CQuaternion& cOrientation =
      m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Orientation;
   CRadians cYaw, cPitch, cRoll;
   cOrientation.ToEulerAngles(cYaw, cPitch, cRoll);
   if(std::fabs(cRoll.GetValue()) > 0.6 || std::fabs(cPitch.GetValue()) > 0.6) {
      THROW_ARGOSEXCEPTION("Drone lost attitude stability at tick "
                           << GetSpace().GetSimulationClock()
                           << " (roll " << cRoll.GetValue()
                           << ", pitch " << cPitch.GetValue() << ")");
   }
   /* The 1 m/s XY velocity limit must be respected: after 2 s the
    * drone cannot have covered much more than 2 m */
   if(GetSpace().GetSimulationClock() == 20) {
      const CVector3& cPosition =
         m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
      if(cPosition.GetX() > 2.2) {
         THROW_ARGOSEXCEPTION("Drone at x = " << cPosition.GetX()
                              << " after 2 s; the XY velocity limit is broken");
      }
   }
   CCI_IMUSensor::SReading sImu =
      m_pcDrone->GetControllableEntity().GetController()
         .GetSensor<CCI_IMUSensor>("imu")->GetReading();
   const Real fGravity = 9.81;
   /* Early in the flight (climbing and accelerating towards the target)
    * the IMU's specific force must be finite and visibly different from
    * pure gravity: this is the actual regression check on the finite-
    * difference implementation, since a bug that left the sensor stuck
    * at its initial (0,0,g) reading would otherwise pass silently. */
   if(GetSpace().GetSimulationClock() == 5) {
      if(!std::isfinite(sImu.LinearAcceleration.GetX()) ||
         !std::isfinite(sImu.LinearAcceleration.GetY()) ||
         !std::isfinite(sImu.LinearAcceleration.GetZ()) ||
         !std::isfinite(sImu.AngularVelocity.GetX()) ||
         !std::isfinite(sImu.AngularVelocity.GetY()) ||
         !std::isfinite(sImu.AngularVelocity.GetZ())) {
         THROW_ARGOSEXCEPTION("IMU reading is not finite at tick 5: accel "
                              << sImu.LinearAcceleration
                              << ", gyro " << sImu.AngularVelocity);
      }
      CVector3 cDeviation = sImu.LinearAcceleration - CVector3(0.0, 0.0, fGravity);
      if(cDeviation.Length() < 0.1) {
         THROW_ARGOSEXCEPTION("IMU specific force at tick 5 barely differs from "
                              "pure gravity (deviation " << cDeviation.Length()
                              << "); the sensor does not appear to be tracking "
                              "the climb/acceleration phase");
      }
   }
   /* After 10 s the drone must hover at the target */
   if(GetSpace().GetSimulationClock() == 100) {
      const CVector3& cPosition =
         m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
      if(Distance(cPosition, CVector3(4.0, 0.0, 1.0)) > 0.15) {
         THROW_ARGOSEXCEPTION("Drone at " << cPosition
                              << " after 10 s, expected ~(4, 0, 1)");
      }
   }
   /* Well into the hover, the IMU must read close to gravity (small net
    * acceleration) with a small angular rate (the drone is not rotating) */
   if(GetSpace().GetSimulationClock() == 290) {
      Real fAccelMagnitude = sImu.LinearAcceleration.Length();
      if(fAccelMagnitude < fGravity - 1.5 || fAccelMagnitude > fGravity + 1.5) {
         THROW_ARGOSEXCEPTION("IMU specific force magnitude " << fAccelMagnitude
                              << " far from gravity (" << fGravity
                              << ") while hovering at tick 290");
      }
      if(sImu.AngularVelocity.Length() > 0.5) {
         THROW_ARGOSEXCEPTION("IMU angular velocity magnitude "
                              << sImu.AngularVelocity.Length()
                              << " too high while hovering at tick 290");
      }
   }
}

/****************************************/
/****************************************/

bool CDroneImuLoopFunctions::IsExperimentFinished() {
   if(GetSpace().GetSimulationClock() < 300) {
      return false;
   }
   /* After 30 s the drone must hover at the target */
   const CVector3& cPosition =
      m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
   if(Distance(cPosition, CVector3(4.0, 0.0, 1.0)) > 0.1) {
      THROW_ARGOSEXCEPTION("Drone at " << cPosition
                           << ", expected hover at ~(4, 0, 1)");
   }
   return true;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CDroneImuLoopFunctions, "drone_imu_loop_functions");
