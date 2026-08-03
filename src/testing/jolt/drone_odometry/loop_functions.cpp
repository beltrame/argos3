#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_odometry_sensor.h>

#include <cmath>

/****************************************/
/****************************************/

void CDroneOdometryLoopFunctions::Init(TConfigurationNode& t_tree) {
   m_pcDrone = &dynamic_cast<CDroneEntity&>(GetSpace().GetEntity("drone"));
}

/****************************************/
/****************************************/

void CDroneOdometryLoopFunctions::PostStep() {
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
   const CVector3& cGroundTruthPosition =
      m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
   CCI_OdometrySensor::SReading sOdometry =
      m_pcDrone->GetControllableEntity().GetController()
         .GetSensor<CCI_OdometrySensor>("odometry")->GetReading();
   if(!std::isfinite(sOdometry.Position.GetX()) ||
      !std::isfinite(sOdometry.Position.GetY()) ||
      !std::isfinite(sOdometry.Position.GetZ())) {
      THROW_ARGOSEXCEPTION("Odometry reading is not finite at tick "
                           << GetSpace().GetSimulationClock()
                           << ": " << sOdometry.Position);
   }
   Real fDeviation = Distance(sOdometry.Position, cGroundTruthPosition);
   /* Early in the flight, almost no distance has been travelled yet, so
    * almost no drift can have accumulated: the odometry estimate must
    * still closely track ground truth. */
   if(GetSpace().GetSimulationClock() == 3) {
      if(fDeviation > 0.05) {
         THROW_ARGOSEXCEPTION("Odometry deviated from ground truth by "
                              << fDeviation << " m after only 0.3 s; drift "
                              "is accumulating far faster than configured");
      }
   }
   /* After 10 s the drone must hover at the target */
   if(GetSpace().GetSimulationClock() == 100) {
      if(Distance(cGroundTruthPosition, CVector3(4.0, 0.0, 1.0)) > 0.15) {
         THROW_ARGOSEXCEPTION("Drone at " << cGroundTruthPosition
                              << " after 10 s, expected ~(4, 0, 1)");
      }
   }
   /* Late in the flight, several metres have been travelled: the drift
    * must have accumulated to a measurable (proving it is not a no-op)
    * but bounded (proving it has not blown up) deviation. */
   if(GetSpace().GetSimulationClock() == 290) {
      if(fDeviation < 0.001) {
         THROW_ARGOSEXCEPTION("Odometry has not measurably drifted from "
                              "ground truth (deviation " << fDeviation
                              << " m) after most of the flight; the drift "
                              "model does not appear to be doing anything");
      }
      if(fDeviation > 3.0) {
         THROW_ARGOSEXCEPTION("Odometry has drifted implausibly far from "
                              "ground truth (deviation " << fDeviation
                              << " m); check the drift integration for a "
                              "runaway feedback loop");
      }
   }
}

/****************************************/
/****************************************/

bool CDroneOdometryLoopFunctions::IsExperimentFinished() {
   if(GetSpace().GetSimulationClock() < 300) {
      return false;
   }
   /* After 30 s the drone must hover at the target (ground truth,
    * unaffected by the odometry sensor's drift) */
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

REGISTER_LOOP_FUNCTIONS(CDroneOdometryLoopFunctions, "drone_odometry_loop_functions");
