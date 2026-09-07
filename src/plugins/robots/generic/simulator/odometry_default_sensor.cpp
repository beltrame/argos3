/**
 * @file <argos3/plugins/robots/generic/simulator/odometry_default_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>

#include "odometry_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   COdometryDriftSensor::COdometryDriftSensor() :
      m_pcEmbodiedEntity(nullptr),
      m_pcRNG(nullptr),
      m_bAddDrift(false),
      m_fPositionDriftStdDev(0.0),
      m_fOrientationDriftStdDev(0.0),
      m_bHasPrevious(false) {}

   /****************************************/
   /****************************************/

   void COdometryDriftSensor::SetRobot(CComposableEntity& c_entity) {
      m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
   }

   /****************************************/
   /****************************************/

   void COdometryDriftSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_OdometrySensor::Init(t_tree);
         GetNodeAttributeOrDefault(t_tree, "position_drift", m_fPositionDriftStdDev, m_fPositionDriftStdDev);
         GetNodeAttributeOrDefault(t_tree, "orientation_drift", m_fOrientationDriftStdDev, m_fOrientationDriftStdDev);
         if(m_fPositionDriftStdDev > 0.0 || m_fOrientationDriftStdDev > 0.0) {
            m_bAddDrift = true;
            m_pcRNG = CRandom::CreateRNG("argos");
         }
         /* sensor is enabled by default */
         Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in default odometry sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void COdometryDriftSensor::Update() {
      /* sensor is disabled--nothing to do */
      if(IsDisabled()) {
         return;
      }
      const CVector3& cGroundTruthPosition = m_pcEmbodiedEntity->GetOriginAnchor().Position;
      const CQuaternion& cGroundTruthOrientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;
      m_sReading.Tick = UInt32(CSimulator::GetInstance().GetSpace().GetSimulationClock());
      m_sReading.Valid = true;
      if(!m_bHasPrevious) {
         /* First tick: the estimate starts at the robot's OWN origin, not at
          * its arena pose. There is no relative motion yet to perturb.
          *
          * Identity rather than ground truth because that is the convention
          * every other odometry source reports in. A real dead-reckoning
          * pipeline has no idea where in the world it was switched on: wheel
          * encoders start at zero, and a SLAM front end defines its map frame
          * at the first keyframe, which is why the "external" implementation
          * (fed through the external_estimator medium by e.g. Fast-LIVO2)
          * delivers a start-relative pose. Seeding this one at the arena pose
          * made it the only source quietly reporting world coordinates, and
          * seeding it at identity is what "aligned with external" means.
          *
          * The difference is not cosmetic downstream. A consumer that knows
          * the spawn poses composes T_world_map = start_pose onto what it
          * receives; against a stream that already carries the spawn, that
          * offset lands twice. Measured on a four-robot SwarmDeck run whose
          * robots spawn along x at -9, -3, 3 and 9 in a 26 m building: the
          * merged map came out spanning 44 m, and the two robots spawned at
          * yaw pi contributed a 180 degree rotation rather than a shift, a
          * doubled pose being a doubled rotation too. Wall agreement against
          * the true floorplan was 27.3%, against 88.9% for the same scene
          * running the external estimator.
          *
          * Ground truth stays available and unchanged on the positioning
          * sensor. Anything comparing this estimate against it must now
          * compose the known start pose, which is the arithmetic a real
          * deployment has to do anyway. */
         m_sReading.Position = CVector3::ZERO;
         m_sReading.Orientation = CQuaternion();
         m_sReading.LinearVelocity = CVector3::ZERO;
         m_sReading.AngularVelocity = CVector3::ZERO;
         m_cPrevGroundTruthPosition = cGroundTruthPosition;
         m_cPrevGroundTruthOrientation = cGroundTruthOrientation;
         m_bHasPrevious = true;
         return;
      }
      /* Relative motion this tick, expressed in the previous
       * ground-truth body frame: this is what a real odometry pipeline
       * (wheel encoders, visual odometry) would locally measure. */
      CVector3 cRelPositionBody = cGroundTruthPosition - m_cPrevGroundTruthPosition;
      cRelPositionBody.Rotate(m_cPrevGroundTruthOrientation.Inverse());
      CQuaternion cRelOrientation = m_cPrevGroundTruthOrientation.Inverse() * cGroundTruthOrientation;
      Real fDistance = cRelPositionBody.Length();
      if(m_bAddDrift) {
         /* Perturb the locally-measured relative motion; the resulting
          * pose estimate is then integrated through the sensor's own
          * (already drifted) frame below, so errors compound over
          * distance travelled exactly as with real dead reckoning. */
         cRelPositionBody += CVector3(m_pcRNG->Gaussian(m_fPositionDriftStdDev * fDistance),
                                      m_pcRNG->Gaussian(m_fPositionDriftStdDev * fDistance),
                                      m_pcRNG->Gaussian(m_fPositionDriftStdDev * fDistance));
         CRadians cRelYaw, cRelPitch, cRelRoll;
         cRelOrientation.ToEulerAngles(cRelYaw, cRelPitch, cRelRoll);
         cRelYaw += CRadians(m_pcRNG->Gaussian(m_fOrientationDriftStdDev * fDistance));
         cRelOrientation.FromEulerAngles(cRelYaw, cRelPitch, cRelRoll);
      }
      /* Integrate the (possibly perturbed) relative motion through the
       * sensor's own drifted frame, not through ground truth. */
      CVector3 cWorldRelPosition = cRelPositionBody;
      cWorldRelPosition.Rotate(m_sReading.Orientation);
      m_sReading.Position += cWorldRelPosition;
      m_sReading.Orientation = m_sReading.Orientation * cRelOrientation;
      /* Body-frame twist, from the same perturbed relative motion: this
       * is what the pipeline measured, not the true velocity. */
      Real fTickLength = CPhysicsEngine::GetSimulationClockTick();
      m_sReading.LinearVelocity = cRelPositionBody / fTickLength;
      CRadians cTwistYaw, cTwistPitch, cTwistRoll;
      cRelOrientation.ToEulerAngles(cTwistYaw, cTwistPitch, cTwistRoll);
      m_sReading.AngularVelocity.Set(cTwistRoll.GetValue() / fTickLength,
                                     cTwistPitch.GetValue() / fTickLength,
                                     cTwistYaw.GetValue() / fTickLength);
      m_cPrevGroundTruthPosition = cGroundTruthPosition;
      m_cPrevGroundTruthOrientation = cGroundTruthOrientation;
   }

   /****************************************/
   /****************************************/

   void COdometryDriftSensor::Reset() {
      /* Same origin convention as the first tick of Update(): a reset robot
       * restarts its own odometry, it does not learn where it is. The
       * ground-truth reference the relative motion is measured against is a
       * separate quantity and does come from the anchor. */
      m_sReading.Position = CVector3::ZERO;
      m_sReading.Orientation = CQuaternion();
      m_sReading.LinearVelocity = CVector3::ZERO;
      m_sReading.AngularVelocity = CVector3::ZERO;
      m_sReading.Tick = 0;
      m_sReading.Valid = false;
      m_cPrevGroundTruthPosition = m_pcEmbodiedEntity->GetOriginAnchor().Position;
      m_cPrevGroundTruthOrientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;
      m_bHasPrevious = false;
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(COdometryDriftSensor,
                   "odometry", "drift",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "A dead-reckoning odometry sensor with configurable drift.",

                   "This sensor returns a pose estimate that drifts away from ground truth\n"
                   "over distance travelled, modelling a real wheel/visual odometry pipeline\n"
                   "without requiring one to actually run inside ARGoS. It exists to feed\n"
                   "SLAM/localization stacks (e.g. Swarm-SLAM) that expect an external, noisy-\n"
                   "but-continuous odometry source and compute no odometry of their own. This\n"
                   "sensor can be used with any robot, since it accesses only the body\n"
                   "component. In controllers, you must include the ci_odometry_sensor.h\n"
                   "header.\n\n"

                   "FRAME. The estimate is START-RELATIVE: it reads identity on the first\n"
                   "tick and after Reset(), whatever the robot's pose in the arena, and\n"
                   "thereafter accumulates the drifted relative motion. This is the same\n"
                   "convention as the 'external' implementation, whose SLAM front end defines\n"
                   "its map frame at the first keyframe, and the same as real hardware, whose\n"
                   "encoders do not know where they were switched on. A consumer that needs\n"
                   "arena coordinates composes the known start pose onto this reading; one\n"
                   "that composes a start pose onto a stream ALREADY carrying it will place\n"
                   "the robot at twice its spawn offset, and rotate it twice as far.\n"
                   "Ground truth is the positioning sensor's job, not this one's.\n\n"

                   "This sensor is enabled by default.\n\n"

                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <odometry implementation=\"drift\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"

                   "OPTIONAL XML CONFIGURATION\n\n"

                   "Attribute 'position_drift' (default 0) sets the standard deviation of the\n"
                   "per-axis Gaussian noise added to the local relative displacement each tick,\n"
                   "as a fraction of the distance travelled that tick (e.g. 0.01 means 1% of\n"
                   "the distance travelled). Attribute 'orientation_drift' (default 0) sets the\n"
                   "standard deviation, in radians per metre travelled, of the Gaussian noise\n"
                   "added to the yaw of the relative rotation each tick. Both are zero (no\n"
                   "drift, i.e. this sensor behaves like the positioning sensor) by default.\n\n"

                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <odometry implementation=\"drift\"\n"
                   "                  position_drift=\"0.01\"\n"
                   "                  orientation_drift=\"0.005\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",

                   "Usable"
                  );

}
