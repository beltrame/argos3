/**
 * @file <argos3/plugins/robots/generic/simulator/odometry_default_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>

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
      if(!m_bHasPrevious) {
         /* First tick: initialize the drifted estimate at ground truth,
          * there is no relative motion yet to perturb. */
         m_sReading.Position = cGroundTruthPosition;
         m_sReading.Orientation = cGroundTruthOrientation;
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
      m_cPrevGroundTruthPosition = cGroundTruthPosition;
      m_cPrevGroundTruthOrientation = cGroundTruthOrientation;
   }

   /****************************************/
   /****************************************/

   void COdometryDriftSensor::Reset() {
      m_sReading.Position = m_pcEmbodiedEntity->GetOriginAnchor().Position;
      m_sReading.Orientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;
      m_cPrevGroundTruthPosition = m_sReading.Position;
      m_cPrevGroundTruthOrientation = m_sReading.Orientation;
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
