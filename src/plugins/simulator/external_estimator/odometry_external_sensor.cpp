/**
 * @file <argos3/plugins/simulator/external_estimator/odometry_external_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/composable_entity.h>

#include "odometry_external_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   COdometryExternalSensor::COdometryExternalSensor() :
      m_pcMedium(nullptr),
      m_bHoldLast(true) {}

   /****************************************/
   /****************************************/

   void COdometryExternalSensor::SetRobot(CComposableEntity& c_entity) {
      /* The medium files estimates under the robot's id, which is what
       * the far side knows this robot as */
      m_strRobotId = c_entity.GetId();
   }

   /****************************************/
   /****************************************/

   void COdometryExternalSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_OdometrySensor::Init(t_tree);
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         GetNodeAttributeOrDefault(t_tree, "hold_last", m_bHoldLast, m_bHoldLast);
         m_pcMedium =
            &CSimulator::GetInstance().GetMedium<CExternalEstimatorMedium>(strMedium);
         /* sensor is enabled by default */
         Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in external odometry sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void COdometryExternalSensor::Update() {
      /* sensor is disabled--nothing to do */
      if(IsDisabled()) {
         return;
      }
      CExternalEstimatorMedium::SEstimate sEstimate;
      if(m_pcMedium->GetEstimate(m_strRobotId, sEstimate) && sEstimate.Valid) {
         m_sReading.Position = sEstimate.Position;
         m_sReading.Orientation = sEstimate.Orientation;
         m_sReading.LinearVelocity = sEstimate.LinearVelocity;
         m_sReading.AngularVelocity = sEstimate.AngularVelocity;
         m_sReading.Tick = sEstimate.Tick;
         m_sReading.Valid = true;
      }
      else if(!m_bHoldLast) {
         m_sReading = SReading();
      }
      /* Otherwise the previous reading stands: an estimator that
       * publishes at 10 Hz into a 100 Hz simulation leaves nine ticks
       * out of ten with nothing new to say, exactly as on a real robot.
       * SReading::Tick tells the controller how old the pose is. */
   }

   /****************************************/
   /****************************************/

   void COdometryExternalSensor::Reset() {
      m_sReading = SReading();
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(COdometryExternalSensor,
                   "odometry", "external",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "Odometry computed by a SLAM front-end running outside ARGoS.",

                   "This sensor reports the pose that an external estimator (e.g. Ultra-Fusion\n"
                   "or any other multi-sensor SLAM front-end) computed from this robot's\n"
                   "simulated IMU, camera, lidar and wheel encoders. ARGoS itself runs no\n"
                   "estimator and links no SLAM library: the <external_estimator> medium\n"
                   "streams the sensor data out over a Unix socket and receives pose estimates\n"
                   "back, and this sensor reports what came back. Unlike implementation\n"
                   "\"drift\", which models drift statistically by perturbing ground truth, the\n"
                   "error here is whatever the real algorithm produced, including its failure\n"
                   "modes (scan degeneracy, wheel slip, visual dropout).\n\n"

                   "The robot must also carry the sensors the estimator fuses: an \"imu\" at\n"
                   "least, plus whichever of \"photorealistic_camera\",\n"
                   "\"photorealistic_lidar\" and \"differential_steering\" apply. In\n"
                   "controllers, you must include the ci_odometry_sensor.h header.\n\n"

                   "The reading is INVALID until the estimator produces its first pose, which\n"
                   "takes a real algorithm anywhere from a few frames to a few seconds. Check\n"
                   "SReading::Valid before using the pose, and SReading::Tick to see how old it\n"
                   "is; a pose that lags the simulation by a few ticks is normal and models the\n"
                   "latency of a real estimator.\n\n"

                   "This sensor is enabled by default.\n\n"

                   "REQUIRED XML CONFIGURATION\n\n"
                   "The 'medium' attribute names the <external_estimator> medium whose\n"
                   "estimates this sensor reads; that medium must be declared under <media>,\n"
                   "otherwise this sensor never reports anything.\n\n"

                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <odometry implementation=\"external\" medium=\"uf\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"

                   "OPTIONAL XML CONFIGURATION\n\n"

                   "Attribute 'hold_last' (default true) keeps the last pose on ticks where the\n"
                   "estimator reported nothing, which is what a real consumer of an odometry\n"
                   "topic sees; SReading::Tick then says how old that pose is. Set it to false\n"
                   "to have the reading go invalid whenever no fresh pose arrived.\n\n"

                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <odometry implementation=\"external\"\n"
                   "                  medium=\"uf\"\n"
                   "                  hold_last=\"true\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",

                   "Usable"
                  );

}
