/**
 * @file <argos3/plugins/robots/generic/simulator/imu_default_sensor.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/utility/math/general.h>

#include "imu_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   CIMUDefaultSensor::CIMUDefaultSensor() :
      m_pcEmbodiedEntity(nullptr),
      m_pcRNG(nullptr),
      m_bAddNoise(false),
      m_fGravity(9.81),
      m_fGyroNoiseStdDev(0.0),
      m_fAccelNoiseStdDev(0.0),
      m_fGyroBiasWalkStdDev(0.0),
      m_fAccelBiasWalkStdDev(0.0),
      m_cGyroBias(CVector3::ZERO),
      m_cAccelBias(CVector3::ZERO),
      m_bHasPrevious(false),
      m_cPrevVelocity(CVector3::ZERO) {}

   /****************************************/
   /****************************************/

   void CIMUDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
   }

   /****************************************/
   /****************************************/

   void CIMUDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_IMUSensor::Init(t_tree);
         GetNodeAttributeOrDefault(t_tree, "gravity", m_fGravity, m_fGravity);
         GetNodeAttributeOrDefault(t_tree, "gyro_noise_std_dev", m_fGyroNoiseStdDev, m_fGyroNoiseStdDev);
         GetNodeAttributeOrDefault(t_tree, "accel_noise_std_dev", m_fAccelNoiseStdDev, m_fAccelNoiseStdDev);
         GetNodeAttributeOrDefault(t_tree, "gyro_bias_walk_std_dev", m_fGyroBiasWalkStdDev, m_fGyroBiasWalkStdDev);
         GetNodeAttributeOrDefault(t_tree, "accel_bias_walk_std_dev", m_fAccelBiasWalkStdDev, m_fAccelBiasWalkStdDev);
         if(m_fGyroNoiseStdDev > 0.0 || m_fAccelNoiseStdDev > 0.0 ||
            m_fGyroBiasWalkStdDev > 0.0 || m_fAccelBiasWalkStdDev > 0.0) {
            m_bAddNoise = true;
            m_pcRNG = CRandom::CreateRNG("argos");
         }
         /* sensor is enabled by default */
         Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in default IMU sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CIMUDefaultSensor::Update() {
      /* sensor is disabled--nothing to do */
      if(IsDisabled()) {
         return;
      }
      const CVector3& cPosition = m_pcEmbodiedEntity->GetOriginAnchor().Position;
      const CQuaternion& cOrientation = m_pcEmbodiedEntity->GetOriginAnchor().Orientation;
      Real fDeltaT = CPhysicsEngine::GetSimulationClockTick();
      if(!m_bHasPrevious || fDeltaT <= 0.0) {
         /* No previous sample to difference against yet: report a
          * plausible at-rest reading (gravity only, no rotation) rather
          * than a spurious spike from an undefined finite difference. */
         m_sReading.AngularVelocity = CVector3::ZERO;
         m_sReading.LinearAcceleration = CVector3(0.0, 0.0, m_fGravity);
         m_cPrevPosition = cPosition;
         m_cPrevOrientation = cOrientation;
         m_cPrevVelocity = CVector3::ZERO;
         m_bHasPrevious = true;
         return;
      }
      /* Linear velocity and world-frame acceleration by finite difference */
      CVector3 cVelocity = (cPosition - m_cPrevPosition) / fDeltaT;
      CVector3 cWorldAcceleration = (cVelocity - m_cPrevVelocity) / fDeltaT;
      /* Specific force = acceleration - gravity vector; a robot at rest
       * (zero world acceleration) therefore reads (0,0,+gravity), matching
       * a real accelerometer, which senses the normal force opposing
       * gravity rather than gravity itself. */
      CVector3 cGravityVector(0.0, 0.0, -m_fGravity);
      CVector3 cSpecificForceBody = cWorldAcceleration - cGravityVector;
      cSpecificForceBody.Rotate(cOrientation.Inverse());
      /* Angular velocity, directly in the body frame: the relative
       * rotation from the previous to the current orientation, expressed
       * in the body frame, is PrevOrientation^-1 * Orientation. */
      CQuaternion cRelOrientation = m_cPrevOrientation.Inverse() * cOrientation;
      /* Repeated quaternion products drift off the unit hypersphere by a
       * few ULPs; left uncorrected, W can end up microscopically outside
       * [-1,1] and ToAngleAxis's ACos(W) then returns NaN (ACos has no
       * domain clamp of its own). Normalizing bounds the drift, and the
       * explicit clamp catches the residual rounding error normalization
       * itself can leave behind. */
      cRelOrientation.Normalize();
      cRelOrientation.SetW(Min(Real(1.0), Max(Real(-1.0), cRelOrientation.GetW())));
      CRadians cAngle;
      CVector3 cAxis;
      cRelOrientation.ToAngleAxis(cAngle, cAxis);
      CVector3 cAngularVelocityBody = cAxis * (cAngle.GetValue() / fDeltaT);
      if(m_bAddNoise) {
         m_cGyroBias += CVector3(m_pcRNG->Gaussian(m_fGyroBiasWalkStdDev),
                                 m_pcRNG->Gaussian(m_fGyroBiasWalkStdDev),
                                 m_pcRNG->Gaussian(m_fGyroBiasWalkStdDev));
         m_cAccelBias += CVector3(m_pcRNG->Gaussian(m_fAccelBiasWalkStdDev),
                                  m_pcRNG->Gaussian(m_fAccelBiasWalkStdDev),
                                  m_pcRNG->Gaussian(m_fAccelBiasWalkStdDev));
         cAngularVelocityBody += m_cGyroBias +
            CVector3(m_pcRNG->Gaussian(m_fGyroNoiseStdDev),
                    m_pcRNG->Gaussian(m_fGyroNoiseStdDev),
                    m_pcRNG->Gaussian(m_fGyroNoiseStdDev));
         cSpecificForceBody += m_cAccelBias +
            CVector3(m_pcRNG->Gaussian(m_fAccelNoiseStdDev),
                    m_pcRNG->Gaussian(m_fAccelNoiseStdDev),
                    m_pcRNG->Gaussian(m_fAccelNoiseStdDev));
      }
      m_sReading.AngularVelocity = cAngularVelocityBody;
      m_sReading.LinearAcceleration = cSpecificForceBody;
      m_cPrevPosition = cPosition;
      m_cPrevOrientation = cOrientation;
      m_cPrevVelocity = cVelocity;
   }

   /****************************************/
   /****************************************/

   void CIMUDefaultSensor::Reset() {
      m_sReading.AngularVelocity = CVector3::ZERO;
      m_sReading.LinearAcceleration = CVector3(0.0, 0.0, m_fGravity);
      m_cGyroBias = CVector3::ZERO;
      m_cAccelBias = CVector3::ZERO;
      m_bHasPrevious = false;
      m_cPrevVelocity = CVector3::ZERO;
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CIMUDefaultSensor,
                   "imu", "default",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "A generic IMU (gyroscope + accelerometer) sensor.",

                   "This sensor returns the angular velocity and specific force (gravity-\n"
                   "compensated proper acceleration) of a robot, both expressed in the robot's\n"
                   "body frame. It is derived from the finite difference of the robot's\n"
                   "ground-truth anchor pose across ticks, so it works with any physics engine\n"
                   "and needs no engine-specific velocity readout. This sensor can be used with\n"
                   "any robot, since it accesses only the body component. In controllers, you\n"
                   "must include the ci_imu_sensor.h header.\n\n"

                   "This sensor is enabled by default.\n\n"

                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <imu implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"

                   "OPTIONAL XML CONFIGURATION\n\n"

                   "The attribute 'gravity' (default 9.81) sets the gravity magnitude used to\n"
                   "compute specific force from world-frame acceleration. Attributes\n"
                   "'gyro_noise_std_dev' and 'accel_noise_std_dev' set the standard deviation of\n"
                   "zero-mean Gaussian white noise added per axis, per tick (rad/s and m/s^2\n"
                   "respectively). Attributes 'gyro_bias_walk_std_dev' and\n"
                   "'accel_bias_walk_std_dev' set the standard deviation of a per-axis Gaussian\n"
                   "random walk added to a persistent bias term each tick, modelling slowly\n"
                   "drifting IMU bias.\n\n"

                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <imu implementation=\"default\"\n"
                   "             gravity=\"9.81\"\n"
                   "             gyro_noise_std_dev=\"0.001\"\n"
                   "             accel_noise_std_dev=\"0.01\"\n"
                   "             gyro_bias_walk_std_dev=\"0.0001\"\n"
                   "             accel_bias_walk_std_dev=\"0.001\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",

                   "Usable"
                  );

}
