/**
 * @file <argos3/plugins/robots/generic/simulator/imu_default_sensor.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef IMU_DEFAULT_SENSOR_H
#define IMU_DEFAULT_SENSOR_H

namespace argos {
   class CIMUDefaultSensor;
   class CEmbodiedEntity;
}

#include <argos3/plugins/robots/generic/control_interface/ci_imu_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

   /**
    * Derives angular velocity and specific force from the finite
    * difference of the robot's ground-truth anchor pose across ticks.
    * This is deliberately physics-engine-agnostic (it works under any
    * engine, not just Jolt, since it only reads SAnchor::Position and
    * SAnchor::Orientation) at the cost of a one-tick lag versus reading
    * a physics engine's native body velocity directly.
    */
   class CIMUDefaultSensor : public CSimulatedSensor,
                             public CCI_IMUSensor {

   public:

      CIMUDefaultSensor();

      virtual ~CIMUDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   protected:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Gravity magnitude (m/s^2); world gravity vector is assumed (0,0,-gravity) */
      Real m_fGravity;

      /** White noise standard deviation, per axis, per tick */
      Real m_fGyroNoiseStdDev;
      Real m_fAccelNoiseStdDev;

      /** Bias random-walk standard deviation, per axis, per tick */
      Real m_fGyroBiasWalkStdDev;
      Real m_fAccelBiasWalkStdDev;

      /** Current accumulated bias, per axis */
      CVector3 m_cGyroBias;
      CVector3 m_cAccelBias;

      /** Whether a previous-tick sample is available for finite differencing */
      bool m_bHasPrevious;

      /** Previous-tick ground-truth state */
      CVector3 m_cPrevPosition;
      CQuaternion m_cPrevOrientation;
      CVector3 m_cPrevVelocity;
   };

}

#endif
