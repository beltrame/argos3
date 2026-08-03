/**
 * @file <argos3/plugins/robots/generic/simulator/odometry_default_sensor.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef ODOMETRY_DEFAULT_SENSOR_H
#define ODOMETRY_DEFAULT_SENSOR_H

namespace argos {
   class COdometryDriftSensor;
   class CEmbodiedEntity;
}

#include <argos3/plugins/robots/generic/control_interface/ci_odometry_sensor.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

   /**
    * Dead-reckoning odometry: each tick, the ground-truth relative
    * motion (in the previous ground-truth body frame) is perturbed by
    * Gaussian noise proportional to the distance travelled and then
    * integrated forward through the sensor's own drifted frame, exactly
    * as a real wheel/visual odometry pipeline would drift. The reading
    * therefore diverges from ground truth over time even though each
    * individual step is only lightly perturbed.
    */
   class COdometryDriftSensor : public CSimulatedSensor,
                                public CCI_OdometrySensor {

   public:

      COdometryDriftSensor();

      virtual ~COdometryDriftSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   protected:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add drift or not */
      bool m_bAddDrift;

      /** Position drift, as a fraction of the distance travelled this tick,
          applied as Gaussian noise standard deviation per axis */
      Real m_fPositionDriftStdDev;

      /** Yaw drift, in radians per metre travelled, applied as Gaussian
          noise standard deviation */
      Real m_fOrientationDriftStdDev;

      /** Whether a previous-tick ground-truth sample is available */
      bool m_bHasPrevious;

      /** Previous-tick ground-truth state, used to compute the relative
          motion applied this tick */
      CVector3 m_cPrevGroundTruthPosition;
      CQuaternion m_cPrevGroundTruthOrientation;
   };

}

#endif
