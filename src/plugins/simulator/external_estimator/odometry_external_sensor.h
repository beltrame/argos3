/**
 * @file <argos3/plugins/simulator/external_estimator/odometry_external_sensor.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef ODOMETRY_EXTERNAL_SENSOR_H
#define ODOMETRY_EXTERNAL_SENSOR_H

namespace argos {
   class COdometryExternalSensor;
}

#include <argos3/plugins/robots/generic/control_interface/ci_odometry_sensor.h>
#include <argos3/plugins/simulator/external_estimator/external_estimator_medium.h>
#include <argos3/core/simulator/sensor.h>

#include <string>

namespace argos {

   /**
    * Odometry computed by a SLAM front-end running outside ARGoS.
    *
    * This sensor estimates nothing. It reports whatever pose the
    * <external_estimator> medium last received for this robot over its
    * socket, so the drift a controller sees is the drift a real
    * algorithm actually produced from this robot's simulated IMU,
    * camera, lidar and wheel encoders, rather than a statistical model
    * of drift (implementation "drift").
    *
    * The reading is invalid until the estimator produces its first pose.
    * Real estimators need to initialize, so controllers must check
    * SReading::Valid instead of assuming a pose from tick 0.
    */
   class COdometryExternalSensor : public CSimulatedSensor,
                                   public CCI_OdometrySensor {

   public:

      COdometryExternalSensor();

      virtual ~COdometryExternalSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   private:


      /** The medium holding this robot's estimates */
      CExternalEstimatorMedium* m_pcMedium;

      /** This robot's id, the key the medium files estimates under */
      std::string m_strRobotId;



      /** When the estimator reports nothing new, whether to keep the
       *  last pose (true, what a real consumer of an odometry topic
       *  sees) or blank the reading back to invalid (false) */
      bool m_bHoldLast;

   };

}

#endif
