/**
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "controller.h"

#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

namespace argos {

   /****************************************/
   /****************************************/

   void CExternalEstimatorTestController::Init(TConfigurationNode& t_tree) {
      CCI_DifferentialSteeringActuator* pcActuator =
         GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      /* 10 cm/s straight ahead: enough for the encoders to accumulate a
       * pose the test can compare against ground truth */
      pcActuator->SetLinearVelocity(10.0, 10.0);
   }

   /****************************************/
   /****************************************/

   REGISTER_CONTROLLER(CExternalEstimatorTestController,
                       "external_estimator_test_controller");

}
