/**
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef EXTERNAL_ESTIMATOR_TEST_CONTROLLER_H
#define EXTERNAL_ESTIMATOR_TEST_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>

namespace argos {

   /**
    * Drives straight forward, so the wheel encoders have something to
    * count. The sensors it declares are what the <external_estimator>
    * medium finds and streams; the assertions live in the loop
    * functions.
    */
   class CExternalEstimatorTestController : public CCI_Controller {

   public:

      CExternalEstimatorTestController() {}

      virtual ~CExternalEstimatorTestController() {}

      virtual void Init(TConfigurationNode& t_tree);

   };

}

#endif
