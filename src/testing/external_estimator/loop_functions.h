/**
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef EXTERNAL_ESTIMATOR_TEST_LOOP_FUNCTIONS_H
#define EXTERNAL_ESTIMATOR_TEST_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CFootBotEntity;
}

namespace argos {

   /**
    * Asserts what <odometry implementation="external"> reports, against
    * a stub estimator whose replies are known exactly (see
    * stub_estimator.py). Two modes:
    *
    * - "protocol" checks the wire contract: the pose, twist, tick and
    *   validity that come back are exactly what the stub sent, and the
    *   one-tick round trip holds.
    * - "aligned" is "wheels" with the medium placing the estimate in the
    *   world frame, so the reading must match ground truth with NO
    *   manual offset. That exercises the frame composition itself.
    * - "wheels" has the stub echo the wheel-encoder pose ARGoS
    *   dead-reckoned, so the reading can be compared against ground
    *   truth. That checks the differential-drive integration, including
    *   the centimetre-to-metre conversion the sensor's units demand.
    */
   class CExternalEstimatorTestLoopFunctions : public CLoopFunctions {

   public:

      virtual void Init(TConfigurationNode& t_tree);

      virtual void PostStep();

      virtual bool IsExperimentFinished();

   private:

      void CheckProtocol();
      void CheckWheels();
      void CheckAligned();

   private:

      CFootBotEntity* m_pcFootBot = nullptr;
      /** "protocol" or "wheels" */
      std::string m_strMode;
      /** First tick the stub reports a valid pose for */
      UInt32 m_unValidFrom = 5;
      /** Number of ticks on which a valid reading was seen, so a test
       *  that silently never delivers anything cannot pass */
      UInt32 m_unValidReadings = 0;

   };

}

#endif
