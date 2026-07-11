#ifndef PHOTOREALISM_CAMERA_TEST_LOOP_FUNCTIONS_H
#define PHOTOREALISM_CAMERA_TEST_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CBoxEntity;
   class CPhotorealismMedium;
}

class CCameraTestController;

using namespace argos;

/**
 * Checks the camera frames captured by the test controller against
 * ground truth: the depth of a wall at a known distance, the
 * segmentation ids of the wall, and the frame latency.
 */
class CCameraTestLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();

private:

   CCameraTestController* m_pcController = nullptr;
   CBoxEntity* m_pcWall = nullptr;
   CPhotorealismMedium* m_pcMedium = nullptr;
   /* Expected (clock - frame tick), from XML: 1 = pipelined, 0 = immediate */
   UInt32 m_unExpectedLatency = 1;
   bool m_bChecked = false;

};

#endif
