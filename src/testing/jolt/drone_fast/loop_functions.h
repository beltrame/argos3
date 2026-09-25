#ifndef JOLT_DRONE_FAST_LOOP_FUNCTIONS_H
#define JOLT_DRONE_FAST_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CDroneEntity;
}

using namespace argos;

/** With max_xy_velocity = 5 the drone flies well past the 1 m/s default,
 *  stays within the tilt limit, and still settles on its target. */
class CDroneFastLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual bool IsExperimentFinished();

private:

   CDroneEntity* m_pcDrone = nullptr;
   Real m_fPeakSpeed = 0.0;
   CVector3 m_cPrevious;

};

#endif
