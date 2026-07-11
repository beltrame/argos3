#ifndef JOLT_DRONE_FLY_LOOP_FUNCTIONS_H
#define JOLT_DRONE_FLY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CDroneEntity;
}

using namespace argos;

/**
 * Checks that the drone takes off, flies at the XY velocity limit
 * (like the pointmass3d model), stays upright, and converges to a
 * hover at the target position.
 */
class CDroneFlyLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual bool IsExperimentFinished();

private:

   CDroneEntity* m_pcDrone = nullptr;

};

#endif
