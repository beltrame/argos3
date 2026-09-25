#ifndef JOLT_DRONE_ARM_LOOP_FUNCTIONS_H
#define JOLT_DRONE_ARM_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CDroneEntity;
}

using namespace argos;

/**
 * A disarmed drone stays on the floor even with a target 1 m up; armed, it
 * climbs to the target; disarmed in the air, it falls: the motors are off.
 */
class CDroneArmLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual bool IsExperimentFinished();

private:

   CDroneEntity* m_pcDrone = nullptr;

};

#endif
