#ifndef JOLT_FOOTBOT_DRIVE_LOOP_FUNCTIONS_H
#define JOLT_FOOTBOT_DRIVE_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CFootBotEntity;
}

using namespace argos;

/**
 * Checks that the foot-bot drives 1 m forward in 10 s under the Jolt
 * engine (dynamics2d semantics), stays on the floor, and that ray
 * casts hit it.
 */
class CFootBotDriveLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual bool IsExperimentFinished();

private:

   CFootBotEntity* m_pcFootBot = nullptr;

};

#endif
