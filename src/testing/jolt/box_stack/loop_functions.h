#ifndef JOLT_BOX_STACK_LOOP_FUNCTIONS_H
#define JOLT_BOX_STACK_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

#include <string>
#include <vector>

namespace argos {
   class CBoxEntity;
   class CCylinderEntity;
}

using namespace argos;

/**
 * Checks that a stack of boxes dropped under gravity settles into a
 * stable stack, and dumps the final poses with full precision for the
 * bitwise-determinism test.
 */
class CBoxStackLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void PostExperiment();

private:

   std::vector<CBoxEntity*> m_vecBoxes;
   CCylinderEntity* m_pcCylinder = nullptr;
   std::string m_strPosesFile;
   /* Positions at the previous tick, to measure settling */
   std::vector<CVector3> m_vecLastPositions;
   Real m_fMaxRecentMotion = 0.0;
   UInt32 m_unMotionWindow = 0;

};

#endif
