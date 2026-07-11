#ifndef PHOTOREALISM_TEST_LOOP_FUNCTIONS_H
#define PHOTOREALISM_TEST_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CBoxEntity;
}

using namespace argos;

/**
 * Moves a box along a circle every tick, so that the dumped frames
 * prove that entity transforms are synced into the rendered scene.
 */
class CPhotorealismTestLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PreStep();

private:

   CBoxEntity* m_pcBox = nullptr;

};

#endif
