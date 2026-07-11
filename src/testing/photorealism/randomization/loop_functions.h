#ifndef PHOTOREALISM_RANDOMIZATION_LOOP_FUNCTIONS_H
#define PHOTOREALISM_RANDOMIZATION_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CPhotorealismMedium;
   class CBoxEntity;
}

using namespace argos;

/**
 * Exercises the domain randomization loop-function API; the actual
 * verification (same seed = identical images, different seed =
 * different lighting but identical segmentation) is done by the test
 * script comparing the dumped frames.
 */
class CRandomizationLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PreStep();

private:

   CPhotorealismMedium* m_pcMedium = nullptr;
   CBoxEntity* m_pcBox = nullptr;

};

#endif
