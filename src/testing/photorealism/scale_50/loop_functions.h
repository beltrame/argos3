#ifndef PHOTOREALISM_SCALE_50_LOOP_FUNCTIONS_H
#define PHOTOREALISM_SCALE_50_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

#include <vector>

using namespace argos;

class CCameraTestController;

/**
 * Checks that every camera in the 50-robot scale experiment keeps
 * delivering frames at the expected cadence.
 */
class CScale50LoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostExperiment();

private:

   std::vector<CCameraTestController*> m_vecControllers;

};

#endif
