#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/photorealism/photorealism_medium.h>

/****************************************/
/****************************************/

void CRandomizationLoopFunctions::Init(TConfigurationNode& t_tree) {
   m_pcMedium = &CSimulator::GetInstance().GetMedium<CPhotorealismMedium>("pr");
   m_pcBox = &dynamic_cast<CBoxEntity&>(GetSpace().GetEntity("box_1"));
}

/****************************************/
/****************************************/

void CRandomizationLoopFunctions::PreStep() {
   UInt32 unClock = GetSpace().GetSimulationClock();
   if(unClock == 1) {
      /* Exercise the per-entity material API (deterministic values) */
      m_pcMedium->SetMaterialColor(m_pcBox->GetEmbodiedEntity(),
                                   CVector3(0.8, 0.25, 0.2));
      m_pcMedium->SetMaterialParam(m_pcBox->GetEmbodiedEntity(),
                                   "roughness", 0.35);
   }
   if(unClock == 3) {
      /* Exercise the full redraw API (deterministic per seed) */
      m_pcMedium->RandomizeAll();
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CRandomizationLoopFunctions,
                        "randomization_loop_functions");
