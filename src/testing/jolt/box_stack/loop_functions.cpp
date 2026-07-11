#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>

#include <cmath>
#include <cstdio>

/****************************************/
/****************************************/

void CBoxStackLoopFunctions::Init(TConfigurationNode& t_tree) {
   GetNodeAttribute(t_tree, "poses_file", m_strPosesFile);
   for(UInt32 i = 0; i < 3; ++i) {
      m_vecBoxes.push_back(
         &dynamic_cast<CBoxEntity&>(
            GetSpace().GetEntity("stack_" + std::to_string(i))));
   }
   m_pcCylinder = &dynamic_cast<CCylinderEntity&>(GetSpace().GetEntity("roller"));
}

/****************************************/
/****************************************/

void CBoxStackLoopFunctions::PostStep() {
   std::vector<CVector3> vecPositions;
   for(CBoxEntity* pc_box : m_vecBoxes) {
      vecPositions.push_back(
         pc_box->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
   vecPositions.push_back(
      m_pcCylinder->GetEmbodiedEntity().GetOriginAnchor().Position);
   /* Track the largest per-tick motion over the last second */
   if(!m_vecLastPositions.empty()) {
      Real fMotion = 0.0;
      for(size_t i = 0; i < vecPositions.size(); ++i) {
         fMotion = std::max(fMotion,
                            (vecPositions[i] - m_vecLastPositions[i]).Length());
      }
      if(GetSpace().GetSimulationClock() + 10 >
         CSimulator::GetInstance().GetMaxSimulationClock()) {
         m_fMaxRecentMotion = std::max(m_fMaxRecentMotion, fMotion);
         ++m_unMotionWindow;
      }
   }
   m_vecLastPositions = vecPositions;
}

/****************************************/
/****************************************/

void CBoxStackLoopFunctions::PostExperiment() {
   /* The boxes must have settled into a stack: box k rests at height
    * k * 0.2 (within a small tolerance for solver drift) */
   for(UInt32 i = 0; i < 3; ++i) {
      const CVector3& cPosition =
         m_vecBoxes[i]->GetEmbodiedEntity().GetOriginAnchor().Position;
      Real fExpectedZ = 0.2 * i;
      if(std::fabs(cPosition.GetZ() - fExpectedZ) > 0.02) {
         THROW_ARGOSEXCEPTION("Box stack_" << i << " is at z = "
                              << cPosition.GetZ() << ", expected ~"
                              << fExpectedZ << " (stack toppled?)");
      }
   }
   /* The cylinder must rest on the floor */
   Real fCylinderZ =
      m_pcCylinder->GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();
   if(std::fabs(fCylinderZ) > 0.02) {
      THROW_ARGOSEXCEPTION("Cylinder rests at z = " << fCylinderZ
                           << ", expected ~0");
   }
   /* Everything must be at rest at the end */
   if(m_unMotionWindow == 0 || m_fMaxRecentMotion > 1e-4) {
      THROW_ARGOSEXCEPTION("Bodies still moving at the end of the experiment ("
                           << m_fMaxRecentMotion << " m/tick)");
   }
   /* Dump the final poses in full precision (%a is exact) for the
    * bitwise-determinism comparison */
   std::FILE* ptFile = std::fopen(m_strPosesFile.c_str(), "w");
   if(ptFile == nullptr) {
      THROW_ARGOSEXCEPTION("Cannot write \"" << m_strPosesFile << "\"");
   }
   auto fnDump = [ptFile](const char* pch_id, CEmbodiedEntity& c_body) {
      const CVector3& cPosition = c_body.GetOriginAnchor().Position;
      const CQuaternion& cOrientation = c_body.GetOriginAnchor().Orientation;
      std::fprintf(ptFile, "%s %a %a %a %a %a %a %a\n", pch_id,
                   cPosition.GetX(), cPosition.GetY(), cPosition.GetZ(),
                   cOrientation.GetW(), cOrientation.GetX(),
                   cOrientation.GetY(), cOrientation.GetZ());
   };
   for(UInt32 i = 0; i < 3; ++i) {
      fnDump(m_vecBoxes[i]->GetId().c_str(),
             m_vecBoxes[i]->GetEmbodiedEntity());
   }
   fnDump("roller", m_pcCylinder->GetEmbodiedEntity());
   std::fclose(ptFile);
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CBoxStackLoopFunctions, "box_stack_loop_functions");
