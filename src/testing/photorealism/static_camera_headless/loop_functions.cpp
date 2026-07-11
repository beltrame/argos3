#include "loop_functions.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

/****************************************/
/****************************************/

void CPhotorealismTestLoopFunctions::Init(TConfigurationNode& t_tree) {
   m_pcBox = &dynamic_cast<CBoxEntity&>(GetSpace().GetEntity("moving_box"));
}

/****************************************/
/****************************************/

void CPhotorealismTestLoopFunctions::PreStep() {
   Real fAngle = 0.1 * GetSpace().GetSimulationClock();
   CVector3 cPosition(0.8 * Cos(CRadians(fAngle)),
                      0.8 * Sin(CRadians(fAngle)),
                      0.0);
   MoveEntity(m_pcBox->GetEmbodiedEntity(),
              cPosition,
              CQuaternion(CRadians(fAngle), CVector3::Z));
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CPhotorealismTestLoopFunctions,
                        "photorealism_test_loop_functions");
