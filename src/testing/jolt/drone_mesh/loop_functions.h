#ifndef JOLT_DRONE_MESH_LOOP_FUNCTIONS_H
#define JOLT_DRONE_MESH_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {
   class CDroneEntity;
}

using namespace argos;

/**
 * The drone against static mesh geometry, no floor plugin:
 * - it never sinks below `min_z` (the mesh floor carries it);
 * - between `wall_from` and `wall_to` it is sent through a wall: it must
 *   touch it (collision reported), come within `wall_reach_y` of it, and
 *   never pass `wall_limit_y`;
 * - at `end_tick` it rests upright at `expect_final` (world frame).
 */
class CDroneMeshLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual bool IsExperimentFinished();

private:

   CDroneEntity* m_pcDrone = nullptr;
   UInt32 m_unWallFrom = 0;
   UInt32 m_unWallTo = 0;
   Real m_fWallLimitY = 0.0;
   Real m_fWallReachY = 0.0;
   Real m_fMinZ = 0.0;
   UInt32 m_unEndTick = 0;
   CVector3 m_cExpectFinal;
   Real m_fFinalTolerance = 0.15;
   bool m_bCollided = false;
   Real m_fMaxWallY = -1e9;

};

#endif
