#ifndef JOLT_DRONE_IMU_LOOP_FUNCTIONS_H
#define JOLT_DRONE_IMU_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>

namespace argos {
   class CDroneEntity;
}

using namespace argos;

/**
 * Checks that the drone takes off, flies at the XY velocity limit, stays
 * upright, and converges to a hover at the target position (same flight
 * profile as the drone_fly test), and additionally checks the IMU sensor
 * added on top of it: during the climb/acceleration phase the specific
 * force must visibly differ from pure gravity (proving the finite
 * difference actually tracks the dynamics, not just returning its
 * initial value), and once hovering it must read close to gravity with
 * a small angular rate.
 */
class CDroneImuLoopFunctions : public CLoopFunctions {

public:

   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual bool IsExperimentFinished();

private:

   CDroneEntity* m_pcDrone = nullptr;

};

#endif
