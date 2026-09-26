#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>
#include <cmath>
#include <iostream>

using namespace argos;

/* A 3 m/s ramp held at 10 Hz, with the 1 s position-loop lead. Clipping
 * the derivative of each setpoint jump biases attitude between updates;
 * the drone then needs ~1.5 m extra position error just to maintain speed. */
class CDroneTrackingLoopFunctions : public CLoopFunctions {
public:
   void Init(TConfigurationNode& t_tree) override {
      m_pcDrone = &dynamic_cast<CDroneEntity&>(GetSpace().GetEntity("drone"));
      GetNodeAttribute(t_tree, "axis", m_strAxis);
   }
   void PostStep() override {
      UInt32 unTick = GetSpace().GetSimulationClock();
      if(unTick < 1100) return; // 8 s to settle after the ramp starts
      const auto& cPosition = m_pcDrone->GetEmbodiedEntity().GetOriginAnchor().Position;
      Real fPosition = m_strAxis == "x" ? cPosition.GetX() : cPosition.GetY();
      Real fError = 3.0 * (Real(unTick) / 100.0 - 3.0) - fPosition;
      m_fErrorSum += fError;
      m_fMaxError = std::max(m_fMaxError, std::abs(fError));
      ++m_unSamples;
   }
   bool IsExperimentFinished() override {
      if(GetSpace().GetSimulationClock() < 1500) return false;
      Real fMean = m_fErrorSum / m_unSamples;
      std::cout << "[drone_tracking " << m_strAxis << "] mean " << fMean
                << " m, max " << m_fMaxError << " m" << std::endl;
      /* Half a 10 Hz command interval costs 0.15 m; allow only that
       * sample-and-hold error, not a derivative-induced steady bias. */
      if(std::abs(fMean) > 0.25 || m_fMaxError > 0.35) {
         THROW_ARGOSEXCEPTION("Staircase setpoints biased the attitude controller");
      }
      return true;
   }
private:
   CDroneEntity* m_pcDrone = nullptr;
   std::string m_strAxis;
   Real m_fErrorSum = 0.0, m_fMaxError = 0.0;
   UInt32 m_unSamples = 0;
};
REGISTER_LOOP_FUNCTIONS(CDroneTrackingLoopFunctions, "drone_tracking_loop_functions");
