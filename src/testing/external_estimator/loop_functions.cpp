/**
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "loop_functions.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_odometry_sensor.h>

#include <cmath>

namespace argos {

   /* Must match stub_estimator.py */
   static const Real STUB_POSE_X_PER_TICK = 0.001;
   static const Real STUB_POSE_Y = 2.0;
   static const Real STUB_POSE_Z = 3.0;
   static const Real STUB_TWIST[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

   /* The foot-bot starts here, and the wheel odometry starts at its own
    * origin, so the two frames differ by exactly this offset */
   static const CVector3 START_POSITION(-0.5, 0.0, 0.0);

   /* Total ticks the experiment runs for */
   static const UInt32 RUN_TICKS = 100;

   /****************************************/
   /****************************************/

   static const CCI_OdometrySensor::SReading& GetOdometry(CFootBotEntity& c_footbot) {
      return c_footbot.GetControllableEntity().GetController()
         .GetSensor<CCI_OdometrySensor>("odometry")->GetReading();
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorTestLoopFunctions::Init(TConfigurationNode& t_tree) {
      m_pcFootBot = &dynamic_cast<CFootBotEntity&>(GetSpace().GetEntity("fb"));
      GetNodeAttribute(t_tree, "mode", m_strMode);
      GetNodeAttributeOrDefault(t_tree, "valid_from", m_unValidFrom, m_unValidFrom);
      if(m_strMode != "protocol" && m_strMode != "wheels" &&
         m_strMode != "aligned") {
         THROW_ARGOSEXCEPTION("Unknown mode \"" << m_strMode
                              << "\"; expected \"protocol\", \"wheels\" or "
                              "\"aligned\"");
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorTestLoopFunctions::PostStep() {
      if(m_strMode == "protocol") {
         CheckProtocol();
      }
      else if(m_strMode == "aligned") {
         CheckAligned();
      }
      else {
         CheckWheels();
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorTestLoopFunctions::CheckProtocol() {
      const CCI_OdometrySensor::SReading& sReading = GetOdometry(*m_pcFootBot);
      UInt32 unClock = UInt32(GetSpace().GetSimulationClock());
      /*
       * Media update before the sense phase, so the frame sent during
       * tick N carries the readings of tick N-1 and the estimate that
       * comes back is stamped N-1. The sensor picks it up during the
       * same tick's sense phase. The reading at tick N is therefore the
       * estimate for tick N-1, and the stub only marks poses valid from
       * m_unValidFrom onwards.
       */
      bool bExpectValid = unClock >= 2 && (unClock - 1) >= m_unValidFrom;
      if(sReading.Valid != bExpectValid) {
         THROW_ARGOSEXCEPTION("At tick " << unClock << " the external odometry "
                              "reports Valid=" << sReading.Valid
                              << ", expected " << bExpectValid
                              << ". The stub reports poses from tick "
                              << m_unValidFrom << " onwards, and the round trip "
                              "costs exactly one tick.");
      }
      if(!bExpectValid) {
         return;
      }
      ++m_unValidReadings;
      UInt32 unExpectedTick = unClock - 1;
      if(sReading.Tick != unExpectedTick) {
         THROW_ARGOSEXCEPTION("At tick " << unClock << " the external odometry "
                              "reports an estimate for tick " << sReading.Tick
                              << ", expected " << unExpectedTick
                              << ". The one-tick round trip is broken.");
      }
      CVector3 cExpected(unExpectedTick * STUB_POSE_X_PER_TICK,
                         STUB_POSE_Y, STUB_POSE_Z);
      if(Distance(sReading.Position, cExpected) > 1e-9) {
         THROW_ARGOSEXCEPTION("At tick " << unClock << " the external odometry "
                              "reports position " << sReading.Position
                              << ", expected " << cExpected
                              << ". The pose did not survive the wire intact.");
      }
      CVector3 cExpectedLinear(STUB_TWIST[0], STUB_TWIST[1], STUB_TWIST[2]);
      CVector3 cExpectedAngular(STUB_TWIST[3], STUB_TWIST[4], STUB_TWIST[5]);
      if(Distance(sReading.LinearVelocity, cExpectedLinear) > 1e-9 ||
         Distance(sReading.AngularVelocity, cExpectedAngular) > 1e-9) {
         THROW_ARGOSEXCEPTION("At tick " << unClock << " the external odometry "
                              "reports twist " << sReading.LinearVelocity << " / "
                              << sReading.AngularVelocity << ", expected "
                              << cExpectedLinear << " / " << cExpectedAngular);
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorTestLoopFunctions::CheckWheels() {
      const CCI_OdometrySensor::SReading& sReading = GetOdometry(*m_pcFootBot);
      if(!sReading.Valid) {
         return;
      }
      ++m_unValidReadings;
      /* The stub echoes back the wheel-encoder pose the medium
       * dead-reckoned. It is expressed in the robot's own start frame,
       * so shift it into world coordinates before comparing. */
      CVector3 cEstimated = sReading.Position + START_POSITION;
      const CVector3& cGroundTruth =
         m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
      /* Compare in the plane: the encoders cannot observe height, and
       * the foot-bot's body origin sits above the floor. */
      Real fError = std::hypot(cEstimated.GetX() - cGroundTruth.GetX(),
                               cEstimated.GetY() - cGroundTruth.GetY());
      if(fError > 0.02) {
         THROW_ARGOSEXCEPTION("At tick " << GetSpace().GetSimulationClock()
                              << " the wheel-encoder odometry says "
                              << cEstimated << " but the robot is at "
                              << cGroundTruth << " (planar error " << fError
                              << " m). A 100x error here means the "
                              "centimetre-to-metre conversion was lost.");
      }
   }

   /****************************************/
   /****************************************/

   void CExternalEstimatorTestLoopFunctions::CheckAligned() {
      const CCI_OdometrySensor::SReading& sReading = GetOdometry(*m_pcFootBot);
      if(!sReading.Valid) {
         return;
      }
      ++m_unValidReadings;
      /* The medium placed the estimate in the world frame, so no offset
       * is applied here: the reading must stand on its own against
       * ground truth. A frame composition that is transposed, inverted
       * or anchored at the wrong instant shows up immediately as a
       * constant offset of the robot's start pose. */
      const CVector3& cGroundTruth =
         m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
      Real fError = std::hypot(sReading.Position.GetX() - cGroundTruth.GetX(),
                               sReading.Position.GetY() - cGroundTruth.GetY());
      if(fError > 0.02) {
         THROW_ARGOSEXCEPTION("At tick " << GetSpace().GetSimulationClock()
                              << " the world-frame odometry says "
                              << sReading.Position << " but the robot is at "
                              << cGroundTruth << " (planar error " << fError
                              << " m). The estimator frame is not being placed "
                              "correctly in the world.");
      }
   }

   /****************************************/
   /****************************************/

   bool CExternalEstimatorTestLoopFunctions::IsExperimentFinished() {
      if(GetSpace().GetSimulationClock() < RUN_TICKS) {
         return false;
      }
      /* A run in which the sensor never reported anything would satisfy
       * every check above by doing nothing at all */
      if(m_unValidReadings < RUN_TICKS / 2) {
         THROW_ARGOSEXCEPTION("The external odometry only reported "
                              << m_unValidReadings << " valid readings in "
                              << RUN_TICKS << " ticks. Nothing is coming back "
                              "from the estimator.");
      }
      /* The robot must actually have moved, otherwise the wheel check
       * compares two stationary points and proves nothing */
      const CVector3& cPosition =
         m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position;
      if(Distance(cPosition, START_POSITION) < 0.5) {
         THROW_ARGOSEXCEPTION("The foot-bot only moved "
                              << Distance(cPosition, START_POSITION)
                              << " m; it should have driven about 1 m");
      }
      return true;
   }

   /****************************************/
   /****************************************/

   REGISTER_LOOP_FUNCTIONS(CExternalEstimatorTestLoopFunctions,
                           "external_estimator_test_loop_functions");

}
