/**
 * @file <argos3/plugins/robots/foot-bot/simulator/jolt_footbot_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_FOOTBOT_MODEL_H
#define JOLT_FOOTBOT_MODEL_H

namespace argos {
   class CJoltFootBotModel;
   class CFootBotEntity;
   class CWheeledEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>

namespace argos {

   /**
    * The foot-bot in the Jolt engine: a single dynamic cylinder with
    * differential-drive kinematics. The wheel velocities from the
    * controller are applied as body velocities (like dynamics2d, so
    * existing controllers behave the same), the rotation is locked to
    * the z axis, and gravity keeps the robot on the floor.
    */
   class CJoltFootBotModel : public CJoltSingleBodyObjectModel {

   public:

      CJoltFootBotModel(CJoltEngine& c_engine,
                        CFootBotEntity& c_entity);

      virtual ~CJoltFootBotModel() {}

      virtual void UpdateFromEntityStatus();

      void UpdateTurretAnchor(SAnchor& s_anchor);

      void UpdatePerspectiveCameraAnchor(SAnchor& s_anchor);

   private:

      CFootBotEntity& m_cFootBotEntity;
      CWheeledEntity& m_cWheeledEntity;

   };

}

#endif
