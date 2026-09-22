/**
 * @file <argos3/plugins/robots/spot/simulator/jolt_spot_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_SPOT_MODEL_H
#define JOLT_SPOT_MODEL_H

namespace argos {
   class CJoltSpotModel;
   class CSpotEntity;
   class CWheeledEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_ground_robot_model.h>

namespace argos {

   class CJoltSpotModel : public CJoltGroundRobotModel {

   public:

      CJoltSpotModel(CJoltEngine& c_engine,
                           CSpotEntity& c_entity);

      virtual ~CJoltSpotModel() {}

      /* Set contact-surface drive targets without overriding chassis motion. */
      virtual void UpdateFromEntityStatus() override;

      void UpdateAuxiliaryAnchor(SAnchor& s_anchor);

   private:

      CSpotEntity& m_cSpotEntity;
      CWheeledEntity& m_cWheeledEntity;

      static const Real SPOT_LENGTH;
      static const Real SPOT_WIDTH;
      static const Real SPOT_HEIGHT;
      static const Real SPOT_MASS;
      static const Real SPOT_TRACK_GAUGE;

   };

}

#endif
