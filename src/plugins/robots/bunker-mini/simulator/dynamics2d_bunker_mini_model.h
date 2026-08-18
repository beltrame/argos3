/**
 * @file <argos3/plugins/robots/bunker-mini/simulator/dynamics2d_bunker_mini_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef DYNAMICS2D_BUNKER_MINI_MODEL_H
#define DYNAMICS2D_BUNKER_MINI_MODEL_H

namespace argos {
   class CDynamics2DVelocityPlayer;
   class CDynamics2DBunkerMiniModel;
   class CBunkerMiniEntity;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include "bunker_mini_entity.h"


namespace argos {

   class CDynamics2DBunkerMiniModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DBunkerMiniModel(CDynamics2DEngine& c_engine,
                                 CBunkerMiniEntity& c_entity);

      virtual ~CDynamics2DBunkerMiniModel() {}

      virtual void Reset();

      virtual void UpdateFromEntityStatus();

   private:

      void UpdateAuxiliaryAnchor(SAnchor& s_anchor);

      CBunkerMiniEntity& m_cBunkerMiniEntity;
      CWheeledEntity& m_cWheeledEntity;
      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      static const Real BUNKER_MINI_LENGTH;
      static const Real BUNKER_MINI_WIDTH;
      static const Real BUNKER_MINI_HEIGHT;
      static const Real BUNKER_MINI_MASS;
      static const Real BUNKER_MINI_MAX_FORCE;
      static const Real BUNKER_MINI_MAX_TORQUE;
      static const Real BUNKER_MINI_TRACK_GAUGE;

   };

}

#endif
