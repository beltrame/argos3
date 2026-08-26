/**
 * @file <argos3/plugins/robots/scout-mini/simulator/dynamics2d_scout_mini_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef DYNAMICS2D_SCOUT_MINI_MODEL_H
#define DYNAMICS2D_SCOUT_MINI_MODEL_H

namespace argos {
   class CDynamics2DVelocityPlayer;
   class CDynamics2DScoutMiniModel;
   class CScoutMiniEntity;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include "scout_mini_entity.h"


namespace argos {

   class CDynamics2DScoutMiniModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DScoutMiniModel(CDynamics2DEngine& c_engine,
                                 CScoutMiniEntity& c_entity);

      virtual ~CDynamics2DScoutMiniModel() {}

      virtual void Reset();

      virtual void UpdateFromEntityStatus();

   private:

      void UpdateAuxiliaryAnchor(SAnchor& s_anchor);

      CScoutMiniEntity& m_cScoutMiniEntity;
      CWheeledEntity& m_cWheeledEntity;
      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      static const Real SCOUT_MINI_LENGTH;
      static const Real SCOUT_MINI_WIDTH;
      static const Real SCOUT_MINI_HEIGHT;
      static const Real SCOUT_MINI_MASS;
      static const Real SCOUT_MINI_MAX_FORCE;
      static const Real SCOUT_MINI_MAX_TORQUE;
      static const Real SCOUT_MINI_TRACK_GAUGE;

   };

}

#endif
