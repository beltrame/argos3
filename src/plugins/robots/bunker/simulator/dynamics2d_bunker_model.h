/**
 * @file <argos3/plugins/robots/bunker/simulator/dynamics2d_bunker_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef DYNAMICS2D_BUNKER_MODEL_H
#define DYNAMICS2D_BUNKER_MODEL_H

namespace argos {
   class CDynamics2DVelocityPlayer;
   class CDynamics2DBunkerModel;
   class CBunkerEntity;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include "bunker_entity.h"


namespace argos {

   class CDynamics2DBunkerModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DBunkerModel(CDynamics2DEngine& c_engine,
                                 CBunkerEntity& c_entity);

      virtual ~CDynamics2DBunkerModel() {}

      virtual void Reset();

      virtual void UpdateFromEntityStatus();

   private:

      void UpdateAuxiliaryAnchor(SAnchor& s_anchor);

      CBunkerEntity& m_cBunkerEntity;
      CWheeledEntity& m_cWheeledEntity;
      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      static const Real BUNKER_LENGTH;
      static const Real BUNKER_WIDTH;
      static const Real BUNKER_HEIGHT;
      static const Real BUNKER_MASS;
      static const Real BUNKER_MAX_FORCE;
      static const Real BUNKER_MAX_TORQUE;
      static const Real BUNKER_TRACK_GAUGE;

   };

}

#endif
