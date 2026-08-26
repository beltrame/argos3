/**
 * @file <argos3/plugins/robots/spot/simulator/dynamics2d_spot_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef DYNAMICS2D_SPOT_MODEL_H
#define DYNAMICS2D_SPOT_MODEL_H

namespace argos {
   class CDynamics2DVelocityPlayer;
   class CDynamics2DSpotModel;
   class CSpotEntity;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include "spot_entity.h"


namespace argos {

   class CDynamics2DSpotModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DSpotModel(CDynamics2DEngine& c_engine,
                                 CSpotEntity& c_entity);

      virtual ~CDynamics2DSpotModel() {}

      virtual void Reset();

      virtual void UpdateFromEntityStatus();

   private:

      void UpdateAuxiliaryAnchor(SAnchor& s_anchor);

      CSpotEntity& m_cSpotEntity;
      CWheeledEntity& m_cWheeledEntity;
      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      static const Real SPOT_LENGTH;
      static const Real SPOT_WIDTH;
      static const Real SPOT_HEIGHT;
      static const Real SPOT_MASS;
      static const Real SPOT_MAX_FORCE;
      static const Real SPOT_MAX_TORQUE;
      static const Real SPOT_TRACK_GAUGE;

   };

}

#endif
