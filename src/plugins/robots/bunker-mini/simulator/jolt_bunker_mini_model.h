/**
 * @file <argos3/plugins/robots/bunker-mini/simulator/jolt_bunker_mini_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_BUNKER_MINI_MODEL_H
#define JOLT_BUNKER_MINI_MODEL_H

namespace argos {
   class CJoltBunkerMiniModel;
   class CBunkerMiniEntity;
   class CWheeledEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>

namespace argos {

   class CJoltBunkerMiniModel : public CJoltSingleBodyObjectModel {

   public:

      CJoltBunkerMiniModel(CJoltEngine& c_engine,
                           CBunkerMiniEntity& c_entity);

      virtual ~CJoltBunkerMiniModel() {}

      virtual void UpdateFromEntityStatus();

      void UpdateAuxiliaryAnchor(SAnchor& s_anchor);

   private:

      CBunkerMiniEntity& m_cBunkerMiniEntity;
      CWheeledEntity& m_cWheeledEntity;

      static const Real BUNKER_MINI_LENGTH;
      static const Real BUNKER_MINI_WIDTH;
      static const Real BUNKER_MINI_HEIGHT;
      static const Real BUNKER_MINI_MASS;
      static const Real BUNKER_MINI_TRACK_GAUGE;

   };

}

#endif
