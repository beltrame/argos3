/**
 * @file <argos3/plugins/robots/bunker/simulator/jolt_bunker_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_BUNKER_MODEL_H
#define JOLT_BUNKER_MODEL_H

namespace argos {
   class CJoltBunkerModel;
   class CBunkerEntity;
   class CWheeledEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>

namespace argos {

   class CJoltBunkerModel : public CJoltSingleBodyObjectModel {

   public:

      CJoltBunkerModel(CJoltEngine& c_engine,
                           CBunkerEntity& c_entity);

      virtual ~CJoltBunkerModel() {}

      virtual void UpdateFromEntityStatus();

      void UpdateAuxiliaryAnchor(SAnchor& s_anchor);

   private:

      CBunkerEntity& m_cBunkerEntity;
      CWheeledEntity& m_cWheeledEntity;

      static const Real BUNKER_LENGTH;
      static const Real BUNKER_WIDTH;
      static const Real BUNKER_HEIGHT;
      static const Real BUNKER_MASS;
      static const Real BUNKER_TRACK_GAUGE;

   };

}

#endif
