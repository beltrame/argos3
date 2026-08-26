/**
 * @file <argos3/plugins/robots/scout-mini/simulator/jolt_scout_mini_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_SCOUT_MINI_MODEL_H
#define JOLT_SCOUT_MINI_MODEL_H

namespace argos {
   class CJoltScoutMiniModel;
   class CScoutMiniEntity;
   class CWheeledEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>

namespace argos {

   class CJoltScoutMiniModel : public CJoltSingleBodyObjectModel {

   public:

      CJoltScoutMiniModel(CJoltEngine& c_engine,
                           CScoutMiniEntity& c_entity);

      virtual ~CJoltScoutMiniModel() {}

      virtual void UpdateFromEntityStatus();

      void UpdateAuxiliaryAnchor(SAnchor& s_anchor);

   private:

      CScoutMiniEntity& m_cScoutMiniEntity;
      CWheeledEntity& m_cWheeledEntity;

      static const Real SCOUT_MINI_LENGTH;
      static const Real SCOUT_MINI_WIDTH;
      static const Real SCOUT_MINI_HEIGHT;
      static const Real SCOUT_MINI_MASS;
      static const Real SCOUT_MINI_TRACK_GAUGE;

   };

}

#endif
