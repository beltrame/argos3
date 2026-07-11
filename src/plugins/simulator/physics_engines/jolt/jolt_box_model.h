/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_box_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_BOX_MODEL_H
#define JOLT_BOX_MODEL_H

namespace argos {
   class CBoxEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>

namespace argos {

   class CJoltBoxModel : public CJoltSingleBodyObjectModel {

   public:

      CJoltBoxModel(CJoltEngine& c_engine,
                    CBoxEntity& c_box);

      virtual ~CJoltBoxModel() {}

   };

}

#endif
