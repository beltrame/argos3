/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_cylinder_model.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_CYLINDER_MODEL_H
#define JOLT_CYLINDER_MODEL_H

namespace argos {
   class CCylinderEntity;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>

namespace argos {

   class CJoltCylinderModel : public CJoltSingleBodyObjectModel {

   public:

      CJoltCylinderModel(CJoltEngine& c_engine,
                         CCylinderEntity& c_cylinder);

      virtual ~CJoltCylinderModel() {}

   };

}

#endif
