/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/jolt_single_body_object_model.h>
 *
 * Base for models made of a single Jolt body bound to the origin
 * anchor.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef JOLT_SINGLE_BODY_OBJECT_MODEL_H
#define JOLT_SINGLE_BODY_OBJECT_MODEL_H

namespace argos {
   class CJoltSingleBodyObjectModel;
}

#include <argos3/plugins/simulator/physics_engines/jolt/jolt_model.h>

namespace argos {

   class CJoltSingleBodyObjectModel : public CJoltModel {

   public:

      CJoltSingleBodyObjectModel(CJoltEngine& c_engine,
                                 CComposableEntity& c_entity);

      virtual ~CJoltSingleBodyObjectModel() {}

      virtual void MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation);

   };

}

#endif
