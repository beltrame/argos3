/**
 * @file <argos3/plugins/robots/spot/simulator/spot_entity.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef SPOT_ENTITY_H
#define SPOT_ENTITY_H

namespace argos {
   class CSpotEntity;
   class CEmbodiedEntity;
   class CWheeledEntity;
   class CControllableEntity;
   class CBatteryEquippedEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   class CSpotEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      CSpotEntity();

      CSpotEntity(const std::string& str_id,
                        const std::string& str_controller_id,
                        const CVector3& c_position = CVector3(),
                        const CQuaternion& c_orientation = CQuaternion(),
                        const std::string& str_bat_model = "");

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void UpdateComponents();

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline const CEmbodiedEntity& GetEmbodiedEntity() const {
         return *m_pcEmbodiedEntity;
      }

      inline CWheeledEntity& GetWheeledEntity() {
         return *m_pcWheeledEntity;
      }

      inline const CWheeledEntity& GetWheeledEntity() const {
         return *m_pcWheeledEntity;
      }

      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      inline const CControllableEntity& GetControllableEntity() const {
         return *m_pcControllableEntity;
      }

      inline CBatteryEquippedEntity& GetBatterySensorEquippedEntity() {
         return *m_pcBatteryEquippedEntity;
      }

      inline const CBatteryEquippedEntity& GetBatterySensorEquippedEntity() const {
         return *m_pcBatteryEquippedEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "spot";
      }

   private:

      CEmbodiedEntity*        m_pcEmbodiedEntity;
      CWheeledEntity*         m_pcWheeledEntity;
      CControllableEntity*    m_pcControllableEntity;
      CBatteryEquippedEntity* m_pcBatteryEquippedEntity;

   };

}

#endif
