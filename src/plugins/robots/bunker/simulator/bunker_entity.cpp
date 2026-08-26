/**
 * @file <argos3/plugins/robots/bunker/simulator/bunker_entity.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "bunker_entity.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* Physical dimensions of AgileX Bunker */
   static const Real BUNKER_LENGTH             = 1.023f;
   static const Real BUNKER_WIDTH              = 0.778f;
   static const Real BUNKER_HEIGHT             = 0.380f;
   static const Real BUNKER_HALF_HEIGHT        = BUNKER_HEIGHT * 0.5f;
   static const Real BUNKER_TRACK_GAUGE        = 0.620f;
   static const Real BUNKER_HALF_TRACK_GAUGE   = BUNKER_TRACK_GAUGE * 0.5f;
   static const Real BUNKER_WHEEL_RADIUS    = 0.100f;

   /****************************************/
   /****************************************/

   CBunkerEntity::CBunkerEntity() :
      CComposableEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcControllableEntity(nullptr),
      m_pcBatteryEquippedEntity(nullptr) {
   }

   /****************************************/
   /****************************************/

   CBunkerEntity::CBunkerEntity(const std::string& str_id,
                                        const std::string& str_controller_id,
                                        const CVector3& c_position,
                                        const CQuaternion& c_orientation,
                                        const std::string& str_bat_model) :
      CComposableEntity(nullptr, str_id),
      m_pcEmbodiedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcControllableEntity(nullptr),
      m_pcBatteryEquippedEntity(nullptr) {
      try {
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
         AddComponent(*m_pcEmbodiedEntity);
         /* Anchors */
         m_pcEmbodiedEntity->AddAnchor("body",
                                       CVector3(0.0f, 0.0f, BUNKER_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("left_tracks",
                                       CVector3(0.0f, BUNKER_HALF_TRACK_GAUGE, BUNKER_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("right_tracks",
                                       CVector3(0.0f, -BUNKER_HALF_TRACK_GAUGE, BUNKER_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("lidar",
                                       CVector3(-0.150f, 0.0f, 0.720f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("camera",
                                       CVector3(0.515f, 0.0f, 0.300f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("imu",
                                       CVector3(0.0f, 0.0f, 0.200f),
                                       CQuaternion()).Enable();

         /* Wheeled entity (2 tracks) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  BUNKER_HALF_TRACK_GAUGE, 0.0f), BUNKER_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -BUNKER_HALF_TRACK_GAUGE, 0.0f), BUNKER_WHEEL_RADIUS);

         /* Battery entity */
         m_pcBatteryEquippedEntity = new CBatteryEquippedEntity(this, "battery_0", str_bat_model);
         AddComponent(*m_pcBatteryEquippedEntity);

         /* Controllable entity */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->SetController(str_controller_id);

         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBunkerEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Initialize the base class */
         CComposableEntity::Init(t_tree);

         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));

         /* Anchors */
         m_pcEmbodiedEntity->AddAnchor("body",
                                       CVector3(0.0f, 0.0f, BUNKER_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("left_tracks",
                                       CVector3(0.0f, BUNKER_HALF_TRACK_GAUGE, BUNKER_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("right_tracks",
                                       CVector3(0.0f, -BUNKER_HALF_TRACK_GAUGE, BUNKER_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("lidar",
                                       CVector3(-0.150f, 0.0f, 0.720f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("camera",
                                       CVector3(0.515f, 0.0f, 0.300f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("imu",
                                       CVector3(0.0f, 0.0f, 0.200f),
                                       CQuaternion()).Enable();

         /* Wheeled entity */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  BUNKER_HALF_TRACK_GAUGE, 0.0f), BUNKER_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -BUNKER_HALF_TRACK_GAUGE, 0.0f), BUNKER_WHEEL_RADIUS);

         /* Battery entity */
         m_pcBatteryEquippedEntity = new CBatteryEquippedEntity(this, "battery_0");
         if(NodeExists(t_tree, "battery")) {
            m_pcBatteryEquippedEntity->Init(GetNode(t_tree, "battery"));
         }
         AddComponent(*m_pcBatteryEquippedEntity);

         /* Controllable entity */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->Init(GetNode(t_tree, "controller"));

         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBunkerEntity::Reset() {
      /* Reset base and components */
      CComposableEntity::Reset();
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CBunkerEntity::UpdateComponents() {
      if(m_pcBatteryEquippedEntity != nullptr) {
         m_pcBatteryEquippedEntity->Update();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CBunkerEntity,
                   "bunker",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "The AgileX Bunker full-size tracked mobile robot.",
                   "The AgileX Bunker is a full-size tracked platform, driven through the\n"
                   "generic differential_steering actuator. It is NOT the bunker_mini\n"
                   "entity, which is a different and much smaller robot: 1.023 x 0.778 m\n"
                   "and 170 kg against 0.660 x 0.584 m and 55 kg.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <bunker id=\"bk0\">\n"
                   "      <body position=\"0,0,0\" orientation=\"0,0,0\" />\n"
                   "      <controller config=\"bk0_controller\" />\n"
                   "    </bunker>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "  None.\n",
                   "Usable"
   );

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CBunkerEntity);

}
