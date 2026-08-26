/**
 * @file <argos3/plugins/robots/scout-mini/simulator/scout_mini_entity.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "scout_mini_entity.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* Physical dimensions of AgileX Scout Mini */
   static const Real SCOUT_MINI_LENGTH             = 0.612f;
   static const Real SCOUT_MINI_WIDTH              = 0.580f;
   static const Real SCOUT_MINI_HEIGHT             = 0.245f;
   static const Real SCOUT_MINI_HALF_HEIGHT        = SCOUT_MINI_HEIGHT * 0.5f;
   static const Real SCOUT_MINI_TRACK_GAUGE        = 0.450f;
   static const Real SCOUT_MINI_HALF_TRACK_GAUGE   = SCOUT_MINI_TRACK_GAUGE * 0.5f;
   static const Real SCOUT_MINI_WHEEL_RADIUS    = 0.0875f;

   /****************************************/
   /****************************************/

   CScoutMiniEntity::CScoutMiniEntity() :
      CComposableEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcControllableEntity(nullptr),
      m_pcBatteryEquippedEntity(nullptr) {
   }

   /****************************************/
   /****************************************/

   CScoutMiniEntity::CScoutMiniEntity(const std::string& str_id,
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
                                       CVector3(0.0f, 0.0f, SCOUT_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("left_wheels",
                                       CVector3(0.0f, SCOUT_MINI_HALF_TRACK_GAUGE, SCOUT_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("right_wheels",
                                       CVector3(0.0f, -SCOUT_MINI_HALF_TRACK_GAUGE, SCOUT_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("lidar",
                                       CVector3(-0.080f, 0.0f, 0.4525f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("camera",
                                       CVector3(0.322f, 0.0f, 0.2125f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("imu",
                                       CVector3(0.0f, 0.0f, 0.1225f),
                                       CQuaternion()).Enable();

         /* Wheeled entity (2 virtual wheels: skid steer is driven as a differential) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  SCOUT_MINI_HALF_TRACK_GAUGE, 0.0f), SCOUT_MINI_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -SCOUT_MINI_HALF_TRACK_GAUGE, 0.0f), SCOUT_MINI_WHEEL_RADIUS);

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

   void CScoutMiniEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Initialize the base class */
         CComposableEntity::Init(t_tree);

         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));

         /* Anchors */
         m_pcEmbodiedEntity->AddAnchor("body",
                                       CVector3(0.0f, 0.0f, SCOUT_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("left_wheels",
                                       CVector3(0.0f, SCOUT_MINI_HALF_TRACK_GAUGE, SCOUT_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("right_wheels",
                                       CVector3(0.0f, -SCOUT_MINI_HALF_TRACK_GAUGE, SCOUT_MINI_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("lidar",
                                       CVector3(-0.080f, 0.0f, 0.4525f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("camera",
                                       CVector3(0.322f, 0.0f, 0.2125f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("imu",
                                       CVector3(0.0f, 0.0f, 0.1225f),
                                       CQuaternion()).Enable();

         /* Wheeled entity */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  SCOUT_MINI_HALF_TRACK_GAUGE, 0.0f), SCOUT_MINI_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -SCOUT_MINI_HALF_TRACK_GAUGE, 0.0f), SCOUT_MINI_WHEEL_RADIUS);

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

   void CScoutMiniEntity::Reset() {
      /* Reset base and components */
      CComposableEntity::Reset();
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CScoutMiniEntity::UpdateComponents() {
      if(m_pcBatteryEquippedEntity != nullptr) {
         m_pcBatteryEquippedEntity->Update();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CScoutMiniEntity,
                   "scout_mini",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "The AgileX Scout Mini 4WD skid-steer mobile robot.",
                   "The AgileX Scout Mini is a compact four-wheel skid-steer platform, driven\n"
                   "through the generic differential_steering actuator. Anchors are provided\n"
                   "for a lidar, a camera and an IMU, but sensors may equally be mounted on\n"
                   "the origin anchor with an explicit position.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <scout_mini id=\"sm0\">\n"
                   "      <body position=\"0,0,0\" orientation=\"0,0,0\" />\n"
                   "      <controller config=\"sm0_controller\" />\n"
                   "    </scout_mini>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "  None.\n",
                   "Usable"
   );

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CScoutMiniEntity);

}
