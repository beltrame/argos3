/**
 * @file <argos3/plugins/robots/spot/simulator/spot_entity.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "spot_entity.h"

#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/plugins/simulator/entities/battery_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* Physical dimensions of Boston Dynamics Spot */
   static const Real SPOT_LENGTH             = 1.100f;
   static const Real SPOT_WIDTH              = 0.500f;
   static const Real SPOT_HEIGHT             = 0.620f;
   static const Real SPOT_HALF_HEIGHT        = SPOT_HEIGHT * 0.5f;
   static const Real SPOT_TRACK_GAUGE        = 0.500f;
   static const Real SPOT_HALF_TRACK_GAUGE   = SPOT_TRACK_GAUGE * 0.5f;
   static const Real SPOT_WHEEL_RADIUS    = 0.160f;

   /****************************************/
   /****************************************/

   CSpotEntity::CSpotEntity() :
      CComposableEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_pcWheeledEntity(nullptr),
      m_pcControllableEntity(nullptr),
      m_pcBatteryEquippedEntity(nullptr) {
   }

   /****************************************/
   /****************************************/

   CSpotEntity::CSpotEntity(const std::string& str_id,
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
                                       CVector3(0.0f, 0.0f, SPOT_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("left_legs",
                                       CVector3(0.0f, SPOT_HALF_TRACK_GAUGE, SPOT_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("right_legs",
                                       CVector3(0.0f, -SPOT_HALF_TRACK_GAUGE, SPOT_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("lidar",
                                       CVector3(-0.180f, 0.0f, 0.970f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("camera",
                                       CVector3(0.598f, 0.0f, 0.520f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("imu",
                                       CVector3(0.0f, 0.0f, 0.500f),
                                       CQuaternion()).Enable();

         /* Wheeled entity (2 virtual wheels standing in for the gait) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  SPOT_HALF_TRACK_GAUGE, 0.0f), SPOT_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -SPOT_HALF_TRACK_GAUGE, 0.0f), SPOT_WHEEL_RADIUS);

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

   void CSpotEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Initialize the base class */
         CComposableEntity::Init(t_tree);

         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));

         /* Anchors */
         m_pcEmbodiedEntity->AddAnchor("body",
                                       CVector3(0.0f, 0.0f, SPOT_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("left_legs",
                                       CVector3(0.0f, SPOT_HALF_TRACK_GAUGE, SPOT_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("right_legs",
                                       CVector3(0.0f, -SPOT_HALF_TRACK_GAUGE, SPOT_HALF_HEIGHT),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("lidar",
                                       CVector3(-0.180f, 0.0f, 0.970f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("camera",
                                       CVector3(0.598f, 0.0f, 0.520f),
                                       CQuaternion()).Enable();
         m_pcEmbodiedEntity->AddAnchor("imu",
                                       CVector3(0.0f, 0.0f, 0.500f),
                                       CQuaternion()).Enable();

         /* Wheeled entity */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  SPOT_HALF_TRACK_GAUGE, 0.0f), SPOT_WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -SPOT_HALF_TRACK_GAUGE, 0.0f), SPOT_WHEEL_RADIUS);

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

   void CSpotEntity::Reset() {
      /* Reset base and components */
      CComposableEntity::Reset();
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CSpotEntity::UpdateComponents() {
      if(m_pcBatteryEquippedEntity != nullptr) {
         m_pcBatteryEquippedEntity->Update();
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CSpotEntity,
                   "spot",
                   "Giovanni Beltrame [giovanni.beltrame@polymtl.ca]",
                   "1.0",
                   "Boston Dynamics Spot, as a differentially steered rigid body.",
                   "Spot is modelled as a rigid body driven by differential steering. The gait is\n"
                   "NOT simulated: what this entity reproduces is the footprint, the sensor\n"
                   "mount heights, and the body a neighbour lidar sees. The collision shape is\n"
                   "the standing envelope, floor to deck top, so nothing drives through the\n"
                   "space the legs occupy; the legs themselves appear only in the glTF visual,\n"
                   "which is what the cameras and the photorealistic lidar raytrace.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <spot id=\"sp0\">\n"
                   "      <body position=\"0,0,0\" orientation=\"0,0,0\" />\n"
                   "      <controller config=\"sp0_controller\" />\n"
                   "    </spot>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "  None.\n",
                   "Usable"
   );

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CSpotEntity);

}
