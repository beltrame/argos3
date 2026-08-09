/**
 * @file <argos3/plugins/simulator/physics_engines/jolt/mesh_entity.cpp>
 *
 * @author lemonci - <monica.li@outlook.com>
 */

#include "mesh_entity.h"

#include <argos3/core/simulator/space/space.h>

namespace argos {

   /****************************************/
   /****************************************/

   CMeshEntity::CMeshEntity() :
      CComposableEntity(nullptr),
      m_pcEmbodiedEntity(nullptr),
      m_bYUp(true),
      m_bDoubleSided(true),
      m_fScale(1.0) {}

   /****************************************/
   /****************************************/

   void CMeshEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Initialize the parent, which parses the id */
         CComposableEntity::Init(t_tree);
         GetNodeAttribute(t_tree, "file", m_strFile);
         GetNodeAttributeOrDefault(t_tree, "y_up", m_bYUp, m_bYUp);
         GetNodeAttributeOrDefault(t_tree, "double_sided", m_bDoubleSided,
                                   m_bDoubleSided);
         GetNodeAttributeOrDefault(t_tree, "scale", m_fScale, m_fScale);
         CVector3 cPosition;
         CQuaternion cOrientation;
         GetNodeAttributeOrDefault(t_tree, "position", cPosition, CVector3());
         GetNodeAttributeOrDefault(t_tree, "orientation", cOrientation, CQuaternion());
         /* The pose is given by the attributes of this node, so the
          * embodied entity is built directly rather than Init()ed from
          * a <body> child. A mesh is world geometry: never movable. */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0",
                                                  cPosition, cOrientation,
                                                  false);
         AddComponent(*m_pcEmbodiedEntity);
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize mesh entity \""
                                     << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CMeshEntity::Reset() {
      CComposableEntity::Reset();
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CMeshEntity,
                   "mesh",
                   "lemonci [monica.li@outlook.com]",
                   "1.0",
                   "A static triangle mesh loaded from a glTF file.",
                   "The mesh entity brings real world geometry into the arena: robots collide\n"
                   "with its triangles and ray-cast sensors (lidar, proximity, ToF) return hits\n"
                   "on them. It is the only way to represent sloped ground, curved walls or\n"
                   "arbitrary terrain, which box and cylinder entities can only approximate.\n\n"
                   "The mesh is static and requires the jolt physics engine; no other engine\n"
                   "implements a triangle-mesh shape.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <mesh id=\"world\" file=\"worlds/tunnel.glb\" />\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities.\n"
                   "The 'file' attribute is the path of a glTF 2.0 asset, either binary (.glb)\n"
                   "or JSON (.gltf) with embedded or side-car buffers. It is resolved against\n"
                   "the working directory first, then against the directory of the .argos file.\n"
                   "Every triangle of every mesh node of the file is cooked into a single Jolt\n"
                   "mesh shape; node transforms are applied. The same file declared by several\n"
                   "mesh entities is loaded and cooked only once.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <mesh id=\"world\" file=\"worlds/tunnel.glb\"\n"
                   "          position=\"0,0,0\" orientation=\"0,0,0\"\n"
                   "          scale=\"1.0\" y_up=\"true\" double_sided=\"true\" />\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'position' attribute (default 0,0,0) places the file's origin in the\n"
                   "arena, in the X,Y,Z order.\n"
                   "The 'orientation' attribute (default 0,0,0) rotates the mesh about that\n"
                   "position. The order of the angles is Z,Y,X in degrees, as everywhere else\n"
                   "in ARGoS.\n"
                   "The 'scale' attribute (default 1.0) uniformly scales the mesh. It wraps the\n"
                   "cooked shape rather than re-cooking it, so scaling is free.\n"
                   "The 'y_up' attribute (default true) converts the asset from the glTF Y-up\n"
                   "convention to the ARGoS Z-up convention by rotating +90 degrees about X.\n"
                   "Set it to false for assets already exported Z-up. Note that this differs\n"
                   "from the photorealism <scenery><prop> path, which performs no conversion\n"
                   "and expects orientation=\"0,0,90\" instead; here 'orientation' is free for\n"
                   "actual placement.\n"
                   "The 'double_sided' attribute (default true) makes collision independent of\n"
                   "how the asset is wound. Triangles are single-sided for simulation: a robot\n"
                   "only collides with the face a triangle's winding points at, so a surface\n"
                   "wound the wrong way lets robots fall through it. With 'double_sided' every\n"
                   "triangle is loaded twice, once as read and once with the winding reversed,\n"
                   "which removes that failure mode for any asset; the price is twice the\n"
                   "triangles, hence twice the cook time and shape memory and roughly twice\n"
                   "the time per ray cast. Set it to false for assets known to be\n"
                   "consistently wound with the normals facing the space the robots move in.\n"
                   "Ray casts are two-sided in either case, so what a sensor reports does not\n"
                   "depend on this attribute, only how fast it is answered.",
                   "Usable"
      );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CMeshEntity);

   /****************************************/
   /****************************************/

}
