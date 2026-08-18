/**
 * @file <argos3/plugins/robots/bunker-mini/simulator/qtopengl_bunker_mini.cpp>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#include "qtopengl_bunker_mini.h"
#include "bunker_mini_entity.h"

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real HULL_LENGTH = 0.58f;
   static const Real HULL_WIDTH  = 0.36f;
   static const Real HULL_HEIGHT = 0.18f;

   static const Real TRACK_LENGTH = 0.66f;
   static const Real TRACK_WIDTH  = 0.10f;
   static const Real TRACK_HEIGHT = 0.22f;
   static const Real TRACK_OFFSET_Y = 0.235f;

   /****************************************/
   /****************************************/

   static void DrawBox(float sx, float sy, float sz) {
      float hx = sx * 0.5f;
      float hy = sy * 0.5f;
      float hz = sz * 0.5f;

      glBegin(GL_QUADS);
      /* Front */
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertex3f(hx, -hy, -hz);
      glVertex3f(hx,  hy, -hz);
      glVertex3f(hx,  hy,  hz);
      glVertex3f(hx, -hy,  hz);
      /* Back */
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertex3f(-hx, -hy, -hz);
      glVertex3f(-hx, -hy,  hz);
      glVertex3f(-hx,  hy,  hz);
      glVertex3f(-hx,  hy, -hz);
      /* Top */
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertex3f(-hx, -hy, hz);
      glVertex3f( hx, -hy, hz);
      glVertex3f( hx,  hy, hz);
      glVertex3f(-hx,  hy, hz);
      /* Bottom */
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertex3f(-hx, -hy, -hz);
      glVertex3f(-hx,  hy, -hz);
      glVertex3f( hx,  hy, -hz);
      glVertex3f( hx, -hy, -hz);
      /* Right */
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertex3f(-hx, -hy, -hz);
      glVertex3f( hx, -hy, -hz);
      glVertex3f( hx, -hy,  hz);
      glVertex3f(-hx, -hy,  hz);
      /* Left */
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertex3f(-hx, hy, -hz);
      glVertex3f(-hx, hy,  hz);
      glVertex3f( hx, hy,  hz);
      glVertex3f( hx, hy, -hz);
      glEnd();
   }

   /****************************************/
   /****************************************/

   static void DrawCylinder(float radius, float height, int slices = 16) {
      float half_h = height * 0.5f;
      /* Side */
      glBegin(GL_QUAD_STRIP);
      for(int i = 0; i <= slices; ++i) {
         float angle = 2.0f * M_PI * float(i) / float(slices);
         float x = radius * std::cos(angle);
         float y = radius * std::sin(angle);
         glNormal3f(std::cos(angle), std::sin(angle), 0.0f);
         glVertex3f(x, y, -half_h);
         glVertex3f(x, y,  half_h);
      }
      glEnd();
      /* Top cap */
      glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertex3f(0.0f, 0.0f, half_h);
      for(int i = 0; i <= slices; ++i) {
         float angle = 2.0f * M_PI * float(i) / float(slices);
         glVertex3f(radius * std::cos(angle), radius * std::sin(angle), half_h);
      }
      glEnd();
      /* Bottom cap */
      glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertex3f(0.0f, 0.0f, -half_h);
      for(int i = slices; i >= 0; --i) {
         float angle = 2.0f * M_PI * float(i) / float(slices);
         glVertex3f(radius * std::cos(angle), radius * std::sin(angle), -half_h);
      }
      glEnd();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLBunkerMini::CQTOpenGLBunkerMini() {
      m_unHullList = glGenLists(1);
      glNewList(m_unHullList, GL_COMPILE);
      MakeHull();
      glEndList();

      m_unTrackList = glGenLists(1);
      glNewList(m_unTrackList, GL_COMPILE);
      MakeTrack();
      glEndList();

      m_unLidarList = glGenLists(1);
      glNewList(m_unLidarList, GL_COMPILE);
      MakeLidar();
      glEndList();

      m_unCameraList = glGenLists(1);
      glNewList(m_unCameraList, GL_COMPILE);
      MakeCamera();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLBunkerMini::~CQTOpenGLBunkerMini() {
      glDeleteLists(m_unHullList, 1);
      glDeleteLists(m_unTrackList, 1);
      glDeleteLists(m_unLidarList, 1);
      glDeleteLists(m_unCameraList, 1);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBunkerMini::Draw(const CBunkerMiniEntity& c_entity) {
      const SAnchor& sAnchor = c_entity.GetEmbodiedEntity().GetOriginAnchor();
      CRadians cZAngle, cYAngle, cXAngle;
      sAnchor.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

      glPushMatrix();
      glTranslatef(sAnchor.Position.GetX(),
                   sAnchor.Position.GetY(),
                   sAnchor.Position.GetZ());
      glRotatef(ToDegrees(cXAngle).GetValue(), 1.0f, 0.0f, 0.0f);
      glRotatef(ToDegrees(cYAngle).GetValue(), 0.0f, 1.0f, 0.0f);
      glRotatef(ToDegrees(cZAngle).GetValue(), 0.0f, 0.0f, 1.0f);

      /* Main body hull */
      glPushMatrix();
      glTranslatef(0.0f, 0.0f, 0.155f);
      glCallList(m_unHullList);
      glPopMatrix();

      /* Left Track */
      glPushMatrix();
      glTranslatef(0.0f, TRACK_OFFSET_Y, 0.11f);
      glCallList(m_unTrackList);
      glPopMatrix();

      /* Right Track */
      glPushMatrix();
      glTranslatef(0.0f, -TRACK_OFFSET_Y, 0.11f);
      glCallList(m_unTrackList);
      glPopMatrix();

      /* Top Lidar */
      glPushMatrix();
      glTranslatef(0.0f, 0.0f, 0.281f + 0.045f);
      glCallList(m_unLidarList);
      glPopMatrix();

      /* Front Camera */
      glPushMatrix();
      glTranslatef(0.29f, 0.0f, 0.245f);
      glCallList(m_unCameraList);
      glPopMatrix();

      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBunkerMini::MakeHull() {
      /* Main central dark metal body */
      GLfloat pfMetallicDark[] = { 0.22f, 0.24f, 0.26f, 1.0f };
      GLfloat pfMetallicTop[]  = { 0.35f, 0.38f, 0.40f, 1.0f };
      GLfloat pfShininess[]    = { 50.0f };

      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfMetallicDark);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfMetallicTop);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
      DrawBox(HULL_LENGTH, HULL_WIDTH, HULL_HEIGHT);

      /* Top aluminum mounting deck */
      glPushMatrix();
      glTranslatef(0.0f, 0.0f, HULL_HEIGHT * 0.5f + 0.005f);
      GLfloat pfAluminium[] = { 0.45f, 0.47f, 0.50f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfAluminium);
      DrawBox(HULL_LENGTH * 0.9f, HULL_WIDTH * 0.95f, 0.01f);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBunkerMini::MakeTrack() {
      /* Black rubber tread assembly */
      GLfloat pfRubber[]     = { 0.08f, 0.08f, 0.08f, 1.0f };
      GLfloat pfRubberSpec[] = { 0.15f, 0.15f, 0.15f, 1.0f };
      GLfloat pfShininess[]  = { 10.0f };

      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfRubber);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfRubberSpec);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
      DrawBox(TRACK_LENGTH, TRACK_WIDTH, TRACK_HEIGHT);

      /* Gold/silver drive sprocket accent */
      GLfloat pfSprocket[] = { 0.6f, 0.5f, 0.2f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfSprocket);
      glPushMatrix();
      glTranslatef(TRACK_LENGTH * 0.35f, 0.0f, 0.0f);
      glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
      DrawCylinder(0.065f, TRACK_WIDTH * 1.05f, 12);
      glPopMatrix();

      glPushMatrix();
      glTranslatef(-TRACK_LENGTH * 0.35f, 0.0f, 0.0f);
      glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
      DrawCylinder(0.065f, TRACK_WIDTH * 1.05f, 12);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBunkerMini::MakeLidar() {
      /* Base stand */
      GLfloat pfStand[] = { 0.25f, 0.25f, 0.25f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfStand);
      DrawCylinder(0.045f, 0.03f, 16);

      /* Blue/cyan lidar sensor band */
      glPushMatrix();
      glTranslatef(0.0f, 0.0f, 0.035f);
      GLfloat pfLidarBlue[] = { 0.1f, 0.4f, 0.7f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfLidarBlue);
      DrawCylinder(0.051f, 0.04f, 24);
      glPopMatrix();

      /* Top cap */
      glPushMatrix();
      glTranslatef(0.0f, 0.0f, 0.06f);
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfStand);
      DrawCylinder(0.052f, 0.01f, 24);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBunkerMini::MakeCamera() {
      /* Front RGB/Depth camera housing */
      GLfloat pfCamBody[] = { 0.15f, 0.15f, 0.15f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfCamBody);
      DrawBox(0.035f, 0.12f, 0.03f);

      /* Dual lenses */
      GLfloat pfLens[] = { 0.05f, 0.05f, 0.05f, 1.0f };
      GLfloat pfLensSpec[] = { 0.8f, 0.8f, 0.9f, 1.0f };
      GLfloat pfShininess[] = { 100.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfLens);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfLensSpec);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);

      glPushMatrix();
      glTranslatef(0.018f, 0.035f, 0.0f);
      glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
      DrawCylinder(0.010f, 0.005f, 12);
      glPopMatrix();

      glPushMatrix();
      glTranslatef(0.018f, -0.035f, 0.0f);
      glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
      DrawCylinder(0.010f, 0.005f, 12);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

}
