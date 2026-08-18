/**
 * @file <argos3/plugins/robots/bunker-mini/simulator/qtopengl_bunker_mini.h>
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef QTOPENGL_BUNKER_MINI_H
#define QTOPENGL_BUNKER_MINI_H

namespace argos {
   class CQTOpenGLBunkerMini;
   class CBunkerMiniEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLBunkerMini {

   public:

      CQTOpenGLBunkerMini();

      virtual ~CQTOpenGLBunkerMini();

      virtual void Draw(const CBunkerMiniEntity& c_entity);

   private:

      void MakeHull();
      void MakeTrack();
      void MakeLidar();
      void MakeCamera();

      GLuint m_unHullList;
      GLuint m_unTrackList;
      GLuint m_unLidarList;
      GLuint m_unCameraList;

   };

}

#endif
