/**
 * @file <argos3/plugins/simulator/media/mesh_medium.h>
 *
 * A multi-hop wireless mesh between a set of robots and a fixed base station.
 *
 * Every tick the medium samples the robots' positions and recomputes the
 * route between every ordered pair of nodes. Two nodes are linked when they
 * are at most `range_m` apart in 3-D, and
 *
 *   link rate = 300 - 298 * d / range_m   Mbps (300 close up, 2 at range_m)
 *   path rate = slowest link rate on the path / hops
 *
 * Routing takes the fewest hops, then the greatest bottleneck link rate, then
 * the lexicographic order of node ids. A node's route to itself has 0 hops at
 * 300 Mbps. With `unlimited`, every distinct pair is one hop at 300 Mbps.
 *
 * The medium only computes connectivity; it carries no payload. Whatever
 * delivers the messages reads GetRoutes() and gates them itself.
 *
 * @author Giovanni Beltrame - <giovanni.beltrame@polymtl.ca>
 */

#ifndef MESH_MEDIUM_H
#define MESH_MEDIUM_H

namespace argos {
   class CMeshMedium;
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/medium/medium.h>
#include <argos3/core/utility/math/vector3.h>

#include <string>
#include <vector>

namespace argos {

   class CMeshMedium : public CMedium {

   public:

      /** A node of the mesh: a robot or the base station. */
      struct SNode {
         std::string Id;
         CVector3 Position;
      };

      /** The route from one node to another. */
      struct SRoute {
         std::string Source;
         std::string Destination;
         bool Reachable;
         /** 0 when unreachable, and for a node's route to itself. */
         UInt32 Hops;
         /** 0 when unreachable. */
         Real RateMbps;

         bool operator==(const SRoute& s_other) const {
            return Source == s_other.Source && Destination == s_other.Destination &&
                   Reachable == s_other.Reachable && Hops == s_other.Hops &&
                   RateMbps == s_other.RateMbps;
         }
      };

      /** Rate of a link at zero distance, and of every link when unlimited. */
      static constexpr Real MAX_RATE_MBPS = 300.0;
      /** Rate of a link exactly `range_m` long. */
      static constexpr Real EDGE_RATE_MBPS = 2.0;

      /**
       * Returns the rate of one link.
       * @param f_distance The length of the link, in meters.
       * @param f_range The communication range, in meters.
       * @return The rate in Mbps; 0 beyond f_range, where there is no link.
       */
      static Real LinkRateMbps(Real f_distance, Real f_range);

      /**
       * Computes the route between every ordered pair of nodes, self pairs
       * included, sorted by source then destination id, so the result does
       * not depend on the order of the nodes.
       * @param vec_nodes The nodes.
       * @param f_range The communication range, in meters.
       * @param b_unlimited Whether every pair is linked regardless of range.
       * @return The routes.
       * @throws CARGoSException on duplicate ids, a non-finite position, or a
       *         non-finite or non-positive range when not unlimited.
       */
      static std::vector<SRoute> ComputeRoutes(const std::vector<SNode>& vec_nodes,
                                               Real f_range,
                                               bool b_unlimited);

   public:

      CMeshMedium() = default;

      virtual ~CMeshMedium() {}

      virtual void Init(TConfigurationNode& t_tree);
      /** Resolves the robots' bodies, which exist only once the space does. */
      virtual void PostSpaceInit();
      virtual void Reset();
      virtual void Destroy();
      virtual void Update();

      /**
       * Returns the routes computed at GetTick(): every ordered pair, self
       * pairs included, sorted by source then destination id.
       */
      inline const std::vector<SRoute>& GetRoutes() const {
         return m_vecRoutes;
      }

      /** Returns the simulation clock when GetRoutes() was computed; 0 after Init and Reset. */
      inline UInt32 GetTick() const {
         return m_unTick;
      }

      inline Real GetRange() const {
         return m_fRange;
      }

      inline bool IsUnlimited() const {
         return m_bUnlimited;
      }

      /**
       * Whether whatever delivers the messages should cap each pair's
       * bandwidth at its route rate. The medium itself never applies it.
       */
      inline bool IsBandwidthCapped() const {
         return m_bCapBandwidth;
      }

      inline const std::string& GetBaseStationId() const {
         return m_strBaseId;
      }

   private:

      void UpdateRoutes();

   private:

      std::vector<std::string> m_vecRobotIds;
      std::vector<CEmbodiedEntity*> m_vecBodies;
      Real m_fRange = 50.0;
      bool m_bUnlimited = false;
      bool m_bCapBandwidth = true;
      std::string m_strBaseId = "base_station";
      CVector3 m_cBasePosition;
      std::vector<SRoute> m_vecRoutes;
      UInt32 m_unTick = 0;

   };

}

#endif
