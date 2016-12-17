//------------------------------------------------------------------------------
// avigle-common -- common classes/tools
//
// Developed during the research project AVIGLE
// which was part of the Hightech.NRW research program
// funded by the ministry for Innovation, Science, Research and Technology
// of the German state Northrhine-Westfalia, and by the European Union.
//
// Copyright (c) 2010--2013, Tom Vierjahn et al.
//------------------------------------------------------------------------------
//                                License
//
// This library/program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// If you are using this library/program in a project, work or publication,
// please cite [1,2].
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------------------------------------------------
//                                References
//
// [1] S. Rohde, N. Goddemeier, C. Wietfeld, F. Steinicke, K. Hinrichs,
//     T. Ostermann, J. Holsten, D. Moormann:
//     "AVIGLE: A System of Systems Concept for an
//      Avionic Digital Service Platform based on
//      Micro Unmanned Aerial Vehicles".
//     In Proc. IEEE Int'l Conf. Systems Man and Cybernetics (SMC),
//     pp. 459--466. 2010. DOI: 10.1109/ICSMC.2010.5641767
// [2] S. Strothoff, D. Feldmann, F. Steinicke, T. Vierjahn, S. Mostafawy:
//     "Interactive generation of virtual environments using MUAVs".
//     In Proc. IEEE Int. Symp. VR Innovations, pp. 89--96, 2011.
//     DOI: 10.1109/ISVRI.2011.5759608
//------------------------------------------------------------------------------

#ifndef TRIANGULATION_COMMON_AABB_H_
#define TRIANGULATION_COMMON_AABB_H_

#include <cmath>

#include <algorithm>
#include <iostream>
#include <limits>
#include <utility>

#ifndef Q_MOC_RUN
#include <boost/numeric/interval.hpp>
#endif

#include <common/vec3.h>


template <typename T>
class AABB_T
{
public:
  friend class OctreeNode;
  
  /// \brief Constructs an invalid (i.e. empty) AABB
  ///
  /// \note Invalidity is defined as fMinCorner == +inf
  ///       and fMaxCorner = -inf.
  AABB_T()
  : fMinCorner(Vec3<T>(std::numeric_limits<T>::infinity()))
  , fMaxCorner(Vec3<T>(-std::numeric_limits<T>::infinity()))
  {
  }
  
  AABB_T(const Vec3<T>& minCorner, const Vec3<T>& maxCorner)
  : fMinCorner(minCorner)
  , fMaxCorner(maxCorner)
  {
  }

  AABB_T(T minX, T minY, T minZ, T maxX, T maxY, T maxZ)
  : fMinCorner(minX, minY, minZ)
  , fMaxCorner(maxX, maxY, maxZ)
  {
  }

  bool IsValid() const
  {
    return (fMinCorner.GetX() <= fMaxCorner.GetX());
  }

  const Vec3<T>& GetMinCorner() const { return fMinCorner; }
  const Vec3<T>& GetMaxCorner() const { return fMaxCorner; }

  Vec3<T> GetCenter() const
  { 
    return static_cast<T>(0.5) * (fMinCorner + fMaxCorner);
  }

  T MinDistanceTo(const Vec3<T>& position) const
  {
    const Vec3<T> minDist = fMinCorner - position;
    const Vec3<T> maxDist = position - fMaxCorner;
    const Vec3<T> dist = Vec3<T>::MaxComponents(Vec3<T>(0.0),
                         Vec3<T>::MaxComponents(minDist, maxDist));
    return dist.GetLength();
  }

  boost::numeric::interval<T> GetIntervalX() const
  {
    return boost::numeric::interval<T>(
      fMinCorner.GetX(),
      fMaxCorner.GetX());
  }

  boost::numeric::interval<T> GetIntervalY() const
  {
    return boost::numeric::interval<T>(
      fMinCorner.GetY(),
      fMaxCorner.GetY());
  }

  boost::numeric::interval<T> GetIntervalZ() const
  {
    return boost::numeric::interval<T>(
      fMinCorner.GetZ(),
      fMaxCorner.GetZ());
  }

  bool Contains(const Vec3<T>& position) const
  {
    return boost::numeric::in(position.GetX(), GetIntervalX())
        && boost::numeric::in(position.GetY(), GetIntervalY())
        && boost::numeric::in(position.GetZ(), GetIntervalZ());
  }

  bool IsOnSurface(const Vec3<T>& position) const
  {
    const T epsilon = std::numeric_limits<T>::epsilon();
    return (this->Contains(position) &&
            (fabs(position.GetX() - fMinCorner.GetX()) <= epsilon || 
             fabs(position.GetX() - fMaxCorner.GetX()) <= epsilon ||
             fabs(position.GetY() - fMinCorner.GetY()) <= epsilon || 
             fabs(position.GetY() - fMaxCorner.GetY()) <= epsilon ||
             fabs(position.GetZ() - fMinCorner.GetZ()) <= epsilon || 
             fabs(position.GetZ() - fMaxCorner.GetZ()) <= epsilon));
  }

  void ResizeToIncludePoint(const Vec3<T>& point)
  {
    fMinCorner = Vec3<T>::MinComponents(point, fMinCorner);
    fMaxCorner = Vec3<T>::MaxComponents(point, fMaxCorner);
  }
  
  void Union(const AABB_T<T>& boxA, const Vec3<T>& point)
  {
    assert(boxA.IsValid());
    
    this->fMinCorner = Vec3<T>::MinComponents(boxA.GetMinCorner(),
                                              point);
    this->fMaxCorner = Vec3<T>::MaxComponents(boxA.GetMaxCorner(),
                                              point);
  }
  
  void Union(const Vec3<T>& point, const AABB_T<T>& boxB)
  {
    assert(boxB.IsValid());
    
    this->fMinCorner = Vec3<T>::MinComponents(point,
                                              boxB.GetMinCorner());
    this->fMaxCorner = Vec3<T>::MaxComponents(point,
                                              boxB.GetMaxCorner());
  }
  
  void Union(const Vec3<T>& pointA, const Vec3<T>& pointB)
  {
    this->fMinCorner = Vec3<T>::MinComponents(pointA, pointB);
    this->fMaxCorner = Vec3<T>::MaxComponents(pointA, pointB);
  }
  
  void Union(const AABB_T<T>& boxA, const AABB_T<T>& boxB)
  {
    assert(boxA.IsValid());
    assert(boxB.IsValid());
    
    this->fMinCorner = Vec3<T>::MinComponents(boxA.GetMinCorner(),
                                              boxB.GetMinCorner());
    this->fMaxCorner = Vec3<T>::MaxComponents(boxA.GetMaxCorner(),
                                              boxB.GetMaxCorner());
  }
  
  std::pair<T, T> GetMinMaxSquaredDistances(const Vec3<T>& position)
  const
  {
    // LBN - Left Bottom Near ...
    const Vec3<T> toLBN(Vec3<T>(this->fMinCorner.GetX(),
                                this->fMinCorner.GetY(),
                                this->fMinCorner.GetZ()) - position);
    const Vec3<T> toRBN(Vec3<T>(this->fMaxCorner.GetX(),
                                this->fMinCorner.GetY(),
                                this->fMinCorner.GetZ()) - position);
    const Vec3<T> toLBF(Vec3<T>(this->fMinCorner.GetX(),
                                this->fMinCorner.GetY(),
                                this->fMaxCorner.GetZ()) - position);
    const Vec3<T> toRBF(Vec3<T>(this->fMaxCorner.GetX(),
                                this->fMinCorner.GetY(),
                                this->fMaxCorner.GetZ()) - position);
    
    const Vec3<T> toLTN(Vec3<T>(this->fMinCorner.GetX(),
                                this->fMaxCorner.GetY(),
                                this->fMinCorner.GetZ()) - position);
    const Vec3<T> toRTN(Vec3<T>(this->fMaxCorner.GetX(),
                                this->fMaxCorner.GetY(),
                                this->fMinCorner.GetZ()) - position);
    const Vec3<T> toLTF(Vec3<T>(this->fMinCorner.GetX(),
                                this->fMaxCorner.GetY(),
                                this->fMaxCorner.GetZ()) - position);
    const Vec3<T> toRTF(Vec3<T>(this->fMaxCorner.GetX(),
                                this->fMaxCorner.GetY(),
                                this->fMaxCorner.GetZ()) - position);
    
    
    const T toLBN2(toLBN.GetSquaredLength());
    const T toRBN2(toRBN.GetSquaredLength());
    const T toLBF2(toLBF.GetSquaredLength());
    const T toRBF2(toRBF.GetSquaredLength());
    
    const T toLTN2(toLTN.GetSquaredLength());
    const T toRTN2(toRTN.GetSquaredLength());
    const T toLTF2(toLTF.GetSquaredLength());
    const T toRTF2(toRTF.GetSquaredLength());
    
    
    const T toBN2min(std::min(toLBN2, toRBN2));
    const T toBF2min(std::min(toLBF2, toRBF2));
    const T toTN2min(std::min(toLTN2, toRTN2));
    const T toTF2min(std::min(toLTF2, toRTF2));
    
    const T toB2min(std::min(toBN2min, toBF2min));
    const T toT2min(std::min(toTN2min, toTF2min));
    
    const T to2min(std::min(toB2min, toT2min));
    
    
    const T toBN2max(std::max(toLBN2, toRBN2));
    const T toBF2max(std::max(toLBF2, toRBF2));
    const T toTN2max(std::max(toLTN2, toRTN2));
    const T toTF2max(std::max(toLTF2, toRTF2));
    
    const T toB2max(std::max(toBN2max, toBF2max));
    const T toT2max(std::max(toTN2max, toTF2max));
    
    const T to2max(std::max(toB2max, toT2max));
    

    return std::pair<T, T>(to2min, to2max);
  }
  
  
  T GetMaxSquaredDistance(const Vec3<T>& position)
  const
  {
    // LBN - Left Bottom Near ...
    const Vec3<T> toLBN(Vec3<T>(this->fMinCorner.GetX(),
                                this->fMinCorner.GetY(),
                                this->fMinCorner.GetZ()) - position);
    const Vec3<T> toRBN(Vec3<T>(this->fMaxCorner.GetX(),
                                this->fMinCorner.GetY(),
                                this->fMinCorner.GetZ()) - position);
    const Vec3<T> toLBF(Vec3<T>(this->fMinCorner.GetX(),
                                this->fMinCorner.GetY(),
                                this->fMaxCorner.GetZ()) - position);
    const Vec3<T> toRBF(Vec3<T>(this->fMaxCorner.GetX(),
                                this->fMinCorner.GetY(),
                                this->fMaxCorner.GetZ()) - position);
    
    const Vec3<T> toLTN(Vec3<T>(this->fMinCorner.GetX(),
                                this->fMaxCorner.GetY(),
                                this->fMinCorner.GetZ()) - position);
    const Vec3<T> toRTN(Vec3<T>(this->fMaxCorner.GetX(),
                                this->fMaxCorner.GetY(),
                                this->fMinCorner.GetZ()) - position);
    const Vec3<T> toLTF(Vec3<T>(this->fMinCorner.GetX(),
                                this->fMaxCorner.GetY(),
                                this->fMaxCorner.GetZ()) - position);
    const Vec3<T> toRTF(Vec3<T>(this->fMaxCorner.GetX(),
                                this->fMaxCorner.GetY(),
                                this->fMaxCorner.GetZ()) - position);
    
    
    const T toLBN2(toLBN.GetSquaredLength());
    const T toRBN2(toRBN.GetSquaredLength());
    const T toLBF2(toLBF.GetSquaredLength());
    const T toRBF2(toRBF.GetSquaredLength());
    
    const T toLTN2(toLTN.GetSquaredLength());
    const T toRTN2(toRTN.GetSquaredLength());
    const T toLTF2(toLTF.GetSquaredLength());
    const T toRTF2(toRTF.GetSquaredLength());
    
    
    const T toBN2max(std::max(toLBN2, toRBN2));
    const T toBF2max(std::max(toLBF2, toRBF2));
    const T toTN2max(std::max(toLTN2, toRTN2));
    const T toTF2max(std::max(toLTF2, toRTF2));
    
    const T toB2max(std::max(toBN2max, toBF2max));
    const T toT2max(std::max(toTN2max, toTF2max));
    
    const T to2max(std::max(toB2max, toT2max));
    
    
    return to2max;
  }

  T GetMaxSquaredDistanceOptimized(const Vec3<T>& position)
  const
  {
    const Vec3<T> delta = this->GetCenter() - position;
    const T dx = delta.GetX();
    const T dy = delta.GetY();
    const T dz = delta.GetZ();

    const T distX = (dx >= 0 ?
       (this->fMaxCorner.GetX() - position.GetX()) :
       (this->fMinCorner.GetX() - position.GetX()));
    const T distY = (dy >= 0 ?
       (this->fMaxCorner.GetY() - position.GetY()) :
       (this->fMinCorner.GetY() - position.GetY()));
    const T distZ = (dz >= 0 ?
      (this->fMaxCorner.GetZ() - position.GetZ()) :
      (this->fMinCorner.GetZ() - position.GetZ()));
    
    return distX * distX + distY * distY + distZ * distZ;
//    if (delta.GetX() >= 0)
//    {
//      const T dx = this->fMaxCorner.GetX() - position.GetX();
//      maxSquaredDistance += dx * dx;
//    }
//    else
//    {
//      const T dx = this->fMinCorner.GetX() - position.GetX();
//      maxSquaredDistance += dx * dx;
//    }
//    
//    if (delta.GetY() >= 0)
//    {
//      const T dy = this->fMaxCorner.GetY() - position.GetY();
//      maxSquaredDistance += dy * dy;
//    }
//    else
//    {
//      const T dy = this->fMinCorner.GetY() - position.GetY();
//      maxSquaredDistance += dy * dy;
//    }
//    
//    if (delta.GetZ() >= 0)
//    {
//      const T dz = this->fMaxCorner.GetZ() - position.GetZ();
//      maxSquaredDistance += dz * dz;
//    }
//    else
//    {
//      const T dz = this->fMinCorner.GetZ() - position.GetZ();
//      maxSquaredDistance += dz * dz;
//    }
//    
//    return maxSquaredDistance;
    
//    if (delta.GetX() >= 0)
//    {
//      //           ........
//      //           |      |
//      //           |      |
//      //           `------'
//      //     ,..
//      //    |   |
//      //    `..,'
//      // 
//      if (delta.GetY() >= 0)
//      {
//        if (delta.GetZ() >= 0)
//        {
//          const Vec3<T> toRTF(Vec3<T>(this->fMaxCorner.GetX(),
//            this->fMaxCorner.GetY(),
//            this->fMaxCorner.GetZ()) - position);
//          return toRTF.GetSquaredLength();
//        }
//        else
//        {
//          const Vec3<T> toRTN(Vec3<T>(this->fMaxCorner.GetX(),
//            this->fMaxCorner.GetY(),
//            this->fMinCorner.GetZ()) - position);
//          return toRTN.GetSquaredLength();
//        }
//      }
//      //     ,..
//      //    |   |
//      //    `..,'
//      //           ........
//      //           |      |
//      //           |      |
//      //           `------'
//      else
//      {
//        if (delta.GetZ() >= 0)
//        {
//          const Vec3<T> toRBF(Vec3<T>(this->fMaxCorner.GetX(),
//            this->fMinCorner.GetY(),
//            this->fMaxCorner.GetZ()) - position);
//          return toRBF.GetSquaredLength();
//        }
//        else
//        {
//          const Vec3<T> toRBN(Vec3<T>(this->fMaxCorner.GetX(),
//            this->fMinCorner.GetY(),
//            this->fMinCorner.GetZ()) - position);
//          return toRBN.GetSquaredLength();
//        }
//      }
//    }
//    else
//    {
//      //    ........
//      //    |      |
//      //    |      |
//      //    `------'
//      //                  ..,
//      //                 |   |
//      //                 `,..'
//      if (delta.GetY() > 0)
//      {
//        if (delta.GetZ() >= 0)
//        {
//          const Vec3<T> toLTF(Vec3<T>(this->fMinCorner.GetX(),
//            this->fMaxCorner.GetY(),
//            this->fMaxCorner.GetZ()) - position);
//          return toLTF.GetSquaredLength();
//        }
//        else
//        {
//          const Vec3<T> toLTN(Vec3<T>(this->fMinCorner.GetX(),
//            this->fMaxCorner.GetY(),
//            this->fMinCorner.GetZ()) - position);
//          return toLTN.GetSquaredLength();
//        }
//      }    
//      //                  ..,
//      //                 |   |
//      //                 `,..'
//      //    ........
//      //    |      |
//      //    |      |
//      //    `------'
//      else
//      {
//        if (delta.GetZ() >= 0)
//        {
//          const Vec3<T> toLBF(Vec3<T>(this->fMinCorner.GetX(),
//            this->fMinCorner.GetY(),
//            this->fMaxCorner.GetZ()) - position);
//          return toLBF.GetSquaredLength();
//        }
//        else
//        {
//          const Vec3<T> toLBN(Vec3<T>(this->fMinCorner.GetX(),
//            this->fMinCorner.GetY(),
//            this->fMinCorner.GetZ()) - position);
//          return toLBN.GetSquaredLength();
//        }
//
//      }    
//    }
  }

  bool ContainedInBoundingSphere(const Vec3<T>& bsCenter, T bsRadiusSquared) const
  {    
    std::pair<T, T> minMaxDist2 = this->GetMinMaxSquaredDistances(bsCenter);
    if ((minMaxDist2.first < bsRadiusSquared) && (minMaxDist2.second < bsRadiusSquared))
    {
      return true;
    }
    return false;
  }

  bool IntersectsBoundingSphere(const Vec3<T>& bsCenter, T bsRadiusSquared) const
  {
    T dmin = 0;
    
    if( bsCenter.GetX() < this->fMinCorner.GetX() )
    {
      const T dist = bsCenter.GetX() - this->fMinCorner.GetX();
      dmin += ( dist * dist ); 
    }
    else if( bsCenter.GetX() > this->fMaxCorner.GetX() )
    {
      const T dist = bsCenter.GetX() - this->fMaxCorner.GetX();
      dmin += ( dist * dist );
    }

    if( bsCenter.GetY() < this->fMinCorner.GetY() )
    {
      const T dist = bsCenter.GetY() - this->fMinCorner.GetY();
      dmin += ( dist * dist ); 
    }
    else if( bsCenter.GetY() > this->fMaxCorner.GetY() )
    {
      const T dist = bsCenter.GetY() - this->fMaxCorner.GetY();
      dmin += ( dist * dist ); 
    }

    if( bsCenter.GetZ() < this->fMinCorner.GetZ() )
    {
      const T dist = bsCenter.GetZ() - this->fMinCorner.GetZ();
      dmin += ( dist * dist );
    }
    else if( bsCenter.GetZ() > this->fMaxCorner.GetZ() )
    {
      const T dist = bsCenter.GetZ() - this->fMaxCorner.GetZ();
      dmin += ( dist * dist );
    }

    if (dmin <= bsRadiusSquared) 
      return true;
    else
      return false;
  }

protected:
  Vec3<T> fMinCorner;
  Vec3<T> fMaxCorner;
};

typedef AABB_T<double> AABB;

#endif  // #ifndef TRIANGULATION_COMMON_AABB_H_
