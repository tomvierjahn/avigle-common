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

#ifndef TRIANGULATION_COMMON_VEC3_H_
#define TRIANGULATION_COMMON_VEC3_H_

#include <cmath>

#include <algorithm>
#include <limits>
#include <ostream>

#include <common/vec3_forward.h>
// This file uses std::min and std::max. The windows.h header file 
// (or more correctly, windef.h that it includes in turn) has macros
// for min and max which are interfering. 
// The solution is to #define NOMINMAX before including windows.h it.
// This is done in the appropriate file, but the macros still seem
// to be defined. So undef both of them here.
#undef min
#undef max


template <typename T>
class Vec3
{
public:
  friend class OctreeNode;
  
  Vec3(T v = T(0))
  : fX(v), fY(v), fZ(v)
  {}
  
  Vec3(T x, T y, T z)
  : fX(x), fY(y), fZ(z)
  {}
  
  ~Vec3(){}
  
  //T GetX() const { return this->fX; }
  //T GetY() const { return this->fY; }
  //T GetZ() const { return this->fZ; }

  const T& GetX() const { return this->fX; }
  const T& GetY() const { return this->fY; }
  const T& GetZ() const { return this->fZ; }
  
  void SetX(T x) { this->fX = x; }
  void SetY(T y) { this->fY = y; }
  void SetZ(T z) { this->fZ = z; }
  
  T GetLength() const 
  { 
    return sqrt(this->fX * this->fX + this->fY * this->fY + this->fZ * this->fZ); 
  }
  
  T GetSquaredLength() const 
  { 
    return this->fX * this->fX + this->fY * this->fY + this->fZ * this->fZ; 
  }
  
  T DistanceTo(const Vec3<T>& v) const
  { 
    return (*this - v).GetLength();
  }
  
  static Vec3<T> MinComponents(const Vec3<T>& v1, const Vec3<T>& v2)
  {
    return Vec3<T>(std::min(v1.fX, v2.fX), std::min(v1.fY, v2.fY), 
                   std::min(v1.fZ, v2.fZ));
  }

  static Vec3<T> MaxComponents(const Vec3<T>& v1, const Vec3<T>& v2)
  {
    return Vec3<T>(std::max(v1.fX, v2.fX), std::max(v1.fY, v2.fY), 
                   std::max(v1.fZ, v2.fZ));
  }

  inline void AsmFldX();
  inline void AsmFldY();
  inline void AsmFldZ();
  
  friend Vec3<T> crossProduct<>(const Vec3<T>& v1, const Vec3<T>& v2);
  friend T dotProduct<>(const Vec3<T>& v1, const Vec3<T>& v2);
  friend Vec3<T> normalize<>(const Vec3<T>& v1);
  
  friend Vec3<T> rotate<>(const Vec3<T>& v, const T angle, const Vec3<T>& axis);
  friend Vec3<T> rotateY<>(const Vec3<T>& v, const T angle);
  
  friend Vec3<T> operator+<>(const Vec3<T>& v1, const Vec3<T>& v2);
  friend Vec3<T> operator-<>(const Vec3<T>& v1, const Vec3<T>& v2);
  
  friend Vec3<T>& operator+=<>(Vec3<T>& v1, const Vec3<T>& v2);
  friend Vec3<T>& operator-=<>(Vec3<T>& v1, const Vec3<T>& v2);
  
  friend Vec3<T> operator*<>(const T s, const Vec3<T>& v);
  friend Vec3<T> operator*<>(const Vec3<T>& v, const T s);
  friend Vec3<T> operator/<>(const Vec3<T>& v, const T s);
  
  friend Vec3<T>& operator*=<>(Vec3<T>& v, const T s);
  friend Vec3<T>& operator/=<>(Vec3<T>& v, const T s);
  
private:
  T fX;
  T fY;
  T fZ;
};






////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T> crossProduct(const Vec3<T>& v1, const Vec3<T>& v2)
{
  return Vec3<T>(v1.fY * v2.fZ - v1.fZ * v2.fY,
                 v1.fZ * v2.fX - v1.fX * v2.fZ,
                 v1.fX * v2.fY - v1.fY * v2.fX);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline T dotProduct(const Vec3<T>& v1, const Vec3<T>& v2)
{
  return (v1.fX * v2.fX + v1.fY * v2.fY + v1.fZ * v2.fZ);
}





////////////////////////////////////////////////////////////////////////////////
/// \brief Calculate normalized vector
/// \return null-vector, if v1 = null-vector, normalized(v1) otherwise
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T> normalize(const Vec3<T>& v1)
{
  T abs = sqrt(v1.fX * v1.fX + v1.fY * v1.fY + v1.fZ * v1.fZ);
  if (abs <= std::numeric_limits<T>::epsilon())
  {
    return Vec3<T>(0.0, 0.0, 0.0);
  }

  T absInv = 1.0 / abs;
  return Vec3<T>(absInv * v1.fX, absInv * v1.fY, absInv * v1.fZ);
}





////////////////////////////////////////////////////////////////////////////////
/// \brief Calculate normalized vector
/// \return null-vector, if v1 = null-vector, normalized(v1) otherwise
////////////////////////////////////////////////////////////////////////////////
template <>
inline Vec3<double> normalize(const Vec3<double>& v1)
{
  double abs = sqrt(v1.fX * v1.fX + v1.fY * v1.fY + v1.fZ * v1.fZ);

  if (abs <= std::numeric_limits<double>::epsilon())
  {
    return Vec3<double>(0.0, 0.0, 0.0);
  }

  double absInv = 1.0 / abs;
  return Vec3<double>(absInv * v1.fX, absInv * v1.fY, absInv * v1.fZ);
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T> rotate(const Vec3<T>& v, const T angle, const Vec3<T>& axis)
{
  const T rad = angle / 180.0 * 3.1415926;
  const T c = cos(rad);
  const T s = sin(rad);
  const T t = 1.0 - c;
  
  const T X = axis.fX;
  const T Y = axis.fY;
  const T Z = axis.fZ;
  
  return Vec3<T>((t * X * X + c) * v.fX + 
                   (t * X * Y + s * Z) * v.fY + 
                   (t * X * Z - s * Y) * v.fZ,
                 (t * X * Y - s * Z) * v.fX +
                   (t * Y * Y + c) * v.fY +
                   (t * Y * Z + s * X) * v.fZ,
                 (t * X * Y + s * Y) * v.fX +
                   (t * Y * Z - s * X) * v.fY +
                   (t * Z * Z + c) * v.fZ);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <>
inline Vec3<double> rotate(const Vec3<double>& v, const double angle, const Vec3<double>& axis)
{
  static const double pi = 4.0 * atan(1.0);
  const double rad = angle / 180.0 * pi;
  const double c = cos(rad);
  const double s = sin(rad);
  const double t = 1.0 - c;

  const double X = axis.fX;
  const double Y = axis.fY;
  const double Z = axis.fZ;

  return Vec3<double>((t * X * X + c) * v.fX +
                   (t * X * Y + s * Z) * v.fY +
                   (t * X * Z - s * Y) * v.fZ,
                 (t * X * Y - s * Z) * v.fX +
                   (t * Y * Y + c) * v.fY +
                   (t * Y * Z + s * X) * v.fZ,
                 (t * X * Y + s * Y) * v.fX +
                   (t * Y * Z - s * X) * v.fY +
                   (t * Z * Z + c) * v.fZ);
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T> rotateY(const Vec3<T>& v, const T angle)
{
  const T rad = angle / 180.0 * 3.1415926;
  const T c = cos(rad);
  const T s = sin(rad);
  
  return Vec3<T>(c * v.fX + s * v.fZ,
                 v.fY,
                 -s * v.fX + s * v.fZ);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <>
inline Vec3<double> rotateY(const Vec3<double>& v, const double angle)
{
  static const double pi = 4.0 * atan(1.0);
  const double rad = angle / 180.0 * pi;
  const double c = cos(rad);
  const double s = sin(rad);

  return Vec3<double>(c * v.fX + s * v.fZ,
		             v.fY,
                     -s * v.fX + s * v.fZ);
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T> operator+(const Vec3<T>& v1, const Vec3<T>& v2)
{
  return Vec3<T>(v1.fX + v2.fX, v1.fY + v2.fY, v1.fZ + v2.fZ);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T> operator-(const Vec3<T>& v1, const Vec3<T>& v2)
{
  return Vec3<T>(v1.fX - v2.fX, v1.fY - v2.fY, v1.fZ - v2.fZ);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T>& operator+=(Vec3<T>& v1, const Vec3<T>& v2)
{
  v1 = v1 + v2;
  return v1;
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T>& operator-=(Vec3<T>& v1, const Vec3<T>& v2)
{
  v1 = v1 - v2;
  return v1;
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T> operator*(const T s, const Vec3<T>& v)
{
  return Vec3<T>(s * v.fX, s * v.fY, s * v.fZ);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T> operator*(const Vec3<T>& v, const T s)
{
  return Vec3<T>(s * v.fX, s * v.fY, s * v.fZ);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T> operator/(const Vec3<T>& v, const T s)
{
  return Vec3<T>((1.0 / s) * v);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T>& operator*=(Vec3<T>& v, const T s)
{
  return (v = v * s);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline Vec3<T>& operator/=(Vec3<T>& v, const T s)
{
  return (v = v / s);
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline bool operator==(const Vec3<T>& v1, const Vec3<T>& v2)
{
  return v1.GetX() == v2.GetX()
      && v1.GetY() == v2.GetY()
      && v1.GetZ() == v2.GetZ();
}





////////////////////////////////////////////////////////////////////////////////
/// 
////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline std::ostream& operator<<(std::ostream& os, const Vec3<T>& v)
{
  os << "(" << v.GetX() << " ; " << v.GetY() << " ; " << v.GetZ() << ")";
  return os;
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <>
inline void
Vec3<float>::AsmFldX
()
{
  __asm__ __volatile__ ("flds (%[ptr]);\n\t"
                        : /* no output */
                        : [ptr] "r" (&this->fX));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <>
inline void
Vec3<float>::AsmFldY
()
{
  __asm__ __volatile__ ("flds (%[ptr]);\n\t"
                        : /* no output */
                        : [ptr] "r" (&this->fY));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <>
inline void
Vec3<float>::AsmFldZ
()
{
  __asm__ __volatile__ ("flds (%[ptr]);\n\t"
                        : /* no output */
                        : [ptr] "r" (&this->fZ));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <>
inline void
Vec3<double>::AsmFldX
()
{
  __asm__ __volatile__ ("fldl (%[ptr]);\n\t"
                        : /* no output */
                        : [ptr] "r" (&this->fX));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <>
inline void
Vec3<double>::AsmFldY
()
{
  __asm__ __volatile__ ("fldl (%[ptr]);\n\t"
                        : /* no output */
                        : [ptr] "r" (&this->fY));
}





////////////////////////////////////////////////////////////////////////////////
///
////////////////////////////////////////////////////////////////////////////////
template <>
inline void
Vec3<double>::AsmFldZ
()
{
  __asm__ __volatile__ ("fldl (%[ptr]);\n\t"
                        : /* no output */
                        : [ptr] "r" (&this->fZ));
}

#endif  // ifndef TRIANGULATION_COMMON_VEC3_H_
