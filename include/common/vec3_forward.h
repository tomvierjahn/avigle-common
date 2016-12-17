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

#ifndef TRIANGULATION_COMMON_VEC3_FORWARD_H_
#define TRIANGULATION_COMMON_VEC3_FORWARD_H_

template <typename T> class Vec3;
typedef Vec3<float> Vec3f;
typedef Vec3<double> Vec3d;

template <typename T> Vec3<T> crossProduct(const Vec3<T>& v1, const Vec3<T>& v2);
template <typename T> T dotProduct(const Vec3<T>& v1, const Vec3<T>& v2);
template <typename T> Vec3<T> normalize(const Vec3<T>& v1);
template <typename T> Vec3<T> rotate(const Vec3<T>& v, const T angle, const Vec3<T>& axis);
template <typename T> Vec3<T> rotateY(const Vec3<T>& v, const T angle);
template <typename T> Vec3<T> operator+(const Vec3<T>& v1, const Vec3<T>& v2);
template <typename T> Vec3<T> operator-(const Vec3<T>& v1, const Vec3<T>& v2);
template <typename T> Vec3<T>& operator+=(Vec3<T>& v1, const Vec3<T>& v2);
template <typename T> Vec3<T>& operator-=(Vec3<T>& v1, const Vec3<T>& v2);
template <typename T> Vec3<T> operator*(const T s, const Vec3<T>& v);
template <typename T> Vec3<T> operator*(const Vec3<T>& v, const T s);
template <typename T> Vec3<T> operator/(const Vec3<T>& v, const T s);
template <typename T> Vec3<T>& operator*=(Vec3<T>& v, const T s);
template <typename T> Vec3<T>& operator/=(Vec3<T>& v, const T s);
template <typename T> bool operator==(const Vec3<T>& v1, const Vec3<T>& v2);

#endif  // ifndef TRIANGULATION_COMMON_VEC3_FORWARD_H_
