##------------------------------------------------------------------------------
## avigle-common -- common classes/tools
##
## Developed during the research project AVIGLE
## which was part of the Hightech.NRW research program
## funded by the ministry for Innovation, Science, Research and Technology
## of the German state Northrhine-Westfalia, and by the European Union.
##
## Copyright (c) 2010--2013, Tom Vierjahn et al.
##------------------------------------------------------------------------------
##                                License
##
## This library/program is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published
## by the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## If you are using this library/program in a project, work or publication,
## please cite [1,2].
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##------------------------------------------------------------------------------
##                                References
##
## [1] S. Rohde, N. Goddemeier, C. Wietfeld, F. Steinicke, K. Hinrichs,
##     T. Ostermann, J. Holsten, D. Moormann:
##     "AVIGLE: A System of Systems Concept for an
##      Avionic Digital Service Platform based on
##      Micro Unmanned Aerial Vehicles".
##     In Proc. IEEE Int'l Conf. Systems Man and Cybernetics (SMC),
##     pp. 459--466. 2010. DOI: 10.1109/ICSMC.2010.5641767
## [2] S. Strothoff, D. Feldmann, F. Steinicke, T. Vierjahn, S. Mostafawy:
##     "Interactive generation of virtual environments using MUAVs".
##     In Proc. IEEE Int. Symp. VR Innovations, pp. 89--96, 2011.
##     DOI: 10.1109/ISVRI.2011.5759608
##------------------------------------------------------------------------------

################################################################################
### multiple include guard
################################################################################
if (NOT TARGET common)



################################################################################
### cmake
################################################################################
cmake_minimum_required(VERSION 2.8)
if (NOT CMAKE_MODULE_PATH)
  set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)
endif()



################################################################################
### project info / version
################################################################################
project(common)
set(VERSION_MAJOR 16)	# year
set(VERSION_MINOR 12)	# month
set(VERSION_PATCH 15)	# day
set(VERSION_TWEAK 00)	# count


# produce version string
set(VERSION_STR "${VERSION_MAJOR}")
set(VERSION_STR "${VERSION_STR}.${VERSION_MINOR}")
set(VERSION_STR "${VERSION_STR}.${VERSION_PATCH}")
set(VERSION_STR "${VERSION_STR}.${VERSION_TWEAK}")

# produce version int
string(REPLACE "." "" VERSION_INT ${VERSION_STR})

# compile version into headers
configure_file(
  ${PROJECT_SOURCE_DIR}/include/common/version.h.in 
  ${PROJECT_SOURCE_DIR}/include/common/version.h 
  @ONLY)

# use version as postfix
set(CMAKE_DEBUG_POSTFIX "-${VERSION_STR}d")
set(CMAKE_RELEASE_POSTFIX "-${VERSION_STR}")


################################################################################
### source files
################################################################################
file(GLOB COMMON_SOURCES src/*)
file(GLOB COMMON_HEADERS include/common/*)
file(GLOB NO_WARNING_HEADERS include/no_warning/*)

### include directories ###
include_directories(./include/)



################################################################################
### dependencies
################################################################################

### boost ###
find_package(Boost REQUIRED thread system)
if(MSVC)
  # Do not link Boost libraries automatically, since we explicitly link in CMake.
  add_definitions(-DBOOST_ALL_NO_LIB)
endif(MSVC)
include_directories(${Boost_INCLUDE_DIRS})



################################################################################
### library
################################################################################
add_library(common SHARED 
  ${COMMON_SOURCES} 
  ${COMMON_HEADERS} 
  ${NO_WARNING_HEADERS}
)

target_link_libraries(common 
  ${Boost_LIBRARIES}
)

set_target_properties(common PROPERTIES 
  COMPILE_FLAGS "${GCC_CUSTOM_WARNING_LEVEL}"
)



################################################################################
### install
################################################################################
configure_file(
  ${PROJECT_SOURCE_DIR}/commonConfig.cmake.in 
  ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/commonConfig.cmake 
  @ONLY)
configure_file(
  ${PROJECT_SOURCE_DIR}/commonConfigVersion.cmake.in 
  ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/commonConfigVersion.cmake 
  @ONLY)
INSTALL(
  FILES ${COMMON_HEADERS} 
  DESTINATION include/${PROJECT_NAME}-${VERSION_STR}/common)
INSTALL(
  FILES ${NO_WARNING_HEADERS} 
  DESTINATION include/${PROJECT_NAME}-${VERSION_STR}/no_warning)
INSTALL(
  TARGETS common 
  DESTINATION lib)
INSTALL(
  FILES ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/commonConfig.cmake
        ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/commonConfigVersion.cmake
  DESTINATION lib/${PROJECT_NAME}-${VERSION_STR}) 



endif() ### multiple include guard
################################################################################
