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

/// This file defines several logging macros for the different severity levels
/// defined in 
/// RFC 5424 - The Syslog Protocol <http://datatracker.ietf.org/doc/rfc5424/>
/// Additionally the lowest level TRACE was used here for convenience
/// Level 6 is called INFO for convenience.


#ifndef TRIANGULATION_COMMON_LOGGING_H_
#define TRIANGULATION_COMMON_LOGGING_H_

#include <sstream>

#include <common/logger.h>


/// \brief Concatenates the logging -- for internal use only
#define INTERNAL_LOG(logLevel, logStream) \
  { \
    const char* pFileName = __FILE__; int lineNumber = __LINE__; \
    std::stringstream logStringStream; \
    logStringStream << logStream; \
    Logger::GetInstance()->Log(logLevel, logStringStream.str(), \
                               pFileName, lineNumber); \
  }


// logging macros for different severity levels according to RFC 5424
#define LOG_ERROR(logStream) \
  INTERNAL_LOG(Logger::kLogLevelError, logStream)
#define LOG_WARNING(logStream) \
  INTERNAL_LOG(Logger::kLogLevelWarning, logStream)
#define LOG_INFO(logStream) \
  INTERNAL_LOG(Logger::kLogLevelInfo, logStream)
#define LOG_DEBUG(logStream) \
  INTERNAL_LOG(Logger::kLogLevelDebug, logStream)

#endif  // #ifndef TRIANGULATION_COMMON_LOGGING_H_
