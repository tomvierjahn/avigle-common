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

#ifndef TRIANGULATION_COMMON_LOGGER_H_
#define TRIANGULATION_COMMON_LOGGER_H_


#include <cstddef>

#include <ostream>
#include <string>

#include <common/common_api.h>
#ifndef Q_MOC_RUN
#include <no_warning/boost__thread__mutex.h>
#endif

/// \brief Logger singleton class
class COMMON_API Logger
{
public:
  /// \brief Log levels
  enum LogLevel
  {
    kLogLevelQuiet = 0,
    kLogLevelError,
    kLogLevelWarning,
    kLogLevelInfo,
    kLogLevelDebug,
    kLogLevelMaximum
  };
  
  /// \brief Destructor
  ~Logger()
  {
    this->fpInstance = NULL;
  }
  
  /// \brief Returns a pointer to the singleton. 
  ///         If necessary, an instance is created before.
  static Logger* GetInstance()
  {
    if (Logger::fpInstance == NULL)
    {
      Logger::fpInstance = new Logger();
    }
    return Logger::fpInstance;
  }
  
  /// \brief Sets the output stream for logging
  void SetOutputStream(std::ostream* pOutputStream)
  {
    this->fpOutputStream = pOutputStream;
  }
  
  /// \brief Sets the maximum log level
  void SetMaximumLogLevel(Logger::LogLevel maximumLogLevel)
  {
    this->fMaximumLogLevel = maximumLogLevel;
  }
  
  /// \brief Logging method
  void Log(Logger::LogLevel logLevel, const std::string& logString,
           const char* filename = NULL, int lineNumber = 0) const;
  
private:
  /// \brief Standard constructor
  Logger();
  
  static Logger* fpInstance;      /// \brief stores a pointer to the instance
  std::ostream* fpOutputStream;   /// \brief stores a pointer to the output stream
  LogLevel fMaximumLogLevel;      /// \brief stores the maximum log level
  
  mutable boost::mutex fLoggingMutex; /// \brief Mutex for thread safety
};

#endif  // #ifndef TRIANGULATION_COMMON_LOGGING_H_
