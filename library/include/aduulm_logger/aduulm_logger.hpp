/**
 * \file aduulmlogger.h
 *
 * @brief Remaps ADTF log commands to std::cout (as before) or ROS.
 * Remapping to ROS if built with -DHAVE_LOG
 */
#if !defined(_ADUULM_LOGGER_H_) and !defined(_LOGGER_H_)  // Guard aduulm_logger from being included if Defines/Logger
                                                          // is already included in project and vice a versa
#define _ADUULM_LOGGER_H_

#if defined(IS_ROS) || defined(USE_ROS_LOG)
#include <ros/ros.h>
#endif

#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <thread>
#include <mutex>
#include <fstream>
#include <array>
#include <vector>
#include <boost/function.hpp>
#include "boost/bind.hpp"

#define UTC_TIME false

#define LOG_RESET "\033[0m"
#define LOG_NORMAL "\033[m\017"
#define LOG_RED "\033[1m\033[31m"
#define LOG_GREEN "\033[1m\033[32m"
#define LOG_YELLOW "\033[1m\033[33m"
//#define LOG_WHITE "\033[1m"
#define LOG_WHITE "\033[37m"   /* White */
#define LOG_BLUE "\033[34m"    /* Blue */
#define LOG_MAGENTA "\033[35m" /* Magenta */
#define LOG_CYAN "\033[36m"    /* Cyan */
#define LOG_GRAY "\033[0;37m"
#define LOG_BOLDBLUE "\033[1m\033[34m" /* Bold Blue */

namespace aduulm_logger
{
namespace LoggerLevels
{
enum Level
{
  None,
  Error,
  Warn,
  Info,
  Debug
};
}
typedef LoggerLevels::Level LoggerLevel;

extern std::recursive_mutex g_oLoggerMutex;
extern std::ofstream g_oFile;
extern std::string g_file_name;
extern LoggerLevel g_log_level;
extern bool g_log_to_file;
extern int g_nLogCount;
extern int g_nLogNr;
extern std::string g_stream_name;
using LoggerInitCallback = boost::function<void(void)>;
using LoggerLevelChangeCallback = boost::function<void(LoggerLevel log_level)>;
using LoggerStreamNameChangeCallback = boost::function<void(std::string stream_name)>;
extern std::vector<LoggerInitCallback> g_sublogger_init_callbacks;
extern std::vector<LoggerLevelChangeCallback> g_sublogger_level_change_callbacks;
extern std::vector<LoggerStreamNameChangeCallback> g_sublogger_stream_name_change_callbacks;

#if defined(IS_ROS) || defined(USE_ROS_LOG)
extern const std::array<ros::console::Level, 5> level_mapping;
#endif

static inline void CheckLogCnt()
{
  ++g_nLogCount;
  if (g_nLogCount > 100000)
  {
    g_oFile.close();
    std::string _strLogname = g_file_name + std::to_string(g_nLogNr) + ".log";
    g_oFile.open(_strLogname);  //, std::ios_base::app);
    ++g_nLogNr;
    g_nLogCount = 0;
  }
}
}  // namespace aduulm_logger

#if defined(IS_ROS) || defined(USE_ROS_LOG)
#define LOGGER_ROS_EXTRA_DEFINES                                                                                       \
  std::string __attribute__((visibility("hidden"))) g_stream_name = ROSCONSOLE_DEFAULT_NAME;                           \
  const std::array<ros::console::Level, 5> __attribute__((visibility("hidden")))                                       \
      level_mapping = { ros::console::levels::Fatal,                                                                   \
                        ros::console::levels::Error,                                                                   \
                        ros::console::levels::Warn,                                                                    \
                        ros::console::levels::Info,                                                                    \
                        ros::console::levels::Debug };
#else
#define LOGGER_ROS_EXTRA_DEFINES std::string __attribute__((visibility("hidden"))) g_stream_name = "default";
#endif

#define DEFINE_LOGGER_VARIABLES                                                                                        \
  namespace aduulm_logger                                                                                              \
  {                                                                                                                    \
  std::recursive_mutex __attribute__((visibility("hidden"))) g_oLoggerMutex;                                           \
  std::ofstream __attribute__((visibility("hidden"))) g_oFile;                                                         \
  std::string __attribute__((visibility("hidden"))) g_file_name;                                                       \
  LoggerLevel __attribute__((visibility("hidden"))) g_log_level = LoggerLevels::Warn;                                  \
  bool __attribute__((visibility("hidden"))) g_log_to_file = false;                                                    \
  int __attribute__((visibility("hidden"))) g_nLogCount = 0;                                                           \
  int __attribute__((visibility("hidden"))) g_nLogNr = 0;                                                              \
  std::vector<LoggerInitCallback> __attribute__((visibility("hidden"))) g_sublogger_init_callbacks;                    \
  std::vector<LoggerLevelChangeCallback> __attribute__((visibility("hidden"))) g_sublogger_level_change_callbacks;     \
  std::vector<LoggerStreamNameChangeCallback> __attribute__((visibility("hidden")))                                    \
      g_sublogger_stream_name_change_callbacks;                                                                        \
  LOGGER_ROS_EXTRA_DEFINES                                                                                             \
  }

#define LOGGER_ADD_SUBLOGGER_LIBRARY(_namespace)                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    aduulm_logger::g_sublogger_init_callbacks.emplace_back(_namespace::_initLogger);                                   \
    aduulm_logger::g_sublogger_level_change_callbacks.emplace_back(_namespace::_setLogLevel);                          \
    aduulm_logger::g_sublogger_stream_name_change_callbacks.emplace_back(_namespace::_setStreamName);                  \
  } while (0)

#define LOGGER_ADD_SUBLOGGER_CLASS(_class, _instance)                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    aduulm_logger::g_sublogger_init_callbacks.emplace_back(boost::bind(&_class::_initLogger, _instance));              \
    aduulm_logger::g_sublogger_level_change_callbacks.emplace_back(boost::bind(&_class::_setLogLevel, _instance, _1)); \
    aduulm_logger::g_sublogger_stream_name_change_callbacks.emplace_back(                                              \
        boost::bind(&_class::_setStreamName, _instance, _1));                                                          \
  } while (0)

#define DEFINE_LOGGER_LIBRARY_INTERFACE_HEADER                                                                         \
  void _initLogger();                                                                                                  \
  void _setStreamName(std::string stream_name);                                                                        \
  void _setLogLevel(aduulm_logger::LoggerLevel log_level);

#define DEFINE_LOGGER_LIBRARY_INTERFACE_IMPLEMENTATION                                                                 \
  void _initLogger()                                                                                                   \
  {                                                                                                                    \
    aduulm_logger::initLogger();                                                                                       \
    for (auto callback : aduulm_logger::g_sublogger_init_callbacks)                                                    \
    {                                                                                                                  \
      callback();                                                                                                      \
    }                                                                                                                  \
  }                                                                                                                    \
                                                                                                                       \
  void _setStreamName(std::string stream_name)                                                                         \
  {                                                                                                                    \
    aduulm_logger::setStreamName(stream_name);                                                                         \
    for (auto callback : aduulm_logger::g_sublogger_stream_name_change_callbacks)                                      \
    {                                                                                                                  \
      callback(stream_name);                                                                                           \
    }                                                                                                                  \
  }                                                                                                                    \
                                                                                                                       \
  void _setLogLevel(aduulm_logger::LoggerLevel log_level)                                                              \
  {                                                                                                                    \
    aduulm_logger::setLogLevel(log_level);                                                                             \
    for (auto callback : aduulm_logger::g_sublogger_level_change_callbacks)                                            \
    {                                                                                                                  \
      callback(log_level);                                                                                             \
    }                                                                                                                  \
  }

#define DEFINE_LOGGER_CLASS_INTERFACE_HEADER                                                                           \
  void _initLogger();                                                                                                  \
  void _setStreamName(std::string stream_name);                                                                        \
  void _setLogLevel(aduulm_logger::LoggerLevel log_level);

#define DEFINE_LOGGER_CLASS_INTERFACE_IMPLEMENTATION(_class)                                                           \
  void _class ::_initLogger()                                                                                          \
  {                                                                                                                    \
    aduulm_logger::initLogger();                                                                                       \
    for (auto callback : aduulm_logger::g_sublogger_init_callbacks)                                                    \
    {                                                                                                                  \
      callback();                                                                                                      \
    }                                                                                                                  \
  }                                                                                                                    \
                                                                                                                       \
  void _class ::_setStreamName(std::string stream_name)                                                                \
  {                                                                                                                    \
    aduulm_logger::setStreamName(stream_name);                                                                         \
    for (auto callback : aduulm_logger::g_sublogger_stream_name_change_callbacks)                                      \
    {                                                                                                                  \
      callback(stream_name);                                                                                           \
    }                                                                                                                  \
  }                                                                                                                    \
                                                                                                                       \
  void _class ::_setLogLevel(aduulm_logger::LoggerLevel log_level)                                                     \
  {                                                                                                                    \
    aduulm_logger::setLogLevel(log_level);                                                                             \
    for (auto callback : aduulm_logger::g_sublogger_level_change_callbacks)                                            \
    {                                                                                                                  \
      callback(log_level);                                                                                             \
    }                                                                                                                  \
  }

/** use it remove all the directories from __FILE__. */
namespace DataTypesLogger
{
__inline__ std::string simpleFileName(const std::string& file)
{
  unsigned int i = file.rfind("/");
  if (file.substr(i + 1, file.size() - 1).size() > 24)
  {
    return file.substr(i + 1, 10) + ".." + file.substr(file.size() - 12, 12);
  }
  return file.substr(i + 1, file.size() - 1);
}

__inline__ std::string longTime()
{
  auto _aNow = std::chrono::system_clock::now();
  std::time_t _ctNow = std::chrono::system_clock::to_time_t(_aNow);
  std::tm _tmBrokenTime;
  ;
  if (UTC_TIME)
    _tmBrokenTime = *std::gmtime(&_ctNow);
  else
    _tmBrokenTime = *std::localtime(&_ctNow);

  std::time_t _nUSSinceDay = std::chrono::duration_cast<std::chrono::microseconds>(
                                 _aNow - std::chrono::time_point_cast<std::chrono::seconds>(_aNow))
                                 .count();  // Millisecond fraction

  _nUSSinceDay += _tmBrokenTime.tm_hour * 60ULL * 60 * 1000 * 1000;  // microseconds for the hours today
  _nUSSinceDay += _tmBrokenTime.tm_min * 60ULL * 1000 * 1000;        // microseconds for the minutes today
  _nUSSinceDay += _tmBrokenTime.tm_sec * 1000ULL * 1000;             // microseconds for the seconds today

  int64_t _ntm_us =
      _nUSSinceDay - ((_tmBrokenTime.tm_hour * 3600 + _tmBrokenTime.tm_min * 60 + _tmBrokenTime.tm_sec) * 1000000ULL);
  char buf[40];
  sprintf(buf,
          "%04d-%02d-%02d %02d:%02d:%02d.%06ld",
          _tmBrokenTime.tm_year + 1900,
          _tmBrokenTime.tm_mon + 1,
          _tmBrokenTime.tm_mday,
          _tmBrokenTime.tm_hour,
          _tmBrokenTime.tm_min,
          _tmBrokenTime.tm_sec,
          _ntm_us);

  return buf;
}
__inline__ std::thread::id thread_id()
{
  return std::this_thread::get_id();
}
}  // namespace DataTypesLogger
#define _LOG_BASE                                                                                                      \
  "(" << std::setw(24) << DataTypesLogger::simpleFileName(__FILE__) << ":" << std::setw(3) << __LINE__ << ") "         \
      << std::setw(30) << __FUNCTION__ << "() "  // NOLINT(bugprone-lambda-function-name)

// Log Level:
// 1 = ERR
// 2 = WARN
// 3 = INF
// 4 = DEBUG
//
// Set the default log level

#if defined(IS_ROS) || defined(USE_ROS_LOG)
#ifndef LOG_FATAL
#define LOG_FATAL(expr) ROS_FATAL_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_BASE << ": " << expr)
#endif

#ifndef LOG_ERR
#define LOG_ERR(expr) ROS_ERROR_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_BASE << ": " << expr)
#endif

#ifndef LOG_WARN
#define LOG_WARN(expr) ROS_WARN_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_BASE << ": " << expr)
#endif

#ifndef LOG_INF
#define LOG_INF(expr) ROS_INFO_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_BASE << ": " << expr)
#endif

#ifndef LOG_DEB
#define LOG_DEB(expr) ROS_DEBUG_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_BASE << ": " << expr)
/* #define LOG_DEB(expr) do { ROS_DEBUG_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_BASE << ": " << expr); std::cout
 * << expr << " (" << &aduulm_logger::g_stream_name << ")" << std::endl; } while(0) */
#endif

#else  // IS_ROS

#ifndef LOG_FATAL
#define LOG_FATAL(expr)                                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    std::lock_guard<std::mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                                           \
    aduulm_logger::CheckLogCnt();                                                                                      \
    std::cout << LOG_MAGENTA << "[FATAL] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr        \
              << LOG_NORMAL << std::endl;                                                                              \
    if (aduulm_logger::g_log_to_file)                                                                                  \
    {                                                                                                                  \
      aduulm_logger::g_oFile << "[FATAL] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr        \
                             << std::endl;                                                                             \
    }                                                                                                                  \
  } while (0)
#endif

#ifndef LOG_ERR
#define LOG_ERR(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_log_level >= aduulm_logger::LoggerLevel::Error)                                               \
    {                                                                                                                  \
      std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                               \
      aduulm_logger::CheckLogCnt();                                                                                    \
      std::cout << LOG_RED << "[ERROR] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr          \
                << LOG_NORMAL << std::endl;                                                                            \
      if (aduulm_logger::g_log_to_file)                                                                                \
      {                                                                                                                \
        aduulm_logger::g_oFile << "[ERROR] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr      \
                               << std::endl;                                                                           \
      }                                                                                                                \
    }                                                                                                                  \
  } while (0)
#endif

#ifndef LOG_WARN
#define LOG_WARN(expr)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_log_level >= aduulm_logger::LoggerLevel::Warn)                                                \
    {                                                                                                                  \
      std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                               \
      aduulm_logger::CheckLogCnt();                                                                                    \
      std::cout << LOG_YELLOW << "[WARN ] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr       \
                << LOG_NORMAL << std::endl;                                                                            \
      if (aduulm_logger::g_log_to_file)                                                                                \
      {                                                                                                                \
        aduulm_logger::g_oFile << "[WARN ] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr      \
                               << std::endl;                                                                           \
      }                                                                                                                \
    }                                                                                                                  \
  } while (0)
#else
#define LOG_WARN(expr) ;
#endif

#ifndef LOG_INF
#define LOG_INF(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_log_level >= aduulm_logger::LoggerLevel::Info)                                                \
    {                                                                                                                  \
      std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                               \
      aduulm_logger::CheckLogCnt();                                                                                    \
      std::cout << LOG_GREEN << "[INFO ] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr        \
                << LOG_NORMAL << std::endl;                                                                            \
      if (aduulm_logger::g_log_to_file)                                                                                \
      {                                                                                                                \
        aduulm_logger::g_oFile << "[INFO ] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr      \
                               << std::endl;                                                                           \
      }                                                                                                                \
    }                                                                                                                  \
  } while (0)
#else
#define LOG_INF(expr) ;
#endif

#ifndef LOG_DEBUG
#define LOG_DEB(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_log_level >= aduulm_logger::LoggerLevel::Debug)                                               \
    {                                                                                                                  \
      std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                               \
      aduulm_logger::CheckLogCnt();                                                                                    \
      std::cout << LOG_BLUE << "[DEBUG] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr         \
                << LOG_NORMAL << std::endl;                                                                            \
      if (aduulm_logger::g_log_to_file)                                                                                \
      {                                                                                                                \
        aduulm_logger::g_oFile << "[DEBUG] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr      \
                               << std::endl;                                                                           \
      }                                                                                                                \
    }                                                                                                                  \
  } while (0)
#else
#define LOG_DEB(expr) ;
#endif
#endif  // !IS_ROS

#ifdef LOG_TRACEING
#ifndef LOG_TRACE
#define LOG_TRACE(expr)                                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                                 \
    aduulm_logger::CheckLogCnt();                                                                                      \
    std::cout << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec \
              << ")" << LOG_BLUE << std::setw(9) << " TRACE " << LOG_NORMAL << _LOG_BASE << ": " << LOG_BLUE << expr   \
              << LOG_NORMAL << std::endl;                                                                              \
    aduulm_logger::g_oFile << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex                                \
                           << DataTypesLogger::thread_id() << std::dec << ")" << std::setw(9) << " TRACE "             \
                           << _LOG_BASE << ": " << expr << std::endl;                                                  \
  } while (0)
#endif
#else
#ifndef LOG_TRACE
#define LOG_TRACE(expr) ;
#endif
#endif

#ifdef LOG_VERBOSE
#ifndef LOG_VERB
#define LOG_VERB(expr)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                                 \
    LOG_WARN("Use of LOG_VERB() is deprecated, please use LOG_DEB() instead.");                                        \
    aduulm_logger::CheckLogCnt();                                                                                      \
    std::cout << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec \
              << ")" << std::setw(9) << " VERBOSE " << _LOG_BASE << ": " << expr << std::endl;                         \
    aduulm_logger::g_oFile << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex                                \
                           << DataTypesLogger::thread_id() << std::dec << ")" << std::setw(9) << " VERBOSE "           \
                           << _LOG_BASE << ": " << expr << std::endl;                                                  \
  } while (0)
#endif
#else
#ifndef LOG_VERB
#define LOG_VERB(expr) ;
#endif
#endif

// TODO: Implement
#ifndef LOG_EVAL
#define LOG_EVAL(expr) ;
//#define LOG_EVAL(expr) std::cout << "W " << _LOG_BASE << ": "  << LOG_BLUE << expr << LOG_NORMAL << std::endl;
#endif

#ifndef PROGRESS
#define PROGRESS(_PRGS)                                                                                                \
  const int _BARWidth = 70;                                                                                            \
  std::cout << LOG_BOLDBLUE << "[";                                                                                    \
  int pos = _BARWidth * _PRGS;                                                                                         \
  for (int i = 0; i < _BARWidth; ++i)                                                                                  \
  {                                                                                                                    \
    if (i < pos)                                                                                                       \
      std::cout << "=";                                                                                                \
    else if (i == pos)                                                                                                 \
      std::cout << ">";                                                                                                \
    else                                                                                                               \
      std::cout << " ";                                                                                                \
  }                                                                                                                    \
  std::cout << std::setprecision(2) << "] " << int(_PRGS * 100.0) << " %\r";                                           \
  std::cout.flush();                                                                                                   \
  std::cout << LOG_NORMAL;
#endif

#ifndef DISABLE_LOG_VAR
///////////////
// begin LOG_VAR code
//
// LOG_VAR extends LOG_INF etc to also log expression's values together with their names.
//
// LOG_VAR usage:
// std::string str = "str";
// int i = 3;
// Eigen::MatrixXd mat(2,2);
// mat << 1,2,3,4;
//
// LOG_VAR("Variables", str, "integer", i, mat, mat->determinant());
// output:
// I (          logvar_test.cp: 55) main() : Variables str='str', integer i=3,
//                                        mat=
//                                        [1, 2]
//                                        [3, 4],
//                                        mat.determinant()=-2
//
// any number of arguments can be used for LOG_VAR, up to 10
// You can also call LOG_WARN_VAR which uses LOG_WARN instead of LOG_INF to print
// out the variables.
// LOG_DEB_VAR and LOG_ERR_VAR also exist.

////////////
// CHECK_IF_IS_EXPRESSION
// returns bool value true if NAME is a expression as opposed to a literal
// examples:
// std::string s = "123";
// CHECK_IF_IS_EXPRESSION(s) returns true
// CHECK_IF_IS_EXPRESSION("abc") returns false
#ifndef CHECK_IF_IS_EXPRESSION
#define CHECK_IF_IS_EXPRESSION(VAR)                                                                                    \
  std::char_traits<char>::length(#VAR) > 0 && #VAR[0] == '"' && #VAR[std::char_traits<char>::length(#VAR) - 1] == '"'
#endif

#ifndef STRINGIFY_VAR
#include <iostream>
namespace LOGGER_H_INTERNAL
{
template <class T>
inline std::string stringify(T arg)
{
  std::stringstream ss;
  ss << arg;
  return ss.str();
}

template <>
inline std::string stringify(std::string arg)
{
  std::stringstream ss;
  ss << "'" << arg << "'";
  return ss.str();
}

template <>
inline std::string stringify(const char* arg)
{
  std::stringstream ss;
  ss << "'" << arg << "'";
  return ss.str();
}

template <class T>
std::string stringify(bool isLiteral, std::string name, T arg, bool isLastValue = false);

#ifdef EIGEN_MATRIX_H
// beginning of special output handling for Eigen Matrixes
const std::string FILLER = "\t\t\t\t\t";
template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline std::string stringify(Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> arg)
{
  std::stringstream ss;
  //    Eigen::IOFormat SingleLine_MatrixFormat(3, Eigen::DontAlignCols, "  ", ";", "{", "}", "[", "]");
  //    Eigen::IOFormat MatrixFormat2(3, Eigen::DontAlignCols, "  ", ";\n", "{", "}", "[", "]");
  //    Eigen::IOFormat MatrixFormat3(3, Eigen::DontAlignCols, "  ", ";\nt\t\t\t", "{", "}", "[", "]");
  Eigen::IOFormat MatrixFormat4(4, 0, ", ", "\n" + FILLER, "[", "]");

  ss << "\n" << FILLER << arg.format(MatrixFormat4) << ",\n" << FILLER;

  return ss.str();
}

// create a new line before a matrix
template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline std::string stringify(bool isLiteral,
                             std::string name,
                             Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> arg,
                             bool isLastValue = false)
{
  std::stringstream ss;
  if (isLiteral)
    ss << arg << " ";
  else
    ss << "\n" << FILLER << name << "=" << stringify(arg) << (isLastValue ? "" : "");
  return ss.str();
}
#endif  // EIGEN_MATRIX_H

template <class T>
inline std::string stringify(bool isLiteral, std::string name, T arg, bool isLastValue)
{
  std::stringstream ss;
  if (isLiteral)
    ss << arg << " ";
  else
    ss << name << "=" << stringify(arg) << (isLastValue ? "" : ", ");
  return ss.str();
}
}  // namespace LOGGER_H_INTERNAL

#define STRINGIFY_VAR(VAR) LOGGER_H_INTERNAL::stringify(CHECK_IF_IS_EXPRESSION(VAR), #VAR, VAR)
#define STRINGIFY_VAR_LAST(VAR) LOGGER_H_INTERNAL::stringify(CHECK_IF_IS_EXPRESSION(VAR), #VAR, VAR, true)
#endif  // STRINGIFY_VAR

// The following macros implement a macro "overloading" for each number of arguments
#ifndef LOG_X_VAR_1
#define LOG_X_VAR_1(macroname, var) macroname(STRINGIFY_VAR_LAST(var));
#endif

#ifndef LOG_X_VAR_2
#define LOG_X_VAR_2(macroname, var1, var2) macroname(STRINGIFY_VAR(var1) << STRINGIFY_VAR_LAST(var2));
#endif

#ifndef LOG_X_VAR_3
#define LOG_X_VAR_3(macroname, var1, var2, var3)                                                                       \
  macroname(STRINGIFY_VAR(var1) << STRINGIFY_VAR(var2) << STRINGIFY_VAR_LAST(var3));
#endif

#ifndef LOG_X_VAR_4
#define LOG_X_VAR_4(macroname, var1, var2, var3, var4)                                                                 \
  macroname(STRINGIFY_VAR(var1) << STRINGIFY_VAR(var2) << STRINGIFY_VAR(var3) << STRINGIFY_VAR_LAST(var4));
#endif

#ifndef LOG_X_VAR_5
#define LOG_X_VAR_5(macroname, var1, var2, var3, var4, var5)                                                           \
  macroname(STRINGIFY_VAR(var1) << STRINGIFY_VAR(var2) << STRINGIFY_VAR(var3) << STRINGIFY_VAR(var4)                   \
                                << STRINGIFY_VAR_LAST(var5));
#endif

#ifndef LOG_X_VAR_6
#define LOG_X_VAR_6(macroname, var1, var2, var3, var4, var5, var6)                                                     \
  macroname(STRINGIFY_VAR(var1) << STRINGIFY_VAR(var2) << STRINGIFY_VAR(var3) << STRINGIFY_VAR(var4)                   \
                                << STRINGIFY_VAR(var5) << STRINGIFY_VAR_LAST(var6));
#endif

#ifndef LOG_X_VAR_7
#define LOG_X_VAR_7(macroname, var1, var2, var3, var4, var5, var6, var7)                                               \
  macroname(STRINGIFY_VAR(var1) << STRINGIFY_VAR(var2) << STRINGIFY_VAR(var3) << STRINGIFY_VAR(var4)                   \
                                << STRINGIFY_VAR(var5) << STRINGIFY_VAR(var6) << STRINGIFY_VAR_LAST(var7));
#endif

#ifndef LOG_X_VAR_8
#define LOG_X_VAR_8(macroname, var1, var2, var3, var4, var5, var6, var7, var8)                                         \
  macroname(STRINGIFY_VAR(var1) << STRINGIFY_VAR(var2) << STRINGIFY_VAR(var3) << STRINGIFY_VAR(var4)                   \
                                << STRINGIFY_VAR(var5) << STRINGIFY_VAR(var6) << STRINGIFY_VAR(var7)                   \
                                << STRINGIFY_VAR_LAST(var8));
#endif

#ifndef LOG_X_VAR_9
#define LOG_X_VAR_9(macroname, var1, var2, var3, var4, var5, var6, var7, var8, var9)                                   \
  macroname(STRINGIFY_VAR(var1) << STRINGIFY_VAR(var2) << STRINGIFY_VAR(var3) << STRINGIFY_VAR(var4)                   \
                                << STRINGIFY_VAR(var5) << STRINGIFY_VAR(var6) << STRINGIFY_VAR(var7)                   \
                                << STRINGIFY_VAR(var8) << STRINGIFY_VAR_LAST(var9));
#endif

#ifndef LOG_X_VAR_10
#define LOG_X_VAR_10(macroname, var1, var2, var3, var4, var5, var6, var7, var8, var9, var10)                           \
  macroname(STRINGIFY_VAR(var1) << STRINGIFY_VAR(var2) << STRINGIFY_VAR(var3) << STRINGIFY_VAR(var4)                   \
                                << STRINGIFY_VAR(var5) << STRINGIFY_VAR(var6) << STRINGIFY_VAR(var7)                   \
                                << STRINGIFY_VAR(var8) << STRINGIFY_VAR(var9) << STRINGIFY_VAR_LAST(var10));
#endif

// Since Macros cant be overloaded, we have to work around that through
// using GET_RIGHT_MACRO_OVERLOAD
// Thus LOG_VAR_X can be called with up to 11 arguments (MACRONAME + 10 arguments)
#ifndef GET_RIGHT_MACRO_OVERLOAD
#define GET_RIGHT_MACRO_OVERLOAD(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, NAME, ...) NAME
#endif

// Lets say we call LOG_X_VAR(LOG_INF, x,y,z)
// that means __VA_ARGS__ resembles 3 arguments, making parameter NAME in GET_RIGHT_MACRO_OVERLOAD
// NAME=LOG_X_VAR_3, so GET_RIGHT_MACRO_OVERLOAD returns the macro name which corresponds to
// number of arguments
// MACRONAME is the macro to be called for printing the message
#ifndef LOG_X_VAR
#define LOG_X_VAR(MACRONAME, ...)                                                                                      \
  GET_RIGHT_MACRO_OVERLOAD(__VA_ARGS__,                                                                                \
                           LOG_X_VAR_10,                                                                               \
                           LOG_X_VAR_9,                                                                                \
                           LOG_X_VAR_8,                                                                                \
                           LOG_X_VAR_7,                                                                                \
                           LOG_X_VAR_6,                                                                                \
                           LOG_X_VAR_5,                                                                                \
                           LOG_X_VAR_4,                                                                                \
                           LOG_X_VAR_3,                                                                                \
                           LOG_X_VAR_2,                                                                                \
                           LOG_X_VAR_1)                                                                                \
  (MACRONAME, __VA_ARGS__)
#endif

// convenience macros that hide LOG_VAR_X to the user
#ifndef LOG_INF_VAR
#define LOG_INF_VAR(...) LOG_X_VAR(LOG_INF, __VA_ARGS__);
#endif

#ifndef LOG_DEB_VAR
#define LOG_DEB_VAR(...) LOG_X_VAR(LOG_DEB, __VA_ARGS__);
#endif

#ifndef LOG_WARN_VAR
#define LOG_WARN_VAR(...) LOG_X_VAR(LOG_WARN, __VA_ARGS__);
#endif

#ifndef LOG_ERR_VAR
#define LOG_ERR_VAR(...) LOG_X_VAR(LOG_ERR, __VA_ARGS__);
#endif

#ifndef LOG_VAR
#define LOG_VAR(...) LOG_INF_VAR(__VA_ARGS__);
#endif
#endif  // DISABLE_LOG_VAR

namespace aduulm_logger
{
static inline void setLogLevel(LoggerLevel log_level)
{
#if defined(IS_ROS) || defined(USE_ROS_LOG)
  if (log_level >= LoggerLevels::None)  // should always be true, but just in case
  {
    if (log_level > LoggerLevels::Debug)  // should also not happen, just in case
    {
      log_level = LoggerLevels::Debug;
    }
    auto lvl = level_mapping[log_level];
    if (ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + "." + g_stream_name, lvl))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
#endif
  g_log_level = log_level;
}

#if defined(IS_ROS) || defined(USE_ROS_LOG)
#define DEFAULT_LOG_LEVEL aduulm_logger::LoggerLevel::Info
#else
#define DEFAULT_LOG_LEVEL aduulm_logger::LoggerLevel::Warn
#endif

static inline bool initLogger(LoggerLevel log_level = DEFAULT_LOG_LEVEL)
{
  LOG_INF("Logger initialized");

  setLogLevel(log_level);
  return true;
}

static inline void setStreamName(std::string name)
{
  g_stream_name = name;
}

static inline bool initLogger(std::string file_name, LoggerLevel log_level = DEFAULT_LOG_LEVEL)
{
  //	std::cout << "Test: " << file_name << log_level << std::endl;
  g_file_name = file_name;
  g_oFile.close();
  g_oFile.open(g_file_name);
  g_log_to_file = true;
  return initLogger(log_level);
}

}  // namespace aduulm_logger

#endif  // _ADUULM_LOGGER_H_

/**
 * @}
 */
