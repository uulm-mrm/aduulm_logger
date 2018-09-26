/**
 * \file aduulmlogger.h
 *
 * @brief Remaps ADTF log commands to std::cout (as before) or ROS.
 * Remapping to ROS if built with -DHAVE_LOG
 */
#ifndef _LOGGER_H_
#define _LOGGER_H_

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

static std::mutex g_oLoggerMutex;
extern std::ofstream g_oFile;  // = std::ofstream();
extern std::string g_file_name;
extern uint16_t g_log_level;  // = 2;
extern std::string aduulm_logger_VERSION_ext;
static int g_nLogCount = 0;
static int g_nLogNr = 0;
__inline void CheckLogCnt()
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
extern inline bool initLogger(std::string file_name, uint16_t log_level);
extern inline bool initLogger();

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
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d.%06ld", _tmBrokenTime.tm_year + 1900, _tmBrokenTime.tm_mon + 1,
          _tmBrokenTime.tm_mday, _tmBrokenTime.tm_hour, _tmBrokenTime.tm_min, _tmBrokenTime.tm_sec, _ntm_us);

  return buf;
}
__inline__ std::thread::id thread_id()
{
  return std::this_thread::get_id();
}
}  // namespace DataTypesLogger
#define _LOG_BASE                                                                                                      \
  "(" << std::setw(24) << DataTypesLogger::simpleFileName(__FILE__) << ":" << std::setw(3) << __LINE__ << ") "         \
      << std::setw(30) << __FUNCTION__ << "() "

// Log Level:
// 1 = ERR
// 2 = WARN
// 3 = INF
// 4 = DEBUG
//
// Set the default log level

#if defined(IS_ROS) || defined(USE_ROS_LOG)
#ifndef LOG_ERR
#define LOG_ERR(expr) ROS_ERROR_STREAM(_LOG_BASE << ": " << expr);
#endif

#ifndef LOG_WARN
#define LOG_WARN(expr) ROS_WARN_STREAM(_LOG_BASE << ": " << expr);
#endif

#ifndef LOG_INF
#define LOG_INF(expr) ROS_INFO_STREAM(_LOG_BASE << ": " << expr);
#endif

#ifndef LOG_DEB
#define LOG_DEB(expr) ROS_DEBUG_STREAM(_LOG_BASE << ": " << expr);
#endif
#else

#ifndef LOG_ERR
#define LOG_ERR(expr)                                                                                                  \
  {                                                                                                                    \
    if (g_log_level >= 1)                                                                                              \
    {                                                                                                                  \
      std::lock_guard<std::mutex> _oLockLogger(g_oLoggerMutex);                                                        \
      CheckLogCnt();                                                                                                   \
      std::cout << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id()           \
                << std::dec << ")" << LOG_RED << std::setw(9) << " ERROR " << LOG_NORMAL << _LOG_BASE << ": "          \
                << LOG_RED << expr << LOG_NORMAL << std::endl;                                                         \
      g_oFile << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec \
              << ")" << std::setw(9) << " ERROR " << _LOG_BASE << ": " << expr << std::endl;                           \
    }                                                                                                                  \
  }
#endif

#ifndef LOG_WARN
#define LOG_WARN(expr)                                                                                                 \
  {                                                                                                                    \
    if (g_log_level >= 2)                                                                                              \
    {                                                                                                                  \
      std::lock_guard<std::mutex> _oLockLogger(g_oLoggerMutex);                                                        \
      CheckLogCnt();                                                                                                   \
      std::cout << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id()           \
                << std::dec << ")" << LOG_YELLOW << std::setw(9) << " WARNING " << LOG_NORMAL << _LOG_BASE << ": "     \
                << LOG_YELLOW << expr << LOG_NORMAL << std::endl;                                                      \
      g_oFile << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec \
              << ")" << std::setw(9) << " WARNING " << _LOG_BASE << ": " << expr << std::endl;                         \
    }                                                                                                                  \
  }
#else
#define LOG_WARN(expr) ;
#endif

#ifndef LOG_INF
#define LOG_INF(expr)                                                                                                  \
  {                                                                                                                    \
    if (g_log_level >= 3)                                                                                              \
    {                                                                                                                  \
      std::lock_guard<std::mutex> _oLockLogger(g_oLoggerMutex);                                                        \
      CheckLogCnt();                                                                                                   \
      std::cout << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id()           \
                << std::dec << ")" << LOG_GREEN << std::setw(9) << " INFO " << LOG_NORMAL << _LOG_BASE << ": "         \
                << LOG_GREEN << expr << LOG_NORMAL << std::endl;                                                       \
      g_oFile << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec \
              << ")" << std::setw(9) << " INFO " << _LOG_BASE << ": " << expr << std::endl;                            \
    }                                                                                                                  \
  }
#else
#define LOG_INF(expr) ;
#endif

#ifndef LOG_DEBUG
#define LOG_DEB(expr)                                                                                                  \
  {                                                                                                                    \
    if (g_log_level >= 4)                                                                                              \
    {                                                                                                                  \
      std::lock_guard<std::mutex> _oLockLogger(g_oLoggerMutex);                                                        \
      CheckLogCnt();                                                                                                   \
      std::cout << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id()           \
                << std::dec << ")" << LOG_CYAN << std::setw(9) << " DEBUG " << LOG_NORMAL << _LOG_BASE << ": "         \
                << LOG_CYAN << expr << LOG_NORMAL << std::endl;                                                        \
      g_oFile << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec \
              << ")" << std::setw(9) << " DEBUG " << _LOG_BASE << ": " << expr << std::endl;                           \
    }                                                                                                                  \
  }
#else
#define LOG_DEB(expr) ;
#endif
#endif

#ifdef LOG_TRACEING
#ifndef LOG_TRACE
#define LOG_TRACE(expr)                                                                                                \
  {                                                                                                                    \
    std::lock_guard<std::mutex> _oLockLogger(g_oLoggerMutex);                                                          \
    CheckLogCnt();                                                                                                     \
    std::cout << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec \
              << ")" << LOG_BLUE << std::setw(9) << " TRACE " << LOG_NORMAL << _LOG_BASE << ": " << LOG_BLUE << expr   \
              << LOG_NORMAL << std::endl;                                                                              \
    g_oFile << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec   \
            << ")" << std::setw(9) << " TRACE " << _LOG_BASE << ": " << expr << std::endl;                             \
  }
#endif
#else
#ifndef LOG_TRACE
#define LOG_TRACE(expr) ;
#endif
#endif

#ifdef LOG_VERBOSE
#ifndef LOG_VERB
#define LOG_VERB(expr)                                                                                                 \
  {                                                                                                                    \
    std::lock_guard<std::mutex> _oLockLogger(g_oLoggerMutex);                                                          \
    CheckLogCnt();                                                                                                     \
    std::cout << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec \
              << ")" << std::setw(9) << " VERBOSE " << _LOG_BASE << ": " << expr << std::endl;                         \
    g_oFile << "[" << DataTypesLogger::longTime() << "] (0x" << std::hex << DataTypesLogger::thread_id() << std::dec   \
            << ")" << std::setw(9) << " VERBOSE " << _LOG_BASE << ": " << expr << std::endl;                           \
  }
#endif
#else
#ifndef LOG_VERB
#define LOG_VERB(expr) ;
#endif
#endif

extern inline bool initLogger(std::string file_name, uint16_t log_level)
{
  //	std::cout << "Test: " << file_name << log_level << std::endl;
  g_file_name = file_name;
  g_oFile.close();
  g_oFile.open(g_file_name);
  g_log_level = log_level;
  LOG_INF("Using aduulm_logger version " << aduulm_logger_VERSION_ext);
  //	LOG_INF("Logger initialized");
  return true;
}
extern inline bool initLogger()
{
  //  LOG_INF("Logger initialized");
  LOG_INF("Using aduulm_logger version " << aduulm_logger_VERSION_ext);
  return true;
}

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
inline std::string stringify(bool isLiteral, std::string name,
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
  GET_RIGHT_MACRO_OVERLOAD(__VA_ARGS__, LOG_X_VAR_10, LOG_X_VAR_9, LOG_X_VAR_8, LOG_X_VAR_7, LOG_X_VAR_6, LOG_X_VAR_5, \
                           LOG_X_VAR_4, LOG_X_VAR_3, LOG_X_VAR_2, LOG_X_VAR_1)                                         \
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

#endif  // _LOGGER_H_

/**
 * @}
 */
