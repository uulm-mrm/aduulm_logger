#ifndef _ADUULM_LOGGER_H_
#define _ADUULM_LOGGER_H_

// Check if there are conflicting definitions of our macros
#if defined(LOG_DEB) || defined(LOG_INF) || defined(LOG_WARN) || defined(LOG_ERR) || defined(LOG_FATAL)
#warning "One or more of the logging macros were already defined! This should not happen!"
#endif

#if defined(IS_ROS2)
#include <rclcpp/rclcpp.hpp>
#elif defined(IS_ROS)
#include <ros/ros.h>
#endif

#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <functional>

#define UTC_TIME false

#define LOG_RESET "\033[0m"
#define LOG_NORMAL "\033[m\017"
#define LOG_RED "\033[1m\033[31m"
#define LOG_GREEN "\033[1m\033[32m"
#define LOG_YELLOW "\033[1m\033[33m"
// #define LOG_WHITE "\033[1m"
#define LOG_WHITE "\033[37m"
#define LOG_BLUE "\033[34m"
#define LOG_MAGENTA "\033[35m"
#define LOG_CYAN "\033[36m"
#define LOG_GRAY "\033[0;37m"
#define LOG_BOLDBLUE "\033[1m\033[34m"

#ifdef NDEBUG
#define ADUULM_LOGGER_SHORT_CIRCUIT true
#else
#define ADUULM_LOGGER_SHORT_CIRCUIT false
#endif
#define SHOW_ORIGIN_ENV_VAR_NAME "ADUULM_LOGGER_SHOW_ORIGIN"

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

static const std::unordered_map<std::string, Level> conv_string_to_level = { { "None", Level::None },
                                                                             { "Error", Level::Error },
                                                                             { "Warning", Level::Warn },
                                                                             { "Info", Level::Info },
                                                                             { "Debug", Level::Debug } };
}  // namespace LoggerLevels
using LoggerLevel = LoggerLevels::Level;

extern std::recursive_mutex g_oLoggerMutex;
extern std::ofstream g_oFile;
extern std::string g_file_name;
extern LoggerLevel g_log_level;
extern bool g_log_to_file;
extern int g_nLogCount;
extern int g_nLogNr;
extern std::string g_prefix;
extern bool g_show_origin;
extern std::string g_stream_name;
#if defined(IS_ROS2)
extern std::shared_ptr<rclcpp::Clock> g_rclcpp_clock;
#endif
using LoggerInitCallback = std::function<void(void)>;
using LoggerLevelChangeCallback = std::function<void(LoggerLevel log_level)>;
using LoggerLevelChangeCallbackStr = std::function<void(std::string log_level_string)>;
using LoggerStreamNameChangeCallback = std::function<void(std::string stream_name)>;
using LoggerPrefixChangeCallback = std::function<void(std::string prefix)>;
using LoggerShowOriginChangeCallback = std::function<void(bool show_origin)>;
extern std::vector<LoggerInitCallback> g_sublogger_init_callbacks;
extern std::vector<LoggerLevelChangeCallback> g_sublogger_level_change_callbacks;
extern std::vector<LoggerLevelChangeCallbackStr> g_sublogger_level_change_callbacks_str;
extern std::vector<LoggerStreamNameChangeCallback> g_sublogger_stream_name_change_callbacks;
extern std::vector<LoggerPrefixChangeCallback> g_sublogger_prefix_change_callbacks;
extern std::vector<LoggerShowOriginChangeCallback> g_sublogger_show_origin_change_callbacks;

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

#if defined(IS_ROS2)
static const std::array<rclcpp::Logger::Level, 5> level_mapping = { rclcpp::Logger::Level::Fatal,
                                                                    rclcpp::Logger::Level::Error,
                                                                    rclcpp::Logger::Level::Warn,
                                                                    rclcpp::Logger::Level::Info,
                                                                    rclcpp::Logger::Level::Debug };
#define LOGGER_ROS_EXTRA_DEFINES                                                                                       \
  std::string __attribute__((visibility("hidden"))) g_stream_name = "ROS2_Default_logger_name";                        \
  std::shared_ptr<rclcpp::Clock> __attribute__((visibility("hidden"))) g_rclcpp_clock =                                \
      std::make_shared<rclcpp::Clock>();
#elif defined(IS_ROS)
static const std::array<ros::console::Level, 5> level_mapping = { ros::console::levels::Fatal,
                                                                  ros::console::levels::Error,
                                                                  ros::console::levels::Warn,
                                                                  ros::console::levels::Info,
                                                                  ros::console::levels::Debug };
#define LOGGER_ROS_EXTRA_DEFINES                                                                                       \
  std::string __attribute__((visibility("hidden"))) g_stream_name = ROSCONSOLE_DEFAULT_NAME;
#else  // no IS_ROS or IS_ROS2
#define LOGGER_ROS_EXTRA_DEFINES std::string __attribute__((visibility("hidden"))) g_stream_name = "default";
#endif

#define DEFINE_LOGGER_VARIABLES                                                                                        \
  namespace aduulm_logger                                                                                              \
  {                                                                                                                    \
  std::recursive_mutex __attribute__((visibility("hidden"))) g_oLoggerMutex;                                           \
  std::ofstream __attribute__((visibility("hidden"))) g_oFile;                                                         \
  std::string __attribute__((visibility("hidden"))) g_file_name;                                                       \
  LoggerLevel __attribute__((visibility("hidden"))) g_log_level = DEFAULT_LOG_LEVEL;                                   \
  bool __attribute__((visibility("hidden"))) g_log_to_file = false;                                                    \
  int __attribute__((visibility("hidden"))) g_nLogCount = 0;                                                           \
  int __attribute__((visibility("hidden"))) g_nLogNr = 0;                                                              \
  std::string __attribute__((visibility("hidden"))) g_prefix;                                                          \
  bool __attribute__((visibility("hidden"))) g_show_origin = true;                                                     \
  std::vector<LoggerInitCallback> __attribute__((visibility("hidden"))) g_sublogger_init_callbacks;                    \
  std::vector<LoggerLevelChangeCallback> __attribute__((visibility("hidden"))) g_sublogger_level_change_callbacks;     \
  std::vector<LoggerLevelChangeCallbackStr> __attribute__((visibility("hidden")))                                      \
  g_sublogger_level_change_callbacks_str;                                                                              \
  std::vector<LoggerStreamNameChangeCallback> __attribute__((visibility("hidden")))                                    \
  g_sublogger_stream_name_change_callbacks;                                                                            \
  std::vector<LoggerPrefixChangeCallback> __attribute__((visibility("hidden"))) g_sublogger_prefix_change_callbacks;   \
  std::vector<LoggerShowOriginChangeCallback> __attribute__((visibility("hidden")))                                    \
  g_sublogger_show_origin_change_callbacks;                                                                            \
  LOGGER_ROS_EXTRA_DEFINES                                                                                             \
  }

#define LOGGER_ADD_SUBLOGGER_LIBRARY(_namespace)                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    aduulm_logger::g_sublogger_init_callbacks.emplace_back(_namespace::_initLogger);                                   \
    aduulm_logger::g_sublogger_level_change_callbacks.emplace_back(                                                    \
        static_cast<void (*)(aduulm_logger::LoggerLevel)>(_namespace::_setLogLevel));                                  \
    aduulm_logger::g_sublogger_level_change_callbacks_str.emplace_back(                                                \
        static_cast<void (*)(std::string)>(_namespace::_setLogLevel));                                                 \
    aduulm_logger::g_sublogger_stream_name_change_callbacks.emplace_back(_namespace::_setStreamName);                  \
    aduulm_logger::g_sublogger_prefix_change_callbacks.emplace_back(_namespace::_setPrefix);                           \
    aduulm_logger::g_sublogger_show_origin_change_callbacks.emplace_back(_namespace::_setShowOrigin);                  \
  } while (0)

#define LOGGER_ADD_SUBLOGGER_CLASS(_class, _instance)                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    auto* inst = &_instance;                                                                                           \
    aduulm_logger::g_sublogger_init_callbacks.emplace_back([inst]() { inst->_initLogger(); });                         \
    aduulm_logger::g_sublogger_level_change_callbacks.emplace_back(                                                    \
        [inst](aduulm_logger::LoggerLevel level) { inst->_setLogLevel(level); });                                      \
    aduulm_logger::g_sublogger_level_change_callbacks_str.emplace_back(                                                \
        [inst](std::string level) { inst->_setLogLevel(level); });                                                     \
    aduulm_logger::g_sublogger_stream_name_change_callbacks.emplace_back(                                              \
        [inst](std::string name) { inst->_setStreamName(name); });                                                     \
    aduulm_logger::g_sublogger_prefix_change_callbacks.emplace_back(                                                   \
        [inst](std::string name) { inst->_setPrefix(name); });                                                         \
    aduulm_logger::g_sublogger_show_origin_change_callbacks.emplace_back(                                              \
        [inst](bool show_origin) { inst->_setShowOrigin(show_origin); });                                              \
  } while (0)

#define LOGGER_ADD_SUBLOGGER_PARENT_CLASS(_class, _this)                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    auto* inst = static_cast<_class*>(_this);                                                                          \
    aduulm_logger::g_sublogger_init_callbacks.emplace_back([inst]() { inst->_initLogger(); });                         \
    aduulm_logger::g_sublogger_level_change_callbacks.emplace_back(                                                    \
        [inst](aduulm_logger::LoggerLevel level) { inst->_setLogLevel(level); });                                      \
    aduulm_logger::g_sublogger_level_change_callbacks_str.emplace_back(                                                \
        [inst](std::string level) { inst->_setLogLevel(level); });                                                     \
    aduulm_logger::g_sublogger_stream_name_change_callbacks.emplace_back(                                              \
        [inst](std::string name) { inst->_setStreamName(name); });                                                     \
    aduulm_logger::g_sublogger_prefix_change_callbacks.emplace_back(                                                   \
        [inst](std::string name) { inst->_setPrefix(name); });                                                         \
    aduulm_logger::g_sublogger_show_origin_change_callbacks.emplace_back(                                              \
        [inst](bool show_origin) { inst->_setShowOrigin(show_origin); });                                              \
  } while (0)

#define DEFINE_LOGGER_LIBRARY_INTERFACE_HEADER                                                                         \
  void _initLogger();                                                                                                  \
  void _setStreamName(std::string stream_name);                                                                        \
  void _setLogLevel(aduulm_logger::LoggerLevel log_level);                                                             \
  void _setLogLevel(std::string log_level_string);                                                                     \
  void _setPrefix(std::string prefix);                                                                                 \
  void _setShowOrigin(bool show_origin);

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
  }                                                                                                                    \
                                                                                                                       \
  void _setLogLevel(std::string log_level_string)                                                                      \
  {                                                                                                                    \
    aduulm_logger::setLogLevel(log_level_string);                                                                      \
    for (auto callback : aduulm_logger::g_sublogger_level_change_callbacks_str)                                        \
    {                                                                                                                  \
      callback(log_level_string);                                                                                      \
    }                                                                                                                  \
  }                                                                                                                    \
                                                                                                                       \
  void _setPrefix(std::string prefix)                                                                                  \
  {                                                                                                                    \
    aduulm_logger::setPrefix(prefix);                                                                                  \
    for (auto callback : aduulm_logger::g_sublogger_prefix_change_callbacks)                                           \
    {                                                                                                                  \
      callback(prefix);                                                                                                \
    }                                                                                                                  \
  }                                                                                                                    \
                                                                                                                       \
  void _setShowOrigin(bool show_origin)                                                                                \
  {                                                                                                                    \
    aduulm_logger::setShowOrigin(show_origin);                                                                         \
    for (auto callback : aduulm_logger::g_sublogger_show_origin_change_callbacks)                                      \
    {                                                                                                                  \
      callback(show_origin);                                                                                           \
    }                                                                                                                  \
  }

#define DEFINE_LOGGER_CLASS_INTERFACE_HEADER                                                                           \
  void _initLogger();                                                                                                  \
  void _setStreamName(std::string stream_name);                                                                        \
  void _setLogLevel(aduulm_logger::LoggerLevel log_level);                                                             \
  void _setLogLevel(std::string log_level_string);                                                                     \
  void _setPrefix(std::string prefix);                                                                                 \
  void _setShowOrigin(bool show_origin);

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
  }                                                                                                                    \
                                                                                                                       \
  void _class ::_setLogLevel(std::string log_level_string)                                                             \
  {                                                                                                                    \
    aduulm_logger::setLogLevel(log_level_string);                                                                      \
    for (auto callback : aduulm_logger::g_sublogger_level_change_callbacks_str)                                        \
    {                                                                                                                  \
      callback(log_level_string);                                                                                      \
    }                                                                                                                  \
  }                                                                                                                    \
                                                                                                                       \
  void _class ::_setPrefix(std::string prefix)                                                                         \
  {                                                                                                                    \
    aduulm_logger::setPrefix(prefix);                                                                                  \
    for (auto callback : aduulm_logger::g_sublogger_prefix_change_callbacks)                                           \
    {                                                                                                                  \
      callback(prefix);                                                                                                \
    }                                                                                                                  \
  }                                                                                                                    \
                                                                                                                       \
  void _class ::_setShowOrigin(bool show_origin)                                                                       \
  {                                                                                                                    \
    aduulm_logger::setShowOrigin(show_origin);                                                                         \
    for (auto callback : aduulm_logger::g_sublogger_show_origin_change_callbacks)                                      \
    {                                                                                                                  \
      callback(show_origin);                                                                                           \
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

  uint64_t _ntm_us =
      _nUSSinceDay - ((_tmBrokenTime.tm_hour * 3600 + _tmBrokenTime.tm_min * 60 + _tmBrokenTime.tm_sec) * 1000000ULL);
  char buf[100];
  sprintf(buf,
          "%04d-%02d-%02d %02d:%02d:%02d.%06d",
          _tmBrokenTime.tm_year + 1900,
          _tmBrokenTime.tm_mon + 1,
          _tmBrokenTime.tm_mday,
          _tmBrokenTime.tm_hour,
          _tmBrokenTime.tm_min,
          _tmBrokenTime.tm_sec,
          static_cast<int32_t>(_ntm_us));

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

#define _LOG_PREFIX_EXPR(expr) aduulm_logger::g_prefix << (aduulm_logger::g_prefix.empty() ? "" : ": ") << expr
#define _LOG_PREFIX_EXPR_WITH_ORIGIN(expr) _LOG_BASE << ": " << _LOG_PREFIX_EXPR(expr)

#if defined(IS_ROS2)
#define LOG_FATAL(expr)                                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_FATAL_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));       \
    else                                                                                                               \
      RCLCPP_FATAL_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR(expr));                   \
  } while (0)

#define LOG_ERR(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));       \
    else                                                                                                               \
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR(expr));                   \
  } while (0)

#define LOG_WARN(expr)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_WARN_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));        \
    else                                                                                                               \
      RCLCPP_WARN_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR(expr));                    \
  } while (0)

#define LOG_INF(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_INFO_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));        \
    else                                                                                                               \
      RCLCPP_INFO_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR(expr));                    \
  } while (0)

#define LOG_DEB(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));       \
    else                                                                                                               \
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(aduulm_logger::g_stream_name), _LOG_PREFIX_EXPR(expr));                   \
  } while (0)

#define LOG_FATAL_THROTTLE(period, expr)                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_FATAL_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                   \
                                   *aduulm_logger::g_rclcpp_clock,                                                     \
                                   (period) * 1000,                                                                    \
                                   _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                                                \
    else                                                                                                               \
      RCLCPP_FATAL_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                   \
                                   *aduulm_logger::g_rclcpp_clock,                                                     \
                                   (period) * 1000,                                                                    \
                                   _LOG_PREFIX_EXPR(expr));                                                            \
  } while (0)

#define LOG_ERR_THROTTLE(period, expr)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_ERROR_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                   \
                                   *aduulm_logger::g_rclcpp_clock,                                                     \
                                   (period) * 1000,                                                                    \
                                   _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                                                \
    else                                                                                                               \
      RCLCPP_ERROR_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                   \
                                   *aduulm_logger::g_rclcpp_clock,                                                     \
                                   (period) * 1000,                                                                    \
                                   _LOG_PREFIX_EXPR(expr));                                                            \
  } while (0)

#define LOG_WARN_THROTTLE(period, expr)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                    \
                                  *aduulm_logger::g_rclcpp_clock,                                                      \
                                  (period) * 1000,                                                                     \
                                  _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                                                 \
    else                                                                                                               \
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                    \
                                  *aduulm_logger::g_rclcpp_clock,                                                      \
                                  (period) * 1000,                                                                     \
                                  _LOG_PREFIX_EXPR(expr));                                                             \
  } while (0)

#define LOG_INF_THROTTLE(period, expr)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                    \
                                  *aduulm_logger::g_rclcpp_clock,                                                      \
                                  (period) * 1000,                                                                     \
                                  _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                                                 \
    else                                                                                                               \
      RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                    \
                                  *aduulm_logger::g_rclcpp_clock,                                                      \
                                  (period) * 1000,                                                                     \
                                  _LOG_PREFIX_EXPR(expr));                                                             \
  } while (0)

#define LOG_DEB_THROTTLE(period, expr)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                   \
                                   *aduulm_logger::g_rclcpp_clock,                                                     \
                                   (period) * 1000,                                                                    \
                                   _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                                                \
    else                                                                                                               \
      RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger(aduulm_logger::g_stream_name),                                   \
                                   *aduulm_logger::g_rclcpp_clock,                                                     \
                                   (period) * 1000,                                                                    \
                                   _LOG_PREFIX_EXPR(expr));                                                            \
  } while (0)

#elif defined(IS_ROS)

#define LOG_FATAL(expr)                                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_FATAL_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                        \
    else                                                                                                               \
      ROS_FATAL_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                                    \
  } while (0)

#define LOG_ERR(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_ERROR_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                        \
    else                                                                                                               \
      ROS_ERROR_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                                    \
  } while (0)

#define LOG_WARN(expr)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_WARN_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                         \
    else                                                                                                               \
      ROS_WARN_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                                     \
  } while (0)

#define LOG_INF(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_INFO_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                         \
    else                                                                                                               \
      ROS_INFO_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                                     \
  } while (0)

#define LOG_DEB(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_DEBUG_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));                        \
    else                                                                                                               \
      ROS_DEBUG_STREAM_NAMED(aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                                    \
  } while (0)

#define LOG_FATAL_THROTTLE(period, expr)                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_FATAL_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));       \
    else                                                                                                               \
      ROS_FATAL_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                   \
  } while (0)

#define LOG_ERR_THROTTLE(period, expr)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_ERROR_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));       \
    else                                                                                                               \
      ROS_ERROR_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                   \
  } while (0)

#define LOG_WARN_THROTTLE(period, expr)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_WARN_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));        \
    else                                                                                                               \
      ROS_WARN_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                    \
  } while (0)

#define LOG_INF_THROTTLE(period, expr)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_INFO_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));        \
    else                                                                                                               \
      ROS_INFO_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                    \
  } while (0)

#define LOG_DEB_THROTTLE(period, expr)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
      ROS_DEBUG_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR_WITH_ORIGIN(expr));       \
    else                                                                                                               \
      ROS_DEBUG_STREAM_THROTTLE_NAMED(period, aduulm_logger::g_stream_name, _LOG_PREFIX_EXPR(expr));                   \
  } while (0)

#else  // no IS_ROS or IS_ROS2

#define LOG_FATAL(expr)                                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                                 \
    aduulm_logger::CheckLogCnt();                                                                                      \
    std::stringstream __msg_logger;                                                                                    \
    if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                   \
    {                                                                                                                  \
      __msg_logger << LOG_MAGENTA << "[FATAL] [" << DataTypesLogger::longTime() << "] "                                \
                   << _LOG_PREFIX_EXPR_WITH_ORIGIN(expr) << LOG_NORMAL;                                                \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
      __msg_logger << LOG_MAGENTA << "[FATAL] [" << DataTypesLogger::longTime() << "] " << _LOG_PREFIX_EXPR(expr)      \
                   << LOG_NORMAL;                                                                                      \
    }                                                                                                                  \
    if (aduulm_logger::g_log_to_file)                                                                                  \
    {                                                                                                                  \
      aduulm_logger::g_oFile << "[FATAL] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr        \
                             << std::endl;                                                                             \
    }                                                                                                                  \
    std::cout << __msg_logger.str() << std::endl;                                                                      \
  } while (0)

#define LOG_ERR(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_log_level >= aduulm_logger::LoggerLevel::Error)                                               \
    {                                                                                                                  \
      std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                               \
      aduulm_logger::CheckLogCnt();                                                                                    \
      std::stringstream __msg_logger;                                                                                  \
      if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                 \
      {                                                                                                                \
        __msg_logger << LOG_RED << "[ERROR] [" << DataTypesLogger::longTime() << "] "                                  \
                     << _LOG_PREFIX_EXPR_WITH_ORIGIN(expr) << LOG_NORMAL;                                              \
      }                                                                                                                \
      else                                                                                                             \
      {                                                                                                                \
        __msg_logger << LOG_RED << "[ERROR] [" << DataTypesLogger::longTime() << "] " << _LOG_PREFIX_EXPR(expr)        \
                     << LOG_NORMAL;                                                                                    \
      }                                                                                                                \
      if (aduulm_logger::g_log_to_file)                                                                                \
      {                                                                                                                \
        aduulm_logger::g_oFile << "[ERROR] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr      \
                               << std::endl;                                                                           \
      }                                                                                                                \
      std::cout << __msg_logger.str() << std::endl;                                                                    \
    }                                                                                                                  \
  } while (0)

#define LOG_WARN(expr)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_log_level >= aduulm_logger::LoggerLevel::Warn)                                                \
    {                                                                                                                  \
      std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                               \
      aduulm_logger::CheckLogCnt();                                                                                    \
      std::stringstream __msg_logger;                                                                                  \
      if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                 \
      {                                                                                                                \
        __msg_logger << LOG_YELLOW << "[WARN ] [" << DataTypesLogger::longTime() << "] "                               \
                     << _LOG_PREFIX_EXPR_WITH_ORIGIN(expr) << LOG_NORMAL;                                              \
      }                                                                                                                \
      else                                                                                                             \
      {                                                                                                                \
        __msg_logger << LOG_YELLOW << "[WARN ] [" << DataTypesLogger::longTime() << "] " << _LOG_PREFIX_EXPR(expr)     \
                     << LOG_NORMAL;                                                                                    \
      }                                                                                                                \
      if (aduulm_logger::g_log_to_file)                                                                                \
      {                                                                                                                \
        aduulm_logger::g_oFile << "[WARN ] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr      \
                               << std::endl;                                                                           \
      }                                                                                                                \
      std::cout << __msg_logger.str() << std::endl;                                                                    \
    }                                                                                                                  \
  } while (0)

#define LOG_INF(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_log_level >= aduulm_logger::LoggerLevel::Info)                                                \
    {                                                                                                                  \
      std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                               \
      aduulm_logger::CheckLogCnt();                                                                                    \
      std::stringstream __msg_logger;                                                                                  \
      if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                 \
      {                                                                                                                \
        __msg_logger << LOG_GREEN << "[INFO ] [" << DataTypesLogger::longTime() << "] "                                \
                     << _LOG_PREFIX_EXPR_WITH_ORIGIN(expr) << LOG_NORMAL;                                              \
      }                                                                                                                \
      else                                                                                                             \
      {                                                                                                                \
        __msg_logger << LOG_GREEN << "[INFO ] [" << DataTypesLogger::longTime() << "] " << _LOG_PREFIX_EXPR(expr)      \
                     << LOG_NORMAL;                                                                                    \
      }                                                                                                                \
      if (aduulm_logger::g_log_to_file)                                                                                \
      {                                                                                                                \
        aduulm_logger::g_oFile << "[INFO ] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr      \
                               << std::endl;                                                                           \
      }                                                                                                                \
      std::cout << __msg_logger.str() << std::endl;                                                                    \
    }                                                                                                                  \
  } while (0)

#define LOG_DEB(expr)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (aduulm_logger::g_log_level >= aduulm_logger::LoggerLevel::Debug)                                               \
    {                                                                                                                  \
      std::lock_guard<std::recursive_mutex> _oLockLogger(aduulm_logger::g_oLoggerMutex);                               \
      aduulm_logger::CheckLogCnt();                                                                                    \
      std::stringstream __msg_logger;                                                                                  \
      if (aduulm_logger::g_show_origin or ADUULM_LOGGER_SHORT_CIRCUIT)                                                 \
      {                                                                                                                \
        __msg_logger << LOG_BLUE << "[DEBUG] [" << DataTypesLogger::longTime() << "] "                                 \
                     << _LOG_PREFIX_EXPR_WITH_ORIGIN(expr) << LOG_NORMAL;                                              \
      }                                                                                                                \
      else                                                                                                             \
      {                                                                                                                \
        __msg_logger << LOG_BLUE << "[DEBUG] [" << DataTypesLogger::longTime() << "] " << _LOG_PREFIX_EXPR(expr)       \
                     << LOG_NORMAL;                                                                                    \
      }                                                                                                                \
      if (aduulm_logger::g_log_to_file)                                                                                \
      {                                                                                                                \
        aduulm_logger::g_oFile << "[DEBUG] [" << DataTypesLogger::longTime() << "] " << _LOG_BASE << ": " << expr      \
                               << std::endl;                                                                           \
      }                                                                                                                \
      std::cout << __msg_logger.str() << std::endl;                                                                    \
    }                                                                                                                  \
  } while (0)

// Throttling is not currently supported for non-ROS environments.
#define LOG_FATAL_THROTTLE(period, expr) LOG_FATAL(expr)
#define LOG_ERR_THROTTLE(period, expr) LOG_ERR(expr)
#define LOG_WARN_THROTTLE(period, expr) LOG_WARN(expr)
#define LOG_INF_THROTTLE(period, expr) LOG_INF(expr)
#define LOG_DEB_THROTTLE(period, expr) LOG_DEB(expr)

#endif  // !IS_ROS

// deprecated
#define LOG_EVAL(expr)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    std::cout << expr << std::endl;                                                                                    \
  } while (0)

namespace aduulm_logger
{
static inline void setLogLevel(LoggerLevel log_level)
{
#if defined(IS_ROS2)
  if (log_level >= LoggerLevels::None)  // should always be true, but just in case
  {
    if (log_level > LoggerLevels::Debug)  // should also not happen, just in case
    {
      log_level = LoggerLevels::Debug;
    }
    auto lvl = level_mapping[log_level];
    rclcpp::get_logger(aduulm_logger::g_stream_name).set_level(lvl);
  }
#elif defined(IS_ROS)
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

static inline void setLogLevel(std::string log_level_string)
{
  auto log_level = LoggerLevels::conv_string_to_level.at(log_level_string);
  setLogLevel(log_level);
}

static inline LoggerLevel getLogLevel()
{
  return g_log_level;
}

#if defined(IS_ROS2)
#define DEFAULT_LOG_LEVEL aduulm_logger::LoggerLevel::Warn
#elif defined(IS_ROS)
#define DEFAULT_LOG_LEVEL aduulm_logger::LoggerLevel::Warn
#else
#define DEFAULT_LOG_LEVEL aduulm_logger::LoggerLevel::Warn
#endif

static inline bool initLogger(LoggerLevel log_level, bool is_test = false)
{
#if defined(IS_ROS)
  if (is_test)
  {
    // This is required in unit tests that do not use ROS to be able to use throttled logging.
    ros::Time::init();
  }
#endif

  // if env variable is not set, the origin is shown by default.
  if (const char* const env_val = getenv(SHOW_ORIGIN_ENV_VAR_NAME))
  {
    std::string show_orig_env{ env_val };
    g_show_origin = (show_orig_env != "0");
  }

  setLogLevel(log_level);
  LOG_DEB("Logger initialized");

  return true;
}

static inline bool initLogger(bool is_test = false)
{
  return initLogger(getLogLevel(), is_test);
}

static inline void setStreamName(std::string name)
{
  g_stream_name = name;
}

static inline void setPrefix(std::string prefix)
{
  g_prefix = prefix;
}

static inline void setShowOrigin(bool show_origin)
{
  g_show_origin = show_origin;
}

static inline bool initLogger(std::string file_name, LoggerLevel log_level, bool is_test = false)
{
  //	std::cout << "Test: " << file_name << log_level << std::endl;
  g_file_name = file_name;
  g_oFile.close();
  g_oFile.open(g_file_name);
  g_log_to_file = true;
  return initLogger(log_level, is_test);
}

static inline bool initLogger(std::string file_name, bool is_test = false)
{
  return initLogger(file_name, getLogLevel(), is_test);
}

}  // namespace aduulm_logger

#endif  // _ADUULM_LOGGER_H_
