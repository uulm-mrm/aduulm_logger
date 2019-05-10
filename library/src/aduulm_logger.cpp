#include "aduulm_logger/aduulm_logger.hpp"

namespace aduulm_logger
{
std::ofstream g_oFile;
std::string g_file_name;
LoggerLevel g_log_level = LoggerLevels::Warn;
std::string aduulm_logger_VERSION_ext = aduulm_logger_lib_VERSION;
bool g_log_to_file = false;

}  // namespace aduulm_logger
