# aduulm_logger

# PLEASE MAKE SURE YOU READ THIS WHOLE README BEFORE USAGE OF THE LOGGER!

## Overview

The aduulm logger package is thought to be used in projects with libraries for ROS and ADTF. It redirects logging messages either to a cout or ROS_LOG command and therefore simplifies the portation of packages from ADTF to ROS which use the ADTF logging commands. It also contains a node for simple log message printing from command line or launch file.

**Keywords:** mrm, logger, ros, aduulm, ADTF, ROS, logging


### License

The source code is not official released and is only for internal use.

**Author(s): Martin Herrmann   
Maintainer 1:  Martin Herrmann,  Martin.Herrmann@uni-ulm.de  
Maintainer 2:  Jan Strohbeck, Jan.Strohbeck@uni-ulm.de  
Affiliation: Institute of Measurements, Control and Microtechnology, Ulm University**

The aduulm_logger package has been tested under [ROS] Kinetic and Ubuntu 16.04. 
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Dependencies

* [Robot Operating System (ROS)](http://wiki.ros.org) (optional)
* Boost

## Usage

### Logger
* The aduulm_logger uses the same logging commands as the mrm ADTF logger (LOG_ERR, LOG_WARN, LOG_INF, ...) and has two modes. It can print the messages like the mrm ADTF logger via std::cout or redirect the messages to the ROS logger (ROS_ERROR, ROS_WARN, ROS_INFO, ...). This should be set by global defines, e.g. in the catkin profile:
```yaml
cmake_args:
- -DCMAKE_CXX_COMPILER_ARG1=-DIS_ROS
```
or
```cmake
add_definitions(-DUSE_ROS_LOG)
```

* Add the package to your project (you should already have included the aduulm_cmake_tools):
```cmake
find_aduulm_package(aduulm_logger_lib)
```
* Link the logger to your target:
```cmake
link_aduulm_package_targets(TARGET MY_TARGET_NAME
	ACCESS PUBLIC
	PACKAGE_TARGETS
		aduulm_logger_lib::aduulm_logger_lib
)
```

* To be able to use the logger, it needs to be initialized before the first usage: If you used the library/class setup macros, you can cann `_initLogger();`. Otherwise, you can call `aduulm_logger::initLogger()`. If the logger is used in non ROS mode (no flag set), it can be initialized with the optional parameters file_name and log_level `initLogger(std::string file_name, LoggerLevel log_level)`.

* Example usage can be seen in the ros_example in the ros_template package (root/ros_template).

* In contrast to the mrm logger a locking function is added to enable multithreaded usage. However this prevents from concurrent logging commands, but not from concurrent std::cout commands.

## Internal workings (IMPORTANT, READ THIS BEFORE USAGE!)

In [https://mrm-git.e-technik.uni-ulm.de/aduulm/source/ros/aduulm_logger/merge_requests/7](aduulm/source/ros/aduulm_logger!7), major changes to how the logger works were introduced. Before, it had a shared library in which the variables holding the logger level and other stuff were stored. One consequence of this was, that all libraries which were linked into a single executable or library shared the same logger level. Also, in ROS nodelets, all nodelets loaded in one nodelet manager shared the same logger levels. The MR improved this by storing the logger variables once per shared library or executable. This was achieved by setting a visibility attribute on the variables with `__attribute__((visibility("hidden")))`, which does not allow them to be linked outside of the shared library or executable that they are part of. They can be linked outside of static libraries, so one has to be careful when using static libraries, so that the logger variables are not multiply defined.

This MR introduced several code generation macros:

1. `DEFINE_LOGGER_VARIABLES`: This macro defines the variables that the logger uses, e.g. the logger level. This macro should be used exactly once per shared library or executable, in the global namespace.

2. `DEFINE_LOGGER_LIBRARY_INTERFACE_HEADER` and `DEFINE_LOGGER_LIBRARY_INTERFACE_IMPLEMENTATION`: These two macros are to be used in shared libraries which want to use the logger. They declare and implement functions for initializing the logger and setting the logger level for that shared library. Example usage, extracted from the ros_package_creation script:

```diff
// LoggerSetup.h
#if !defined LIBRARY_NAME_LOGGER_SETUP_H
#define LIBRARY_NAME_LOGGER_SETUP_H

#include <aduulm_logger/aduulm_logger.hpp>

namespace LIBRARY_NAME
{
+DEFINE_LOGGER_LIBRARY_INTERFACE_HEADER
}  // namespace LIBRARY_NAME

#endif  // !defined(LIBRARY_NAME_LOGGER_SETUP_H)
```

```diff
// LoggerSetup.cpp
#include <LIBRARY_NAME/LoggerSetup.h>

+DEFINE_LOGGER_VARIABLES

namespace LIBRARY_NAME
{
+DEFINE_LOGGER_LIBRARY_INTERFACE_IMPLEMENTATION
}  // namespace LIBRARY_NAME
```
This should be everything that is required for a library. LIBRARY_NAME::_initLogger() can then be called from the library/executable that uses that library.

3. `DEFINE_LOGGER_CLASS_INTERFACE_HEADER` and `DEFINE_LOGGER_CLASS_INTERFACE_IMPLEMENTATION`: These two macros are to be used in libraries or executables which have a main class of which there is always an object available when the library/executable is used, e.g. ROS nodes or nodelets. They declare and implement class methods for initializing the logger and setting the logger level for that shared library/executable. Example usage, extracted from the ros_package_creation script:

```diff
// MY_NODE.h
#ifndef UPPER_PACKAGE_NAME_UPPER_CLASS_NAME_H
#define UPPER_PACKAGE_NAME_UPPER_CLASS_NAME_H

#include <ros/ros.h>
#include "aduulm_logger/aduulm_logger.hpp"

namespace PACKAGE_NAME
{
class CLASS_NAME
{
public:
  CLASS_NAME(ros::NodeHandle, ros::NodeHandle);

+  DEFINE_LOGGER_CLASS_INTERFACE_HEADER

private:
};
}  // namespace PACKAGE_NAME

#endif  // UPPER_PACKAGE_NAME_UPPER_CLASS_NAME_H
```

```diff
// MY_NODE.cpp
#include <PACKAGE_NAME/NODE_NAME.h>

+DEFINE_LOGGER_VARIABLES

namespace PACKAGE_NAME
{

CLASS_NAME::CLASS_NAME(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{
+  _setStreamName(ros::this_node::getName());  // for ROS nodes
+  _setStreamName(getName()); // for ROS nodelets
+  _initLogger();
}

+DEFINE_LOGGER_CLASS_INTERFACE_IMPLEMENTATION(CLASS_NAME)

}  // namespace PACKAGE_NAME
```

4. `LOGGER_ADD_SUBLOGGER_LIBRARY` and `LOGGER_ADD_SUBLOGGER_CLASS`: These two macros can be used to control the logger levels of dependent libraries automatically, e.g. if the logger level of one library/executable is changed, the logger levels of other linked shared libraries can be adjusted automatically to the same level. Example usage, extracted from the ros_package_creation script:

```diff
// MY_NODE.cpp
#include <PACKAGE_NAME/NODE_NAME.h>
+#include <LIBRARY_NAME/LoggerSetup.h>

DEFINE_LOGGER_VARIABLES

namespace PACKAGE_NAME
{

CLASS_NAME::CLASS_NAME(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{
+  LOGGER_ADD_SUBLOGGER_LIBRARY(LIBRARY_NAME);
  // The following three function calls get forwarded to the library
  _setStreamName(ros::this_node::getName());
  _initLogger();
  _setLogLevel(aduulm_logger::LoggerLevels::Debug);
}

DEFINE_LOGGER_CLASS_INTERFACE_IMPLEMENTATION(CLASS_NAME)

}  // namespace PACKAGE_NAME
```

### Simple message printer
* Via rosrun: `rosrun 'rosrun aduulm_logger print.py <type> "<msg>"
* Via launch file: `<node name="pub_err" pkg="aduulm_logger" type="print.py" args="<type> '<msg>'" output="screen" />`
    * `<type>` may be either "fatal", "err", "info", "warn" or "debug"
    * `<msg>` is your message

### Building
* `catkin build`

### Unit Tests

### Config files

### Launch files

### Nodes

#### Subscribed Topics

#### Published Topics

#### Services

#### Actions

#### Static Parameters

#### Dynamic Parameters

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://mrm-git.e-technik.uni-ulm.de/aduulm/source/ros/aduulm_logger/issues).
