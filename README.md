# aduulm_logger

## Overview

The aduulm logger package is thought to be used in projects with libraries for ROS and ADTF. It redirects logging messages either to a cout or ROS_LOG command and therefore simplifies the portation of packages from ADTF to ROS which use the ADTF logging commands. It also contains a node for simple log message printing from command line or launch file

**Keywords:** mrm, logger, ros, aduulm, ADTF, ROS, logging


### License

The source code is not official released and is only for internal use.

**Author(s): Martin Herrmann   
Maintainer:  Martin Herrmann,  Martin.Herrmann@uni-ulm.de  
Affiliation: Institute of Measurements, Control and Microtechnology, Ulm University**

The aduulm_logger package has been tested under [ROS] Kinetic and Ubuntu 16.04. 
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## Dependencies

* [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)


## Usage
### Logger
* The aduulm_logger uses the same logging commands as the mrm ADTF logger (LOG_ERR, LOG_WARN, LOG_INF, ...) and has two modes. It can print the messages like the mrm ADTF logger via std::cout or redirect the messages to the ROS logger (ROS_ERROR, ROS_WARN, ROS_INFO, ...). This needs to be defined at build time via compiler flag `USE_ROS_LOG`:
`add_definitions(-DUSE_ROS_LOG)`
* Add the package to your project (catkin: find_package(catkin REQUIRED aduulm_logger), plain cmake include the logger's include directory).
* To be able to use the logger, it needs to be initialized before the first usage: `initLogger();`. If the logger is used in non ROS mode (no flag set), it can be initialized with the optional parameters file_name and log_level `initLogger(std::string file_name, uint16_t log_level)`. This can be used to adjust the log level at runtime and store a logfile.
* In contrast to the mrm logger a locking function is added to enable multithreaded usage. However this prevents from concurrent logging commands, but not from concurrent std::cout commands.
* Example usage can be seen in the ros_example in the ros_template package (root/ros_template).
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

Please report bugs and request features using the [Issue Tracker](https://mrm-130.e-technik.uni-ulm.de/herrmann/mrm_ros_template/issues).



