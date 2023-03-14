__all__ = ['create_logger', 'get_logger', 'logger']

import logging


class LoggerWrapper:
    def __init__(self, logger):
        self.logger = logger

    def __getattribute__(self, key):
        if key != "logger":
            return getattr(self.logger, key)
        return super().__getattribute__(key)


class Logger:
    _levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]

    def __init__(self, *args, **kwargs):
        def use_print_logger():
            self.functions = {
                "DEBUG": lambda *args, **kwargs: self._do_print("DEBUG", *args),
                "INFO": lambda *args, **kwargs: self._do_print("INFO", *args),
                "WARNING": lambda *args, **kwargs: self._do_print("WARNING", *args),
                "ERROR": lambda *args, **kwargs: self._do_print("ERROR", *args),
                "CRITICAL": lambda *args, **kwargs: self._do_print("CRITICAL", *args),
                "SET_LEVEL": lambda level: None,
            }
        # use_print_logger()
        # self.set_level("INFO")
        # return
        try:
            import rospy
            self.functions = {
                "DEBUG": rospy.logdebug,
                "INFO": rospy.loginfo,
                "WARNING": rospy.logwarn,
                "ERROR": rospy.logerr,
                "CRITICAL": rospy.logfatal,
                "SET_LEVEL": lambda level: logging.getLogger('rosout').setLevel(getattr(logging, level)),
            }
        except ImportError:
            try:
                import rclpy
                import rclpy.logging
                from rclpy.impl.rcutils_logger import _internal_callers
                import os.path
                _internal_callers.append(os.path.realpath(__file__),)
                rclpy.logging
                from rclpy.logging import LoggingSeverity
                node = args[0]
                args = args[1:]
                self.functions = {
                    "DEBUG": node.get_logger().debug,
                    "INFO": node.get_logger().info,
                    "WARNING": node.get_logger().warning,
                    "ERROR": node.get_logger().error,
                    "CRITICAL": node.get_logger().fatal,
                    "SET_LEVEL": lambda level: rclpy.logging.set_logger_level(node.get_logger().name, getattr(LoggingSeverity, level)),
                }
            except IndexError:
                # Maybe the logger will be initialized later, do not complain for now...
                use_print_logger()
            except ImportError:
                print("No logging backend found, falling back to print().")
                use_print_logger()
        self.set_level("INFO")

    def set_level(self, level):
        self.level = level
        self.functions["SET_LEVEL"](level)

    def _do_print(self, level, *args):
        if self._levels.index(level) < self._levels.index(self.level):
            return
        print(f"[{level}]", *args)

    def _print(self, level, args):
        s = " ".join(str(a) for a in args)
        self.functions[level](s)

    def debug(self, *args):
        self._print("DEBUG", args)

    def info(self, *args):
        self._print("INFO", args)

    def warning(self, *args):
        self._print("WARNING", args)

    def error(self, *args):
        self._print("ERROR", args)

    def critical(self, *args):
        self._print("CRITICAL", args)


logger = None

def create_logger(*args, **kwargs):
    global logger
    if logger is not None:
        logger.logger = _logger = Logger(*args, **kwargs)
        return logger
    return get_logger(*args, **kwargs)

def get_logger(*args, **kwargs):
    global logger
    if logger is not None:
        return logger
    logger = LoggerWrapper(Logger(*args, **kwargs))
    return logger

logger = get_logger()
