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
    _levels = ["None", "Error", "Warn", "Info", "Debug"]
    _levels_ints = list(range(len(_levels)))

    def __init__(self, *args, **kwargs):
        self.level = 'Info'

        def use_print_logger():
            self.functions = {
                "Debug": lambda *args, **kwargs: self._do_print("Debug", *args),
                "Info": lambda *args, **kwargs: self._do_print("Info", *args),
                "Warn": lambda *args, **kwargs: self._do_print("Warn", *args),
                "Error": lambda *args, **kwargs: self._do_print("Error", *args),
                "SET_LEVEL": lambda level: None,
            }

        # For debug purposes:
        # use_print_logger()
        # self.set_level("Info")
        # return

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
            logger_levels_to_severity = {
                "Debug": LoggingSeverity.DEBUG,
                "Info": LoggingSeverity.INFO,
                "Warn": LoggingSeverity.WARN,
                "Error": LoggingSeverity.ERROR,
                "None": LoggingSeverity.FATAL,
            }
            self.functions = {
                "Debug": node.get_logger().debug,
                "Info": node.get_logger().info,
                "Warn": node.get_logger().warning,
                "Error": node.get_logger().error,
                "SET_LEVEL": lambda level: rclpy.logging.set_logger_level(node.get_logger().name, logger_levels_to_severity[level]),
            }
        except IndexError:
            # Maybe the logger will be initialized later, do not complain for now...
            use_print_logger()
        except ImportError:
            print("No logging backend found, falling back to print().")
            use_print_logger()

        self.set_level("Info")

    def set_level(self, level):
        is_int = isinstance(level, int)
        is_str = isinstance(level, str)
        if is_int:
            try:
                level = self._levels[level]
            except IndexError:
                self.error(f"Not a valid logger level number: {level}")
                return
        elif (is_str and level not in self._levels) or not is_str:
            self.error(f"Not a valid logger level: {level}")
            return
        self.level = level
        self.functions["SET_LEVEL"](level)

    def get_level(self):
        return self.level

    def _do_print(self, level, *args):
        if self._levels.index(level) > self._levels.index(self.level):
            return
        print(f"[{level.upper()}]", *args)

    def _print(self, level, args):
        s = " ".join(str(a) for a in args)
        self.functions[level](s)

    def debug(self, *args):
        self._print("Debug", args)

    def deb(self, *args):
        self._print("Debug", args)

    def info(self, *args):
        self._print("Info", args)

    def inf(self, *args):
        self._print("Info", args)

    def warning(self, *args):
        self._print("Warn", args)

    def warn(self, *args):
        self._print("Warn", args)

    def error(self, *args):
        self._print("Error", args)

    def err(self, *args):
        self._print("Error", args)


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
