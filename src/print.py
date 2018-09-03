#!/usr/bin/env python

'''
# Name: launch_file_printer.py
# Author: Martin Herrmann
'''

# Simply launch this node from a launch file with the following arguments
#
# rosrun launch_file_printer print

import rospy
import sys

def usage():
    print("usage: rosrun launch_file_printer print <type> '<msg>'")
    print("    Hereby type can be from 'fatal', 'err', 'warn', 'info', 'debug'")
    print("    Print whatever message shall be printed")

if __name__ == '__main__':
    rospy.init_node('print', anonymous=True)
    #print 'Number of arguments:', len(sys.argv), 'arguments.'
    #print 'Argument List:', str(sys.argv)
    if len(sys.argv) < 3:
        usage()
    else:
        if sys.argv[1] == "fatal":
            rospy.logfatal(sys.argv[2])
        elif sys.argv[1] == "err":
            rospy.logerr(sys.argv[2])
        elif sys.argv[1] == "warn":
            rospy.logwarn(sys.argv[2])
        elif sys.argv[1] == "info":
            rospy.loginfo(sys.argv[2])
        elif sys.argv[1] == "debug":
            rospy.logdebug(sys.argv[2])
        else:
            usage()
