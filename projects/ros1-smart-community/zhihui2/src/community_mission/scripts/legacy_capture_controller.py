#!/usr/bin/env python3
"""Compatibility entry point for the original timed capture mission.

The refactored mission_manager is the recommended state-machine interface;
this file is kept so existing capture workflows can still be ported here.
"""
from mission_manager import MissionManager


if __name__ == "__main__":
    import rospy
    rospy.init_node("legacy_capture_controller")
    MissionManager()
    rospy.spin()
