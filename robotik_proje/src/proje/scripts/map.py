#!/usr/bin/env python3
import os, time

# time.sleep(3)
os.system("rosrun gmapping slam_gmapping scan:=/rtg/hokuyo _odom_frame:=/rtg/odom _base_frame:=rtg/base_link")
