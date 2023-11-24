#!/usr/bin/env python3

import sys
import rospy
import ctypes
# from client_lib import KetiROSBridge
from ctypes import cdll

def main():
    rospy.init_node('grpc_client')
    nh = rospy.NodeHandle()

    host_name = sys.argv[0]
    bridge_name = sys.argv[1]

    osi_bridge = None

    if bridge_name == "keti":
        rospy.loginfo("Setting Keti ROS bridge")

        client_lib = cdll.LoadLibrary('../../devel/lib/libclient_lib.so')
        ketirosbridge = client_lib.KetiROSBridge()
        osi_bridge = ketirosbridge(host_name)

    else:
        rospy.logerr("There is no bridge named %s", bridge_name)
        return -1

    if osi_bridge is None:
      return
    else:
      osi_bridge.ClientStartListen()
      osi_bridge.StartBridge()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
