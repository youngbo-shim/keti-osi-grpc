#!/usr/bin/env python3

import osi_bridge
import threading

def run():
  osi_bridge.ros_init('grpc_client')

  # host_name = sys.argv[0]
  host_name = "localhost:50051"
  keti_ros_bridge = osi_bridge.KetiROSBridge(host_name)

  client_thread = threading.Thread(target=keti_ros_bridge.ClientStartListen, args=())
  bridge_thread = threading.Thread(target=keti_ros_bridge.StartBridge,  args=())

  client_thread.start()
  bridge_thread.start()

if __name__ == '__main__':
  run()