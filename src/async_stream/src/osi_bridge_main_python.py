#!/usr/bin/env python3

import osi_bridge
import threading

def run():
  osi_bridge.ros_init('grpc_client')

  host_name = "localhost:50051"
  cmd_host_name = "localhost:50052"
  keti_ros_bridge = osi_bridge.KetiROSBridge(host_name, cmd_host_name)

  bridge_thread = threading.Thread(target=keti_ros_bridge.StartBridge,  args=())
  client_thread = threading.Thread(target=keti_ros_bridge.ClientStartListen, args=())
  server_thread = threading.Thread(target=keti_ros_bridge.ServerStartStream, args=())

  bridge_thread.start()
  client_thread.start()
  server_thread.start()

if __name__ == '__main__':
  run()
