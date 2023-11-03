#pragma once

#include <future>
#include <utility>

#include "task/task_manager.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensorview_rpc.grpc.pb.h"

using osi3::Request;
using osi3::SensorView;
using osi3::CameraSensorView;
using osi3::LidarSensorView;
using osi3::SensorViewRPC;

namespace keti {
namespace task {

template <typename F, typename... Args>
static auto Async(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type> {
  return TaskManager::Instance()->Enqueue(std::forward<F>(f),
                                                std::forward<Args>(args)...);
}

}  // namespace task
}  // namespace keti 
