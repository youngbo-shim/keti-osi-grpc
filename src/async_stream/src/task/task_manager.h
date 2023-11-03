#pragma once

#include <atomic>
#include <future>
#include <condition_variable>
#include <mutex>
#include <functional>
#include <thread>
#include <queue>
#include <vector>

#include "common/macros.h"

namespace keti {
namespace task {

#define DEFAULT_POOL_PROCESSOR_NUM 8

class TaskManager {
 public:
  virtual ~TaskManager();

  void Shutdown();

  void TaskThread();

  template <typename F, typename... Args>
  auto Enqueue(F&& func, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type> {
    using return_type = typename std::result_of<F(Args...)>::type;
    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<F>(func), std::forward<Args>(args)...));    
    if (!stop_.load()) {
      std::lock_guard<std::mutex> lock(mutex_task_);
      task_queue_->push([task]() { (*task)(); });      
    }
    std::future<return_type> res(task->get_future());
    cv_task_.notify_one();
    return res;
  }

 private:
  uint32_t num_threads_ = 0;
  std::vector<std::thread> task_threads_;
  std::atomic<bool> stop_ = {false};
  std::shared_ptr<std::queue<std::function<void()>>> task_queue_;
  std::condition_variable cv_task_;
  std::mutex mutex_task_;
  DECLARE_SINGLETON(TaskManager);
};

}  // namespace task
}  // namespace keti
