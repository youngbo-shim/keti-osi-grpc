#include "task/task_manager.h"

namespace keti {
namespace task {

TaskManager::TaskManager() {  
  num_threads_ = DEFAULT_POOL_PROCESSOR_NUM;

  task_queue_ = std::make_shared<std::queue<std::function<void()>>>();

  task_threads_.reserve(num_threads_);
  for (uint32_t i = 0; i < num_threads_; i++) {
    task_threads_.emplace_back([this]() { this->TaskThread(); });
  }
}

TaskManager::~TaskManager() { Shutdown(); }

void TaskManager::Shutdown() {
  if (stop_.exchange(true)) {
    cv_task_.notify_all();
    return;
  }
  for (auto& t : task_threads_) {
    t.join();
  }
}

void TaskManager::TaskThread() {
  while(true) {
    std::unique_lock<std::mutex> lock(mutex_task_);    
    cv_task_.wait(lock, [this]() { return !this->task_queue_->empty() || stop_.load(); });    
    if (stop_.load() && this->task_queue_->empty()) {
      return;
    }

    std::function<void()> task = std::move(task_queue_->front());
    task_queue_->pop();
    lock.unlock();
    task();
  }
}

}  // namespace task
}  // namespace keti
