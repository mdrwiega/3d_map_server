#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <future>
#include <atomic>
#include <functional>
#include <condition_variable>

namespace thread_pool {

class Semaphore {
 public:
  Semaphore(int count = 0) :  count_(count) {}

  inline void Wait() {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    condition_var_.wait(lock, [&]() { return count_; }); // block if value_ == 0
    --count_;
  }

  inline void Notify() {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    ++count_;
    condition_var_.notify_one();
  }

 private:
  Semaphore(const Semaphore&) = delete;
  const Semaphore& operator=(const Semaphore&) = delete;

  std::mutex mutex_;
  std::condition_variable condition_var_;
  std::uint32_t count_;
};

class ThreadPool {
 public:
  ThreadPool(int threads_num = std::thread::hardware_concurrency()) {
    if (threads_num == 0) {
      throw std::invalid_argument("Wrong number of threads.");
    }

    terminate_ = false;

    for (int i = 0; i < threads_num; ++i) {
      threads_.emplace_back(std::thread(&ThreadPool::WorkerThread, this));
    }
  }

  ~ThreadPool() {
    terminate_ = true;

    for (unsigned i = 0; i < threads_.size(); ++i) {
        active_sem_.Notify();
    }

    for (auto& it: threads_) {
        it.join();
    }
  }

  int GetThreadsNum() const {
    return threads_.size();
  }

  template<typename T, typename... Ts>
  auto Submit(T&& function, Ts&&... params) -> std::future<typename std::result_of<T(Ts...)>::type> {

    auto task = std::make_shared<std::packaged_task<typename std::result_of<T(Ts...)>::type()>>(
        std::bind(std::forward<T>(function), std::forward<Ts>(params)...)
    );
    auto task_result = task->get_future();

    queue_sem_.Wait();
    task_queue_.emplace([task]() { (*task)(); });
    queue_sem_.Notify();
    active_sem_.Notify();

    return task_result;
  }

private:
    ThreadPool(const ThreadPool&) = delete;
    const ThreadPool& operator=(const ThreadPool&) = delete;

    void WorkerThread() {
      while (true) {
        active_sem_.Wait();
        if (terminate_) {
            break;
        }

        queue_sem_.Wait();

        auto task = std::move(task_queue_.front());
        task_queue_.pop();

        queue_sem_.Notify();

        if (terminate_) {
            break;
        }
        task();
      }
    }

    std::vector<std::thread> threads_;

    std::queue<std::function<void()>> task_queue_;

    Semaphore queue_sem_{1};
    Semaphore active_sem_{0};

    std::atomic<bool> terminate_;
};

} // namespace thread_pool