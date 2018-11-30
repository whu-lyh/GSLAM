#ifndef GSLAM_CORE_THREADPOOL_H
#define GSLAM_CORE_THREADPOOL_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>
#include <future>
#include <queue>
#include "Glog.h"

namespace GSLAM{

// A simple threadpool implementation.
class ThreadPool {
 public:
  // All the threads are created upon construction.
  explicit ThreadPool(const int num_threads): stop(false) {
        CHECK_GE(num_threads, 1)
            << "The number of threads specified to the ThreadPool is insufficient.";
        for (size_t i = 0; i < num_threads; ++i) {
          workers.emplace_back([this] {
            for (;;) {
              std::function<void()> task;

              {
                std::unique_lock<std::mutex> lock(this->queue_mutex);
                this->condition.wait(lock, [this] {
                  return this->stop || !this->tasks.empty();
                });
                if (this->stop && this->tasks.empty()) return;
                task = std::move(this->tasks.front());
                this->tasks.pop();
              }

              task();
            }
            });
        }
      }
  ~ThreadPool(){
        {
          std::unique_lock<std::mutex> lock(queue_mutex);
          stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : workers)
          worker.join();
      }


  // Adds a task to the threadpool.
  template <class F, class... Args>
  auto Add(F&& f, Args&& ... args)
      ->std::future<typename std::result_of<F(Args...)>::type>;

  size_t taskNumLeft() { return tasks.size(); }
 private:
  // Keep track of threads so we can join them
  std::vector<std::thread> workers;
  // The task queue
  std::queue<std::function<void()> > tasks;

  // Synchronization
  std::mutex queue_mutex;
  std::condition_variable condition;
  bool stop;

};

// add new work item to the pool
template <class F, class... Args>
auto ThreadPool::Add(F&& f, Args&& ... args)
    ->std::future<typename std::result_of<F(Args...)>::type> {
  using return_type = typename std::result_of<F(Args...)>::type;

  auto task = std::make_shared<std::packaged_task<return_type()> >(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> res = task->get_future();
  {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // don't allow enqueueing after stopping the pool
    CHECK(!stop) << "The ThreadPool object has been destroyed! Cannot add more "
                    "tasks to the ThreadPool!";

    tasks.emplace([task]() {
      (*task)();
    });
  }
  condition.notify_one();
  return res;
}

}

#endif
