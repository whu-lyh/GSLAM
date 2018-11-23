// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: zd5945@126.com (Yong Zhao) 353184965@qq.com(Guochen Liu)
//
// Messenger: A light-weight, efficient, thread-safe message publish and
// subscribe tool similar with ROS, a popular robot operating system
// The tool has the following features:
// * Header only based on c++11, no extra dependency, makes it portable.
// * Thread safe and support multi-thread condition notify mode by setting the queue size.
// * Able to transfer any classes efficiently, including ROS defined messages, which means it can replace ROS messagging or work with it.
#ifndef GSLAM_MESSENGER_H
#define GSLAM_MESSENGER_H
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#define HAS_GSLAM

#if defined(HAS_GSLAM)
#include <GSLAM/core/Glog.h>
#include <GSLAM/core/Mutex.h>
#elif !defined(GSLAM_MUTEX__H)
#include <atomic>
#include <condition_variable>
#include <future>
#include <queue>
#include <thread>

// A simple threadpool implementation.
class ThreadPool {
 public:
  // All the threads are created upon construction.
  explicit ThreadPool(const int num_threads) : stop(false) {
    for (size_t i = 0; i < num_threads; ++i) {
      workers.emplace_back([this] {
        for (;;) {
          std::function<void()> task;

          {
            std::unique_lock<std::mutex> lock(this->queue_mutex);
            this->condition.wait(
                lock, [this] { return this->stop || !this->tasks.empty(); });
            if (this->stop && this->tasks.empty()) return;
            task = std::move(this->tasks.front());
            this->tasks.pop();
          }

          task();
        }
      });
    }
  }
  ~ThreadPool() {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      stop = true;
    }
    condition.notify_all();
    for (std::thread& worker : workers) worker.join();
  }

  // Adds a task to the threadpool.
  template <class F, class... Args>
  auto Add(F&& f, Args&&... args)
      -> std::future<typename std::result_of<F(Args...)>::type>;

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
auto ThreadPool::Add(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type> {
  using return_type = typename std::result_of<F(Args...)>::type;

  auto task = std::make_shared<std::packaged_task<return_type()> >(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> res = task->get_future();
  {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // don't allow enqueueing after stopping the pool
    if (stop)
      std::cerr << "The ThreadPool object has been destroyed! Cannot add more "
                   "tasks to the ThreadPool!";

    tasks.emplace([task]() { (*task)(); });
  }
  condition.notify_one();
  return res;
}
#endif

namespace GSLAM {

class Messenger;
class Publisher;

class Subscriber {
 public:
  typedef std::function<void(const std::shared_ptr<void>&)> CallBackFunc;

  Subscriber() {}
  ~Subscriber() {}

  /**
   * \brief Unsubscribe the callback associated with this Subscriber
   *
   * This method usually does not need to be explicitly called, as automatic
   * shutdown happens when
   * all copies of this Subscriber go out of scope
   *
   * This method overrides the automatic reference counted unsubscribe, and
   * immediately
   * unsubscribes the callback associated with this Subscriber
   */
  void shutdown() ;

  std::string getTopic() const {
    if (impl_) return impl_->topic_;
    return "";
  }

  std::string getTypeName() const {
    if (impl_) return impl_->type_;
    return "";
  }

  /**
   * \brief Returns the number of publishers this subscriber is connected to
   */
  uint32_t getNumPublishers() const {
    if (!impl_) return 0;
    return impl_->publishers_.size();
  }

  operator void*() const { return (impl_) ? (void*)1 : (void*)0; }

  bool operator<(const Subscriber& rhs) const { return impl_ < rhs.impl_; }

  bool operator==(const Subscriber& rhs) const { return impl_ == rhs.impl_; }

  bool operator!=(const Subscriber& rhs) const { return impl_ != rhs.impl_; }

 protected:
  friend class Messenger;
  friend class Publisher;
  struct Impl {
    Impl(const std::string& topic, const std::string& type,
         const CallBackFunc& callback, size_t queue_size = 0)
        : topic_(topic),
          type_(type),
          callback_(callback),
          unsubscribed_(false),
          queue_size_(queue_size),
          workthread_(queue_size ? new ThreadPool(1) : nullptr) {}

    std::string topic_, type_;
    CallBackFunc callback_;
    bool unsubscribed_;
    std::set<Publisher> publishers_;
    std::mutex mutex_;
    size_t queue_size_;
    std::shared_ptr<ThreadPool> workthread_;
  };

  Subscriber(std::shared_ptr<Subscriber::Impl> impl) : impl_(impl) {}
  Subscriber(const std::string& topic, const std::string& type,
             const CallBackFunc& callback, size_t queue_size = 0)
      : impl_(new Impl(topic,type,callback,queue_size)) {}
  virtual void publish(const std::type_info& typeinfo,
                       const std::shared_ptr<int>& message) const {
    //        LOG(INFO)<<"Node "<<impl_->topic_<<" publishing message
    //        "<<typeinfo.name();
    if (!impl_) return;
    if (impl_->unsubscribed_) return;
    if (impl_->workthread_ &&
        impl_->workthread_->taskNumLeft() < impl_->queue_size_) {// FIXME: what to do when queue size large
      impl_->workthread_->Add([this, message]() { impl_->callback_(message); });
      return;
    }
    impl_->callback_(message);
  }
  std::string key()const{return getTopic()+"#"+getTypeName();}
  void addPublisher(const Publisher& pub)const{
      std::unique_lock<std::mutex> lock(impl_->mutex_);
      impl_->publishers_.insert(pub);
  }

  void erasePublisher(const Publisher& pub)const{
      std::unique_lock<std::mutex> lock(impl_->mutex_);
      impl_->publishers_.erase(pub);
  }

  std::shared_ptr<Impl> impl_;
};

class Publisher {
 public:
  Publisher() {}

  virtual ~Publisher() { }

  /**
   * \brief Publish a message on the topic associated with this Publisher.
   * The message should be copyable.
   */
  template <typename M>
  void publish(const M& message) const;

  /**
   * \brief Publish a message without a copy!
   */
  template <typename M>
  void publish(const std::shared_ptr<M>& message) const;

  /**
   * \brief Shutdown the advertisement associated with this Publisher
   *
   * This method usually does not need to be explicitly called, as automatic
   * shutdown happens when
   * all copies of this Publisher go out of scope
   *
   * This method overrides the automatic reference counted unadvertise, and does
   * so immediately.
   * \note Note that if multiple advertisements were made through
   * NodeHandle::advertise(), this will
   * only remove the one associated with this Publisher
   */
  void shutdown() {
      if (!impl_) return;

      for(Subscriber it:impl_->subscribers)
          it.erasePublisher(*this);
      impl_.reset();
  }

  /**
   * \brief Returns the topic that this Publisher will publish on.
   */
  std::string getTopic() const {
    if (impl_) return impl_->topic_;
    return "";
  }

  /**
   * \brief Returns the topic that this Publisher will publish on.
   */
  std::string getTypeName() const {
    if (impl_) return impl_->type_;
    return "";
  }

  /**
   * \brief Returns the number of subscribers that are currently connected to
   * this Publisher
   */
  uint32_t getNumSubscribers() const {
    if (impl_) return impl_->subscribers.size();
    return 0;
  }

  /**
   * \brief Returns whether or not this topic is latched
   */
  bool isLatched() const { return getNumSubscribers(); }

  operator void*() const { return (impl_) ? (void*)1 : (void*)0; }

  bool operator<(const Publisher& rhs) const { return impl_ < rhs.impl_; }

  bool operator==(const Publisher& rhs) const { return impl_ == rhs.impl_; }

  bool operator!=(const Publisher& rhs) const { return impl_ != rhs.impl_; }


 protected:
  friend class Messenger;
  friend class Subscriber;

  struct Impl {
    Impl(const std::string& topic, const std::string& type,
         size_t queue_size = 0)
        : topic_(topic),
          type_(type),
          queue_size_(queue_size),
          workthread_(queue_size ? new ThreadPool(1) : nullptr) {}

    std::string topic_;
    std::string type_;
    std::set<Subscriber> subscribers;
    size_t queue_size_;
    std::shared_ptr<ThreadPool> workthread_;
    std::mutex mutex_;
  };
  Publisher(Impl* implement) : impl_(implement) {}
  Publisher(const std::string& topic, const std::string& type,
            size_t queue_size = 0)
      :impl_(new Impl(topic,type,queue_size)){}
  std::string key()const{return getTopic()+"#"+getTypeName();}
  void addSubscriber(const Subscriber& sub)const{
      std::unique_lock<std::mutex> lock(impl_->mutex_);
      impl_->subscribers.insert(sub);
  }
  void eraseSubscriber(const Subscriber& sub)const{
      std::unique_lock<std::mutex> lock(impl_->mutex_);
      impl_->subscribers.erase(sub);
  }

  std::shared_ptr<Impl> impl_;
};

class Messenger {
 public:
    Messenger():d(new Data()){}
  virtual ~Messenger() {}

  static Messenger& instance(){
      static std::shared_ptr<Messenger> inst(new Messenger());
      return *inst;
  }

  template <class M>
  Publisher advertise(const std::string& topic, uint32_t queue_size = 0,
                      bool latch = false) {
    Publisher pub(topic, std::string(typeid(M).name()),queue_size);
    join(pub);
    return pub;
  }

  template <class M>
  Subscriber subscribe(
      const std::string& topic, uint32_t queue_size,
      std::function<void(const std::shared_ptr<M>&)> callback) {
      Subscriber sub(topic, typeid(M).name(),
                     *(Subscriber::CallBackFunc*)(&callback),
                     queue_size);
      join(sub);
      return sub;
  }

  template <class T, class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                       void (T::*fp)(const std::shared_ptr<M>&), T* obj) {
    std::function<void(const std::shared_ptr<M>&)> cbk =
        std::bind(fp, obj, std::placeholders::_1);
    return subscribe(topic, queue_size, cbk);
  }

  template <class M>
  Subscriber subscribe(const std::string& topic, int queue_size,
                       void (*fp)(const std::shared_ptr<M>&)) {
    return subscribe(topic, queue_size,
                     std::function<void(const std::shared_ptr<M>&)>(fp));
  }

  template <class M>
  Subscriber subscribe(const std::string& topic, int queue_size,
                       void (*fp)(const M&)){
      std::function<void(const std::shared_ptr<M>&)> cbk=
              [fp](const std::shared_ptr<M>& msg){
          fp(*msg);
    };
      return subscribe(topic,queue_size,cbk);
  }


  template <class T, class M>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                       void (T::*fp)(const M&), T* obj) {
      std::function<void(const std::shared_ptr<M>&)> cbk=
              [fp,obj](const std::shared_ptr<M>& msg){
          std::function<void(const M&)> call =
                  std::bind(fp, obj, std::placeholders::_1);
          call(*msg);
    };
    return subscribe(topic, queue_size, cbk);
  }

  bool findSubscriber(const Publisher& pub) const{
    std::unique_lock<std::mutex> lock(d->mutex_);
    auto it = d->subscribers_.find(pub.key());
    if (it == d->subscribers_.end()) return false;
    {
        {
            std::unique_lock<std::mutex> lock(pub.impl_->mutex_);
            pub.impl_->subscribers = it->second;
        }
      for (const Subscriber& sub : it->second) {
          if (!sub) continue;
          if (sub.impl_->type_ != pub.impl_->type_) continue;
            sub.addPublisher(pub);
        }
    }
    return true;
  }

  bool findPublisher(const Subscriber& sub) const{
    std::unique_lock<std::mutex> lock(d->mutex_);
    if (!sub) return false;
    auto it = d->publishers_.find(sub.key());
    if (it == d->publishers_.end()) return false;

    {
      std::unique_lock<std::mutex> lock(sub.impl_->mutex_);
      sub.impl_->publishers_ = it->second;
      for (const Publisher& pub : it->second) {
        if (!pub) continue;
        if (sub.impl_->type_ != pub.impl_->type_) continue;
        pub.addSubscriber(sub);
      }
    }
  }

  const std::map<std::string, std::set<Publisher> >& getPublishers()const{
      return d->publishers_;
  }

  const std::map<std::string, std::set<Subscriber> >& getSubscribers()const{
      return d->subscribers_;
  }

  std::string introduction()const{
      std::stringstream sst;
      if(getPublishers().size())
          sst<<"The following publishers are advertised:\n";
      for(auto it:getPublishers())
          for(const Publisher& pub:it.second){
              sst<<pub.getTopic()<<" : "<<pub.getTypeName()<<std::endl;
          }
      if(getSubscribers().size())
          sst<<"The following subscribers are subscribed:\n";
      for(auto it:getSubscribers())
          for(const Subscriber& pub:it.second){
              sst<<pub.getTopic()<<" : "<<pub.getTypeName()<<std::endl;
          }
      return sst.str();
  }

  void join(const Publisher& pub){
      {
        std::unique_lock<std::mutex> lock(d->mutex_);
        d->publishers_[pub.key()].insert(pub);
      }
      findSubscriber(pub);
  }

  void join(const Subscriber& sub){
      {
        std::unique_lock<std::mutex> lock(d->mutex_);
        d->subscribers_[sub.key()].insert(sub);
      }
      findPublisher(sub);
  }

  void join(Messenger another){
      for(auto it:another.getPublishers())
          for(const Publisher& pub:it.second){
              join(pub);
          }
      for(auto it:another.getSubscribers())
          for(const Subscriber& sub:it.second){
              join(sub);
          }
  }

 private:
  struct Data{
      std::mutex mutex_;
      std::map<std::string, std::set<Publisher> >  publishers_;
      std::map<std::string, std::set<Subscriber> > subscribers_;
  };
  std::shared_ptr<Data> d;
};

inline void Subscriber::shutdown()
{
    if (!impl_) return;

    impl_->unsubscribed_ = true;
    for(Publisher it:impl_->publishers_)
        it.eraseSubscriber(*this);
    impl_.reset();
}

template <typename M>
void Publisher::publish(const std::shared_ptr<M>& message) const {
  if (!impl_) return;

  if (typeid(M).name() != impl_->type_) {
#ifdef HAS_GSLAM
    LOG(ERROR) << "Type mismatch:" << typeid(M).name() << " to "
               << impl_->type_;
#else
    std::cerr << "Type mismatch:" << typeid(M).name() << " to " << impl_->type_
              << "\n";
#endif
    return;
  }

  if (impl_->subscribers.empty()) {
      return;
  }

  if (impl_->workthread_ &&
      impl_->workthread_->taskNumLeft() < impl_->queue_size_) {
      // FIXME: what to do when queue size large
    impl_->workthread_->Add([this, message]() {

      std::set<Subscriber> subscribers;
      {
        std::unique_lock<std::mutex> lock(impl_->mutex_);
        subscribers = impl_->subscribers;
      }

      for (const auto& s : subscribers) {
        s.publish(typeid(M), *(const std::shared_ptr<int>*)&message);
      }
    });
    return;
  }

  std::set<Subscriber> subscribers;
  {
    std::unique_lock<std::mutex> lock(impl_->mutex_);
    subscribers = impl_->subscribers;
  }

  for (auto& s : subscribers) {
    s.publish(typeid(M), *(const std::shared_ptr<int>*)&message);
  }
}

template <typename M>
void Publisher::publish(const M& m) const {
  M* message = new M(m);
  std::shared_ptr<M> msg(message);
  publish(msg);
}

}  // end of namespace GSLAM

#endif
