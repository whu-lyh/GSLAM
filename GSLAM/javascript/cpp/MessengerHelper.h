#ifndef MESSENGERHELPER_H
#define MESSENGERHELPER_H
#include "NodeBind.h"
#include <GSLAM/core/Svar.h>
#include <GSLAM/core/Messenger.h>

namespace GSLAM{

uv_loop_t* jsloop;
std::list<std::function<void(void)> > job_queue;
std::mutex job_mutex;

struct TaskInfo
{
    uv_work_t request;                  // libuv
    std::function<void(void)> task;
};

void processAll(){
    std::list<std::function<void(void)> > jobs;
    std::unique_lock<std::mutex> lock(job_mutex);
    {
        jobs=job_queue;
        job_queue.clear();
    }
    if(jobs.empty()) return;
    for(auto job:jobs){
        job();
    }
}
// called by libuv worker in separate thread
static void WorkThread(uv_work_t *req)
{
}

// called by libuv in event loop when async function completes
static void JSThread(uv_work_t *req,int status)
{
    // get the reference to the baton from the request
    TaskInfo *baton = static_cast<TaskInfo *>(req->data);
    LOG(INFO)<<"JSThread:"<<std::this_thread::get_id();
    baton->task();
//    std::string str;
//    nbind::WireType msg=nbind::convertToWire(str);
    // delete the baton object
    delete baton;
}

template <class F, class... Args>
auto addJSTask(F&& f, Args&& ... args)
    ->std::future<typename std::result_of<F(Args...)>::type> {
  using return_type = typename std::result_of<F(Args...)>::type;

  auto task = std::make_shared<std::packaged_task<return_type()> >(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));
  std::future<return_type> res = task->get_future();

  {
    std::unique_lock<std::mutex> lock(job_mutex);
    job_queue.push_back([task](){(*task)();});
  }
  return res;
  TaskInfo *info = new TaskInfo;
  // attach baton to uv work request
  info->request.data = info;
  info->task=[task]() {
      (*task)();
    };
  // queue the async function to the event loop
  // the uv default loop is the node.js event loop
  uv_queue_work(jsloop,&info->request,WorkThread,JSThread);

  return res;
}

template <typename T>
void addMessengerSupportPtr(std::string name){
    SvarWithType<const std::type_info*>::instance().insert(name,&typeid(T));
    SvarWithType<std::function<void(const Publisher&,nbind::WireType)> >::instance()
            .insert(typeid(T).name(),[name](const Publisher& pub,nbind::WireType obj){
//        LOG(INFO)<<"Publish"<<name;
        std::shared_ptr<T> shared=nbind::BindWrapper<T>::getShared(v8::Handle<v8::Object>::Cast(obj),nbind::TypeFlags::isSharedPtr);

        if(shared){
//            LOG(INFO)<<"Publish with shared";
            return pub.publish<T>(shared);
        }
    });

    SvarWithType<std::function<void(std::shared_ptr<nbind::cbFunction>,const std::shared_ptr<void>&)> >::instance()
            .insert(typeid(T).name(),[](std::shared_ptr<nbind::cbFunction> cbk,const std::shared_ptr<void>& obj){
        std::shared_ptr<T> msg=*(const std::shared_ptr<T>*)&obj;
        auto task=[cbk,msg](){(*cbk).call<void>(msg);};
        addJSTask(task);
        return;
    });
}

template <typename T>
void addMessengerSupport(std::string name){
    SvarWithType<const std::type_info*>::instance().insert(name,&typeid(T));
    SvarWithType<std::function<void(const Publisher&,nbind::WireType)> >::instance()
            .insert(typeid(T).name(),[name](const Publisher& pub,nbind::WireType obj){

        T msg=nbind::convertFromWire<T>(obj);
        pub.publish<T>(msg);
    });

    SvarWithType<std::function<void(std::shared_ptr<nbind::cbFunction>,const std::shared_ptr<void>&)> >::instance()
            .insert(typeid(T).name(),[](std::shared_ptr<nbind::cbFunction> cbk,const std::shared_ptr<void>& obj){
        std::shared_ptr<T> msg=*(const std::shared_ptr<T>*)&obj;
        auto task=[cbk,msg](){(*cbk).call<void>(msg);};
        addJSTask(task);
        return;
//        (*cbk).call<void>(*msg);
//        auto func=cbk->getJsFunction();
//        v8::Local<v8::Value> argv[] = {
//            nbind::convertToWire(*msg),
//            // Avoid error C2466: cannot allocate an array of constant size 0.
//            Nan::Null()
//        };
//        LOG(INFO)<<"3";

//        Nan::MaybeLocal<v8::Value> result = Nan::Call(func, Nan::GetCurrentContext()->Global(), 1, argv);

//        addJSTask([func,msg](){
//            LOG(INFO)<<"1";
//            LOG(INFO)<<"2";

//            LOG(INFO)<<"4";
////            LOG(INFO)<<"TaskThread:"<<std::this_thread::get_id();
////            LOG(INFO)<<"MSG"<<*msg;
//////            auto context=v8::Isolate::GetCurrentContext();
////            LOG(INFO)<<"Context obtained.";
////            (*cbk).call<void>();
//        });
    });
}
}
#endif // MESSENGERHELPER_H
