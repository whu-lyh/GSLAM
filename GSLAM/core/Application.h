#ifndef GSLAM_CORE_APPLICATION
#define GSLAM_CORE_APPLICATION

#include <GSLAM/core/Svar.h>
#include <GSLAM/core/Messenger.h>
#include <GSLAM/core/SharedLibrary.h>
#include <GSLAM/core/GSLAM.h>

#define REGISTER_APPLICATION(APP)                                  \
  extern "C" GSLAM::Application* createApplicationInstance() {          \
    return new APP();                      \
}
#define REGISTER_BUILDIN_APPLICATION(APP,NAME)                                  \
  extern "C" GSLAM::Application* createApplication##NAME() {          \
    return new APP();                      \
}\
    class APP##NAME##_Register{ \
    public: APP##NAME##_Register(){\
    GSLAM::SvarWithType<GSLAM::funcCreateApplication>::instance()\
    .insert(#NAME,createApplication##NAME);\
}} APP##NAME##_instance;

namespace GSLAM{

class Application;
typedef std::shared_ptr<Application> ApplicationPtr;
typedef Application* (*funcCreateApplication)();

class Application{
public:
    Application(std::string name)
        : running_(true), name_(name),
          gslam_version_(GSLAM_VERSION_STR){}

    bool        isRunning()const{return running_;}
    std::string name()const{return name_;}
    std::string gslam_version()const{return gslam_version_;}

    virtual Messenger init(Svar configuration)=0;

    static ApplicationPtr create(const std::string& path){
        auto createFunc=GSLAM::SvarWithType<GSLAM::funcCreateApplication>::instance()
                .get_var(path,NULL);
        if(createFunc)
            return ApplicationPtr(createFunc());
        SharedLibraryPtr plugin=Registry::get(path);
        if(!plugin)
            plugin=Registry::get("libgslam_"+path);
        if(!plugin)
            plugin=Registry::get("lib"+path);
        if(!plugin) return ApplicationPtr();
        createFunc = (funcCreateApplication)plugin->getSymbol(
            "createApplicationInstance");
        if (!createFunc)
          return ApplicationPtr();
        else
          return ApplicationPtr(createFunc());
    }
protected:
    bool        running_;
private:
    std::string name_;
    std::string gslam_version_;
};

}
#endif
