#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/MemoryMetric.h>

namespace GSLAM{

class Evaluate: public Application{
public:
    Evaluate():Application("evaluate"){}

    ~Evaluate(){
        running_=false;
        if(_workThread.joinable())
            _workThread.join();
    }

    void initMetrics(Messenger messenger,std::vector<ApplicationPtr>& metrics){
        metrics.clear();
        if(_config.GetInt("mem"))
            metrics.push_back(Application::create("metric_memory"));
        if(_config.GetInt("time"))
            metrics.push_back(Application::create("metric_time"));
        if(_config.GetInt("traj"))
            metrics.push_back(Application::create("metric_traj"));

        for(ApplicationPtr app:metrics){
            if(!app) continue;
            messenger.join(app->init(_config));
        }
    }


    virtual Messenger init(Svar conf)
    {
        _config=conf;
        std::string slam=conf.Arg<std::string>("slam","","The SLAM name if you wanna to run the slam outside.");
        std::string slams=conf.Arg<std::string>("slams","","The SLAM systems want to evaluate.");
        conf.Arg<std::string>("dataset","","The dataset used to perform evaluation.");
        bool mem=conf.Arg<bool>("mem"  ,true,"Evaluate the memory usage or not.");
        bool time=conf.Arg<bool>("time",true,"Evaluate the time used to process each frame or not.");
        bool traj=conf.Arg<bool>("traj",true,"Evaluate the trajectory or not.");

        initMetrics(_messenger,_metrics);

        if(conf.Get<bool>("help")){
            return _messenger;
        }

        if(slam.empty()&&slams.empty()){
            conf.GetInt("help")=1;
            running_=false;
            return _messenger;
        }

        if(!slam.empty()){
            return _messenger;
        }

        if(!slams.empty()){
            _workThread=std::thread(&Evaluate::evaluateThread,this);
        }

        return _messenger;
    }

    void evaluateThread()
    {
        VecParament<std::string> slams=_config.Get<VecParament<std::string> >("slams");
        for(std::string slam:slams.data){
            evaluateSLAM(slam);
        }
        running_=false;
    }

    void evaluateSLAM(std::string slam_name){
        if(!running_) return;
        MemoryMetric::instanceCPU().enable();
        auto usage=MemoryMetric::instanceCPU().usage();
        auto count=MemoryMetric::instanceCPU().count();

        {
            Messenger localMessenger;
            std::vector<ApplicationPtr> localMetrics;
            initMetrics(localMessenger,localMetrics);
            Dataset dataset(_config.GetString("dataset"));
            if(!dataset.isOpened()) return;

            ApplicationPtr slam=Application::create(slam_name);
            if(!slam) return;
            localMessenger.join(slam->init(_config));

            DatasetPlayer player(dataset);
            //        player.start();
        }
        auto leakUsage=MemoryMetric::instanceCPU().usage()-usage;
        auto leakCount=MemoryMetric::instanceCPU().count()-count;

        LOG(INFO)<<"SLAM "<<slam_name<<" memory leak check: "
                <<leakUsage<<" byte leaked with "<<leakCount<<" malloc.";
    }

    std::vector<ApplicationPtr> _metrics;
    Messenger _messenger;
    Svar      _config;
    std::thread _workThread;
};

REGISTER_APPLICATION(Evaluate);

}
