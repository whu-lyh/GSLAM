#include <list>

#include <GSLAM/core/Application.h>
#include <GSLAM/core/MemoryMetric.inc>
#include <GSLAM/core/Evaluation.h>

namespace GSLAM {

class MetricMemory: public Application
{
public:
    MetricMemory():Application("metric_memory"){
    }
    ~MetricMemory(){
        _sub.shutdown();
        _pub.shutdown();
        if(_data.empty()) return;
        std::ofstream ofs(_slam_name+"_metric_memory.txt");
        for(std::tuple<FrameID,size_t,size_t> it:_data){
            ofs<<std::get<0>(it)<<" "
              <<std::get<1>(it)<<" "
             <<std::get<1>(it)<<"\n";
        }
    }

    virtual Messenger init(Svar conf)
    {
        _slam_name=conf.GetString("slam","slam");
        if(_slam_name.empty()) _slam_name="<slam_name>";
        LOG(INFO)<<"MetricMemory started for slam "<<_slam_name;
        _sub=_messenger.subscribe(_slam_name+"/curframe",0,
                                  &MetricMemory::endFrame,this);
        _pub=_messenger.advertise<std::tuple<FrameID,size_t,size_t> >("evaluate/memory",0);
        return _messenger;
    }

    void endFrame(const MapFrame& fr)
    {
        std::tuple<FrameID,size_t,size_t> status(fr.id(),
                                       MemoryMetric::instanceCPU().usage(),
                                       MemoryMetric::instanceCPU().count());
        _data.push_back(status);
        _pub.publish(status);
    }


    std::string _slam_name;
    Messenger _messenger;
    std::list<std::tuple<FrameID,size_t,size_t> > _data;
    Subscriber                           _sub;
    Publisher                            _pub;
};

REGISTER_BUILDIN_APPLICATION(MetricMemory,metric_memory);
}
