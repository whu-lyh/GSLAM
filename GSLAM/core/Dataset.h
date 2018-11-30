#ifndef GSLAM_DATASET_H
#define GSLAM_DATASET_H
#include "GSLAM.h"
#include "Svar.h"
#include "SharedLibrary.h"
#include "Messenger.h"
#include "Timer.h"

namespace GSLAM{

#define REGISTER_DATASET(D,E) \
    extern "C" GSLAM::Dataset* createDataset##E(){ return new D();}\
    class D##E##_Register{ \
    public: D##E##_Register(){\
    GSLAM::DatasetFactory::instance()._ext2creator.insert(#E,createDataset##E);\
}}D##E##_instance;

/// create
class Dataset;
typedef SPtr<Dataset> DatasetPtr;
typedef Dataset* (*funcCreateDataset)();

// A dataset configuration file : DatasetName.DatasetType --eg. Sequence1.kitti desk.tumrgbd
class Dataset : public GSLAM::GObject
{
public:
    typedef std::vector<std::string> StrVec;
    Dataset():_name("Untitled"){}
    Dataset(const std::string& dataset){open(dataset);}
    virtual ~Dataset(){}

    std::string         name() const{if(_impl) return _impl->type();else return _name;}
    virtual std::string type() const{if(_impl) return _impl->type();else return "Dataset";}
    virtual bool        isOpened(){if(_impl) return _impl->isOpened();else return false;}
    virtual FramePtr    grabFrame(){if(_impl) return _impl->grabFrame();else return FramePtr();}

    virtual bool        open(const std::string& dataset);
    virtual bool        isLive()const{
        if(_impl) return _impl->isLive();
        return false;
    }
protected:
    std::string _name;
    DatasetPtr  _impl;
};


class DatasetPlayer{
public:
    virtual ~DatasetPlayer(){stop();}
    enum Status{
        READY,PLAYING,PAUSING,PAUSED,FINISHING,FINISHED
    };
    DatasetPlayer(Dataset dataset=Dataset())
        :_dataset(dataset),_status(READY){
        Messenger msg=Messenger::instance();
        _pub_dataset_status=msg.advertise<int>("dataset/status",100);
        _pub_images=msg.advertise<MapFrame>("images",0);
        _pub_imu=msg.advertise<MapFrame>("imu",0);
    }

    virtual void        start(){
        if(!_dataset.isOpened()) return;
        if(_dataset.isLive()) return ;
        if(_status==READY){
            _status=PLAYING;
            _playThread=std::thread(&DatasetPlayer::playthread,this);
        }
        if(_status==PAUSED){
            _status=PLAYING;
            _pub_dataset_status.publish<int>(_status);
        }
    }

    virtual void        pause(){
        _status=PAUSING;
    }

    virtual void        stop(){
        if(_status==READY||_status==FINISHED) return;
        _status=FINISHING;
        _playThread.join();
        _status=FINISHED;
        _dataset=Dataset();
        _pub_dataset_status.publish<int>(FINISHED);
    }

    virtual void        step(){
        if(_status==READY){
            _status=PAUSED;
            _playThread=std::thread(&DatasetPlayer::playthread,this);
        }
        if(_status==PAUSED){
            auto frame=_dataset.grabFrame();
            if(!frame){
                _pub_dataset_status.publish<int>(FINISHING);
                return;
            }
            if(frame->cameraNum()>0)
                _pub_images.publish(frame);
            else if(frame->getIMUNum()>0)
                _pub_imu.publish(frame);
        }
    }

    virtual void        playControl(const std::string& cmd){
        if(cmd=="Dataset Start") start();
        else if(cmd=="Dataset Pause") pause();
        else if(cmd=="Dataset Stop") stop();
        else if(cmd=="Dataset Step") step();
        else if(cmd.find("Dataset Open")==0)
            open(cmd.substr(13));
    }

    virtual bool open(const std::string& dataset)
    {
        stop();
        _dataset=Dataset(dataset);
        if(!_dataset.isOpened()) return false;
        _status=READY;
        _pub_dataset_status.publish<int>(READY);
        if(svar.GetInt("AutoStart")) start();
    }

    virtual void playthread(){
        double speed=svar.GetDouble("PlaySpeed",1.);
        double startTime=-1;
        double& playSpeedWarningTime=svar.GetDouble("PlaySpeedWarning",5);
        GSLAM::TicToc tictoc,tictocWarning;
        GSLAM::FramePtr frame;
        _pub_dataset_status.publish<int>(_status);
        while (_status!=FINISHED) {
            switch (_status) {
            case FINISHING:
            {
                return;
            }
                break;
            case PAUSING:
            {
                _status=PAUSED;
                _pub_dataset_status.publish<int>(PAUSED);
            }
                break;
            case PAUSED:
            {
                GSLAM::Rate::sleep(0.001);
                startTime=-1;
            }
                break;
            case PLAYING:
            {
                frame=_dataset.grabFrame();
                if(!frame){
                    _pub_dataset_status.publish<int>(FINISHING);
                    return;
                }
                if(frame->cameraNum()>0)
                    _pub_images.publish(frame);
                else if(frame->getIMUNum()>0)
                    _pub_imu.publish(frame);
                if(startTime<0){
                    startTime=frame->timestamp();
                    tictoc.Tic();
                }
                else{
                    double shouldSleep=(frame->timestamp()-startTime)/speed-tictoc.Tac();
                    if(shouldSleep<-2){
                        if(tictocWarning.Tac()>playSpeedWarningTime)// Don't bother
                        {
                            LOG(WARNING)<<"Play speed not realtime! Speed approximate "
                                       <<(frame->timestamp()-startTime)/tictoc.Tac();
                            tictocWarning.Tic();
                        }
                    }
                    else GSLAM::Rate::sleep(shouldSleep);
                }
            }
                break;
            default:
                break;
            }
        }
    }

    Publisher   _pub_dataset_status,_pub_images,_pub_imu;
    Dataset     _dataset;
    std::thread _playThread;
    int         _status;
};

class DatasetFactory
{
public:
    typedef SPtr<Dataset> DatasetPtr;

    static DatasetFactory& instance(){
        static SPtr<DatasetFactory> inst(new DatasetFactory());
        return *inst;
    }

    static DatasetPtr create(std::string dataset);

    SvarWithType<funcCreateDataset>        _ext2creator;
};

inline bool Dataset::open(const std::string& dataset){
    DatasetPtr impl=DatasetFactory::create(dataset);
    if(impl) {_impl=impl;return _impl->open(dataset);}
    return false;
}

inline DatasetPtr DatasetFactory::create(std::string dataset)
{
    std::string extension;
    // The input path could be dataset configuration file or just a folder path
    size_t dotPosition=dataset.find_last_of('.');
    if(dotPosition!=std::string::npos)// call by extension
    {
        extension=dataset.substr(dotPosition+1);
    }
    if(extension.empty()) return DatasetPtr();

    if(!instance()._ext2creator.exist(extension))
    {
        SharedLibraryPtr plugin=Registry::get("libgslamDB_"+extension);
        if(!plugin.get()) return DatasetPtr();
        funcCreateDataset createFunc=(funcCreateDataset)plugin->getSymbol("createDataset"+extension);
        if(createFunc) return DatasetPtr(createFunc());
    }

    if(!instance()._ext2creator.exist(extension)) return DatasetPtr();
    funcCreateDataset createFunc=instance()._ext2creator.get_var(extension,NULL);
    if(!createFunc) return DatasetPtr();

    return DatasetPtr(createFunc());
}

}

#endif // VIDEOREADER_H
