#ifndef GSLAM_CORE_SLAM_H
#define GSLAM_CORE_SLAM_H
#include <vector>
#include <map>
#include "Map.h"

#define USE_GSLAM_PLUGIN(SLAMCLASS) extern "C"{\
    GSLAM::SLAMPtr createSLAMInstance(){return GSLAM::SLAMPtr(new SLAMCLASS());}}

namespace GSLAM {

class SLAM : public GObject
{
public:
    SLAM(){}
    virtual ~SLAM(){}
    virtual std::string type()const{return "InvalidSLAM";}
    virtual bool valid()const{return false;}
    virtual bool isDrawable()const{return false;}

    bool    setMap(const MapPtr& map);
    MapPtr  getMap()const;

    virtual bool    setSvar(Svar& var);
    virtual bool    setCallback(GObjectHandle* cbk){_handle=cbk;return true;}
    virtual bool    track(FramePtr& frame){return false;}
    virtual bool    finalize(){return false;}

    bool feed(FramePtr frame){return track(frame);}

    static SLAMPtr create(const std::string& slamPlugin);

protected:
    MapPtr          _curMap;
    mutable MutexRW _mutexMap;
    GObjectHandle*  _handle;
};


inline bool SLAM::setMap(const MapPtr& map)
{
    ReadMutex lock(_mutexMap);
    _curMap=map;
    return true;
}

inline MapPtr SLAM::getMap()const
{
    ReadMutex lock(_mutexMap);
    return _curMap;
}

inline bool SLAM::setSvar(Svar &var)
{
    svar=var;
    return true;
}

inline SLAMPtr SLAM::create(const std::string& slamPlugin){
    SharedLibraryPtr plugin=Registry::get(slamPlugin);
    if(!plugin) return SLAMPtr();
    funcCreateSLAMInstance createFunc=(funcCreateSLAMInstance)plugin->getSymbol("createSLAMInstance");
    if(!createFunc) return SLAMPtr();
    else return createFunc();
}
} // end of namespace GSLAM
#endif
