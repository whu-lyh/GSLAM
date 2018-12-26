#ifndef GSLAM_JAVASCRIPT_CPP_H
#define GSLAM_JAVASCRIPT_CPP_H
#endif

#include <GSLAM/core/GSLAM.h>
#include "NodeBind.h"
#include "MessengerHelper.h"

namespace GSLAM {
using namespace std;
using nbind::class_;

NBIND_MODULE{
    addMessengerSupport<int>("Int");
    addMessengerSupport<long>("Long");
    addMessengerSupport<int>("int");
    addMessengerSupport<long>("long");
    addMessengerSupport<bool>("Boolean");
    addMessengerSupport<double>("Number");
    addMessengerSupport<std::string>("String");
    addMessengerSupportPtr<Point2i>("Point2i");
    addMessengerSupportPtr<GImage>("GImage");
    addMessengerSupportPtr<Publisher>("Publisher");
    addMessengerSupportPtr<MapFrame>("MapFrame");
    addMessengerSupport<Json>("Object");

    NBIND_FUNCTION(processAll);

    class_<Point2i>("Point2i")
            .constructor<>()
            .constructor<int,int>()
            .method("dot",&Point2i::dot)
            .method("norm",&Point2i::norm)
            .method("at",&Point2i::at)
            .method("toString",&Point2i::toString)
            .method("sub",&Point2i::sub)
            .method("mul",&Point2i::mul)
            .method("div",&Point2i::div)
            .property("x",&Point2i::getX,&Point2i::setX)
            .property("y",&Point2i::getY,&Point2i::setY);

    class_<Point2d>("Point2d")
            .constructor<>()
            .constructor<double,double>()
            .def("dot",&Point2d::dot)
            .def("norm",&Point2d::norm)
            .def("at",&Point2d::at)
            .def("toString",&Point2d::toString)
            .def("add",&Point2d::add)
            .def("sub",&Point2d::sub)
            .def("mul",&Point2d::mul)
            .def("div",&Point2d::div)
            .property("x",&Point2d::getX,&Point2d::setX)
            .property("y",&Point2d::getY,&Point2d::setY);

    class_<Point3d>("Point3d")
            .constructor<>()
            .constructor<double,double,double>()
            .def("dot",&Point3d::dot)
            .def("norm",&Point3d::norm)
            .def("at",&Point3d::at)
            .def("str",&Point3d::toString)
            .def("add",&Point3d::add)
            .def("sub",&Point3d::sub)
            .def("mul",&Point3d::mul)
            .def("div",&Point3d::div)
            .property("x",&Point3d::getX,&Point3d::setX)
            .property("y",&Point3d::getY,&Point3d::setY)
            .property("z",&Point3d::getZ,&Point3d::setZ);


    class_<Point3ub>("Point3ub")
            .constructor<>()
            .constructor<double,double,double>()
            .def("dot",&Point3ub::dot)
            .def("norm",&Point3ub::norm)
            .def("at",&Point3ub::at)
            .def("str",&Point3ub::toString)
            .def("add",&Point3ub::add)
            .def("sub",&Point3ub::sub)
            .def("mul",&Point3ub::mul)
            .def("div",&Point3ub::div)
            .property("x",&Point3ub::getX,&Point3ub::setX)
            .property("y",&Point3ub::getY,&Point3ub::setY)
            .property("z",&Point3ub::getZ,&Point3ub::setZ);


    class_<Camera>("Camera")
            .constructor<>()
            .constructor<std::vector<double> >()
            .def("CameraType",&Camera::CameraType)
            .def("info",&Camera::info)
            .def("isValid",&Camera::isValid)
            .def("width",&Camera::width)
            .def("height",&Camera::height)
            .def("getParameters",&Camera::getParameters)
            .def("estimatePinHoleCamera",&Camera::estimatePinHoleCamera)
            .def("Project",(Point2d (Camera::*)(const Point3d&)const) &Camera::Project,
                 "Project a point from camera coordinate to image coordinate")
            .def("UnProject",(Point3d (Camera::*)(const Point2d&)const) &Camera::UnProject,"");

    class_<SO3>("SO3")
            .constructor<>()
            .constructor<double,double,double,double>()
            .constructor<const Point3d&,double>()
            .constructor<const double*>()
            .def("log",&SO3::log)
            .def_static("exp",&SO3::exp<double>)
            .def("normalise",&SO3::normalise)
            .def("getMatrix",&SO3::getMatrix)
            .def("__mul__",&SO3::mul)
            .def("trans",&SO3::trans)
            .def("inverse",&SO3::inv)
            .def("__repr__",&SO3::toString)
            .property("x", &SO3::getX,&SO3::setX)
            .property("y", &SO3::getY,&SO3::setY)
            .property("z", &SO3::getZ,&SO3::setZ)
            .property("w", &SO3::getW,&SO3::setW)
            ;

    class_<SE3>("SE3")
            .constructor<>()
            .constructor<const SO3&,const Point3d&>()
            .def("log",&SE3::log)
            .def("inverse",&SE3::inverse)
            .def_static("exp",&SE3::exp<double>)
            .def("__mul__",&SE3::mul)
            .def("trans",&SE3::trans)
            .def("toString",&SE3::toString)
            .def("__repr__",&SE3::toString)
            .property("translation", &SE3::getTranslation, &SE3::setTranslation)
            .property("rotation", &SE3::getRotation, &SE3::setRotation)
            ;

    class_<SIM3>("SIM3")
            .constructor<>()
            .constructor<const SE3&,const double&>()
//            .def_property("translation", &SE3::getTranslation, &SE3::setTranslation)
//            .def_property("rotation", &SE3::getRotation, &SE3::setRotation)
            ;

    class_<GImage>("GImage")
            .constructor()
//            .constructor<int,int,int,uchar*,bool>()
            .def("empty",&GImage::empty)
            .def("data",[](const GImage& img){return img.data;})
            .def("elemSize",&GImage::elemSize)
            .def("elemSize1",&GImage::elemSize1)
            .def("channels",&GImage::channels)
            .def("type",&GImage::type)
            .def("total",&GImage::total)
            .def("clone",&GImage::clone)
            .def("row",&GImage::row)
            .def("width",&GImage::getWidth)
            .def("height",&GImage::getHeight)
            .def("__repr__",[](const GImage& img){
                return to_string(img.cols)+"x"
                        +to_string(img.rows)+"x"+to_string(img.channels());})
            ;

    class_<TicToc>("TicToc")
            .constructor<>()
            .def_static("timestamp",&TicToc::timestamp)
            .def("tic",&TicToc::Tic)
            .def("toc",&TicToc::Toc)
            ;

    class_<Timer>("Timer")
            .constructor<bool>()
            .def("enter",&Timer::enter)
            .def("leave",&Timer::leave)
            .def_static("instance",&Timer::instance)
            .def("disable",&Timer::disable)
            .def("enable",&Timer::enable)
            .def("getMeanTime",&Timer::getMeanTime)
            .def("getStatsAsText",&Timer::getStatsAsText)
            .def("dumpAllStats",&Timer::dumpAllStats)
            ;

    class_<Svar>("Svar")
            .constructor<>()
            .method("instance",&Svar::instance)
            .method("ParseLine",&Svar::ParseLine)
            .method("ParseStream",&Svar::ParseStream)
            .method("ParseFile",&Svar::ParseFile)
            .method("exist",&Svar::exist)
            .method("Set",&Svar::Set<std::string>)
            .method("GetInt",&Svar::GetInt)
            .method("GetDouble",&Svar::GetDouble)
            .method("GetString",&Svar::GetString)
            .method("erase",&Svar::erase)
            .method("clear",&Svar::clear)
            .method("getStatsAsText",&Svar::getStatsAsText)
            .method("dumpAllVars",&Svar::dumpAllVars)
            .method("save2file",&Svar::save2file)
            .method("help",&Svar::help)
            .method("loadJson",[](Svar& var,Json json){
        loadJSON(var,json);
    });

    class_<Messenger>("Messenger")
            .constructor<>()
            .def_static("singleton",&Messenger::instance)
            .def("findSubscriber",&Messenger::findSubscriber)
            .def("findPublisher",&Messenger::findPublisher)
            .def("getPublishers",&Messenger::getPublishers)
            .def("getSubscribers",&Messenger::getSubscribers)
            .def("introduction",&Messenger::introduction)
            .def("accept",(void (Messenger::*)(Messenger))&Messenger::join)
            .def("advertise",[](Messenger& messenger, nbind::WireType py_class,
                 const std::string& topic, uint32_t queue_size = 0,
                 bool latch = false){

        std::string class_name=getV8Type(py_class);
        if(class_name.empty()){
            LOG(ERROR)<<"Unable to subscribe .";
        }
        auto info=svar.Get<const std::type_info*>(class_name,nullptr);
        if(!info) {
            LOG(INFO)<<"Unable to advertise "<<class_name;
            return Publisher();
        }
        Publisher pub(topic,info->name(),queue_size);
        messenger.join(pub);
        return pub;
    })
    .def("subscribe",[](Messenger& messenger, nbind::WireType py_class,
         const std::string& topic, nbind::cbFunction callback){

        std::string class_name=getV8Type(py_class);
        if(class_name.empty()){
            LOG(ERROR)<<"Unable to subscribe .";
            return Subscriber();
        }
        auto info=svar.Get<const std::type_info*>(class_name,nullptr);
        if(!info) {
            LOG(INFO)<<"Unable to subscribe "<<class_name;
            return Subscriber();
        }
        auto transFunc=svar.Get<std::function<void(std::shared_ptr<nbind::cbFunction>,
                                                   const std::shared_ptr<void>&)> >(info->name(),nullptr);
        std::shared_ptr<nbind::cbFunction> cbk(new nbind::cbFunction(callback));
        Subscriber sub(topic,info->name(),[transFunc,cbk](const std::shared_ptr<void>& msg){
            transFunc(cbk,msg);
        });
        messenger.join(sub);
        return sub;
    });

    class_<Publisher>("Publisher")
            .constructor<>()
            .def("shutdown",&Publisher::shutdown)
            .def("getTopic",&Publisher::getTopic)
            .def("getTypeName",&Publisher::getTypeName)
            .def("getNumSubscribers",&Publisher::getNumSubscribers)
            .def("isLatched",&Publisher::isLatched)
            .def("publish",[](const Publisher& pub,nbind::WireType msg)->void{
        if(!pub) return;
//        DLOG(INFO)<<"Publishing "<<Messenger::translate(pub.getTypeName());

        auto pubFunc=svar.Get<std::function<void(const Publisher&,nbind::WireType)> >
                (pub.getTypeName(),nullptr);
        if(!pubFunc){
            LOG(ERROR)<<"Message type "<<Messenger::translate(pub.getTypeName())<<" are not registed.\n";
            return;
        }
        pubFunc(pub,msg);
    });

    class_<Subscriber>("Subscriber")
            .def("shutdown",&Subscriber::shutdown)
            .def("getTopic",&Subscriber::getTopic)
            .def("getTypeName",&Subscriber::getTypeName)
            .def("getNumPublishers",&Subscriber::getNumPublishers);


    class_<Application>("Application")
            .def("isRunning",&Application::isRunning)
            .def("name",&Application::name)
            .def("gslam_version",&Application::gslam_version)
            .def("init",&Application::init)
            .def_static("create",&Application::create)
            ;

    class_<MapPoint>("MapPoint")
            .constructor<const PointID&,const Point3d&>()
            .def("id",&MapPoint::id)
            .def("getPose",&MapPoint::getPose)
            .def("setPose",&MapPoint::setPose)
            .def("getNormal",&MapPoint::getNormal)
            .def("setNormal",&MapPoint::setNormal)
            .def("getColor",&MapPoint::setColor)
            .def("getDescriptor",&MapPoint::getDescriptor)
            .def("isPoseRelative",&MapPoint::isPoseRelative)
            .def("refKeyframeID",&MapPoint::refKeyframeID)
            .def("refKeyframe",&MapPoint::refKeyframe)
            .def("observationNum",&MapPoint::observationNum)
            .def("getObservations",(MapPointObsVec (MapPoint::*)()const) &MapPoint::getObservations)
            .def("eraseObservation",&MapPoint::eraseObservation)
            .def("clearObservation",&MapPoint::clearObservation)
            .property("pose", &MapPoint::getPose, &MapPoint::setPose)
            .property("normal", &MapPoint::getNormal, &MapPoint::setNormal)
            .property("color", &MapPoint::getColor, &MapPoint::setColor)
            .property("descriptor", &MapPoint::getDescriptor, &MapPoint::setDescriptor)
            ;


    class_<FrameConnection>("FrameConnection")
            .constructor<>()
            .def("matchesNum",&FrameConnection::matchesNum)
            .def("getInformation",&FrameConnection::getInformation)
            .def("setMatches",&FrameConnection::setMatches)
            .def("setInformation",&FrameConnection::setInformation)
            .def("getMatches",(std::vector<std::pair<int,int> >(FrameConnection::*)())&FrameConnection::getMatches)
            ;

    class_<MapFrame>("MapFrame")
            .constructor<const FrameID&,const double&>()
            .def("id",&MapFrame::id)
            .def("timestamp",&MapFrame::timestamp)
            .def("setPose",(void (MapFrame::*)(const SE3&))&MapFrame::setPose)
            .def("setPoseSim3",(void (MapFrame::*)(const SIM3&))&MapFrame::setPose)
            .def("getPose",(SE3(MapFrame::*)()const)&MapFrame::getPose)
            .def("getPoseScale",&MapFrame::getPoseScale)
            .def("cameraNum",&MapFrame::cameraNum)
            .def("getCameraPose",&MapFrame::getCameraPose)
            .def("imageChannels",&MapFrame::imageChannels)
            .def("getCamera",&MapFrame::getCamera)
            .def("getImage",&MapFrame::getImage)
            .def("setImage",&MapFrame::setImage)
            .def("setCamera",&MapFrame::setCamera)
            .def("getIMUNum",&MapFrame::getIMUNum)
            .def("getIMUPose",&MapFrame::getIMUPose)
            .def("getAcceleration",&MapFrame::getAcceleration)
            .def("getAngularVelocity",&MapFrame::getAngularVelocity)
            .def("getMagnetic",&MapFrame::getMagnetic)
            .def("getAccelerationNoise",&MapFrame::getAccelerationNoise)
            .def("getAngularVNoise",&MapFrame::getAngularVNoise)
            .def("getPitchYawRoll",&MapFrame::getPitchYawRoll)
            .def("getPYRSigma",&MapFrame::getPYRSigma)
            .def("getGPSNum",&MapFrame::getGPSNum)
            .def("getGPSPose",&MapFrame::getGPSPose)
            .def("getGPSLLA",&MapFrame::getGPSLLA)
            .def("getGPSLLASigma",&MapFrame::getGPSLLASigma)
            .def("getGPSECEF",&MapFrame::getGPSECEF)
            .def("getHeight2Ground",&MapFrame::getHeight2Ground)
            .def("getGPSECEF",&MapFrame::getGPSECEF)
            .def("getHeight2Ground",&MapFrame::getHeight2Ground)
            .def("keyPointNum",&MapFrame::keyPointNum)
            .def("setKeyPoints",&MapFrame::setKeyPoints)
            .def("getKeyPoints",(std::vector<KeyPoint>(MapFrame::*)() const)&MapFrame::getKeyPoints)
            .def("getKeyPointColor",&MapFrame::getKeyPointColor)
            .def("getKeyPointIDepthInfo",&MapFrame::getKeyPointIDepthInfo)
            .def("getKeyPointObserve",&MapFrame::getKeyPointObserve)
            .def("getDescriptor",&MapFrame::getDescriptor)
            .def("getBoWVector",(BowVector (MapFrame::*)()const)&MapFrame::getBoWVector)
            .def("getFeatureVector",(FeatureVector (MapFrame::*)()const)&MapFrame::getFeatureVector)
            .def("getFeaturesInArea",&MapFrame::getFeaturesInArea)
            .def("observationNum",&MapFrame::observationNum)
            .def("getObservations",(std::map<GSLAM::PointID,size_t>(MapFrame::*)()const)&MapFrame::getObservations)
            .def("addObservation",&MapFrame::addObservation)
            .def("eraseObservation",&MapFrame::eraseObservation)
            .def("clearObservations",&MapFrame::clearObservations)
            .def("getParent",&MapFrame::getParent)
            .def("getChild",&MapFrame::getChild)
            .def("getParents",(FrameConnectionMap (MapFrame::*)()const)&MapFrame::getParents)
            .def("getChildren",(FrameConnectionMap (MapFrame::*)()const)&MapFrame::getChildren)
            .def("addParent",&MapFrame::addParent)
            .def("addChildren",&MapFrame::addChildren)
            .def("eraseParent",&MapFrame::eraseParent)
            .def("eraseChild",&MapFrame::eraseChild)
            .def("clearParents",&MapFrame::clearParents)
            .def("clearChildren",&MapFrame::clearChildren)
            .def("getMedianDepth",&MapFrame::getMedianDepth)
            .def_static("channelTypeString",&MapFrame::channelTypeString)
            .def("channelString",&MapFrame::channelString)
            ;

    class_<Dataset>("Dataset")
            .constructor<>()
            .constructor<const std::string&>()
            .def("name",&Dataset::name)
            .def("type",&Dataset::type)
            .def("isOpened",&Dataset::isOpened)
            .def("grabFrame",&Dataset::grabFrame)
            .def("open",&Dataset::open)
            ;


    class_<Map>("Map")
            .constructor<>()
            .def("insertMapPoint",&Map::insertMapPoint)
            .def("insertMapFrame",&Map::insertMapFrame)
            .def("eraseMapPoint",&Map::eraseMapPoint)
            .def("eraseMapFrame",&Map::eraseMapFrame)
            .def("clear",&Map::clear)
            .def("frameNum",&Map::frameNum)
            .def("pointNum",&Map::pointNum)
            .def("getFrame",&Map::getFrame)
            .def("getPoint",&Map::getPoint)
            .def("getFrames",(FrameArray(Map::*)()const)&Map::getFrames)
            .def("getPoints",(PointArray(Map::*)()const)&Map::getPoints)
            .def("setLoopDetector",&Map::setLoopDetector)
            .def("getLoopDetector",&Map::getLoopDetector)
            .def("obtainCandidates",(LoopCandidates(Map::*)(const FramePtr&))&Map::obtainCandidates)
            .def("save",&Map::save)
            .def("load",&Map::load)
            .def("getPid",&Map::getPid)
            .def("getFid",&Map::getFid)
            ;

    class_<HashMap>("HashMap")
            .constructor<>()
            ;
}

}// end of namespace GSLAM
