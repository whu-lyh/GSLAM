#include <GSLAM/core/GSLAM.h>
#include <GSLAM/core/Vocabulary.h>
#include <GSLAM/core/Estimator.h>
#include <GSLAM/core/Optimizer.h>
#include <string>
#include <complex>

/*
<%
cfg['compiler_args'] = ['-std=c++11','-O3']
setup_pybind11(cfg)
%>
*/
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace std;
using namespace pybind11::literals;

namespace py = pybind11;


namespace GSLAM{

template <typename T>
void addMessengerSupport(){
    SvarWithType<std::function<void(const Publisher&,py::object)> >::instance()
            .insert(typeid(T).name(),[](const Publisher& pub,py::object obj){
        T* message = new T(obj.cast<T>());
        std::shared_ptr<T> msg(message);
        pub.publish(msg);
    });

    SvarWithType<std::function<void(py::object,const std::shared_ptr<void>&)> >::instance()
            .insert(typeid(T).name(),[](py::object cbk,const std::shared_ptr<void>& obj){
        std::shared_ptr<T> msg=*(const std::shared_ptr<T>*)&obj;
        return cbk(*msg);
    });
}

template <typename T>
void addMessengerSupportPtr(){
    SvarWithType<std::function<void(const Publisher&,py::object)> >::instance()
            .insert(typeid(T).name(),[](const Publisher& pub,py::object obj){
        py::detail::type_caster<std::shared_ptr<T>> sharedObj;
        if(!sharedObj.load(obj,true)){
            return;
        }

        pub.publish((std::shared_ptr<T>)sharedObj);
    });

    SvarWithType<std::function<void(py::object,const std::shared_ptr<void>&)> >::instance()
            .insert(typeid(T).name(),[](py::object cbk,const std::shared_ptr<void>& obj){
        std::shared_ptr<T> msg=*(const std::shared_ptr<T>*)&obj;
        return cbk(msg);
    });
}

const std::type_info* getTypeInfo(PyTypeObject* obj){
    if(!obj){
        return nullptr;
    }
    static std::map<std::string,const std::type_info*> pythonTypes={
        {std::string("int"),&typeid(int)},
        {std::string("long"),&typeid(long)},
        {std::string("float"),&typeid(double)},
        {std::string("bool"),&typeid(bool)},
        {std::string("str"),&typeid(std::string)},
        {std::string("complex"),&typeid(std::complex<double>),
        {std::string("list"),&typeid(GSLAM::Json)},
        {std::string("tuple"),&typeid(GSLAM::Json)},
        {std::string("set"),&typeid(GSLAM::Json)},
        {std::string("dict"),&typeid(GSLAM::Json)}}
    };

    auto it=pythonTypes.find(obj->tp_name);
    if(it!=pythonTypes.end()){
        return it->second;
    }

    py::detail::type_info* info = py::detail::get_type_info(obj);
    if(info) return info->cpptype;
    LOG(ERROR)<<(std::string)py::str(py::handle((PyObject*)obj).get_type())
             <<" is not a python class.";
    return nullptr;
}

class PyGObjectHandle: public GObjectHandle{
public:
    virtual void handle(const GObjectPtr &obj)override{
        /* Acquire GIL before calling Python code */
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(
                    void, /* Return type */
                    GObjectHandle,      /* Parent class */
                    handle,          /* Name of function in C++ (must match Python name) */
                    obj      /* Argument(s) */
                );
    }
};

class PyDataset: public Dataset{
public:
    PyDataset(){}
    PyDataset(const std::string& dataset):Dataset(dataset){}
    virtual std::string type() const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(std::string,Dataset,type,);
    }
    virtual bool        isOpened(){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,Dataset,isOpened,);
    }
    virtual FramePtr    grabFrame(){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(FramePtr,Dataset,grabFrame,);
    }

    virtual bool open(const std::string& dataset){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,Dataset,open,dataset);
    }
};

class PyMap: public Map{
public:
    virtual std::string type()const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(std::string,Map,type,);
    }

    /// MapFrame & MapPoint interface
    virtual bool insertMapPoint(const PointPtr& point){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,Map,insertMapPoint,point);
    }
    virtual bool insertMapFrame(const FramePtr& frame){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,Map,insertMapFrame,frame);
    }
    virtual bool eraseMapPoint(const PointID& pointId){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,Map,eraseMapPoint,pointId);
    }
    virtual bool eraseMapFrame(const FrameID& frameId){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,Map,eraseMapFrame,frameId);
    }
    virtual void clear(){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(void,Map,clear,);
    }

    virtual std::size_t frameNum()const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(std::size_t,Map,frameNum,);
    }
    virtual std::size_t pointNum()const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(std::size_t,Map,pointNum,);
    }

    virtual FramePtr getFrame(const FrameID& id)const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(FramePtr,Map,getFrame,id);
    }
    virtual PointPtr getPoint(const PointID& id)const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(PointPtr,Map,getPoint,id);
    }

    virtual FrameArray  getFrames()const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(FrameArray,Map,getFrames,);
    }
    virtual PointArray  getPoints()const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(PointArray,Map,getPoints,);
    }
    /// Save or load the map from/to the file
    virtual bool save(std::string path)const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,Map,save,path);
    }
    virtual bool load(std::string path){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,Map,load,path);
    }
};

class PySLAM: public SLAM{
public:
    virtual std::string type()const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(std::string,PySLAM,type,);
    }
    virtual bool valid()const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,PySLAM,valid,);
    }
    virtual bool isDrawable()const{
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,PySLAM,isDrawable,);
    }

    virtual bool    setSvar(Svar& var){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,PySLAM,setSvar,var);
    }
    virtual bool    setCallback(GObjectHandle* cbk){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,PySLAM,setCallback,cbk);
    }
    virtual bool    track(FramePtr& frame){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,PySLAM,track,frame);
    }
    virtual bool    finalize(){
        py::gil_scoped_acquire acquire;
        PYBIND11_OVERLOAD(bool,PySLAM,finalize,);
    }
};

PYBIND11_MODULE(gslam,m) {
    PyEval_InitThreads();
    m.doc()="This is the python APIs for GSLAM "+string(GSLAM_VERSION_STR)
            +"(https://github.com/zdzhaoyong/GSLAM)";

    py::class_<Point2d>(m,"Point2d")
            .def(py::init<>())
            .def(py::init<double,double>())
            .def("dot",&Point2d::dot)
            .def("norm",&Point2d::norm)
            .def("at",&Point2d::at)
            .def("__repr__",&Point2d::toString)
            .def("__str__",&Point2d::toString)
            .def("__add__",&Point2d::add)
            .def("__sub__",&Point2d::sub)
            .def("__mul__",&Point2d::mul)
            .def("__div__",&Point2d::div)
            .def_readwrite("x", &Point2d::x)
            .def_readwrite("y", &Point2d::y);

    py::class_<Point3d>(m,"Point3d")
            .def(py::init<>())
            .def(py::init<double,double,double>())
            .def("dot",&Point3d::dot)
            .def("norm",&Point3d::norm)
            .def("at",&Point3d::at)
            .def("__repr__",&Point3d::toString)
            .def("__str__",&Point3d::toString)
            .def("__add__",&Point3d::add)
            .def("__sub__",&Point3d::sub)
            .def("__mul__",&Point3d::mul)
            .def("__div__",&Point3d::div)
            .def_readwrite("x", &Point3d::x)
            .def_readwrite("y", &Point3d::y)
            .def_readwrite("z", &Point3d::z);


    py::class_<Point3ub>(m,"Point3ub")
            .def(py::init<>())
            .def(py::init<uchar,uchar,uchar>())
            .def("dot",&Point3ub::dot)
            .def("norm",&Point3ub::norm)
            .def("at",&Point3ub::at)
            .def("__repr__",&Point3ub::toString)
            .def("__str__",&Point3ub::toString)
            .def("__add__",&Point3ub::add)
            .def("__sub__",&Point3ub::sub)
            .def("__mul__",&Point3ub::mul)
            .def("__div__",&Point3ub::div)
            .def_readwrite("x", &Point3ub::x)
            .def_readwrite("y", &Point3ub::y)
            .def_readwrite("z", &Point3ub::z);


    py::class_<Camera>(m,"Camera")
            .def(py::init<>())
            .def(py::init<std::vector<double> >())
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

    py::class_<SO3>(m,"SO3")
            .def(py::init<>())
            .def(py::init<double,double,double,double>())
            .def(py::init<const Point3d&,double>())
            .def(py::init<const double*>())
            .def("log",&SO3::log)
            .def_static("exp",&SO3::exp<double>)
            .def("normalise",&SO3::normalise)
            .def("getMatrix",&SO3::getMatrix)
            .def("__mul__",&SO3::mul)
            .def("trans",&SO3::trans)
            .def("inverse",&SO3::inv)
            .def("__repr__",&SO3::toString)
            .def_readwrite("x", &SO3::x)
            .def_readwrite("y", &SO3::y)
            .def_readwrite("z", &SO3::z)
            .def_readwrite("w", &SO3::w)
            ;

    py::class_<SE3>(m,"SE3")
            .def(py::init<>())
            .def(py::init<const SO3&,const Point3d&>())
            .def("log",&SE3::log)
            .def("inverse",&SE3::inverse)
            .def_static("exp",&SE3::exp<double>)
            .def("__mul__",&SE3::mul)
            .def("trans",&SE3::trans)
            .def("toString",&SE3::toString)
            .def("__repr__",&SE3::toString)
            .def_property("translation", &SE3::getTranslation, &SE3::setTranslation)
            .def_property("rotation", &SE3::getRotation, &SE3::setRotation)
            ;

    py::class_<SIM3>(m,"SIM3")
            .def(py::init<>())
            .def(py::init<const SE3&,const double&>())
//            .def_property("translation", &SE3::getTranslation, &SE3::setTranslation)
//            .def_property("rotation", &SE3::getRotation, &SE3::setRotation)
            ;


    py::class_<GImage>(m,"GImage",py::buffer_protocol())
            .def(py::init<>())
            .def(py::init<int,int,int,uchar*,bool>())
            .def("empty",&GImage::empty)
            .def_readonly("data",&GImage::data)
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
            .def_buffer([](GImage &m) -> py::buffer_info {
                return py::buffer_info(
                    m.data,                                  /* Pointer to buffer */
                    m.elemSize1(),                           /* Size of one scalar */
                    std::string(1,"BbHhifdd"[m.type()&0x7]), /* Python struct-style format descriptor */
                    3,                     /* Number of dimensions */
                    { m.rows, m.cols, m.channels()},                  /* Buffer dimensions */
                    { m.elemSize() * m.cols,
                      m.elemSize(), m.elemSize1() }/* Strides (in bytes) for each index */
                );
             })
            .def(py::init([](py::buffer b) {
                /* Request a buffer descriptor from Python */
                py::buffer_info info = b.request();
                auto idx=std::string("BbHhifd").find(info.format);
                if(idx==std::string::npos)
                     throw std::runtime_error("Incompatible format: expected GImage formats!");

                if (info.ndim > 3)
                    throw std::runtime_error("Incompatible GImage dimension!");

                     if(info.ndim == 1)
                          return GImage(info.shape[0],1,((idx&0x7)+((1-1)<<3)),(uchar*)info.ptr,true);
                     if(info.ndim == 2)
                          return GImage(info.shape[0],info.shape[1],((idx&0x7)+((1-1)<<3)),(uchar*)info.ptr,true);
                     if(info.ndim == 3)
                          return GImage(info.shape[0],info.shape[1],((idx&0x7)+((info.shape[2]-1)<<3)),(uchar*)info.ptr,true);

                     return GImage();
            }));

    py::class_<TicToc>(m,"TicToc")
            .def(py::init<>())
            .def("timestamp",&TicToc::timestamp)
            .def("tic",&TicToc::Tic)
            .def("toc",&TicToc::Toc)
            ;

    py::class_<Timer,TicToc>(m,"Timer")
            .def(py::init<bool>())
            .def("enter",&Timer::enter)
            .def("leave",&Timer::leave)
            .def_static("instance",&Timer::instance, py::return_value_policy::reference)
            .def("disable",&Timer::disable)
            .def("enable",&Timer::enable)
            .def("getMeanTime",&Timer::getMeanTime)
            .def("getStatsAsText",&Timer::getStatsAsText)
            .def("dumpAllStats",&Timer::dumpAllStats)
            ;

    py::class_<Svar>(m,"Svar")
            .def(py::init<>())
            .def_static("instance",&Svar::instance, py::return_value_policy::reference)
            .def_static("singleton",&Svar::instance, py::return_value_policy::reference)
            .def("insert",&Svar::insert,py::arg("name")="",
                 py::arg("var")="",py::arg("overwrite") = true)
//            .def("expandVal",&Svar::expandVal)
//            .def("setvar",&Svar::setvar)
            .def("getvar",&Svar::getvar)
            .def("parseLine",&Svar::ParseLine,"s"_a="","bSilentFailure"_a=false)
            .def("parseStream",&Svar::ParseStream)
            .def("parseFile",&Svar::ParseFile)
//            .def("parseMain",&Svar::ParseMain) // This have some problem
            .def("exist",&Svar::exist)
            .def("getInt",&Svar::GetInt, py::return_value_policy::reference,
                 py::arg("name")="",py::arg("def") = 0)
            .def("getDouble",&Svar::GetDouble, py::return_value_policy::reference,
                 py::arg("name")="",py::arg("def") = 0)
            .def("getString",&Svar::GetString, py::return_value_policy::reference,
                 py::arg("name")="",py::arg("def") = "")
            .def("getPointer",&Svar::GetPointer, py::return_value_policy::reference,
                 py::arg("name")="",py::arg("def") = nullptr)
            .def("erase",&Svar::erase)
            .def("update",&Svar::update)
            .def("get_data",&Svar::get_data)
            .def("clear",&Svar::clear)
            .def("clearAll",&Svar::clearAll)
            .def("getStatsAsText",&Svar::getStatsAsText)
            .def("dumpAllVars",&Svar::dumpAllVars)
            .def("save2file",&Svar::save2file)
            ;

    py::class_<GObject,std::shared_ptr<GObject> >(m,"GObject")
            .def(py::init<>())
            .def("type",&GObject::type)
            .def("call",&GObject::call)
            .def("draw",&GObject::draw)
            ;

    py::class_<GObjectHandle,PyGObjectHandle>(m,"GObjectHandle")
            .def(py::init<>())
            .def("handle",(void (GObjectHandle::*)(const GObjectPtr&)) &GObjectHandle::handle)
            ;

    py::class_<MapPoint,GObject,std::shared_ptr<MapPoint> >(m,"MapPoint")
            .def(py::init<const PointID&,const Point3d&>())
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
            .def_property("pose", &MapPoint::getPose, &MapPoint::setPose)
            .def_property("normal", &MapPoint::getNormal, &MapPoint::setNormal)
            .def_property("color", &MapPoint::getColor, &MapPoint::setColor)
            .def_property("descriptor", &MapPoint::getDescriptor, &MapPoint::setDescriptor)
            ;


    py::class_<FrameConnection,GObject,std::shared_ptr<FrameConnection> >(m,"FrameConnection")
            .def(py::init<>())
            .def("matchesNum",&FrameConnection::matchesNum)
            .def("getInformation",&FrameConnection::getInformation)
            .def("setMatches",&FrameConnection::setMatches)
            .def("setInformation",&FrameConnection::setInformation)
            .def("getMatches",(std::vector<std::pair<int,int> >(FrameConnection::*)())&FrameConnection::getMatches)
            ;

    py::class_<MapFrame,GObject,std::shared_ptr<MapFrame>>(m,"MapFrame")
            .def(py::init<const FrameID&,const double&>())
            .def("id",&MapFrame::id)
            .def("timestamp",&MapFrame::timestamp)
            .def("setPose",(void (MapFrame::*)(const SE3&))&MapFrame::setPose)
            .def("setPoseSim3",(void (MapFrame::*)(const SIM3&))&MapFrame::setPose)
            .def("getPose",(SE3(MapFrame::*)()const)&MapFrame::getPose)
            .def("getPoseScale",&MapFrame::getPoseScale)
            .def("cameraNum",&MapFrame::cameraNum)
            .def("getCameraPose",&MapFrame::getCameraPose)
            .def("imageChannels",&MapFrame::imageChannels)
            .def("getCamera",&MapFrame::getCamera,py::arg("idx") = 0)
            .def("getImage",&MapFrame::getImage,py::arg("idx") = 0,py::arg("channalMask")=0)
            .def("setImage",&MapFrame::setImage,py::arg("img") = GImage(),py::arg("idx") = 0,py::arg("channalMask")=0)
            .def("setCamera",&MapFrame::setCamera,py::arg("camera")=Camera(),py::arg("idx") = 0)
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
            .def("channelTypeString",&MapFrame::channelTypeString)
            .def("channelString",&MapFrame::channelString)
            ;

    py::class_<Dataset,PyDataset,GObject,SPtr<Dataset> >(m,"Dataset")
            .def(py::init<>())
            .def(py::init<const std::string&>())
            .def("name",&Dataset::name)
            .def("type",&Dataset::type)
            .def("isOpened",&Dataset::isOpened)
            .def("grabFrame",&Dataset::grabFrame)
            .def("open",&Dataset::open)
            ;


    py::class_<Map,PyMap,GObject,SPtr<Map> >(m,"Map")
            .def(py::init<>())
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

    py::class_<HashMap,Map,SPtr<HashMap> >(m,"HashMap")
            .def(py::init<>())
            ;

    py::class_<SLAM,PySLAM,GObject,SPtr<SLAM> >(m,"SLAM")
            .def(py::init<>())
            .def("valid",&SLAM::valid)
            .def("isDrawable",&SLAM::isDrawable)
            .def("setMap",&SLAM::setMap)
            .def("getMap",&SLAM::getMap)
            .def("setSvar",&SLAM::setSvar)
            .def("setCallback",&SLAM::setCallback)
            .def("track",&SLAM::track)
            .def("feed",&SLAM::feed)
            .def("finalize",&SLAM::finalize)
            .def_static("create",&SLAM::create)
            ;

    py::class_<Vocabulary,SPtr<Vocabulary> >(m,"Vocabulary")
            .def(py::init<const std::string &>())
            .def_static("create",&Vocabulary::create)
            .def("save",&Vocabulary::save)
            .def("load",(bool(Vocabulary::*)(const std::string &))&Vocabulary::load)
            .def("size",&Vocabulary::size)
            .def("empty",&Vocabulary::empty)
            .def("clear",&Vocabulary::clear)
            .def("transformImage",(void (Vocabulary::*)(const TinyMat&,
                 BowVector &, FeatureVector &, int)const)&Vocabulary::transform)
            .def("transformFeature",(void (Vocabulary::*)(const TinyMat &,
                 WordId &, WordValue &, NodeId*, int) const)&Vocabulary::transform)
            .def("getBranchingFactor",&Vocabulary::getBranchingFactor)
            .def("getDepthLevels",&Vocabulary::getDepthLevels)
            .def("getWord",&Vocabulary::getWord)
            .def("getWordWeight",&Vocabulary::getWordWeight)
            .def("getWeightingType",&Vocabulary::getWeightingType)
            .def("getScoringType",&Vocabulary::getScoringType)
            .def("setWeightingType",&Vocabulary::setWeightingType)
            .def("getDescritorSize",&Vocabulary::getDescritorSize)
            .def("getDescritorType",&Vocabulary::getDescritorType)
            .def_static("meanValue",&Vocabulary::meanValue)
            .def_static("distance",&Vocabulary::distance)
            ;

    py::class_<FileResource>(m,"FileResource")
            .def_static("toHex",&FileResource::toHex)
            .def_static("exportResourceFile",&FileResource::exportResourceFile)
            .def_static("Register",&FileResource::Register)
            .def_static("getResource",&FileResource::getResource)
            .def_static("saveResource2File",&FileResource::saveResource2File)
            ;

    py::class_<Undistorter>(m,"Undistorter")
            .def(py::init<Camera, Camera>())
            .def("undistort",&Undistorter::undistort)
            .def("undistortFast",&Undistorter::undistortFast)
            .def("cameraIn",&Undistorter::cameraIn)
            .def("cameraOut",&Undistorter::cameraOut)
            .def("prepareReMap",&Undistorter::prepareReMap)
            .def("valid",&Undistorter::valid)
            ;

    py::class_<TileBase, GObject, TilePtr >(m,"Tile")
            .def(py::init<>())
            .def("type",&TileBase::type)
            .def("getTileImage",&TileBase::getTileImage)
            .def("getTileHeight",&TileBase::getTileHeight)
            .def("getTilePosition",&TileBase::getTilePosition)
            .def("getTimeStamp",&TileBase::getTimeStamp)
            .def("memSizeInBytes",&TileBase::memSizeInBytes)
            .def("toStream",&TileBase::toStream)
            .def("fromStream",&TileBase::fromStream)
            ;

    py::class_<TileManager, GObject, TileManagerPtr >(m,"TileManager")
            .def("type",&TileManager::type)
            .def("getTile",&TileManager::getTile)
            .def("setTile",&TileManager::setTile)
            .def("maxZoomLevel",&TileManager::maxZoomLevel)
            .def("minZoomLevel",&TileManager::minZoomLevel)
            .def("save",&TileManager::save)
            ;

    py::class_<Messenger>(m,"Messenger")
            .def(py::init<>())
            .def_static("singleton",&Messenger::instance)
            .def("findSubscriber",&Messenger::findSubscriber)
            .def("findPublisher",&Messenger::findPublisher)
            .def("getPublishers",&Messenger::getPublishers)
            .def("getSubscribers",&Messenger::getSubscribers)
            .def("introduction",&Messenger::introduction)
            .def("accept",(void (Messenger::*)(Messenger))&Messenger::join)
            .def("advertise",[](Messenger messenger, py::object py_class,
                 const std::string& topic, uint32_t queue_size = 0,
                 bool latch = false){
        auto info=getTypeInfo((PyTypeObject*)py_class.ptr());
        if(!info) return Publisher();
        Publisher pub(topic,info->name(),queue_size);
        messenger.join(pub);
        return pub;
    })
    .def("subscribe",[](Messenger messenger, py::object py_class,
         const std::string& topic, uint32_t queue_size,
         py::object callback){

        auto info=getTypeInfo((PyTypeObject*)py_class.ptr());
        if(!info) return Subscriber();
        auto transFunc=SvarWithType<std::function<void(py::object,const std::shared_ptr<void>&)> >::instance()
                .get_var(info->name(),nullptr);
        Subscriber sub(topic,info->name(),[transFunc,callback](const std::shared_ptr<void>& msg){
            py::gil_scoped_acquire acc;
            transFunc(callback,msg);
        },queue_size);
        messenger.join(sub);
        return sub;
    });

    py::class_<Publisher>(m,"Publisher")
            .def("shutdown",&Publisher::shutdown)
            .def("getTopic",&Publisher::getTopic)
            .def("getTypeName",&Publisher::getTypeName)
            .def("getNumSubscribers",&Publisher::getNumSubscribers)
            .def("isLatched",&Publisher::isLatched)
            .def("publish",[](const Publisher& pub,py::object msg){
        if(!pub) return;
        auto pubFunc=SvarWithType<std::function<void(const Publisher&,py::object)> >::instance()
                .get_var(pub.getTypeName(),nullptr);
        if(!pubFunc){
            LOG(ERROR)<<"Message type "<<Messenger::translate(pub.getTypeName())<<" are not registed.\n";
            return;
        }
        pubFunc(pub,msg);
    });

    py::class_<Subscriber>(m,"Subscriber")
            .def("shutdown",&Subscriber::shutdown)
            .def("getTopic",&Subscriber::getTopic)
            .def("getTypeName",&Subscriber::getTypeName)
            .def("getNumPublishers",&Subscriber::getNumPublishers);


    py::class_<Application,ApplicationPtr>(m,"Application")
            .def("isRunning",&Application::isRunning)
            .def("name",&Application::name)
            .def("gslam_version",&Application::gslam_version)
            .def("init",&Application::init)
            .def_static("create",&Application::create)
            ;

    py::class_<DatasetPlayer>(m,"DatasetPlayer")
            .def(py::init<Dataset,Messenger,Svar>(),
                 py::arg("dataset")=Dataset(),
                 py::arg("messenger") = Messenger::instance(),
                 py::arg("config")=Svar::instance())
            .def("open",&DatasetPlayer::open)
            .def("play",&DatasetPlayer::play)
            .def("start",&DatasetPlayer::start);

    addMessengerSupport<int>();
    addMessengerSupport<bool>();
    addMessengerSupport<long>();
    addMessengerSupport<double>();
    addMessengerSupport<std::string>();
    addMessengerSupport<Point3d>();
    addMessengerSupport<GImage>();
    addMessengerSupport<Publisher>();
    addMessengerSupport<Json>();

    addMessengerSupportPtr<MapFrame>();
    addMessengerSupportPtr<Map>();
}


}
