import gslam as gs

class DeepGSLAM(gs.SLAM):
    def type(self):
        return "DeepSLAM";
    def valid(self):
        return True;
    def isDrawable(self):
        return False;
    def setSvar(self,var):
        self.config=var;
    def setCallback(self,cbk):
        self.callback=cbk;
    def track(self,fr):
        image=fr.getImage(0)
        cam  =fr.getCamera(0);
        print("Processing frame ",fr.id(),"time:",fr.timestamp())
        #fr.setImage(0,depth)
        #fr.setPose(gs.SE3())
        self.callback.handle(fr)

class GObjectHandle(gs.GObjectHandle):
  def handle(self,obj):
    print("Pose:",obj.getPose())

slam=DeepGSLAM()
dataset=gs.Dataset()
dataset.open("/mnt/server0/users/zhaoyong/Dataset/TUM/RGBD/rgbd_dataset_freiburg3_sitting_xyz/.tumrgbd")

callback=GObjectHandle()
slam.setCallback(callback)

fr=dataset.grabFrame()

while(fr):
  slam.track(fr)
  fr=dataset.grabFrame()
