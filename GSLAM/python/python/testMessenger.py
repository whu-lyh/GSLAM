from gslam import *
import time

def showStringMsg(obj):
  print("object is ",obj)

def showStatus(status):
  print("Status:",status)

class DeepSLAM:
  def init(self,config):
    self.messenger=Messenger()
    self.messenger.subscribe(MapFrame,"images",0,self.handleFrame)
    self.pubCurframe=self.messenger.advertise(MapFrame,"deep/curframe",1,False)
    self.pubImage=self.messenger.advertise(GImage,"deep/curImage",1,False)
    self.pubMap=self.messenger.advertise(Map,"deep/map",1,False)
    self.map=HashMap()
    return self.messenger
  def handleFrame(self,fr):
    image=fr.getImage(0)
    cam  =fr.getCamera(0);
    print("Processing frame ",fr.id(),"time:",fr.timestamp())
    self.map.insertMapFrame(fr)
    self.pubCurframe.publish(fr)
    self.pubImage.publish(image)
    self.pubMap.publish(self.map)

msg=Messenger.singleton()
svar=Svar.singleton()

svar.parseLine("Dataset=/mnt/server0/users/zhaoyong/Dataset/KITTI/odomentry/gray/00/stereo.kitti")

slam=DeepSLAM()
msg.accept(slam.init(svar))

qviz=Application.create("qviz")
qviz.init(svar).accept(msg)

while not svar.getInt("ShouldStop"):
  time.sleep(0.1)








