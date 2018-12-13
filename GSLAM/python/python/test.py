#import cppimport
#gslam = cppimport.imp("gslam") #This will pause for a moment to compile the module
import gslam
pt=gslam.Point2d(1,2)


camera=gslam.Camera([640,480,400,400,320,240])
pt_world=camera.UnProject(pt)

print(pt,"world",pt_world)


dataset=gslam.Dataset()
dataset.open("/mnt/server0/users/zhaoyong/Dataset/KITTI/odomentry/color/00/mono.kitti")
frame=gslam.MapFrame(1,3)
print("Frame:",frame.id(),",time:",frame.timestamp(),",type:",frame.type(),frame.getImage())

frame=dataset.grabFrame()

if dataset.isOpened():
  print("Opened dataset")
  frame=dataset.grabFrame()
  print("Frame:",frame.id(),",time:",frame.timestamp(),",type:",frame.type(),frame.getImage())
else:
  print("Failed to open dataset")
  exit()


class GObjectHandle(gslam.GObjectHandle):
  def handle(self,obj):
    print("Received ",obj.type())

callback=GObjectHandle()
gslam.Timer.instance().disable()

paras=gslam.Svar()
paras.insert("SLAM.Verbose","3",1)
slam=gslam.SLAM.create("/data/zhaoyong/Program/Apps/RTMapper3InOne/GSLAM-DIYSLAM/build/libdiyslam.so")

print("SLAM "+slam.type()+" created")
slam.setSvar(paras)
slam.setCallback(callback)

for i in range(10):
  slam.track(dataset.grabFrame())

map=slam.getMap()
map.save("map.gmap")



