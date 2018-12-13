from gslam import Camera,Point3d,Point2d

nocam=Camera()
ideal=Camera([640,480])
pinhole=Camera([640,480,400,400,320,240])
atan   =Camera([640,480,400,400,320,240,0.1])
openCV =Camera([640,480,400,400,320,240,0,0,0,0,0])

cameras=(nocam,ideal,pinhole,atan,openCV)

pt3d=Point3d(1,2,3)
pt2d=Point2d(0,0)
for cam in cameras:
    print(cam.CameraType(),cam.width(),cam.height(),cam.info(),cam.Project(pt3d),cam.UnProject(pt2d))
    print("Para:",cam.getParameters())
