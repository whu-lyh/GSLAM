from gslam import SE3,SO3,Point3d

T =SE3(SO3.exp(Point3d(1,2,3)),Point3d(1,0,0))
T1=SE3()
p =Point3d(3,4,5)
print("T",T,"T1",T1,"T.t",T.translation,"T.r",T.rotation)
print("T*T1",T*T1,"T*p",T.trans(p));


