from gslam import SO3
from gslam import Point3d


r=SO3.exp(Point3d(1,2,3))
r1=SO3()
r2=SO3(0,0,0,1)
print(r,r1,r2,r.log(),r*r1*r2,r.trans(Point3d(1,0,0)))

