from gslam import Point3d

pt=Point3d()
pt1=Point3d(1,2,3)
pt.x=3
pt.y=4

print("pt",pt,"pt1",pt1,"sum",pt+pt1,"dot",pt.dot(pt1),"norm",pt.norm(),pt.x,pt.y)
print(pt*2,pt/2,pt+pt1,pt-pt1)
