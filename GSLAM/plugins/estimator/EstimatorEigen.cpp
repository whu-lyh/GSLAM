#if !defined(HAS_OPENCV)&&defined(HAS_EIGEN3)

#include "Estimators.h"
#include "RANSAC.h"
#include <GSLAM/core/Estimator.h>


namespace GSLAM
{

class EstimatorEigen : public Estimator
{
public:
    EstimatorEigen()
    {
    }

    virtual std::string type()const{return "EstimatorEigen";}

    inline std::vector<Eigen::Vector2d> toEigenArray(const std::vector<Point2d>& input)const
    {
        return *(std::vector<Eigen::Vector2d>*)(&input);
    }

    inline std::vector<Eigen::Vector3d> toEigenArray(const std::vector<Point3d>& input)const
    {
        return *(std::vector<Eigen::Vector3d>*)(&input);
    }

    // 2D corrospondences
    virtual bool findHomography(Homography2D& H,  // 3x3 dof=8
                                const std::vector<Point2d>& srcPoints,
                                const std::vector<Point2d>& dstPoints,
                                int    method = H4_Point&RANSAC,
                                double threshold = 3,
                                double confidence = 0.99,
                                std::vector<uchar>* mask = NULL) const {

        ransac::RANSACOptions options;
        options.max_error=threshold;
        options.confidence=confidence;
        ransac::LORANSAC<HomographyMatrixEstimator,HomographyMatrixEstimator> hransac(options);
        auto report=hransac.Estimate(toEigenArray(srcPoints),toEigenArray(dstPoints));
        if(!report.success) return false;
        Eigen::Matrix3d m=report.model/report.model(2,2);
        H=m;
        if(mask)
            *mask=report.inlier_mask;
        return report.success;
    }


    virtual bool findAffine2D(Affine2D& A,  // 2X3
                              const std::vector<Point2d>& srcPoints,
                              const std::vector<Point2d>& dstPoints,
                              int method = A3_Point&RANSAC,
                              double threshold = 3,
                              double confidence = 0.99,
                              std::vector<uchar>* mask = NULL) const
    {
        return false;
    }

    virtual bool findFundamental(Fundamental& F,  // 3x3
                                 const std::vector<Point2d>& points1,
                                 const std::vector<Point2d>& points2,
                                 int method = F8_Point&RANSAC,
                                 double threshold = 3.,
                                 double confidence = 0.99,
                                 std::vector<uchar>* mask = NULL) const
    {
        ransac::RANSACOptions options;
        options.max_error=threshold;
        options.confidence=confidence;
        if(method==F7_Point)
        {
            ransac::LORANSAC<FundamentalMatrixSevenPointEstimator,FundamentalMatrixSevenPointEstimator> hransac(options);
            auto report=hransac.Estimate(toEigenArray(points1),toEigenArray(points2));
            if(!report.success) return false;
            Eigen::Matrix3d m=report.model/report.model(2,2);
            F=m;
            if(mask)
                *mask=report.inlier_mask;
            return report.success;
        }
        else
        {
            ransac::LORANSAC<FundamentalMatrixEightPointEstimator,FundamentalMatrixEightPointEstimator> hransac(options);
            auto report=hransac.Estimate(toEigenArray(points1),toEigenArray(points2));
            if(!report.success) return false;
            Eigen::Matrix3d m=report.model/report.model(2,2);
            F=m;
            if(mask)
                *mask=report.inlier_mask;
            return report.success;
        }
        return false;
    }

    virtual bool findEssentialMatrix(Essential E,  // 3x3 dof=5
                                     const std::vector<Point2d>& points1,
                                     const std::vector<Point2d>& points2,
                                     int method = E5_Nister&RANSAC,
                                     double threshold = 0.01,
                                     double confidence = 0.99,
                                     std::vector<uchar>* mask = NULL) const  {
        ransac::RANSACOptions options;
        options.max_error=threshold;
        options.confidence=confidence;
        ransac::RANSAC<EssentialMatrixFivePointEstimator> hransac(options);
        auto report=hransac.Estimate(toEigenArray(points1),toEigenArray(points2));
        if(!report.success) return false;
        Eigen::Matrix3d m=report.model/report.model(2,2);
        E=m;
        if(mask)
            *mask=report.inlier_mask;
        return report.success;
    }

    // 3D corrospondences
    virtual bool findSIM3(SIM3& S,
                          const std::vector<Point3d>& from,
                          const std::vector<Point3d>& to,
                          int    method = S3_Horn&RANSAC,
                          double threshold = 0.01,
                          double confidence = 0.99,
                          std::vector<uchar>* mask = NULL) const {
        return false;
    }

    virtual bool findAffine3D(Affine3D& A,
                              const std::vector<Point3d>& src,
                              const std::vector<Point3d>& dst,
                              int    method = A4_Point&RANSAC,
                              double threshold =  0.01,
                              double confidence = 0.99,
                              std::vector<uchar>* mask = NULL) const
    {
        return false;
    }

    virtual bool findPlane(SE3& plane,
                           const std::vector<Point3d>& points,  // NOLINT
                           int    method = P3_Plane&RANSAC,
                           double threshold =  0.01,
                           double confidence = 0.99,
                           std::vector<uchar>* mask = NULL) const {
        ransac::RANSACOptions options;
        options.max_error=threshold;
        options.confidence=confidence;
        ransac::RANSAC<SE3PlaneEstimator> hransac(options);
        auto report=hransac.Estimate(points,points);
        if(!report.success) return false;
        plane=report.model;
        if(mask)
            *mask=report.inlier_mask;
        return report.success;
    }

    // 2D&3D corrospondences
    virtual bool findPnP( SE3& world2camera,
                          const std::vector<Point3d>& objectPoints,
                          const std::vector<Point2d>& imagePoints,
                          int    method = P3_ITERATIVE&RANSAC,
                          double threshold =  0.01,
                          double confidence = 0.99,
                          std::vector<uchar>* mask = NULL) const
    {
        ransac::RANSACOptions options;
        options.max_error=threshold;
        options.confidence=confidence;

        ransac::RANSAC<P3PEstimator> hransac(options);
        auto report=hransac.Estimate(toEigenArray(imagePoints),toEigenArray(objectPoints));
        if(!report.success) return false;
        auto& m=report.model;
        double t[12]={m(0,0),m(0,1),m(0,2),m(0,3),
                      m(1,0),m(1,1),m(1,2),m(1,3),
                      m(2,0),m(2,1),m(2,2),m(2,3)};
        world2camera.fromMatrix(t);
        if(mask)
            *mask=report.inlier_mask;
        return report.success;
    }

    virtual bool trianglate(const SE3 &t21, const Point3d &xn1, const Point3d &xn2, Point3d &pt) const
    {
        double t1[12]={1.,0.,0.,0,
                       0.,1.,0.,0.,
                       0.,0.,1.,0.};
        double t2[12];
        t21.getMatrix(t2);
        Eigen::Matrix4d A;
        A<<xn1[0]*t1[8]-t1[0], xn1[0]*t1[9]-t1[1], xn1[0]*t1[10]-t1[2], xn1[0]*t1[11]-t1[3],
           xn1[1]*t1[8]-t1[4], xn1[1]*t1[9]-t1[5], xn1[1]*t1[10]-t1[6], xn1[1]*t1[11]-t1[7],
           xn2[0]*t2[8]-t2[0], xn2[0]*t2[9]-t2[1], xn2[0]*t2[10]-t2[2], xn2[0]*t2[11]-t2[3],
           xn2[1]*t2[8]-t2[4], xn2[1]*t2[9]-t2[5], xn2[1]*t2[10]-t2[6], xn2[1]*t2[11]-t2[7];

        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A,Eigen::ComputeFullU | Eigen::ComputeFullV);
        auto v=svd.matrixV();
        if(v(3,3)==0) return false;
        Eigen::Vector4d x3D;
        x3D<<v(0,3),v(1,3),v(2,3),v(3,3);


        pt=Point3d(x3D[0]/x3D[3],x3D[1]/x3D[3],x3D[2]/x3D[3]);
        return true;
    }
};

USE_ESTIMATOR_PLUGIN(EstimatorEigen);

}

#endif
