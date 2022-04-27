#include "alignment.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/make_shared.hpp>
#pragma clang diagnostic pop

auto align(const boost::shared_ptr<point_cloud> &cloud1, const boost::shared_ptr<point_cloud> &cloud2)
-> std::pair<mat44, boost::shared_ptr<point_cloud>>
{
    pcl::IterativeClosestPointWithNormals<point, point> icp;
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);
//    icp.setMaxCorrespondenceDistance (0.1);
//    icp.setMaximumIterations (50000);
//    icp.setTransformationEpsilon (1e-8);
//    icp.setEuclideanFitnessEpsilon (0.001);
//    icp.setUseReciprocalCorrespondences(true);
//    icp.setEnforceSameDirectionNormals(true);
//    icp.setRANSACIterations(10000);
//    icp.setUseSymmetricObjective(true);
//    icp.setRANSACOutlierRejectionThreshold(0.01);
//    icp.setTransformationRotationEpsilon(0.001);
    
    
    const auto result = boost::make_shared<point_cloud>();
    icp.align(*result);
    icp.getFitnessScore();
    return {icp.getFinalTransformation(), result};
}
