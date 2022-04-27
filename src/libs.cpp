#include "geometry.hpp"
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/make_shared.hpp>

auto function() -> void
{
    auto cloud_projected = boost::make_shared<point_cloud>();
    auto point = pcl::PointNormal();
    point.x = 1.f;
    point.y = 1.f;
    point.z = 1.f;
    std::cout << "Test pass\n";
}