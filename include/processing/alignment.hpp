#pragma once

#include "geometry.hpp"
#include <memory>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <Eigen/Geometry>
#pragma clang diagnostic pop

auto align(const boost::shared_ptr<point_cloud> &cloud1, const boost::shared_ptr<point_cloud> &cloud2)
-> std::pair<mat44, boost::shared_ptr<point_cloud>>;
