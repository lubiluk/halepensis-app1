#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>

auto detect_objects(const boost::shared_ptr<point_cloud> &cloud, const int n = 2)
-> std::vector<boost::shared_ptr<point_cloud>>;
