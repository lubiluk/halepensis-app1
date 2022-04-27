#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>

auto detect_pegs(const boost::shared_ptr<point_cloud> &cloud)
-> std::vector<boost::shared_ptr<point_cloud>>;
