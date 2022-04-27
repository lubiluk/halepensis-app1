#pragma once

#include "geometry.hpp"
#include <memory>
#include <vector>

auto estimate_boundaries(const boost::shared_ptr<point_cloud>& cloud)
-> boost::shared_ptr<point_indices>;
