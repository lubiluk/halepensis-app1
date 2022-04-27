#pragma once

#include "geometry.hpp"
#include <memory>

auto scale(const boost::shared_ptr<point_cloud>& cloud, const float factor)
-> boost::shared_ptr<point_cloud>;
