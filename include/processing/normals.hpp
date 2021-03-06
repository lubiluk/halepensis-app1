#pragma once

#include "geometry.hpp"
#include <memory>

auto compute_normals(const boost::shared_ptr<point_cloud> &cloud,
                     const float normal_radius = 0.02f) -> void;
