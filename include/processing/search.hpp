#pragma once

#include "geometry.hpp"
#include <vector>
#include <memory>
#include <optional>

auto count_nearby_points(const boost::shared_ptr<point_cloud>& input_cloud,
                         const point search_point, const float radius) -> int;

auto point_from_coefficients(boost::shared_ptr<model_coefficients> coefficients) -> point;
