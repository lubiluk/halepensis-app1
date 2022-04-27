#pragma once

#include "geometry.hpp"
#include <vector>
#include <memory>


auto segment_regions(const boost::shared_ptr<point_cloud>& input_cloud)
-> std::vector<boost::shared_ptr<point_indices>>;
