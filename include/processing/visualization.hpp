#pragma once

#include "geometry.hpp"
#include <vector>

auto view(const boost::shared_ptr<point_cloud>& cloud) -> void;
auto view(const boost::shared_ptr<point_cloud>& cloud1, const boost::shared_ptr<point_cloud>& cloud2) -> void;
auto view_clusters(const boost::shared_ptr<point_cloud>& cloud,
                   const std::vector<boost::shared_ptr<point_cloud>>& clusters) -> void;
