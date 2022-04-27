#pragma once

#include "geometry.hpp"
#include <memory>
#include <string>

auto load_cloud(const std::string& pcd_file) -> boost::shared_ptr<point_cloud>;
