#pragma once

#include "geometry.hpp"
#include <memory>
#include <string>

auto extract_cloud(const boost::shared_ptr<point_cloud>& input_cloud,
                   const boost::shared_ptr<point_indices>& indices,
                   const bool negative = false) -> boost::shared_ptr<point_cloud>;

auto extract_normals(const boost::shared_ptr<surface_normals>& input_normals,
                     const boost::shared_ptr<point_indices>& indices) -> boost::shared_ptr<surface_normals>;

auto downsample(const boost::shared_ptr<point_cloud>& input,
                const float voxel_grid_size = 0.003f) -> boost::shared_ptr<point_cloud>;

auto filter_field(const boost::shared_ptr<point_cloud>& input,
                  const std::string& field_name,
                  const double low,
                  const double high,
                  const bool negative = false) -> boost::shared_ptr<point_cloud>;

auto remove_outliers(const boost::shared_ptr<point_cloud>& input) -> boost::shared_ptr<point_cloud>;
