#pragma once

#include "geometry.hpp"
#include <memory>
#include <tuple>
#include <optional>
#include <boost/optional.hpp>


auto fit_cylinder(const boost::shared_ptr<point_cloud>& input_cloud)
-> boost::optional<std::tuple<boost::shared_ptr<point_indices>, boost::shared_ptr<model_coefficients>>>;

auto fit_plane(const boost::shared_ptr<point_cloud>& input_cloud)
-> boost::optional<std::tuple<boost::shared_ptr<point_indices>, boost::shared_ptr<model_coefficients>>>;

auto fit_sphere(const boost::shared_ptr<point_cloud>& input_cloud)
-> boost::optional<std::tuple<boost::shared_ptr<point_indices>, boost::shared_ptr<model_coefficients>>>;

auto fit_circle(const boost::shared_ptr<point_cloud>& input_cloud)
-> boost::optional<std::tuple<boost::shared_ptr<point_indices>, boost::shared_ptr<model_coefficients>>>;
