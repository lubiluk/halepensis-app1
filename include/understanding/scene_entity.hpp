#pragma once

#include "geometry.hpp"
#include <string>
#include <memory>
#include <boost/optional.hpp>

using entity_id = std::string;

enum class entity_type {
    object,
    mass_center,
    hole,
    peg
};

class scene_entity {
public:
    entity_type type;
    entity_id id;
    vec3 position;
    rot_mat orientation;
    boost::shared_ptr<point_cloud> cloud;
    point min_corner;
    point max_corner;
    
    scene_entity(entity_type type,
           entity_id id,
           vec3 position,
           rot_mat orientation = rot_mat{},
           boost::shared_ptr<point_cloud> cloud = nullptr,
           point min_corner = {},
           point max_corner = {});
    
    scene_entity();
    scene_entity(const scene_entity& original) = default;
    
    auto transformed(const mat44& transform) const -> scene_entity;
    auto type_description() const -> std::string;
};


auto entity_type_to_string(entity_type type) -> std::string;
auto entity_type_from_string(const std::string& string) -> boost::optional<entity_type>;
