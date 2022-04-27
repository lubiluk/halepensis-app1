#include "scene_entity.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <boost/make_shared.hpp>
#pragma clang diagnostic pop

using std::string;
using boost::shared_ptr;
using boost::make_shared;
using pcl::copyPointCloud;
using Eigen::Affine3f;
using boost::optional;
using boost::none;

scene_entity::scene_entity(entity_type type,
               entity_id id,
               vec3 position,
               rot_mat orientation,
               boost::shared_ptr<point_cloud> cloud,
               point min_corner,
               point max_corner):
type(type),
id(id),
position(position),
orientation(orientation),
cloud(cloud),
min_corner(min_corner),
max_corner(max_corner)
{
    
}

scene_entity::scene_entity():
type(entity_type::object),
id(""),
position(vec3{}),
orientation(quat{}),
cloud(nullptr),
min_corner({0,0,0,0,0,0}),
max_corner({0,0,0,0,0,0})
{
    
}

auto scene_entity::transformed(const mat44& transform) const -> scene_entity
{
    Affine3f t{transform};
    auto t_position = t * position;
    auto t_orientation = t * orientation;
    auto t_cloud = make_shared<point_cloud>();
    auto t_min_corn = t * vec3{min_corner.x, min_corner.y, min_corner.z};
    auto t_max_corn = t * vec3{max_corner.x, max_corner.y, max_corner.z};
    
    if (cloud) {
        transformPointCloud(*cloud, *t_cloud, transform);
    }

    point min_corner;
    min_corner.x = t_min_corn[0];
    min_corner.y = t_min_corn[1];
    min_corner.z = t_min_corn[2];

    point max_corner;
    max_corner.x = t_max_corn[0];
    max_corner.y = t_max_corn[1];
    max_corner.z = t_max_corn[2];
    
    return {
        type,
        id,
        t_position,
        t_orientation,
        t_cloud,
        min_corner,
        max_corner
    };
}

auto scene_entity::type_description() const -> string
{
    return entity_type_to_string(type);
}


auto entity_type_to_string(entity_type type) -> string
{
    switch (type) {
        case entity_type::object:
            return "object";
        case entity_type::mass_center:
            return "mass_center";
        case entity_type::hole:
            return "hole";
        case entity_type::peg:
            return "peg";
        default:
            return "unknown_entity_type";
    }
}

auto entity_type_from_string(const string& string) -> optional<entity_type>
{
    if (string == entity_type_to_string(entity_type::object)) {
        return entity_type::object;
    }
    
    if (string == entity_type_to_string(entity_type::mass_center)) {
        return entity_type::mass_center;
    }
    
    if (string == entity_type_to_string(entity_type::hole)) {
        return entity_type::hole;
    }
    
    if (string == entity_type_to_string(entity_type::peg)) {
        return entity_type::peg;
    }
    
    return none;
}

