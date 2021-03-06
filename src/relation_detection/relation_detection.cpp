#include "relation_detection.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#pragma clang diagnostic pop

using namespace boost::geometry;
using model::box;

using point_xyz = model::point<float, 3, cs::cartesian>;

const float epsilon_vertical = 0.01;
const float epsilon_horizontal = 0.025;

auto is_below(const scene_entity& entity1, const scene_entity& entity2) -> bool
{
    return entity1.position.y() < entity2.position.y() - epsilon_vertical
    && entity1.position.x() < entity2.position.x() + epsilon_horizontal
    && entity1.position.x() > entity2.position.x() - epsilon_horizontal
    && entity1.position.z() < entity2.position.z() + epsilon_horizontal
    && entity1.position.z() > entity2.position.z() - epsilon_horizontal;
}

auto is_inside(const scene_entity& entity1, const scene_entity& entity2) -> bool
{
    box<point_xyz> box1{{entity1.min_corner.x, entity1.min_corner.y, entity1.min_corner.z},
        {entity1.max_corner.x, entity1.max_corner.y, entity1.max_corner.z}};
    box<point_xyz> box2{{entity2.min_corner.x, entity2.min_corner.y, entity2.min_corner.z},
        {entity2.max_corner.x, entity2.max_corner.y, entity2.max_corner.z}};
    
    auto o = overlaps(box1, box2);
    auto d1 = abs(box1.max_corner().get<0>() - box1.min_corner().get<0>())
    + abs(box1.max_corner().get<1>() - box1.min_corner().get<1>())
    + abs(box1.max_corner().get<2>() - box1.min_corner().get<2>());
    auto d2 = abs(box2.max_corner().get<0>() - box2.min_corner().get<0>())
    + abs(box2.max_corner().get<1>() - box2.min_corner().get<1>())
    + abs(box2.max_corner().get<2>() - box2.min_corner().get<2>());
    
    return o && d2 < d1;
}
