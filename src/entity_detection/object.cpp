#include "object.hpp"
#include "clusters.hpp"
#include "ransac.hpp"
#include "filtering.hpp"

auto detect_objects(const boost::shared_ptr<point_cloud> &cloud, const int n)
-> std::vector<boost::shared_ptr<point_cloud>>
{
    const auto indices = extract_euclidean_clusters(cloud, 0.03, 100, 10000);

    std::vector<boost::shared_ptr<point_cloud>> clusters;
    std::transform(indices.begin(), indices.end(), std::back_inserter(clusters), [&cloud](const auto& i) -> auto
    {
        return extract_cloud(cloud, i);
    });
    
    const std::vector<boost::shared_ptr<point_cloud>> objects( clusters.begin(), clusters.begin() + n);
    
    return objects;
}
