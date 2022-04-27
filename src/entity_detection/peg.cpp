#include "peg.hpp"
#include "ransac.hpp"
#include "regions.hpp"
#include "filtering.hpp"

auto detect_pegs(const boost::shared_ptr<point_cloud>& cloud)
-> std::vector<boost::shared_ptr<point_cloud>>
{
    auto peg_indices = segment_regions(cloud);
    
    std::vector<boost::shared_ptr<point_cloud>> pegs;
    std::transform(peg_indices.begin(), peg_indices.end(),
                   std::back_inserter(pegs),
                   [&cloud](const auto& i) -> auto {
        return extract_cloud(cloud, i);
    });
    
    return pegs;
}
