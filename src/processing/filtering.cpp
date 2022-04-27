#include "filtering.hpp"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/make_shared.hpp>
#pragma clang diagnostic pop


auto extract_cloud(const boost::shared_ptr<point_cloud>& input_cloud,
                   const boost::shared_ptr<point_indices>& indices,
                   const bool negative) -> boost::shared_ptr<point_cloud>
{
    pcl::ExtractIndices<point> extract;
    extract.setInputCloud(input_cloud);
    
    const auto result_cloud = boost::make_shared<point_cloud>();
    
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*result_cloud);
    
    return result_cloud;
}

auto extract_normals(const boost::shared_ptr<surface_normals>& input_normals,
                   const boost::shared_ptr<point_indices>& indices) -> boost::shared_ptr<surface_normals>
{
    pcl::ExtractIndices<normal> extract;
    extract.setInputCloud(input_normals);
    
    const auto result_normals = boost::make_shared<surface_normals>();
    
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*result_normals);
    
    return result_normals;
}

auto downsample(const boost::shared_ptr<point_cloud>& input,
                const float voxel_grid_size) -> boost::shared_ptr<point_cloud>
{
    pcl::VoxelGrid<point> vox_grid;
    auto output = boost::make_shared<point_cloud>();
    vox_grid.setInputCloud(input);
    vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter(*output);
    
    return output;
}

auto filter_field(const boost::shared_ptr<point_cloud>& input,
                  const std::string& field_name,
                  const double low,
                  const double high,
                  const bool negative) -> boost::shared_ptr<point_cloud>
{
    pcl::PassThrough<point> pass;
    auto output = boost::make_shared<point_cloud>();
    pass.setInputCloud(input);
    pass.setFilterFieldName(field_name);
    pass.setFilterLimits(low, high);
    pass.setNegative(negative);
    pass.filter(*output);
    
    return output;
}

auto remove_outliers(const boost::shared_ptr<point_cloud>& input) -> boost::shared_ptr<point_cloud>
{
    auto output = boost::make_shared<point_cloud>();
    pcl::StatisticalOutlierRemoval<point> sor;
    sor.setInputCloud(input);
    sor.setMeanK(50);
    sor.setStddevMulThresh(2.0);
    sor.filter(*output);
    
    return output;
}
