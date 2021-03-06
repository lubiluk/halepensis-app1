#include "node.hpp"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "filtering.hpp"
#include "normals.hpp"
#include "ransac.hpp"
#include "task_understanding.hpp"
#include "visualization.hpp"
#include "scene_visualization.hpp"
// #include "graph_visualization.hpp"
#include <pcl/io/pcd_io.h>

using namespace cvp;

Node::Node(ros::NodeHandle &nh,
           ros::NodeHandle &pnh) : _nh(nh),
                                   _pnh(pnh),
                                   _enabled(false),
                                   _rate(1),
                                   _processingFrame("")
{
    _nh.setCallbackQueue(&_cbQueue);

    pnh.param<double>("rate", _rate, _rate);
    pnh.param<std::string>("frame", _processingFrame, _processingFrame);

    ROS_INFO_STREAM("The node will operate at maximum " << _rate << " Hz");

    if (_processingFrame.empty())
        ROS_INFO("The point cloud will be filtered in its original frame");
    else
        ROS_INFO_STREAM("The point cloud will be filtered after transforming it to the "
                        << _processingFrame << " frame");
}

Node::~Node()
{
}

void Node::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    if ((cloud->width * cloud->height) == 0)
        return;

    sensor_msgs::PointCloud2Ptr cloudInProcFrame;

    // Transform the point cloud to the frame specified if any
    if (!_processingFrame.empty())
    {
        cloudInProcFrame.reset(new sensor_msgs::PointCloud2);
        // ROS_INFO_STREAM("Transforming point cloud from frame " << cloud->header.frame_id << " to frame " << _processingFrame);
        pcl_ros::transformPointCloud(_processingFrame, *cloud, *cloudInProcFrame, _tfListener);
        cloudInProcFrame->header.frame_id = _processingFrame;
    }
    else
        *cloudInProcFrame = *cloud;

    // Transform cloud to PCL format
    pcl::PointCloud<pcl::PointNormal>::Ptr pclCloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*cloud, *pclCloud);
    _lastCloud = pclCloud;

    // ROS_INFO_STREAM("Processing:");
    // ROS_INFO_STREAM("\tInput cloud:                 " << _lastCloud->points.size() << " points");
}

void Node::joyCallback(const sensor_msgs::JoyConstPtr &joy)
{
    // ROS_INFO_STREAM("Joy " << joy->axes[0] << " " << joy->axes[1] << " " << joy->axes[2] << " " << joy->axes[3] << " " << joy->axes[4] << " " << joy->axes[5] << " " <<"\n");

    if (joy->axes[4] == 1.0)
    {
        ROS_INFO("Before cloud captured");
        _beforeCloud = _lastCloud;
        pcl::io::savePCDFileASCII("/home/lubiluk/Scratch/before_cloud.pcd", *_lastCloud);
        ROS_INFO_STREAM("\tBefore cloud:                 " << _beforeCloud->points.size() << " points");
    }

    if (joy->axes[4] == -1.0)
    {
        ROS_INFO("After cloud captured");
        _afterCloud = _lastCloud;
        pcl::io::savePCDFileASCII("/home/lubiluk/Scratch/after_cloud.pcd", *_lastCloud);
        ROS_INFO_STREAM("\tAfter cloud:                 " << _afterCloud->points.size() << " points");
    }

    if (joy->axes[5] == -1.0)
    {
        ROS_INFO("Reasoning triggered");
        runTaskUnderstanding();
    }
}

void Node::start()
{
    _cloudSub = _nh.subscribe("cloud", 1, &Node::cloudCallback, this);
    _joySub = _nh.subscribe("joy", 1, &Node::joyCallback, this);
    _enabled = true;
}

void Node::stop()
{
    _cloudSub.shutdown();
    _enabled = false;
}

void Node::runTaskUnderstanding()
{
    auto cloud_before = _beforeCloud;
    auto cloud_after = _afterCloud;

    view(cloud_before, cloud_after);

    cloud_before = remove_outliers(cloud_before);
    cloud_after = remove_outliers(cloud_after);
    cloud_before = downsample(cloud_before);
    cloud_after = downsample(cloud_after);

    // view(cloud_before, cloud_after);

    compute_normals(cloud_before);
    compute_normals(cloud_after);

    /* Remove walls */
    auto indics = fit_plane(cloud_before);
    cloud_before = extract_cloud(cloud_before, std::get<0>(indics.value()), true);
    indics = fit_plane(cloud_after);
    cloud_after = extract_cloud(cloud_after, std::get<0>(indics.value()), true);

    view(cloud_before, cloud_after);

    /* Task Reasoning Part */

    task_understanding task{cloud_before, cloud_after};
    task.detect_objects(false);
    view_clusters(task.before_scene.cloud, task.before_scene.object_clouds());
    task.detect_change();
    task.detect_features();
    task.describe_relations(false);

    view_scenes(task);
    // There is a bug that prevents us from showing graphs side by side...
    // view(task.before_scene.graph);
    // view(task.after_scene.graph);

    task.describe_task();

    ROS_INFO("It worked!");
}

void Node::run()
{
    ros::Rate loopRate(_rate);

    double halfPeriod = 0.5 * 1.0 / _rate;

    start();

    while (ros::ok())
    {
        // check for subscriber's callbacks
        _cbQueue.callAvailable(ros::WallDuration(halfPeriod));

        loopRate.sleep();
    }

    stop();
}
