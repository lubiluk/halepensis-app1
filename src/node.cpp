#include "node.hpp"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

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
        ROS_INFO_STREAM("Transforming point cloud from frame " << cloud->header.frame_id << " to frame " << _processingFrame);
        pcl_ros::transformPointCloud(_processingFrame, *cloud, *cloudInProcFrame, _tfListener);
        cloudInProcFrame->header.frame_id = _processingFrame;
    }
    else
        *cloudInProcFrame = *cloud;

    // Transform cloud to PCL format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloudInProcFrame, *pclCloud);

    ROS_INFO_STREAM("Processing:");
    ROS_INFO_STREAM("\tInput cloud:                 " << pclCloud->points.size() << " points");
}

void Node::start()
{
    _cloudSub = _nh.subscribe("cloud", 1, &Node::cloudCallback, this);
    _enabled = true;
}

void Node::stop()
{
    _cloudSub.shutdown();
    _enabled = false;
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