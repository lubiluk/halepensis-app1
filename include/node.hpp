#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cvp
{

    class Node
    {
    public:
        Node(ros::NodeHandle &nh,
             ros::NodeHandle &pnh);

        virtual ~Node();

        void run();

    protected:
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
        void joyCallback(const sensor_msgs::JoyConstPtr &joy);

        void start();
        void stop();

        void publish(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &planeCloud,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &nonPlaneCloud,
                     pcl::uint64_t &stamp,
                     const std::string &frameId);

        void publishEmptyClouds(pcl::uint64_t &stamp,
                                const std::string &frameId);

        void normalizeVector(std::vector<double> &v);

        ros::NodeHandle &_nh, _pnh;
        ros::CallbackQueue _cbQueue;
        bool _enabled;
        double _rate;

        tf::TransformListener _tfListener;

        // frame in which the point cloud will be transformed
        std::string _processingFrame;

        // ROS subscribers
        ros::Subscriber _cloudSub;
        ros::Subscriber _joySub;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _lastCloud;
    };
} // cvp