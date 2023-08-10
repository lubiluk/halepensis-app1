#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>

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
        void cloudCallback(const sensor_msgs::PointCloud2Ptr &cloud);
        void joyCallback(const sensor_msgs::JoyConstPtr &joy);

        void start();
        void stop();
        void runTaskUnderstanding();

        ros::NodeHandle &_nh;
        ros::CallbackQueue _cbQueue;
        bool _enabled;
        double _rate;
        int _axis_before;
        int _axis_after;
        std::string _dir;

        // ROS subscribers
        ros::Subscriber _cloudSub;
        ros::Subscriber _joySub;

        pcl::PointCloud<pcl::PointNormal>::Ptr _lastCloud;
        pcl::PointCloud<pcl::PointNormal>::Ptr _beforeCloud;
        pcl::PointCloud<pcl::PointNormal>::Ptr _afterCloud;
    };
} // cvp