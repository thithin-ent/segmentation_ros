#ifndef SEGMENTATION_POINT
#define SEGMENTATION_POINT

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace Eigen;

class Segment_point
{
    public:
        Segment_point();
        ~Segment_point();
        void scan_callback(const sensor_msgs::PointCloud2ConstPtr &data);

    private:
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber velodyne_sub;
        ros::Publisher scan_pub;
};

#endif