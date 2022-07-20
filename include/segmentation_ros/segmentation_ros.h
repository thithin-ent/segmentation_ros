#ifndef SEGMENTATION_POINT
#define SEGMENTATION_POINT

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <chrono>

using namespace std;
using namespace Eigen;

class Segment_point
{
    public:
        Segment_point();
        ~Segment_point();
        void scan_callback(const sensor_msgs::PointCloud2ConstPtr &data);
        void run();
        std::vector<float> read_lidar_data(const std::string folder_path);

    private:
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber velodyne_sub_;
        ros::Publisher scan_pub_;
        ros::Publisher planar_pub_;
        std::size_t line_num_;
        std::string dataset_folder_;
        pcl::SACSegmentation<pcl::PointXYZI> seg_;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> eulideanclusterextraction_;
};

#endif