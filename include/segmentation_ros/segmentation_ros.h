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
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h> 
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>



using namespace std;
using namespace Eigen;
using namespace pcl;

class Segment_point
{
    public:
        Segment_point();
        ~Segment_point();
        void scan_callback(const sensor_msgs::PointCloud2ConstPtr &data);

        // read file

        std::vector<float> read_lidar_data(const std::string folder_path);
        pcl::PointCloud<pcl::PointXYZI>::Ptr read_bin(const std::string folder_path);
        pcl::PointCloud<pcl::PointXYZI>::Ptr read_pcd(const std::string folder_path);

        // down sampling

        PointCloud<PointXYZI>::Ptr voxel_grid(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr);

        // get point normal

        pcl::PointCloud<pcl::PointNormal>::Ptr get_pointnormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr,const double radius );
        pcl::PointCloud<pcl::PointNormal>::Ptr get_MLSpointnormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr,const double radius );


        // ground remove

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ransac(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr);
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Progressive(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr);
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> diff_normal(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr);

        // segmentation

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr eulidean(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_glow(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr);

        void kitti_run();
        void pcd_run();
        bool get_kitti();

    private:
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber velodyne_sub_;
        ros::Publisher scan_pub_;
        ros::Publisher planar_pub_;
        std::size_t line_num_;
        std::string dataset_folder_;
        std::string sequences_;
        bool use_bag_;
        bool use_kitti_;
        // down sampling class

        // point normal class

        // ground remove class
        pcl::SACSegmentation<pcl::PointXYZI> seg_;
        // segmentation class
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> eulideanclusterextraction_;
};

#endif
