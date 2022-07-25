#include <segmentation_ros/segmentation_ros.h>

Segment_point::Segment_point()
{
    scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segment_point", 1);
    planar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("planar_point", 1);
    line_num_ = 0;
    nh_.getParam("/segmentation_ros/dataset_folder", dataset_folder_);
    nh_.getParam("/segmentation_ros/sequences", sequences_);
    nh_.getParam("/segmentation_ros/use_dataset", use_bag_);
    nh_.getParam("/segmentation_ros/use_kitti", use_kitti_);
//    cout << dataset_folder_ << endl;
//    cout << sequences_ << endl;
//    cout << use_bag_ << endl;
//    cout << use_kitti_ << endl;
    if (!use_bag_)
    {
        velodyne_sub_ = nh_.subscribe("velodyne_points", 1, &Segment_point::scan_callback, this);
    }

    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setMaxIterations(200);
    seg_.setDistanceThreshold(0.3);
    seg_.setEpsAngle(15.0f * (M_PI / 180.0f));

    eulideanclusterextraction_.setClusterTolerance(0.2);
    eulideanclusterextraction_.setMinClusterSize(200);
    eulideanclusterextraction_.setMaxClusterSize(15000);
}

Segment_point::~Segment_point()
{
}

void Segment_point::scan_callback(const sensor_msgs::PointCloud2ConstPtr &data)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*data, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *ptr_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_ptr = voxel_grid(ptr_cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ptr_vector;
    ptr_vector = ransac(voxel_ptr);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_cluster = eulidean(ptr_vector[0]);


    sensor_msgs::PointCloud2 segment_cloud;
    pcl::toROSMsg(*cloud_cluster, segment_cloud);
    segment_cloud.header.stamp = ros::Time().now();
    segment_cloud.header.frame_id = "base_link";
    scan_pub_.publish(segment_cloud);

    sensor_msgs::PointCloud2 planar_cloud;
    pcl::toROSMsg((*ptr_vector[0]), planar_cloud);
    planar_cloud.header.stamp = ros::Time().now();
    planar_cloud.header.frame_id = "base_link";
    planar_pub_.publish(planar_cloud);
}

// read file

pcl::PointCloud<pcl::PointXYZI>::Ptr Segment_point::read_bin(const std::string folder_path)
{
    std::vector<float> lidar_data = read_lidar_data(folder_path);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> laser_cloud;
    for (std::size_t i = 0; i < lidar_data.size(); i += 4)
    {
        pcl::PointXYZI point;
        point.x = lidar_data[i];
        point.y = lidar_data[i + 1];
        point.z = lidar_data[i + 2];
        point.intensity = lidar_data[i + 3];
        laser_cloud.push_back(point);
    }
    *ptr_cloud = laser_cloud;
    return ptr_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Segment_point::read_pcd(const std::string folder_path)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (folder_path, *ptr_cloud) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return nullptr;
      }
    return ptr_cloud;
}

std::vector<float> Segment_point::read_lidar_data(const std::string folder_path)
{
    std::ifstream lidar_data_file(folder_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char *>(&lidar_data[0]), num_elements * sizeof(float));
    return lidar_data;
}

// down sampling

PointCloud<PointXYZI>::Ptr Segment_point::voxel_grid(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr)
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    vg.setInputCloud(src_ptr);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.setMinimumPointsNumberPerVoxel(1);
    vg.filter(*cloud_filtered);
    return cloud_filtered;
}

// get point normal

// ground remove

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Segment_point::ransac(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ptr_vector;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>());

    seg_.setInputCloud(src_ptr);
    seg_.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(src_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_nonground);
    ptr_vector.push_back(cloud_nonground);
    extract.setNegative(false);
    extract.filter(*cloud_ground);
    ptr_vector.push_back(cloud_ground);

    return ptr_vector;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Segment_point::Progressive(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ptr_vector;
    pcl::PointIndicesPtr inliers(new pcl::PointIndices);
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_nonground(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZI>());

    pmf.setInputCloud(src_ptr);
    pmf.setMaxWindowSize(20);
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(0.5f);
    pmf.setMaxDistance(3.0f);
    pmf.extract(inliers->indices);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(src_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_nonground);
    ptr_vector.push_back(cloud_nonground);
    extract.setNegative(false);
    extract.filter(*cloud_ground);
    ptr_vector.push_back(cloud_ground);

    return ptr_vector;
}

// segmentation

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segment_point::eulidean(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> cluster_indices;

    eulideanclusterextraction_.setSearchMethod(kd_tree);
    eulideanclusterextraction_.setInputCloud(src_ptr);
    eulideanclusterextraction_.extract(cluster_indices);


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        int A = rand() % 256;
        int B = rand() % 256;
        int C = rand() % 256;

        for (const auto &idx : it->indices)
        {
            pcl::PointXYZRGB point_temp;
            point_temp.x = src_ptr->points[idx].x;
            point_temp.y = src_ptr->points[idx].y;
            point_temp.z = src_ptr->points[idx].z;
            point_temp.r = A;
            point_temp.g = B;
            point_temp.b = C;
            cloud_cluster->push_back(point_temp);
        }
    }
    return cloud_cluster;
}


void Segment_point::kitti_run()
{
    std::string timestamp_path = "sequences/" + sequences_ + "/times.txt";
    std::ifstream timestamp_file(dataset_folder_ + timestamp_path, std::ifstream::in);
    std::string line;
    ros::Rate r(10.0);
//    cout << timestamp_path << endl;
    while (use_bag_ && std::getline(timestamp_file, line) && ros::ok())
    {
        std::stringstream folder_path;
        float timestamp = stof(line);
        folder_path << dataset_folder_ << "velodyne/sequences/" << sequences_ << "/velodyne/" << std::setfill('0') << std::setw(6) << line_num_ << ".bin";

//        cout << folder_path.str() << endl;

        pcl::PointCloud<pcl::PointXYZI>::Ptr source_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        source_ptr = read_bin(folder_path.str());

        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        voxel_ptr = voxel_grid(source_ptr);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ptr_vector;
        ptr_vector = ransac(voxel_ptr);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_cluster = eulidean(ptr_vector[0]);

        sensor_msgs::PointCloud2 segment_cloud;
        pcl::toROSMsg(*cloud_cluster, segment_cloud);
        segment_cloud.header.stamp = ros::Time().fromSec(timestamp);
        segment_cloud.header.frame_id = "base_link";
        scan_pub_.publish(segment_cloud);

        sensor_msgs::PointCloud2 planar_cloud;
        pcl::toROSMsg((*ptr_vector[0]), planar_cloud);
        planar_cloud.header.stamp = ros::Time().fromSec(timestamp);
        planar_cloud.header.frame_id = "base_link";
        planar_pub_.publish(planar_cloud);
        line_num_++;
        r.sleep();
    }
}

void Segment_point::pcd_run()
{
    ros::Rate r(10.0);
    while (use_bag_ && ros::ok())
    {
        std::stringstream folder_path;
        folder_path << dataset_folder_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr source_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        source_ptr = read_pcd(folder_path.str());

        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        voxel_ptr = voxel_grid(source_ptr);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ptr_vector;
        ptr_vector = ransac(voxel_ptr);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_cluster = eulidean(ptr_vector[0]);

        sensor_msgs::PointCloud2 segment_cloud;
        pcl::toROSMsg(*cloud_cluster, segment_cloud);
        segment_cloud.header.stamp = ros::Time().now();
        segment_cloud.header.frame_id = "base_link";
        scan_pub_.publish(segment_cloud);

        sensor_msgs::PointCloud2 planar_cloud;
        pcl::toROSMsg((*ptr_vector[0]), planar_cloud);
        planar_cloud.header.stamp = ros::Time().now();
        planar_cloud.header.frame_id = "base_link";
        planar_pub_.publish(planar_cloud);
        r.sleep();
    }
}

bool Segment_point::get_kitti()
{
    return use_kitti_;
}

