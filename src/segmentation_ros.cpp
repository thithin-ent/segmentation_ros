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

    eulideanclusterextraction_.setClusterTolerance(0.5);
    eulideanclusterextraction_.setMinClusterSize(100);
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

    ptr_cloud = voxel_grid(ptr_cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ptr_vector;
    ptr_vector = ransac(ptr_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_cluster = eulidean(ptr_vector[0]);


    sensor_msgs::PointCloud2 segment_cloud;
    pcl::toROSMsg(*cloud_cluster, segment_cloud);
    segment_cloud.header.stamp = ros::Time().now();
    segment_cloud.header.frame_id = "base_link";
    scan_pub_.publish(segment_cloud);

    sensor_msgs::PointCloud2 planar_cloud;
    pcl::toROSMsg((*ptr_vector[1]), planar_cloud);
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

pcl::PointCloud<pcl::PointNormal>::Ptr Segment_point::get_pointnormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr,const double radius )
{

    pcl::NormalEstimation<pcl::PointXYZI, pcl::PointNormal> ne;
    ne.setInputCloud (src_ptr);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
    ne.setSearchMethod (tree);

        pcl::PointCloud<pcl::PointNormal>::Ptr normal_ptr(new pcl::PointCloud<pcl::PointNormal>);
    ne.setRadiusSearch (radius);

    ne.compute (*normal_ptr);
    return normal_ptr;
}

pcl::PointCloud<pcl::PointNormal>::Ptr Segment_point::get_MLSpointnormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr,const double radius )
{

    pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointNormal> mls;
    mls.setInputCloud (src_ptr);
    mls.setComputeNormals (true);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
    mls.setSearchMethod (tree);
    mls.setPolynomialOrder (2);
    pcl::PointCloud<pcl::PointNormal>normal;
    mls.setSearchRadius (radius);

    mls.process (normal);
    pcl::PointCloud<pcl::PointNormal>::Ptr normal_ptr(new pcl::PointCloud<pcl::PointNormal>);
    *normal_ptr = normal;
    return normal_ptr;
}

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

std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Segment_point::diff_normal(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ptr_vector;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>());
    PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
    copyPointCloud (*src_ptr, *doncloud);

    normals_small_scale = get_pointnormal(src_ptr,0.75);
    normals_large_scale = get_pointnormal(src_ptr,2.0);
    pcl::DifferenceOfNormalsEstimation<PointXYZI, PointNormal, PointNormal> don;
    don.setInputCloud (src_ptr);

    don.setNormalScaleLarge (normals_large_scale);

    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
      std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
      exit (EXIT_FAILURE);
    }

    don.computeFeature (*doncloud);
//    cout << "test1" << endl;
    pcl::ConditionOr<PointNormal>::Ptr range_cond_nonground (new pcl::ConditionOr<PointNormal> ());
    pcl::ConditionOr<PointNormal>::Ptr range_cond_ground (new pcl::ConditionOr<PointNormal> ());
    range_cond_nonground->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                                 new pcl::FieldComparison<PointNormal> ("z", pcl::ComparisonOps::GT, -1.0)));
    range_cond_nonground->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                                 new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, 0.2)));
    pcl::ConditionalRemoval<PointNormal> condrem(true);
    condrem.setCondition (range_cond_nonground);
    condrem.setInputCloud (doncloud);
//    cout << "test2" << endl;
    pcl::PointCloud<PointNormal>::Ptr donground_filtered (new pcl::PointCloud<PointNormal>);
    pcl::PointCloud<PointNormal>::Ptr nonground_filtered (new pcl::PointCloud<PointNormal>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud_nonground (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud_ground (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    condrem.filter (*nonground_filtered);
    condrem.getRemovedIndices(*inliers);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(src_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*ptr_cloud_nonground);
    ptr_vector.push_back(ptr_cloud_nonground);
    extract.setNegative(false);
    extract.filter(*ptr_cloud_ground);
    ptr_vector.push_back(ptr_cloud_ground);

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segment_point::region_glow(const pcl::PointCloud<pcl::PointXYZI>::Ptr src_ptr)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (kd_tree);
    normal_estimator.setInputCloud (src_ptr);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (kd_tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (src_ptr);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    reg.extract (cluster_indices);

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

    while (use_bag_ && std::getline(timestamp_file, line) && ros::ok())
    {
        std::stringstream folder_path;
        float timestamp = stof(line);
        folder_path << dataset_folder_ << "velodyne/sequences/" << sequences_ << "/velodyne/" << std::setfill('0') << std::setw(6) << line_num_ << ".bin";

        pcl::PointCloud<pcl::PointXYZI>::Ptr source_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        source_ptr = read_bin(folder_path.str());

        source_ptr = voxel_grid(source_ptr);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ptr_vector;
//        ptr_vector = ransac(source_ptr);
        ptr_vector = diff_normal(source_ptr);
//        ptr_vector = Progressive(source_ptr);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_cluster = eulidean(ptr_vector[0]);
//        cloud_cluster = region_glow(ptr_vector[0]);

        sensor_msgs::PointCloud2 segment_cloud;
        pcl::toROSMsg(*cloud_cluster, segment_cloud);
        segment_cloud.header.stamp = ros::Time().fromSec(timestamp);
        segment_cloud.header.frame_id = "base_link";
        scan_pub_.publish(segment_cloud);

        sensor_msgs::PointCloud2 planar_cloud;
        pcl::toROSMsg((*ptr_vector[1]), planar_cloud);
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
//        source_ptr = voxel_grid(source_ptr);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ptr_vector;
//        ptr_vector = ransac(source_ptr);
        ptr_vector = diff_normal(source_ptr);
//        ptr_vector = Progressive(source_ptr);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_cluster = eulidean(ptr_vector[0]);
//        cloud_cluster = region_glow(ptr_vector[0]);

        sensor_msgs::PointCloud2 segment_cloud;
        pcl::toROSMsg(*cloud_cluster, segment_cloud);
        segment_cloud.header.stamp = ros::Time().now();
        segment_cloud.header.frame_id = "base_link";
        scan_pub_.publish(segment_cloud);

        sensor_msgs::PointCloud2 planar_cloud;
        pcl::toROSMsg((*ptr_vector[1]), planar_cloud);
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

