#include <segmentation_ros/segmentation_ros.h>

Segment_point::Segment_point()
{
    velodyne_sub_ = nh_.subscribe("velodyne_points", 1, &Segment_point::scan_callback, this);
    scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("segment_point", 1);
    planar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("planar_point", 1);
    line_num_ = 0;
    dataset_folder_ = "/media/user/F47CC21E7CC1DC0C/linux/KITTI/dataset/";

    seg_.setOptimizeCoefficients(true);
    seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setMaxIterations(200);
    seg_.setDistanceThreshold(0.3);
    seg_.setEpsAngle(  15.0f * (M_PI/180.0f) );

    eulideanclusterextraction_.setClusterTolerance(0.2);
    eulideanclusterextraction_.setMinClusterSize(200);
    eulideanclusterextraction_.setMaxClusterSize(15000);

}

Segment_point::~Segment_point()
{
}

void Segment_point::scan_callback(const sensor_msgs::PointCloud2ConstPtr &data)
{
    // std::chrono::high_resolution_clock::time_point start, end;

    // pcl::PCLPointCloud2 pcl_pc2;
    // pcl_conversions::toPCL(*data,pcl_pc2);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::fromPCLPointCloud2(pcl_pc2,*ptr_cloud);

    // // cout <<  "laser_cloud :" << (*ptr_cloud).size() << endl;

    // // start = std::chrono::high_resolution_clock::now();
    // pcl::SACSegmentation<pcl::PointXYZI> seg;
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI> ());
    // pcl::PCDWriter writer;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(100);
    // seg.setDistanceThreshold(0.5);
    // seg.setInputCloud (ptr_cloud);
    // seg.segment (*inliers, *coefficients);

    // pcl::ExtractIndices<pcl::PointXYZI> extract;
    // extract.setInputCloud(ptr_cloud);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter (*cloud_f);
    // // end = std::chrono::high_resolution_clock::now();
    // // double time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    // // cout << "time_plane: "<< time_elapsed <<" m second(s)."<< endl;

    // pcl::EuclideanClusterExtraction<pcl::PointXYZI> eulideanclusterextraction;
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZI>);
    // std::vector<pcl::PointIndices> cluster_indices;

    // eulideanclusterextraction.setClusterTolerance(0.2);
    // eulideanclusterextraction.setMinClusterSize(200);
    // eulideanclusterextraction.setMaxClusterSize(15000);
    // eulideanclusterextraction.setSearchMethod(kd_tree);
    // // start = std::chrono::high_resolution_clock::now();
    // eulideanclusterextraction.setInputCloud(cloud_f);
    // eulideanclusterextraction.extract(cluster_indices);
    // // end = std::chrono::high_resolution_clock::now();

    // // time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    // // cout << "time_cluster: "<< time_elapsed <<" m second(s)."<< endl;

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    // {
    //     int A = rand()%256;
    //     int B = rand()%256;
    //     int C = rand()%256;

    //     for (const auto& idx : it->indices)
    //     {
    //         pcl::PointXYZRGB point_temp;
    //         point_temp.x = cloud_f->points[idx].x;
    //         point_temp.y = cloud_f->points[idx].y;
    //         point_temp.z = cloud_f->points[idx].z;
    //         point_temp.r = A;
    //         point_temp.g = B;
    //         point_temp.b = C;
    //         cloud_cluster->push_back(point_temp);
    //     }
    // }

    // // pcl::toPCLPointCloud2(*cloud_cluster, pcl_pc2);
    // // pcl_conversions::toPCL(pcl_pc2,);

    // sensor_msgs::PointCloud2 segment_cloud;
    // pcl::toROSMsg(*cloud_cluster, segment_cloud);
    // segment_cloud.header.stamp = data->header.stamp;
    // segment_cloud.header.frame_id = data->header.frame_id;
    // scan_pub.publish(segment_cloud);

    // sensor_msgs::PointCloud2 planar_cloud;
    // pcl::toROSMsg(*cloud_f, planar_cloud);
    // segment_cloud.header.stamp = data->header.stamp;
    // segment_cloud.header.frame_id = data->header.frame_id;
    // planar_pub.publish(planar_cloud);
}

void Segment_point::eulidean_run()
{

    std::string timestamp_path =  "sequences/00/times.txt";
    std::ifstream timestamp_file(dataset_folder_ + timestamp_path, std::ifstream::in);
    std::string line;
    ros::Rate r(10.0);
    // cout << "run" << endl;
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        std::stringstream folder_path;
        float timestamp = stof(line);
        folder_path << dataset_folder_<< "velodyne/sequences/00/velodyne/" << std::setfill('0') << std::setw(6) << line_num_ << ".bin";
        std::vector<float> lidar_data = read_lidar_data(folder_path.str());

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

        pcl::VoxelGrid<pcl::PointXYZI> vg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        vg.setInputCloud(ptr_cloud);
        vg.setLeafSize(0.1f, 0.1f, 0.1f);
        vg.setMinimumPointsNumberPerVoxel(1);
        vg.filter(*cloud_filtered);
        ptr_cloud = cloud_filtered;

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        coefficients->values.resize (4);
        coefficients->values[3] = -0.5;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PCDWriter writer;

        seg_.setInputCloud(ptr_cloud);
        seg_.segment(*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(ptr_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_f);
        
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZI>);
        std::vector<pcl::PointIndices> cluster_indices;


        eulideanclusterextraction_.setSearchMethod(kd_tree);
        eulideanclusterextraction_.setInputCloud(cloud_f);
        eulideanclusterextraction_.extract(cluster_indices);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            int A = rand() % 256;
            int B = rand() % 256;
            int C = rand() % 256;

            for (const auto &idx : it->indices)
            {
                pcl::PointXYZRGB point_temp;
                point_temp.x = cloud_f->points[idx].x;
                point_temp.y = cloud_f->points[idx].y;
                point_temp.z = cloud_f->points[idx].z;
                point_temp.r = A;
                point_temp.g = B;
                point_temp.b = C;
                cloud_cluster->push_back(point_temp);
            }
        }
        sensor_msgs::PointCloud2 segment_cloud;
        pcl::toROSMsg(*cloud_cluster, segment_cloud);
        segment_cloud.header.stamp = ros::Time().fromSec(timestamp);
        segment_cloud.header.frame_id = "base_link";
        scan_pub_.publish(segment_cloud);

        sensor_msgs::PointCloud2 planar_cloud;
        pcl::toROSMsg(*cloud_f, planar_cloud);
        planar_cloud.header.stamp = ros::Time().fromSec(timestamp);
        planar_cloud.header.frame_id = "base_link";
        planar_pub_.publish(planar_cloud);
        line_num_++;
        r.sleep();
    }
}

void Segment_point::region_run()
{

    std::string timestamp_path =  "sequences/00/times.txt";
    std::ifstream timestamp_file(dataset_folder_ + timestamp_path, std::ifstream::in);
    std::string line;
    ros::Rate r(10.0);
    cout << "run" << endl;
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        std::stringstream folder_path;
        float timestamp = stof(line);
        folder_path << dataset_folder_<< "velodyne/sequences/00/velodyne/" << std::setfill('0') << std::setw(6) << line_num_ << ".bin";
        std::vector<float> lidar_data = read_lidar_data(folder_path.str());
        //  cout << folder_path.str() << endl;
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

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (ptr_cloud);
        normal_estimator.setKSearch (50);
        normal_estimator.compute (*normals);

        pcl::IndicesPtr indices (new std::vector <int>);
        pcl::removeNaNFromPointCloud(*ptr_cloud, *indices);
        pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
        reg.setMinClusterSize (50);
        reg.setMaxClusterSize (1000000);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (30);
        reg.setInputCloud (ptr_cloud);
        reg.setIndices (indices);
        reg.setInputNormals (normals);
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (1.0);       

        std::vector <pcl::PointIndices> cluster_indices;
        reg.extract (cluster_indices);




        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            int A = rand() % 256;
            int B = rand() % 256;
            int C = rand() % 256;

            for (const auto &idx : it->indices)
            {
                pcl::PointXYZRGB point_temp;
                point_temp.x = ptr_cloud->points[idx].x;
                point_temp.y = ptr_cloud->points[idx].y;
                point_temp.z = ptr_cloud->points[idx].z;
                point_temp.r = A;
                point_temp.g = B;
                point_temp.b = C;
                cloud_cluster->push_back(point_temp);
            }
        }

        sensor_msgs::PointCloud2 segment_cloud;
        pcl::toROSMsg(*cloud_cluster, segment_cloud);
        segment_cloud.header.stamp = ros::Time().fromSec(timestamp);
        segment_cloud.header.frame_id = "base_link";
        scan_pub_.publish(segment_cloud);

        // sensor_msgs::PointCloud2 planar_cloud;
        // pcl::toROSMsg(*cloud_f, planar_cloud);
        // planar_cloud.header.stamp = ros::Time().fromSec(timestamp);
        // planar_cloud.header.frame_id = "base_link";
        // planar_pub_.publish(planar_cloud);
        line_num_++;
        r.sleep();
    }
}



std::vector<float> Segment_point::read_lidar_data(const std::string folder_path)
{
    std::ifstream lidar_data_file(folder_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data[0]), num_elements*sizeof(float));
    return lidar_data;
}