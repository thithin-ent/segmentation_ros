#include <segmentation_ros/segmentation_ros.h>

Segment_point::Segment_point()
{
    velodyne_sub = nh_.subscribe("velodyne_points", 1, &Segment_point::scan_callback,this);
    scan_pub = nh_.advertise<sensor_msgs::PointCloud2>("pointcloude_pub",1);
}

Segment_point::~Segment_point()
{
}

void Segment_point::scan_callback(const sensor_msgs::PointCloud2ConstPtr &data)
{
    std::chrono::high_resolution_clock::time_point start, end;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*data,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*ptr_cloud);

    // cout <<  "laser_cloud :" << (*ptr_cloud).size() << endl;

    // start = std::chrono::high_resolution_clock::now();   
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.5);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(ptr_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter (*cloud_f);
    // end = std::chrono::high_resolution_clock::now();
    // double time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    // cout << "time_plane: "<< time_elapsed <<" m second(s)."<< endl;


    
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> eulideanclusterextraction;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> cluster_indices;

    eulideanclusterextraction.setClusterTolerance(0.2);
    eulideanclusterextraction.setMinClusterSize(200);
    eulideanclusterextraction.setMaxClusterSize(15000);
    eulideanclusterextraction.setSearchMethod(kd_tree);
    // start = std::chrono::high_resolution_clock::now();   
    eulideanclusterextraction.setInputCloud(cloud_f);
    eulideanclusterextraction.extract(cluster_indices);
    // end = std::chrono::high_resolution_clock::now();
    
    // time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    // cout << "time_cluster: "<< time_elapsed <<" m second(s)."<< endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        int A = rand()%256;
        int B = rand()%256;
        int C = rand()%256;
        
        for (const auto& idx : it->indices)
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

    // pcl::toPCLPointCloud2(*cloud_cluster, pcl_pc2);
    // pcl_conversions::toPCL(pcl_pc2,);

    sensor_msgs::PointCloud2 segment_cloud;
    pcl::toROSMsg(*cloud_cluster, segment_cloud);
    segment_cloud.header.stamp = data->header.stamp;
    segment_cloud.header.frame_id = data->header.frame_id;
    scan_pub.publish(segment_cloud);

}


