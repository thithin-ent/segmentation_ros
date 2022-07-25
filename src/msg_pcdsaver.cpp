#include <msg_pcdsaver/msg_pcdsaver.h>

std::string topic_name;
std::string file_path;


void msg_callback(const sensor_msgs::PointCloud2ConstPtr &data)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*data, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *ptr_cloud);

    pcl::io::savePCDFileASCII (file_path, *ptr_cloud);
    std::cout << "save" << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "msg_pcdsaver");
    ros::NodeHandle n;
    n.param<std::string>("/msg_pcdsaver/topic_name", topic_name, "laser_cloud_surround");
    n.param<std::string>("/msg_pcdsaver/file_path", file_path, "/media/user/F47CC21E7CC1DC0C/linux/pcd/test.pcd");
    ros::Subscriber sub = n.subscribe(topic_name, 1, msg_callback);
    ros::spin();

    return 0;
}

