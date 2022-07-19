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
    cout << "success" <<endl;
}


