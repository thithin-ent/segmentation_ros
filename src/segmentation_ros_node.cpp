#include <segmentation_ros/segmentation_ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "segmentation_ros");
    Segment_point segment_point;
    if (segment_point.get_kitti())
    {
        segment_point.kitti_run();
    }
    else
    {
        segment_point.pcd_run();
    }

    ros::spin();

    return 0;
}
