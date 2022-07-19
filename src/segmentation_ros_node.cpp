#include <segmentation_ros/segmentation_ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "segmentation_ros");
    Segment_point segment_point;
    ros::spin();

    return 0;
}
