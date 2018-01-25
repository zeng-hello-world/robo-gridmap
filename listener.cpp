#include "ros/ros.h"
#include "grid_map.h"

static ros::Publisher g_chatter_pub;

void dealCloudData( const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud)
{
    GridMap removal_test;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    removal_test.getGroundSeedPts( input_cloud, output_cloud );
    output_cloud->header.frame_id = "rslidar";
    g_chatter_pub.publish( output_cloud );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 10, dealCloudData);
    g_chatter_pub = n.advertise< pcl::PointCloud< pcl::PointXYZ> >("chatter2", 10);

    ros::spin();

    return 0;
}