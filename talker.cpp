#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include "grid_map.h"

static  ros::Publisher g_chatter_pub;

void removeCloudGround( PointCloudXYZ::Ptr input_cloud_ptr )
{
    input_cloud_ptr->header.frame_id = "rslidar";
    GridMap *test_cloud = new GridMap;

    ///--- 1.generate grid map ---
    test_cloud->setInputCloud( input_cloud_ptr );
    test_cloud->setGridCellLength( 1.0 );
    ///--- 2.remove ground points ---
    PointCloudXYZ::Ptr ground_remove_cloud_ptr( new PointCloudXYZ );
    unsigned int property_calculate_method = differ_height_method;
    test_cloud->setDifferenceHeightCondition( 0.2, -g_lidar_height + 0.15 );
    test_cloud->getGroundRemoveCloud( property_calculate_method, ground_remove_cloud_ptr );
    ///--- 3. Euclidean cluster and color ---
    PointCLoudXYZRGB::Ptr cluster_cloud_ptr( new PointCLoudXYZRGB );
    test_cloud->EuclideanCluster( ground_remove_cloud_ptr, 0.6, cluster_cloud_ptr );
    cluster_cloud_ptr->header.frame_id = "rslidar";
    g_chatter_pub.publish( cluster_cloud_ptr );
}

void removeCloudGround_Bootstrapping( PointCloudXYZ::Ptr input_cloud_ptr )
{
    input_cloud_ptr->header.frame_id = "rslidar";
    GridMap *test_cloud = new GridMap;

    ///--- 1.generate grid map ---
    test_cloud->setInputCloud( input_cloud_ptr );
    ///--- 2.remove ground points (Bootstrapping)---
    PointCloudXYZ::Ptr ground_remove_cloud_ptr( new PointCloudXYZ );
//    test_cloud->PretreatOriginCloudToFeatureCalculateCloud( ground_remove_cloud_ptr );

    ground_remove_cloud_ptr->header.frame_id = "rslidar";
    g_chatter_pub.publish( ground_remove_cloud_ptr );
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe( "rslidar_points", 10, removeCloudGround_Bootstrapping );
    g_chatter_pub = n.advertise< pcl::PointCloud< pcl::PointXYZ> >("chatter", 10);

    ros::spin();

    return 0;
}