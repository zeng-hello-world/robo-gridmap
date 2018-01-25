//
// Created by zzy on 18-1-15.
//

#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <string.h>
#include <vector>
#include <limits>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCLoudXYZRGB;


static double g_lidar_height;
const double g_grid_length_type[5] = {0.1, 0.2, 0.25, 0.5, 1.0};
enum g_calculate_grid_cell_method{ differ_height_method = 1, sigma_height_method = 2, average_height_method = 3, RANSAC_plane_estimate = 4 };
enum g_grid_cell_property{ unknown_cell = -1, ground_cell = 0, obstacle_cell = 1 };

double g_point_feature[8];///1.beam range; 2.beam remission; 3.left beam range -range; 4.right beam range - beam range; 5.top beam range -beam range; 6.bottom beam range - beam range; 7.distance below sensor; 8.height above lowest measurement in grid cell

struct PointFeatureStruct
{
    double self_range_;///beam_range_
    double remission_;///beam_remission_
    double left_minus_self;///left_beam_range_minus_beam_range_
    double right_minus_self;///right_beam_range_minus_beam_range_
    double top_minus_self;///top_beam_range_minus_beam_range_
    double bottom_minus_self_;///bottom_beam_range_minus_beam_range_
    double lidar_height_;///distance_below_sensor_
    double height_above_lowest_;///height_above_lowest_measurement_in_grid_cell_
};

class GridMap
{
public:
    explicit GridMap( );
     ~GridMap(){}

public:
    void setInputCloud( const PointCloudXYZ::ConstPtr input_cloud );
    void setGridCellLength( const double& cell_length );
    void setDifferenceHeightCondition( const double & input_difference_height_threshold, const double &input_lowest_height_threshold );
    void setSigmaGridCondition( const double & input_sigma_height_threshold, const double &input_lowest_height_threshold );
    void setAverageGridCondition( const double & input_average_height_threshold, const double &input_lowest_height_threshold );
    void getGroundRemoveCloud( const unsigned int& property_method, PointCloudXYZ::Ptr output_cloud );
    void generateArbitraryLengthGridMap( const PointCloudXYZ::ConstPtr input_cloud, const double &input_grid_length, std::vector< std::vector< std::vector<pcl::PointXYZ> > > out_put_grid_map );///[input_grid_length] is only choosed from array [g_grid_length_type]
    void EuclideanCluster( const PointCloudXYZ::ConstPtr input_cloud_ptr, double input_max_cluster_distance, PointCLoudXYZRGB::Ptr output_cloud_ptr);

private:
    void initialCloudPointsDataStructure();
    void originCloudPointsPretreat();
    void generateOneMeterGridMap();
    void gridPropertyHeightDifferenceFunc();
    void gridPropertyHeightSigmaFunc();
    void gridPropertyAverageFunc();
    void gridPropertyRANSACFunc();
    void generateGroundRemoveCloud();
    void calculateGridMapProperty( const unsigned int &property_method );
    ///------- Boostrapping -------///
    void fixSingleLeftEdgeColumnNANPoints();
    void fixCentralNANPoints();
    void calculateLowestPointInGridCell();
    void calculateTopRowPointsFeature();
    void calculateBottomRowPointsFeature();
    void calculateCentralPointsFeature();

    void CalculateCloudPointsFeatures();


private:
    PointCloudXYZ::Ptr origin_cloud_ptr_;
    PointCloudXYZ::Ptr pretreated_cloud_ptr_;
    PointCloudXYZ::Ptr ground_remove_cloud_ptr_;
    std::vector< std::vector< std::vector<pcl::PointXYZ> > > grid_map_vec_;
    double grid_cell_length_;
    unsigned int grid_size_;
    unsigned int x_grid_num_;
    unsigned int y_grid_num_;
    std::vector< std::vector <int> > grid_map_property_;//use enum g_grid_cell_property
    double average_height_threshold_;
    double sigma_height_threshold_;
    double difference_height_threshold_;
    double lowest_height_threshold_;
    unsigned int cluster_min_size;
    unsigned int cluster_max_size;

private:
    bool is_initial_;
    unsigned  int lidar_point_struct_row_num_;
    unsigned  int lidar_point_struct_column_num_;
    double map_x_range_[2];
    double map_y_range_[2];

    ///--- for Bootstrapping ---
private:
    PointCloudXYZ::Ptr feature_calculate_cloud_ptr_;
    std::vector<std::vector< std::vector<PointFeatureStruct> > > cloud_point_feature_vec_;
    std::vector<std::vector<double>> grid_cell_lowest_height_vec_;



};


#endif //GRID_MAP_H
