//
// Created by zzy on 18-1-15.
//

#include "grid_map.h"


GridMap::GridMap( )
{
    origin_cloud_ptr_.reset( new PointCloudXYZ );
    pretreated_cloud_ptr_.reset( new PointCloudXYZ );
    ground_remove_cloud_ptr_.reset( new PointCloudXYZ );
    feature_calculate_cloud_ptr_.reset( new PointCloudXYZ );

    g_lidar_height = 1.8;
    lidar_point_struct_row_num_ = 16;
    lidar_point_struct_column_num_ = 2016;
    map_x_range_[0] = -75.0; map_x_range_[1] = 75.0;// x range [-75.0, 75.0]
    map_y_range_[0] = -75.0; map_y_range_[1] = 75.0;// y range [-75.0, 75.0]

    grid_cell_length_ = 1.0;
    grid_size_ = map_x_range_[1] / grid_cell_length_;
    average_height_threshold_ = -g_lidar_height + 0.1;
    lowest_height_threshold_ = -g_lidar_height + (-0.05);
    sigma_height_threshold_ = 0.0; ///----todo
    difference_height_threshold_ = 0.2;
    cluster_min_size = 20;
    cluster_max_size = 10000;

    this->initialCloudPointsDataStructure();
}

void GridMap::setInputCloud(const PointCloudXYZ::ConstPtr input_cloud)
{
    *origin_cloud_ptr_ = *input_cloud;

    this->originCloudPointsPretreat();
}

void GridMap::setGridCellLength( const double & cell_length )
{
    grid_cell_length_ = cell_length;
}

void GridMap::getGroundRemoveCloud(const unsigned int& property_method, PointCloudXYZ::Ptr output_cloud)
{
    output_cloud->points.clear();
    this->calculateGridMapProperty( property_method );

    *output_cloud = *ground_remove_cloud_ptr_;
}

void GridMap::generateArbitraryLengthGridMap( const PointCloudXYZ::ConstPtr input_cloud, const double &input_grid_length, std::vector< std::vector< std::vector<pcl::PointXYZ> > > out_put_grid_map )
{
    if ( input_cloud->empty() )
    {
        PCL_ERROR( "Pretreated cloud is empty!" );
        return;
    }

    unsigned int temp_cloud_points_num;
    temp_cloud_points_num = (unsigned int) input_cloud->size();
    unsigned int temp_grid_num;
    temp_grid_num = ((map_x_range_[1] - map_x_range_[0]) / input_grid_length) * ((map_y_range_[1] - map_y_range_[0]) / input_grid_length);
    double temp_length_times_factor = 0;// the times of input grid length to 1 meter
    temp_length_times_factor = 1 / input_grid_length;
    double temp_point_x = 0;
    double temp_point_y = 0;
    int temp_point_x_int = 0;
    int temp_point_y_int = 0;
    unsigned int point_x_vec_No = 0;
    unsigned int point_y_vec_No = 0;
    for (int i = 0; i < temp_cloud_points_num; ++i)
    {
        temp_point_x = input_cloud->points[i].x;
        temp_point_y = input_cloud->points[i].y;

        temp_point_x = temp_point_x * temp_length_times_factor;
        temp_point_y = temp_point_y * temp_length_times_factor;

        temp_point_x_int = (int) temp_point_x;
        temp_point_y_int = (int) temp_point_y;

        point_x_vec_No = temp_point_x_int + grid_size_ * temp_length_times_factor;///---todo: if [temp_length_times_factor] is not an integer, code will corrupt
        point_y_vec_No = temp_point_y_int + grid_size_ * temp_length_times_factor;///---todo: if [temp_length_times_factor] is not an integer, code will corrupt

        out_put_grid_map[point_x_vec_No][point_x_vec_No].push_back( input_cloud->points[i] );
    }
}

void GridMap::EuclideanCluster(const PointCloudXYZ::ConstPtr input_cloud_ptr, double input_max_cluster_distance, PointCLoudXYZRGB::Ptr output_cloud_ptr)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    //create 2d pc
    PointCloudXYZ::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*input_cloud_ptr, *cloud_2d);
    //make it flat
    for (size_t i=0; i<cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud (cloud_2d);

    std::vector<pcl::PointIndices> cluster_indices;

    //perform clustering on 2d cloud
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance ( input_max_cluster_distance ); //
    ec.setMinClusterSize (cluster_min_size);
    ec.setMaxClusterSize (cluster_max_size);
    ec.setSearchMethod( tree );
    ec.setInputCloud( cloud_2d );
    ec.extract ( cluster_indices );

    ///---	3. Color clustered points  ---
    unsigned int k = 0;
    std::vector<PointCLoudXYZRGB> clusters;
    clusters.resize( cluster_indices.size() );
    unsigned int temp_single_cluster_points = 0;
    for(std::vector<pcl::PointIndices>::iterator iter = cluster_indices.begin(); iter != cluster_indices.end(); ++iter )
    {
        unsigned int cloud_color_rgb[3];
        cloud_color_rgb[0] = rand()%255;
        cloud_color_rgb[1] = rand()%255;
        cloud_color_rgb[2] = rand()%255;
        for (std::vector<int>::iterator i = iter->indices.begin(); i != iter->indices.end() ; ++i)
        {
            pcl::PointXYZRGB p;
            p.x = input_cloud_ptr->points[*i].x;
            p.y = input_cloud_ptr->points[*i].y;
            p.z = input_cloud_ptr->points[*i].z;
            p.r = cloud_color_rgb[0];
            p.g = cloud_color_rgb[1];
            p.b = cloud_color_rgb[2];

            output_cloud_ptr->push_back( p );
        }
    }
}

void GridMap::initialCloudPointsDataStructure()
{
    x_grid_num_ = (unsigned int) abs((map_x_range_[1] - map_x_range_[0]) / grid_cell_length_);
    y_grid_num_ = (unsigned int) abs((map_y_range_[1] - map_y_range_[0]) / grid_cell_length_);
    if (x_grid_num_ != 0 && y_grid_num_!= 0)
    {
        grid_map_vec_.resize( x_grid_num_ );
        grid_map_property_.resize( x_grid_num_ );
        for (int i = 0; i < x_grid_num_; ++i)
        {
            grid_map_vec_[i].resize( y_grid_num_ );

            grid_map_property_[i].resize( y_grid_num_ );
            for (int j = 0; j < y_grid_num_; ++j)
            {
                grid_map_property_[i][j] = unknown_cell;
            }
        }
    }

    ///--- initial data of points features vector  ---
    /// add 2 columns and 2 rows to original point cloud data structure
    cloud_point_feature_vec_.resize( lidar_point_struct_row_num_ );//column size of RS-16-Lidar
    for (int k = 0; k < cloud_point_feature_vec_.size(); ++k)
    {
        cloud_point_feature_vec_[k].resize( lidar_point_struct_column_num_ );//row size of RS-16-Lidar
    }

    is_initial_ = true;
}

void GridMap::originCloudPointsPretreat()
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud ( origin_cloud_ptr_ );
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.7, 0.0);// lidar height is 1.8m
    pass.filter ( *pretreated_cloud_ptr_ );
}



void GridMap::gridPropertyHeightSigmaFunc( )
{
    double temp_point_z=0;
    double temp_average_height = 0;
    double temp_lowest_height = 0;
    double temp_sigma_height = 0;
    unsigned int temp_grid_cell_points_num;
    for (int i = 0; i < x_grid_num_; ++i)
    {
        for (int j = 0; j < y_grid_num_; ++j)
        {
            temp_grid_cell_points_num = (unsigned int)grid_map_vec_[i][j].size();
            for (int k = 0; k < temp_grid_cell_points_num; ++k)
            {
                temp_point_z = grid_map_vec_[i][j][k].z;
                temp_average_height += temp_point_z;

                if ( temp_lowest_height > temp_point_z)
                {
                    temp_lowest_height = temp_point_z;
                }
            }

            temp_average_height = temp_average_height / temp_grid_cell_points_num;
            for (int k = 0; k < temp_grid_cell_points_num; ++k)
            {
                temp_sigma_height += (grid_map_vec_[i][j][k].z - temp_average_height)*(grid_map_vec_[i][j][k].z - temp_average_height);
            }
            temp_sigma_height = temp_sigma_height / temp_grid_cell_points_num;

            ///------------[todo] the sigma decision condition-----------///
            if ( temp_sigma_height > sigma_height_threshold_ )
            {
                grid_map_property_[i][j] = obstacle_cell;
            }
            else
            {
                if ( temp_lowest_height > lowest_height_threshold_ && temp_lowest_height < fabs(lowest_height_threshold_) )
                {
                    grid_map_property_[i][j] = ground_cell;
                }
            }
        }
    }
}

void GridMap::gridPropertyAverageFunc( )
{
    double temp_point_z=0;
    double temp_average_height = 0;
    double temp_lowest_height = 0;
    unsigned int temp_grid_cell_points_num;
    for (int i = 0; i < x_grid_num_; ++i)
    {
        for (int j = 0; j < y_grid_num_; ++j)
        {
            temp_grid_cell_points_num = (unsigned int)grid_map_vec_[i][j].size();
            for (int k = 0; k < temp_grid_cell_points_num; ++k)
            {
                temp_point_z = grid_map_vec_[i][j][k].z;
                temp_average_height += temp_point_z;

                if ( temp_lowest_height > temp_point_z)
                {
                    temp_lowest_height = temp_point_z;
                }
            }
            temp_average_height = temp_average_height / temp_grid_cell_points_num;

            if ( temp_average_height > average_height_threshold_ )
            {
                grid_map_property_[i][j] = obstacle_cell;
            }
            else
            {
                if ( temp_lowest_height > lowest_height_threshold_ && temp_lowest_height < fabs(lowest_height_threshold_) )
                {
                    grid_map_property_[i][j] = ground_cell;
                }
            }
        }
    }
}

void GridMap::gridPropertyRANSACFunc()
{
///----todo
}





void GridMap::generateOneMeterGridMap()
{
    unsigned int temp_cloud_points_num;
    temp_cloud_points_num =  pretreated_cloud_ptr_->size();
    int temp_point_x_int;
    int temp_point_y_int;
    unsigned int temp_point_x_vec_No;
    unsigned int temp_point_y_vec_No;
    for (int i = 0; i < temp_cloud_points_num; ++i)
    {
        if ( isnan( pretreated_cloud_ptr_->points[i].x ) )
            continue;

        temp_point_x_int = (int)pretreated_cloud_ptr_->points[i].x;
        temp_point_x_vec_No = temp_point_x_int + grid_size_;
        temp_point_y_int = (int)pretreated_cloud_ptr_->points[i].y;
        temp_point_y_vec_No = temp_point_y_int + grid_size_;
        if ( temp_point_x_int < -(grid_size_-1) || temp_point_x_int > grid_size_-1 ||
                temp_point_y_int < -(grid_size_-1) || temp_point_y_int > grid_size_-1 )
            continue;

        grid_map_vec_[temp_point_x_vec_No][temp_point_y_vec_No].push_back( pretreated_cloud_ptr_->points[i] );
    }
}

void GridMap::generateGroundRemoveCloud()
{
    for (int i = 0; i < x_grid_num_; ++i)
    {
        for (int j = 0; j < y_grid_num_; ++j)
        {
            unsigned int temp_points_num;
            if ( grid_map_property_[i][j] == obstacle_cell )
            {
                temp_points_num = (unsigned int) grid_map_vec_[i][j].size();
                for (int k = 0; k < temp_points_num; ++k)
                {
                    ground_remove_cloud_ptr_->push_back( grid_map_vec_[i][j][k] );
                }
            }
        }
    }
}

void GridMap::calculateGridMapProperty( const unsigned int &property_method )
{
    if (!is_initial_)
    {
        PCL_ERROR("Cloud points is empty! Try again!");
        return;
    }

    this->generateOneMeterGridMap();

    if (property_method == differ_height_method)
    {
        this->gridPropertyHeightDifferenceFunc( );
    }
    else if ( property_method == sigma_height_method )
    {
        this->gridPropertyHeightSigmaFunc( );
    }
    else if ( property_method == average_height_method )
    {
        this->gridPropertyAverageFunc( );
    }
    else if ( property_method == RANSAC_plane_estimate )
    {
        this->gridPropertyRANSACFunc( );
    }

    this->generateGroundRemoveCloud();
}

void GridMap::setAverageGridCondition(const double &input_average_height_threshold, const double &input_lowest_height_threshold)
{
    average_height_threshold_ = input_average_height_threshold;
    lowest_height_threshold_ = input_lowest_height_threshold;
}

void GridMap::setSigmaGridCondition(const double &input_sigma_height_threshold, const double &input_lowest_height_threshold)
{
    sigma_height_threshold_ = input_sigma_height_threshold;
    lowest_height_threshold_ = input_lowest_height_threshold;
}

void GridMap::setDifferenceHeightCondition(const double &input_difference_height_threshold, const double &input_lowest_height_threshold)
{
    difference_height_threshold_ = input_difference_height_threshold;
    lowest_height_threshold_ = input_lowest_height_threshold;
}

void GridMap::gridPropertyHeightDifferenceFunc( )
{
    double temp_point_z = 0;
    double temp_height_difference = 0;
    unsigned int temp_grid_cell_points_num;
    for (int i = 0; i < x_grid_num_; ++i)
    {
        for (int j = 0; j < y_grid_num_; ++j)
        {
            if (grid_map_vec_[i][j].empty())
                continue;
            temp_grid_cell_points_num = static_cast<int>(grid_map_vec_[i][j].size());

            double temp_highest_height =  - std::numeric_limits<double >::max();//
            double temp_lowest_height = std::numeric_limits<double >::max();
            for (int k = 0; k < temp_grid_cell_points_num; ++k)
            {
                temp_point_z = grid_map_vec_[i][j][k].z;

                if ( temp_lowest_height > temp_point_z )
                {
                    temp_lowest_height = temp_point_z;
                }
                if ( temp_highest_height < temp_point_z )
                {
                    temp_highest_height = temp_point_z;
                }
            }

            temp_height_difference = fabs( temp_highest_height - temp_lowest_height );

            if ( temp_height_difference > difference_height_threshold_ )
            {
                grid_map_property_[i][j] = obstacle_cell;
            }
            else
            {
                if ( temp_lowest_height > lowest_height_threshold_
                     && temp_lowest_height < ( -g_lidar_height + 0.2 ) )// the lowest point height is between[-lidar_height-0.05, -lidar_height + 0.2]
                {
                    grid_map_property_[i][j] = ground_cell;
                }
            }
        }
    }
}


///---------   Bootstrappping   ----------///
void GridMap::fixSingleLeftEdgeColumnNANPoints()
{
    for (int i = 0; i < feature_calculate_cloud_ptr_->height; ++i)
    {
        if ( isnan( feature_calculate_cloud_ptr_->at(0,i).x ) )
        {
            int temp_num = 1;
            while ( isnan( feature_calculate_cloud_ptr_->at(temp_num, i).x ) )
            {
                ++temp_num;
            }

            for (int j = temp_num; j > 1 ; --j)
            {
                feature_calculate_cloud_ptr_->at(j-1,i) = feature_calculate_cloud_ptr_->at(j,i);
            }
        }
    }
}

void GridMap::fixCentralNANPoints()
{
    std::vector<pcl::PointXYZ> temp_points_template;
    double temp_distance;
    double temp_distance_threshold = 0.5;
    for (int i = 0; i < feature_calculate_cloud_ptr_->height; ++i)
    {
        for (int j = 1; j < feature_calculate_cloud_ptr_->width-1; ++j)
        {
            if ( isnan(feature_calculate_cloud_ptr_->at(j, i).x) )
            {
                temp_points_template.push_back(feature_calculate_cloud_ptr_->at(j - 1, i));
                if (!isnan(feature_calculate_cloud_ptr_->at(j + 1, i).x))
                {
                    temp_points_template.push_back(feature_calculate_cloud_ptr_->at(j + 1, i));
                    temp_distance = sqrt( pow( temp_points_template[0].x - temp_points_template[1].x, 2 ) + pow( temp_points_template[0].y - temp_points_template[1].y, 2 ) + pow( temp_points_template[0].z - temp_points_template[1].z, 2 ) );
                    if ( temp_distance > temp_distance_threshold )
                    {
                        feature_calculate_cloud_ptr_->at(j, i).x = temp_points_template[0].x;
                        feature_calculate_cloud_ptr_->at(j, i).y = temp_points_template[0].y;
                        feature_calculate_cloud_ptr_->at(j, i).z = temp_points_template[0].z;
                    }
                    else
                    {
                        feature_calculate_cloud_ptr_->at(j, i).x = (temp_points_template[0].x + temp_points_template[1].x) / 2;
                        feature_calculate_cloud_ptr_->at(j, i).y = (temp_points_template[0].y + temp_points_template[1].y) / 2;
                        feature_calculate_cloud_ptr_->at(j, i).z = (temp_points_template[0].z + temp_points_template[1].z) / 2;
                    }
                }
                else
                {
                    feature_calculate_cloud_ptr_->at(j, i).x = temp_points_template[0].x;
                    feature_calculate_cloud_ptr_->at(j, i).y = temp_points_template[0].y;
                    feature_calculate_cloud_ptr_->at(j, i).z = temp_points_template[0].z;
                }
            }

            temp_points_template.clear();
        }
    }
}

void GridMap::calculateLowestPointInGridCell()
{
    grid_cell_lowest_height_vec_.resize( grid_map_vec_.size() );
    for (int i = 0; i < grid_cell_lowest_height_vec_.size(); ++i)
    {
        grid_cell_lowest_height_vec_[i].resize( grid_map_vec_[i].size() );
        for (int j = 0; j < grid_cell_lowest_height_vec_[i].size(); ++j)
        {
            grid_cell_lowest_height_vec_[i][j] = std::numeric_limits<double >::max();
            for (int k = 0; k < grid_map_vec_[i][j].size(); ++k)
            {
                if ( grid_cell_lowest_height_vec_[i][j] > grid_map_vec_[i][j][k].z )
                {
                    grid_cell_lowest_height_vec_[i][j] = grid_map_vec_[i][j][k].z;
                }
            }
        }
    }
}

void GridMap::calculateTopRowPointsFeature()
{
    double temp_range[4];//store beam range of: [self, left, right, bottom], no top
    unsigned int temp_point_x_vec_No;
    unsigned int temp_point_y_vec_No;
    PointFeatureStruct temp_point_feature;
    for (int i = 1; i < feature_calculate_cloud_ptr_->width-1; ++i)
    {
        if ( isnan( feature_calculate_cloud_ptr_->at(i,0).x ) )
            continue;

        temp_point_x_vec_No = (int)pretreated_cloud_ptr_->points[i].x + grid_size_;
        temp_point_y_vec_No = (int)pretreated_cloud_ptr_->points[i].y + grid_size_;

        temp_range[0] = sqrt( pow( feature_calculate_cloud_ptr_->at(i,0).x, 2 ) + pow( feature_calculate_cloud_ptr_->at(i,0).y, 2 ) + pow( feature_calculate_cloud_ptr_->at(i,0).z, 2 ) );
        temp_range[1] = sqrt( pow( feature_calculate_cloud_ptr_->at(i-1,0).x, 2 ) + pow( feature_calculate_cloud_ptr_->at(i-1,0).y, 2 ) + pow( feature_calculate_cloud_ptr_->at(i-1,0).z, 2 ) );
        temp_range[2] = sqrt( pow( feature_calculate_cloud_ptr_->at(i+1,0).x, 2 ) + pow( feature_calculate_cloud_ptr_->at(i+1,0).y, 2 ) + pow( feature_calculate_cloud_ptr_->at(i+1,0).z, 2 ) );
        temp_range[3] = sqrt( pow( feature_calculate_cloud_ptr_->at(i,1).x, 2 ) + pow( feature_calculate_cloud_ptr_->at(i,1).y, 2 ) + pow( feature_calculate_cloud_ptr_->at(i,1).z, 2 ) );

        temp_point_feature.self_range_ = temp_range[0];
        temp_point_feature.remission_ = feature_calculate_cloud_ptr_->at( i,0 ).data[3];
        temp_point_feature.left_minus_self = temp_range[1] - temp_range[0];
        temp_point_feature.right_minus_self = temp_range[2] - temp_range[0];
        temp_point_feature.top_minus_self = 0;
        temp_point_feature.bottom_minus_self_ = temp_range[3] - temp_range[0];
        temp_point_feature.lidar_height_ = g_lidar_height;
        temp_point_feature.height_above_lowest_ = feature_calculate_cloud_ptr_->at(i,0).z - grid_cell_lowest_height_vec_[temp_point_x_vec_No][temp_point_x_vec_No];

        cloud_point_feature_vec_[0][i].push_back( temp_point_feature );
    }
}

void GridMap::calculateBottomRowPointsFeature()
{

}

void GridMap::calculateCentralPointsFeature()
{

}

void GridMap::CalculateCloudPointsFeatures()
{
    *feature_calculate_cloud_ptr_=*origin_cloud_ptr_;

    this->fixSingleLeftEdgeColumnNANPoints();
    this->fixCentralNANPoints();
    this->generateOneMeterGridMap();

    this->calculateLowestPointInGridCell();
    this->calculateTopRowPointsFeature();
    this->calculateBottomRowPointsFeature();


}




























































