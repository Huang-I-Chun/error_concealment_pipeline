// Matching

class Stage2_Nearest_Neighbor: public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud


public:

    Stage2_Nearest_Neighbor(int my_mode): mode(my_mode){
    }

    void run(Pipeline_Object& pipeline_obj){

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        if(mode == 1){
            ds_prev_cloud = pipeline_obj.ds1_prev_cloud;
            ds_next_cloud = pipeline_obj.ds1_next_cloud;
        }
        else if(mode == 2){
            ds_prev_cloud = pipeline_obj.ds2_prev_cloud;
            ds_next_cloud = pipeline_obj.ds2_next_cloud;
        }

        // will save downsample point cloud match to another downsample point cloud
        std::vector<int> nearest_search_result; // use nearest_search_result[ds_NEXT_FRAME_POINT_INDEX] = ds_PREV_FRAME_POINT_INDEX
        for (int idx = 0; idx < ds_next_cloud->points.size() + 2; idx++)
        {
            pipeline_obj.matching_table.push_back(-1);
        }

        pcl::search::KdTree<pcl::PointXYZRGB> prev_kdtree;
        prev_kdtree.setInputCloud(ds_prev_cloud);

        for (int idx = 0; idx < ds_next_cloud->points.size() + 2; idx++)
        {
            std::vector<int> prev_pointIdxRadiusSearch;
            std::vector<float> prev_pointRadiusSquaredDistance;
            prev_kdtree.nearestKSearch(ds_next_cloud->points[idx], 1, prev_pointIdxRadiusSearch, prev_pointRadiusSquaredDistance);
            pipeline_obj.matching_table[idx] = prev_pointIdxRadiusSearch[0];
        }
    }



};

// class Stage2_NN
// {
// private:
// public:
//     double resolution = 1024; // initial resolution, actually, this is the height of point cloud

//     void run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
//              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
//              std::vector<int> &matching_table);
// };

// class Stage2_QR
// {
// private:
// public:
//     double resolution = 1024; // initial resolution, actually, this is the height of point cloud
//     double search_radius = 2; // initial search radius for query radius in kdtree

//     void run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
//              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
//              std::vector<int> &matching_table);
// };

// class Stage2_CFDS
// {
// private:
// public:
//     double resolution = 1024; // initial resolution, actually, this is the height of point cloud
//     double magnificant = 1.5; // magnificant * nn_dist will be the search radius of CFDS

//     void run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
//              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
//              std::vector<int> &matching_table);
// };
