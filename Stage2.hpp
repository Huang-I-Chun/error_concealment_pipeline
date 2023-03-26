// Matching

class Stage2_None: public Stage
{
private:
public:

    Stage2_None(){
    }

    void run(Pipeline_Object& pipeline_obj){
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
