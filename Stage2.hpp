// Matching

class Stage2_Nearest_Neighbor : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud

public:
    Stage2_Nearest_Neighbor(int my_mode) : mode(my_mode)
    {
    }

    void run(Pipeline_Object &pipeline_obj)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        if (mode == 1)
        {
            ds_prev_cloud = pipeline_obj.ds1_prev_cloud;
            ds_next_cloud = pipeline_obj.ds1_next_cloud;
        }
        else if (mode == 2)
        {
            ds_prev_cloud = pipeline_obj.ds2_prev_cloud;
            ds_next_cloud = pipeline_obj.ds2_next_cloud;
        }

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

        if (_next != NULL)
        {
            _next->run(pipeline_obj);
        }
    }
};

class Stage2_CFDS : public Stage
{
private:
    int mode;             // 1 for ds1_point_cloud, 2 for ds2_point_cloud
    double magnification; // CFDS dynamic decide the search radius, will search for 'magnification * nn_dist'

public:
    Stage2_CFDS(int my_mode) : mode(my_mode)
    {
    }

    void run(Pipeline_Object &pipeline_obj)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        if (mode == 1)
        {
            ds_prev_cloud = pipeline_obj.ds1_prev_cloud;
            ds_next_cloud = pipeline_obj.ds1_next_cloud;
        }
        else if (mode == 2)
        {
            ds_prev_cloud = pipeline_obj.ds2_prev_cloud;
            ds_next_cloud = pipeline_obj.ds2_next_cloud;
        }

        bool use_next_search_prev;
        int length = ds_next_cloud->points.size() > ds_prev_cloud->points.size() ? ds_next_cloud->points.size() : ds_prev_cloud->points.size();

        for (int idx = 0; idx < length + 2; idx++)
        {
            pipeline_obj.matching_table.push_back(-1);
        }

        int color_dist = 32;
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            origin_prev_cloud = ds_prev_cloud;
            origin_next_cloud = ds_next_cloud;

            use_next_search_prev = from_prev_or_next(origin_prev_cloud, origin_next_cloud);

            if (!use_next_search_prev)
            {
                // prev_cloud->points = next_cloud->points;
                // next_cloud->points = prev_cloud->points;
                prev_cloud = origin_next_cloud;
                next_cloud = origin_prev_cloud;
            }
            else
            {
                next_cloud = origin_next_cloud;
                prev_cloud = origin_prev_cloud;
            }

            // kmeans to categorize color
            std::vector<pcl::PointXYZRGB> kmeans_points;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> kmeans_point_clouds;
            std::vector<pcl::search::KdTree<pcl::PointXYZRGB>> kmeans_kdtrees;

            for (int idx = 0; idx < prev_cloud->points.size(); idx++)
            {
                bool flag = true;
                for (int i = 0; i < kmeans_points.size(); i++)
                {
                    if (abs(kmeans_points[i].r - prev_cloud->points[idx].r) < color_dist &&
                        abs(kmeans_points[i].g - prev_cloud->points[idx].g) < color_dist &&
                        abs(kmeans_points[i].b - prev_cloud->points[idx].b) < color_dist)
                    {
                        flag = false;
                        break;
                    }
                }

                if (flag)
                {
                    kmeans_points.push_back(prev_cloud->points[idx]);
                }
            }

            for (int kmeans_round = 0; kmeans_round < 2; kmeans_round++)
            {

                kmeans_point_clouds.clear();
                for (int i = 0; i < kmeans_points.size(); i++)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kmeans_new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                    kmeans_point_clouds.push_back(kmeans_new_cloud);
                }

                for (int idx = 0; idx < prev_cloud->points.size(); idx++)
                {
                    int matching_idx = 0;
                    double color_diff = pow(prev_cloud->points[idx].r - kmeans_points[0].r, 2) +
                                        pow(prev_cloud->points[idx].g - kmeans_points[0].g, 2) +
                                        pow(prev_cloud->points[idx].b - kmeans_points[0].b, 2);
                    for (int i = 1; i < kmeans_points.size(); i++)
                    {
                        double new_color_diff = pow(prev_cloud->points[idx].r - kmeans_points[i].r, 2) +
                                                pow(prev_cloud->points[idx].g - kmeans_points[i].g, 2) +
                                                pow(prev_cloud->points[idx].b - kmeans_points[i].b, 2);
                        if (new_color_diff < color_diff)
                        {
                            color_diff = new_color_diff;
                            matching_idx = i;
                        }
                    }
                    kmeans_point_clouds[matching_idx]->push_back(prev_cloud->points[idx]);
                }

                kmeans_points.clear();
                for (int i = 0; i < kmeans_point_clouds.size(); i++)
                {
                    if (kmeans_point_clouds[i]->points.size() == 0)
                        continue;

                    double axis_x = 0, axis_y = 0, axis_z = 0;
                    double red = 0, green = 0, blue = 0;

                    for (int idx = 0; idx < kmeans_point_clouds[i]->points.size(); idx++)
                    {
                        axis_x += kmeans_point_clouds[i]->points[idx].x;
                        axis_y += kmeans_point_clouds[i]->points[idx].y;
                        axis_z += kmeans_point_clouds[i]->points[idx].z;
                        red += kmeans_point_clouds[i]->points[idx].r;
                        green += kmeans_point_clouds[i]->points[idx].g;
                        blue += kmeans_point_clouds[i]->points[idx].b;
                    }
                    pcl::PointXYZRGB kmeans_point;
                    kmeans_point.x = axis_x / kmeans_point_clouds[i]->points.size();
                    kmeans_point.y = axis_y / kmeans_point_clouds[i]->points.size();
                    kmeans_point.z = axis_z / kmeans_point_clouds[i]->points.size();
                    kmeans_point.r = red / kmeans_point_clouds[i]->points.size();
                    kmeans_point.g = green / kmeans_point_clouds[i]->points.size();
                    kmeans_point.b = blue / kmeans_point_clouds[i]->points.size();
                    kmeans_points.push_back(kmeans_point);
                }
            }

            for (int i = 0; i < kmeans_point_clouds.size(); i++)
            {
                pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
                kdtree.setInputCloud(kmeans_point_clouds[i]);
                kmeans_kdtrees.push_back(kdtree);
            }

            // get max_x - min_x in a point cloud to get the voxel size for skelton downsample
            double max_x = -9999999, min_x = 9999999;
            double max_y = -9999999, min_y = 9999999;
            double max_z = -9999999, min_z = 9999999;

            for (int idx = 0; idx < next_cloud->points.size(); idx++)
            {
                if (max_x < next_cloud->points[idx].x)
                {
                    max_x = next_cloud->points[idx].x;
                }
                if (max_y < next_cloud->points[idx].y)
                {
                    max_y = next_cloud->points[idx].y;
                }
                if (max_z < next_cloud->points[idx].z)
                {
                    max_z = next_cloud->points[idx].z;
                }

                if (min_x > next_cloud->points[idx].x)
                {
                    min_x = next_cloud->points[idx].x;
                }
                if (min_y > next_cloud->points[idx].y)
                {
                    min_y = next_cloud->points[idx].y;
                }
                if (min_z > next_cloud->points[idx].z)
                {
                    min_z = next_cloud->points[idx].z;
                }
            }

            double voxel_length = max_x - min_x;
            if (max_y - min_y > voxel_length)
            {
                voxel_length = max_y - min_y;
            }
            if (max_z - min_z > voxel_length)
            {
                voxel_length = max_z - min_z;
            }
            voxel_length = voxel_length / 10;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr skelton_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::VoxelGrid<pcl::PointXYZRGB> sor2;
            sor2.setInputCloud(next_cloud);
            sor2.setLeafSize(voxel_length, voxel_length, voxel_length);
            sor2.filter(*skelton_next_cloud);

            // for each point in next_cloud, we will find the nn in skelton_next_cloud;'
            std::vector<int> next_cloud_nn_skelton_idx;
            for (int idx = 0; idx < next_cloud->points.size(); idx++)
            {
                next_cloud_nn_skelton_idx.push_back(-1);
            }
            pcl::search::KdTree<pcl::PointXYZRGB> skleton_next_kdtree;
            skleton_next_kdtree.setInputCloud(skelton_next_cloud);
            for (int idx = 0; idx < next_cloud->points.size(); idx++)
            {
                std::vector<int> nearest_idx;
                std::vector<float> nearest_dist;
                skleton_next_kdtree.nearestKSearch(next_cloud->points[idx], 1, nearest_idx, nearest_dist);
                next_cloud_nn_skelton_idx[idx] = nearest_idx[0];
            }

            // initialize a list with same length of skelton_next_cloud, store the max search distance of each point in skelton_next_cloud
            std::vector<double> next_skelton_search_radius;
            for (int idx = 0; idx < skelton_next_cloud->points.size(); idx++)
            {
                next_skelton_search_radius.push_back(-1);
            }

            // main process

            std::vector<int> match_time; // see if prev cloud index have been matched is_matched[PREV_CLOUD_IDX] = NEXT_CLOUD_IDX
            for (int idx = 0; idx < prev_cloud->points.size() + 2; idx++)
            {
                match_time.push_back(0);
            }

            pcl::search::KdTree<pcl::PointXYZRGB> prev_kdtree;
            prev_kdtree.setInputCloud(prev_cloud);

            pcl::search::KdTree<pcl::PointXYZRGB> next_kdtree;
            next_kdtree.setInputCloud(next_cloud);

            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

            // first query nearest neighbor using kmeans result
            std::vector<int> next_nn_idx;
            std::vector<double> next_nn_distance;

            std::vector<pcl::PointXYZ> skelton_point_mv; // average motion vector of skelton point
            for (int i = 0; i < skelton_next_cloud->points.size(); i++)
            {
                pcl::PointXYZ temp;
                temp.x = 0;
                temp.y = 0;
                temp.z = 0;
                skelton_point_mv.push_back(temp);
            }

            for (int idx = 0; idx < next_cloud->points.size(); idx++)
            {
                int matching_idx = 0;
                double color_diff = pow(next_cloud->points[idx].r - kmeans_points[0].r, 2) +
                                    pow(next_cloud->points[idx].g - kmeans_points[0].g, 2) +
                                    pow(next_cloud->points[idx].b - kmeans_points[0].b, 2);
                for (int i = 1; i < kmeans_points.size(); i++)
                {
                    double new_color_diff = pow(next_cloud->points[idx].r - kmeans_points[i].r, 2) +
                                            pow(next_cloud->points[idx].g - kmeans_points[i].g, 2) +
                                            pow(next_cloud->points[idx].b - kmeans_points[i].b, 2);
                    if (new_color_diff < color_diff)
                    {
                        color_diff = new_color_diff;
                        matching_idx = i;
                    }
                }

                std::vector<int> nearest_idx;
                std::vector<float> nearest_dist;
                kmeans_kdtrees[matching_idx].nearestKSearch(next_cloud->points[idx], 1, nearest_idx, nearest_dist);

                next_nn_idx.push_back(nearest_idx[0]);
                next_nn_distance.push_back(nearest_dist[0]);

                if (next_skelton_search_radius[next_cloud_nn_skelton_idx[idx]] < nearest_dist[0])
                {
                    next_skelton_search_radius[next_cloud_nn_skelton_idx[idx]] = nearest_dist[0];
                }
            }

            std::vector<double> nn_distance;

            nn_distance = next_nn_distance;
            // initialize original index locations
            std::vector<int> sort_idx(nn_distance.size());
            iota(sort_idx.begin(), sort_idx.end(), 0);

            // sort indexes based on comparing values in v
            // using std::stable_sort instead of std::sort
            // to avoid unnecessary index re-orderings
            // when v contains elements of equal values
            stable_sort(sort_idx.begin(), sort_idx.end(),
                        [&nn_distance](int i1, int i2)
                        { return nn_distance[i1] < nn_distance[i2]; });

            for (auto idx : sort_idx)
            {
                MyMatching temp;
                std::vector<int> prev_pointIdxRadiusSearch;
                std::vector<float> prev_pointRadiusSquaredDistance;

                float search_radius = next_skelton_search_radius[next_cloud_nn_skelton_idx[idx]];
                // magnification
                prev_kdtree.radiusSearchT(next_cloud->points[idx], double(magnification * search_radius + 1), prev_pointIdxRadiusSearch, prev_pointRadiusSquaredDistance);

                for (int i = 0; i < prev_pointIdxRadiusSearch.size(); i++)
                {
                    if (abs(next_cloud->points[idx].r - prev_cloud->points[prev_pointIdxRadiusSearch[i]].r) < color_dist &&
                        abs(next_cloud->points[idx].g - prev_cloud->points[prev_pointIdxRadiusSearch[i]].g) < color_dist &&
                        abs(next_cloud->points[idx].b - prev_cloud->points[prev_pointIdxRadiusSearch[i]].b) < color_dist)
                    {
                        temp.match_idx_list.push_back(prev_pointIdxRadiusSearch[i]);
                        temp.match_dist_list.push_back(prev_pointRadiusSquaredDistance[i]);
                    }
                }

                if (temp.match_idx_list.size() == 0)
                {
                    temp.match_idx_list.push_back(prev_pointIdxRadiusSearch[0]);
                    temp.match_dist_list.push_back(prev_pointRadiusSquaredDistance[0]);
                }

                double min_distance = -1;
                int matching_idx = -1;
                double new_gamma = 0.00721688;
                double utility_alpha = 0.9;
                double utility_beta = 0.1;

                for (int i = 0; i < temp.match_idx_list.size(); i++)
                {
                    double cur_color_dist = sqrt(pow(((int)prev_cloud->points[temp.match_idx_list[i]].r - next_cloud->points[idx].r), 2) +
                                                 pow(((int)prev_cloud->points[temp.match_idx_list[i]].g - next_cloud->points[idx].g), 2) +
                                                 pow(((int)prev_cloud->points[temp.match_idx_list[i]].b - next_cloud->points[idx].b), 2));

                    double cur_total_dist = utility_alpha * sqrt(temp.match_dist_list[i]) +
                                            utility_beta * cur_color_dist +
                                            new_gamma * match_time[temp.match_idx_list[i]];

                    if (min_distance > cur_total_dist || min_distance < 0)
                    {
                        min_distance = cur_total_dist;
                        matching_idx = temp.match_idx_list[i];
                    }
                }

                if (matching_idx != -1)
                    match_time[matching_idx] += 1;

                pipeline_obj.matching_table[idx] = matching_idx;
            }
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
