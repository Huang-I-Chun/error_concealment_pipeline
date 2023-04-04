// Motion Estimation

class Stage3_Cube_Estimation : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud
    int cube_length;

public:
    Stage3_Cube_Estimation(int my_mode, int my_cube_length) : mode(my_mode), cube_length(my_cube_length)
    {
    }

    void run(Pipeline_Object &pipeline_obj)
    {

        pipeline_obj.cube_length = cube_length;

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

        std::map<MyPoint, std::vector<int>, MyPointCompare> ds_block_list; // key: point xyz, value: query point index

        for (int x = cube_length / 2; x < pipeline_obj.resolution; x += cube_length)
        {
            for (int y = cube_length / 2; y < pipeline_obj.resolution; y += cube_length)
            {
                for (int z = cube_length / 2; z < pipeline_obj.resolution; z += cube_length)
                {
                    ds_block_list[MyPoint(x, y, z)] = std::vector<int>();
                }
            }
        }

        for (int w = 0; w < ds_next_cloud->points.size(); w++)
        {
            ds_block_list[MyPoint(find_center(ds_next_cloud->points[w].x, cube_length), find_center(ds_next_cloud->points[w].y, cube_length), find_center(ds_next_cloud->points[w].z, cube_length))].push_back(w);
        }

        std::vector<MyPoint> non_empty_block_center; // save those center points which query points > 10

        for (int x = cube_length / 2; x < pipeline_obj.resolution; x += cube_length)
        {
            for (int y = cube_length / 2; y < pipeline_obj.resolution; y += cube_length)
            {
                for (int z = cube_length / 2; z < pipeline_obj.resolution; z += cube_length)
                {
                    if (ds_block_list[MyPoint(x, y, z)].size() == 0)
                        continue;

                    non_empty_block_center.push_back(MyPoint(x, y, z));

                    double vector_x = 0.0, vector_y = 0.0, vector_z = 0.0, vector_r = 0.0, vector_g = 0.0, vector_b = 0.0;
                    double correct_matching_num = 0.0;
                    for (int idx = 0; idx < ds_block_list[MyPoint(x, y, z)].size(); idx++)
                    {
                        if (pipeline_obj.matching_table[ds_block_list[MyPoint(x, y, z)][idx]] != -1)
                        {
                            correct_matching_num += 1;
                            vector_x += (ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].x - (pipeline_obj.temporal_index * ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].x + (1 - pipeline_obj.temporal_index) * ds_prev_cloud->points[pipeline_obj.matching_table[ds_block_list[MyPoint(x, y, z)][idx]]].x));
                            vector_y += (ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].y - (pipeline_obj.temporal_index * ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].y + (1 - pipeline_obj.temporal_index) * ds_prev_cloud->points[pipeline_obj.matching_table[ds_block_list[MyPoint(x, y, z)][idx]]].y));
                            vector_z += (ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].z - (pipeline_obj.temporal_index * ds_next_cloud->points[ds_block_list[MyPoint(x, y, z)][idx]].z + (1 - pipeline_obj.temporal_index) * ds_prev_cloud->points[pipeline_obj.matching_table[ds_block_list[MyPoint(x, y, z)][idx]]].z));
                        }
                    }

                    vector_x = vector_x / correct_matching_num;
                    vector_y = vector_y / correct_matching_num;
                    vector_z = vector_z / correct_matching_num;
                    vector_r = 0;
                    vector_g = 0;
                    vector_b = 0;

                    std::vector<double> temp;
                    temp.push_back(vector_x);
                    temp.push_back(vector_y);
                    temp.push_back(vector_z);
                    temp.push_back(vector_r);
                    temp.push_back(vector_g);
                    temp.push_back(vector_b);

                    pipeline_obj.non_empty_block_vector.push_back(temp);
                }
            }
        }

        if (_next != NULL)
        {
            _next->run(pipeline_obj);
        }
    }
};

// class Stage3_Point_Estimation
// {
// private:
// public:
//     double resolution = 1024; // initial resolution, actually, this is the height of point cloud

//     void run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
//              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
//              const std::vector<int> &matching_table,
//              std::vector<std::vector<int>> &motion_estimation, );
// };

// class Stage3_Cube_Estimation
// {
// private:
// public:
//     double resolution = 1024; // initial resolution, actually, this is the height of point cloud
//     double cube_length = 128;

//     void run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
//              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
//              const std::vector<int> &matching_table,
//              std::vector<std::vector<int>> &motion_estimation, );
// };
