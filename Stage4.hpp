// Prediction

class Stage4_Cube_Estimation : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud
    int iterate_round;

public:
    Stage4_Cube_Estimation(int my_mode, int my_iterate_round) : mode(my_mode), iterate_round(my_iterate_round)
    {
    }

    void run(Pipeline_Object &pipeline_obj)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        if (mode == 1)
        {
            prev_cloud = pipeline_obj.ds1_prev_cloud;
            next_cloud = pipeline_obj.ds1_next_cloud;
        }
        else if (mode == 2)
        {
            prev_cloud = pipeline_obj.ds2_prev_cloud;
            next_cloud = pipeline_obj.ds2_next_cloud;
        }

        std::map<MyPoint, std::vector<int>, MyPointCompare> ds_block_list; // key: point xyz, value: query point index

        for (int x = pipeline_obj.cube_length / 2; x < pipeline_obj.resolution; x += pipeline_obj.cube_length)
        {
            for (int y = pipeline_obj.cube_length / 2; y < pipeline_obj.resolution; y += pipeline_obj.cube_length)
            {
                for (int z = pipeline_obj.cube_length / 2; z < pipeline_obj.resolution; z += pipeline_obj.cube_length)
                {
                    ds_block_list[MyPoint(x, y, z)] = std::vector<int>();
                }
            }
        }

        for (int w = 0; w < next_cloud->points.size(); w++)
        {
            ds_block_list[MyPoint(find_center(next_cloud->points[w].x, pipeline_obj.cube_length), find_center(next_cloud->points[w].y, pipeline_obj.cube_length), find_center(next_cloud->points[w].z, pipeline_obj.cube_length))].push_back(w);
        }

        std::vector<MyPoint> non_empty_block_center; // save those center points which query points > 10

        for (int x = pipeline_obj.cube_length / 2; x < pipeline_obj.resolution; x += pipeline_obj.cube_length)
        {
            for (int y = pipeline_obj.cube_length / 2; y < pipeline_obj.resolution; y += pipeline_obj.cube_length)
            {
                for (int z = pipeline_obj.cube_length / 2; z < pipeline_obj.resolution; z += pipeline_obj.cube_length)
                {
                    if (ds_block_list[MyPoint(x, y, z)].size() == 0)
                        continue;

                    non_empty_block_center.push_back(MyPoint(x, y, z));
                }
            }
        }

        // block_list will use in prediction
        std::map<MyPoint, std::vector<int>, MyPointCompare> block_list; // key: point xyz, value: query point index

        for (int x = pipeline_obj.cube_length / 2; x < pipeline_obj.resolution; x += pipeline_obj.cube_length)
        {
            for (int y = pipeline_obj.cube_length / 2; y < pipeline_obj.resolution; y += pipeline_obj.cube_length)
            {
                for (int z = pipeline_obj.cube_length / 2; z < pipeline_obj.resolution; z += pipeline_obj.cube_length)
                {
                    block_list[MyPoint(x, y, z)] = std::vector<int>();
                }
            }
        }

        for (int w = 0; w < next_cloud->points.size(); w++)
        {
            block_list[MyPoint(find_center(next_cloud->points[w].x, pipeline_obj.cube_length), find_center(next_cloud->points[w].y, pipeline_obj.cube_length), find_center(next_cloud->points[w].z, pipeline_obj.cube_length))].push_back(w);
        }

        int round = 0;
        std::vector<double> old_length_multiplier;
        for (int i = 0; i < non_empty_block_center.size(); i++)
        {
            old_length_multiplier.push_back(1.0);
        }

        for (; round < iterate_round; round++)
        {
            // check original(before motion) non empty block center point's adjacent situation
            std::vector<std::vector<int>> origin_adjacent_center;

            for (int i = 0; i < non_empty_block_center.size(); i++)
            {
                std::vector<int> temp;
                for (int j = 0; j < non_empty_block_center.size(); j++)
                {
                    if ((non_empty_block_center[i].x + (pipeline_obj.cube_length + 1) > non_empty_block_center[j].x && non_empty_block_center[i].x - (pipeline_obj.cube_length + 1) < non_empty_block_center[j].x) &&
                        (non_empty_block_center[i].y + (pipeline_obj.cube_length + 1) > non_empty_block_center[j].y && non_empty_block_center[i].y - (pipeline_obj.cube_length + 1) < non_empty_block_center[j].y) &&
                        (non_empty_block_center[i].z + (pipeline_obj.cube_length + 1) > non_empty_block_center[j].z && non_empty_block_center[i].z - (pipeline_obj.cube_length + 1) < non_empty_block_center[j].z))
                    {
                        temp.push_back(j);
                    }
                }
                origin_adjacent_center.push_back(temp);
            }

            // check result(after motion) non empty block center point's adjacent situation
            std::vector<std::vector<int>> result_adjacent_center;
            for (int i = 0; i < non_empty_block_center.size(); i++)
            {
                std::vector<int> temp;
                double curr_x = non_empty_block_center[i].x + pipeline_obj.non_empty_block_vector[i][0];
                double curr_y = non_empty_block_center[i].y + pipeline_obj.non_empty_block_vector[i][1];
                double curr_z = non_empty_block_center[i].z + pipeline_obj.non_empty_block_vector[i][2];

                for (int j = 0; j < non_empty_block_center.size(); j++)
                {
                    double ref_x = non_empty_block_center[j].x + pipeline_obj.non_empty_block_vector[j][0];
                    double ref_y = non_empty_block_center[j].y + pipeline_obj.non_empty_block_vector[j][1];
                    double ref_z = non_empty_block_center[j].z + pipeline_obj.non_empty_block_vector[j][2];
                    // std::cout << "------------" << std::endl;
                    // std::cout << curr_x << " " << curr_y << " " << curr_z << std::endl;
                    // std::cout << ref_x << " " << ref_y << " " << ref_z << std::endl;
                    if ((curr_x + (pipeline_obj.cube_length + 1) > ref_x && curr_x - (pipeline_obj.cube_length + 1) < ref_x) &&
                        (curr_y + (pipeline_obj.cube_length + 1) > ref_y && curr_y - (pipeline_obj.cube_length + 1) < ref_y) &&
                        (curr_z + (pipeline_obj.cube_length + 1) > ref_z && curr_z - (pipeline_obj.cube_length + 1) < ref_z))
                    {
                        temp.push_back(j);
                    }
                }
                result_adjacent_center.push_back(temp);
            }

            // get the index from large to small of the vector magnitude
            std::vector<int> sort_idx = sort_indexes(pipeline_obj.non_empty_block_vector);

            // compare before and after center point adjacent situation and get multiplier
            std::vector<double> length_multiplier(sort_idx.size());
            std::set<std::pair<int, int>> explored_pair;
            bool is_change = false;

            for (int i = 0; i < sort_idx.size(); i++)
            {
                std::vector<int> disappear_idx;
                // std::cout << 2 << std::endl;
                for (int j = 0; j < origin_adjacent_center[sort_idx[i]].size(); j++)
                {
                    if (std::find(result_adjacent_center[sort_idx[i]].begin(), result_adjacent_center[sort_idx[i]].end(), origin_adjacent_center[sort_idx[i]][j]) == result_adjacent_center[sort_idx[i]].end())
                    {
                        if (sort_idx[i] > origin_adjacent_center[sort_idx[i]][j] && explored_pair.find(std::make_pair(origin_adjacent_center[sort_idx[i]][j], sort_idx[i])) == explored_pair.end())
                        {
                            disappear_idx.push_back(origin_adjacent_center[sort_idx[i]][j]);
                        }
                        else if (explored_pair.find(std::make_pair(sort_idx[i], origin_adjacent_center[sort_idx[i]][j])) == explored_pair.end())
                        {
                            disappear_idx.push_back(origin_adjacent_center[sort_idx[i]][j]);
                        }
                    }
                }
                if (disappear_idx.size() == 0)
                {
                    length_multiplier[sort_idx[i]] = (old_length_multiplier[sort_idx[i]]);
                    continue;
                }
                is_change = true;
                // std::cout << sort_idx[i] << " " << non_empty_block_center.size() << std::endl;

                double max_distance = abs((non_empty_block_center[sort_idx[i]].x + pipeline_obj.non_empty_block_vector[sort_idx[i]][0]) - (non_empty_block_center[disappear_idx[0]].x + pipeline_obj.non_empty_block_vector[disappear_idx[0]][0]));
                for (int j = 0; j < disappear_idx.size(); j++)
                {
                    // std::cout << pipeline_obj.non_empty_block_vector[disappear_idx[j]].size() << " " << non_empty_block_center.size() << std::endl;
                    if (max_distance < abs((non_empty_block_center[sort_idx[i]].x + pipeline_obj.non_empty_block_vector[sort_idx[i]][0]) - (non_empty_block_center[disappear_idx[j]].x + pipeline_obj.non_empty_block_vector[disappear_idx[j]][0])))
                        max_distance = abs((non_empty_block_center[sort_idx[i]].x + pipeline_obj.non_empty_block_vector[sort_idx[i]][0]) - (non_empty_block_center[disappear_idx[j]].x + pipeline_obj.non_empty_block_vector[disappear_idx[j]][0]));
                    if (max_distance < abs((non_empty_block_center[sort_idx[i]].y + pipeline_obj.non_empty_block_vector[sort_idx[i]][1]) - (non_empty_block_center[disappear_idx[j]].y + pipeline_obj.non_empty_block_vector[disappear_idx[j]][1])))
                        max_distance = abs((non_empty_block_center[sort_idx[i]].y + pipeline_obj.non_empty_block_vector[sort_idx[i]][1]) - (non_empty_block_center[disappear_idx[j]].y + pipeline_obj.non_empty_block_vector[disappear_idx[j]][1]));
                    if (max_distance < abs((non_empty_block_center[sort_idx[i]].z + pipeline_obj.non_empty_block_vector[sort_idx[i]][2]) - (non_empty_block_center[disappear_idx[j]].z + pipeline_obj.non_empty_block_vector[disappear_idx[j]][2])))
                        max_distance = abs((non_empty_block_center[sort_idx[i]].z + pipeline_obj.non_empty_block_vector[sort_idx[i]][2]) - (non_empty_block_center[disappear_idx[j]].z + pipeline_obj.non_empty_block_vector[disappear_idx[j]][2]));
                }
                // std::cout << 4 << std::endl;
                // std::cout << max_distance / pipeline_obj.cube_length << " " << old_length_multiplier[sort_idx[i]] << std::endl;
                if ((max_distance / pipeline_obj.cube_length) > old_length_multiplier[sort_idx[i]])
                {
                    length_multiplier[sort_idx[i]] = (max_distance / pipeline_obj.cube_length);
                }
                else
                {
                    length_multiplier[sort_idx[i]] = (old_length_multiplier[sort_idx[i]]);
                }

                for (int j = 0; j < disappear_idx.size(); j++)
                {
                    if (sort_idx[i] > disappear_idx[j])
                    {
                        explored_pair.insert(std::make_pair(disappear_idx[j], sort_idx[i]));
                    }
                    else
                    {
                        explored_pair.insert(std::make_pair(sort_idx[i], disappear_idx[j]));
                    }
                }
            }

            // check if length_multiplier is same as last turn

            bool multiplier_same = true;
            for (int i = 0; i < length_multiplier.size(); i++)
            {
                if (abs(old_length_multiplier[i] - length_multiplier[i]) > 0.0001)
                {
                    multiplier_same = false;
                    break;
                }
            }
            if (multiplier_same && round != 0)
            {
                break;
            }
            else
            {
                old_length_multiplier.clear();
                for (int i = 0; i < length_multiplier.size(); i++)
                {
                    old_length_multiplier.push_back(length_multiplier[i]);
                }
            }

            // do cheb again
            pipeline_obj.predict_point_cloud->points.clear();

            for (int i = 0; i < non_empty_block_center.size(); i++)
            {
                // std::cout << block_list[MyPoint(x, y, z)].size() << std::endl;
                if (block_list[non_empty_block_center[i]].size() == 0)
                    continue;
                double denominator = 0;
                double vector_x = 0.0, vector_y = 0.0, vector_z = 0.0, vector_r = 0.0, vector_g = 0.0, vector_b = 0.0;
                for (int idx = 0; idx < block_list[non_empty_block_center[i]].size(); idx++)
                {
                    std::vector<int> prev_pointIdxRadiusSearch;
                    std::vector<float> prev_pointRadiusSquaredDistance;

                    prev_pointIdxRadiusSearch.push_back(pipeline_obj.matching_table[block_list[non_empty_block_center[i]][idx]]);

                    double distance_reciprocal = 1.0 / (sqrt(pow(next_cloud->points[block_list[non_empty_block_center[i]][idx]].x - prev_cloud->points[prev_pointIdxRadiusSearch[0]].x, 2) +
                                                             pow(next_cloud->points[block_list[non_empty_block_center[i]][idx]].y - prev_cloud->points[prev_pointIdxRadiusSearch[0]].y, 2) +
                                                             pow(next_cloud->points[block_list[non_empty_block_center[i]][idx]].z - prev_cloud->points[prev_pointIdxRadiusSearch[0]].z, 2)) +
                                                        pipeline_obj.cube_length / 2);
                    distance_reciprocal = 1;
                    denominator += distance_reciprocal;

                    vector_x += distance_reciprocal * (next_cloud->points[block_list[non_empty_block_center[i]][idx]].x - (pipeline_obj.temporal_index * next_cloud->points[block_list[non_empty_block_center[i]][idx]].x + (1 - pipeline_obj.temporal_index) * prev_cloud->points[prev_pointIdxRadiusSearch[0]].x));
                    vector_y += distance_reciprocal * (next_cloud->points[block_list[non_empty_block_center[i]][idx]].y - (pipeline_obj.temporal_index * next_cloud->points[block_list[non_empty_block_center[i]][idx]].y + (1 - pipeline_obj.temporal_index) * prev_cloud->points[prev_pointIdxRadiusSearch[0]].y));
                    vector_z += distance_reciprocal * (next_cloud->points[block_list[non_empty_block_center[i]][idx]].z - (pipeline_obj.temporal_index * next_cloud->points[block_list[non_empty_block_center[i]][idx]].z + (1 - pipeline_obj.temporal_index) * prev_cloud->points[prev_pointIdxRadiusSearch[0]].z));
                }

                vector_x = vector_x / block_list[non_empty_block_center[i]].size();
                vector_y = vector_y / block_list[non_empty_block_center[i]].size();
                vector_z = vector_z / block_list[non_empty_block_center[i]].size();
                vector_r = vector_r / block_list[non_empty_block_center[i]].size();
                vector_g = vector_g / block_list[non_empty_block_center[i]].size();
                vector_b = vector_b / block_list[non_empty_block_center[i]].size();

                for (int idx = 0; idx < block_list[non_empty_block_center[i]].size(); idx++)
                {
                    pcl::PointXYZRGB interpolate_point;
                    interpolate_point.x = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].x - vector_x;
                    interpolate_point.y = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].y - vector_y;
                    interpolate_point.z = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].z - vector_z;
                    interpolate_point.r = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].r - vector_r;
                    interpolate_point.g = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].g - vector_g;
                    interpolate_point.b = (int)next_cloud->points[block_list[non_empty_block_center[i]][idx]].b - vector_b;
                    pipeline_obj.predict_point_cloud->push_back(interpolate_point);
                }

                std::vector<double> temp;
                temp.push_back(vector_x);
                temp.push_back(vector_y);
                temp.push_back(vector_z);
                temp.push_back(vector_r);
                temp.push_back(vector_g);
                temp.push_back(vector_b);

                pipeline_obj.non_empty_block_vector.push_back(temp);
            }

            // std::cout << output_filename.substr(0, output_filename.size() - 4) + "_" + std::string(number) + ".ply" << std::endl;
            // pcl::io::savePLYFileASCII(output_filename.substr(0, output_filename.size() - 4) + "_" + std::string(number) + ".ply", *pipeline_obj.predict_point_cloud);
            if (round == iterate_round - 1)
            {
                break;
            }

            if (!is_change)
            {
                break;
            }
        }

        if (_next != NULL)
        {
            _next->run(pipeline_obj);
        }
    }
};

// class Stage4_PB // point based
// {
// private:
// public:
//     double pipeline_obj.resolution = 1024; // initial pipeline_obj.resolution, actually, this is the height of point cloud
//     double t = 0.5            // the interpolate value, 0 < t < 1

//         void run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
//             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
//             const std::vector<std::vector<int>> &motion_estimation,
//             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud, );
// };

// class Stage4_CB // cube based
// {
// private:
// public:
//     double pipeline_obj.resolution = 1024; // initial pipeline_obj.resolution, actually, this is the height of point cloud
//     double pipeline_obj.cube_length = 128;
//     int iteration_round = 1;
//     double t = 0.5 // the interpolate value, 0 < t < 1

//         void
//         run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
//             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
//             const std::vector<std::vector<int>> &motion_estimation,
//             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud, );
// };

// class Stage4_NB // Neighboring based
// {
// private:
// public:
//     double pipeline_obj.resolution = 1024; // initial pipeline_obj.resolution, actually, this is the height of point cloud
//     double pipeline_obj.cube_length = 128;
//     double t = 0.5 // the interpolate value, 0 < t < 1

//         void
//         run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
//             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
//             const std::vector<std::vector<int>> &motion_estimation,
//             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud, );
// };