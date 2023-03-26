#include <iostream>
#include <stdio.h>
#include <stddef.h>
#include <vector>
#include <algorithm>
#include <string>
#include <ctime>
#include <bits/stdc++.h>
#include <math.h>
#include <chrono>
#include <map>
#include <numeric>
#include <set>
#include <utility>
#include <stdlib.h>

#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

#include "base.hpp"
#include "Stage1.hpp"
// #include "Stage2.hpp"
// #include "Stage3.hpp"
// #include "Stage4.hpp"
// #include "Stage5.hpp"

// class Pipeline
// {
// private:
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // input origin point cloud
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // input origin point cloud

//     // Output variable of each stage
//     // Stage 1 Preprocessing: After pre-process point cloud
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds1_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // using downsample1
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds1_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // using downsample1
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds2_prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // using downsample2
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds2_next_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // using downsample2

//     // Stage 2 Matching: Obtain matching table
//     std::vector<int> matching_table;

//     // Stage 3 Motion Estimation: Motion estimation per point/center
//     std::vector<std::vector<int>> motion_estimation;

//     // Stage 4 Prediction:
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr predict_point_cloud;

//     // Stage 5 Post-Processing:
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // output point cloud

// public:
//     std::string prev_filename = "";
//     std::string next_filename = "";
//     std::string output_filename = "";

//     // Each Stage instance
//     Stage1_Downsample1 stage1_downsample1;
//     Stage1_Downsample2 stage1_downsample2;
//     Stage1_None stage1_none;

//     Stage2_NN stage2_nn;
//     Stage2_QR stage2_qr;
//     Stage2_CFDS stage2_cfds;

//     Stage3_Cube_Estimation stage3_cube_estimation;
//     Stage3_Point_Estimation stage3_point_estimation;

//     Stage4_PB stage4_pb;
//     Stage4_CB stage4_cb;
//     Stage4_NB stage4_nb;

//     Stage5_None stage5_none;
//     Stage5_PSR stage5_psr;

//     // run function
//     void run_stage2(str::string method_name)
//     {
//         Stag2.run(method_name);
//         // if (method_name == "NN")
//         // {
//         //     stage2_nn.run(ds1_prev_cloud, ds2_next_cloud, matching_table);
//         // }
//         // else if (method_name == "QR")
//         // {
//         //     stage2_qr.run(ds1_prev_cloud, ds2_next_cloud, matching_table);
//         // }
//         // else if (method_name == "CFDS")
//         // {
//         //     stage2_cfds.run(ds1_prev_cloud, ds2_next_cloud, matching_table);
//         // }
//     };

//     // Output Each Stage's Value
//     void output_Stage2(std::string filename)
//     {
//         // output std::vector<int> matching_table into txt;
//     }
// };

// ---------------------------------

// if needed, I can use args

// ---------------------------------

int main()
{
    std::string prev_filename = "/mnt/m2_ssd/PC_Concealment/experiments_ssd/loot_vox10_ai/no_drop/loot_vox10_ai_dec_1000.ply";
    std::string next_filename = "/mnt/m2_ssd/PC_Concealment/experiments_ssd/loot_vox10_ai/no_drop/loot_vox10_ai_dec_1003.ply";
    Pipeline_Object my_obj(prev_filename, next_filename, 0.5);


    Stage1_None my_stage1;
    my_stage1.run(my_obj);

    // Pipeline my_pipeline;

    // // setting filename
    // my_pipeline.prev_filename = "frame1.ply";
    // my_pipeline.next_filename = "frame3.ply";
    // my_pipeline.output_filename = "frame2.ply";

    // // setting parameter of each stage
    // my_pipeline.stage2_nn.resolution = 1024;
    // my_pipeline.stage3_cube_estimation.resolution = 1024;
    // my_pipeline.stage3_cube_estimation.cube_length = 128;
    // my_pipeline.stage4_cb.resolution = 1024;
    // my_pipeline.stage4_cb.cube_length = 128;
    // my_pipeline.stage4_cb.iteration_round = 1;
    // my_pipeline.stage4_cb.t = 0.5;

    // // Start Concealemnt
    // my_pipeline.run_stage1("None");
    // my_pipeline.run_stage2("NN");
    // my_pipeline.run_stage3("Cube_Estimation");
    // my_pipeline.run_stage4("CB");
    // my_pipeline.run_stage5("None");

    // // if user want to get each stage's output
    // my_pipeline.output_Stage2("stage2_filename.txt");

    // my_pipeline.set_stage2(Stage2_LL);

    return 0;
}
