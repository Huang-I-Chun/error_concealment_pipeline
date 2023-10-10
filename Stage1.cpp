// Pre-Processing
#include "base.hpp"
#include "Stage1.hpp"

Stage1_None::Stage1_None()
{
}

void Stage1_None::run(Pipeline_Object &pipeline_obj)
{
    pipeline_obj.ds1_prev_cloud = pipeline_obj.readfile_prev_cloud;
    pipeline_obj.ds1_next_cloud = pipeline_obj.readfile_next_cloud;
    pipeline_obj.ds2_prev_cloud = pipeline_obj.readfile_prev_cloud;
    pipeline_obj.ds2_next_cloud = pipeline_obj.readfile_next_cloud;

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}

Stage1_Downsample::Stage1_Downsample(double ds1, double ds2)
{
    downsample_radius1 = ds1;
    downsample_radius2 = ds2;
}

void Stage1_Downsample::run(Pipeline_Object &pipeline_obj)
{
    // downsample1
    pcl::VoxelGrid<pcl::PointXYZRGB> preprocess_downsample;
    preprocess_downsample.setInputCloud(pipeline_obj.readfile_prev_cloud);
    preprocess_downsample.setLeafSize(downsample_radius1, downsample_radius1, downsample_radius1);
    preprocess_downsample.filter(*pipeline_obj.ds1_prev_cloud);

    pcl::VoxelGrid<pcl::PointXYZRGB> preprocess_downsample2;
    preprocess_downsample2.setInputCloud(pipeline_obj.readfile_next_cloud);
    preprocess_downsample2.setLeafSize(downsample_radius1, downsample_radius1, downsample_radius1);
    preprocess_downsample2.filter(*pipeline_obj.ds1_next_cloud);

    // downsample2
    pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
    sor1.setInputCloud(pipeline_obj.readfile_prev_cloud);
    sor1.setLeafSize(downsample_radius2, downsample_radius2, downsample_radius2);
    sor1.filter(*pipeline_obj.ds2_prev_cloud);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor2;
    sor2.setInputCloud(pipeline_obj.readfile_next_cloud);
    sor2.setLeafSize(downsample_radius2, downsample_radius2, downsample_radius2);
    sor2.filter(*pipeline_obj.ds2_next_cloud);

    if (_next != NULL)
    {
        _next->run(pipeline_obj);
    }
}