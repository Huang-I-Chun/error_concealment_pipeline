// Post-Processing

#ifndef STAGE5_HPP
#define STAGE5_HPP

class Stage5_None : public Stage
{
private:
    std::string output_filename;

public:
    Stage5_None(std::string my_output_filename);
    void run(Pipeline_Object &pipeline_obj);
};

// class Stage5_PSR
// {
// private:
// public:
//     void run(const std::string output_filename,
//              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud,
//              pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud);
// };
#endif