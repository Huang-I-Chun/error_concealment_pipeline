// Post-Processing

class Stage5_None
{
private:
public:
    void run(const std::string output_filename,
             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud);
};

// class Stage5_PSR
// {
// private:
// public:
//     void run(const std::string output_filename,
//              const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud,
//              pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud);
// };