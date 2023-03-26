// Motion Estimation

class Stage3_Point_Estimation
{
private:
public:
    double resolution = 1024; // initial resolution, actually, this is the height of point cloud

    void run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
             const std::vector<int> &matching_table,
             std::vector<std::vector<int>> &motion_estimation, );
};

class Stage3_Cube_Estimation
{
private:
public:
    double resolution = 1024; // initial resolution, actually, this is the height of point cloud
    double cube_length = 128;

    void run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
             const std::vector<int> &matching_table,
             std::vector<std::vector<int>> &motion_estimation, );
};
