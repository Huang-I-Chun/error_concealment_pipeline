// Prediction

class Stage4_PB // point based
{
private:
public:
    double resolution = 1024; // initial resolution, actually, this is the height of point cloud
    double t = 0.5            // the interpolate value, 0 < t < 1

        void
        run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
            const std::vector<std::vector<int>> &motion_estimation,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud, );
};

class Stage4_CB // cube based
{
private:
public:
    double resolution = 1024; // initial resolution, actually, this is the height of point cloud
    double cube_length = 128;
    int iteration_round = 1;
    double t = 0.5 // the interpolate value, 0 < t < 1

        void
        run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
            const std::vector<std::vector<int>> &motion_estimation,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud, );
};

class Stage4_NB // Neighboring based
{
private:
public:
    double resolution = 1024; // initial resolution, actually, this is the height of point cloud
    double cube_length = 128;
    double t = 0.5 // the interpolate value, 0 < t < 1

        void
        run(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &prev_cloud,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &next_cloud,
            const std::vector<std::vector<int>> &motion_estimation,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &predict_point_cloud, );
};