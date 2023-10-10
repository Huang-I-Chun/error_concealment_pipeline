#ifndef UTILITY_HPP
#define UTILITY_HPP

class MyPoint
{
public:
    int x;
    int y;
    int z;
    MyPoint(int my_x, int my_y, int my_z);
};

struct MyPointCompare
{
    bool operator()(const MyPoint &lhs, const MyPoint &rhs) const;
};

int find_center(int x, int cube_length);

std::vector<int> sort_indexes(const std::vector<std::vector<double>> &v);

class MyMatching
{
    // 宣告 public 成員
public:
    std::vector<int> match_idx_list;
    std::vector<float> match_dist_list;
};

bool from_prev_or_next(pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_cloud);
#endif