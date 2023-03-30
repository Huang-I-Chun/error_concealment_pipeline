class MyPoint
{
    // 宣告 public 成員
public:
    int x;
    int y;
    int z;
    MyPoint(int my_x, int my_y, int my_z) : x(my_x), y(my_y), z(my_z) {}
};

struct MyPointCompare
{
    bool operator()(const MyPoint &lhs, const MyPoint &rhs) const
    {
        // int num = (resolution - cube_length / 2) / cube_length + 1;
        int num = 100;
        return lhs.x + lhs.y * num + lhs.z * num * num < rhs.x + rhs.y * num + rhs.z * num * num;
    }
};

int find_center(int x, int cube_length)
{
    int times = (x - cube_length / 2) / cube_length;
    if (times < 0)
    {
        return cube_length / 2;
    }
    else
    {
        int big_bound = cube_length / 2 + (times + 1) * cube_length;
        int small_bound = cube_length / 2 + times * cube_length;
        if (abs(big_bound - x) > abs(x - small_bound))
        {
            return small_bound;
        }
        else
        {
            return big_bound;
        }
    }
}

std::vector<int> sort_indexes(const std::vector<std::vector<double>> &v)
{

    // initialize original index locations
    std::vector<int> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    std::stable_sort(idx.begin(), idx.end(),
                     [&v](int i1, int i2)
                     { 
                    int len1 = v[i1][0] * v[i1][0] + v[i1][1] * v[i1][1] + v[i1][2] * v[i1][2];
                    int len2 = v[i2][0] * v[i2][0] + v[i2][1] * v[i2][1] + v[i2][2] * v[i2][2];
                    return len1 > len2; });

    return idx;
}
