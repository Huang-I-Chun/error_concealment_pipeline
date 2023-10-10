// Motion Estimation
#ifndef STAGE3_HPP
#define STAGE3_HPP

class Stage3_Cube_Estimation : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud
    int cube_length;

public:
    Stage3_Cube_Estimation(int my_mode, int my_cube_length);
    void run(Pipeline_Object &pipeline_obj);
};

class Stage3_Point_Estimation : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud

public:
    Stage3_Point_Estimation(int my_mode);
    void run(Pipeline_Object &pipeline_obj);
};
#endif