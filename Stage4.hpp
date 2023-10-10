// Prediction

#ifndef STAGE4_HPP
#define STAGE4_HPP

class Stage4_Point_Based : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud

public:
    Stage4_Point_Based(int my_mode);
    void run(Pipeline_Object &pipeline_obj);
};

class Stage4_Cube_Based : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud
    int iterate_round;

public:
    Stage4_Cube_Based(int my_mode, int my_iterate_round);
    void run(Pipeline_Object &pipeline_obj);
};

class Stage4_Neighboring_Based : public Stage
{
private:
    int mode; // 1 for ds1_point_cloud, 2 for ds2_point_cloud

public:
    Stage4_Neighboring_Based(int my_mode);
    void run(Pipeline_Object &pipeline_obj);
};
#endif