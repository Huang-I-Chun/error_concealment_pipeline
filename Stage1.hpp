// Pre-Processing
#ifndef STAGE1_HPP
#define STAGE1_HPP

class Stage1_None : public Stage
{
private:
public:
    Stage1_None();
    void run(Pipeline_Object &pipeline_obj);
};

class Stage1_Downsample : public Stage
{
private:
public:
    Stage1_Downsample(double ds1, double ds2);
    void run(Pipeline_Object &pipeline_obj);
    double downsample_radius1; // initial downsample voxel length
    double downsample_radius2; // initial downsample voxel length
};

#endif