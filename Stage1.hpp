// Pre-Processing


class Stage1_None: public Stage
{
private:
public:

    Stage1_None(){
    }

    void run(Pipeline_Object& pipeline_obj){
        pipeline_obj.ds1_prev_cloud = pipeline_obj.readfile_prev_cloud;
        pipeline_obj.ds1_next_cloud = pipeline_obj.readfile_next_cloud;
        pipeline_obj.ds2_prev_cloud = pipeline_obj.readfile_prev_cloud;
        pipeline_obj.ds2_next_cloud = pipeline_obj.readfile_next_cloud;
    }


};



class Stage1_Downsample: public Stage
{
private:
public:
    double downsample_radius1; // initial downsample voxel length
    double downsample_radius2; // initial downsample voxel length

    Stage1_Downsample(double ds1, double ds2){
        downsample_radius1 = ds1;
        downsample_radius2 = ds2;
    }

    void run(Pipeline_Object& pipeline_obj){
        std::cout << downsample_radius1 << std::endl;
    }


};