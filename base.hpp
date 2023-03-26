class Pipeline_Object{
public:
    double temporal_index; // should be 0 - 1

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr readfile_prev_cloud; // input origin point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr readfile_next_cloud; // input origin point cloud

    // Output variable of each stage
    // Stage 1 Preprocessing: After pre-process point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds1_prev_cloud; // using downsample1
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds1_next_cloud; // using downsample1
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds2_prev_cloud; // using downsample2
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ds2_next_cloud; // using downsample2

    // Stage 2 Matching: Obtain matching table
    std::vector<int> matching_table;

    // Stage 3 Motion Estimation: Motion estimation per point/center
    std::vector<std::vector<int>> motion_estimation;

    // Stage 4 Prediction:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr predict_point_cloud;

    // Stage 5 Post-Processing:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud; // output point cloud

    Pipeline_Object(std::string prev_filename, std::string next_filename, double t): 
    temporal_index(t),
    readfile_prev_cloud(new pcl::PointCloud< pcl::PointXYZRGB>),
    readfile_next_cloud(new pcl::PointCloud< pcl::PointXYZRGB>),
    ds1_prev_cloud(new pcl::PointCloud< pcl::PointXYZRGB>),
    ds1_next_cloud(new pcl::PointCloud< pcl::PointXYZRGB>),
    ds2_prev_cloud(new pcl::PointCloud< pcl::PointXYZRGB>),
    ds2_next_cloud(new pcl::PointCloud< pcl::PointXYZRGB>),
    predict_point_cloud(new pcl::PointCloud< pcl::PointXYZRGB>),
    new_cloud(new pcl::PointCloud< pcl::PointXYZRGB>)
    {   
        if (temporal_index < 0.5)
        {
            std::string tmp;
            tmp = prev_filename;
            prev_filename = next_filename;
            next_filename = tmp;
            temporal_index = 1 - temporal_index;
        }

        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(prev_filename, *readfile_prev_cloud) == -1 || pcl::io::loadPLYFile<pcl::PointXYZRGB>(next_filename, *readfile_next_cloud) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file ply or pcd file \n");
            return;
        }
    };

    ~Pipeline_Object(){};
};

class Stage
{
private:
public:
    Stage *_next;

    Stage(){
        _next = NULL;
    }

    virtual void run(Pipeline_Object& pipeline_obj) = 0;

 	void connect(Stage *nextstage) {
		_next = nextstage;
	}
};

void read_ply_file(Pipeline_Object& pipeline_obj, std::string prev_filename, std::string next_filename){
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(prev_filename, *pipeline_obj.readfile_prev_cloud) == -1 || pcl::io::loadPLYFile<pcl::PointXYZRGB>(next_filename, *pipeline_obj.readfile_next_cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file ply or pcd file \n");
        return;
    }
}