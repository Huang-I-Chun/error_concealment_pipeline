// Pipeline_Object parameters
std::string prev_filename = "/mnt/m2_ssd/PC_Concealment/experiments_ssd/longdress_vox10_ai/no_drop/longdress_vox10_ai_dec_1051.ply";
std::string next_filename = "/mnt/m2_ssd/PC_Concealment/experiments_ssd/longdress_vox10_ai/no_drop/longdress_vox10_ai_dec_1055.ply";
double temporal_index = 0.5;
double resolution = 1024;

// My combination of stages
Stage *my_stage1 = new Stage1_None();
Stage *my_stage2 = new Stage2_Nearest_Neighbor(1);
Stage *my_stage3 = new Stage3_Cube_Estimation(1, 128);
Stage *my_stage4 = new Stage4_Cube_Estimation(1, 1);
Stage *my_stage5 = new Stage5_None("output.ply");
