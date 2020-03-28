#ifndef VISION_ARM_COMBO_H_
#define VISION_ARM_COMBO_H_
#define _CRT_SECURE_NO_WARNINGS
#include <thread> 
#include <future>
#include <boost/filesystem.hpp> 
#include <iostream>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ml/kmeans.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/search/search.h>
#include <pcl/registration/icp.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Eigenvalues>
#include <vector>
#include <ctime>
#include "RobotArmClient.h" // need to be 1st due to winsock
#include "KeyenceLineProfiler.h"
#include "KinectThread.h"
#include "PathPlanner.h"
//roboteq
#include "Constants.h"	
#include "ErrorCodes.h"	
#include "RoboteqDevice.h"
//robotiq 
#include "GripperModbusRTU.h"

#include "exo_rgb_cam.h"

#include <boost/filesystem.hpp>
#include "Motor.h"
#include "Serial.h"
#include <windows.h> //open file ialog
#include <Commdlg.h>//OPEN FILENAME

#ifndef NOMINMAX
#define NOMINMAX
#endif

using namespace boost::filesystem;

#define PICK_POT 0x0
#define PLACE_POT 0x1

//#define JUST_SCAN_A_POT
#define NO_VISUALIZATION

//open file dialog
#define NOMINMAX

struct VisionArmCombo
{
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef pcl::PointNormal PointNT;
	typedef pcl::PointCloud<PointNT> PointNCloudT;
	typedef pcl::PointXYZL PointLT;
	typedef pcl::PointCloud<PointLT> PointLCloudT;

	string label_file_path = "";

	struct ArmConfig
	{
		double joint_pos_d[6];
		float joint_pos_f[6];
		
		void setJointPos(double j1, double j2, double j3, double j4, double j5, double j6)
		{
			joint_pos_d[0] = j1; joint_pos_d[1] = j2;
			joint_pos_d[2] = j3; joint_pos_d[3] = j4;
			joint_pos_d[4] = j5; joint_pos_d[5] = j6;
			joint_pos_f[0] = (float)j1; joint_pos_f[1] = (float)j2;
			joint_pos_f[2] = (float)j3; joint_pos_f[3] = (float)j4;
			joint_pos_f[4] = (float)j5; joint_pos_f[5] = (float)j6;
		}

		void toRad() 
		{ 
			for (int i = 0; i < 6; i++) joint_pos_d[i] = joint_pos_d[i] / 180.*M_PI; 
			for (int i = 0; i < 6; i++) joint_pos_f[i] = joint_pos_f[i] / 180.f*M_PI;
		}
	};

	std::vector<ArmConfig> imaging_config_vec;

	// UR10 dh parameters
	const double d1 = 0.1273;
	const double a2 = -0.612;
	const double a3 = -0.5723;
	const double d4 = 0.163941;
	const double d5 = 0.1157;
	const double d6 = 0.0922;

	const int scanOrder[2][3] = { {4,5,6},{1,2,3} };

	const double ZERO_THRESH = 0.00000001;
	int SIGN(double x) { return (x > 0) - (x < 0); }
	const double PI = M_PI;

	// UR10 joint range
	double joint_range_for_probe_[12] = { -200. / 180.*M_PI, 20. / 180.*M_PI,	// base
										  -180. / 180.*M_PI, 0. / 180.*M_PI,	// shoulder
										  -160.f / 180.f*M_PI, -10.f / 180.f*M_PI,	// elbow
										  -160. / 180.*M_PI, 60. / 180.* M_PI,	// wrist 1
										  0.f / 180.f*M_PI, 180.f / 180.f*M_PI,	// wrist 2
										  -240.f/180.f*M_PI, -60.f/180.f*M_PI // wrist 3
										};

	const int num_joints_ = 6;

	RobotArmClient* robot_arm_client_ = NULL;

	Motor* pump = NULL;

	KeyenceLineProfiler* line_profiler_ = NULL;

	KinectThread* kinect_thread_ = NULL;
	
	std::ofstream result_file_;

	std::ofstream spherefile;

	std::ofstream current_status_file_;

	std::string CurrentOutputFolder = "";

	std::string CurrentOutputName = "";

	std::vector<PointCloudT::Ptr> calibration_point_cloud_vec;

	Eigen::Vector3f pre_point_;

	int pre_point_idx_;

	string current_pot_label_ = "current pot label";

	int start_pot_id_ = 0;
	int start_robot_stop_ = 0;
	int start_arm_side_ = 0;

	pcl::Normal pre_normal_;

#ifndef NO_VISUALIZATION	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
#endif


	Eigen::Matrix4f cam2hand_kinect_;

	// Distance vector of calibration plane in sensor frame
	std::vector<Eigen::Vector4d*> normalized_plane_coefficients_vec_sensor_frame_;
	std::vector<Eigen::Matrix4d*> hand_pose_vec_;
	std::vector<Eigen::Vector3d*> plane_normal_embed_dist_vec;

	Eigen::Matrix4f guessHandToScanner_;
	Eigen::Matrix4f handToScanner_;
	Eigen::Vector3f tool_center_point_;
	Eigen::Matrix4d probe_to_hand_;
	Eigen::Matrix4d scan_start_to_hand_;
	Eigen::Matrix4d scan_end_to_hand_;

	PointCloudT::Ptr kinect_cloud_;

	PointCloudT::Ptr laser_cloud_;

	pcl::VoxelGrid<PointT> vox_;

	pcl::StatisticalOutlierRemoval<PointT> sor_;

	float voxel_grid_size_;

	pcl::PassThrough<PointT> pass_;

	pcl::ExtractIndices<PointT> extract_indices_;

	PathPlanner pp_;

	double ik_sols_[8 * 6];

	Eigen::Affine3f pre_viewer_pose_;

	float scan_acceleration_ = 0.5f;
	
	float scan_speed_ = 0.1f;

	float move_arm_speed_ = 0.2f;

	float move_arm_acceleration_ = 0.05f;

	float move_arm_speed_water_ = 0.1f;

	float move_arm_acceleration_water_ = 0.05f;

	float move_joint_speed_ = 0.7f;

	float move_joint_acceleration_ = 0.2f;

	float speed_correction_ = 0;// -0.0085f;

	int view_time_ = 0;

	float exposure_time_ = 1000000.;

	int day_th_1_ = 7;

	int day_th_2_ = 21;

	int day_th_3_ = 30;

	string corn_height_ = "high";

	float well_watered_target_weight_ = 152.f;

	bool slower_drought_ = 1;

	//int current_day_count = 1;

	float final_target_weight = 145.f;


	int counter_;

	// kmeans
	cv::Mat object_centers_;

	//cv::Mat undistort; //acquire 2d image

	float scan_radius_;

	cv::Mat kinect_rgb_hand_to_eye_cv_, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_;

	cv::Ptr<cv::aruco::Dictionary> marker_dictionary_;

	cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

	float marker_length_;

	Eigen::Matrix4d cur_rgb_to_marker_, hand_to_rgb_, hand_to_gripper_, hand_to_tube1_, hand_to_tube2_;

	RoboteqDevice motor_controller_;

	GripperModbusRTU gripper_;

	EXO_RGB_CAM* exo_rgb_cam_ = NULL;

	const double table_offset_x_ = 0.0889;
	const double table_offset_y_ = 0.0508;
	const double pot2pot_dist_ = 0.1016;
	Eigen::Matrix4d hand_pose_above_pot_eigen_;
	Eigen::Vector3d sphere_pos1_, sphere_pos2_, sphere_pos3_;
	Eigen::Vector3d table_normal_x_, table_normal_y_, table_normal_z_;

	struct Pot
	{
		float target_weight;
		int robot_stop;
		int side;
		int local_pot_x;
		int local_pot_y;
		bool water = true;
		int pot_day_th2;
		string label;
	};

	const int table_rows_ = 20;
	const int table_cols_ = 12;
	const int max_pots_ = table_rows_*table_cols_;
	std::vector<Pot> pot_map_;

	Eigen::Matrix4d release_pose_;

	std::vector<std::string> pot_labels_;
	std::vector<std::string> corn_pot_labels_;
	std::vector<float> pot_target_weight_;
	std::vector<bool> pot_water_;
	std::vector<int> pot_day_th2_; 

	bool pause_operation_ = false;

	//bool active_ = true;

	bool perfect_stage_ = true;

	std::vector<std::thread> thread_vec_;

	bool place_pot_control_ = false;

	bool pick_pot_control_ = false;

	bool just_scan_control_ = false;

	bool open_file_dialog = false;

	int sphere_count_ = 0;

	static std::mutex lock;

	VisionArmCombo();

	~VisionArmCombo();

	double magnitudeVec3(double * vec);

	void array6ToEigenMat4d(double * array6, Eigen::Matrix4d & mat4d);

	void eigenMat4dToArray6(Eigen::Matrix4d & mat4d, double * array6);

	void array6ToEigenMat4(double* array6, Eigen::Matrix4f & mat4);

	void initVisionCombo();
	
	void initRobotArmClient();

	void initLineProfiler();

	void initKinectThread();

	void resetCameras();

	void calibrateToolCenterPoint(int numPoseNeeded=4);

	void calibrateGripperTip(int numPoseNeeded, std::string type);

	//void scanTranslateOnly(double * vec3d, PointCloudT::Ptr cloud, float acceleration, float speed);

	void scanTranslateOnly(double * vec3d, float acceleration, float speed);

	void scanLine(PointCloudT::Ptr & cloud);

	std::string getCurrentDateStr();

	std::string getCurrentTimeStr();

	void readCloudAndPoseFromFile();

	void lineScannerHandEyeCalibration(int num_lines_per_plane);

	void acquireLinesOnPlanes();

	void pp_callback(const pcl::visualization::PointPickingEvent& event, void*);

	void addArmModelToViewer(std::vector<PathPlanner::RefPoint> & ref_points);

	void addOBBArmModelToViewer(std::vector<PathPlanner::OBB> & arm_obbs);

	int inverseKinematics(Eigen::Matrix4d & T, std::vector<int> & ik_sols_vec);

	void forward(const double* q, double* T);

	void float2double(float* array6_f, double* array6_d);

	void double2float(double* array6_d, float* array6_f);

	double L2Norm(double* array6_1, double* array6_2);

	void getCurHandPose(Eigen::Matrix4f & pose);

	void getCurHandPoseD(Eigen::Matrix4d & pose);

	void smallClusterRemoval(PointCloudT::Ptr cloud_in, double clusterTolerance, int minClusterSize, PointCloudT::Ptr cloud_out);

	void setScanRadius(float radius);

	void display();

	void calibrateKinectRGBCamera();

	void KinectRGBHandEyeCalibration();

	void markerDetection();

	void cvTransformToEigenTransform(cv::Mat & cv_transform, Eigen::Matrix4d & eigen_transform);

	// RoAd functions
	int sendRoboteqVar(int id, int value);
	bool scanSphereOnTable(Eigen::Vector3d & hand_pos, Eigen::Vector3d & sphere_pos, int side=0);
	void initEXO_RGB_Cam();
	bool localizeByScanSpheres(int robot_stop=0, int side = 0);	//side 0-left side 1-right
	void gotoPot(int robot_stop, int side, int local_pot_x, int local_pot_y, bool open_gripper=false);
	void scanPot(int robot_stop, int side, int local_pot_x, int local_pot_y);
	void transportPotOnBalance(bool open_finger_on_balance=true);
	void gotoBalance(int pot_id);
	void scanOnePot(int pot_id);
	void justScan();
	void justImage();
	
	void placePots(int operation);
	int initPotMap();
	//void registerRGBandPointCloud(cv::Mat & rgb, PointCloudT::Ptr cloud_in_base, Eigen::Matrix4d & rgb_pose, bool clear_viewer=true);
	void registerRGBandPointCloud(cv::Mat & rgb, Eigen::Matrix4d & rgb_pose, bool clear_viewer = true);

	int getWater(float weight, bool b);

	void EigenMatrix4dToCVMat4d(Eigen::Matrix4d & eigen_mat, cv::Mat & cv_mat);

	void pause_operation_if_necessary();

	void openFileDialog();

//	void mainControlLoop();

};


#endif
