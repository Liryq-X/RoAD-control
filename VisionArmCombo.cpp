#include "VisionArmCombo.h"
//#define disableCam

std::mutex VisionArmCombo::lock;

VisionArmCombo::PointCloudT::Ptr scan_cloud(new VisionArmCombo::PointCloudT);

VisionArmCombo::PointCloudT::Ptr tmp_cloud_test(new VisionArmCombo::PointCloudT);

VisionArmCombo::VisionArmCombo() :
	voxel_grid_size_(0.002f),
	counter_(0)
{
	//initVisionCombo();

//	thread_vec_.push_back(std::thread(&VisionArmCombo::mainControlLoop, this));
}

VisionArmCombo::~VisionArmCombo()
{
	//motor_controller_.Disconnect();
	//active_ = false;
	thread_vec_.front().join();

	//if (exo_rgb_cam_ != NULL)
	//	exo_rgb_cam_->stop();

	//if (line_profiler_ != NULL)
	//	line_profiler_->finalize();

	robot_arm_client_->setDigitalOutput(7, false);
	robot_arm_client_->setDigitalOutput(6, false);
}

void VisionArmCombo::initVisionCombo()
{
	initPotMap();
	
	initLineProfiler();

#if 1
	initRobotArmClient();
	gripper_.activate();
	gripper_.open();
	gripper_.close();
#endif
	robot_arm_client_->setDigitalOutput(7, false);
	robot_arm_client_->setDigitalOutput(6, false);

#ifndef NO_VISUALIZATION	
	viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer_->addCoordinateSystem(0.3);
	viewer_->setSize(800, 600);
	viewer_->setPosition(0, 0);
	viewer_->registerPointPickingCallback(&VisionArmCombo::pp_callback, *this);
#endif


	// load line scanner hand eye calibration matrix
	guessHandToScanner_ = Eigen::Matrix4f::Identity();
	guessHandToScanner_.block<3, 3>(0, 0) = Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitZ()).matrix();
	guessHandToScanner_.col(3).head(3) << 0.076, 0.0, 0.094;

	handToScanner_ = guessHandToScanner_;

	//std::cout << "hand to scanner\n" << handToScanner_ << "\n";

	std::ifstream file("lineScannerHandEyeCalibration.bin", std::ios::in | std::ios::binary);
	if (file.is_open())
	{
		char buffer[64];
		file.read(buffer, 64);
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
			{
				float* ptr = (float*)buffer + i * 4 + j;
				handToScanner_.row(i)(j) = *(ptr);
			}
		std::cout << "handToScanner:\n" << handToScanner_ << "\n";
	}
	else std::cout << "lineScannerHandEyeCalibration load fail\n";
	file.close();

	cv::FileStorage fs("gripper_tip_calib.yml", cv::FileStorage::READ);
	cv::Vec3d tcp;
	cv::Vec3d tcp_tube_1, tcp_tube_2;
	if (fs.isOpened())
	{
		fs["tcp"] >> tcp;
		fs["tube_1"] >> tcp_tube_1;
		fs["tube_2"] >> tcp_tube_2;
	}
	fs.release();

	for (int i = 0; i < 3; i++) tool_center_point_(i) = tcp[i];

	//std::cout << "tcp: " << tcp<<"\n";

	probe_to_hand_ = Eigen::Matrix4d::Identity();

	probe_to_hand_.col(3).head<3>() = tool_center_point_.cast<double>();

	probe_to_hand_ = probe_to_hand_.inverse();

	marker_length_ = 0.1016f;	//4 inch

	hand_to_gripper_ = Eigen::Matrix4d::Identity();
	hand_to_gripper_(0, 0) = -1.;
	hand_to_gripper_(2, 2) = -1.;
	
	hand_to_gripper_.col(3).head(3) << 0.0381, -0.0827999, -0.3683;

	hand_to_tube1_ = Eigen::Matrix4d::Identity();

	hand_to_tube1_(0, 0) = -1.;
	hand_to_tube1_(2, 2) = -1.;

	hand_to_tube1_.col(3).head(3) << tcp_tube_1[0], tcp_tube_1[1], tcp_tube_1[2];

	fs.open("kinectRGBCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["camera_matrix"] >> kinect_rgb_camera_matrix_cv_;

		fs["distortion_coefficients"] >> kinect_rgb_dist_coeffs_cv_;

		fs.release();
	}

	fs.open("kinectRGBHandEyeCalibration.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["hand to eye"] >> kinect_rgb_hand_to_eye_cv_;

		fs.release();
	}

	cvTransformToEigenTransform(kinect_rgb_hand_to_eye_cv_, hand_to_rgb_);

#if 0
	marker_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

	detector_params_ = cv::aruco::DetectorParameters::create();

	detector_params_->doCornerRefinement = true; // do corner refinement in markers
#endif

	cur_rgb_to_marker_ = Eigen::Matrix4d::Identity();

	pre_point_ << 0, 0, 0;

	scan_start_to_hand_ = Eigen::Matrix4d::Identity();

	scan_start_to_hand_(2, 3) = 0.3; scan_start_to_hand_(1, 3) = -0.05;

	scan_start_to_hand_ = scan_start_to_hand_.inverse()*handToScanner_.cast<double>().inverse();

	scan_end_to_hand_ = Eigen::Matrix4d::Identity();

	scan_end_to_hand_(2, 3) = 0.3; scan_end_to_hand_(1, 3) = +0.05;

	scan_end_to_hand_ = scan_end_to_hand_.inverse()*handToScanner_.cast<double>().inverse();

	kinect_cloud_.reset(new PointCloudT);
	kinect_cloud_->is_dense = true;

	laser_cloud_.reset(new PointCloudT);
	laser_cloud_->is_dense = true;

	// kinect
	cam2hand_kinect_ = Eigen::Matrix4f::Identity();
	cam2hand_kinect_(0, 3) = 0.0540247f;
	cam2hand_kinect_(1, 3) = 0.1026325f;
	cam2hand_kinect_(2, 3) = 0.0825227f;

	ArmConfig imaging_config;
	/*imaging_config.setJointPos(-94.64, -11.94, -134.11, -79.05, 93.28, -183.29);
	imaging_config.toRad();
	imaging_config_vec.push_back(imaging_config);*/

	imaging_config.setJointPos(-87.72, -92.01, -106.85, -71.06, 90.44, -174.31);
	imaging_config.toRad();
	imaging_config_vec.push_back(imaging_config);

	sor_.setMeanK(50);
	sor_.setStddevMulThresh(1.0);


	fs.open("road_parameters.yml", cv::FileStorage::READ);

	if (fs.isOpened())
	{
		fs["move_arm_speed_"] >> move_arm_speed_;
		fs["move_arm_acceleration_"] >> move_arm_acceleration_;
		fs["move_joint_speed_"] >> move_joint_speed_;
		fs["move_joint_acceleration_"] >> move_joint_acceleration_;
		fs["view_time_"] >> view_time_;
		fs["exposure_time_"] >> exposure_time_;
		fs["day_th_1_"] >> day_th_1_;
		fs["day_th_2_"] >> day_th_2_;
		fs["day_th_3_"] >> day_th_3_;
		fs["well_watered_target_weight_"] >> well_watered_target_weight_;
		fs["slower_drought_"] >> slower_drought_;
		fs["move_arm_speed_water_"] >> move_arm_speed_water_;
		fs["move_arm_acceleration_water_"] >> move_arm_acceleration_water_;
		fs["corn_height_"] >> corn_height_;

		fs.release();
	}
	cout << "day_th_1_: " << day_th_1_ << endl;
	cout << "day_th_2_: " << day_th_2_ << endl;
	cout << "day_th_3_: " << day_th_3_ << endl;
	cout << "well_watered_target_weight_: " << well_watered_target_weight_ << endl;
	cout << "slower_drought_: " << slower_drought_ << endl;


#if 1
	initEXO_RGB_Cam();
#endif

	pump = new Motor();

	int status = motor_controller_.Connect("COM1");

	if (status != RQ_SUCCESS)
		std::cout << "Error connecting to motor controller: " << status << "\n";
}

void VisionArmCombo::pp_callback(const pcl::visualization::PointPickingEvent& event, void*)
{
	if (event.getPointIndex() == -1)
		return;
	Eigen::Vector3f current_point;
	event.getPoint(current_point[0], current_point[1], current_point[2]);

	pre_point_idx_ = event.getPointIndex();

	std::cout << "current point:\n" << current_point << "\n";
	std::cout << "distance: " << (current_point - pre_point_).norm() << "\n";
	pre_point_ = current_point;

#ifndef NO_VISUALIZATION	
	pre_viewer_pose_ = viewer_->getViewerPose(0);
	std::cout << "viewer pose\n"<< pre_viewer_pose_.matrix() << "\n";
#endif
}

double VisionArmCombo::magnitudeVec3(double * vec)
{
	return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

void VisionArmCombo::array6ToEigenMat4d(double* array6, Eigen::Matrix4d & mat4d)
{
	mat4d = Eigen::Matrix4d::Identity();
	double angle = magnitudeVec3(array6 + 3);
	Eigen::Vector3d axis(array6[3] / angle, array6[4] / angle, array6[5] / angle);
	Eigen::Matrix3d rotationMatrix;
	rotationMatrix = Eigen::AngleAxisd(angle, axis);
	mat4d.block<3, 3>(0, 0) = rotationMatrix;
	mat4d(0, 3) = array6[0]; mat4d(1, 3) = array6[1]; mat4d(2, 3) = array6[2];
}

void VisionArmCombo::eigenMat4dToArray6(Eigen::Matrix4d & mat4d, double * array6)
{
	Eigen::AngleAxisd angle_axis(mat4d.topLeftCorner<3,3>());

	array6[0] = mat4d(0, 3); array6[1] = mat4d(1, 3); array6[2] = mat4d(2, 3);

	Eigen::Vector3d rotation_vector = angle_axis.angle()*angle_axis.axis();

	array6[3] = rotation_vector[0]; array6[4] = rotation_vector[1]; array6[5] = rotation_vector[2];
}

void VisionArmCombo::array6ToEigenMat4(double* array6, Eigen::Matrix4f & mat4)
{
	mat4 = Eigen::Matrix4f::Identity();
	double angle = magnitudeVec3(array6 + 3);
	Eigen::Vector3f axis(array6[3] / angle, array6[4] / angle, array6[5] / angle);
	Eigen::Matrix3f rotationMatrix;
	rotationMatrix = Eigen::AngleAxisf(angle, axis);
	mat4.block<3, 3>(0, 0) = rotationMatrix;
	mat4(0, 3) = array6[0]; mat4(1, 3) = array6[1]; mat4(2, 3) = array6[2];
}


void VisionArmCombo::initRobotArmClient()
{
	robot_arm_client_ = new RobotArmClient();
}

void VisionArmCombo::initLineProfiler()
{
	line_profiler_ = new KeyenceLineProfiler();
}

void VisionArmCombo::initKinectThread()
{
	kinect_thread_ = new KinectThread();
}

void VisionArmCombo::calibrateToolCenterPoint(int numPoseNeeded)
{
	if (robot_arm_client_ == NULL) initRobotArmClient(); 

	int poseIdx = 0;

	std::vector<double*> poseVec;
	poseVec.resize(numPoseNeeded);

	for (int i = 0; i < numPoseNeeded; i++)
	{
		poseVec[i] = new double[6];
	}

	if (numPoseNeeded % 2 != 0 || numPoseNeeded < 4)
	{
		std::cout << "Num of poses needed wrong" << std::endl;
		return;
	}

	Eigen::Vector3d vec3d;

	while (true)
	{
		std::cout << "Press Enter to save pose " << poseIdx << std::endl;
		std::getchar();

		robot_arm_client_->getCartesianInfo(poseVec[poseIdx]);
		robot_arm_client_->printCartesianInfo(poseVec[poseIdx]);

		if (poseIdx == numPoseNeeded - 1)
		{
			//std::cout << "size: "<<poseVec.size() << std::endl;
			Eigen::MatrixXd A(3 * (numPoseNeeded)*(numPoseNeeded-1), 3);
			Eigen::VectorXd b(3 * (numPoseNeeded)*(numPoseNeeded-1));

			int idx = 0;

			for (int i = 0; i < numPoseNeeded; i++)
			{
				for (int j = 0; j < numPoseNeeded; j++)
				{
					if (i != j)
					{
						Eigen::Matrix4d T0;

						array6ToEigenMat4d(poseVec[i], T0);

						//std::cout << "T0" << std::endl << T0 << std::endl;

						Eigen::Matrix4d T1;

						array6ToEigenMat4d(poseVec[j], T1);

						//std::cout << "T1" << std::endl << T1 << std::endl;

						T0 = T0 - T1;

						//std::cout << "T0-T1" << std::endl << T0 << std::endl;

						A.block<3, 3>(3 *idx, 0) = T0.block<3, 3>(0, 0);

						b.block<3, 1>(3 *idx, 0) = T0.block<3, 1>(0, 3);
						++idx;
					}
				}
			}

		/*	Eigen::MatrixXd A(3 * numPoseNeeded / 2, 3);
			Eigen::VectorXd b(3 * numPoseNeeded / 2);

			for (int i = 0; i < numPoseNeeded; i += 2)
			{
				Eigen::Matrix4d T0;

				array6ToEigenMat4d(poseVec[i], T0);

				std::cout << "T0" << std::endl << T0 << std::endl;

				Eigen::Matrix4d T1;

				array6ToEigenMat4d(poseVec[i + 1], T1);

				std::cout << "T1" << std::endl << T1 << std::endl;

				T0 = T0 - T1;

				std::cout << "T0-T1" << std::endl << T0 << std::endl;

				A.block<3, 3>(3 * (i / 2), 0) = T0.block<3, 3>(0, 0);
				b.block<3, 1>(3 * (i / 2), 0) = T0.block<3, 1>(0, 3);
			}*/

			// Solve Ax=b
			//std::cout << "A:" << std::endl << A << std::endl << "b:" << std::endl << b << std::endl;

			vec3d = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

			vec3d *= -1.;

			std::cout << "x (hand to TCP):" << std::endl << vec3d << std::endl;

			break;
		}

		poseIdx++;
	}

	cv::FileStorage fs("tool_center_point_calib.yml", cv::FileStorage::WRITE);

	cv::Vec3d tcp;
	tcp[0] = vec3d(0); tcp[1] = vec3d(1); tcp[2] = vec3d(2);

	fs << "tcp" << tcp;

	fs.release();
	
	std::cout << "Saved" << std::endl;
}

void VisionArmCombo::calibrateGripperTip(int numPoseNeeded = 4, std::string type = "gripper")
{
	if (robot_arm_client_ == NULL) initRobotArmClient();

	int poseIdx = 0;

	std::vector<double*> poseVec;
	poseVec.resize(numPoseNeeded);

	for (int i = 0; i < numPoseNeeded; i++)
	{
		poseVec[i] = new double[6];
	}

	if (numPoseNeeded % 2 != 0 || numPoseNeeded < 4)
	{
		std::cout << "Num of poses needed wrong" << std::endl;
		return;
	}

	Eigen::Vector3d vec3d;

	while (true)
	{
		std::cout << "Press Enter to save pose " << poseIdx << std::endl;
		std::getchar();

		robot_arm_client_->getCartesianInfo(poseVec[poseIdx]);
		robot_arm_client_->printCartesianInfo(poseVec[poseIdx]);

		if (poseIdx == numPoseNeeded - 1)
		{
			//std::cout << "size: "<<poseVec.size() << std::endl;
			Eigen::MatrixXd A(3 * (numPoseNeeded)*(numPoseNeeded - 1), 3);
			Eigen::VectorXd b(3 * (numPoseNeeded)*(numPoseNeeded - 1));

			int idx = 0;

			for (int i = 0; i < numPoseNeeded; i++)
			{
				for (int j = 0; j < numPoseNeeded; j++)
				{
					if (i != j)
					{
						Eigen::Matrix4d T0;

						array6ToEigenMat4d(poseVec[i], T0);

						Eigen::Matrix4d T1;

						array6ToEigenMat4d(poseVec[j], T1);

						T0 = T0 - T1;

						A.block<3, 3>(3 * idx, 0) = T0.block<3, 3>(0, 0);

						b.block<3, 1>(3 * idx, 0) = T0.block<3, 1>(0, 3);
						++idx;
					}
				}
			}

			vec3d = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

			vec3d *= -1.;

			std::cout << "x (hand to TCP):" << std::endl << vec3d << std::endl;

			break;
		}

		poseIdx++;
	}

	cv::FileStorage fs_read("gripper_tip_calib.yml", cv::FileStorage::READ);

	std::vector<cv::Vec3d> tcp3(3);

	fs_read["gripper"] >> tcp3[0];
	fs_read["tube_1"] >> tcp3[1];
	fs_read["tube_2"] >> tcp3[2];

	fs_read.release();

	cv::FileStorage fs("gripper_tip_calib.yml", cv::FileStorage::WRITE);

	cv::Vec3d tcp;
	tcp[0] = vec3d(0); tcp[1] = vec3d(1); tcp[2] = vec3d(2);

	if (type == "gripper")
		tcp3[0] = tcp;
	else if (type == "tube_1")
		tcp3[1] = tcp;
	else if (type == "tube_2")
		tcp3[2] = tcp;

	fs << "gripper" << tcp3[0];
	fs << "tube_1" << tcp3[1];
	fs << "tube_2" << tcp3[2];

	fs.release();

	std::cout << "Saved" << std::endl;
}

/*
	assume robot arm already at start pose
	vec3d: motion vector in base frame
	cloud: PCL point cloud to save the data
*/
//void VisionArmCombo::scanTranslateOnly(double * vec3d, PointCloudT::Ptr  cloud_test, float acceleration, float speed)
void VisionArmCombo::scanTranslateOnly(double * vec3d, float acceleration, float speed)
{
	
	if (acceleration == 0 || speed == 0)
	{
		std::cout << "acceleration or speed = 0\n";
		return;
	}

	if (robot_arm_client_ == NULL || line_profiler_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	double curPoseD[6];
	robot_arm_client_->getCartesianInfo(curPoseD);
	//robot_arm_client_->printCartesianInfo(curPoseD);

	double endPoseD[6];
	std::memcpy(endPoseD, curPoseD, 48);
	for (int i = 0; i < 3; i++) endPoseD[i] += vec3d[i];

	//double sync_pose[6];
	cout << "in scan translate\n";

	if (line_profiler_->device_initialized == false)
		line_profiler_->init();

	line_profiler_->m_vecProfileData.clear();
	
	//double tcp_sync_speed[6];
	robot_arm_client_->setStartPoseXYZ();

	unsigned int start_frame_id = 0;

	robot_arm_client_->moveHandL(endPoseD, acceleration, speed);

	cout << "moveHandL\n";

	line_profiler_->start(20);

	robot_arm_client_->waitTillTCPMove();

	start_frame_id = line_profiler_->frame_counter_;
	
	//old synchronization
	//line_profiler_->start(20);
	//robot_arm_client_->getCartesianInfo(sync_pose);
	//robot_arm_client_->getTCPSpeed(tcp_sync_speed);
	
	if(robot_arm_client_->waitTillHandReachDstPose(endPoseD)==-1)
		std::cerr << "get wait pose timeout" << std::endl;
	line_profiler_->stop();

	//std::cerr << "start_frame_id: " << start_frame_id << "\n";

	// register point cloud
	scan_cloud->clear();

	int num_profiles = line_profiler_->m_vecProfileData.size();

	cout << "num_profiles\n";

	double sync_speed = sqrt(robot_arm_client_->tcp_sync_speed_[0] * robot_arm_client_->tcp_sync_speed_[0] 
					+ robot_arm_client_->tcp_sync_speed_[1] * robot_arm_client_->tcp_sync_speed_[1] 
					+ robot_arm_client_->tcp_sync_speed_[2] * robot_arm_client_->tcp_sync_speed_[2]);


	Eigen::Matrix4f startPose;
	Eigen::Matrix4f endPose;

	array6ToEigenMat4(curPoseD, startPose);
	array6ToEigenMat4(endPoseD, endPose);

	double sync_distance = robot_arm_client_->EuclideanDistance(curPoseD, robot_arm_client_->sync_pose_);

	//std::cout << "calculated end pose\n" << endPose << "\n\n";

	/*double testPoseD[6];
	robot_arm_client_->getCartesianInfo(testPoseD);
	Eigen::Matrix4f testPose;
	array6ToEigenMat4(testPoseD, testPose);

	std::cout << "true end pose\n" << testPose << "\n\n";*/


	// express motion vector in base frame
	Eigen::Vector3f motionVector( endPoseD[0] - curPoseD[0],
							      endPoseD[1] - curPoseD[1],
								  endPoseD[2] - curPoseD[2] );

	//std::cout << "motion vector in base frame:" << motionVector << std::endl;

	// transform motion vector in scanner frame
	/*Eigen::Matrix3f m;
	m = startPose.block<3, 3>(0,0);
	// A'*B' = (BA)' where A and B are rotation matrices. vec_senosr = Te2s*Tb2e*vec_base
	m = m*Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitZ());
	motionVector = m.transpose()*motionVector;*/

	motionVector = (handToScanner_.block<3,3>(0,0)).transpose()*(startPose.block<3, 3>(0, 0)).transpose()*motionVector;

	//std::cout << "motion vector in sensor frame:" << motionVector << std::endl;

	float magnitude = motionVector.norm();

	Eigen::Vector3f motionDelta = motionVector / (num_profiles - 1);

	//std::cout << "motion delta:" << motionDelta << std::endl;

	//std::cout << "x start: " << line_profiler_->m_profileInfo.lXStart << " pitch: " << line_profiler_->m_profileInfo.lXPitch << "\n";

	// 2k sampling frequency (0.5ms)
	float distance = 0;

	double time = 0;

	time = sync_speed / acceleration;

	time = sqrt(sync_distance * 2 / acceleration);

	std::cerr << "start time from dist: " << time << "\n";

	const float start_cruise_time = speed/acceleration;

	const float stop_cruise_time = start_cruise_time + (magnitude -start_cruise_time*start_cruise_time*acceleration)/speed;

	const float cruise_time = stop_cruise_time - start_cruise_time;

	const float total_time = start_cruise_time + stop_cruise_time;

	const float acceleration_total_distance = 0.5f*acceleration*start_cruise_time*start_cruise_time;

	motionVector.normalize();

	PointT point;

	uint32_t r, rgb;

		for (int i = start_frame_id; i < num_profiles; i++)
		{

			if (time <= start_cruise_time)
				distance = 0.5f*acceleration*time*time;
			else if (time <= stop_cruise_time)
				distance = acceleration_total_distance + (time - start_cruise_time)*speed;
			else
				distance = magnitude - pow(total_time - time, 2.f)*0.5f*acceleration;


			time += 5e-4;	// 0.5 ms

			// the offset maybe be related to scan speed
			Eigen::Vector3f displacement = motionVector*(distance + speed_correction_);

				for (int j = 10; j < 790; j++)
				{
					

					point.z = line_profiler_->m_vecProfileData[i].m_pnProfileData[j] * (-1e-8f);

		
						if (abs(point.z) < 0.140f)
						{

							point.y = displacement(1);
							point.x = (float)(line_profiler_->m_profileInfo.lXStart + j*line_profiler_->m_profileInfo.lXPitch)*(1e-8f) + displacement(0);
							point.z += 0.3f + displacement(2);

							r = ((uint32_t)(point.z * 100000)) % 255;

							rgb = r << 16 | r << 8 | (uint32_t)255;

							point.rgb = *reinterpret_cast<float*>(&rgb);

							//point.r = ((int)(point.z*100000))%255;//(uint8_t)(255.f - 255.f*(point.z + 0.141f) / 0.282f);
							//point.g = point.r;
							//point.b = 255;

							try {
								scan_cloud->push_back(point);
								//cloud_test->points.push_back(point);
							}
							catch (std::exception e)
							{
								std::cerr << "push back, " << e.what() << std::endl;
							}
						}
				}
		}

	//std::cerr << "point cloud size: " << scan_cloud->size() << std::endl;

}

void VisionArmCombo::scanLine(PointCloudT::Ptr & cloud)
{
	if (robot_arm_client_ == NULL || line_profiler_ == NULL)
	{
		std::cout << "robot arm or line profiler not initialized \n";
		return;
	}

	if (line_profiler_->device_initialized == false)
		line_profiler_->init();

	double curPoseD[6];
	robot_arm_client_->getCartesianInfo(curPoseD);
	robot_arm_client_->printCartesianInfo(curPoseD);

	//!before start scan,  must clear old data
	line_profiler_->m_vecProfileData.clear();
	line_profiler_->start(10);
	line_profiler_->stop();

	cloud->clear();

	int num_profiles = line_profiler_->m_vecProfileData.size();

	std::cout << "num profiles: " << num_profiles << std::endl;

	//for (int i = 0; i < num_profiles; i++)
	int i = num_profiles / 2;
	{
		for (int j = 10; j < 790; j++)
		{
			PointT point;

			point.z = line_profiler_->m_vecProfileData[i].m_pnProfileData[j] * (-1e-8f);

			if (abs(point.z) < 0.140f)
			{
				point.z += 0.3f;
				point.y = 0.f;
				point.x = (float)(line_profiler_->m_profileInfo.lXStart + j*line_profiler_->m_profileInfo.lXPitch)*(1e-8f);

				point.r = (uint8_t)(255.f - 255.f*(point.z + 0.14f) / 0.28f);
				point.g = point.r;
				point.b = 255;

				cloud->push_back(point);
			}
		}
	}

	std::cerr << "point cloud size: " << cloud->size() << std::endl;
}


void VisionArmCombo::readCloudAndPoseFromFile()
{
	std::ifstream file("ScannerRobotArmCalibrationFile.txt");
	
	if (file.is_open())
	{
		std::string line;

		calibration_point_cloud_vec.clear();

		while (std::getline(file, line))
		{
			std::size_t found = line.find_first_of(",");

			std::string front_part;

			double pose[6];

			if (found != std::string::npos && found > 0)
			{
				front_part = line.substr(0, found);
				//std::cout << "file name: " << front_part << "\n";
				
				// trim line
				line = line.substr(found + 1, line.size()-1);

				// read point cloud
				PointCloudT::Ptr tmp_point_cloud (new PointCloudT);
				if(pcl::io::loadPCDFile(front_part, *tmp_point_cloud) == 0)
					calibration_point_cloud_vec.push_back(tmp_point_cloud);
				else
					std::cout<<"load pcd file "<<front_part<<" fail\n";
				
				//std::cout << line << "\n";	

				for (int i = 0; i < 5; i++)
				{
					found = line.find_first_of(",");

					if (found != std::string::npos)
					{
						front_part = line.substr(0, found);
						pose[i] = std::stod(front_part, 0);
						//std::cout << i << " " << pose[i] << "\n";
						line = line.substr(found + 1, line.size() - 1);
					}
					else
					{
						std::cout << "wrong line\n";
						break;
					}
				}

				pose[5] = std::stod(line, 0);
				//std::cout << "5 " << pose[5] << "\n";

				Eigen::Matrix4d *tmp_pose = new Eigen::Matrix4d;
				array6ToEigenMat4d(pose, *tmp_pose);
				hand_pose_vec_.push_back(tmp_pose);
			}
			else
			{
				std::cout << "wrong line\n";
				break;
			}				
		}

		file.close();
	}
	else std::cout << "Unable to open file";
}

std::string VisionArmCombo::getCurrentDateStr()
{
	SYSTEMTIME st;

	char currentTime[84] = "";

	GetLocalTime(&st); 

	std::sprintf(currentTime, "%d-%d-%d", st.wYear, st.wMonth, st.wDay);

	//GetSystemTime(&st); in UTC format

	return std::string(currentTime);
}

std::string VisionArmCombo::getCurrentTimeStr()
{
	SYSTEMTIME st;

	char currentTime[84] = "";

	GetLocalTime(&st);

	std::sprintf(currentTime, "%d-%d-%d-%d-%d-%d-%d", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds); 

	//GetSystemTime(&st); in UTC format

	return std::string(currentTime);
}


void VisionArmCombo::acquireLinesOnPlanes()
{
	if(robot_arm_client_ == NULL) initRobotArmClient();

	if(line_profiler_ == NULL) initLineProfiler();

	int line_count = 0;
	int plane_count = 0;
	int num_lines_per_plane = 6;

	while (true)
	{
		std::cout << "Move robot hand to plane id " <<plane_count<<" then press Enter to scan on the plane or 'q'+Enter to close\n";

		char key = std::getchar();

		if (key == 'q')
			break;

		//std::vector<PointCloudT::Ptr> line_vec;

		for (int i = 0; i < num_lines_per_plane; i++)
		{
			std::cout << "	press Enter to scan line "<<i<<" on plane " <<plane_count<<"\n";

			char key = std::getchar();

			std::cout << "scan line id " << line_count << "\n";

			PointCloudT::Ptr point_cloud(new PointCloudT);

			scanLine(point_cloud);

			// save data
			std::string filename = getCurrentTimeStr() + ".pcd";

			pcl::io::savePCDFile(filename, *point_cloud, true);

			result_file_.open("ScannerRobotArmCalibrationFile.txt", std::ios::app);

			double curPoseD[6];

			robot_arm_client_->getCartesianInfo(curPoseD);

			Eigen::Matrix4d mat4d;
			array6ToEigenMat4d(curPoseD, mat4d);
			std::cout << "pose:\n" << mat4d << "\n";

			char buffer[100];

			sprintf(buffer, "%f,%f,%f,%f,%f,%f", curPoseD[0], curPoseD[1], curPoseD[2], curPoseD[3], curPoseD[4], curPoseD[5]);

			std::string startPoseStr(buffer);

			result_file_ << filename << "," << startPoseStr << "," << plane_count << "," << line_count << std::endl;

			result_file_.close();

#ifndef NO_VISUALIZATION	
			viewer_->removeAllPointClouds();
			viewer_->addPointCloud(point_cloud);
			display();
#endif

			//line_vec.push_back(point_cloud);

			line_count++;
		}

		plane_count++;
	}

	robot_arm_client_->stopRecvTCP();
	line_profiler_->stop();
	line_profiler_->finalize();
}

/*
Solve scanner pose wrt robot hand assuming normalDistanceVec populated.
Carlson, F. B., Johansson, R., & Robertsson, A. (2015, September).
Six DOF eye-to-hand calibration from 2D measurements using planar constraints.
In Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on (pp. 3628-3632). IEEE.
*/
void VisionArmCombo::lineScannerHandEyeCalibration(int num_lines_per_plane)
{
	Eigen::Matrix4d curHandToScanner_ = guessHandToScanner_.cast<double>();

	std::cout << "initial guess:\n" << curHandToScanner_ << "\n";

	int rows = 0; for (auto c : calibration_point_cloud_vec) rows += c->points.size();

	Eigen::MatrixXd A(rows, 9);
	Eigen::VectorXd Y(rows);
	Eigen::VectorXd w(6);
	double min_rms = 1e5;
	std::cout << "A rows: " << rows << "\n";

	for (int iter = 0; iter < 20; iter++)
	{
		std::cout << "\niteration " << iter << "\n";
		double rms = 0.;
		plane_normal_embed_dist_vec.clear();

		// compute plane normal
		for (int i = 0; i < calibration_point_cloud_vec.size(); i += num_lines_per_plane)
		{
			PointCloudT::Ptr plane_cloud(new PointCloudT);

			for (int j = 0; j < num_lines_per_plane; j++)
			{
				Eigen::Matrix4f base_to_scanner;
				base_to_scanner = hand_pose_vec_[i + j]->cast<float>()*curHandToScanner_.cast<float>();
				PointCloudT::Ptr tmp_cloud(new PointCloudT);
				pcl::transformPointCloud(*calibration_point_cloud_vec[i + j], *tmp_cloud, base_to_scanner);
				*plane_cloud += *tmp_cloud;
				/*viewer_->removeAllPointClouds();
				viewer_->addPointCloud(calibration_point_cloud_vec[i + j]);
				display();*/
			}

			//Eigen::Matrix3d covar_mat = Eigen::Matrix3d::Identity();
			//double xmean = 0.; double ymean = 0.; double zmean = 0.;
			////compute mean
			//for (int j = 0; j < plane_cloud->points.size(); j++)
			//{
			//	xmean += plane_cloud->points[j].x;
			//	ymean += plane_cloud->points[j].y;
			//	zmean += plane_cloud->points[j].z;
			//}

			//xmean /= plane_cloud->points.size();
			//ymean /= plane_cloud->points.size();
			//zmean /= plane_cloud->points.size();

			//double xy_covar = 0.; double xz_covar = 0.; double yz_covar = 0.;
			//double xx_covar = 0.; double yy_covar = 0.; double zz_covar = 0.;
			//for (int j = 0; j < plane_cloud->points.size(); j++)
			//{
			//	xy_covar += (plane_cloud->points[j].x - xmean)*(plane_cloud->points[j].y - ymean);
			//	xz_covar += (plane_cloud->points[j].x - xmean)*(plane_cloud->points[j].z - zmean);
			//	yz_covar += (plane_cloud->points[j].z - zmean)*(plane_cloud->points[j].y - ymean);

			//	xx_covar += (plane_cloud->points[j].x - xmean)*(plane_cloud->points[j].x - xmean);
			//	yy_covar += (plane_cloud->points[j].y - ymean)*(plane_cloud->points[j].y - ymean);
			//	zz_covar += (plane_cloud->points[j].z - zmean)*(plane_cloud->points[j].z - zmean);
			//}

			//covar_mat.diagonal() << xx_covar, yy_covar, zz_covar;
			//covar_mat(0, 1) = xy_covar; covar_mat(0, 2) = xz_covar; covar_mat(1, 2) = yz_covar;
			//covar_mat(1, 0) = xy_covar; covar_mat(2, 0) = xz_covar; covar_mat(2, 1) = yz_covar;

			//Eigen::Vector3d centroid_d(xmean, ymean, zmean);

			//Eigen::EigenSolver<Eigen::Matrix3d> es(covar_mat);

			//int min_idx = 0;
			//double min_eigen = es.eigenvalues()[0].real();
			//for (int j = 1; j < 3; j++)
			//{
			//	if (es.eigenvalues()[j].real() < min_eigen)
			//	{
			//		min_idx = j; min_eigen = es.eigenvalues()[j].real();
			//	}
			//}

			//Eigen::Vector3d min_eigen_vector = es.eigenvectors().col(min_idx).real();


			// estimate normal
			pcl::PCA<PointT> pca(*plane_cloud);

			Eigen::Vector4d centroid = pca.getMean().cast<double>();

			//std::cout << "centroid \n" << centroid << "\n";

			Eigen::Vector3d eigen_values = pca.getEigenValues().cast<double>();
			//std::cout << "eigen values \n" << eigen_values << "\n";

			Eigen::Matrix3d eigen_vectors = pca.getEigenVectors().cast<double>();

			//std::cout << "eigen vector \n" << eigen_vectors << "\n";

			// n in the paper
			Eigen::Vector3d* plane_normal_embed_dist = new Eigen::Vector3d;

			(*plane_normal_embed_dist) = eigen_vectors.col(2)*(eigen_vectors.col(2).dot(centroid.head<3>()));
			// try my own eigen decompo
			//(*plane_normal_embed_dist) = min_eigen_vector*(min_eigen_vector.dot(centroid_d));

			plane_normal_embed_dist_vec.push_back(plane_normal_embed_dist);

		/*	pcl::ModelCoefficients line;
			line.values.resize(6);
			line.values[0] = 0.f; line.values[1] = 0.f; line.values[2] = 0.f;
			line.values[3] = (*plane_normal_embed_dist)(0); line.values[4] = (*plane_normal_embed_dist)(1);
			line.values[5] = (*plane_normal_embed_dist)(2);

			viewer_->removeAllShapes();
			std::string name = "line";
			viewer_->addLine(line, name, 0);
			viewer_->removeAllPointClouds();
			viewer_->addPointCloud(plane_cloud);
			display();*/
		}

		// compute A and Y
		int row_idx = 0; rms = 0.;
		for (int i = 0; i < calibration_point_cloud_vec.size(); i += num_lines_per_plane)
		{
			Eigen::Vector3d n = *plane_normal_embed_dist_vec[i / num_lines_per_plane];

			for (int j = 0; j < num_lines_per_plane; j++)
			{
				PointCloudT::Ptr cloud = calibration_point_cloud_vec[i + j];

				Eigen::RowVector3d nR = n.transpose()*hand_pose_vec_[i + j]->topLeftCorner<3, 3>();

				double np = n.dot(hand_pose_vec_[i + j]->topRightCorner<3,1>());

				for (auto & ps : cloud->points)
				{
					A.block<1, 3>(row_idx, 0) = nR*(double)ps.x;
					A.block<1, 3>(row_idx, 3) = nR*(double)ps.z;
					A.block<1, 3>(row_idx, 6) = nR;
					Y(row_idx++) = n.dot(n) - np;
				}
			}
		}

		w.head<3>() = curHandToScanner_.topLeftCorner<3, 1>(); w.segment<3>(3) = curHandToScanner_.col(2).head(3);
		w.tail<3>() = curHandToScanner_.col(3).head(3);
		rms = sqrt((A*w - Y).squaredNorm() / rows);
		std::cout << "Before RMS: " <<  rms << "\n";

		// solve w using SVD
		w = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
		
		std::cout << "w:\n" << w.transpose() << "\n";

		Eigen::Matrix3d tmp_rot;
		Eigen::Vector3d rx = w.head<3>();
		Eigen::Vector3d rz = w.segment<3>(3);
		tmp_rot.col(0) = rx;
		tmp_rot.col(1) = rz.cross(rx);
		tmp_rot.col(2) = rz;

		// Orthogonalize rotational matrix
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(tmp_rot, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Matrix3d U = svd.matrixU();
		Eigen::Matrix3d V = svd.matrixV().transpose();
		Eigen::Matrix3d tmp = Eigen::Matrix3d::Identity();
		tmp(2, 2) = (U*V).determinant();
		Eigen::Matrix3d rot_hand_to_sensor = U*tmp*V;

		Eigen::MatrixXd A16 = A.topLeftCorner(rows, 6);
		Eigen::MatrixXd A79 = A.topRightCorner(rows, 3);

		Eigen::VectorXd R_tilde_star(6);
		R_tilde_star.head<3>() = rot_hand_to_sensor.col(0);
		R_tilde_star.tail<3>() = rot_hand_to_sensor.col(2);
		
		Eigen::VectorXd Y_tilde = Y - A16*R_tilde_star;

		Eigen::Vector3d tran_hand_to_sensor = A79.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y_tilde);

		curHandToScanner_ = Eigen::Matrix4d::Identity();
		curHandToScanner_.topLeftCorner<3, 3>() = rot_hand_to_sensor;
		curHandToScanner_.topRightCorner<3, 1>() = tran_hand_to_sensor;

		w.head<6>() = R_tilde_star; w.tail<3>() = curHandToScanner_.topRightCorner<3, 1>();

		double rms_after = sqrt((A*w - Y).squaredNorm() / rows);

		std::cout << "After RMS: " << rms_after << "\n";

		if (rms_after < min_rms)
		{
			min_rms = rms_after;
			handToScanner_ = curHandToScanner_.cast<float>();
		}

		std::cout << "curHandToScanner:\n" << curHandToScanner_ << "\n";
	}

	std::cout << "\nmin RMS: " << min_rms << "\n final handToScanner:\n" << handToScanner_ << "\n";

	std::ofstream file("lineScannerHandEyeCalibration.bin", std::ios::out | std::ios::binary | std::ios::trunc);
	if (file.is_open())
	{
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				file.write((char*)&handToScanner_.row(i)(j), sizeof(float));
	}
	else
	{
		std::cout << "save line scanner hand eye calibration fail\n" ;
	}

	file.close();
}

void VisionArmCombo::addArmModelToViewer(std::vector<PathPlanner::RefPoint>& ref_points)
{
#ifndef NO_VISUALIZATION	
	viewer_->removeAllShapes();
#endif

	for (int i = 0; i < ref_points.size()-1; i++)
	{
		pcl::ModelCoefficients cylinder_coeff;
		cylinder_coeff.values.resize(7);

		PointT p; p.x = ref_points[i].coordinates[0]; p.y = ref_points[i].coordinates[1]; p.z = ref_points[i].coordinates[2];
		PointT p1; p1.x = ref_points[i + 1].coordinates[0]; p1.y = ref_points[i + 1].coordinates[1]; p1.z = ref_points[i + 1].coordinates[2];

		cylinder_coeff.values[0] = p.x; cylinder_coeff.values[1] = p.y; cylinder_coeff.values[2] = p.z;
		cylinder_coeff.values[3] = p1.x - p.x; cylinder_coeff.values[4] = p1.y - p.y; cylinder_coeff.values[5] = p1.z - p.z;
		cylinder_coeff.values[6] = pp_.arm_radius_lookup[i];

#ifndef NO_VISUALIZATION	
		viewer_->addCylinder(cylinder_coeff, "cylinder" + std::to_string(i), 0);
#endif
	}

#ifndef NO_VISUALIZATION	
	display();;
#endif
}

void VisionArmCombo::addOBBArmModelToViewer(std::vector<PathPlanner::OBB> & arm_obbs)
{
#ifndef NO_VISUALIZATION	
	viewer_->removeAllShapes();
	viewer_->removeAllCoordinateSystems();
	viewer_->addCoordinateSystem(0.3, "world", 0);
#endif

	for (int j = 0; j < arm_obbs.size(); j++)
	{
		pcl::ModelCoefficients cube_coeff;
		cube_coeff.values.resize(10);
		for (int i = 0; i < 3; i++) cube_coeff.values[i] = arm_obbs[j].C(i);
		Eigen::Quaternionf quat(arm_obbs[j].A);

		cube_coeff.values[3] = quat.x();
		cube_coeff.values[4] = quat.y();
		cube_coeff.values[5] = quat.z();
		cube_coeff.values[6] = quat.w();

		cube_coeff.values[7] = arm_obbs[j].a(0)*2.f;
		cube_coeff.values[8] = arm_obbs[j].a(1)*2.f;
		cube_coeff.values[9] = arm_obbs[j].a(2)*2.f;

#ifndef NO_VISUALIZATION	
		viewer_->addCube(cube_coeff, "cube"+std::to_string(j), 0);
#endif

		Eigen::Affine3f transform;
		Eigen::Matrix4f temp_mat = Eigen::Matrix4f::Identity();
		temp_mat.block<3, 3>(0, 0) = arm_obbs[j].A;;
		temp_mat.block<3, 1>(0, 3) << arm_obbs[j].C(0), arm_obbs[j].C(1), arm_obbs[j].C(2);
		transform.matrix() = temp_mat;

		//if(j>0) std::cout << "obb collision of " <<j-1<<" and "<<j<<" = " << pp_.collisionOBB(arm_obbs[j-1], arm_obbs[j]) <<"\n";	
		//viewer->addCoordinateSystem(0.2, transform, "co"+std::to_string(j), 0);
	}

#ifndef NO_VISUALIZATION	
	display();;
#endif
}


/*
	https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp
	Analytical solutions + picking the feasible one
*/
int VisionArmCombo::inverseKinematics(Eigen::Matrix4d & T, std::vector<int> & ik_sols_vec)
{
	ik_sols_vec.clear();
	const double q6_des = -PI;
	int num_sols = 0;
	/*double T02 = -*T; T++; double T00 = *T; T++; double T01 = *T; T++; double T03 = -*T; T++;
	double T12 = -*T; T++; double T10 = *T; T++; double T11 = *T; T++; double T13 = -*T; T++;
	double T22 = *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 = *T;*/
	double T00 = T(0, 0); double T01 = T(0, 1); double T02 = T(0, 2); double T03 = T(0, 3);
	double T10 = T(1, 0); double T11 = T(1, 1); double T12 = T(1, 2); double T13 = T(1, 3);
	double T20 = T(2, 0); double T21 = T(2, 1); double T22 = T(2, 2); double T23 = T(2, 3);

	////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
	double q1[2];
	{
		double A = d6*T12 - T13;
		double B = d6*T02 - T03;
		double R = A*A + B*B;
		if (fabs(A) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
				div = -SIGN(d4)*SIGN(B);
			else
				div = -d4 / B;
			double arcsin = asin(div);
			if (fabs(arcsin) < ZERO_THRESH)
				arcsin = 0.0;
			if (arcsin < 0.0)
				q1[0] = arcsin + 2.0*PI;
			else
				q1[0] = arcsin;
			q1[1] = PI - arcsin;
		}
		else if (fabs(B) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
				div = SIGN(d4)*SIGN(A);
			else
				div = d4 / A;
			double arccos = acos(div);
			q1[0] = arccos;
			q1[1] = 2.0*PI - arccos;
		}
		else if (d4*d4 > R) {
			return num_sols;
		}
		else {
			double arccos = acos(d4 / sqrt(R));
			double arctan = atan2(-B, A);
			double pos = arccos + arctan;
			double neg = -arccos + arctan;
			if (fabs(pos) < ZERO_THRESH)
				pos = 0.0;
			if (fabs(neg) < ZERO_THRESH)
				neg = 0.0;
			if (pos >= 0.0)
				q1[0] = pos;
			else
				q1[0] = 2.0*PI + pos;
			if (neg >= 0.0)
				q1[1] = neg;
			else
				q1[1] = 2.0*PI + neg;
		}
	}

	////////////////////////////// wrist 2 joint (q5) //////////////////////////////
	double q5[2][2];
	{
		for (int i = 0; i<2; i++) {
			double numer = (T03*sin(q1[i]) - T13*cos(q1[i]) - d4);
			double div;
			if (fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
				div = SIGN(numer) * SIGN(d6);
			else
				div = numer / d6;
			double arccos = acos(div);
			q5[i][0] = arccos;
			q5[i][1] = 2.0*PI - arccos;
		}
	}
	////////////////////////////////////////////////////////////////////////////////
	{
		for (int i = 0; i<2; i++) {
			for (int j = 0; j<2; j++) {
				double c1 = cos(q1[i]), s1 = sin(q1[i]);
				double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
				double q6;
				////////////////////////////// wrist 3 joint (q6) //////////////////////////////
				if (fabs(s5) < ZERO_THRESH)
					q6 = q6_des;
				else {
					q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1),
						SIGN(s5)*(T00*s1 - T10*c1));
					if (fabs(q6) < ZERO_THRESH)
						q6 = 0.0;
					if (q6 < 0.0)
						q6 += 2.0*PI;
				}
				////////////////////////////////////////////////////////////////////////////////

				double q2[2], q3[2], q4[2];
				///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
				double c6 = cos(q6), s6 = sin(q6);
				double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
				double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
				double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +
					T03*c1 + T13*s1;
				double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

				double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
				if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
					c3 = SIGN(c3);
				else if (fabs(c3) > 1.0) {
					// TODO NO SOLUTION
					continue;
				}
				double arccos = acos(c3);
				q3[0] = arccos;
				q3[1] = 2.0*PI - arccos;
				double denom = a2*a2 + a3*a3 + 2 * a2*a3*c3;
				double s3 = sin(arccos);
				double A = (a2 + a3*c3), B = a3*s3;
				q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
				q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
				double c23_0 = cos(q2[0] + q3[0]);
				double s23_0 = sin(q2[0] + q3[0]);
				double c23_1 = cos(q2[1] + q3[1]);
				double s23_1 = sin(q2[1] + q3[1]);
				q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
				q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
				////////////////////////////////////////////////////////////////////////////////
				for (int k = 0; k<2; k++) {
					if (fabs(q2[k]) < ZERO_THRESH)
						q2[k] = 0.0;
					else if (q2[k] < 0.0) q2[k] += 2.0*PI;
					if (fabs(q4[k]) < ZERO_THRESH)
						q4[k] = 0.0;
					else if (q4[k] < 0.0) q4[k] += 2.0*PI;
					ik_sols_[num_sols * 6 + 0] = q1[i];    ik_sols_[num_sols * 6 + 1] = q2[k];
					ik_sols_[num_sols * 6 + 2] = q3[k];    ik_sols_[num_sols * 6 + 3] = q4[k];
					ik_sols_[num_sols * 6 + 4] = q5[i][j]; ik_sols_[num_sols * 6 + 5] = q6;
					num_sols++;
				}
			}
		}
	}

	// the solution joint angle may not be in the range we want
	for (int i = 0; i < num_sols; i++)
	{
		bool valid_solution = true;

		// try to bring the joint angle back to the range we want
		for (int j = 0; j < 6; j++)
		{
			double min = joint_range_for_probe_[j * 2];
			double max = joint_range_for_probe_[j * 2 + 1];
			double q = ik_sols_[i * 6 + j];

			if (q > max) q -= 2 * PI;
			else if (q < min) q += 2 * PI;
			else continue;

			if (q <= max && q >= min) ik_sols_[i * 6 + j] = q;
			else
			{
				valid_solution = false;
				break;
			}
		}

		if (valid_solution)
		{
			ik_sols_vec.push_back(i);
		/*	std::cout << ik_sols_vec.back() << ": ";
			for (int k = 0; k < 6; k++)
				std::cout << ik_sols_[i * 6 + k] << " ";
			std::cout << "\n";*/
		}
	}

	return num_sols;
}


void VisionArmCombo::forward(const double* q, double* T)
{
	double s1 = sin(*q), c1 = cos(*q); q++;
	double q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
	double s3 = sin(*q), c3 = cos(*q); q234 += *q; q++;
	q234 += *q; q++;
	double s5 = sin(*q), c5 = cos(*q); q++;
	double s6 = sin(*q), c6 = cos(*q);
	double s234 = sin(q234), c234 = cos(q234);
	*T = ((c1*c234 - s1*s234)*s5) / 2.0 - c5*s1 + ((c1*c234 + s1*s234)*s5) / 2.0; T++;
	*T = (c6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0) -
		(s6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0); T++;
	*T = (-(c6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0 -
		s6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0)); T++;
	*T = ((d5*(s1*c234 - c1*s234)) / 2.0 - (d5*(s1*c234 + c1*s234)) / 2.0 -
		d4*s1 + (d6*(c1*c234 - s1*s234)*s5) / 2.0 + (d6*(c1*c234 + s1*s234)*s5) / 2.0 -
		a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3); T++;
	*T = c1*c5 + ((s1*c234 + c1*s234)*s5) / 2.0 + ((s1*c234 - c1*s234)*s5) / 2.0; T++;
	*T = (c6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0) +
		s6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0)); T++;
	*T = (c6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0) -
		s6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0)); T++;
	*T = ((d5*(c1*c234 - s1*s234)) / 2.0 - (d5*(c1*c234 + s1*s234)) / 2.0 + d4*c1 +
		(d6*(s1*c234 + c1*s234)*s5) / 2.0 + (d6*(s1*c234 - c1*s234)*s5) / 2.0 + d6*c1*c5 -
		a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3); T++;
	*T = ((c234*c5 - s234*s5) / 2.0 - (c234*c5 + s234*s5) / 2.0); T++;
	*T = ((s234*c6 - c234*s6) / 2.0 - (s234*c6 + c234*s6) / 2.0 - s234*c5*c6); T++;
	*T = (s234*c5*s6 - (c234*c6 + s234*s6) / 2.0 - (c234*c6 - s234*s6) / 2.0); T++;
	*T = (d1 + (d6*(c234*c5 - s234*s5)) / 2.0 + a3*(s2*c3 + c2*s3) + a2*s2 -
		(d6*(c234*c5 + s234*s5)) / 2.0 - d5*c234); T++;
	*T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
}

void VisionArmCombo::float2double(float* array6_f, double* array6_d)
{
	for (int i = 0; i < 6; i++) array6_d[i] = array6_f[i];
}

void VisionArmCombo::double2float(double* array6_d, float* array6_f)
{
	for (int i = 0; i < 6; i++) array6_f[i] = array6_d[i];
}



double VisionArmCombo::L2Norm(double* array6_1, double* array6_2)
{
	double distance = 0.;
	for (int i = 0; i < 6; i++)
	{
		double  r = array6_1[i] - array6_2[i];
		distance += r*r;
	}
	return sqrt(distance);
}

void VisionArmCombo::getCurHandPose(Eigen::Matrix4f & pose)
{
	double array6[6];

	robot_arm_client_->getCartesianInfo(array6);

	array6ToEigenMat4(array6, pose);
}

void VisionArmCombo::getCurHandPoseD(Eigen::Matrix4d & pose)
{
	double array6[6];

	robot_arm_client_->getCartesianInfo(array6);

	array6ToEigenMat4d(array6, pose);
}

void VisionArmCombo::smallClusterRemoval(PointCloudT::Ptr cloud_in, double clusterTolerance, int minClusterSize, PointCloudT::Ptr cloud_out)
{
	// Euclidean cluster, remove small clusters
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(clusterTolerance); //distance m
	ec.setMinClusterSize(1);
	ec.setMaxClusterSize(cloud_in->points.size());
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_in);
	ec.extract(cluster_indices);

	cloud_out->points.clear();

	for (int j = 0; j<cluster_indices.size(); j++)
	{
		if (cluster_indices[j].indices.size() > minClusterSize)
			for (int i = 0; i<cluster_indices[j].indices.size(); i++)
				cloud_out->push_back(cloud_in->points[cluster_indices[j].indices[i]]);
	}
}

void VisionArmCombo::setScanRadius(float radius)
{
	scan_start_to_hand_(1, 3) = -radius;
	scan_end_to_hand_(1, 3) = radius;
}

#ifndef NO_VISUALIZATION	
void VisionArmCombo::display()
{
	if (view_time_ == 0) viewer_->spin();
	else viewer_->spinOnce(view_time_);
}
#endif

void VisionArmCombo::calibrateKinectRGBCamera()
{
	if (robot_arm_client_ == NULL) initRobotArmClient();
//	if (kinect_thread_ == NULL)	initKinectThread();

//	initEXO_RGB_Cam();
//	exo_rgb_cam_->init();

	int nframes = 36;

	if (nframes % 2 != 0 || nframes < 6)
	{
		std::cout << "number of frames not even or not enough\n";
		return;
	}

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<cv::Mat> image_vec;
	std::vector<Eigen::Matrix4d*> tcp_pose_vec; tcp_pose_vec.resize(nframes);
	cv::Size boardSize, imageSize;
	float squareSize, aspectRatio;

	std::vector<std::vector<cv::Point2f>> imagePoints;

	// IMPORTANT
	cv::SimpleBlobDetector::Params params;
	params.maxArea = 200 * 200;
	params.minArea = 20 * 20;
	cv::Ptr<cv::FeatureDetector> blobDetect = cv::SimpleBlobDetector::create(params);

#if 0
	imageSize.width = kinect_thread_->cColorWidth;
	imageSize.height = kinect_thread_->cColorHeight;
#else
	imageSize.width = 1920;
	imageSize.height = 1200;
#endif

	boardSize.width = 4;
	boardSize.height = 11;

	squareSize = 0.02f;

	for (int i = 0; i < nframes; i++)
	{
		cv::Mat view, viewGray;

		std::vector<cv::Point2f> pointbuf;

		while (true)
		{
#if 0
			view = kinect_thread_->getCurRGB();
#else
			if (exo_rgb_cam_->acquireRGBImage() == -1)
				std::cout << "get RGB time out" << std::endl;
			view = exo_rgb_cam_->rgb_frame;
#endif

			cv::cvtColor(view, viewGray, CV_BGR2GRAY);

			pointbuf.clear();

			// ASYMMETRIC CIRCLE GRID PATTERN
			bool found = findCirclesGrid(view, boardSize, pointbuf, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blobDetect);

			cv::Mat view_copy;
			
			view.copyTo(view_copy);

			if (found)
			{
				cv::drawChessboardCorners(view_copy, boardSize, cv::Mat(pointbuf), found);
				cv::circle(view_copy, pointbuf[0], 30, cv::Scalar(0, 255, 0), 4);
				cv::circle(view_copy, pointbuf[1], 30, cv::Scalar(0, 0, 255), 4);
			}

			cv::Mat shrinked;

			cv::resize(view_copy, shrinked, cv::Size(), 0.5, 0.5);

			cv::imshow("rgb", shrinked);

			int key = cv::waitKey(1);

			//hit space
			if (key == 32)	break;
		}

		std::cout << "Image " << i << " done\n";

		cv::Mat view_save;

		view.copyTo(view_save);

		image_vec.push_back(view_save);

		imagePoints.push_back(pointbuf);

		double array6[6];

		robot_arm_client_->getCartesianInfo(array6);

		Eigen::Matrix4d* tcp_pose = new Eigen::Matrix4d;

		array6ToEigenMat4d(array6, *tcp_pose);

		std::cout << *tcp_pose << "\n";

		tcp_pose_vec[i] = tcp_pose;
	}

	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	std::vector<std::vector<cv::Point3f>> objectPoints(1);

	std::vector<cv::Point3f> corners;

	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(cv::Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
		}
	}

	objectPoints[0] = corners;

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	std::vector<cv::Mat> camera_rotation_vec, camera_translation_vec;	//camera to calibration pattern

	double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, camera_rotation_vec, camera_translation_vec, 0);

	printf("RMS error reported by calibrateCamera: %g\n", rms);

	/*for (int i = 0; i < image_vec.size(); i++)
	{
		std::cout << "rotation\n" << camera_rotation_vec[i] << "\ntranslation\n" << camera_translation_vec[i]<<"\n";
		cv::imshow("image", image_vec[i]);
		cv::waitKey(0);
	}*/

	bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);

	cv::FileStorage fs("kinectRGBCalibration.yml", cv::FileStorage::WRITE);

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "nframes" << nframes;
	fs << "camera poses" << "[";
	for (int i = 0; i < nframes; i++)
	{
		cv::Mat rotation;
		cv::Rodrigues(camera_rotation_vec[i], rotation);
		cv::Mat transform = cv::Mat::eye(4, 4,CV_64F);
		cv::Mat sub = transform(cv::Rect(0, 0, 3, 3));
		rotation.copyTo(sub);
		sub = transform(cv::Rect(3, 0, 1, 3));
		camera_translation_vec[i].copyTo(sub);

		fs << transform;
	}
	fs << "]";

	fs << "TCP poses" << "[";
	for (int i = 0; i < nframes; i++)
	{
		cv::Mat tcp_pose(4, 4, CV_64F);
		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				tcp_pose.at<double>(y, x) = (*tcp_pose_vec[i])(y, x);

		fs << tcp_pose;
	}
	fs << "]";

	fs.release();
}

/*
	robot sensor calibration solving ax=xb on euclidean group 1994
*/
void VisionArmCombo::KinectRGBHandEyeCalibration()
{
	cv::FileStorage fs("kinectRGBCalibration.yml", cv::FileStorage::READ);
	cv::FileNode camera_poses = fs["camera poses"];
	cv::FileNode tcp_poses = fs["TCP poses"];

	int nframes;

	fs["nframes"] >> nframes;

	std::vector<Eigen::Matrix4d*> camera_pose_vec; 
	std::vector<Eigen::Matrix4d*> tcp_pose_vec;	

	// iterate through a sequence using FileNodeIterator
	for (cv::FileNodeIterator it = camera_poses.begin(); it != camera_poses.end(); ++it)
	{
		cv::Mat camera_pose;
		(*it) >> camera_pose;

		Eigen::Matrix4d* transform = new Eigen::Matrix4d;

		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				(*transform)(y, x) = camera_pose.at<double>(y, x);

		camera_pose_vec.push_back(transform);
	}

	for (cv::FileNodeIterator it = tcp_poses.begin(); it != tcp_poses.end(); ++it)
	{
		cv::Mat tcp_pose;
		(*it) >> tcp_pose;

		Eigen::Matrix4d* transform = new Eigen::Matrix4d;

		for (int y = 0; y < 4; y++)
			for (int x = 0; x < 4; x++)
				(*transform)(y, x) = tcp_pose.at<double>(y, x);

		tcp_pose_vec.push_back(transform);
	}

	fs.release();

	// hand eye calibration
	Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
	Eigen::MatrixXd C(3 * nframes *(nframes-1), 3);
	Eigen::VectorXd d(3 * nframes * (nframes-1));
	Eigen::VectorXd bA(3 * nframes * (nframes-1));
	Eigen::VectorXd bB(3 * nframes * (nframes-1));

	int count = 0;

	for (int i = 0; i < nframes; i++)
	{
		for (int j = 0; j < nframes; j++)
		{
			if (i == j) continue;

			// TCP pose motion
			Eigen::Matrix4d A;
			A = (*tcp_pose_vec[i]).inverse() * (*tcp_pose_vec[j]);	//base to robot hand

			// camera pose motion
			Eigen::Matrix4d B;
			B = (*camera_pose_vec[i])*(*camera_pose_vec[j]).inverse();	//camera to calibration board

			//log Rotation
			Eigen::Matrix3d alpha, beta;

			double theta = acos(0.5*(A.block<3, 3>(0, 0).trace() - 1.));

			alpha = theta*0.5 / sin(theta)*(A.block<3, 3>(0, 0) - A.block<3, 3>(0, 0).transpose());

			theta = acos(0.5*(B.block<3, 3>(0, 0).trace() - 1.));

			beta = theta*0.5 / sin(theta)*(B.block<3, 3>(0, 0) - B.block<3, 3>(0, 0).transpose());

			M = M + beta*alpha.transpose();

			C.block<3, 3>(count * 3, 0) = Eigen::Matrix3d::Identity() - A.block<3, 3>(0, 0);
			bA.block<3, 1>(count * 3, 0) = A.block<3, 1>(0, 3);
			bB.block<3, 1>(count * 3, 0) = B.block<3, 1>(0, 3);
			count++;
		}
	}

	Eigen::EigenSolver<Eigen::Matrix3d> es(M.transpose()*M);

	Eigen::Matrix3d lambda;

	lambda = es.eigenvalues().real().cwiseSqrt().cwiseInverse().asDiagonal();
	
	Eigen::Matrix3d hand_to_eye_rotation = es.eigenvectors().real()*lambda*es.eigenvectors().real().inverse()*M.transpose();

	for (int i = 0; i < nframes*(nframes - 1); i++)
		bB.block<3, 1>(i * 3, 0) = hand_to_eye_rotation*bB.block<3, 1>(i * 3, 0);

	d = bA - bB;

	Eigen::Vector3d hand_to_eye_translation = (C.transpose()*C).inverse()*C.transpose()*d;

	cv::Mat hand_to_eye = cv::Mat::eye(4, 4, CV_64F);

	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			hand_to_eye.at<double>(y, x) = hand_to_eye_rotation(y, x);

	for (int i = 0; i < 3; i++)
		hand_to_eye.at<double>(i, 3) = hand_to_eye_translation(i);

	std::cout << "hand to eye\n" << hand_to_eye << "\n";

	cv::FileStorage fs1("kinectRGBHandEyeCalibration.yml", cv::FileStorage::WRITE);

	fs1 << "hand to eye" << hand_to_eye;

	fs1.release();
}

void VisionArmCombo::markerDetection()
{
	cv::Mat markerImage;
	//make pot grid with marker 
	int marker_img_size = 1200;	//multiple of 5+2
	int grid_width = 7;
	int grid_height = 5;

	//cv::aruco::drawMarker(marker_dictionary_, 0, marker_img_size, markerImage, 1);

	cv::Mat center_square;
	center_square.create(marker_img_size, marker_img_size, CV_8UC3);

	std::memset(center_square.ptr(), 255, marker_img_size * marker_img_size * 3);

	float square_size = marker_img_size/4.f*3.5f;
	cv::rectangle(center_square, cv::Point2f((marker_img_size - square_size)*0.5f, (marker_img_size - square_size)*0.5f),
					cv::Point2f(marker_img_size-(marker_img_size - square_size)*0.5f, marker_img_size - (marker_img_size - square_size)*0.5f), cv::Scalar(0,0,0),4);

	cv::Mat block;
	block.create(marker_img_size, marker_img_size, CV_8UC3);
	std::memset(block.ptr(), 255, marker_img_size * marker_img_size * 3);
	
	cv::circle(block, cv::Point2f(marker_img_size*0.5f, marker_img_size*0.5f), 780/2/*marker_img_size / 3*/, cv::Scalar(0, 0, 0), 4);

	cv::circle(block, cv::Point2f(marker_img_size*0.5f, marker_img_size*0.5f), 40, cv::Scalar(0, 0, 0), 40);

	//cv::imshow("block", block);

	//cv::waitKey(0);

	cv::Mat block_grid;

	block_grid.create(marker_img_size * grid_height, marker_img_size * grid_width, CV_8UC3);

	for (int y = 0; y < grid_height; y++)
	{
		for (int x = 0; x < grid_width; x++)
		{
			block.copyTo(block_grid(cv::Rect(x*marker_img_size, y*marker_img_size, marker_img_size, marker_img_size)));
		}
	}

	//cv::Mat marker_img;
	//cv::cvtColor(markerImage, marker_img, CV_GRAY2BGR);
	//marker_img.copyTo(block_grid(cv::Rect((grid_width/2)*marker_img_size, (grid_height/2)*marker_img_size, marker_img_size, marker_img_size)));
	center_square.copyTo(block_grid(cv::Rect((grid_width / 2)*marker_img_size, (grid_height / 2)*marker_img_size, marker_img_size, marker_img_size)));

	cv::imwrite("block_grid.png", block_grid);

	cv::Mat shrink;
	cv::resize(block_grid, shrink, cv::Size(), 0.1, 0.1);

	cv::imshow("block gird", shrink);
	cv::waitKey(0);

	return;
	

	/*	//generate marker images and save
	for (int i = 0; i < 50; i++)
	{
		cv::aruco::drawMarker(marker_dictionary_, i, 700, markerImage, 1);

		cv::imshow("marker", markerImage);

		cv::imwrite("Markers\\marker_" + std::to_string(i) + ".png", markerImage);

		cv::waitKey(100);
	}*/

	if (robot_arm_client_ == NULL) initRobotArmClient();

	if (kinect_thread_ == NULL)	initKinectThread();

	cv::Mat rgb;
	cv::Vec3d rot;
	cv::Vec3d tran;

	ArmConfig config;
	config.setJointPos(-86.5, -99.37, -155.46, -12.91, 90., -178.94);
	config.toRad();

	robot_arm_client_->moveHandJ(config.joint_pos_d, 0.1, 0.1, true);


	while (true)
	{
		rgb = kinect_thread_->getCurRGB();

		std::vector<int> markerIds; 
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::aruco::detectMarkers(rgb, marker_dictionary_, markerCorners, markerIds, detector_params_, rejectedCandidates);
		
		std::vector<cv::Vec3d> rvecs, tvecs;

		cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_length_, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_, rvecs, tvecs);

		cv::aruco::drawDetectedMarkers(rgb, markerCorners, markerIds);

		for (unsigned int i = 0; i < markerIds.size(); i++)
		{
			cv::aruco::drawAxis(rgb, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_, rvecs[i], tvecs[i], marker_length_*0.5f);
			std::cout <<"id "<< i<<" rot " << rvecs[i] << " tran " << tvecs[i]<<"\n";
			rot = rvecs[i];
			tran = tvecs[i];
		}

		cv::imshow("marker", rgb);

		int key = cv::waitKey(10);

		if (key == 113)	//q
		{
			break;
		}
	}

	cv::Mat rgb_to_marker_rot_cv;

	cv::Rodrigues(rot, rgb_to_marker_rot_cv);

	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			cur_rgb_to_marker_(y, x) = rgb_to_marker_rot_cv.at<double>(y, x);

	for (int y = 0; y < 3; y++) cur_rgb_to_marker_(y, 3) = tran[y];

	std::cout << "rgb to marker:\n" << cur_rgb_to_marker_ << "\n";

	Eigen::Matrix4d base_to_hand;

	getCurHandPoseD(base_to_hand);

	Eigen::Matrix3d  rotate_x_pi;

	rotate_x_pi = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

	Eigen::Matrix4d marker_to_gripper = Eigen::Matrix4d::Identity();

	marker_to_gripper.topLeftCorner<3, 3>() = rotate_x_pi.matrix();

	

	Eigen::Matrix4d base_to_marker = base_to_hand*hand_to_rgb_*cur_rgb_to_marker_;

	std::cout << "base to marker:\n" << base_to_marker << "\n";

	for (int y = 2; y >= -2; y--)
	{
		for (int x = -3; x <= 3; x++)
		{
			std::getchar();

			Eigen::Matrix4d marker_to_gripper_translate = Eigen::Matrix4d::Identity();
			marker_to_gripper_translate.col(3).head(3) << x* marker_length_, y* marker_length_, -0.03;

			Eigen::Matrix4d probe_pose_eigen = base_to_marker*marker_to_gripper*marker_to_gripper_translate*probe_to_hand_;

			std::cout << "probe pose:\n" << probe_pose_eigen << "\n";

			// move arm
			double pose[6];

			eigenMat4dToArray6(probe_pose_eigen, pose);

			robot_arm_client_->moveHandL(pose, 0.1, 0.1);
		}
	}

}

void VisionArmCombo::cvTransformToEigenTransform(cv::Mat & cv_transform, Eigen::Matrix4d & eigen_transform)
{
	for (int y = 0; y < 4; y++)
		for (int x = 0; x < 4; x++)
			eigen_transform(y, x) = cv_transform.at<double>(y, x);
}

bool VisionArmCombo::scanSphereOnTable(Eigen::Vector3d & hand_pos, Eigen::Vector3d & sphere_pos, int side)
{
	sphere_count_++;

	if (robot_arm_client_ == NULL) initRobotArmClient();
	if (line_profiler_ == NULL) initLineProfiler();

	double pose[6];
	double tran[3];


	Eigen::Matrix4f cur_pose;
	getCurHandPose(cur_pose);
	if (cur_pose(2, 2) >= 0.)	//check z direction
	{
		std::cout << "gripper pointing down!\n";
		return false;
	}

	double sign;
	if (side == 0) sign = 1.0;
	else if (side == 1) sign = -1.0;
	else std::cout << "wrong side\n";

	//::Ptr scan_cloud(new PointCloudT);

	Eigen::Matrix4d pose_eigen = Eigen::Matrix4d::Zero();

	pose_eigen(1, 0) = -sign;
	pose_eigen(0, 1) = -sign;
	pose_eigen(2, 2) = -1.;
	pose_eigen(3, 3) = 1.;

	pose_eigen(0, 3) = hand_pos(0);
	pose_eigen(1, 3) = hand_pos(1) + sign*0.075;
	pose_eigen(2, 3) = hand_pos(2);

	eigenMat4dToArray6(pose_eigen, pose);

	robot_arm_client_->moveHandL(pose, 0.2, 0.2);
	if (robot_arm_client_->waitTillHandReachDstPose(pose) == -1)
		std::cerr << "get wait pose timeout" << std::endl;

	PointCloudT::Ptr tmp_cloud(new PointCloudT);
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	std::vector<float> center_vec;

	while (true) {
		center_vec.clear();
		//for
		for (int i = 0; i < 2; i++) {

			float scan_vec = 0.3 - i*0.6; // 0.25 - i*0.5;
			//pose[0] = hand_pos(0); pose[1] = hand_pos(1); pose[2] = hand_pos(2);
			//pose[3] = rot_vec[0], pose[4] = rot_vec[1], pose[5] = rot_vec[2];

			tran[0] = 0.;
			tran[1] = scan_vec;
			tran[2] = 0.;

			Sleep(1000);
			/*std::cout << "Ready\n";
			std::getchar();*/

			getCurHandPose(cur_pose);

			line_profiler_->m_vecProfileData.clear();
			//scanTranslateOnly(tran, scan_cloud, scan_acceleration_, scan_speed_);


			scanTranslateOnly(tran, scan_acceleration_, scan_speed_);

			pcl::transformPointCloud(*scan_cloud, *scan_cloud, cur_pose*handToScanner_);

			//viewer_->addPointCloud(scan_cloud, "sphere_cloud" + std::to_string(GetTickCount()), 0); viewer_->spin();


			pass_.setInputCloud(scan_cloud);
			pass_.setFilterFieldName("z");
			pass_.setFilterLimits(0.12, 0.25);
			pass_.setNegative(false);
			pass_.filter(*tmp_cloud);

			//filter x range
			float x_low = -10.f;
			float x_high = 10.f;

			if (sphere_count_ % 3 != 2) {
				x_low = -0.47; x_high = 0.46;
			}
			else if (side == 1 && sphere_count_ % 3 == 2)
			{
				x_high = -1.08;
			}
			else if (side == 0 && sphere_count_ % 3 == 2) {
				x_low = 1.08;
			}

			std::cerr << x_low << "  " << x_high << std::endl;

			pass_.setInputCloud(tmp_cloud);
			pass_.setFilterFieldName("x");
			pass_.setFilterLimits(x_low, x_high);
			pass_.setNegative(false);
			pass_.filter(*tmp_cloud);


			//pcl::io::savePCDFile("Data\\corn\\" + to_string(sphere_count_)+".pcd", *tmp_cloud, true);
			pcl::io::savePCDFile("Data\\corn\\" + getCurrentTimeStr() + ".pcd", *tmp_cloud, true);

			/*
			pcl::visualization::PCLVisualizer viewer;
			viewer.addPointCloud(tmp_cloud, "tmp_cloud");
			viewer.spinOnce(3000);
			viewer.close(); */

			//viewer_->addPointCloud(tmp_cloud, "sphere_cloud" + std::to_string(GetTickCount()), 0); viewer_->spin();


			pcl::SACSegmentation<PointT> seg;
			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_SPHERE); //detecting SPHERE
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.0004);//0.0005
			seg.setRadiusLimits(0.012, 0.013);
			seg.setMaxIterations(100000);
			seg.setInputCloud(tmp_cloud);
			seg.segment(*inliers, coefficients);

			if (coefficients.values.size() == 4)
			{
				std::cout << "sphere_count: " << sphere_count_ << "\n" << coefficients << "\n";
#ifndef NO_VISUALIZATION	
				viewer_->addSphere(coefficients, "sphere" + std::to_string(GetTickCount()), 0);
#endif
				center_vec.push_back(coefficients.values[0]);
				center_vec.push_back(coefficients.values[1]);
				center_vec.push_back(coefficients.values[2]);
				//sphere_pos(0) = coefficients.values[0];
				//sphere_pos(1) = coefficients.values[1];
				//sphere_pos(2) = coefficients.values[2];
			}


			//write loaction to spherelocation
			spherefile.open("spherelocations.txt", ios::app);
			spherefile << getCurrentTimeStr() << "\n";
			spherefile << "side: " << side << "\n";
			spherefile << "coefficients:\n" << coefficients << "\n\n";
			spherefile.close();

		}

		float y_offset = abs(center_vec[1] - center_vec[4]);
		std::cerr << "y direction offset: " << y_offset << std::endl;
		if (y_offset < 0.004) break;
	}

	sphere_pos(0) = (center_vec[0] + center_vec[3]) / 2.f;
	sphere_pos(1) = (center_vec[1] + center_vec[4]) / 2.f;
	sphere_pos(2) = (center_vec[2] + center_vec[5]) / 2.f;

	std::cout << "sphere_pos(0): " << sphere_pos(0) << " sphere_pos(1): " << sphere_pos(1) << " sphere_pos(2): " << sphere_pos(2) << std::endl;

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(tmp_cloud);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*scan_cloud);

#ifndef NO_VISUALIZATION
	viewer_->addPointCloud(scan_cloud, "sphere_cloud" + std::to_string(GetTickCount()), 0);
#endif
	//viewer_->spin();
	kinect_cloud_->clear();
	*kinect_cloud_ = *scan_cloud;

	return true;
}

int VisionArmCombo::sendRoboteqVar(int id, int value)
{
	int status = motor_controller_.SetCommand(_VAR, id, value);

	// send success, but status != RQ_SUCCESS, roboteq bug
	if ( status != RQ_SUCCESS)
	{
		cout << "set Roboteq VAR failed --> " << status << endl;
		return -1;
	}

	sleepms(1000);

	int result = -1;
	status = motor_controller_.GetValue(_VAR, id, result);

	if (status != RQ_SUCCESS)
	{
		cout << "get var failed --> " << status << endl;
	}

	std::cout << "result: " << result << "\n";

	return 0;
}

bool VisionArmCombo::localizeByScanSpheres(int robot_stop, int side)
{
	if (robot_arm_client_ == NULL) initRobotArmClient();
	if (line_profiler_ == NULL) initLineProfiler();
	
	Eigen::Matrix4f cur_pose; getCurHandPose(cur_pose);

	if (cur_pose(2, 2) >= 0.)	//check z direction
	{
		std::cout << "gripper pointing down!\n";
		return false;
	}

	double sign;
	if (side == 0) sign = 1.0;
	else if (side == 1) sign = -1.0;
	else std::cout << "wrong side\n";

	double array6[6];

	// if hand is on the other side, go to an intermediate position first
	if (cur_pose(0, 3)*sign < 0.)	
	{
		double cur_joints[6];
		robot_arm_client_->getCurJointPose(cur_joints);

		array6[0] = 90; array6[1] = -90;
		array6[2] = -90; array6[3] = -90;
		array6[4] = 90; array6[5] = 0;
		for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;

		double dist = 0.;
		for (int i = 0; i < 6; i++)
		{
			dist += std::abs(cur_joints[i] - array6[i]);
		}

		if (dist > 0.01)
		{
			//check hand x coordinate
			if (cur_pose(0, 3) > 0)	//left side table 0
			{
				array6[0] = 0; array6[1] = -90;
				array6[2] = -90; array6[3] = -90;
				array6[4] = 90; array6[5] = 0;
				for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
				robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

				std::cout << "above table 0\n";
			}
			else
			{
				array6[0] = 180; array6[1] = -90;
				array6[2] = -90; array6[3] = -90;
				array6[4] = 90; array6[5] = 0;
				for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
				robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

				std::cerr << "above table 1\n";
			}

			//above balance
			array6[0] = 90; array6[1] = -90;
			array6[2] = -90; array6[3] = -90;
			array6[4] = 90; array6[5] = 0;
			for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
			robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

			std::cerr << "above balance\n";
		}
	}

	if (side == 0)
	{
		array6[0] = 0; array6[1] = -90;
		array6[2] = -90; array6[3] = -90;
		array6[4] = 90; array6[5] = 0;
		for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
		robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

		std::cerr << "above table 0\n";
	}
	else if (side == 1)
	{
		array6[0] = 180; array6[1] = -90;
		array6[2] = -90; array6[3] = -90;
		array6[4] = 90; array6[5] = 0;
		for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
		robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

		std::cerr << "above table 1\n";
	}


	// first ball
	Eigen::Vector3d hand_pos;
	hand_pos << sign*0.44, -0.385, 0.44;

	scanSphereOnTable(hand_pos, sphere_pos1_, side);

	if (sphere_pos1_(2) < 0.04 || sphere_pos1_(2) > 0.16)
	{
		std::cerr << "sphere position out of range\n";
		return false;
	}


	// second ball the other side
	hand_pos(0) += sign*0.685; hand_pos(1) += 0.305;
	scanSphereOnTable(hand_pos, sphere_pos2_, side);

	if (sphere_pos2_(2) < 0.04 || sphere_pos2_(2) > 0.16)
	{
		std::cout << "sphere position out of range\n";
		return false;
	}

	// third ball
	hand_pos(0) -= sign*0.685; hand_pos(1) += 0.3048;

	scanSphereOnTable(hand_pos, sphere_pos3_, side);

	if (sphere_pos3_(2) < 0.04 || sphere_pos3_(2) > 0.16)
	{
		std::cout << "sphere position out of range\n";
		return false;
	}



	//chack fitted sphere is correct
	float x_abs = abs(sphere_pos1_(0) - sphere_pos3_(0));

	float y_abs = sphere_pos1_(1) + sphere_pos3_(1) - 2.f*sphere_pos2_(1);

	std::cerr << "x_abs: " << x_abs << "  y_abs: " << y_abs << std::endl;

	if ((x_abs > 0.05f))
		return false;

//	std::cout << "1-3: " << (sphere_pos1_ - sphere_pos3_).norm() << "\n";
//	std::cout << "1-2: " << (sphere_pos1_ - sphere_pos2_).norm() << "\n";
//	std::cout << "3-2: " << (sphere_pos3_ - sphere_pos2_).norm() << "\n";

	//table surface normal
	table_normal_y_ = (sphere_pos3_ - sphere_pos1_).normalized();
	// projection of sphere 2 on the line of sphere 1 and sphere 3
	Eigen::Vector3d p = sphere_pos1_ + (sphere_pos2_ - sphere_pos1_).dot(table_normal_y_)*table_normal_y_;
	if(side == 0) // left
		table_normal_x_ = (sphere_pos2_ - p).normalized();	
	else
		table_normal_x_ = (p - sphere_pos2_).normalized();
	 
	table_normal_z_ = table_normal_x_.cross(table_normal_y_);

	hand_pose_above_pot_eigen_ = Eigen::Matrix4d::Identity();
	hand_pose_above_pot_eigen_.col(0).head<3>() = -sign*table_normal_y_;
	hand_pose_above_pot_eigen_.col(1).head<3>() = -sign*table_normal_x_;
	hand_pose_above_pot_eigen_.col(2).head<3>() = -table_normal_z_;

	/*
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(kinect_cloud_, "kinect cloud");
	viewer.spinOnce(5000);
	viewer.close();*/

	//display();

	return true;
}

void VisionArmCombo::gotoPot(int robot_stop, int side, int local_pot_x, int local_pot_y, bool open_gripper)
{
	Eigen::Matrix4f cur_pose; getCurHandPose(cur_pose);

	if (cur_pose(2, 2) <= 0.)	//check z direction
	{
		std::cout << "gripper pointing up!\n";
		return;
	}

	double sign;
	if (side == 0) sign = 1.0;
	else if (side == 1) sign = -1.0;
	else std::cout << "wrong side in goto pot\n";
	
	hand_pose_above_pot_eigen_.col(3).head<3>() = sphere_pos3_
		+ table_normal_z_*(0.15)
		+ sign*(table_offset_x_ + local_pot_x*pot2pot_dist_)*table_normal_x_
		+ (table_offset_y_ - local_pot_y*pot2pot_dist_)*table_normal_y_
		;

	// move to pot 
	Eigen::Matrix4d tmp = hand_pose_above_pot_eigen_*hand_to_gripper_.inverse();
	//Eigen::Matrix4d tmp = hand_pose_above_pot_eigen_;

	double array6[6];
	eigenMat4dToArray6(tmp, array6);

	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	if (robot_arm_client_->waitTillHandReachDstPose(array6) == -1)
	{

		std::ofstream myfile;
		myfile.open("timeout_output.txt");
		myfile << "get waitTillHandReachDstPose timeout.\n";
		myfile << current_pot_label_;
		myfile.close();

		std::cerr << "get wait pose timeout" << std::endl;
	}

	//return;

	hand_pose_above_pot_eigen_.col(3).head<3>() += table_normal_z_*(-0.20);

	tmp = hand_pose_above_pot_eigen_*hand_to_gripper_.inverse();

	eigenMat4dToArray6(tmp, array6);

	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	if (robot_arm_client_->waitTillHandReachDstPose(array6) == -1)
	{
		std::ofstream myfile;
		myfile.open("timeout_output.txt");
		myfile << "get waitTillHandReachDstPose timeout.\n";
		myfile << current_pot_label_;
		myfile.close();

		std::cerr << "get wait pose timeout" << std::endl;
	}

	if (open_gripper) gripper_.open();
	else gripper_.close();

	hand_pose_above_pot_eigen_.col(3).head<3>() += table_normal_z_*(0.20);

	tmp = hand_pose_above_pot_eigen_*hand_to_gripper_.inverse();
	eigenMat4dToArray6(tmp, array6);


	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	if (robot_arm_client_->waitTillHandReachDstPose(array6) == -1)
	{

		std::ofstream myfile;
		myfile.open("timeout_output.txt");
		myfile << "get waitTillHandReachDstPose timeout.\n";
		myfile << current_pot_label_;
		myfile.close();

		std::cerr << "get wait pose timeout" << std::endl;
	}

	std::cerr << "reached\n";
}

void VisionArmCombo::scanPot(int robot_stop, int side, int local_pot_x, int local_pot_y)
{
	Eigen::Matrix4f cur_pose; getCurHandPose(cur_pose);

	if (cur_pose(2, 2) >= 0.)	//check z direction
	{
		std::cout << "gripper pointing down!\n";
		return;
	}

	double sign;
	if (side == 0) sign = 1.0;
	else if (side == 1) sign = -1.0;
	else std::cout << "wrong side in scan pot\n";
	
	hand_pose_above_pot_eigen_.col(3).head<3>() = sphere_pos3_
		+ table_normal_z_*(0.17)
		+ sign*(table_offset_x_ + local_pot_x*pot2pot_dist_)*table_normal_x_
		+ (table_offset_y_ - local_pot_y*pot2pot_dist_)*table_normal_y_
		;

	// move to pot 
	Eigen::Matrix4d hand_to_scan_start = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d hand_to_scan_end = Eigen::Matrix4d::Identity();

	hand_to_scan_start(0, 3) = -0.06;
	hand_to_scan_end(0, 3) = 0.06;

	Eigen::Matrix4d start_scan_hand_pose = hand_pose_above_pot_eigen_*hand_to_scan_start;
	Eigen::Matrix4d end_scan_hand_pose = hand_pose_above_pot_eigen_*hand_to_scan_end;

	start_scan_hand_pose.block<3,3>(0,0) *= Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ()).matrix();

	end_scan_hand_pose.block<3, 3>(0, 0) *= Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ()).matrix();

	start_scan_hand_pose *= handToScanner_.cast<double>().inverse();
	end_scan_hand_pose *= handToScanner_.cast<double>().inverse();

	double array6[6];
	eigenMat4dToArray6(start_scan_hand_pose, array6);
	robot_arm_client_->moveHandL(array6, 0.1, 0.1);
	if(robot_arm_client_->waitTillHandReachDstPose(array6)==-1)
		std::cerr << "get wait pose timeout" << std::endl;

	Sleep(1000);

	Eigen::Vector3d translation = end_scan_hand_pose.col(3).head<3>() - start_scan_hand_pose.col(3).head<3>();

	double tran[3] = { translation(0), translation(1), translation(2) };
	
	//PointCloudT::Ptr scan_cloud(new PointCloudT);

	getCurHandPose(cur_pose);

	line_profiler_->m_vecProfileData.clear();
	scanTranslateOnly(tran, scan_acceleration_, scan_speed_);

	pcl::transformPointCloud(*scan_cloud, *scan_cloud, cur_pose*handToScanner_);

	hand_pose_above_pot_eigen_.col(3).head<3>() -= table_normal_z_*(0.07);
	Eigen::Matrix4d hand_pose_for_rgb = hand_pose_above_pot_eigen_*hand_to_rgb_.inverse();

	eigenMat4dToArray6(hand_pose_for_rgb, array6);
	robot_arm_client_->moveHandL(array6, 0.1, 0.1);
	if(robot_arm_client_->waitTillHandReachDstPose(array6)==-1)
		std::cerr << "get wait pose timeout" << std::endl;
	
	Sleep(1000);
	if (exo_rgb_cam_->acquireRGBImage() == -1) {
		std::cout << "get RGB timeout\n";
	}
	cv::Mat undistort;
	cv::undistort(exo_rgb_cam_->rgb_frame, undistort, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_);
	cv::Mat tmp;
	cv::resize(undistort, tmp, cv::Size(), 0.5, 0.5);
	cv::imshow("exo", tmp);
	cv::waitKey(1000);

	std::vector<cv::Point3f> object_points(scan_cloud->points.size());
	for (int i=0; i<scan_cloud->points.size(); i++)
	{
		object_points[i].x = scan_cloud->points[i].x;
		object_points[i].y = scan_cloud->points[i].y;
		object_points[i].z = scan_cloud->points[i].z;
	}
	
	Eigen::Matrix4d tmp_inverse = hand_pose_above_pot_eigen_.inverse();

	cv::Mat rot, tvec, rvec;
	rot.create(3, 3, CV_64F);
	tvec.create(3, 1, CV_64F);
	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			rot.at<double>(y, x) = tmp_inverse(y, x);

	tvec.at<double>(0, 0) = tmp_inverse(0, 3);
	tvec.at<double>(1, 0) = tmp_inverse(1, 3);
	tvec.at<double>(2, 0) = tmp_inverse(2, 3);

	cv::Rodrigues(rot, rvec);
	std::vector<cv::Point2f> img_points;
	cv::projectPoints(object_points, rvec, tvec, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_, img_points);

	for (int i = 0; i < scan_cloud->points.size(); i++)
	{
		int x = std::round(img_points[i].x);
		int y = std::round(img_points[i].y);

		//std::cout << x << "  " << y << "\n";

		if (x >= 0 && x < 1920 && y >= 0 && y < 1200)
		{
			scan_cloud->points[i].b = exo_rgb_cam_->rgb_frame.at<cv::Vec3b>(y, x).val[0];
			scan_cloud->points[i].g = exo_rgb_cam_->rgb_frame.at<cv::Vec3b>(y, x).val[1];
			scan_cloud->points[i].r = exo_rgb_cam_->rgb_frame.at<cv::Vec3b>(y, x).val[2];
		}
	}
#ifndef NO_VISUALIZATION
	viewer_->addPointCloud(scan_cloud, "pot_" + std::to_string(local_pot_x)+"_"+ std::to_string(local_pot_y), 0);
	viewer_->spin();
#endif
}

//void VisionArmCombo::registerRGBandPointCloud(cv::Mat & rgb, PointCloudT::Ptr cloud_in_base, Eigen::Matrix4d & rgb_pose, bool clear_viewer)
void VisionArmCombo::registerRGBandPointCloud(cv::Mat & rgb, Eigen::Matrix4d & rgb_pose, bool clear_viewer)
{
	std::vector<cv::Point3f> object_points(scan_cloud->points.size());
	for (int i = 0; i<scan_cloud->points.size(); i++)
	{
		object_points[i].x = scan_cloud->points[i].x;
		object_points[i].y = scan_cloud->points[i].y;
		object_points[i].z = scan_cloud->points[i].z;
	}

	Eigen::Matrix4d tmp_inverse = rgb_pose.inverse();

	cv::Mat rot, tvec, rvec;
	rot.create(3, 3, CV_64F);
	tvec.create(3, 1, CV_64F);
	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			rot.at<double>(y, x) = tmp_inverse(y, x);

	tvec.at<double>(0, 0) = tmp_inverse(0, 3);
	tvec.at<double>(1, 0) = tmp_inverse(1, 3);
	tvec.at<double>(2, 0) = tmp_inverse(2, 3);

	cv::Rodrigues(rot, rvec);
	std::vector<cv::Point2f> img_points;
	cv::projectPoints(object_points, rvec, tvec, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_, img_points);

	for (int i = 0; i < scan_cloud->points.size(); i++)
	{
		int x = std::round(img_points[i].x);
		int y = std::round(img_points[i].y);

		//std::cout << x << "  " << y << "\n";

		if (x >= 0 && x < 1920 && y >= 0 && y < 1200)
		{
			scan_cloud->points[i].b = exo_rgb_cam_->rgb_frame.at<cv::Vec3b>(y, x).val[0];
			scan_cloud->points[i].g = exo_rgb_cam_->rgb_frame.at<cv::Vec3b>(y, x).val[1];
			scan_cloud->points[i].r = exo_rgb_cam_->rgb_frame.at<cv::Vec3b>(y, x).val[2];
		}
	}

#if 0
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud(cloud_in_base);
	ne.setRadiusSearch(0.002);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
	ne.compute(*cloud_normals);

	std::cout << "normal done\n";

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*cloud_in_base, *cloud);

	pcl::concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);

	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(9);
	poisson.setInputCloud(cloud_smoothed_normals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct(mesh);

	viewer_->addPolygonMesh(mesh, "mesh", 0);
	viewer_->spin();
#endif

#if 0
	if(clear_viewer)
		viewer_->removeAllPointClouds();
	std::string name = "rgb_cloud" + std::to_string(cv::getTickCount());
	viewer_->addPointCloud(cloud_in_base, name);
	viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
	display();
	if(clear_viewer)
		viewer_->removeAllPointClouds();
#endif

	/*std::string name = "rgb_cloud" + std::to_string(cv::getTickCount());
	
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud_in_base, name);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
	viewer.spinOnce(5000);
	viewer.close();*/
}

void VisionArmCombo::transportPotOnBalance(bool open_finger_on_balance)
{
	double array6[6];

	//move above balance
	array6[0] = 124.05;	array6[1] = -49.11;
	array6[2] = -129.18; array6[3] = -91.61;
	array6[4] = 270.06; array6[5] = -34.01;
	for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;

#ifndef JUST_SCAN_A_POT
	robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);
#endif

	std::cerr << "move above balance done\n";

	// put it down
	array6[0] = -0.020; array6[1] = 0.3458; array6[2] = 0.500;
	array6[3] = 0; array6[4] = 0; array6[5] = M_PI;
#ifndef JUST_SCAN_A_POT
	robot_arm_client_->moveHandL(array6, 0.2, 0.1);
	if(robot_arm_client_->waitTillHandReachDstPose(array6)==-1)
		std::cerr << "get wait pose timeout" << std::endl;
#endif

	std::cerr << "put it down\n";

	// final 
	array6[0] = -0.02; array6[1] = 0.3458; array6[2] = 0.361;
	array6[3] = 0; array6[4] = 0; array6[5] = M_PI;
#ifndef JUST_SCAN_A_POT
	robot_arm_client_->moveHandL(array6, 0.2, 0.05);
	if(robot_arm_client_->waitTillHandReachDstPose(array6)==-1)
		std::cerr << "get wait pose timeout" << std::endl;
#endif

	array6ToEigenMat4d(array6, release_pose_);

#ifndef JUST_SCAN_A_POT

	if (open_finger_on_balance)
		gripper_.open();
	else
		gripper_.close();

	Sleep(500);

	// bring it up
	array6[0] = -0.020; array6[1] = 0.3458; array6[2] = 0.500;
	array6[3] = 0; array6[4] = 0; array6[5] = M_PI;
	robot_arm_client_->moveHandL(array6, 0.2, 0.2);
	if(robot_arm_client_->waitTillHandReachDstPose(array6)==-1)
		std::cerr << "get wait pose timeout" << std::endl;

	std::cerr << "bring it up\n";

	//move above balance
	array6[0] = 124.05;	array6[1] = -49.11;
	array6[2] = -129.18; array6[3] = -91.61;
	array6[4] = 270.06; array6[5] = -34.01;
	for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
	robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

	std::cerr << "move above balance\n";
#endif
}

void VisionArmCombo::gotoBalance(int pot_id)
{

	double array6[6];

	if(perfect_stage_)
		transportPotOnBalance(true);


	current_status_file_.open("currentStatusFile.txt", ios::app);

	current_status_file_ << "on_the_balance," << true << "\n";

	current_status_file_.close();

	//switch camera
	if (perfect_stage_) {
		robot_arm_client_->getCurJointPose(array6);
		array6[4] = M_PI_2;
		robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);
	}

	if (!perfect_stage_) {
		array6[0] = 90;
		array6[1] = -90;
		array6[2] = -90; array6[3] = -90;
		array6[4] = 90; array6[5] = 0;
		for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
		robot_arm_client_->moveHandJ(array6, move_joint_speed_, 0.2, true);
	}

	//std::vector<PointCloudT::Ptr> cloud_vec;

	std::vector<std::vector<double>> scan_start_poses;

	std::vector<std::vector<double>> scan_translations;
	
	bool scan_closer = false;

	std::vector<double> scan_pose_z(5);

	if(scan_closer)
		scan_pose_z = { 0.292, 0.298, 0.280, 0.298, 0.292 };
	else
		scan_pose_z = { 0.35, 0.298, 0.298, 0.344, 0.35 };

	std::vector<double> scan_start_pose_0 = { -0.2, 0.439, scan_pose_z[0], M_PI, 0, 0 };
	scan_start_poses.push_back(scan_start_pose_0);
	std::vector<double> scan_translation_0 = { 0.15, 0, 0};
	scan_translations.push_back(scan_translation_0);

	std::vector<double> scan_start_pose_1 = { 0.093, 0.439, scan_pose_z[1], 3.0414, 0, -0.787};
	scan_start_poses.push_back(scan_start_pose_1);
	std::vector<double> scan_translation_1 = { -0.15, 0, 0 };
	scan_translations.push_back(scan_translation_1);

	std::vector<double> scan_start_pose_2 = { -0.38, 0.439, scan_pose_z[2], 3.0414, 0, 0.787 };
	scan_start_poses.push_back(scan_start_pose_2);
	std::vector<double> scan_translation_2 = { 0.15, 0, 0 };
	scan_translations.push_back(scan_translation_2);

	std::vector<double> scan_start_pose_3 = { -0.067, 0.612, scan_pose_z[3], 1.8483, 1.9444, -0.5613 };
	scan_start_poses.push_back(scan_start_pose_3);
	std::vector<double> scan_translation_3 = { 0, -0.15, 0 };
	scan_translations.push_back(scan_translation_3);

	std::vector<double> scan_start_pose_4 = { -0.067, 0.287, scan_pose_z[4], 2.1692, 2.2741, 0.0288 };
	scan_start_poses.push_back(scan_start_pose_4);
	std::vector<double> scan_translation_4 = { 0, 0.15, 0 };
	scan_translations.push_back(scan_translation_4);


#if 1
	Eigen::Matrix4d rgb_pose = release_pose_*hand_to_gripper_;
	rgb_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
	rgb_pose(1, 1) = -1.;
	rgb_pose(2, 2) = -1.;
	rgb_pose(2, 3) = 0.20; //0.17  id=28

	Eigen::Matrix4d hand_pose = rgb_pose*hand_to_rgb_.inverse();
	eigenMat4dToArray6(hand_pose, array6);

	if (!perfect_stage_) {
		array6[0] = -0.0196; array6[1] = 0.3466;
		array6[2] = 0.2466; array6[3] = 3.1416;
		array6[4] = 0.f; array6[5] = 0.f;
	}
		
	perfect_stage_ = true;

	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	std::cerr << "wait before scan" << std::endl;
	if(robot_arm_client_->waitTillHandReachDstPose(array6)==-1)
		std::cerr << "get wait pose timeout" << std::endl;
	std::cerr << "after while, wait before scan" << std::endl;

	Sleep(1000);
	std::cerr << "before acquireRGBImage" << std::endl;

#ifndef disableCam
	while (exo_rgb_cam_->acquireRGBImage() == -1) {
		std::ofstream myfile;
		myfile.open("rgb_cam_timeout.txt", ios::app);
		myfile << "get RGB acquiring timeout.\n";
		myfile <<"time: "<< getCurrentTimeStr() << "\n";
		myfile << pot_id<<"\n";
		myfile<< pot_labels_[pot_id]<<"\n";
		myfile.close();

		exo_rgb_cam_->stop();
		Sleep(100);
		exo_rgb_cam_->init();
		std::cerr << "get RGB timeout\n";
	}
		

	std::cerr << "after acquireRGBImage" << std::endl;
	cv::Mat undistort;
	//undistort.create(exo_rgb_cam_->SizeY, exo_rgb_cam_->SizeX, CV_8UC3);

	try {
		cv::undistort(exo_rgb_cam_->rgb_frame, undistort, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_);
	}
	catch (std::exception e) {
		std::cerr << "exception caught in rgb_frame\n";

		std::cerr << e.what() << std::endl;
	}
#endif
//	cv::Mat undistort = cv::Mat(1,1,CV_64F,0.0); //disable  cam uncommon this
	/*
	std::cerr << "before RGBImage visualize" << std::endl;
	cv::Mat tmp;
	cv::resize(undistort, tmp, cv::Size(), 0.5, 0.5);
	cv::imshow("exo", tmp);
	cv::waitKey(100);
	std::cerr << "after RGBImage visualize" << std::endl;*/
#endif

	
	//std::cout << "OutputFolder: " << OutputFolder << std::endl;
	std::string name = "";
	std::string CurrentPotFolder = CurrentOutputFolder + "\\" +pot_map_[pot_id].label;


	//if (pot_id == 239)
	//	CurrentPotFolder = CurrentOutputFolder + "\\S102_H_154.04_8";


	const char* path = CurrentPotFolder.c_str();
	boost::filesystem::path dir(path);
	if (boost::filesystem::create_directory(dir))
	{
		name = CurrentPotFolder + "\\";
		std::cerr << "Directory Created: " << CurrentPotFolder << std::endl;
	}
	else {
		name = CurrentPotFolder + "\\";
	}


	std::cerr << "before write image\n";

	if (pot_id < pot_labels_.size()) {

		//if (pot_id == 239)
		//	name += "S102_H_154.04_8";
		//else
		name += pot_labels_[pot_id];

		cv::imwrite(name + ".bmp", undistort);
		cv::FileStorage fs(name + ".yml", cv::FileStorage::WRITE);

		cv::Mat rgb_pose_cv;
		
		EigenMatrix4dToCVMat4d(rgb_pose, rgb_pose_cv);

		fs << "rgb_pose" << rgb_pose_cv;

		fs.release();
	}

	std::cerr << "after write image\n";

	undistort.release();

	std::cerr << "after release image\n";

	for (int i = 0; i < scan_start_poses.size(); i++) {
		std::cerr << "before move handL\n";
		robot_arm_client_->moveHandL(scan_start_poses[i].data(), move_arm_acceleration_, move_arm_speed_);
		std::cerr << "after move handL\n";
		if (robot_arm_client_->waitTillHandReachDstPose(scan_start_poses[i].data()) == -1)
		{
		
			std::ofstream myfile;
			myfile.open("timeout_output.txt");
			myfile << "get waitTillHandReachDstPose timeout.\n";
			myfile << pot_labels_[pot_id];
			myfile.close();

			std::cerr << "get wait pose timeout" << std::endl;
		
		}
		Sleep(1000);

		//PointCloudT::Ptr scan_cloud(new PointCloudT);
		//PointCloudT::Ptr tmp_cloud(new PointCloudT);

		Eigen::Matrix4f cur_pose; getCurHandPose(cur_pose);
		
		line_profiler_->m_vecProfileData.clear();

		std::cerr << "before scan\n";

		scanTranslateOnly(scan_translations[i].data(), scan_acceleration_, scan_speed_);
		std::cerr << "scan_cloud size: " << scan_cloud->size() << std::endl;

		pcl::transformPointCloud(*scan_cloud, *tmp_cloud_test, cur_pose*handToScanner_);

		pass_.setInputCloud(tmp_cloud_test);
		pass_.setNegative(false);
		pass_.setFilterFieldName("z");
		pass_.setFilterLimits(0.01, 0.1);
		pass_.filter(*scan_cloud);

		std::cerr << "filtered scan_cloud size: " << scan_cloud->size() << std::endl;
		// empty point cloud, return;
		/*
		if (scan_cloud->size() < 100)
			return;*/

		//empty point cloud, reset camera
		while (scan_cloud->size() < 100) {
			
			//reset laser scanner
			//i=i-1

			/*
			std::cerr << "i: " << i << ", point cloud size less than 100.\n";
			line_profiler_->stop();
			Sleep(1000);
			line_profiler_->finalize();
			Sleep(1000);
			line_profiler_->init();
			Sleep(1000);

			std::cerr << "reset laser scanner done\n";

			i = i - 1;

			std::cerr << "redo, i: " << i << std::endl;
			*/
			Sleep(1000);

			//break;
		}



		uint32_t rgb;

		if (i == 0) rgb = ((uint32_t)255 << 16);
		else if (i == 1) rgb = ((uint32_t)255 << 8);
		else if (i == 2) rgb = ((uint32_t)255);
		else if (i == 3) rgb = ((uint32_t)255<<16 | (uint32_t)255 << 8);

		for (auto & p : scan_cloud->points) 
			p.rgb = *reinterpret_cast<float*>(&rgb);

		//cloud_vec.push_back(scan_cloud);

		//viewer_->addPointCloud(scan_cloud, "scan_cloud"+std::to_string(i), 0);
		//viewer_->spin();
		//viewer_->removePointCloud("scan_cloud"+std::to_string(i));
#if 0
		Eigen::Matrix4d rgb_pose = cur_pose.cast<double>();
		rgb_pose.col(3).head<3>() = (cur_pose.cast<double>() * handToScanner_.col(3).cast<double>()).head<3>();

		rgb_pose.col(3).head<3>() += rgb_pose.col(2).head<3>()*0.05;

		rgb_pose(0, 3) += 0.5*scan_translations[i][0];
		rgb_pose(1, 3) += 0.5*scan_translations[i][1];
		rgb_pose(2, 3) += 0.5*scan_translations[i][2];

		std::cout << "rgb_pose:\n" << rgb_pose << "\n";

		Eigen::Vector3d z_axis = rgb_pose.col(2).head<3>();

	//	rgb_pose.block<3, 3>(0, 0) = rgb_pose.block<3, 3>(0, 0)*Eigen::AngleAxisd(-0.5*M_PI, z_axis).matrix();

		//std::cout << "rgb_pose:\n" << rgb_pose << "\n";


		Eigen::Matrix4d hand_pose = rgb_pose*hand_to_rgb_.inverse();
		eigenMat4dToArray6(hand_pose, array6);

		std::cout << "hand_pose:\n" << hand_pose << "\n";

		std::getchar();

		robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
		robot_arm_client_->waitTillHandReachDstPose(array6);

		Sleep(1000);
		exo_rgb_cam_->acquireRGBImage();
		cv::Mat undistort;
		cv::undistort(exo_rgb_cam_->rgb_frame, undistort, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_);
		cv::Mat tmp;
		cv::resize(undistort, tmp, cv::Size(), 0.5, 0.5);
		cv::imshow("exo", tmp);
		cv::waitKey(100);
#endif
#ifndef disableCam
		registerRGBandPointCloud(exo_rgb_cam_->rgb_frame, rgb_pose, true);
#endif

#if 1
		if (pot_id < pot_labels_.size())
		{
			//pcl::io::savePCDFileBinary(name +"_scan_"+std::to_string(i)+ ".pcd", *scan_cloud);
			pcl::PCDWriter writer;
			writer.writeBinary(name + "_scan_" + std::to_string(i) + ".pcd", *scan_cloud);
		}
#endif
	}

	//fs.release();
	//getchar();
	//return;
	
#if 0
	// move to scan start pose
	array6[0] = -0.2; array6[1] = 0.439; array6[2] = 0.35;
	array6[3] = M_PI; array6[4] = 0; array6[5] = 0;
	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	robot_arm_client_->waitTillHandReachDstPose(array6);
	Sleep(1000);

	PointCloudT::Ptr scan_cloud(new PointCloudT);
	PointCloudT::Ptr tmp_cloud(new PointCloudT);

	Eigen::Matrix4f cur_pose; getCurHandPose(cur_pose);

	double tran[3] = {0.15, 0, 0};
	line_profiler_->m_vecProfileData.clear();
	scanTranslateOnly(tran, scan_cloud, scan_acceleration_, scan_speed_);

	pcl::transformPointCloud(*scan_cloud, *tmp_cloud, cur_pose*handToScanner_);

	pass_.setInputCloud(tmp_cloud);
	pass_.setNegative(false);
	pass_.setFilterFieldName("z");
	pass_.setFilterLimits(-0.04, 0.1);
	pass_.filter(*scan_cloud);

//	viewer_->addPointCloud(scan_cloud, "scan_cloud", 0);
//	viewer_->spin();
//	viewer_->removePointCloud("scan_cloud", 0);

	Eigen::Matrix4d rgb_pose = release_pose_*hand_to_gripper_;
	rgb_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
	rgb_pose(1, 1) = -1.;
	rgb_pose(2, 2) = -1.;
	rgb_pose(2, 3) = 0.17;

	Eigen::Matrix4d hand_pose = rgb_pose*hand_to_rgb_.inverse();
	eigenMat4dToArray6(hand_pose, array6);

	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	robot_arm_client_->waitTillHandReachDstPose(array6);

	Sleep(1000);
	exo_rgb_cam_->acquireRGBImage();
	cv::Mat undistort;
	cv::undistort(exo_rgb_cam_->rgb_frame, undistort, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_);
	cv::Mat tmp;
	cv::resize(undistort, tmp, cv::Size(), 0.5, 0.5);
	cv::imshow("exo", tmp);
	cv::waitKey(100);

	registerRGBandPointCloud(exo_rgb_cam_->rgb_frame, scan_cloud, rgb_pose);

	if (pot_id < pot_labels_.size())
	{
		std::string name = "Data\\" + pot_labels_[pot_id];
		
		pcl::io::savePCDFileBinary(name + ".pcd", *scan_cloud);

		//vector<int> compression_params;
		//compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		//compression_params.push_back(9);
		//cv::imwrite(name + ".png", undistort, compression_params);

		cv::imwrite(name + ".bmp", undistort);
	}
#endif

	pause_operation_if_necessary();

#ifndef JUST_SCAN_A_POT
	//move above balance with camera facing down
	array6[0] = -0.020; array6[1] = 0.3458; array6[2] = 0.500;
	array6[3] = M_PI; array6[4] = 0; array6[5] = 0;
	robot_arm_client_->moveHandL(array6, 0.05, 0.1);  //original 0.1
	if(robot_arm_client_->waitTillHandReachDstPose(array6)==-1)
		std::cerr << "get wait pose timeout" << std::endl;

	//switch gripper
	robot_arm_client_->getCurJointPose(array6);
	array6[4] = M_PI_2*3.;
	std::cerr << "before turning\n";
	robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);
	std::cerr << "ready to watering\n";

	std::cerr << "before open file\n";
	std::ofstream myfile;
	myfile.open(name + "parameters.txt");
	myfile << "sample time: \n" << getCurrentTimeStr() << "\n";

	std::cerr << "before get weight\n";

	myfile << "weight before watering: \n" << pump->getWeight(3) << "\n";

	std::cerr << "before watering weight: "<< pump->getWeight(3) <<endl;

	final_target_weight = pot_map_[pot_id].target_weight;

	int current_day_count;
	std::ifstream day_input("day_count.txt");
	std::string str;
	if (day_input.is_open())
	{
		while (std::getline(day_input, str)) {
			current_day_count = stoi(str);
		}
		std::cerr << "current_day_count: " << current_day_count << endl;
	}
	else {
		std::cerr << "cannot open day count file\n";
	}
	day_input.close();

	float tmp;
	tmp = pot_map_[pot_id].target_weight;

	// this part for exp 13
	/*
	if (slower_drought_) {

		// for exp13 only
		if (tmp < 200)
			tmp = 157.36;//control plants /////change this
		else if (tmp < 300) {
			day_th_1_ = 6; 
			day_th_2_ = day_th_1_ + 14;
			tmp = 74.92; //////////////////change this 
		}
		else if (tmp < 400) {
			day_th_1_ = 8; 
			day_th_2_ = day_th_1_ + 14;
			tmp = 74.92;
		}
		else if (tmp < 500) {
			day_th_1_ = 10;
			day_th_2_ = day_th_1_ + 14;
			tmp = 74.92;
		}

		cout << "current_day_count: " << current_day_count << " target_weight: " << pot_map_[pot_id].target_weight << endl;
		//new design for slower drought,
		cout << "day_th_1_: " << day_th_1_ << " day_th_2_: " << day_th_2_ << endl;

		if (current_day_count < day_th_1_)
			final_target_weight = well_watered_target_weight_;
		else if (current_day_count < day_th_2_)
			final_target_weight = (-well_watered_target_weight_ + tmp) / (day_th_2_ - day_th_1_)*(current_day_count - day_th_1_) + well_watered_target_weight_;
		else {
			final_target_weight = tmp;
		}
	
	}*/

	// uncomment this part for exp after 13
	
	if (slower_drought_) {
		
		if (current_day_count < day_th_1_)
			final_target_weight = well_watered_target_weight_;
		else if (current_day_count < day_th_2_)
			final_target_weight = (-well_watered_target_weight_ + pot_map_[pot_id].target_weight) / (day_th_2_ - day_th_1_)*(current_day_count - day_th_1_) + well_watered_target_weight_;
		else if (current_day_count > day_th_3_)
			final_target_weight = well_watered_target_weight_;
		else {
			final_target_weight = pot_map_[pot_id].target_weight;
		}
	
	}

	cout << "slower_drought_: "<< slower_drought_<<"pot_id: "<< pot_map_[pot_id].label<<"final_target_weight: " << final_target_weight << endl;

	if ((pump->getWeight(3)+0.5f) < final_target_weight) {
	
		//water
		//if (pot_map_[pot_id].water) {
		if (1) {  //current_day_count%2 ==1
			double waterconfig[6] = { 129.17, -58.7, -150.48, -60.81, 269.88, -39.22 }; // { 136.01, -48.31, -155.08, -66.46, 270.02, -45.88 }

			for (int i = 0; i < 6; i++) waterconfig[i] = waterconfig[i] / 180.*M_PI;
			robot_arm_client_->moveHandJ(waterconfig, move_joint_speed_, move_joint_acceleration_, true);
			//Sleep(1000);


			std::cerr << "watering...\n";

			Sleep(500);

			getWater(final_target_weight, 1);

		}
		else {
			double waterconfig[6] = { 111.21, -70.22, -145.30, -54.46, 269.87, -21.24 }; // { 112.55, -69.44, -148.48, -52.01, 270.03, -22.41 };
			for (int i = 0; i < 6; i++) waterconfig[i] = waterconfig[i] / 180.*M_PI;
			robot_arm_client_->moveHandJ(waterconfig, move_joint_speed_, move_joint_acceleration_, true);


			std::cerr << "watering...\n";

			Sleep(500);

			std:cerr << "start watering from pcz tank\n";

			getWater(final_target_weight, 0);

		}



		//std::thread thNew(&VisionArmCombo::getWater,this, 3, true); 	//thNew.join();

		//std::future<int> fu = std::async(std::launch::async,&VisionArmCombo::getWater, this, 3, true);


		//std::cout << "pot_id: " << pot_id << " target_weight: " << pot_map_[pot_id].target_weight << " water: " << pot_map_[pot_id].water << std::endl;

		//getWater(final_target_weight, pot_map_[pot_id].water);

		//getWater(pot_map_[pot_id].target_weight, pot_map_[pot_id].water);

		std::cerr << "watering done\n";

		pause_operation_if_necessary();

		robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

		//	pump->getWeightClear(2);
	
	}
	
	myfile << "weight after watering: \n" << pump->getWeight(3) << "\n";
	myfile << "current day count: " << current_day_count << endl;
	myfile << "final_target_weight: " << final_target_weight << endl;

	myfile.close();

	transportPotOnBalance(false);

	current_status_file_.open("currentStatusFile.txt", ios::app);

	current_status_file_ << "on_the_balance," << false << "\n";

	current_status_file_.close();
#endif
}

void VisionArmCombo::justScan() {

	double array6[6];

	array6[0] = -0.02; array6[1] = 0.3458; array6[2] = 0.351;
	array6[3] = 0; array6[4] = 0; array6[5] = M_PI;

	array6ToEigenMat4d(array6, release_pose_);

	std::vector<PointCloudT::Ptr> cloud_vec;

	std::vector<std::vector<double>> scan_start_poses;

	std::vector<std::vector<double>> scan_translations;

	std::vector<double> scan_start_pose_0 = { -0.19, 0.439, 0.35, M_PI, 0, 0 };
	scan_start_poses.push_back(scan_start_pose_0);
	std::vector<double> scan_translation_0 = { 0.10, 0, 0 };
	scan_translations.push_back(scan_translation_0);
	std::vector<double> scan_translation_1 = { -0.10, 0, 0 };
	scan_translations.push_back(scan_translation_1);

	Eigen::Matrix4d rgb_pose = release_pose_*hand_to_gripper_;
	rgb_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
	rgb_pose(1, 1) = -1.;
	rgb_pose(2, 2) = -1.;
	rgb_pose(2, 3) = 0.40; // original: 0.17;  height when taking RGB image   0.24

	Eigen::Matrix4d hand_pose = rgb_pose*hand_to_rgb_.inverse();
	eigenMat4dToArray6(hand_pose, array6);

	for (int i = 0; i < 6; i++)
		std::cerr << array6[i] << std::endl;
	
	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	robot_arm_client_->waitTillHandReachDstPose(array6);

	// SCAN CORN PLANTS
	std::ifstream input(label_file_path);
	//std::ifstream input("corn_labels.txt");

	if (input.is_open())
	{
		int line_num = 0;
		for (std::string line; std::getline(input, line); line_num++)
		{
			//if (line_num == 0) continue;
			//std::cout << line << std::endl;

			boost::replace_all(line, ",", "_");

			corn_pot_labels_.push_back(line);

			std::cout << line << "\n";
		}

		std::cout << "pot num: " << corn_pot_labels_.size() << "\n";
	}
	else
	{
		std::cout << "cannot open corn file\n";
	}
	cout << "corn_height_: " << corn_height_ << endl;
	//CurrentOutputFolder = "C:\\Users\\phenobot\\Desktop\\RoAD\\corn\\" + getCurrentDateStr();
	CurrentOutputFolder = "C:\\RoAdData\\" + getCurrentDateStr();
	const char* path = CurrentOutputFolder.c_str();
	boost::filesystem::path dir(path);
	if (boost::filesystem::create_directory(dir))
	{
		std::cerr << "Corn Directory Created: " << CurrentOutputFolder << std::endl;
	}

	for (int i = 0; i < corn_pot_labels_.size(); i++) { //4x4  22
		//resetCameras();
		CurrentOutputName = CurrentOutputFolder+ "\\"  + corn_pot_labels_[i];
		path = CurrentOutputName.c_str();
		boost::filesystem::path dir2(path);
		if (boost::filesystem::create_directory(dir2))
		{
			std::cerr << "Corn Directory Created: " << CurrentOutputName << std::endl;
		}

		scanOnePot(i);
		
		//cout << "enter to next turn" << endl;	
		//getchar();
	}

}

void VisionArmCombo::justImage() {

	double array6[6];

	array6[0] = -0.02; array6[1] = 0.3458; array6[2] = 0.351;
	array6[3] = 0; array6[4] = 0; array6[5] = M_PI;

	array6ToEigenMat4d(array6, release_pose_);

	std::vector<PointCloudT::Ptr> cloud_vec;

	std::vector<std::vector<double>> scan_start_poses;

	std::vector<std::vector<double>> scan_translations;


	Eigen::Matrix4d rgb_pose = release_pose_*hand_to_gripper_;
	rgb_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
	rgb_pose(1, 1) = -1.;
	rgb_pose(2, 2) = -1.;
	rgb_pose(2, 3) = 0.20; 

	Eigen::Matrix4d hand_pose = rgb_pose*hand_to_rgb_.inverse();
	eigenMat4dToArray6(hand_pose, array6);

	for (int i = 0; i < 6; i++)
		std::cerr << array6[i] << std::endl;

	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	robot_arm_client_->waitTillHandReachDstPose(array6);

	// SCAN ARAB PLANTS
	std::ifstream input(label_file_path);

	if (input.is_open())
	{
		int line_num = 0;
		for (std::string line; std::getline(input, line); line_num++)
		{
			//if (line_num == 0) continue;
			//std::cout << line << std::endl;

			boost::replace_all(line, ",", "_");

			corn_pot_labels_.push_back(line);

			std::cout << line << "\n";
		}

		std::cout << "pot num: " << corn_pot_labels_.size() << "\n";
	}
	else
	{
		std::cout << "cannot open corn file\n";
	}

	CurrentOutputFolder = "C:\\RoAdData\\" + getCurrentDateStr();
	const char* path = CurrentOutputFolder.c_str();
	boost::filesystem::path dir(path);
	if (boost::filesystem::create_directory(dir))
	{
		std::cerr << "Corn Directory Created: " << CurrentOutputFolder << std::endl;
	}

	//take images

	for (int i = 0; i < corn_pot_labels_.size(); i++) { //4x4  22

		cout << "\n\npot label: " << corn_pot_labels_[i] << endl;
		cout << "if the plant is ready, enter to continue" << endl;
		getchar();

		Sleep(1000);
		std::cout << "before acquireRGBImage" << std::endl;
		exo_rgb_cam_->acquireRGBImage();
		std::cout << "after acquireRGBImage" << std::endl;
		cv::Mat undistort;
		cv::undistort(exo_rgb_cam_->rgb_frame, undistort, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_);


		//save image
		CurrentOutputName = CurrentOutputFolder + "\\" + corn_pot_labels_[i];

		cv::imwrite(CurrentOutputName + ".bmp", undistort);
		cv::FileStorage fs(CurrentOutputName + ".yml", cv::FileStorage::WRITE);

		cv::Mat rgb_pose_cv;

		EigenMatrix4dToCVMat4d(rgb_pose, rgb_pose_cv);

		fs << "rgb_pose" << rgb_pose_cv;

		fs.release();

		cout << "image saved\n";

		std::cout << "before RGBImage visualize" << std::endl;
		cv::Mat tmp;
		cv::resize(undistort, tmp, cv::Size(), 0.5, 0.5);
		cv::imshow("exo", tmp);
		cv::waitKey(100);
		std::cout << "after RGBImage visualize" << std::endl;
		
	}
}

void VisionArmCombo::scanOnePot(int pot_id) {

	cout << "\n\npot label: " << corn_pot_labels_[pot_id] << endl;
	cout << "enter to continue" << endl;
	getchar();

	double array6[6];

	array6[0] = -0.02; array6[1] = 0.3458; array6[2] = 0.351;
	array6[3] = 0; array6[4] = 0; array6[5] = M_PI;

	array6ToEigenMat4d(array6, release_pose_);

	std::vector<PointCloudT::Ptr> cloud_vec;

	std::vector<std::vector<double>> scan_start_poses;

	std::vector<std::vector<double>> scan_translations;

	//hieght vec  the last element is for rgb pose
	cout << "corn_height_: " << corn_height_ << endl;

	std::vector<double> height_vec = { 0.35, 0.298, 0.298, 0.344, 0.35, 0.17 };  //orignal
	if(corn_height_.compare("middle")==0)
		height_vec = { 0.46, 0.490, 0.460, 0.497, 0.475, 0.4 };  //middle
	else if(corn_height_.compare("high")==0)
		height_vec = { 0.56, 0.590, 0.560, 0.597, 0.475, 0.65 }; //high
	else if(corn_height_.compare("pretty high") == 0)
		height_vec = { 0.61, 0.640, 0.610, 0.647, 0.525, 0.65 };  //pretty high

	std::vector<double> scan_start_pose_0 = { -0.35, 0.439, height_vec[0], M_PI, 0, 0 };
	scan_start_poses.push_back(scan_start_pose_0);
	std::vector<double> scan_translation_0 = { 0.5, 0, 0 };  //0.15
	scan_translations.push_back(scan_translation_0);

	std::vector<double> scan_start_pose_1 = { 0.190, 0.439, height_vec[1], 3.0414, 0, -0.787 };
	scan_start_poses.push_back(scan_start_pose_1);
	std::vector<double> scan_translation_1 = { -0.39, 0, 0 };  //0.15
	scan_translations.push_back(scan_translation_1);

	std::vector<double> scan_start_pose_2 = { -0.47, 0.439, height_vec[2], 3.0414, 0, 0.787 };
	scan_start_poses.push_back(scan_start_pose_2);
	std::vector<double> scan_translation_2 = { 0.475, 0, 0 }; //0.15
	scan_translations.push_back(scan_translation_2);

	std::vector<double> scan_start_pose_3 = { -0.067, 0.700, height_vec[3], 1.8483, 1.9444, -0.5613 };
	scan_start_poses.push_back(scan_start_pose_3);
	std::vector<double> scan_translation_3 = { 0, -0.25, 0 }; //0.15
	scan_translations.push_back(scan_translation_3);

	std::vector<double> scan_start_pose_4 = { -0.067, 0.287, height_vec[4], 2.1692, 2.2741, 0.0288 };
	scan_start_poses.push_back(scan_start_pose_4);
	std::vector<double> scan_translation_4 = { 0, 0.15, 0 };
	scan_translations.push_back(scan_translation_4);

#if 1
	Eigen::Matrix4d rgb_pose = release_pose_*hand_to_gripper_;
	rgb_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
	rgb_pose(1, 1) = -1.;
	rgb_pose(2, 2) = -1.;
	rgb_pose(2, 3) = height_vec[5]; // original: 0.17;  height when taking RGB image   0.24   last time 0.65

	Eigen::Matrix4d hand_pose = rgb_pose*hand_to_rgb_.inverse();
	eigenMat4dToArray6(hand_pose, array6);

	for (int m = 0; m < 6; m++)
		std::cout << "array " << m << " : " << array6[m] << std::endl;


	robot_arm_client_->moveHandL(array6, move_arm_acceleration_, move_arm_speed_);
	std::cout << "wait before scan" << std::endl;
	robot_arm_client_->waitTillHandReachDstPose(array6);
	std::cout << "after while, wait before scan" << std::endl;

	Sleep(1000);
	std::cout << "before acquireRGBImage" << std::endl;
	exo_rgb_cam_->acquireRGBImage();
	std::cout << "after acquireRGBImage" << std::endl;
	cv::Mat undistort;
	cv::undistort(exo_rgb_cam_->rgb_frame, undistort, kinect_rgb_camera_matrix_cv_, kinect_rgb_dist_coeffs_cv_);
	std::cout << "before RGBImage visualize" << std::endl;
	cv::Mat tmp;
	cv::resize(undistort, tmp, cv::Size(), 0.5, 0.5);
	cv::imshow("exo", tmp);
	cv::waitKey(100);
	std::cout << "after RGBImage visualize" << std::endl;

#endif

	std::string name = "";

	if (pot_id < corn_pot_labels_.size()) {

		//name = "C:\\Users\\phenobot\\Desktop\\RoAD\\corn\\" + corn_pot_labels_[pot_id];
		

		name= CurrentOutputName + "\\" + corn_pot_labels_[pot_id];

		cv::imwrite(name + ".bmp", undistort);
		cv::FileStorage fs(name + ".yml", cv::FileStorage::WRITE);

		cv::Mat rgb_pose_cv;

		EigenMatrix4dToCVMat4d(rgb_pose, rgb_pose_cv);

		fs << "rgb_pose" << rgb_pose_cv;

		fs.release();
	}

	for (int i = 0; i < scan_start_poses.size(); i++) {

		robot_arm_client_->moveHandL(scan_start_poses[i].data(), move_arm_acceleration_, move_arm_speed_);
		robot_arm_client_->waitTillHandReachDstPose(scan_start_poses[i].data());
		Sleep(1000);

	//	PointCloudT::Ptr scan_cloud(new PointCloudT);
		PointCloudT::Ptr tmp_cloud(new PointCloudT);
		auto_ptr<PointCloudT> tme_cloud2(new PointCloudT);

		Eigen::Matrix4f cur_pose; getCurHandPose(cur_pose);

		line_profiler_->m_vecProfileData.clear();
		scanTranslateOnly(scan_translations[i].data(), scan_acceleration_, scan_speed_);

		pcl::transformPointCloud(*scan_cloud, *tmp_cloud, cur_pose*handToScanner_);

		pass_.setInputCloud(tmp_cloud);
		pass_.setNegative(false);
		pass_.setFilterFieldName("z");
		pass_.setFilterLimits(0.01, 0.6);
		pass_.filter(*scan_cloud);

		//delete(tmp_cloud);
		

		// empty point cloud, return;
		if (scan_cloud->size() < 100)
			return;

		uint32_t rgb;

		if (i == 0) rgb = ((uint32_t)255 << 16);
		else if (i == 1) rgb = ((uint32_t)255 << 8);
		else if (i == 2) rgb = ((uint32_t)255);
		else if (i == 3) rgb = ((uint32_t)255 << 16 | (uint32_t)255 << 8);

		for (auto & p : scan_cloud->points)
			p.rgb = *reinterpret_cast<float*>(&rgb);

		cloud_vec.push_back(scan_cloud);

		std::wcout << "scan_cloud " << scan_cloud->size() << std::endl;
		//viewer_->addPointCloud(scan_cloud, "scan_cloud"+std::to_string(i), 0);
		//viewer_->spin();
		//viewer_->removePointCloud("scan_cloud"+std::to_string(i));

		registerRGBandPointCloud(exo_rgb_cam_->rgb_frame, rgb_pose, true);

#if 1
		if (pot_id < corn_pot_labels_.size())
		{
			pcl::io::savePCDFileBinary(name + "_scan_" + std::to_string(i) + ".pcd", *scan_cloud);
		}
#endif
	}
}

void VisionArmCombo::placePots(int operation)
{
	//std::vector<int> noPlants = { 10,21,23,39,45,50,62,72,73,81,84,99,106,129,132,143,146,153,165,186,221,227,237,238 }; 

	//std::vector<int>::iterator it;

	//it = noPlants.begin();

	//get current day_count

	int current_day_count;
	std::ifstream day_input("day_count.txt");
	std::string str;
	if (day_input.is_open())
	{
		while (std::getline(day_input, str)) {
			current_day_count = stoi(str);
		}
		std::cerr << "current_day_count: " << current_day_count << endl;
	}
	else {
		std::cerr << "cannot open day count file\n";
	}
	day_input.close();

	cout << "current_day_count: " << current_day_count << endl;

	std::ifstream input("currentStatusFile.txt");


	if (input.is_open())
	{
		std::cerr << "\n\ncurrent status file\n";
		int line_num = 0;
		for (std::string line; std::getline(input, line); line_num++)
		{
			std::vector<std::string> str_vec;
			
			std::cerr << line << std::endl;
			boost::split(str_vec, line, boost::is_any_of(","));

			if (line_num == 0) start_pot_id_ = stoi(str_vec[1]);

			if (line_num == 1 && stoi(str_vec[1]) == pot_map_[start_pot_id_].robot_stop) start_robot_stop_ = stoi(str_vec[1]);

			if (line_num == 2 && stoi(str_vec[1]) == pot_map_[start_pot_id_].side) start_arm_side_ = stoi(str_vec[1]);

			if (line_num > 2 && str_vec[0] == "on_the_balance" && str_vec[1] == "1") perfect_stage_ = false;	

			if (line_num > 2 && str_vec[0] == "on_the_balance" && str_vec[1] == "0") perfect_stage_ = true;

		}
			
		if (perfect_stage_) {

			if (start_pot_id_ == (pot_map_.size()-1)) {
				start_pot_id_ = 0;
			}
			//else if (line_num > 4) {
			//	start_pot_id_++;
			//}

			start_robot_stop_ = pot_map_[start_pot_id_].robot_stop;
			start_arm_side_ = pot_map_[start_pot_id_].side;
		}


		std::cerr << "\n\n\n******************check status************************\n";
		std::cerr << "1. initial_pot_id_: " << start_pot_id_ << std::endl;
		std::cerr << "2. initial_robot_stop_: " << start_robot_stop_ << std::endl;
		std::cerr << "3. initial_arm_side_: " << start_arm_side_ << std::endl;
		if (perfect_stage_)
			std::cerr << "4. The pot is on the table and with correct position\n";
		else {
			std::cerr << "4. The pot is on the balance\n";

			std::cerr << "If you've checked everything above, enter to continue\n";

			//std::getchar();
			
		}

		std::cerr << "The robot is ready to go\n";

		Sleep(3000);

	}
	else
	{
		std::cerr << "cannot open file\n";
	}

	input.close();
		

	///////////////////////////////////////////////////////
	if (operation != PICK_POT && operation != PLACE_POT)
	{
		std::cerr << "invalid operation\n";
		return;
	}

	Eigen::Matrix4f cur_pose; getCurHandPose(cur_pose);

	double array6[6];
 

	if (!perfect_stage_) {
			
		robot_arm_client_->getCurJointPose(array6);

		array6[1] = -M_PI_2;
		array6[2] = -M_PI_2; 
		array6[3] = -0.5*M_PI;
		array6[5] = 0;
		if (cur_pose(2, 2) >= 0.) {		
			array6[4] = 1.5*M_PI; 
		}
		else {
			array6[4] = 0.5*M_PI; 
		}

		//array6[3]
		robot_arm_client_->moveHandJ(array6, move_joint_speed_, 0.2, true);

	}

	getCurHandPose(cur_pose);

#if 1
	if (cur_pose(2, 2) >= 0.)	//check z direction
	{
		std::cerr << "gripper pointing down!\n";
		robot_arm_client_->getCurJointPose(array6);

		array6[1] = -0.5*M_PI;
		array6[2] = -0.5*M_PI; array6[3] = -0.5*M_PI;
		array6[4] = 1.5*M_PI; array6[5] = 0;

		robot_arm_client_->moveHandJ(array6, move_joint_speed_, 0.2, true);

		array6[4] = 0.5*M_PI; array6[5] = 0;
		robot_arm_client_->moveHandJ(array6, move_joint_speed_, 0.2, true);

		std::cerr << "gripper pointing up!\n";
		
		//return;
	}
#endif

	//if (!perfect stage)  leaf side, right side, go to 90

	//directory for saving data
	if (operation == PICK_POT) {
	
		CurrentOutputFolder = "C:\\RoAdData\\" + getCurrentDateStr();
		const char* path = CurrentOutputFolder.c_str();
		boost::filesystem::path dir(path);
		if (boost::filesystem::create_directory(dir))
		{
			std::cerr << "Directory Created: " << CurrentOutputFolder << std::endl;
		}

	}
	
	for (int cur_side = 1; cur_side >= 0; cur_side--)
	{

		//1: right side of the robot; 0: left side
		for (int cur_robot_stop = 0; cur_robot_stop < 3; cur_robot_stop++)
		{

			if (scanOrder[cur_side][cur_robot_stop] < scanOrder[start_arm_side_][start_robot_stop_]) continue;

			std::cout << "side: " << cur_side << "stop: " << cur_robot_stop << " order: " << scanOrder[cur_side][cur_robot_stop] << std::endl;

			if (perfect_stage_)
				pump->getWeightClear(3);
#if 1
			// move robot to destination
			sendRoboteqVar(1, cur_robot_stop + 1);	//roboteq start from 1

			std::cerr << "sent\n";

			Sleep(1000);

			while (true)
			{
				int result = -1;
				int target_marker_id = -1;
				motor_controller_.GetValue(_VAR, 2, result);	//get current status
				motor_controller_.GetValue(_VAR, 1, target_marker_id);	//get current status
				std::cerr << "status: " << result << " target marker:" << target_marker_id << "\n";
				if (result == 1) break;
				Sleep(500);
			}

			std::cerr << "stay in tis position for 1 s" << std::endl;
			Sleep(1000);

		
#endif
			//resetCameras();

#ifndef NO_VISUALIZATION
			viewer_->removeAllShapes();
#endif

#ifndef JUST_SCAN_A_POT
			if (localizeByScanSpheres(cur_robot_stop, cur_side))	//comment if only scan a pot
#endif
			{ 				
				if (operation == PLACE_POT)	// move to operator
					array6[0] = cur_side == 0 ? 45 : 135;
				else if (operation == PICK_POT ) // move hand above table
					array6[0] = cur_side == 0 ? 0 : 180;
				array6[1] = -90;
				array6[2] = -90; array6[3] = -90;
				array6[4] = 90; array6[5] = 0;
				for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;

				//disable if only scan a pot

				if(perfect_stage_)
					robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

				// rotate gripper down
				array6[4] = M_PI_2 * 3;
				if(perfect_stage_)
					robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);


				int count = 0;

				// iterate through global pot id
				for (int pot_id = 0; pot_id < pot_map_.size(); pot_id++)
				{				
					// target weight < 0 -> no pot		
				
					if (pot_id < start_pot_id_)  continue;

					//no plant
				/*	it = find(noPlants.begin(), noPlants.end(), pot_id);
					if (it != noPlants.end()) continue;
*/
					//for test
					//if (pot_id != 41 && pot_id != 77 && pot_id != 119 && pot_id != 161&&pot_id!=197 && pot_id != 239) continue;
					//if (pot_id == 113 || pot_id == 114) continue;
					
					if (/*count%30 == 0 &&*/ pot_map_[pot_id].target_weight >= 0.f && pot_map_[pot_id].robot_stop == cur_robot_stop && pot_map_[pot_id].side == cur_side)
					{


						//std::cout << pot_map_[pot_id].label << " stop: " << pot_map_[pot_id].robot_stop << " side: " << pot_map_[pot_id].side << " localx: " << pot_map_[pot_id].local_pot_x << " localy: " << pot_map_[pot_id].local_pot_y << std::endl;
						//write finished pot_id to current_status_file_
						
						current_status_file_.open("currentStatusFile.txt");
						current_status_file_ << "pot_id," << pot_id << "\n"; 
						current_status_file_ << "cur_stop," << cur_robot_stop << "\n";
						current_status_file_ << "cur_side," << cur_side << "\n";
						
						current_status_file_ << "on_the_balance," << false << "\n";

						current_status_file_.close();

						if (operation == PLACE_POT)
						{
							std::cerr << "put in the pot\n";

							current_pot_label_ = pot_map_[pot_id].label;
							std::cerr << "pot_id: " << pot_id << "  " << current_pot_label_ << std::endl;
							if(pot_id!=239)
								std::cerr << "next_id: " << pot_id+1 << "  " << pot_map_[pot_id+1].label << std::endl;

							pause_operation_if_necessary();

							array6[0] = 90; // cur_side == 0 ? 45 : 135;
							array6[1] = -90;
							array6[2] = -90; array6[3] = -90;
							array6[4] = 270; array6[5] = 0;
							for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
							robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

							pause_operation_if_necessary();
							gripper_.close();
							
							
							//Sleep(3000);
							std::getchar();

							pause_operation_if_necessary();

							if (pot_id>=210) {

								array6[0] = 27; // cur_side == 0 ? 45 : 135;
								array6[1] = -90;
								array6[2] = -90; array6[3] = -90;
								array6[4] = 270; array6[5] = 0;
								for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
								robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

							}
							gotoPot(cur_robot_stop, cur_side, pot_map_[pot_id].local_pot_x, pot_map_[pot_id].local_pot_y, true);

							if (cur_robot_stop == 0 && cur_side==1 ) {

								array6[0] = 165.53; // cur_side == 0 ? 45 : 135;
								array6[1] = -90;
								array6[2] = -90; array6[3] = -90;
								array6[4] = 270; array6[5] = 0;
								for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
								robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

							}
							if (cur_robot_stop == 0 && cur_side == 0) {

								array6[0] = 45; // cur_side == 0 ? 45 : 135;
								array6[1] = -90;
								array6[2] = -90; array6[3] = -90;
								array6[4] = 270; array6[5] = 0;
								for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
								robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

							}

						}
						else if (operation == PICK_POT)
						{

#ifndef JUST_SCAN_A_POT 
							pause_operation_if_necessary();
							gripper_.open();	//comment to just scan
#else
							//	gripper_.close(); std::getchar();
#endif

							Sleep(500);

#ifndef JUST_SCAN_A_POT 
							pause_operation_if_necessary();

							if(perfect_stage_)
								gotoPot(cur_robot_stop, cur_side, pot_map_[pot_id].local_pot_x, pot_map_[pot_id].local_pot_y, false); //comment to just scan
#endif	
							pause_operation_if_necessary();
			
							current_pot_label_ = pot_map_[pot_id].label;
							std::cerr << "pot_id: " << pot_id << "  " << current_pot_label_ << std::endl;

							//if (cur_robot_stop == 0 && cur_side == 1&&perfect_stage_)
							if (cur_robot_stop == 0 && cur_side == 1) {

								array6[0] = 165; // cur_side == 0 ? 45 : 135;
								array6[1] = -90;
								array6[2] = -90; array6[3] = -90;
								array6[4] = 270; array6[5] = 0;
								if (!perfect_stage_) array6[4] = 90;
								for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
								robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

							}

							//if (cur_robot_stop == 0 && cur_side == 0&&perfect_stage_)
							if (cur_robot_stop == 0 && cur_side == 0) {

								array6[0] = 30; // cur_side == 0 ? 45 : 135;
								array6[1] = -90;
								array6[2] = -90; array6[3] = -90;
								array6[4] = 270; array6[5] = 0;
								if (!perfect_stage_) array6[4] = 90;
								for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
								robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

							}

							gotoBalance(pot_id);

#ifndef JUST_SCAN_A_POT 
				
							pause_operation_if_necessary();

							if ((pot_id > 179 && cur_side == 0)||(cur_side == 0 && pot_id > 59 && pot_id<84)||(cur_side == 0 && pot_id>131 &&pot_id<156)) {

								array6[0] = 27; // cur_side == 0 ? 45 : 135;
								array6[1] = -90;
								array6[2] = -90; array6[3] = -90;
								array6[4] = 270; array6[5] = 0;
								for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
								robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

							}

							gotoPot(cur_robot_stop, cur_side, pot_map_[pot_id].local_pot_x, pot_map_[pot_id].local_pot_y, true); //comment to just scan

							std::cerr << "clear weight\n" << std::endl;

							
							pump->getWeightClear(3);

							while (pump->getWeight(3) < -10.f ) {
								pump->getWeightClear(3);
								Sleep(100);
							}
#endif
						}
					}

					count++;
				}

				if (cur_robot_stop == 0 && cur_side == 1) {

					array6[0] = 165; // cur_side == 0 ? 45 : 135;
					array6[1] = -90;
					array6[2] = -90; array6[3] = -90;
					array6[4] = 270; array6[5] = 0;
					if (!perfect_stage_) array6[4] = 90;
					for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
					robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

				}

				//if (cur_robot_stop == 0 && cur_side == 0&&perfect_stage_)
				if (cur_robot_stop == 0 && cur_side == 0) {

					array6[0] = 30; // cur_side == 0 ? 45 : 135;
					array6[1] = -90;
					array6[2] = -90; array6[3] = -90;
					array6[4] = 270; array6[5] = 0;
					if (!perfect_stage_) array6[4] = 90;
					for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
					robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

				}

			}  //if localizeByScanSpheres
			else {
				std::cerr << "localize return false\n";

				if (cur_robot_stop == 0 && cur_side == 1) {

					array6[0] = 165; array6[1] = -90;
					array6[2] = -90; array6[3] = -90;
					array6[4] = 270; array6[5] = 0;
					for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
					robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

				}


				if (cur_robot_stop == 0 && cur_side == 0) {

					array6[0] = 30;  array6[1] = -90;
					array6[2] = -90; array6[3] = -90;
					array6[4] = 270; array6[5] = 0;
					for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
					robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

				}

			}   //localizeByScanSpheres
				
			// rotate gripper up
			//above balance with gripper down
			array6[0] = 90; array6[1] = -90;
			array6[2] = -90; array6[3] = -90;
			array6[4] = 270; array6[5] = 0;
			for (int i = 0; i < 6; i++)	array6[i] = array6[i] / 180.*M_PI;
			robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

			array6[4] = M_PI*0.5;
			robot_arm_client_->moveHandJ(array6, move_joint_speed_, move_joint_acceleration_, true);

			std::cerr << "robot stop: " << cur_robot_stop << "   side: " << cur_side << "\n";
			//std::getchar();
			//return;  //uncomment for pickpot
			if (current_pot_label_.compare(pot_map_[pot_map_.size()-1].label) == 0)
				break;
		} //for cur_robot_stop		
		if (current_pot_label_.compare(pot_map_[pot_map_.size() - 1].label) == 0)
			break;
	}  //for cur_side
}

void VisionArmCombo::initEXO_RGB_Cam()
{
	exo_rgb_cam_ = new EXO_RGB_CAM();
	exo_rgb_cam_->exposure_time_ = exposure_time_;
	exo_rgb_cam_->init();
}

int VisionArmCombo::initPotMap()
{
	//std::ifstream input("pot_labels.txt");
	//std::ifstream input("experiment3_2018_07_02.csv");
	std::ifstream input(label_file_path);
	//std::ifstream input("exp5_09_24.txt");

	string last_label;

	if (input.is_open())
	{
		int line_num = 0;
		for (std::string line; std::getline(input, line); line_num++)
		{
			if (line_num == 0) continue;
			std::cout << line << std::endl;

		

			std::vector<std::string> str_vec;
			boost::split(str_vec, line, boost::is_any_of(";,"));

			if (str_vec.size() != 4) {

				std::cerr << "cvs file line " << line_num + 1 << " wrong format!\n";

				return -1;
			}

			boost::replace_all(line, ",", "_");
			boost::replace_all(line, ";", "_");
			boost::replace_all(line, "\"", "");


			//pot_labels_ saves file name
			pot_labels_.push_back(line);

			std::string::size_type sz;     // alias of size_t

			//float target_weight = std::stof(str_vec.back(), &sz);

			float target_weight = std::stof(str_vec[2], &sz);

			pot_target_weight_.push_back(target_weight);

			bool water = str_vec[1] == "W";
			pot_water_.push_back(water);

			std::vector<std::string> str_vec_new;
			boost::split(str_vec_new, str_vec[0], boost::is_any_of("-"));

			if (str_vec_new.size() > 2)
				pot_day_th2_.push_back(stoi(str_vec_new[2]));


			std::cerr << line << "\n";

			//std::cout << "target_weight: " << target_weight <<"  water: "<<water<< std::endl;
		}

		std::cerr << "pot num: " << pot_labels_.size() << "\n";

		//last_label = pot_labels_[239];
	}
	else
	{
		std::cerr << "cannot open label file\n";
	}

	input.close();

	pot_map_.resize(pot_labels_.size());  //pot_map_.resize(max_pots_);
	int tmpy;
	for (int i = 0; i< pot_labels_.size(); i++)  //max_pots_
	{

		int x = i % (table_cols_/2);  
		int y = i / (table_cols_/2);

		Pot* p = &pot_map_[i];
		

		if (i < pot_target_weight_.size()) {
			p->target_weight = pot_target_weight_[i];
			p->water = pot_water_[i];
			p->label = pot_labels_[i];
			//p->pot_day_th2 = pot_day_th2_[i];
		}
		else
			p->target_weight = -1;

		tmpy = y % 20;

		if (tmpy < 7)
		{
			p->robot_stop = 0;
			p->local_pot_y = tmpy;
		}
		else if (tmpy < 13)
		{
			p->robot_stop = 1;
			p->local_pot_y = tmpy - 7 + 1;	// 6 rows in the stop 1
		}
		else
		{
			p->robot_stop = 2;
			p->local_pot_y = tmpy - 13 + 1;
		}

		if (y < 20)
		{
			p->side = 1;
			p->local_pot_x = table_cols_ / 2 - 1 - x;
		}
		else
		{
			p->side = 0;
			p->local_pot_x = x;
		}
		
		std::cout << i<<", label: " << p->label << " stop: " << p->robot_stop << " side: " << p->side << " localx: " << p->local_pot_x << " localy: " << p->local_pot_y <<" p->pot_day_th2: "<< std::endl;
	}

	return 0;
}

void VisionArmCombo::EigenMatrix4dToCVMat4d(Eigen::Matrix4d & eigen_mat, cv::Mat & cv_mat)
{
	cv_mat.create(4, 4, CV_64F);
	for (int y = 0; y < 4; y++) {
		for (int x = 0; x < 4; x++) {
			cv_mat.at<double>(y, x) = eigen_mat(y, x);
		}
	}
}

void VisionArmCombo::pause_operation_if_necessary()
{
	while(pause_operation_) {
		Sleep(1000);
	}
}

int VisionArmCombo::getWater(float weight, bool b) {  //target weight
/*
	//double center_x = 0;
	//double center_y = 0;
	//double center_z = 0; 
	//double Rx = 0;
	//double Ry = 0;
	//double Rz = 0;
	//double radius = 0;
	//double x = 0; double y = 0; double angle = 0;
	//std::vector<std::vector<double>> water_config_pos;

	//for (int i = 0; i < 8; i++) {

	//	std::vector<double> water_config_pos_0 = {};

	//	x = radius*cos(angle) + center_x;
	//	y = radius*sin(angle) + center_y;
	//	water_config_pos_0.push_back(x);
	//	water_config_pos_0.push_back(y);
	//	water_config_pos_0.push_back(center_z);
	//	water_config_pos_0.push_back(Rx);
	//	water_config_pos_0.push_back(Ry);
	//	water_config_pos_0.push_back(Rz);

	//	water_config_pos.push_back(water_config_pos_0);
	//}
	  
	//while (pump->getWeight(3) < weight) {
	//	robot_arm_client_->moveHandL(water_config_pos[(count % 8)].data(), 0.05, 1);
	//	count++;
	//}
*/	///////////////////////////////////////////////

	std::vector<std::vector<double>> water_config_pos;
	int count = 0;
	float height = 0.464f;
	float deltax =  -0.0075f;
	float deltay =  -0.01f;
	bool xyz_mode = true;
	float tu = 0.008f;

	if (b) {//z=50mm water

		if (xyz_mode) {
			std::vector<double> water_config_pos_0 = { -0.080+deltax +1.2*tu, 0.312 + deltay-tu, height, 0, 0, M_PI }; // -0.080+deltax, 0.312 + deltay, height, 0, 0, M_PI
			water_config_pos.push_back(water_config_pos_0);

			std::vector<double> water_config_pos_1 = { -0.078 + deltax +2.f*tu, 0.287 + deltay+tu, height, 0, 0, M_PI }; //-0.078 + deltax, 0.287 + deltay, height, 0, 0, M_PI
			water_config_pos.push_back(water_config_pos_1);

			std::vector<double> water_config_pos_2 = { -0.059 + deltax+tu, 0.275 + deltay+tu+tu, height, 0, 0, M_PI }; //-0.059 + deltax, 0.275 + deltay, height, 0, 0, M_PI 
			water_config_pos.push_back(water_config_pos_2);

			std::vector<double> water_config_pos_3 = { -0.038 + deltax-0.6*tu, 0.277 + deltay+tu+0.8*tu, height, 0, 0, M_PI }; // -0.038 + deltax, 0.277 + deltay, height, 0, 0, M_PI
			water_config_pos.push_back(water_config_pos_3);

			std::vector<double> water_config_pos_4 = { -0.019 + deltax-2.f*tu, 0.297 + deltay+tu, height, 0, 0, M_PI }; // -0.019 + deltax, 0.297 + deltay, height, 0, 0, M_PI
			water_config_pos.push_back(water_config_pos_4);

			std::vector<double> water_config_pos_5 = { -0.026 + deltax-1.5f*tu, 0.322 + deltay-tu, height, 0, 0, M_PI }; //-0.026 + deltax, 0.322 + deltay, height, 0, 0, M_PI
			water_config_pos.push_back(water_config_pos_5);

			std::vector<double> water_config_pos_6 = { -0.042 + deltax-tu, 0.334 + deltay-1.5*tu, height, 0, 0, M_PI }; // -0.042 + deltax, 0.334 + deltay, height, 0, 0, M_PI
			water_config_pos.push_back(water_config_pos_6);

			std::vector<double> water_config_pos_7 = { -0.062 + deltax, 0.332 + deltay -1.5f*tu, height, 0, 0, M_PI }; //-0.062 + deltax, 0.332 + deltay, height, 0, 0, M_PI
			water_config_pos.push_back(water_config_pos_7);

		}
		else {
			std::vector<double> water_config_pos_0 = { 133.92, -56.6, -151.27, -62.13, 269.88, -43.97 };
			water_config_pos.push_back(water_config_pos_0);

			std::vector<double> water_config_pos_1 = { 137.44, -51.43, -152.82, -65.76, 269.89, -47.49 };
			water_config_pos.push_back(water_config_pos_1);

			std::vector<double> water_config_pos_2 = { 136.46, -47.73, -153.73, -68.55, 269.89, -46.52 };
			water_config_pos.push_back(water_config_pos_2);

			std::vector<double> water_config_pos_3 = { 132.46, -47.34, -153.82, -68.85, 269.89, -42.52 };
			water_config_pos.push_back(water_config_pos_3);

			std::vector<double> water_config_pos_4 = { 126.04, -51.5, -152.8, -65.7, 269.89, -36.02 };
			water_config_pos.push_back(water_config_pos_4);

			std::vector<double> water_config_pos_5 = { 124.2, -56.81, -151.19, -61.98, 269.89, -34.2 };
			water_config_pos.push_back(water_config_pos_5);

			std::vector<double> water_config_pos_6 = { 125.43, -59.41, -150.28, -60.3, 269.88, -35.45 };
			water_config_pos.push_back(water_config_pos_6);
 
			std::vector<double> water_config_pos_7 = { 128.69, -59.61, -150.20, -60.18, 269.88, -38.69 };
			water_config_pos.push_back(water_config_pos_7);
		}	

	}
	else { //z=60 mm
		cout << "get water from pcz tank\n";
		deltax = 0.012f;
		deltay = 0.01f;
		if (xyz_mode) {
				std::vector<double> water_config_pos_7 = { 0.003 + deltax, 0.383+deltay, 0.464, 0, 0, M_PI };  //111.58, -71.38, -147.42, -51.12, 270.02, -21.44
				water_config_pos.push_back(water_config_pos_7);

				std::vector<double> water_config_pos_0 = { 0.011 + deltax, 0.370 + deltay, 0.464, 0, 0, M_PI }; //112.55, -69.44, -148.48, -52.01, 270.03, -22.41
				water_config_pos.push_back(water_config_pos_0);

				std::vector<double> water_config_pos_1 = { 0.024 + deltax, 0.365+ deltay, 0.464, 0, 0, M_PI }; //113.00, -66.40, -149.98, -53.54, 270.04, -22.86 //////
				water_config_pos.push_back(water_config_pos_1);

				std::vector<double> water_config_pos_2 = { 0.036 + deltax, 0.371 + deltay, 0.464, 0, 0, M_PI }; //110.22, -65.26, -150.50, -54.71, 270.04, -20.08 
				water_config_pos.push_back(water_config_pos_2);

				std::vector<double> water_config_pos_3 = { 0.040 + deltax, 0.381 + deltay, 0.464, 0, 0, M_PI }; //106.44, -67.27, -149.56, -53.11, 270.05, -16.31
				water_config_pos.push_back(water_config_pos_3);

				//std::vector<double> water_config_pos_4 = { 0.043 + deltax, 0.377 + deltay, 0.464, 0, 0, M_PI }; //104.75, -69.99, -148.17, -51.77, 270.04, -14.62 
				//water_config_pos.push_back(water_config_pos_4);

				std::vector<double> water_config_pos_4 = { 0.033 + deltax, 0.394 + deltay, 0.464, 0, 0, M_PI };  //105.57, -72.52, -146.76, -50.66, 270.04, -15.43 
				water_config_pos.push_back(water_config_pos_4);

				std::vector<double> water_config_pos_5 = { 0.017 + deltax, 0.394 + deltay, 0.464, 0, 0, M_PI };  //108.17, -73.30, -146.31, -50.33, 270.04, -18.04
				water_config_pos.push_back(water_config_pos_5);

				std::vector<double> water_config_pos_6 = { 0.008 + deltax, 0.389 + deltay, 0.464, 0, 0, M_PI }; //104.75, -69.99, -148.17, -51.77, 270.04, -14.62 
				water_config_pos.push_back(water_config_pos_6);

		}


	}

	if (!xyz_mode) {
		for (int i = 0; i < water_config_pos.size(); i++) {

			std::vector<double> water_pos_temp = water_config_pos[i];

			for (int j = 0; j < water_pos_temp.size(); j++)
				water_pos_temp[j] = water_pos_temp[j] / 180.*M_PI;

			water_config_pos[i] = water_pos_temp;
		}
	}



	/////////////////////////  for test
	//while (true) {

	//	if (count == 0) {
	//		robot_arm_client_->moveHandL(water_config_pos[(count % 8)].data(), move_arm_acceleration_water_, move_arm_speed_water_);
	//		//if (robot_arm_client_->waitTillHandReachDstPose(water_config_pos[(count % 8)].data()) == -1)
	//			//std::cerr << "get wait pose timeout" << std::endl;

	//		//robot_arm_client_->moveHandJ(water_config_pos[(count % 8)].data(), move_joint_speed_, move_joint_acceleration_, true);

	//		
	//	}	
	//	else
	//		robot_arm_client_->moveHandL(water_config_pos[(count % 8)].data(), move_arm_acceleration_water_, move_arm_speed_water_);
	//		//robot_arm_client_->moveHandJ(water_config_pos[(count % 8)].data(), move_joint_speed_, move_joint_acceleration_, false);
	//	Sleep(800);
	//	count++;
	//}
	//////////////////////////////////////////
	int dig_id, ana_id, switch_id;
	float weight_th, ana_th;
	int sleeptime;
	if (b) { //water
		dig_id = 7; ana_id = 0; weight_th = 0.7f; ana_th = 0.2; //weight_th = 0.6f
		switch_id = 5;//off-forward on-backward
		sleeptime = 500;
	}
	else {  //additive
		dig_id = 6; ana_id = 1; weight_th = 0.75f; ana_th = 0.2; //weight_th = 0.6f
		switch_id = 4;//on forward off-backward
		sleeptime = 1000;
	}

	robot_arm_client_->setDigitalOutput(dig_id, false); //brake
	Sleep(1000);

	float cur_water_weight = pump->getWeight(3);

	float weight_diff = weight - cur_water_weight;

	if (weight_diff < 0.5f)
		return 1;

	//run
	cout << "watering in circle\n";

	if (b) {
		robot_arm_client_->setDigitalOutput(switch_id, false);
	}
	else {
		robot_arm_client_->setDigitalOutput(switch_id, true);
	}
	Sleep(100);
	robot_arm_client_->setAnalogOutput(ana_id, ana_th); Sleep(200);
	robot_arm_client_->setDigitalOutput(dig_id, true); Sleep(200);
	robot_arm_client_->setDigitalOutput(dig_id, true);

	

	while (weight_diff > weight_th /* weight_th*/) {

		if(xyz_mode)
			robot_arm_client_->moveHandL(water_config_pos[(count % 8)].data(), move_arm_acceleration_water_, move_arm_speed_water_);
		else
			robot_arm_client_->moveHandJ(water_config_pos[(count % 8)].data(), move_joint_speed_, move_joint_acceleration_, true);
		if (count == 0)
			Sleep(500);
		//for (auto &i : water_config_pos[(count % 8)].data())
		//cout << water_config_pos[(count % 8)].data()[0] <<" "<< water_config_pos[(count % 8)].data()[1] << endl;

			//robot_arm_client_->moveHandL(water_config_pos[(count % 8)].data(), 0.05, 1);
		
		Sleep(800);   //800
		//std::getchar();
		count++;

		cur_water_weight = pump->getWeight(3);

		weight_diff = weight - cur_water_weight;

		if (weight_diff < 3.f) {  //original 3.f

			robot_arm_client_->setAnalogOutput(ana_id, 0.1);//robot_arm_client_->setAnalogOutput(ana_id, ana_th*(weight_diff / 8.f));

			//std::cerr << "ana_th: " << ana_th*(weight_diff / 3.f) << std::endl;
		}

		//std::cerr << "water: " << pump->getWeight(2) << std::endl;
	}

	robot_arm_client_->setDigitalOutput(dig_id, false); //brake
	Sleep(1000);

	//std::cerr << "water final:  " << pump->getWeight(2) << std::endl;

	//feedback
	while (pump->getWeight(3) - weight < -0.05) {   //-0.1
		robot_arm_client_->setAnalogOutput(ana_id, 0); Sleep(100);
		robot_arm_client_->setDigitalOutput(dig_id, true);
		Sleep(500);
		robot_arm_client_->setDigitalOutput(dig_id, false); //brake
		Sleep(500);
		std::cerr << "The weight is  " << pump->getWeight(3) << std::endl;
	}

	//std::cout << "final weight:  " << pump->getWeight(2) << std::endl;

	//backward start
	cerr << "starting backward!" << endl;
	if (b) {
		robot_arm_client_->setDigitalOutput(switch_id, true);
	}
	else {
		robot_arm_client_->setDigitalOutput(switch_id, false);
	}
	Sleep(100);
	robot_arm_client_->setAnalogOutput(ana_id, 0.3); Sleep(100);//////////////
	robot_arm_client_->setDigitalOutput(dig_id, true);
	Sleep(1500 - sleeptime); //if true sleep 1000

	robot_arm_client_->setDigitalOutput(dig_id, false);
	Sleep(100);
	robot_arm_client_->setAnalogOutput(ana_id, 0.f);

	gripper_.open(0xF900);
	gripper_.close(0xF900);
	gripper_.open(0xF900);

	std::cerr << "final weight:  " << pump->getWeight(3) << std::endl;

}


void VisionArmCombo::openFileDialog() {

	wchar_t filename[MAX_PATH];

	OPENFILENAME ofn;
	ZeroMemory(&filename, sizeof(filename));
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;  // If you have a window to center over, put its HANDLE here
	ofn.lpstrFilter = L"Text Files\0*.txt\0Any File\0*.*\0";
	ofn.lpstrFile = filename;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrTitle = L"Select a File, yo!";
	ofn.Flags = OFN_DONTADDTORECENT | OFN_FILEMUSTEXIST;

	if (GetOpenFileName(&ofn))
	{
		std::cout << "You chose the file \"" << filename << "\"\n";
	}
	else
	{
		// All this stuff below is to tell you exactly how you messed up above. 
		// Once you've got that fixed, you can often (not always!) reduce it to a 'user cancelled' assumption.
		/*
		switch (CommDlgExtendedError())
		{
		case CDERR_DIALOGFAILURE: std::cout << "CDERR_DIALOGFAILURE\n";   break;
		case CDERR_FINDRESFAILURE: std::cout << "CDERR_FINDRESFAILURE\n";  break;
		case CDERR_INITIALIZATION: std::cout << "CDERR_INITIALIZATION\n";  break;
		case CDERR_LOADRESFAILURE: std::cout << "CDERR_LOADRESFAILURE\n";  break;
		case CDERR_LOADSTRFAILURE: std::cout << "CDERR_LOADSTRFAILURE\n";  break;
		case CDERR_LOCKRESFAILURE: std::cout << "CDERR_LOCKRESFAILURE\n";  break;
		case CDERR_MEMALLOCFAILURE: std::cout << "CDERR_MEMALLOCFAILURE\n"; break;
		case CDERR_MEMLOCKFAILURE: std::cout << "CDERR_MEMLOCKFAILURE\n";  break;
		case CDERR_NOHINSTANCE: std::cout << "CDERR_NOHINSTANCE\n";     break;
		case CDERR_NOHOOK: std::cout << "CDERR_NOHOOK\n";          break;
		case CDERR_NOTEMPLATE: std::cout << "CDERR_NOTEMPLATE\n";      break;
		case CDERR_STRUCTSIZE: std::cout << "CDERR_STRUCTSIZE\n";      break;
		case FNERR_BUFFERTOOSMALL: std::cout << "FNERR_BUFFERTOOSMALL\n";  break;
		case FNERR_INVALIDFILENAME: std::cout << "FNERR_INVALIDFILENAME\n"; break;
		case FNERR_SUBCLASSFAILURE: std::cout << "FNERR_SUBCLASSFAILURE\n"; break;
		default: std::cout << "You cancelled.\n";
		}*/
	}
}

void VisionArmCombo::resetCameras() {

	exo_rgb_cam_->stop();
	Sleep(100);
	exo_rgb_cam_->init();
	Sleep(100);

	std::cerr << "reset RGB camera done\n";

	/*
	line_profiler_->stop();
	Sleep(100);
	line_profiler_->finalize();
	Sleep(100);
	line_profiler_->init();
	Sleep(100);

	std:cerr << "reset laser scanner done\n";*/

}


/*
void VisionArmCombo::mainControlLoop()
{
	while (active_) {

		if (place_pot_control_) {

			std::cerr << "hello from place pot\n";

			placePots(PLACE_POT);

			place_pot_control_ = false;

		}

		if (pick_pot_control_) {

			std::cerr << "hello from pick pot\n";

			placePots(PICK_POT);

			pick_pot_control_ = false;

		}

		if (just_scan_control_) {

			std::cerr << "hello from pick pot\n";

			justScan();

			just_scan_control_ = false;

		}

		if (open_file_dialog) {
		
			openFileDialog();

			open_file_dialog = false;
		}

		Sleep(50);
	}
}

*/