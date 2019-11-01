#include "camera.h"

Camera::Camera() {}
Camera::Camera(string camera_name)
{
	this->name = camera_name;
	string camera_setting_file_path("../camera/" + camera_name + "/setting.xml");
	PRINT << camera_setting_file_path << endl;
	ifstream camera_setting_stream;
	camera_setting_stream.open(camera_setting_file_path.c_str());
	if (!camera_setting_stream)
	{
		PRINT << "no setting..." << endl;
		getCameraSetting(camera_name);

	}
	else
	{
		PRINT << "find setting..." << endl;
	}
	camera_setting_stream.close();
	loatMat();
	getMapMatrix();
}

void Camera::loatMat()
{
	cv::FileStorage load("../camera/" + this->name + "/setting.xml", cv::FileStorage::READ);
	load["camera_mat_L"] >> this->CML;
	load["camera_mat_R"] >> this->CMR;
	load["dist_coefficients_mat_L"] >> this->DML;
	load["dist_coefficients_mat_R"] >> this->DMR;
	load["R"] >> this->R;
	load["T"] >> this->T;
	load["R1"] >> this->R1;
	load["R2"] >> this->R2;
	load["T1"] >> this->T1;
	load["T2"] >> this->T2;
	load["E"] >> this->E;
	load["F"] >> this->F;
	load["Q"] >> this->Q;
	load["image_size"] >> this->image_size;
}


void Camera::getCameraSetting(string camera_name)
{
	string chessboard_image_dir_path("../chessboard_image/" + camera_name + "/");
	ifstream chessboard_image_list;
	chessboard_image_list.open((chessboard_image_dir_path + "image_list.txt").c_str());
	assert(chessboard_image_list.is_open());
	vector<string> image_name_list;
	string image_name_buf;
	while (getline(chessboard_image_list, image_name_buf))
	{
		PRINT << "image path is:" << image_name_buf << endl;
		image_name_list.push_back(image_name_buf);
	}
	cv::Size image_size;
	cv::Size corner_size = cv::Size(CORNER_SIZE_X, CORNER_SIZE_Y);
	cv::Size sub_pixel = cv::Size(SUB_PIXEL_X, SUB_PIXEL_Y);
	vector<vector<cv::Point2f>> corner_points_list_L;
	vector<vector<cv::Point2f>> corner_points_list_R;
	vector<cv::Point2f> corner_points;
	for (int i = 0; i < image_name_list.size(); i++)
	{
		string image_full_path(chessboard_image_dir_path + image_name_list[i]);
		PRINT << "read from:" << image_full_path << endl;
		cv::Mat image_src = cv::imread(image_full_path.c_str());
		if (i == 0)
		{
			image_size.width = image_src.cols;
			image_size.height = image_src.rows;
			PRINT << "image width:" << image_size.width << " image height:" << image_size.height << endl;
		}
		if (DISPLAY)
		{
			cv::imshow("src", image_src);
			cv::waitKey(100);
		}

		if (cv::findChessboardCorners(image_src, corner_size, corner_points) == false)
		{
			PRINT << "no corners please check your corner size..." << endl;
			getchar();
			exit(1);
		}
		else
		{
			cv::Mat image_gray;
			cv::cvtColor(image_src, image_gray, CV_RGB2GRAY);
			cv::find4QuadCornerSubpix(image_gray, corner_points, sub_pixel);
			if (DISPLAY)
			{
				cv::drawChessboardCorners(image_gray, corner_size, corner_points, true);
				cv::imshow("corner show", image_gray);
				cv::waitKey(100);
			}
			if (i < image_name_list.size() / 2)
			{
				corner_points_list_L.push_back(corner_points);
			}
			else
			{
				corner_points_list_R.push_back(corner_points);
			}

		}
	}
	PRINT << "get all corners" << endl;
	cv::Mat camera_mat_L = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
	cv::Mat dist_coefficients_mat_L = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));

	cv::Mat camera_mat_R = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
	cv::Mat dist_coefficients_mat_R = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));

	vector<cv::Mat> trans_mat_L;
	vector<cv::Mat> rotate_mat_L;

	vector<cv::Mat> trans_mat_R;
	vector<cv::Mat> rotate_mat_R;
	vector<vector<cv::Point3f>> corner_points3d_list;


	for (int i = 0; i < corner_points_list_L.size(); i++)
	{
		vector<cv::Point3f> corner_points3d;
		for (int j = 0; j < corner_size.height; j++)
		{
			for (int k = 0; k < corner_size.width; k++)
			{
				cv::Point3f point;
				point.x = k * BLOCK_SIZE;
				point.y = j * BLOCK_SIZE;
				point.z = 0.0;
				corner_points3d.push_back(point);
			}
		}
		corner_points3d_list.push_back(corner_points3d);
	}

	double error_L = cv::calibrateCamera(corner_points3d_list, corner_points_list_L,
		image_size, camera_mat_L, dist_coefficients_mat_L, rotate_mat_L, trans_mat_L);
	PRINT << "left camera error:" << error_L << endl;

	double error_R = cv::calibrateCamera(corner_points3d_list, corner_points_list_R,
		image_size, camera_mat_R, dist_coefficients_mat_R, rotate_mat_R, trans_mat_R);
	PRINT << "right camera error:" << error_R << endl;

	cv::Mat R, T, E, F;

	double error = cv::stereoCalibrate(corner_points3d_list,
		corner_points_list_L, corner_points_list_R,
		camera_mat_L, dist_coefficients_mat_L,
		camera_mat_R, dist_coefficients_mat_R,
		image_size,
		R, T, E, F,
		CV_CALIB_FIX_INTRINSIC
	);
	PRINT << "error:" << error << endl;
	cv::Mat R1, R2, T1, T2, Q;
	cv::stereoRectify(
		camera_mat_L, dist_coefficients_mat_L,
		camera_mat_R, dist_coefficients_mat_R,
		image_size, R, T,
		R1, R2, T1, T2, Q
	);
	string setting_path("../camera/" + camera_name + "/");
	cv::FileStorage save((setting_path + "setting.xml").c_str(), cv::FileStorage::WRITE);
	save << "camera_mat_L" << camera_mat_L;
	save << "camera_mat_R" << camera_mat_R;
	save << "dist_coefficients_mat_L" << dist_coefficients_mat_L;
	save << "dist_coefficients_mat_R" << dist_coefficients_mat_R;
	save << "R" << R;
	save << "T" << T;
	save << "E" << E;
	save << "F" << F;
	save << "R1" << R1;
	save << "R2" << R2;
	save << "T1" << T1;
	save << "T2" << T2;
	save << "Q" << Q;
	save << "image_size" << image_size;
	PRINT << "save setting..." << endl;
}

void Camera::getMapMatrix()
{
	cv::initUndistortRectifyMap(
		this->CML, this->DML, this->R1, this->T1, this->image_size, CV_16SC2, this->map_x_L, this->map_y_L
	);
	cv::initUndistortRectifyMap(
		this->CMR, this->DMR, this->R2, this->T2, this->image_size, CV_16SC2, this->map_x_R, this->map_y_R
	);
	PRINT << "get map matrix" << endl;
}
void Camera::printInformation()
{
	cout << "information..." << endl;
	cout << "camera matrix L:" << this->CML << endl;
	cout << "camera matrix R:" << this->CMR << endl;
	cout << "camera distcoefficients matrix L:" << this->DML << endl;
	cout << "camera distcoefficients matrix R:" << this->DMR << endl;
	cout << "rotate matrix L to R:" << this->R << endl;
	cout << "translate matrix L to R:" << this->T << endl;
	cout << "rotate matrix L:" << this->R1 << endl;
	cout << "rotate matrix R:" << this->R2 << endl;
	cout << "project matrix L:" << this->T1 << endl;
	cout << "project matrix R:" << this->T2 << endl;
	cout << "E matrix:" << this->E << endl;
	cout << "F matrix:" << this->F << endl;
	cout << "Q matrix:" << this->Q << endl;
	cout << "image width:" << this->image_size.width << " image height:" << this->image_size.height << endl;
	cout << "over..." << endl;
}


void Camera::getDisparityMap(string input_image_dir)
{
	string dir_path = string("../input_image/") + input_image_dir + "/";
	ifstream image_stream;
	image_stream.open((dir_path + "image_list.txt").c_str());
	assert(image_stream.is_open());
	vector<string> image_list;
	string image_name;
	while (getline(image_stream, image_name))
	{
		image_list.push_back(image_name);
		PRINT <<"read image:"<< image_name << endl;
	}
	int sample_num = image_list.size()/2;
	int mindisparity = 0;
	int ndisparities = 64;
	int SADWindowSize = 11;

	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);

	sgbm->setPreFilterCap(15);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleRange(2);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setDisp12MaxDiff(1);

	for (int i = 0; i < image_list.size() / 2; i++)
	{
		cout << "image: " << i << "..." << endl;
		int l_image_ind = i;
		int r_image_ind = i + sample_num;
		cv::Mat image_scr_L = cv::imread((dir_path + image_list[l_image_ind]).c_str());
		cv::Mat image_scr_R = cv::imread((dir_path + image_list[r_image_ind]).c_str());
		cv::Mat image_res_L;
		cv::Mat image_res_R;
		cv::remap(image_scr_L, image_res_L, this->map_x_L, this->map_y_L, cv::INTER_LINEAR);
		cv::remap(image_scr_R, image_res_R, this->map_x_R, this->map_y_R, cv::INTER_LINEAR);
		//cv::imshow("l", image_res_L);
		//cv::waitKey(100);
		//cv::imshow("r", image_res_R);
		//cv::waitKey(100);
		cv::Mat disp;

		if (i == 0)
		{
			int P1 = 8 * image_res_L.channels() * SADWindowSize* SADWindowSize;
			int P2 = 32 * image_res_R.channels() * SADWindowSize* SADWindowSize;
			sgbm->setP1(P1);
			sgbm->setP2(P2);
		}
		sgbm->compute(image_res_L, image_res_R, disp);
		disp.convertTo(disp, CV_32F, 1.0 / 16);               
		cv::Mat disp8U = cv::Mat(disp.rows, disp.cols, CV_8UC1);      
		cv::normalize(disp, disp8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
		imwrite("../output_image/"+this->name+"/"+to_string(i)+".jpg", disp8U);
		cout << "over" << endl;
	}
}