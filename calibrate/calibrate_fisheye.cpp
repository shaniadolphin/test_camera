/****************
cmake_minimum_required(VERSION 2.8)
project(test)
find_package(OpenCV REQUIRED)
add_executable(test capture.cpp)
target_link_libraries(test ${OpenCV_LIBS})
#./capture -d /dev/video0 -w 1920
***************/
//calibration.cpp
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

vector<cv::String> file_vec;

#define calibration

void getFiles()
{
	//目标文件夹路径
	string src_path = "./images/";//遍历文件夹下的所有.jpg文件
	//std::string src_path = "E:\\ src_imgs\\";
	glob(src_path+"*.jpg", file_vec, false);
	for (string filename : file_vec)
	{
		cout << filename << std::endl;
	}
}

int main()
{
#ifdef calibration
	ofstream fout("caliberation_result.txt");  /**    保存定标结果的文件     **/
	getFiles();									//遍历文件夹下的所有.jpg文件
	/************************************************************************
	读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
	*************************************************************************/
	cout << "开始提取角点………………" << endl;
	int image_count = 0;                    /****    图像数量     ****/
	Size board_size = Size(11,8);            /****    定标板上每行、列的角点数       ****/
	vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
	vector<vector<Point2f>>  corners_Seq;    /****  保存检测到的所有角点       ****/
	vector<Mat>  image_Seq;
	int successImageNum = 0;                /****   成功提取角点的棋盘图数量    ****/
	vector<string> filenames;
	int count = 0;
	
	//for (int i = 0; i != image_count; i++)
	for (string imageFileName : file_vec)
	{
		++image_count;
		cout << "从图片 " << imageFileName << "中查找角点..." << endl;
		std::stringstream StrStm;
		//StrStm << image_count;
		//StrStm >> imageFileName;
		//imageFileName += ".jpg";
		cv::Mat image = imread(imageFileName);
		/* 提取角点 */
		Mat imageGray;
		cvtColor(image, imageGray, CV_RGB2GRAY);
		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
		CALIB_CB_FAST_CHECK);
		if (!patternfound)
		{
			cout << "找不到角点," << imageFileName << "不正确！" << endl;
			exit(1);
		}
		else
		{
			/* 亚像素精确化 */
			cornerSubPix(imageGray, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			#if 0
			/* 绘制检测到的角点并保存 */
			Mat imageTemp = imageGray.clone();//image.clone();
			for (int j = 0; j < corners.size(); j++)
			{
				circle(imageTemp, corners[j], 10, Scalar(0, 0, 255), 2, 8, 0);
			}
			string cornerFileName;
			std::stringstream StrStm;
			StrStm << image_count;
			StrStm >> cornerFileName;
			cornerFileName += "_corner.jpg";
			//imwrite(cornerFileName, imageTemp);
			#endif
			cout << "从图片 " << imageFileName << "中找到角点：" << corners.size() << endl;
			count = count + corners.size();
			successImageNum = successImageNum + 1;
			corners_Seq.push_back(corners);
		}
		image_Seq.push_back(image);
	}
	cout << "角点提取完成！\n";
	/************************************************************************
	摄像机标定
	*************************************************************************/
	cout << "开始标定………………" << endl;
	Size square_size = Size(10, 10);
	vector<vector<Point3f>>  object_Points;        /****  保存定标板上角点的三维坐标   ****/

	Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));  /*****   保存提取的所有角点   *****/
	vector<int>  point_counts;
	/* 初始化定标板上角点的三维坐标 */
	for (int t = 0; t<successImageNum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i<board_size.height; i++)
		{
			for (int j = 0; j<board_size.width; j++)
			{
				/* 假设定标板放在世界坐标系中z=0的平面上 */
				Point3f tempPoint;
				tempPoint.x = i*square_size.width;
				tempPoint.y = j*square_size.height;
				tempPoint.z = 0;
				tempPointSet.push_back(tempPoint);
			}
		}
		object_Points.push_back(tempPointSet);
	}
	for (int i = 0; i< successImageNum; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
	/* 开始标定 */
	Size image_size = image_Seq[0].size();
	cv::Matx33d intrinsic_matrix;    /*****    摄像机内参数矩阵    ****/
	cv::Vec4d distortion_coeffs;     /*****    摄像机的4个畸变系数：k1,k2,k3,k4****/
	std::vector<cv::Vec3d> rotation_vectors;                           /* 每幅图像的旋转向量 */
	std::vector<cv::Vec3d> translation_vectors;                        /* 每幅图像的平移向量 */
	int flags = 0;
	flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	flags |= cv::fisheye::CALIB_CHECK_COND;
	flags |= cv::fisheye::CALIB_FIX_SKEW;
	fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
	cout << "标定完成！\n";
	cout << "生成MAP数据！\n";
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat transformation = Mat::eye(3, 3, CV_32F);
	Mat newcameramtx = Mat::eye(3, 3, CV_32F);
	double balance = 0.5;
	fisheye::estimateNewCameraMatrixForUndistortRectify(intrinsic_matrix, distortion_coeffs,image_size,transformation, newcameramtx,balance,image_size,0);
	//fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, transformation,
	//getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, balance, image_size, 0), image_size, CV_32FC1, mapx, mapy);
	fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, transformation, newcameramtx, image_size, CV_32FC1, mapx, mapy);

	cout << "开始评价标定结果………………" << endl;
	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
	double err = 0.0;                        /* 每幅图像的平均误差 */
	vector<Point2f>  image_points2;             /* 保存重新计算得到的投影点 */

#if 1
	cv::Matx33d matrix(
    866.9125298, 0.0,  858.21629628,
    0.0, 866.83387906, 680.02697262,
    0.0, 0.0, 1.0);
	cv::Vec4d coeffs(
	0.07503968, -0.01039679,-0.00777338,0.00320349);
	
	intrinsic_matrix = matrix;
	distortion_coeffs = coeffs;

#endif

	cout << "每幅图像的定标误差：" << endl;
	for (int i = 0; i<image_count; i++)
	{
		vector<Point3f> tempPointSet = object_Points[i];
		/*通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点*/
		fisheye::projectPoints(tempPointSet, image_points2, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs);
		/*计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = corners_Seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (size_t i = 0; i != tempImagePoint.size(); i++)
		{
			image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
			tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		//fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	//fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
	cout << "评价完成！" << endl;

	/************************************************************************
	保存定标结果
	*************************************************************************/
	cout << "保存定标结果......" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	fout << "\n intrinsic_matrix:\n" << endl;
	fout << intrinsic_matrix << endl;
	fout << "\n distortion_coeffs:\n";
	fout << distortion_coeffs << endl;
	fout << "\n newcameramtx:\n";
	fout << newcameramtx << endl;
	#if 0
	for (int i = 0; i<image_count; i++)
	{
		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		fout << rotation_vectors[i] << endl;
		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(rotation_vectors[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << translation_vectors[i] << endl;
	}
	#endif
	cout << "完成保存" << endl;
	fout << endl;
	/************************************************************************
	显示标定结果
	*************************************************************************/
	image_count = 0;
	for (string imageFileName : file_vec)
	{
		cout << "将图片 " << imageFileName << " 进行反畸变..." << endl;

		Mat t = image_Seq[image_count].clone();
		cv::remap(image_Seq[image_count], t, mapx, mapy, INTER_LINEAR);
		string imageSaveName;
		std::stringstream StrStm;
		StrStm << image_count;
		StrStm >> imageSaveName;
		imageSaveName += "_d.jpg";
		imwrite(imageSaveName, t);
		++image_count;
	}
	cout << "保存结束" << endl;

#else 
		//读取一副图片，不改变图片本身的颜色类型（该读取方式为DOS运行模式）
		Mat src = imread("F:\\lane_line_detection\\left_img\\1.jpg");
		Mat distortion = src.clone();
		Mat camera_matrix = Mat(3, 3, CV_32FC1);
		Mat distortion_coefficients;


		//导入相机内参和畸变系数矩阵
		FileStorage file_storage("F:\\lane_line_detection\\left_img\\Intrinsic.xml", FileStorage::READ);
		file_storage["CameraMatrix"] >> camera_matrix;
		file_storage["Dist"] >> distortion_coefficients;
		file_storage.release();

		//矫正
		cv::undistort(src, distortion, camera_matrix, distortion_coefficients);

		//cv::imshow("img", src);
		//cv::imshow("undistort", distortion);
		cv::imwrite("undistort.jpg", distortion);

		cv::waitKey(0);
#endif // DEBUG
	return 0;
}
