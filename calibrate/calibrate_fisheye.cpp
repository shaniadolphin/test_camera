/****************
cmake_minimum_required(VERSION 2.8)
project(test)
find_package(OpenCV REQUIRED)
add_executable(test capture.cpp)
target_link_libraries(test ${OpenCV_LIBS})

g++ -g -Wall -Wl,-rpath=./lib -I/usr/include/ -L./lib   -c -o calibrate_fisheye.o calibrate_fisheye.cpp

#./capture -d /dev/video0 -w 1920
***************/
//calibration.cpp
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <cmath>

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
	glob(src_path+"*.jpg", file_vec, false);
	for (string filename : file_vec)
	{
		cout << filename << std::endl;
	}
}

// Calculates rotation matrix given euler angles.
Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
	// Calculate rotation about x axis
	Mat R_x = (Mat_<double>(3,3) <<
		1,       0,              0,
		0,       cos(theta[0]),   -sin(theta[0]),
		0,       sin(theta[0]),   cos(theta[0])
		);
	// Calculate rotation about y axis
	Mat R_y = (Mat_<double>(3,3) <<
		cos(theta[1]),    0,      sin(theta[1]),
		0,               1,      0,
		-sin(theta[1]),   0,      cos(theta[1])
		);
	// Calculate rotation about z axis
	Mat R_z = (Mat_<double>(3,3) <<
		cos(theta[2]),    -sin(theta[2]),      0,
		sin(theta[2]),    cos(theta[2]),       0,
		0,               0,                  1);
	// Combined rotation matrix
	Mat R = R_z * R_y * R_x;
	return R;
}
// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R, bool flag)
{
	//assert(isRotationMatrix(R));
	float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
		y = atan2(-R.at<double>(2,0), sy);
		z = atan2(R.at<double>(1,0), R.at<double>(0,0));
	}
	else
	{
		x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
		y = atan2(-R.at<double>(2,0), sy);
		z = 0;
	}
	if(flag)
	{
		x = x*180.0f/3.141592653589793f;
		y = y*180.0f/3.141592653589793f;
		z = z*180.0f/3.141592653589793f;
	}
	//endif
	return Vec3f(x, y, z);
}

Vec2f myUndistortPoints(InputArray K, InputArray D, InputArray R, InputArray P, double x_, double y_)
{
	//从内参矩阵K中取出归一化焦距fx,fy; cx,cy
	cv::Vec2d f, c;
	if (K.depth() == CV_32F)
	{
		Matx33f camMat = K.getMat();
		f = Vec2f(camMat(0, 0), camMat(1, 1));
		c = Vec2f(camMat(0, 2), camMat(1, 2));
	}
	else
	{
		Matx33d camMat = K.getMat();
		f = Vec2d(camMat(0, 0), camMat(1, 1));
		c = Vec2d(camMat(0, 2), camMat(1, 2));
	}
	//从畸变系数矩阵D中取出畸变系数k1,k2,k3,k4
	Vec4d k = Vec4d::all(0);
	if (!D.empty())
		k = D.depth() == CV_32F ? (Vec4d)*D.getMat().ptr<Vec4f>(): *D.getMat().ptr<Vec4d>();

	//旋转矩阵RR转换数据类型为CV_64F，如果不需要旋转，则RR为单位阵
	cv::Matx33d RR  = cv::Matx33d::eye();
	if (!R.empty() && R.size() == Size(3, 3))
		R.getMat().convertTo(RR, CV_64F);//R.size() == Size(3, 3)
	
	//新的内参矩阵PP转换数据类型为CV_64F
	cv::Matx33d PP = cv::Matx33d::eye();
	if (!P.empty())
		P.getMat().colRange(0, 3).convertTo(PP, CV_64F);
	
	//新的内参矩阵*旋转矩阵，利用SVD分解求出逆矩阵iR
	cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);
	
	double i = y_;
	double j = x_;
	//二维图像平面坐标系->摄像机坐标系
	double _x = i * iR(0, 1) + j * iR(0, 0) + iR(0, 2);
	double _y = i * iR(1, 1) + j * iR(1, 0) + iR(1, 2);
	double _w = i * iR(2, 1) + j * iR(2, 0) + iR(2, 2);
	//归一化摄像机坐标系，相当于假定在Z=1平面上
	double x = _x/_w, y = _y/_w;
	//求鱼眼半球体截面半径r
	double r = sqrt(x*x + y*y);
	//求鱼眼半球面上一点与光心的连线和光轴的夹角Theta
	double theta = atan(r);
	//畸变模型求出theta_d，相当于有畸变的角度值
	double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
	double theta_d = theta * (1 + k[0]*theta2 + k[1]*theta4 + k[2]*theta6 + k[3]*theta8);
	//利用有畸变的Theta值，将摄像机坐标系下的归一化三维坐标，重投影到二维图像平面，得到(j,i)对应畸变图像中的(u,v)
	double scale = (r == 0) ? 1.0 : theta_d / r;
	double u = f[0]*x*scale + c[0];
	double v = f[1]*y*scale + c[1];
	float dst_x= 0;
	float dst_y= 0;
	//保存(u,v)坐标到mapx,mapy
	//m1f[j] = (float)u;
	//m2f[j] = (float)v;
	dst_x = (float)u;
	dst_y = (float)v;
	
	return Vec2f(dst_x, dst_y);
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
		
		cout << "从图片 " << imageFileName << "中查找角点..." << endl;
		std::stringstream StrStm;
		//StrStm << image_count;
		//StrStm >> imageFileName;
		//imageFileName += ".jpg";
		cv::Mat image = imread(imageFileName);
		/* 提取角点 */
		Mat imageGray;
		cvtColor(image, imageGray, CV_RGB2GRAY);
		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (!patternfound)
		{
			cout << "找不到角点," << imageFileName << "不正确！" << endl;
			//exit(1);
			continue;
		}
		else
		{
			++image_count;
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
	if(image_count > 3)
	{
		cout << "角点提取完成！\n";
	}
	else
	{
		cout << "图片不够！\n";
		exit(1);
	}
	/************************************************************************
	摄像机标定
	*************************************************************************/
	cout << "开始标定………………" << endl;
	Size square_size = Size(30, 30);
	vector<vector<Point3f>>  object_Points;        /****  保存定标板上角点的三维坐标   ****/

	Mat image_points = Mat(1, count, CV_32FC2, Scalar::all(0));  /*****   保存提取的所有角点   *****/
	vector<int>  point_counts;
	/* 初始化定标板上角点的三维坐标 */
	for (int t = 0; t<successImageNum; t++)
	{
		vector<Point3f> tempPointSet;
		for (int j = 0; j<board_size.height; j++)
		{
			for (int i = 0; i<board_size.width; i++)
			{
				/* 假设定标板放在世界坐标系中z=0的平面上 */
				Point3f tempPoint;
				//tempPoint.x = i*square_size.width;
				//tempPoint.y = j*square_size.height;
				tempPoint.x = i*square_size.height;
				tempPoint.y = j*square_size.width;
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
#if 0
	cv::Matx33d matrix(
    866.9125298, 0.0,  858.21629628,
    0.0, 866.83387906, 680.02697262,
    0.0, 0.0, 1.0);
	cv::Vec4d coeffs(
	0.07503968, -0.01039679,-0.00777338,0.00320349);
	
	intrinsic_matrix = matrix;
	distortion_coeffs = coeffs;

#endif

	cv::Vec4d coeffs(0.0, 0.0, 0.0, 0.0);
	double balance = 0.5;
	fisheye::estimateNewCameraMatrixForUndistortRectify(intrinsic_matrix, distortion_coeffs,image_size,transformation, newcameramtx,balance,image_size,0);
	fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, transformation, newcameramtx, image_size, CV_32FC1, mapx, mapy);

	cout << "开始评价标定结果………………" << endl;
	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
	double err = 0.0;                        /* 每幅图像的平均误差 */
	vector<Point2f>  image_points2;             /* 保存重新计算得到的投影点 */

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
			cout << "投影点-" << i << ":" << image_points2Mat.at<Vec2f>(0, i) << endl;
			cout << "实际点-" << i << ":" << tempImagePointMat.at<Vec2f>(0, i) << endl;
		}
		
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	
	double r_vec[3]={0.78520514,0.0233998,0.00969251};//eulerAngles[45,1,1]
	double R_matrix[9];
	CvMat *pr_vec = cvCreateMat(1,3,CV_64FC1);
	CvMat *pR_matrix = cvCreateMat(3,3,CV_64FC1);
	CvMat *pnew_vec = cvCreateMat(1,3,CV_64FC1);
	/************************************************************************
	保存标定结果
	*************************************************************************/
	cout << "保存标定结果......" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	Vec3f eulerAngles;
	fout << "\n intrinsic_matrix:\n" << endl;
	fout << intrinsic_matrix << endl;
	fout << "\n distortion_coeffs:\n";
	fout << distortion_coeffs << endl;
	fout << "\n newcameramtx:\n";
	fout << newcameramtx << endl;
	cout << "\n intrinsic_matrix:\n" << endl;
	cout << intrinsic_matrix << endl;
	cout << "\n distortion_coeffs:\n";
	cout << distortion_coeffs << endl;
	cout << "\n newcameramtx:\n";
	cout << newcameramtx << endl;
	for (int i = 0; i<image_count; i++)
	{
	#if 1 //两种求欧拉角的方法，1：使用cvRodrigues2
		r_vec[0]=(double)(rotation_vectors[i](0));
		r_vec[1]=(double)(rotation_vectors[i](1));
		r_vec[2]=(double)(rotation_vectors[i](2));
		
		cvInitMatHeader(pr_vec,1,3,CV_64FC1,r_vec,CV_AUTOSTEP);
		cvInitMatHeader(pR_matrix,3,3,CV_64FC1,R_matrix,CV_AUTOSTEP);
		
		cvRodrigues2(pr_vec, pR_matrix, 0);//从旋转向量求旋转矩阵
		
		cvRodrigues2(pR_matrix, pnew_vec, 0);//从旋转矩阵求旋转向量
		Mat rotation_vec_tmp(pnew_vec->rows,pnew_vec->cols,pnew_vec->type,pnew_vec->data.fl);
		
		//cvMat转Mat
		Mat rotation_matrix_tmp(pR_matrix->rows,pR_matrix->cols,pR_matrix->type,pR_matrix->data.fl);
		//eulerAngles = rotationMatrixToEulerAngles(rotation_matrix_tmp, 0);
		//rotation_matrix_tmp = eulerAnglesToRotationMatrix(eulerAngles);
		eulerAngles = rotationMatrixToEulerAngles(rotation_matrix_tmp, 1);//0表示输出为弧度，否则表示输出为角度
	#else//两种求欧拉角的方法，2：使用Rodrigues
		Rodrigues(translation_vectors[i],rotation_matrix); 
		eulerAngles = rotationMatrixToEulerAngles(rotation_matrix, 1);
	#endif
	
	#if 0
		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		fout << rotation_vectors[i] << endl;
		/* 将旋转向量转换为相对应的旋转矩阵 */
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << translation_vectors[i] << endl;
	#else
		cout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		cout << "原始:" <<rotation_vectors[i] << endl;
		cout << "反算:" <<rotation_vec_tmp << endl;
		//cout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		//cout << rotation_matrix_tmp << endl;
		//cout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		//cout << translation_vectors[i] << endl;
		cout << "第" << i + 1 << "幅图像的欧拉角：" << endl;
		cout << eulerAngles << endl;
	#endif
	}
	cout << "完成保存" << endl;
	fout << endl;
	/************************************************************************
	显示标定结果
	*************************************************************************/
	image_count = 0;
	cout << "图片数量：" << endl;
	cout << image_Seq.size() << endl;
	//cout << "角点数量：" << endl;
	//cout << corners_Seq[image_count].size() << endl;
	for (unsigned int i = 0; i < image_Seq.size(); i++)
	//for (string imageFileName : file_vec)
	{
		//cout << "将图片 " << imageFileName << " 进行反畸变..." << endl;
		Mat t = image_Seq[image_count].clone();
		for (unsigned int j = 0; j < corners_Seq[image_count].size(); j++)
		{
			circle(image_Seq[image_count], corners_Seq[image_count][j], 10, Scalar(0, 0, 255), 2, 8, 0);
		}
		//circle(imageTemp, corners[j], 10, Scalar(0, 0, 255), 2, 8, 0);
		cv::remap(image_Seq[image_count], t, mapx, mapy, INTER_LINEAR);
		string imageSaveName;
		std::stringstream StrStm;
		StrStm << image_count;
		StrStm >> imageSaveName;
		imageSaveName += "_d.jpg";
		imwrite(imageSaveName, t);
		#if 0
		vector<Point2f> pts_src;//校正前图片中的矫正点的位置存放处
		vector<Point2f> pts_dst;//标准图片中的矫正点位置存放处
		pts_src.push_back(Point2f(corners_Seq[image_count][2].x, corners_Seq[image_count][2].y));
		pts_src.push_back(Point2f(corners_Seq[image_count][4].x, corners_Seq[image_count][4].y));
		pts_src.push_back(Point2f(corners_Seq[image_count][20].x, corners_Seq[image_count][20].y));
		pts_src.push_back(Point2f(corners_Seq[image_count][22].x, corners_Seq[image_count][22].y));
		pts_dst.push_back(Point2f(corners2[2].x, corners2[2].y));
		pts_dst.push_back(Point2f(corners2[4].x, corners2[4].y));
		pts_dst.push_back(Point2f(corners2[20].x, corners2[20].y));
		pts_dst.push_back(Point2f(corners2[22].x, corners2[22].y));

		#endif
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
