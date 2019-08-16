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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>/* getopt_long() */
#include <iostream>
#include <sstream>
//#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <fstream>
#include <cmath>
#include <tbb/tbb.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"

//#include "nvram/libnvram.h"
//#include "NvRAMUtils/NvRAMUtils.h"
//#include "file_op/libfile_op.h"
#include "targetver.h"

using namespace cv;
using namespace std;

vector<cv::String> file_vec;

string sub_path = "calib/";


void getFiles(string src_path)
{
	//目标文件夹路径
	//遍历文件夹下的所有.jpg文件
	glob(src_path+"*.jpg", file_vec, false);
	for (string filename : file_vec)
	{
		cout << filename << std::endl;
	}
}

// Calculates rotation matrix given euler angles.
inline Mat eulerAnglesToRotationMatrix(Vec3f &theta)
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
	return norm(I, shouldBeIdentity) < 1e-6;
}
 
// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
inline Vec3f rotationMatrixToEulerAngles(Mat &R, int flag)
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
	if(1 == flag)
	{
		x = x*180.0f/3.141592653589793f;
		y = y*180.0f/3.141592653589793f;
		z = z*180.0f/3.141592653589793f;
	}
	else if (flag > 1)
	{
		x = 0;
		y = 0;
		z = 0;
	}
	//endif
	return Vec3f(x, y, z);
}

void myDistortPoints(InputArray undistorted, OutputArray distorted, InputArray K, InputArray D)
{
	//cout << undistorted.size() << undistorted.type() <<endl;
	double alpha = 0;
	size_t n = undistorted.total();
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
		c = Vec2d(camMat(0 ,2), camMat(1, 2));
	}
	//cout << f << c <<endl;
	distorted.create(undistorted.size(), undistorted.type());
	Vec4d k = D.depth() == CV_32F ? (Vec4d)*D.getMat().ptr<Vec4f>(): *D.getMat().ptr<Vec4d>();

	const Vec2f* Xf = undistorted.getMat().ptr<Vec2f>();
	const Vec2d* Xd = undistorted.getMat().ptr<Vec2d>();
	Vec2f *xpf = distorted.getMat().ptr<Vec2f>();
	Vec2d *xpd = distorted.getMat().ptr<Vec2d>();

	for(size_t i = 0; i < n; ++i)
	{
		Vec2d x = undistorted.depth() == CV_32F ? (Vec2d)Xf[i] : Xd[i];
		//cout << "\nx = "<< x << endl;
		Vec2d newx = ((x[0]-c[0])/f[0], (x[1]-c[1])/f[1]);
		newx[0]=(x[0]-c[0])/f[0];
		newx[1]=(x[1]-c[1])/f[1];
		double r2 = newx.dot(newx);
		//double r2 = x.dot(x);
		double r = std::sqrt(r2);
		//cout << "newx = "<< newx << endl;
		// Angle of the incoming ray:
		double theta = atan(r);

		double theta2 = theta*theta, theta3 = theta2*theta, theta4 = theta2*theta2, theta5 = theta4*theta,
				theta6 = theta3*theta3, theta7 = theta6*theta, theta8 = theta4*theta4, theta9 = theta8*theta;

		double theta_d = theta + k[0]*theta3 + k[1]*theta5 + k[2]*theta7 + k[3]*theta9;
		
		double inv_r = r > 1e-8 ? 1.0/r : 1; //  =1/r
		double cdist = r > 1e-8 ? theta_d * inv_r : 1; // = theta_d/r
		//cout << "cdist = "<< cdist << endl;
		Vec2d xd1 = newx * cdist;		// = x * theta_d/r
		Vec2d xd3(xd1[0] + alpha*xd1[1], xd1[1]);
		Vec2d final_point(xd1[0] * f[0] + c[0], xd1[1] * f[1] + c[1]);

		if (undistorted.depth() == CV_32F)
			xpf[i] = final_point;
		else
			xpd[i] = final_point;
		//cout << "xd1 = "<< xd1 << endl;
		//cout << "final_point = "<< final_point << endl;
	}
}

void opencvUndistortPoints(InputArray distorted, OutputArray undistorted, InputArray K, InputArray D)
{
    undistorted.create(distorted.size(), distorted.type());
	
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

    Vec4d k = D.depth() == CV_32F ? (Vec4d)*D.getMat().ptr<Vec4f>(): *D.getMat().ptr<Vec4d>();

    cv::Matx33d RR = cv::Matx33d::eye();
	cv::Matx33d PP;
    K.getMat().colRange(0, 3).convertTo(PP, CV_64F);
	RR = PP * RR;
    // start undistorting
    const cv::Vec2f* srcf = distorted.getMat().ptr<cv::Vec2f>();
    const cv::Vec2d* srcd = distorted.getMat().ptr<cv::Vec2d>();
    cv::Vec2f* dstf = undistorted.getMat().ptr<cv::Vec2f>();
    cv::Vec2d* dstd = undistorted.getMat().ptr<cv::Vec2d>();
	
    size_t n = distorted.total();
    int sdepth = distorted.depth();

    for(size_t i = 0; i < n; i++ )
    {
        Vec2d pi = sdepth == CV_32F ? (Vec2d)srcf[i] : srcd[i];  // image point
        Vec2d pw((pi[0] - c[0])/f[0], (pi[1] - c[1])/f[1]);      // world point
		//cout << "distorted = "<< pi << endl;

        double scale = 1.0;

        double theta_d = sqrt(pw[0]*pw[0] + pw[1]*pw[1]);

        // the current camera model is only valid up to 180 FOV
        // for larger FOV the loop below does not converge
        // clip values so we still get plausible results for super fisheye images > 180 grad
        theta_d = min(max(-CV_PI/2., theta_d), CV_PI/2.);

        if (theta_d > 1e-8)
        {
            // compensate distortion iteratively
            double theta = theta_d;

            const double EPS = 1e-8; // or std::numeric_limits<double>::epsilon();
            for (int j = 0; j < 10; j++)
            {
                double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta6*theta2;
                double k0_theta2 = k[0] * theta2, k1_theta4 = k[1] * theta4, k2_theta6 = k[2] * theta6, k3_theta8 = k[3] * theta8;
                /* new_theta = theta - theta_fix, theta_fix = f0(theta) / f0'(theta) */
                double theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                   (1 + 3*k0_theta2 + 5*k1_theta4 + 7*k2_theta6 + 9*k3_theta8);
                theta = theta - theta_fix;
                if (fabs(theta_fix) < EPS)
                    break;
            }

            scale = std::tan(theta) / theta_d;
        }

        Vec2d pu = pw * scale; //undistorted point

        // reproject
        Vec3d pr = RR * Vec3d(pu[0], pu[1], 1.0); // rotated point optionally multiplied by new camera matrix
        Vec2d fi(pr[0]/pr[2], pr[1]/pr[2]);       // final

        if( sdepth == CV_32F )
            dstf[i] = fi;
        else
            dstd[i] = fi;
    }
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

void estimateNewCameraMatrix(InputArray K, InputArray D, const Size &image_size, InputArray R,
    OutputArray P, double balance, const Size& new_size, double fov_scale)
{
	CV_Assert( K.size() == Size(3, 3)       && (K.depth() == CV_32F || K.depth() == CV_64F));
	CV_Assert(D.empty() || ((D.total() == 4) && (D.depth() == CV_32F || D.depth() == CV_64F)));

	int w = image_size.width, h = image_size.height;
	balance = std::min(std::max(balance, 0.0), 1.0);

	cv::Mat points(1, 4, CV_64FC2);
	Vec2d* pptr = points.ptr<Vec2d>();
	pptr[0] = Vec2d(w/2, 0);
	pptr[1] = Vec2d(w, h/2);
	pptr[2] = Vec2d(w/2, h);
	pptr[3] = Vec2d(0, h/2);

	fisheye::undistortPoints(points, points, K, D, R);
	cv::Scalar center_mass = mean(points);
	cv::Vec2d cn(center_mass.val);

	double aspect_ratio = (K.depth() == CV_32F) ? K.getMat().at<float >(0,0)/K.getMat().at<float> (1,1)
												: K.getMat().at<double>(0,0)/K.getMat().at<double>(1,1);

	// convert to identity ratio
	cn[0] *= aspect_ratio;
	for(size_t i = 0; i < points.total(); ++i)
		pptr[i][1] *= aspect_ratio;

	double minx = DBL_MAX, miny = DBL_MAX, maxx = -DBL_MAX, maxy = -DBL_MAX;
	for(size_t i = 0; i < points.total(); ++i)
	{
		miny = std::min(miny, pptr[i][1]);
		maxy = std::max(maxy, pptr[i][1]);
		minx = std::min(minx, pptr[i][0]);
		maxx = std::max(maxx, pptr[i][0]);
	}

	double f1 = w * 0.5/(cn[0] - minx);
	double f2 = w * 0.5/(maxx - cn[0]);
	double f3 = h * 0.5 * aspect_ratio/(cn[1] - miny);
	double f4 = h * 0.5 * aspect_ratio/(maxy - cn[1]);

	double fmin = std::min(f1, std::min(f2, std::min(f3, f4)));
	double fmax = std::max(f1, std::max(f2, std::max(f3, f4)));

	double f = balance * fmin + (1.0 - balance) * fmax;
	f *= fov_scale > 0 ? 1.0/fov_scale : 1.0;

	cv::Vec2d new_f(f, f), new_c = -cn * f + Vec2d(w, h * aspect_ratio) * 0.5;

	// restore aspect ratio
	new_f[1] /= aspect_ratio;
	new_c[1] /= aspect_ratio;


		double rx = new_size.width /(double)image_size.width;
		double ry = new_size.height/(double)image_size.height;

		new_f[0] *= rx;  new_f[1] *= ry;
		new_c[0] *= rx;  new_c[1] *= ry;


	Mat(Matx33d(new_f[0], 0, new_c[0],
				0, new_f[1], new_c[1],
				0,        0,       1)).convertTo(P, P.empty() ? K.type() : P.type());
}

void cameraToWorld(InputArray cameraMatrix, InputArray rV, InputArray tV, vector<Point2f> imgPoints, vector<Point3f> &worldPoints)
{
	Mat invK64, invK;
	invK64 = cameraMatrix.getMat().inv();
	invK64.convertTo(invK, CV_32F);
	Mat r, t, rMat;
	rV.getMat().convertTo(r, CV_32F);
	tV.getMat().convertTo(t, CV_32F);
	Rodrigues(r, rMat);

	//计算 invR * T
	Mat invR = rMat.inv();
	//cout << "invR\n" << invR << endl;

	Mat transPlaneToCam;
	transPlaneToCam = invR * t.t();
	//cout << "transPlaneToCam\n" << transPlaneToCam << endl;
	//Mat opoints = imgPoints.getMat();
	//int npoints = opoints.checkVector(2);
	//int depth = opoints.depth();
	//CvMat c_objectPoints = cvMat(opoints);

	int npoints = (int)imgPoints.size();
	//cout << "npoints\n" << npoints << endl;
	for (int j = 0; j < npoints; ++j){
		Mat coords(3, 1, CV_32F);
		Point3f pt;
		coords.at<float>(0, 0) = imgPoints[j].x;
		coords.at<float>(1, 0) = imgPoints[j].y;
		coords.at<float>(2, 0) = 1.0f;
		//[x,y,z] = invK * [u,v,1]
		Mat worldPtCam = invK * coords;
		//[x,y,1] * invR
		Mat worldPtPlane = invR * worldPtCam;
		//zc 
		float scale = transPlaneToCam.at<float>(2) / worldPtPlane.at<float>(2);
		//cout << "scale\n" << scale << endl;
		Mat scale_worldPtPlane(3, 1, CV_32F);
		//scale_worldPtPlane.at<float>(0, 0) = worldPtPlane.at<float>(0, 0) * scale;
		//zc * [x,y,1] * invR
		scale_worldPtPlane = scale * worldPtPlane;
		//[X,Y,Z]=zc*[x,y,1]*invR - invR*T
		Mat worldPtPlaneReproject = scale_worldPtPlane - transPlaneToCam;
		pt.x = worldPtPlaneReproject.at<float>(0);
		pt.y = worldPtPlaneReproject.at<float>(1);
		//pt.z = worldPtPlaneReproject.at<float>(2);
		pt.z = 1.0f;
		worldPoints.push_back(pt);
	}
}

static int getCurTime(void)
{
	struct timeval tv;    
	gettimeofday(&tv,NULL);    //该函数在sys/time.h头文件中
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;       
}

int fisheye_calibrate_process(string src_path)
{
	char mk_out_dir[256];

	cout << __FUNCTION__ << " : "<< src_path << endl;

	sprintf(mk_out_dir, "mkdir -p %s%s", src_path.c_str(), sub_path.c_str());
	system(mk_out_dir);

	ofstream fout(src_path+"calibration.csv");  /**    保存定标结果的文件     **/
	getFiles(src_path);							//遍历文件夹下的所有.jpg文件
	/************************************************************************
	读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化
	*************************************************************************/
	int image_count = 0;                    /****    图像数量     ****/
	Size board_size = Size(11,8);            /****    定标板上每行、列的角点数       ****/
	vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
	vector<vector<Point2f>>  corners_Seq;    /****  保存检测到的所有角点       ****/
	vector<Mat>  image_Seq;
	int successImageNum = 0;                /****   成功提取角点的棋盘图数量    ****/
	vector<string> filenames;
	int count = 0;
	int filecnt = file_vec.size();
	cv::Mat imagetmp = imread(file_vec[0], 0);
	int width = imagetmp.cols;//获取图片宽度
	int height = imagetmp.rows;//获取图片高度
	cout << "图片尺寸:" << width  << "x" <<  height << endl;
	cout << "开始提取角点………………" << endl;
	#if 0
	tbb::parallel_for(size_t(0), filenames.size(), [&](size_t i){
		cout << "parallel_for" << i << endl;
	});
	#endif
	#if 1
	tbb::spin_mutex mutexObj; //互斥锁
	tbb::parallel_for(0, filecnt, [&board_size,&corners, &mutexObj, &image_Seq, &filenames, &corners_Seq, &successImageNum, &count, &image_count](int i){
		//cout << i << endl; 
		cout << "从图片 " << file_vec[i] << "中查找角点..." << endl;
		std::stringstream StrStm;
		cv::Mat image = imread(file_vec[i]);
		Mat imageGray;
		cvtColor(image, imageGray, CV_RGB2GRAY);
		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (!patternfound)
		{
			cout << "找不到角点" << file_vec[i] << "不正确！" << endl;
		}
		else
		{	
			/* 亚像素精确化 */
			cornerSubPix(imageGray, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cout << "从图片 " << file_vec[i] << "中找到角点：" << corners.size() << endl;			
			mutexObj.lock(); //互斥
			++image_count;
			count = count + corners.size();
			successImageNum = successImageNum + 1;
			corners_Seq.push_back(corners);
			filenames.push_back(file_vec[i]);
			image_Seq.push_back(image);
			mutexObj.unlock(); //解除互斥
		}
	});
	#else
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
			cout << "找不到角点" << imageFileName << "不正确！" << endl;
			//exit(1);
			continue;
		}
		else
		{
			++image_count;
			/* 亚像素精确化 */
			cornerSubPix(imageGray, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cout << "从图片 " << imageFileName << "中找到角点：" << corners.size() << endl;
			count = count + corners.size();
			successImageNum = successImageNum + 1;
			corners_Seq.push_back(corners);
			filenames.push_back(imageFileName);
			image_Seq.push_back(image);
		}
	}
	#endif
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
				//tempPoint.x = (i-board_size.width/2)*square_size.width;//当为实际格子尺寸时表示平移多少距离
				//tempPoint.y = ((float)(2.0f*j-board_size.height)/2.0f)*square_size.height;
				tempPoint.x = i;//*square_size.height;//当为1个单位时，表示平移n个格子，-board_size.width/2
				tempPoint.y = j;//*square_size.width;
				tempPoint.z = 0;

				tempPointSet.push_back(tempPoint);
			}
		}
		object_Points.push_back(tempPointSet);
	}
	cout << "object_Points:"<< object_Points[0] << endl;
	#if 0
	vector<Point2f> imagePointSet;
	for (int j = 0; j<board_size.height; j++)
	{
		for (int i = 0; i<board_size.width; i++)
		{
			/* 假设定标板放在世界坐标系中z=0的平面上 */
			Point2f tempPoint;
			tempPoint.x = (i-board_size.width/2)*square_size.width;//当为实际格子尺寸时表示平移多少距离
			tempPoint.y = (j-board_size.height/2)*square_size.height;
			tempPointSet.push_back(tempPoint);
		}
	}
	#endif

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

	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat transformation = Mat::eye(3, 3, CV_32F);
	//Mat newcameramtx = Mat::eye(3, 3, CV_32F);
	cv::Matx33d newcameramtx;
	Mat newcame;
	double balance = 0.5;
	int flags = 0;
	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
	double err = 0.0;                        /* 每幅图像的平均误差 */
	vector<Point2f>  image_points2;             /* 保存重新计算得到的投影点 */

	flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	flags |= cv::fisheye::CALIB_CHECK_COND;
	flags |= cv::fisheye::CALIB_FIX_SKEW;
	fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
	//fisheye::estimateNewCameraMatrixForUndistortRectify(intrinsic_matrix, distortion_coeffs,image_size,transformation, newcameramtx,balance,image_size,0);
	estimateNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, Matx33d::eye(), newcameramtx, balance, image_size, 0);
	cout << "\n newcameramtx:\n";
	cout << newcameramtx << endl;
	cout << "标定完成！\n";
	cout << "生成MAP数据！\n";	
	#if 0
	fisheye::estimateNewCameraMatrixForUndistortRectify(intrinsic_matrix, distortion_coeffs, image_size,
		Matx33d::eye(), newcameramtx, 0, image_size, 0);
	cv::Vec2d f, c;
	f = Vec2f(intrinsic_matrix(0, 0), intrinsic_matrix(1, 1));
	c = Vec2f(intrinsic_matrix(0, 2), intrinsic_matrix(1, 2));
	cout << f << c << endl;
	newcameramtx(0, 2) = intrinsic_matrix(0, 2);
	newcameramtx(1, 2) = intrinsic_matrix(1, 2);
	newcameramtx(0, 0) = intrinsic_matrix(0, 0)/(1 + balance);
	newcameramtx(1, 1) = intrinsic_matrix(1, 1)/(1 + balance);
	#endif
	fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, Matx33d::eye(), newcameramtx, image_size, CV_32FC1, mapx, mapy);
	cout << "开始评价标定结果………………" << endl;
	cout << "每幅图像的定标误差：" << endl;
	for (unsigned int i = 0; i<filenames.size(); i++)
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
		#if 0
		Mat rvec1, tvec1;
		cv::Mat1f distCoeffs = cv::Mat1f::zeros(1, 4);
		Vec3f angles;
		solvePnP(object_Points[i], image_points2, intrinsic_matrix, distortion_coeffs, rvec1, tvec1);
		Mat R1;
		Rodrigues(rvec1, R1);//从旋转向量求旋转矩阵
		angles = rotationMatrixToEulerAngles(R1, 1);
		cout << "图" << filenames[i] << "的计算欧拉角：" << angles << endl;
		#endif
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		cout << "图" << filenames[i] << "的平均误差：" << err << "像素" << endl;

#if 0
		string imageSaveName;
		std::stringstream StrStm;
		StrStm << src_path;
		StrStm << sub_path;
		StrStm >> imageSaveName;
		//char *oldname=(char*)str.data();
		int pos1 = filenames[i].find_last_of('/');
		string name2 = filenames[i].substr(pos1 + 1);
		int pos2 = name2.find('.');
		string name3 = name2.substr(0,pos2);
		imageSaveName = imageSaveName + name3 + "_c.jpg";

		vector<Point2f> distort;
		vector<Point2f> undistort;
		cv::Mat t = imread(filenames[i]);
		opencvUndistortPoints(tempImagePoint, undistort, intrinsic_matrix, distortion_coeffs);
		myDistortPoints(undistort, distort, intrinsic_matrix, distortion_coeffs);
		for(int j= 0; j< undistort.size(); j++)
		{
			circle(t, tempImagePoint[j], 5, cv::Scalar(0, 100, 0), 2, 8, 0);
			circle(t, distort[j], 10, cv::Scalar(0, 0, 100), 2, 8, 0);
			circle(t, undistort[j], 20, cv::Scalar(0, 0, 255), 2, 8, 0);
		}
		imwrite(imageSaveName, t);
#endif		
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;

	double r_vec[3]={0.78520514,0.0233998,0.00969251};//eulerAngles[45,1,1]
	double t_vec[3]={0, 0, 0};
	double R_matrix[9];
	CvMat *pr_vec = cvCreateMat(1,3,CV_64FC1);
	CvMat *pt_vec = cvCreateMat(1,3,CV_64FC1);
	CvMat *pR_matrix = cvCreateMat(3,3,CV_64FC1);
	CvMat *pnew_vec = cvCreateMat(1,3,CV_64FC1);
	//CvMat *pD_mat = cvCreateMat(3,3,CV_64FC1);
	/************************************************************************
	保存标定结果
	*************************************************************************/
	cout << "保存标定结果......" << endl;
	fout << "dim,"<< width << ","<< height << "\n";
	fout << "cameraMatrix,";
	fout << intrinsic_matrix(0, 0) << "," << intrinsic_matrix(1, 1) << "," << intrinsic_matrix(0, 2) << "," << intrinsic_matrix(1, 2) <<  "\n";
	fout << "distCoefs,";
	fout << distortion_coeffs(0) << "," << distortion_coeffs(1) << "," << distortion_coeffs(2) << "," << distortion_coeffs(3) << endl;
	
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	Vec3f eulerAngles;
	for (unsigned int i = 0; i<image_Seq.size(); i++)
	{
	#if 1	//两种求欧拉角的方法，1：使用cvRodrigues2
		r_vec[0]=(double)(rotation_vectors[i](0));
		r_vec[1]=(double)(rotation_vectors[i](1));
		r_vec[2]=(double)(rotation_vectors[i](2));
		t_vec[0]=(double)(translation_vectors[i](0));
		t_vec[1]=(double)(translation_vectors[i](1));
		t_vec[2]=(double)(translation_vectors[i](2));
		cvInitMatHeader(pr_vec, 1, 3, CV_64FC1, r_vec, CV_AUTOSTEP);
		cvInitMatHeader(pR_matrix, 3, 3, CV_64FC1, R_matrix, CV_AUTOSTEP);
		cvInitMatHeader(pt_vec, 1, 3, CV_64FC1, t_vec, CV_AUTOSTEP);
		
		cvRodrigues2(pr_vec, pR_matrix, 0);//从旋转向量求旋转矩阵
		
		Mat rotation_vec_tmp(pr_vec->rows, pr_vec->cols, pr_vec->type, pr_vec->data.fl);
		Mat translation_vec_tmp(pt_vec->rows, pt_vec->cols, pt_vec->type, pt_vec->data.fl);
		//cvMat转Mat
		Mat rotation_matrix_tmp(pR_matrix->rows, pR_matrix->cols, pR_matrix->type, pR_matrix->data.fl);
		eulerAngles = rotationMatrixToEulerAngles(rotation_matrix_tmp, 1);
		cout << "图" << filenames[i] << "的欧拉角：" << endl;
		cout << eulerAngles << endl;
		
		//rotation_matrix_tmp = eulerAnglesToRotationMatrix(eulerAngles);
		eulerAngles = rotationMatrixToEulerAngles(rotation_matrix_tmp, 0);	//0表示输出为弧度，1表示输出为角度，>1表示yz为0输出为弧度
		#if 1
		Mat rotationMatrix = eulerAnglesToRotationMatrix(eulerAngles);
		*pR_matrix = rotationMatrix;
		cvRodrigues2(pR_matrix, pnew_vec, 0);	//从旋转矩阵求旋转向量
		//cvRodrigues2(pR_matrix, pD_mat, 0);		//从旋转矩阵求旋转向量
		Mat mat_tmp(pnew_vec->rows, pnew_vec->cols, pnew_vec->type, pnew_vec->data.fl);		//<lfx
		//rotation_vectors[i] = rotationMatrixToEulerAngles(mat_tmp, 1);
		#endif
		//rotation_vectors[i](0) = r_vec[0];
		//rotation_vectors[i](1) = r_vec[1];
		//rotation_vectors[i](2) = r_vec[2];
	#else	//两种求欧拉角的方法，2：使用Rodrigues
		Rodrigues(translation_vectors[i], rotation_matrix); 
		eulerAngles = rotationMatrixToEulerAngles(rotation_matrix, 1);
	#endif

		cout << "图" << filenames[i] << endl;
		cout << "原始旋转向量:" << rotation_vec_tmp << endl;
		cout << "反算旋转向量:" << mat_tmp << endl;
		cout << "原始平移向量:" << translation_vec_tmp << endl;
	}
	cout << "完成保存" << endl;
	fout << endl;
	/************************************************************************
	显示标定结果
	*************************************************************************/
	cout << "图片数量：" << endl;
	cout << image_Seq.size() << endl;
	filecnt = filenames.size();
	int tim1= getCurTime();
	tbb::parallel_for(0, filecnt, [&board_size,&image_Seq,&filenames,&mapx,&mapy,&newcameramtx,&object_Points,&src_path](int i)
	//for (unsigned int i = 0; i < filenames.size(); i++)
	{
		Mat t;// = image_Seq[image_count].clone();
		cv::remap(image_Seq[i], t, mapx, mapy, INTER_LINEAR);
		string imageSaveName, imageSaveName1, imageSaveName2;
		std::stringstream StrStm;
		StrStm << src_path;
		StrStm << sub_path;
		StrStm >> imageSaveName;
		int pos1 = filenames[i].find_last_of('/');
		string name2 = filenames[i].substr(pos1 + 1);
		int pos2 = name2.find('.');
		string name3 = name2.substr(0,pos2);
		imageSaveName1 = imageSaveName + name3 + "_d.jpg";
		imwrite(imageSaveName1, t);		
#if 1
		Mat imagegray;
		vector<Point2f> undistortcorners;
		cvtColor(t, imagegray, CV_RGB2GRAY);
		bool found = findChessboardCorners(t, board_size, undistortcorners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (!found)
		{
			cout << "找不到角点" << endl;
		}
		else
		{
			//cornerSubPix(imagegray, undistortcorners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			#if 1
			Mat rvec1, tvec1;
			cv::Mat1f distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
			Vec3f angles;
			solvePnP(object_Points[i], undistortcorners, newcameramtx, distCoeffs, rvec1, tvec1);
			Mat R1;
			Rodrigues(rvec1, R1);//从旋转向量求旋转矩阵
			angles = rotationMatrixToEulerAngles(R1, 1);
			cout << "图" << filenames[i] << "的计算欧拉角：" << angles << endl;
			#endif
			angles = rotationMatrixToEulerAngles(R1, 2);	//0表示输出为弧度，1表示输出为角度，>1表示yz为0输出为弧度
			CvMat *pRmatrix = cvCreateMat(3,3,CV_64FC1);
			CvMat *pnewvec = cvCreateMat(1,3,CV_64FC1);
			Mat rotationMatrix = eulerAnglesToRotationMatrix(angles);
			*pRmatrix = rotationMatrix;
			cvRodrigues2(pRmatrix, pnewvec, 0);	//从旋转矩阵求旋转向量
			vector<Point2f>  image_points3; 
			Mat mat_tmp(pnewvec->rows, pnewvec->cols, pnewvec->type, pnewvec->data.fl);		//<lfx
			vector<Point2f> tempImagePoint = undistortcorners;
			projectPoints(object_Points[i], mat_tmp, tvec1, newcameramtx, distCoeffs, image_points3);
			Mat image_points2Mat = Mat(1, image_points3.size(), CV_32FC2);
			
			for (size_t i = 0; i != tempImagePoint.size(); i++)
			{
				image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points3[i].x, image_points3[i].y);
			}
			Mat h, im1Reg;
			h = findHomography(undistortcorners, image_points2Mat, RANSAC);
			warpPerspective(t, t, h, t.size());
			imageSaveName2 = imageSaveName + name3 + "_warp.jpg";
			imwrite(imageSaveName2, t);
		}	
#endif
	});
	int tim2= getCurTime();
	cout<<"Running time:"<< tim2 - tim1 << "ms" <<endl;
	cout << "保存结束" << endl;
	//exit(1);
	return 0;
}

int normal_calibrate_process(string src_path)
{
	char mk_out_dir[256];

	cout << __FUNCTION__ << " : "<< src_path << endl;

	sprintf(mk_out_dir, "mkdir -p %s%s", src_path.c_str(), sub_path.c_str());
	system(mk_out_dir);

	ofstream fout(src_path+sub_path+"calibration.csv");  /**    保存定标结果的文件     **/
	getFiles(src_path);							//遍历文件夹下的所有.jpg文件
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
	vector<Mat> hm;
	//for (int i = 0; i != image_count; i++)
	for (string imageFileName : file_vec)
	{
		cout << "从图片 " << imageFileName << "中查找角点..." << endl;
		cv::Mat image = imread(imageFileName);
		/* 提取角点 */
		Mat imageGray;
		cvtColor(image, imageGray, CV_RGB2GRAY);
		bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (!patternfound)
		{
			cout << "找不到角点" << imageFileName << "不正确！" << endl;
			//exit(1);
			continue;
		}
		else
		{
			++image_count;
			/* 亚像素精确化 */
			cornerSubPix(imageGray, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
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
	cv::Mat distortion_coeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	std::vector<cv::Vec3d> rotation_vectors;                           /* 每幅图像的旋转向量 */
	std::vector<cv::Vec3d> translation_vectors;                        /* 每幅图像的平移向量 */
	cout << "标定完成！\n";
	cout << "生成MAP数据！\n";
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat transformation = Mat::eye(3, 3, CV_32F);
	Mat newcameramtx = Mat::eye(3, 3, CV_32F);
	double balance = 0.5;
	double total_err = 0.0;                   /* 所有图像的平均误差的总和 */
	double err = 0.0;                        /* 每幅图像的平均误差 */
	vector<Point2f>  image_points2;             /* 保存重新计算得到的投影点 */
	
	//cv::Vec4d distortion_coeffs(0.0, 0.0, 0.0, 0.0, 0.0);
	calibrateCamera(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, 0);
	newcameramtx = getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, balance, image_size, 0);
	//initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, transformation, newcameramtx, image_size, CV_16SC2, mapx, mapy);
	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, transformation, intrinsic_matrix, image_size, CV_32FC1, mapx, mapy);
	cout << "开始评价标定结果………………" << endl;
	cout << "每幅图像的定标误差：" << endl;
	for (unsigned int i = 0; i<image_Seq.size(); i++)
	{
		vector<Point3f> tempPointSet = object_Points[i];
		/*通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点*/
		projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);

		/*计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = corners_Seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (size_t i = 0; i != tempImagePoint.size(); i++)
		{
			image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
			tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
			//cout << "投影点-" << i << ":" << image_points2Mat.at<Vec2f>(0, i) << endl;
			//cout << "实际点-" << i << ":" << tempImagePointMat.at<Vec2f>(0, i) << endl;
		}

		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;

	double r_vec[3]={0.78520514,0.0233998,0.00969251};//eulerAngles[45,1,1]
	double t_vec[3]={0, 0, 0};
	double R_matrix[9];
	CvMat *pr_vec = cvCreateMat(1,3,CV_64FC1);
	CvMat *pR_matrix = cvCreateMat(3,3,CV_64FC1);
	CvMat *pnew_vec = cvCreateMat(1,3,CV_64FC1);
	CvMat *pt_vec = cvCreateMat(1,3,CV_64FC1);
	#if 1
	Mat camera_matrix = Mat(3, 3, CV_64FC1, Scalar::all(0));
	camera_matrix.ptr<double>(0)[0] = intrinsic_matrix(0, 0);
	camera_matrix.ptr<double>(0)[2] = intrinsic_matrix(0, 2);
	camera_matrix.ptr<double>(1)[1] = intrinsic_matrix(1, 1);
	camera_matrix.ptr<double>(1)[2] = intrinsic_matrix(1, 2);
	camera_matrix.ptr<double>(2)[2] = 1.0f;
	#endif
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
	for (unsigned int i = 0; i<image_Seq.size(); i++)
	{
		r_vec[0]=(double)(rotation_vectors[i](0));
		r_vec[1]=(double)(rotation_vectors[i](1));
		r_vec[2]=(double)(rotation_vectors[i](2));
		t_vec[0]=(double)(translation_vectors[i](0));
		t_vec[1]=(double)(translation_vectors[i](1));
		t_vec[2]=(double)(translation_vectors[i](2));


		cvInitMatHeader(pr_vec, 1, 3, CV_64FC1, r_vec, CV_AUTOSTEP);
		cvInitMatHeader(pR_matrix, 3, 3, CV_64FC1, R_matrix, CV_AUTOSTEP);
		cvInitMatHeader(pt_vec, 1, 3, CV_64FC1, t_vec, CV_AUTOSTEP);
		cvRodrigues2(pr_vec, pR_matrix, 0);//从旋转向量求旋转矩阵
		//cvRodrigues2(pR_matrix, pnew_vec, 0);//从旋转矩阵求旋转向量
		
		Mat rotation_vec_tmp(pr_vec->rows, pr_vec->cols, pr_vec->type, pr_vec->data.fl);
		Mat translation_vec_tmp(pt_vec->rows, pt_vec->cols, pt_vec->type, pt_vec->data.fl);
		//cvMat转Mat
		Mat rotation_matrix_tmp(pR_matrix->rows, pR_matrix->cols, pR_matrix->type, pR_matrix->data.fl);
		//eulerAngles = rotationMatrixToEulerAngles(rotation_matrix_tmp, 0);
		eulerAngles = rotationMatrixToEulerAngles(rotation_matrix_tmp, 1);
		cout << "第" << i + 1 << "幅图像的欧拉角：" << endl;
		cout << eulerAngles << endl;
		//rotation_matrix_tmp = eulerAnglesToRotationMatrix(eulerAngles);
		eulerAngles = rotationMatrixToEulerAngles(rotation_matrix_tmp, 2);	//0表示输出为弧度，1表示输出为角度，>1表示yz为0输出为弧度
		#if 1
		Mat rotationMatrix = eulerAnglesToRotationMatrix(eulerAngles);
		*pR_matrix = rotationMatrix;
		cvRodrigues2(pR_matrix, pnew_vec, 0);	//从旋转矩阵求旋转向量
		Mat mat_tmp(pnew_vec->rows, pnew_vec->cols, pnew_vec->type, pnew_vec->data.fl);		//<lfx
		
		vector<Point3f> tempPointSet = object_Points[i];
		/*通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点*/
		projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);
		/*计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = corners_Seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image1_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (size_t i = 0; i != tempImagePoint.size(); i++)
		{
			image1_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
			tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
		}
		
		//translation_vectors[i](0) = 0;
		//translation_vectors[i](1) = 0;
		cv::Mat distortion_coeffs1 = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
		projectPoints(tempPointSet, mat_tmp, translation_vectors[i], intrinsic_matrix, distortion_coeffs1, image_points2);
		Mat image2_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (size_t i = 0; i != tempImagePoint.size(); i++)
		{
			image2_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
			//temp2ImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
		}
		Mat h, im1Reg;
		
		h = findHomography(image1_points2Mat, image2_points2Mat, RANSAC );
		hm.push_back(h);
		cout << "Homography:\n" << h << endl;
		#endif
		vector<Point3f> worldPoint;
		//Mat worldPoint = Mat(1, tempImagePoint.size(), CV_32FC2);
		cameraToWorld(intrinsic_matrix, mat_tmp, translation_vec_tmp, image_points2, worldPoint);
		
		cout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		cout << "原始:" <<rotation_vec_tmp << endl;
		cout << "反算:" <<mat_tmp << endl;
		cout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		cout << "原始:" << translation_vec_tmp << endl;
		cout << "原始空间点:\n" << image_points2 << endl;
		cout << "计算空间点:\n" << worldPoint << endl;		//object_Points[i]
	}
	cout << "完成保存" << endl;
	fout << endl;
	/************************************************************************
	显示标定结果
	*************************************************************************/
	image_count = 0;
	cout << "图片数量：" << image_Seq.size() << endl;

	//cout << "角点数量：" << endl;
	//cout << corners_Seq[image_count].size() << endl;
	for (unsigned int i = 0; i < image_Seq.size(); i++)
	//for (string imageFileName : file_vec)
	{
		//cout << "将图片 " << imageFileName << " 进行反畸变..." << endl;
		Mat t = image_Seq[image_count].clone();
		#if 0
		for (unsigned int j = 0; j < corners_Seq[image_count].size(); j++)
		{
			circle(image_Seq[image_count], corners_Seq[image_count][j], 10, Scalar(0, 0, 255), 2, 8, 0);
		}
		#endif
		warpPerspective(image_Seq[image_count], t, hm[i], t.size());
		//circle(imageTemp, corners[j], 10, Scalar(0, 0, 255), 2, 8, 0);
		//cv::remap(image_Seq[image_count], t, mapx, mapy, INTER_LINEAR);
		string imageSaveName;
		std::stringstream StrStm;
		StrStm << src_path;
		StrStm << sub_path;
		StrStm << image_count;
		StrStm >> imageSaveName;
		imageSaveName += "_warp.jpg";
		imwrite(imageSaveName, t);
		++image_count;
	}

	#if 0
	{
		double temp[128];
		write_CAM_CALIB(temp, 128);
	}
	#endif

	cout << "保存结束" << endl;

	return 0;
}

cv::Mat get_matrix_H(cv::Point2f srcQuad[], cv::Point2f dstQuad[])
{
	cv::Mat warp_mat = cv::getPerspectiveTransform(srcQuad, dstQuad);
	return warp_mat;
}

double pixel_per_mm = 10.0;

int get_book_corner(double book_distance, double book_width, double book_heigth, double pic_width_pixel, double pic_heigth_pixel, double book_angle, Point2f corner[])
{
	double book_distance_pixel = book_distance * pixel_per_mm;
	double book_width_pixel = book_width * pixel_per_mm;
	double book_heigth_pixel = book_heigth * pixel_per_mm;
	double width_pixel = pic_width_pixel;
	double heigth_pixel = pic_heigth_pixel;
	corner[0].x = (width_pixel - book_width_pixel) / 2.0;
	corner[0].y = book_distance_pixel;
	corner[1].x = width_pixel / 2.0 + book_width_pixel / 2.0;
	corner[1].y = corner[0].y;
	corner[2].x = corner[0].x;
	corner[2].y = book_heigth_pixel + book_distance_pixel;
	corner[3].x = corner[1].x;
	corner[3].y = corner[2].y;
	return 0;
}

int main(int argc, char* argv[])
{
	int res;
	int method = 1;
	//string file_path = "/data/cap/";
	string file_path = "./cap/";
	printf("ver = %d\n", SDK_VER);
	while ((res = getopt(argc, argv, "p:t:")) != -1)
	{
		switch (res)
		{
			case 'p':
				file_path = optarg;
				break;
			case 't':
				method = atoi(optarg);
				break;
			default:
				break;
		}
	}
	if(method == 1)
	{
		fisheye_calibrate_process(file_path);
	}
	else
	{
		normal_calibrate_process(file_path);
	}
	return 0;
}
