// SlovePNPByOpenCV.cpp : �������̨Ӧ�ó������ڵ㡣
//

/*
���̻���VS2013+OpenCV2.4.X�������������ǰ��Ҫ��opencv��·���������ó��Լ������ϵģ�
�����Ļ��ο��ҵĲ��͡�OpenCV2+����ϵ�У�һ����OpenCV2.4.9�İ�װ����ԡ����������ṩ������Ƭ��
����DSC03323���ǡ�ʵ�顱������ͼƬ�������ڼ�����ɺ󣬻���D�̸�Ŀ¼����������txt���ֱ�洢��
�������������ϵ�����ꡢ�����������ת�ǡ�
@Author��VShawn
@URL��http://www.cnblogs.com/singlex/
*/
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;

//���ռ����Z����ת
//������� x yΪ�ռ��ԭʼx y����
//thetazΪ�ռ����Z����ת���ٶȣ��Ƕ��Ʒ�Χ��-180��180
//outx outyΪ��ת��Ľ������
void codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
	double x1 = x;//����������һ�Σ���֤&x == &outx���������Ҳ�ܼ�����ȷ
	double y1 = y;
	double rz = thetaz * CV_PI / 180;
	outx = cos(rz) * x1 - sin(rz) * y1;
	outy = sin(rz) * x1 + cos(rz) * y1;
}

//���ռ����Y����ת
//������� x zΪ�ռ��ԭʼx z����
//thetayΪ�ռ����Y����ת���ٶȣ��Ƕ��Ʒ�Χ��-180��180
//outx outzΪ��ת��Ľ������
void codeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
	double x1 = x;
	double z1 = z;
	double ry = thetay * CV_PI / 180;
	outx = cos(ry) * x1 + sin(ry) * z1;
	outz = cos(ry) * z1 - sin(ry) * x1;
}

//���ռ����X����ת
//������� y zΪ�ռ��ԭʼy z����
//thetaxΪ�ռ����X����ת���ٶȣ��Ƕ��ƣ���Χ��-180��180
//outy outzΪ��ת��Ľ������
void codeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
	double y1 = y;//����������һ�Σ���֤&y == &y���������Ҳ�ܼ�����ȷ
	double z1 = z;
	double rx = thetax * CV_PI / 180;
	outy = cos(rx) * y1 - sin(rx) * z1;
	outz = cos(rx) * z1 + sin(rx) * y1;
}


//��������������ת������ϵ
//�������old_x��old_y��old_zΪ��תǰ�ռ�������
//vx��vy��vzΪ��ת������
//thetaΪ��ת�ǶȽǶ��ƣ���Χ��-180��180
//����ֵΪ��ת�������
cv::Point3f RotateByVector(double old_x, double old_y, double old_z, double vx, double vy, double vz, double theta)
{
	double r = theta * CV_PI / 180;
	double c = cos(r);
	double s = sin(r);
	double new_x = (vx*vx*(1 - c) + c) * old_x + (vx*vy*(1 - c) - vz*s) * old_y + (vx*vz*(1 - c) + vy*s) * old_z;
	double new_y = (vy*vx*(1 - c) + vz*s) * old_x + (vy*vy*(1 - c) + c) * old_y + (vy*vz*(1 - c) - vx*s) * old_z;
	double new_z = (vx*vz*(1 - c) - vy*s) * old_x + (vy*vz*(1 - c) + vx*s) * old_y + (vz*vz*(1 - c) + c) * old_z;
	return cv::Point3f(new_x, new_y, new_z);
}
void test();

int main()
{
	vector<cv::Point2f> Points2D;
	/****************a6000����**********************/
	//��ʼ���������Opencv
	double camD[9] = {
		6800.7, 0, 3065.8,
		0, 6798.1, 1667.6,
		0, 0, 1 };
	cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);

	//�������
	double distCoeffD[5] = { -0.189314, 0.444657, -0.00116176, 0.00164877, -2.57547 };
	cv::Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

	//P1-P4ΪXOY���ϵĹ������Z����Ϊ0��P5��Z���겻Ϊ0

	//������ͼ1 DSC03321
	Points2D.push_back(cv::Point2f(2985, 1688));	//P1
	Points2D.push_back(cv::Point2f(5081, 1690));	//P2
	Points2D.push_back(cv::Point2f(2997, 2797));	//P3
	//Points2D.push_back(cv::Point2f(5544, 2757));	//P4
	Points2D.push_back(cv::Point2f(4148, 673));	//P5

	////������ͼ2 DSC03323
	//Points2D.push_back(cv::Point2f(3062, 3073));	//P1
	//Points2D.push_back(cv::Point2f(3809, 3089));	//P2
	//Points2D.push_back(cv::Point2f(3035, 3208));	//P3
	////p4psolver2.Points2D.push_back(cv::Point2f(3838, 3217));	//P4
	//Points2D.push_back(cv::Point2f(3439, 2691));	//P5

	//��������������
	vector<cv::Point3f> Points3D;
	Points3D.push_back(cv::Point3f(0, 0, 0));		//P1 ��ά����ĵ�λ�Ǻ���
	Points3D.push_back(cv::Point3f(0, 200, 0));		//P2
	Points3D.push_back(cv::Point3f(150, 0, 0));		//P3
	//Points3D.push_back(cv::Point3f(150, 200, 0));	//P4
	Points3D.push_back(cv::Point3f(0, 100, 105));	//P5

	//��ʼ���������
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

	//���ַ������
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_ITERATIVE);	//ʵ��������ƺ�ֻ����4��������������⣬5�����ǹ���4��ⲻ����ȷ�Ľ�
	solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_P3P);			//Gao�ķ�������ʹ�������ĸ������㣬������������������4Ҳ���ܶ���4
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);			//�÷�����������N��λ�˹���

	//��ת��������ת����
	//��ȡ��ת����
	double rm[9];
	cv::Mat rotM(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, rotM);
	double r11 = rotM.ptr<double>(0)[0];
	double r12 = rotM.ptr<double>(0)[1];
	double r13 = rotM.ptr<double>(0)[2];
	double r21 = rotM.ptr<double>(1)[0];
	double r22 = rotM.ptr<double>(1)[1];
	double r23 = rotM.ptr<double>(1)[2];
	double r31 = rotM.ptr<double>(2)[0];
	double r32 = rotM.ptr<double>(2)[1];
	double r33 = rotM.ptr<double>(2)[2];

	/*************************************�˴�������������ת��**********************************************/
	//������������ϵ��������תŷ���ǣ���ת�����ת����������ϵ��
	//��ת˳��Ϊz��y��x
	//ԭ������ӣ�
	double thetaz = atan2(r21, r11) / CV_PI * 180;
	double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
	double thetax = atan2(r32, r33) / CV_PI * 180;

	ofstream fout("pnp_theta.txt");
	//fout << -1 * thetax << endl << -1 * thetay << endl << -1 * thetaz << endl;
	fout << -1 * thetax << ", " << -1 * thetay << ", " << -1 * thetaz << endl;
	cout << "angles:" << -1 * thetax << ", " << -1 * thetay << ", " << -1 * thetaz << endl;
	fout.close();
	/*************************************�˴�������������ת��END**********************************************/

	/*************************************�˴�������������ϵԭ��Oc����������ϵ�е�λ��**********************************************/
	/* ��ԭʼ����ϵ������תz��y��x������ת�󣬻�����������ϵ��ȫƽ�У���������ת������OcOw�������ת */
	/* ��������֪��������������ϵ��ȫƽ��ʱ��OcOw��ֵ */
	/* ��ˣ�ԭʼ����ϵÿ����ת��ɺ󣬶�����OcOw����һ�η�����ת�����տ��Եõ���������ϵ��ȫƽ��ʱ��OcOw */
	/* ����������-1������������ϵ����������� */
	/***********************************************************************************/

	//���ƽ�ƾ��󣬱�ʾ���������ϵԭ�㣬��������(x,y,z)�ߣ��͵�����������ϵԭ��
	double tx = tvec.ptr<double>(0)[0];
	double ty = tvec.ptr<double>(0)[1];
	double tz = tvec.ptr<double>(0)[2];

	//x y z ΪΨһ���������ԭʼ����ϵ�µ�����ֵ
	//Ҳ��������OcOw���������ϵ�µ�ֵ
	double x = tx, y = ty, z = tz;

	//�������η�����ת
	codeRotateByZ(x, y, -1 * thetaz, x, y);
	codeRotateByY(x, z, -1 * thetay, x, z);
	codeRotateByX(y, z, -1 * thetax, y, z);


	//����������������ϵ�µ�λ������
	//������OcOw����������ϵ�µ�ֵ
	double Cx = x*-1;
	double Cy = y*-1;
	double Cz = z*-1;

	ofstream fout2("pnp_t.txt");
	//fout2 << Cx << endl << Cy << endl << Cz << endl;
	fout2 << Cx << ", " << Cy << ", " << Cz << endl;
	fout2.close();
	cout << "world position:" << Cx << ", " << Cy << ", " << Cz << endl;
	
	/*************************************�˴�������������ϵԭ��Oc����������ϵ�е�λ��END**********************************************/

	//��ͶӰ����λ�˽��Ƿ���ȷ
	vector<cv::Point2f> projectedPoints;
	Points3D.push_back(cv::Point3f(0, 100, 105));
	cv::projectPoints(Points3D, rvec, tvec, camera_matrix, distortion_coefficients, projectedPoints);

	test();
	return 0;
}


//�����ã��������
void test()
{
	double x = 1, y = 2, z = 3;
	//codeRotateByZ(x, y, -90, x, y);
	//codeRotateByY(x, z, -90, x, z);
	//codeRotateByX(y, z, 90, y, z);
	//cout << endl << "   (1,2,3) -> (" << x << ',' << y << ',' << z << ")" << endl << endl;

	if (1 == 1)
	{
		double x1 = 0, y1 = 1, z1 = 2;

		vector<cv::Point3f> rotateAxis;//��ת����˳��
		rotateAxis.push_back(cv::Point3f(1, 0, 0));//��x��
		rotateAxis.push_back(cv::Point3f(0, 1, 0));//��y��
		rotateAxis.push_back(cv::Point3f(0, 0, 1));//��z��

		vector<double> theta;//��ת�ĽǶ�˳��
		theta.push_back(90);
		theta.push_back(-90);
		theta.push_back(-180);
		cv::Point3f p1 = RotateByVector(x1, y1, z1, rotateAxis[0].x, rotateAxis[0].y, rotateAxis[0].z, theta[0]);
		rotateAxis[1] = RotateByVector(rotateAxis[1].x, rotateAxis[1].y, rotateAxis[1].z, rotateAxis[0].x, rotateAxis[0].y, rotateAxis[0].z, theta[0]);
		rotateAxis[2] = RotateByVector(rotateAxis[2].x, rotateAxis[2].y, rotateAxis[2].z, rotateAxis[0].x, rotateAxis[0].y, rotateAxis[0].z, theta[0]);
		cv::Point3f p2 = RotateByVector(p1.x, p1.y, p1.z, rotateAxis[1].x, rotateAxis[1].y, rotateAxis[1].z, theta[1]);
		rotateAxis[2] = RotateByVector(rotateAxis[2].x, rotateAxis[2].y, rotateAxis[2].z, rotateAxis[1].x, rotateAxis[1].y, rotateAxis[1].z, theta[1]);
		cv::Point3f p3 = RotateByVector(p2.x, p2.y, p2.z, rotateAxis[2].x, rotateAxis[2].y, rotateAxis[2].z, theta[2]);
	}

	if (1 == 1)
	{
		double x1 = 0, y1 = 1, z1 = 2;

		codeRotateByZ(x1, y1, -180, x1, y1);
		codeRotateByY(x1, z1, -90, x1, z1);
		codeRotateByX(y1, z1, 90, y1, z1);
		cout << x1 << endl;;
	}

	//cv::Point3f np = RotateByVector(x1, y1, z1, 1, 0, 0, 90);
	////codeRotateByX(y1, z1, -90, y1, z1);
	////codeRotateByZ(x1, y1, -90, x1, y1);
	////codeRotateByY(x1, z1, -90, x1, z1);
	//codeRotateByX(y1, z1, 90, y1, z1);
	////codeRotateByZ(x1, y1, -90, x1, y1);
}