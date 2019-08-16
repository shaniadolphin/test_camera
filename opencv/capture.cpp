
#include <unistd.h>
#include <error.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <sys/mman.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <math.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <getopt.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;
#define CLEAR(x) memset(&(x), 0, sizeof(x))
 
#define IMAGEWIDTH 3264
#define IMAGEHEIGHT 2448
 
#define WINDOW_NAME1 "【原始图】"					//为窗口标题定义的宏
#define WINDOW_NAME2 "【图像轮廓】"        //为窗口标题定义的宏
 
Mat g_srcImage; Mat g_grayImage;
int g_nThresh = 90;
int g_nMaxThresh = 255;
RNG g_rng(12345);
Mat g_cannyMat_output;
vector<vector<Point> > g_vContours;
vector<Vec4i> g_vHierarchy;
Point point1[100000];
Point point2[100000];
Point point3[100000];
int ii,iii;
int flag2 = 0;//避障用
float number = 0;
int fps=0;
static unsigned int width = 1920;
static unsigned int height = 1440; 
FILE *fp;
const char* dev_name = nullptr;
const char* file_name = nullptr;
 
class V4L2Capture {
public:
    V4L2Capture(char *devName, int width, int height);
    virtual ~V4L2Capture();
 
    int openDevice();
    int closeDevice();
    int initDevice();
    int startCapture();
    int stopCapture();
    int freeBuffers();
    int getFrame(void **,size_t *);
    int backFrame();
    static void test();
 
private:
    int initBuffers();
 
    struct cam_buffer
    {
        void* start;
        unsigned int length;
    };
    char *devName;
    int capW;
    int capH;
    int fd_cam;
    cam_buffer *buffers;
    unsigned int n_buffers;
    int frameIndex;
};
 
V4L2Capture::V4L2Capture(char *devName, int width, int height) {
    // TODO Auto-generated constructor stub
    this->devName = devName;
    this->fd_cam = -1;
    this->buffers = NULL;
    this->n_buffers = 0;
    this->frameIndex = -1;
    this->capW=width;
    this->capH=height;
}
 
V4L2Capture::~V4L2Capture() {
    // TODO Auto-generated destructor stub
}
 
int V4L2Capture::openDevice() {
    /*设备的打开*/
    printf("video dev : %s\n", devName);
    fd_cam = open(devName, O_RDWR);
    if (fd_cam < 0) {
        perror("Can't open video device");
    }
    return 0;
}
 
int V4L2Capture::closeDevice() {
    if (fd_cam > 0) {
        int ret = 0;
        if ((ret = close(fd_cam)) < 0) {
            perror("Can't close video device");
        }
        return 0;
    } else {
        return -1;
    }
}
 
int V4L2Capture::initDevice() {
    int ret;
    struct v4l2_capability cam_cap;		//显示设备信息
    struct v4l2_cropcap cam_cropcap;	//设置摄像头的捕捉能力
    struct v4l2_fmtdesc cam_fmtdesc;	//查询所有支持的格式：VIDIOC_ENUM_FMT
    struct v4l2_crop cam_crop;			//图像的缩放
    struct v4l2_format cam_format;		//设置摄像头的视频制式、帧格式等
 
    /* 使用IOCTL命令VIDIOC_QUERYCAP，获取摄像头的基本信息*/
    ret = ioctl(fd_cam, VIDIOC_QUERYCAP, &cam_cap);
    if (ret < 0) {
        perror("Can't get device information: VIDIOCGCAP");
    }
    printf(
            "Driver Name:%s\nCard Name:%s\nBus info:%s\nDriver Version:%u.%u.%u\n",
            cam_cap.driver, cam_cap.card, cam_cap.bus_info,
            (cam_cap.version >> 16) & 0XFF, (cam_cap.version >> 8) & 0XFF,
            cam_cap.version & 0XFF);
 
    /* 使用IOCTL命令VIDIOC_ENUM_FMT，获取摄像头所有支持的格式*/
    cam_fmtdesc.index = 0;
    cam_fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format:\n");
    while (ioctl(fd_cam, VIDIOC_ENUM_FMT, &cam_fmtdesc) != -1) {
        printf("\t%d.%s\n", cam_fmtdesc.index + 1, cam_fmtdesc.description);
        cam_fmtdesc.index++;
    }
 
    /* 使用IOCTL命令VIDIOC_CROPCAP，获取摄像头的捕捉能力*/
    cam_cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 == ioctl(fd_cam, VIDIOC_CROPCAP, &cam_cropcap)) {
        printf("Default rec:\n\tleft:%d\n\ttop:%d\n\twidth:%d\n\theight:%d\n",
                cam_cropcap.defrect.left, cam_cropcap.defrect.top,
                cam_cropcap.defrect.width, cam_cropcap.defrect.height);
        /* 使用IOCTL命令VIDIOC_S_CROP，获取摄像头的窗口取景参数*/
        cam_crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        cam_crop.c = cam_cropcap.defrect;		//默认取景窗口大小
        if (-1 == ioctl(fd_cam, VIDIOC_S_CROP, &cam_crop)) {
            //printf("Can't set crop para\n");
        }
    } else {
        printf("Can't set cropcap para\n");
    }
 
    /* 使用IOCTL命令VIDIOC_S_FMT，设置摄像头帧信息*/
    cam_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cam_format.fmt.pix.width = capW;
    cam_format.fmt.pix.height = capH;
    cam_format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;		//要和摄像头支持的类型对应
    cam_format.fmt.pix.field = V4L2_FIELD_INTERLACED;
    ret = ioctl(fd_cam, VIDIOC_S_FMT, &cam_format);
    if (ret < 0) {
        perror("Can't set frame information");
    }
    /* 使用IOCTL命令VIDIOC_G_FMT，获取摄像头帧信息*/
    cam_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd_cam, VIDIOC_G_FMT, &cam_format);
    if (ret < 0) {
        perror("Can't get frame information");
    }
    printf("Current data format information:\n\twidth:%d\n\theight:%d\n",
            cam_format.fmt.pix.width, cam_format.fmt.pix.height);
    ret = initBuffers();
    if (ret < 0) {
        perror("Buffers init error");
        //exit(-1);
    }
    return 0;
}
 
int V4L2Capture::initBuffers() {
    int ret;
    /* 使用IOCTL命令VIDIOC_REQBUFS，申请帧缓冲*/
    struct v4l2_requestbuffers req;
    CLEAR(req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(fd_cam, VIDIOC_REQBUFS, &req);
    if (ret < 0) {
        perror("Request frame buffers failed");
    }
    if (req.count < 2) {
        perror("Request frame buffers while insufficient buffer memory");
    }
    buffers = (struct cam_buffer*) calloc(req.count, sizeof(*buffers));
    if (!buffers) {
        perror("Out of memory");
    }
    for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
        struct v4l2_buffer buf;
        CLEAR(buf);
        // 查询序号为n_buffers 的缓冲区，得到其起始物理地址和大小
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        ret = ioctl(fd_cam, VIDIOC_QUERYBUF, &buf);
        if (ret < 0) {
            printf("VIDIOC_QUERYBUF %d failed\n", n_buffers);
            return -1;
        }
        buffers[n_buffers].length = buf.length;
        //printf("buf.length= %d\n",buf.length);
        // 映射内存
        buffers[n_buffers].start = mmap(
                NULL, // start anywhere
                buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_cam,
                buf.m.offset);
        if (MAP_FAILED == buffers[n_buffers].start) {
            printf("mmap buffer%d failed\n", n_buffers);
            return -1;
        }
    }
    return 0;
}
 
int V4L2Capture::startCapture() {
    unsigned int i;
    for (i = 0; i < n_buffers; i++) {
        struct v4l2_buffer buf;
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (-1 == ioctl(fd_cam, VIDIOC_QBUF, &buf)) {
            printf("VIDIOC_QBUF buffer%d failed\n", i);
            return -1;
        }
    }
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(fd_cam, VIDIOC_STREAMON, &type)) {
        printf("VIDIOC_STREAMON error");
        return -1;
    }
    return 0;
}
 
int V4L2Capture::stopCapture() {
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl(fd_cam, VIDIOC_STREAMOFF, &type)) {
        printf("VIDIOC_STREAMOFF error\n");
        return -1;
    }
    return 0;
}/*ok*/
 
int V4L2Capture::freeBuffers() {
    unsigned int i;
    for (i = 0; i < n_buffers; ++i) {
        if (-1 == munmap(buffers[i].start, buffers[i].length)) {
            printf("munmap buffer%d failed\n", i);
            return -1;
        }
    }
    free(buffers);
    return 0;
}
 
int V4L2Capture::getFrame(void **frame_buf, size_t* len) {
    struct v4l2_buffer queue_buf;
    CLEAR(queue_buf);
    queue_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    queue_buf.memory = V4L2_MEMORY_MMAP;
    if (-1 == ioctl(fd_cam, VIDIOC_DQBUF, &queue_buf)) {
        printf("VIDIOC_DQBUF error\n");
        return -1;
    }
    *frame_buf = buffers[queue_buf.index].start;
    *len = buffers[queue_buf.index].length;
    frameIndex = queue_buf.index;
    return 0;
}
 
int V4L2Capture::backFrame() {
    if (frameIndex != -1) {
        struct v4l2_buffer queue_buf;
        CLEAR(queue_buf);
        queue_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        queue_buf.memory = V4L2_MEMORY_MMAP;
        queue_buf.index = frameIndex;
        if (-1 == ioctl(fd_cam, VIDIOC_QBUF, &queue_buf)) {
            printf("VIDIOC_QBUF error\n");
            return -1;
        }
        return 0;
    }
    return -1;
}

void V4L2Capture::test() {
    unsigned char *yuv422frame = NULL;
    unsigned long yuvframeSize = 0;
 
    string videoDev = videoDev;
    V4L2Capture *vcap = new V4L2Capture(const_cast<char*>(videoDev.c_str()), width, height);
    vcap->openDevice();
    vcap->initDevice();
    vcap->startCapture();
    vcap->getFrame((void **) &yuv422frame, (size_t *)&yuvframeSize);
 
    vcap->backFrame();
    vcap->freeBuffers();
    vcap->closeDevice();
}

void line2(Point point3[100000], int n)
{
    float aa, bb, cc, dd, ee, ff, gg;
    int jj = 0;
    for (;jj <n;jj++)
    {
        aa += point3[jj].x*point3[jj].x;
        bb += point3[jj].x;
        cc += point3[jj].x*point3[jj].y;
        dd += point3[jj].y;
    }
    ee = aa*n - bb*bb;
    if ((int)(ee* 100) != 0)
    {
        ff = (n*cc - bb*dd) / ee;
        gg = (dd - bb*ff) / n;
    }
    else {
        ff = 0;
        gg = 1;
    }
    Point point0, pointn;
    point0.y = 0;
    point0.x = gg;
    pointn.y = (n-1);
    pointn.x = ((n-1) * ff + gg);
 
    Mat draw_ing2 = Mat::zeros(g_cannyMat_output.size(), CV_8UC3);
    line(draw_ing2, point0, pointn, (255, 255, 255));
    //imshow("10", draw_ing2);
    cout << "\n"<<ff <<"      "<< gg << endl;
    float the =180*atan(ff)/3.14159;
    float dis = ff * 160+gg - 160;
    cout << the << "    " << dis << endl;
    //正中心ff=0，gg=160，逆时ff为正，顺时ff为负
}
void findcolor(cv::Mat &image)
{
    cv::Mat_<cv::Vec3b>::iterator it = image.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::iterator itend = image.end<cv::Vec3b>();
    ii = 0;
    iii = 0;
    int flagg = 0;
        cv::Mat srcX(image.rows, image.cols , CV_32F);
        cv::Mat srcY(image.rows, image.cols, CV_32F);
        for (int i = 0;i < image.rows;i++)
        {
            for (int j = 0;j < image.cols;j++)
            {
                if (flagg == 0)/*这样遍历水平方向无法得到有效数据*/
                {
 
                    if ((*it)[0] == 255 && (*it)[1] == 0 && (*it)[2] == 255)
                    {
                        flagg = 1;
                        point1[ii].x = i;
                        point1[ii].y = j;
                        ii++;
                    }
 
                }
                else
                {
                    if ((*it)[0] == 255 && (*it)[1] == 0 && (*it)[2] == 255)
                    {
                        flagg = 0;
                        point2[iii].x = i;
                        point2[iii].y = j;
                        iii++;
                    }
                }
                if (it == itend)
                    break;
                else it++;
            }
        }
        IplImage pImg = IplImage(image);
        CvArr* arr = (CvArr*)&pImg;
        int nn = ii;
        for (;ii > 0;ii--)
        {
            point3[ii].x = (point1[ii].x + point2[ii].x) / 2;
            point3[ii].y = (point1[ii].y + point2[ii].y) / 2;
            //circle(image, point3[ii], 1, (255, 255, 255));
            cvSet2D(arr, point3[ii].x, point3[ii].y, Scalar(255, 255, 255));
        }
        line2(point3, nn);
}
 
void on_ThreshChange(int, void* )
{
    // 使用Canndy检测边缘
    Canny( g_grayImage, g_cannyMat_output, g_nThresh, g_nThresh*2, 3 );
 
    // 找到轮廓
    findContours( g_cannyMat_output, g_vContours, g_vHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
 
    // 计算矩
    vector<Moments> mu(g_vContours.size() );
    for(unsigned int i = 0; i < g_vContours.size(); i++ )
    { mu[i] = moments( g_vContours[i], false ); }
 
    //  计算中心矩
    vector<Point2f> mc( g_vContours.size() );
    for( unsigned int i = 0; i < g_vContours.size(); i++ )
    { mc[i] = Point2f( static_cast<float>(mu[i].m10/mu[i].m00), static_cast<float>(mu[i].m01/mu[i].m00 )); }
 
    // 绘制轮廓
    Mat drawing = Mat::zeros(g_cannyMat_output.size(), CV_8UC3);
    for( unsigned int i = 0; i< g_vContours.size(); i++ )
    {
        //Scalar color = Scalar( g_rng.uniform(0, 255), g_rng.uniform(0,255), g_rng.uniform(0,255) );//随机生成颜色值
        Scalar color = Scalar(255, 0, 255);
        drawContours( drawing, g_vContours, i, color, 2, 8, g_vHierarchy, 0, Point() );//绘制外层和内层轮廓
        circle( drawing, mc[i], 4, color, -1, 8, 0 );;//绘制圆
    }
 
    findcolor(drawing);
    //line1(point1,point2,ii,iii);
 
    // 显示到窗口中
    //imshow( WINDOW_NAME2, drawing );
 
}
 
void findline(Mat image)
{
    cv::Mat_<cv::Vec3b>::iterator it = image.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::iterator itend = image.end<cv::Vec3b>();
    for (;it != itend;it++)
    {
        if ((*it)[1] == 0 && (*it)[2] >= 100)//条件可能需要改变
        {
            if(flag2==0)
            {
                flag2 = 1;
                cout << "注意line1，避障"<<endl;
                //向主控发送消息
            }
            else
            {
                cout << "注意line2，避障" << endl;
                //向主控发送消息
                //避障一与避障二中间要隔一段时间
            }
 
        }
    }
}
void wave(const cv::Mat &image, cv::Mat &result)
{
    cv::Mat srcX(image.rows / 2, image.cols / 2, CV_32F);
    cv::Mat srcY(image.rows / 2, image.cols / 2, CV_32F);
    for (int i = 0;i<image.rows /2;i++)
        for (int j = 0;j < image.cols /2;j++)
        {
            srcX.at<float>(i, j) = 2 * j;
            srcY.at<float>(i, j) = 2 * i;
        }
    cv::remap(image, result, srcX, srcY, cv::INTER_LINEAR);
}
 
void VideoPlayer() {
    unsigned char *yuv422frame = NULL;
    unsigned long yuvframeSize = 0;
 
    string videoDev = dev_name;
    V4L2Capture *vcap = new V4L2Capture(const_cast<char*>(videoDev.c_str()), width, height);
    vcap->openDevice();
    vcap->initDevice();
    vcap->startCapture();
 
    cvNamedWindow("Capture",CV_WINDOW_AUTOSIZE);
    IplImage* img;
    CvMat cvmat;
    double t;
    clock_t start, end;
    double number=0;
    int fps=0;
    while(1){
        start=clock();
        t = (double)cvGetTickCount();
        vcap->getFrame((void **) &yuv422frame, (size_t *)&yuvframeSize);
        cvmat = cvMat(IMAGEHEIGHT,IMAGEWIDTH,CV_8UC3,(void*)yuv422frame);		//CV_8UC3
        //解码
        img = cvDecodeImage(&cvmat,1);
        if(!img){
            printf("DecodeImage error!\n");
        }
        
        cv::Mat g_srcImage = cv::cvarrToMat(img,true);
        //cvShowImage("Capture",img);
        cvReleaseImage(&img);
        vcap->backFrame();


        if((cvWaitKey(1)&255) == 27){
            exit(0);
        }

        wave(g_srcImage, g_srcImage);
#if 0
        findline(g_srcImage);
        // 把原图像转化成灰度图像并进行平滑
        cvtColor(g_srcImage, g_grayImage, COLOR_BGR2GRAY);
        blur(g_grayImage, g_grayImage, Size(3, 3));
        //创建滚动条并进行初始化
        createTrackbar(" 阈值", WINDOW_NAME1, &g_nThresh, g_nMaxThresh, on_ThreshChange);
        on_ThreshChange(0, 0);

#endif
		t = (double)cvGetTickCount() - t;
		printf("Used time is %g ms\n", (t / (cvGetTickFrequency() * 1000)));
		end =clock();
		number=number+end-start;
		fps++;
		if (number/ CLOCKS_PER_SEC>= 0.25)//windows10 for CLK_TCK
		{
			cout<<fps<<endl;
			fps = 0;
			number = 0;
		}
	}
	vcap->stopCapture();
	vcap->freeBuffers();
	vcap->closeDevice();
 
}

int main(int argc, char* argv[]) {
	int res;
	dev_name = "/dev/video0";
	file_name = "save.jpg";
	while((res = getopt(argc, argv, "w:i:d:h")) != -1)
	{
		switch(res)
		{
			case 'i':
				file_name = optarg;
			break;
			case 'd':
				dev_name = optarg;
			break;
			case 'w':
				char *tokenPtr;
				tokenPtr = strtok(optarg, "*");
				width = atoi(tokenPtr);
				printf("width:%s\n",tokenPtr);
				tokenPtr = strtok(NULL, "*");
				printf("height:%s\n",tokenPtr);
				height = atoi(tokenPtr);
			break;	
			case 'h':
				std::cout << "[Usage]: " << argv[0] << " [-h]\n"
					<< "   [-d /dev/video0] [-w 1920*1440] [-i image_file.jpg]\n";
			return 0;
			default:
			break;
        }
    }


    VideoPlayer();
 
    return 0;
}