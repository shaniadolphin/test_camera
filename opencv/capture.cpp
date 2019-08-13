/****************
cmake_minimum_required(VERSION 2.8)
project(test)
find_package(OpenCV REQUIRED)
add_executable(test capture.cpp)
target_link_libraries(test ${OpenCV_LIBS})
#./test -d /dev/video2 -w 1920 -m
***************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <netdb.h> 

using namespace cv;
using namespace std;

static unsigned int width = 1280;
static unsigned int height = 720;

const char* dev_name = nullptr;
const char* ipv4addr = nullptr;
static unsigned int ipv4port;
std::string save_name = "save.jpg";

static int getCurTime(){
	struct timeval tv;    
	gettimeofday(&tv,NULL);    //该函数在sys/time.h头文件中
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;       
}

int main(int argc, char* argv[])
{
	int res;
	int capnum = 0;
	static char* wh;
	dev_name = "/dev/video0";
	ipv4addr = "192.168.199.142";
	ipv4port = atoi("12345");
	while((res = getopt(argc, argv, "w:i:d:muhc:p:")) != -1)
	{
		switch(res)
		{
			case 'm':
			break;
			case 'u':
			break;
			case 'i':
				save_name = optarg;
			break;
			case 'd':
				dev_name = optarg;
				printf("dev_name:%s\r\n",dev_name);
				if(strcmp(dev_name, "/dev/video0") == 0)
				{
					printf("/dev/video0\r\n");
					capnum = 0;
				}
				else if(strcmp(dev_name, "/dev/video1") == 0)
				{
					printf("/dev/video1\r\n");
					capnum = 1;
				}
				else if(strcmp(dev_name, "/dev/video2") == 0)
				{
					printf("/dev/video2\r\n");
					capnum = 2;
				}
				else
				{
					printf("/dev/video0\r\n");
					capnum = 0;
				}
			break;
			case 'w':
				char *tokenPtr;
				wh = optarg;
				tokenPtr = strtok(optarg, "*");
				width = atoi(tokenPtr);
				//printf("token:%s\n",tokenPtr);
				tokenPtr = strtok(NULL, "*");
				//printf("token:%s\n",tokenPtr);
				height = atoi(tokenPtr);
			break;
			case 'p':
				ipv4port = atoi(optarg);
				printf("ipv4port:%d\r\n", ipv4port);
			break;
			case 'c':
				ipv4addr = optarg;
				printf("ipv4addr:%s\r\n", ipv4addr);
			break;
			case 'h':
				std::cout << "[Usage]: " << argv[0] << " [-h]\n"
					<< "   [-p proto_file] [-m model_file] [-i image_file]\n";
			return 0;
			default:
			break;
		}
	}
	printf("capnum = %u\r\n", capnum);
	VideoCapture cap(capnum);
	cap.set(CAP_PROP_FRAME_WIDTH, width);
	cap.set(CAP_PROP_FRAME_HEIGHT, height);
	if(!cap.isOpened())  
	{  
		return -1;  
	}
	Mat frame;
	int tm3 = getCurTime();
	cap>>frame;
	int tm4 = getCurTime();
	imwrite(save_name, frame);
	printf("save jpg use time %ums\r\n", tm4-tm3);
	return 0;  
}