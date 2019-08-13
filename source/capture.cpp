/****************
# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(capture)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
#set(OpenCV_DIR "/mnt/h/proj/opencv/opencv-4.1.0/release")
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
#message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Declare the executable target built from your sources
add_executable(capture capture.cpp)

# Link your application with OpenCV libraries
target_link_libraries(capture ${OpenCV_LIBS})
target_link_libraries(capture -lpthread -lm -lstdc++)
#./capture -d /dev/video0 -i 2.jpg -w 1920*1440 -t 1
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

#include <netinet/in.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "opencv2/core.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

#define  BUF_CNT   5
#define CLEAR(x) memset (&(x), 0, sizeof (x))

typedef enum {
	IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR,
} io_method;
 
typedef struct VideoBuffer {
	void   *start;
	unsigned int  length;
} VideoBuffer;
 
#define MAXLINE 4096

static io_method io = IO_METHOD_MMAP;
static int fd = -1;
static VideoBuffer buffers[BUF_CNT];
static unsigned int n_buffers = 2;
static unsigned int n_pics = 1;
static unsigned int width = 1920;
static unsigned int height = 1080;

static unsigned int ipv4port = 12345;
static unsigned char test_buf[2592*1944*2] = {0};
static unsigned char covBuf[2592*1944*3];
FILE *fp;
const char* dev_name = nullptr;
//std::string save_name = "save.jpg";
char* save_name;
const char* file_name = nullptr;
const char* ipv4addr;
static char* wh;
struct timeval tv;

static void errno_exit(const char * s) {
	fprintf(stderr, "%s error %d, %s/n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

static int getCurTime(){
	gettimeofday(&tv, NULL);    //该函数在sys/time.h头文件中
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;       
}

void myUndistortPoints(const std::vector<cv::Point2d> & src, std::vector<cv::Point2d> & dst,
const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff)
{

	dst.clear();
	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double ux = cameraMatrix.at<double>(0, 2);
	double uy = cameraMatrix.at<double>(1, 2);

	double k1 = distortionCoeff.at<double>(0, 0);
	double k2 = distortionCoeff.at<double>(0, 1);
	double p1 = distortionCoeff.at<double>(0, 2);
	double p2 = distortionCoeff.at<double>(0, 3);
	double k3 = distortionCoeff.at<double>(0, 4);
	double k4 = 0;
	double k5 = 0;
	double k6 = 0;

	for(unsigned int i = 0; i < src.size(); i++)
	{
		const cv::Point2d & p = src[i];
		//首先进行坐标转换；
		double xDistortion = (p.x - ux) / fx;
		double yDistortion = (p.y - uy) / fy;
		double xCorrected, yCorrected;
		double x0 = xDistortion;
		double y0 = yDistortion;
		//这里使用迭代的方式进行求解，因为根据2中的公式直接求解是困难的，所以通过设定初值进行迭代，这也是OpenCV的求解策略；
		for (int j = 0; j < 10; j++)
		{
			double r2 = xDistortion*xDistortion + yDistortion*yDistortion;
			double distRadialA = 1 / (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
			double distRadialB = 1. + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2;
			double deltaX = 2. * p1 * xDistortion * yDistortion + p2 * (r2 + 2. * xDistortion * xDistortion);
			double deltaY = p1 * (r2 + 2. * yDistortion * yDistortion) + 2. * p2 * xDistortion * yDistortion;
			xCorrected = (x0 - deltaX)* distRadialA * distRadialB;
			yCorrected = (y0 - deltaY)* distRadialA * distRadialB;
			xDistortion = xCorrected;
			yDistortion = yCorrected;
		}
		//进行坐标变换；
		xCorrected = xCorrected * fx + ux;
		yCorrected = yCorrected * fy + uy;
		dst.push_back(cv::Point2d(xCorrected, yCorrected));
	}
}

static void savejpg(void)
{
	int tm3 = getCurTime();
	//Mat mat_t(height, width, CV_8UC2, (unsigned char*)buffers[0].start);
	Mat mat_t(height, width, CV_8UC2, test_buf);
	Mat bgrImg_t(height, width, CV_8UC3, covBuf);
	cvtColor(mat_t, bgrImg_t, CV_YUV2BGR_YUYV);
	int tm4 = getCurTime();
	if(n_pics <= 1)
		imwrite(file_name, bgrImg_t);
	else
		imwrite(save_name, bgrImg_t);
	printf("save file using time: %ums\r\n", tm4-tm3);
}

static int xioctl(int fd, int request, void * arg) {
	int r;
	do {
		r = ioctl(fd, request, arg);
	} while (-1 == r && EINTR == errno);
	return r;
}

static void process_image(const void * p, int size) {
	memcpy(test_buf, p, size);
}

static int read_frame(void) {
	struct v4l2_buffer buf;
	unsigned int i;
	//int tim1 = getCurTime();
	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;
	case IO_METHOD_MMAP:
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return 0;
			case EIO:
				/* Could ignore EIO, see spec. */
				/* fall through */
			default:
				errno_exit("VIDIOC_DQBUF");
			}
		}
		assert(buf.index < n_buffers);
		#if 1
		process_image(buffers[buf.index].start, buf.length);
		#endif
		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
		break;
	case IO_METHOD_USERPTR:
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;
 
		if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return 0;
			case EIO:

			default:
				errno_exit("VIDIOC_DQBUF");
			}
		}
		for (i = 0; i < n_buffers; ++i)
			if (buf.m.userptr == (unsigned long) buffers[i].start
					&& buf.length == buffers[i].length)
				break;
		assert(i < n_buffers);
		#if 1
		process_image((void *) buf.m.userptr, buf.length);
		#endif
		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
 
		break;
	}
	//int tim2= getCurTime();
	//printf("time %ums\n", tim2 - tim1);
	return 1;
}

static void mainloop(void) {
	#if 1
	unsigned int count;
	char str[50];
	if(n_pics <= 1)
		count = 5;
	else
		count = n_pics;
	while (count-- > 0){
		for (;;) {
			//int tim1 = getCurTime();
			fd_set fds;
			struct timeval tv;
			int r;
			FD_ZERO(&fds);
			FD_SET(fd, &fds);
			/* Timeout. */
			tv.tv_sec = 2;
			tv.tv_usec = 0;
			r = select(fd + 1, &fds, NULL, NULL, &tv);
			if (-1 == r) {
				if (EINTR == errno)
					continue;
				errno_exit("select");
			}
			if (0 == r) {
				fprintf(stderr, "select timeout/n");
				exit(EXIT_FAILURE);
			}
			if (read_frame())
			{
				//int tim2= getCurTime();
				//printf("time %ums\n", tim2 - tim1);
				gettimeofday(&tv, NULL);    //该函数在sys/time.h头文件中

				if(n_pics <= 1)
				{
					if(count == 1)
						savejpg();
				}
				else
				{
					printf("%dmin-%dms:start\n", (int)(tv.tv_sec)%100,(int)(tv.tv_usec)/1000);
					sprintf(str,"%d_%s",count, file_name);
					save_name = str;
					gettimeofday(&tv, NULL);    //该函数在sys/time.h头文件中
					printf("%dmin-%dms:%s\n", (int)(tv.tv_sec)%100,(int)(tv.tv_usec)/1000, save_name);
					savejpg();
					gettimeofday(&tv, NULL);    //该函数在sys/time.h头文件中
					printf("%dmin-%dms:done\n", (int)(tv.tv_sec)%100,(int)(tv.tv_usec)/1000);					
				}
				break;
			}/* EAGAIN - continue select loop. */
		}
	}
	#else
		for (;;) {
		//int tim1 = getCurTime();
		fd_set fds;
		struct timeval tv;
		int r;
		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		/* Timeout. */
		tv.tv_sec = 2;
		tv.tv_usec = 0;
		r = select(fd + 1, &fds, NULL, NULL, &tv);
		if (-1 == r) {
			if (EINTR == errno)
				continue;
			errno_exit("select");
		}
		if (0 == r) {
			fprintf(stderr, "select timeout/n");
			exit(EXIT_FAILURE);
		}
		if (read_frame())
		{
			//int tim2= getCurTime();
			//printf("time %ums\n", tim2 - tim1);
			break;
		}/* EAGAIN - continue select loop. */
	}	
	
	#endif
}
 
static void stop_capturing(void) {
	enum v4l2_buf_type type;
 
	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;
	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
			errno_exit("VIDIOC_STREAMOFF");
		break;
	}
}
 
static void start_capturing(void) {
	unsigned int i;
	enum v4l2_buf_type type;
	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;
	case IO_METHOD_MMAP:
		for (i = 0; i < n_buffers; ++i) {
			struct v4l2_buffer buf;
 
			CLEAR(buf);
 
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.index = i;
 
			if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
				errno_exit("VIDIOC_QBUF");
		}
 
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 
		if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
			errno_exit("VIDIOC_STREAMON");
		break;
	case IO_METHOD_USERPTR:
		for (i = 0; i < n_buffers; ++i) {
			struct v4l2_buffer buf;
			CLEAR(buf);
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_USERPTR;
			buf.index = i;
			buf.m.userptr = (unsigned long) buffers[i].start;
			buf.length = buffers[i].length;
			if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
				errno_exit("VIDIOC_QBUF");
		}
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
			errno_exit("VIDIOC_STREAMON");
		break;
	}
}
 
static void uninit_device(void) {
	unsigned int i;
 
	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;
	case IO_METHOD_MMAP:
		for (i = 0; i < n_buffers; ++i)
			if (-1 == munmap(buffers[i].start, buffers[i].length))
				errno_exit("munmap");
		break;
	case IO_METHOD_USERPTR:
		for (i = 0; i < n_buffers; ++i)
			free(buffers[i].start);
		break;
	}
	//free(buffers);
}
#if 0
static void init_read(unsigned int buffer_size) {
	/*
	buffers = calloc(1, sizeof(*buffers));
	if (!buffers) {
		fprintf(stderr, "Out of memory/n");
		exit(EXIT_FAILURE);
	}*/
	buffers[0].length = buffer_size;
	buffers[0].start = malloc(buffer_size);
	if (!buffers[0].start) {
		fprintf(stderr, "Out of memory/n");
		exit(EXIT_FAILURE);
	}
}
#endif
static void init_mmap(void) {
	struct v4l2_requestbuffers req;
	CLEAR(req);
	#if 1
	req.count = 4;
	#else
	req.count = 2;	
	#endif
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
					"memory mapping/n", dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}
	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s/n", dev_name);
		exit(EXIT_FAILURE);
	}
	/*
	buffers = calloc(req.count, sizeof(*buffers));
	if (!buffers) {
		fprintf(stderr, "Out of memory/n");
		exit(EXIT_FAILURE);
	}
	*/
	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;
		if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
			errno_exit("VIDIOC_QUERYBUF");
		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = mmap(NULL /* start anywhere */, buf.length,
				PROT_READ | PROT_WRITE /* required */,
				MAP_SHARED /* recommended */, fd, buf.m.offset);
 
		if (MAP_FAILED == buffers[n_buffers].start)
			errno_exit("mmap");
	}
}
 
static void init_userp(unsigned int buffer_size) {
	struct v4l2_requestbuffers req;
	unsigned int page_size;
	page_size = getpagesize();
	buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);
	CLEAR(req);
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;
	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
					"user pointer i/o/n", dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}
	/*
	buffers = calloc(4, sizeof(*buffers));
	if (!buffers) {
		fprintf(stderr, "Out of memory/n");
		exit(EXIT_FAILURE);
	}
	*/
	for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
		buffers[n_buffers].length = buffer_size;
		buffers[n_buffers].start = memalign(/* boundary */page_size,
				buffer_size);
 
		if (!buffers[n_buffers].start) {
			fprintf(stderr, "Out of memory/n");
			exit(EXIT_FAILURE);
		}
	}
}
 
static void init_device(void) {
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device/n", dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device/n", dev_name);
		exit(EXIT_FAILURE);
	}
	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;
	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
			fprintf(stderr, "%s does not support streaming i/o/n", dev_name);
			exit(EXIT_FAILURE);
		}
		break;
	}
	/* Select video input, video standard and tune here. */
	CLEAR(cropcap);
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */
 
		if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {
		/* Errors ignored. */
	}
	CLEAR(fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
		errno_exit("VIDIOC_S_FMT");
	/* Note VIDIOC_S_FMT may change width and height. */
	if(-1 == xioctl(fd, VIDIOC_G_FMT, &fmt)) {
		printf("Unable to get format\n");
		errno_exit("VIDIOC_G_FMT");
	} 
	else {
		/**
		printf("fmt.type:\t\t%d\n",fmt.type);
		printf("pix.pixelformat:\t%c%c%c%c\n",
				 fmt.fmt.pix.pixelformat & 0xFF, 
				 (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
				 (fmt.fmt.pix.pixelformat >> 16) & 0xFF, 
				 (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
		**/
		printf("pix.width:\t\t%d\n",fmt.fmt.pix.width);
		printf("pix.height:\t\t%d\n",fmt.fmt.pix.height);
		//printf("pix.field:\t\t%d\n",fmt.fmt.pix.field);
	}
	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;
	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;
	case IO_METHOD_MMAP:
		init_mmap();
		break;
 
	case IO_METHOD_USERPTR:
		init_userp(fmt.fmt.pix.sizeimage);
		break;
	}
}
 
static void close_device(void) {
	if (-1 == close(fd))
		errno_exit("close");
	fd = -1;
}
 
static void open_device(void) {
	struct stat st;
	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s/n", dev_name, errno,
				strerror(errno));
		exit(EXIT_FAILURE);
	}
	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device/n", dev_name);
		exit(EXIT_FAILURE);
	}
	fd = open(dev_name, O_RDWR /* required */| O_NONBLOCK, 0);
	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s/n", dev_name, errno,
				strerror(errno));
		exit(EXIT_FAILURE);
	}
}


static long get_file_size(const char *path)
{
    long filesize = -1;
    struct stat statbuff;

    if (stat(path, &statbuff) < 0)
    {
        return filesize;
    }
    else
    {
        filesize = statbuff.st_size;
    }
    return filesize;
}

int socket_client_send_data(char *serverip, int port, char *file)
{
    int   i, sockfd, len;
    char  buffer[MAXLINE];
    struct sockaddr_in  servaddr;
    FILE *fq;
    long file_size;

    printf("###### %s ######, connect server ip = %s, port = %d\r\n", __FUNCTION__, serverip, port);
    if( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
        return 0;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    if ( inet_pton(AF_INET, serverip, &servaddr.sin_addr) <= 0)
    {
        printf("inet_pton error for %s\n", serverip);
        return 0;
    }

    if ( connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
    {
        printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
        return 0;
    }
    if ( ( fq = fopen(file, "rb") ) == NULL )
    {
        printf("File open err = %s.\n", file);
        close(sockfd);
        return 0;
    }
    file_size = get_file_size((const char *)file);
    if (file_size <= 0)
    {
        printf("File size error\n");
        close(sockfd);
        fclose(fq);
    }

    bzero(buffer, sizeof(buffer));
    sprintf(buffer, "%ld", file_size);
    for (i = 0; i < 16; i++)
    {
        if (buffer[i] == 0)
        {
            buffer[i] = ' ';
        }
    }
    write(sockfd, buffer, 16);

    while (!feof(fq))
    {
        len = fread(buffer, 1, sizeof(buffer), fq);
        if(len != write(sockfd, buffer, len))
        {
            printf("write error.\n");
            break;
        }
    }
    close(sockfd);
    fclose(fq);
    printf("socket_client_send_data end\r\n");

    return 0;
}

int main(int argc, char* argv[])
{
	int res;
	dev_name = "/dev/video0";
	ipv4addr = "192.168.199.142";
	file_name = "save.jpg";
	int listenfd;
	
	if( (listenfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 ){
		printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);
		return 0;
	}
	while((res = getopt(argc, argv, "w:i:d:t:muh")) != -1)
	{
		switch(res)
		{
			case 'm':
				io = IO_METHOD_MMAP;
			break;
			case 'u':
				io = IO_METHOD_USERPTR;
			break;
			case 'i':
				file_name = optarg;
			break;
			case 'd':
				dev_name = optarg;
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
			case 't':
				n_pics = atoi(optarg);
			break;
			case 'h':
				std::cout << "[Usage]: " << argv[0] << " [-h]\n"
					<< "   [-p proto_file] [-m model_file] [-i image_file]\n";
			return 0;
			default:
			break;
		}
	}

	open_device();
	init_device();
	start_capturing();
	mainloop();
	//savejpg();
	stop_capturing();
	uninit_device();
	close_device();
	exit(EXIT_SUCCESS);
}