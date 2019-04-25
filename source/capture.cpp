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

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

#define  BUF_CNT   6
#define CLEAR(x) memset (&(x), 0, sizeof (x))

typedef enum {
	IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR,
} io_method;
 
typedef struct VideoBuffer {
	void   *start;
	unsigned int  length;
} VideoBuffer;
 

static io_method io = IO_METHOD_MMAP;
static int fd = -1;
static VideoBuffer buffers[BUF_CNT];
static unsigned int n_buffers = 0;
static unsigned int width = 1280;
static unsigned int height = 720;

static unsigned char test_buf[2592*1944*3] = {0};
static unsigned char covBuf[2592*1944*3];
FILE *fp;
const char* dev_name = nullptr;
std::string save_name = "save.jpg";
 
static void errno_exit(const char * s) {
	fprintf(stderr, "%s error %d, %s/n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

static int getCurTime(){
	struct timeval tv;    
	gettimeofday(&tv,NULL);    //该函数在sys/time.h头文件中
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;       
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
	int tim1 = getCurTime();
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
		process_image(buffers[buf.index].start, buf.length);
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
		process_image((void *) buf.m.userptr, buf.length);
		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
 
		break;
	}
	int tim2= getCurTime();
	printf("time %ums\n", tim2 - tim1);
	return 1;
}

static void mainloop(void) {
	unsigned int count;
	count = 1;
	while (count-- > 0) {
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
	}
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
	req.count = 4;
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
	} else {
		 printf("fmt.type:\t\t%d\n",fmt.type);
		 printf("pix.pixelformat:\t%c%c%c%c\n",
				 fmt.fmt.pix.pixelformat & 0xFF, 
				 (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
				 (fmt.fmt.pix.pixelformat >> 16) & 0xFF, 
				 (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
		 printf("pix.width:\t\t%d\n",fmt.fmt.pix.width);
		 printf("pix.height:\t\t%d\n",fmt.fmt.pix.height);
		 printf("pix.field:\t\t%d\n",fmt.fmt.pix.field);
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

static void savejpg(void)
{
	int tm3 = getCurTime();
	Mat mat_t(height, width, CV_8UC2, (unsigned char*)buffers[0].start);
	Mat bgrImg_t(height, width, CV_8UC3, covBuf);
	cvtColor(mat_t, bgrImg_t, CV_YUV2BGR_YUYV);
	int tm4 = getCurTime();
	imwrite(save_name, bgrImg_t);
	printf("save jpg use time %ums\r\n", tm4-tm3);
}

int main(int argc, char* argv[])
{
	int res;
	dev_name = "/dev/video0";
	while((res = getopt(argc, argv, "w:i:d:muh")) != -1)
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
				save_name = optarg;
			break;
			case 'd':
				dev_name = optarg;
			break;
			case 'w':
				width = atoi(optarg);
				if(width == 1920)
					height = 1080;
				else if(width >= 2048)
				{
					width = 2592;
					height = 1944;
				}
				else
				{
					width = 1280;
					height = 720;
				}
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
	savejpg();
	stop_capturing();
	uninit_device();
	close_device();
	exit(EXIT_SUCCESS);
}