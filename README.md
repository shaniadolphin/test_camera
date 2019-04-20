# 测试摄像头
## 自己写yuv转RGB，使用jpeg库存成jpg图
以下代码，通过savejpg的调用，可以存储jpg图片：
```
#include <jpeglib.h>
void jpeg(FILE* dest, unsigned char* rgb, unsigned int width, unsigned int height, int quality)
{
	JSAMPARRAY image;
	image = calloc(height, sizeof (JSAMPROW));
	for (size_t i = 0; i < height; i++) {
		image[i] = calloc(width * 3, sizeof (JSAMPLE));
		for (size_t j = 0; j < width; j++) {
			image[i][j * 3 + 0] = rgb[(i * width + j) * 3 + 0];
			image[i][j * 3 + 1] = rgb[(i * width + j) * 3 + 1];
			image[i][j * 3 + 2] = rgb[(i * width + j) * 3 + 2];
		}
	}
	struct jpeg_compress_struct compress;
	struct jpeg_error_mgr error;
	compress.err = jpeg_std_error(&error);
	jpeg_create_compress(&compress);
	jpeg_stdio_dest(&compress, dest);
	compress.image_width = width;
	compress.image_height = height;
	compress.input_components = 3;
	compress.in_color_space = JCS_RGB;
	jpeg_set_defaults(&compress);
	jpeg_set_quality(&compress, quality, TRUE);
	jpeg_start_compress(&compress, TRUE);
	jpeg_write_scanlines(&compress, image, height);
	jpeg_finish_compress(&compress);
	jpeg_destroy_compress(&compress);

	for (size_t i = 0; i < height; i++) {
		free(image[i]);
	}
	free(image);
}

int minmax(int min, int v, int max)
{
	return (v < min) ? min : (max < v) ? max : v;
}
 
unsigned char* yuyv2rgb(unsigned char* yuyv, unsigned int iwidth, unsigned int iheight)
{
	unsigned char* rgb = calloc(iwidth * iheight * 3, sizeof (unsigned char));
	for (size_t i = 0; i < iheight; i++) {
		for (size_t j = 0; j < iwidth; j += 2) {
			size_t index = i * iwidth + j;
			int y0 = yuyv[index * 2 + 0] << 8;
			int u = yuyv[index * 2 + 1] - 128;
			int y1 = yuyv[index * 2 + 2] << 8;
			int v = yuyv[index * 2 + 3] - 128;
			rgb[index * 3 + 0] = minmax(0, (y0 + 359 * v) >> 8, 255);
			rgb[index * 3 + 1] = minmax(0, (y0 + 88 * v - 183 * u) >> 8, 255);
			rgb[index * 3 + 2] = minmax(0, (y0 + 454 * u) >> 8, 255);
			rgb[index * 3 + 3] = minmax(0, (y1 + 359 * v) >> 8, 255);
			rgb[index * 3 + 4] = minmax(0, (y1 + 88 * v - 183 * u) >> 8, 255);
			rgb[index * 3 + 5] = minmax(0, (y1 + 454 * u) >> 8, 255);
		}
	}
	return rgb;
}

void savejpg(void)
{
	unsigned char* rgb = yuyv2rgb(test_buf, width, height);
	FILE* out = fopen("result.jpg", "w");
	jpeg(out, rgb, width, height, 100);
	fclose(out);
	free(rgb);
}
```
由于使用了jpeg库，在编译的时候需要对这个进行链接，编译的指令如下：
```
gcc capture.c -o capture -ljpeg
```

## 使用opencv进行转换和存储
opencv实现了空间转换和存图的接口，通过调用以下`savejpg`可存储jpg图片。
```
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

static void savejpg(void)
{
	char files[100];
	int tm3 = getCurTime();
	Mat mat_t(height, width, CV_8UC2, (unsigned char*)buffers[0].start);
	Mat bgrImg_t(height, width, CV_8UC3, covBuf);
	cvtColor(mat_t, bgrImg_t, CV_YUV2BGR_YUYV);
	int tm4 = getCurTime();
	imwrite(save_name, bgrImg_t);
	sprintf(files, "%s_%d", save_name, 123);
	printf("save %s use time %ums\r\n", files, tm4-tm3);
}
```
为了编译代码，编写了供cmake使用的cmakelists文件：
```
cmake_minimum_required(VERSION 2.8)
project(test)
find_package(OpenCV REQUIRED)
add_executable(test capture.cpp)
#add_executable(test capture.c)
target_link_libraries(test ${OpenCV_LIBS})
```
运行`cmake .`生成包括了opencv链接的makefile，再通过`make`编译，生成可执行文件即可测试。
测试命令如下：
```
./test -d /dev/video0 -w 1920 -m
```
