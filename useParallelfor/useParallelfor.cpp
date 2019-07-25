/**@ Test parallel_for and parallel_for_
/**@ Author: chouclee
/**@ 03/17/2013*/
#include <opencv2/opencv.hpp>
#include <time.h>

using namespace cv;
using namespace std;

#if 0
#include <time.h>
namespace test
{
	class parallelTestBody : public ParallelLoopBody//参考官方给出的answer，构造一个并行的循环体类
	{
	public:
		parallelTestBody(Mat& _src)//class constructor
		{
			src = &_src;
		}
		void operator()(const Range& range) const//重载操作符（）
		{
			Mat& srcMat = *src;
			int stepSrc = (int)(srcMat.step/srcMat.elemSize1());//获取每一行的元素总个数（相当于cols*channels，等同于step1)
			for (int colIdx = range.start; colIdx < range.end; ++colIdx)
			{
				float* pData = (float*)srcMat.col(colIdx).data;
				for (int i = 0; i < srcMat.rows; ++i)
					pData[i*stepSrc] = std::pow(pData[i*stepSrc],3);
			}	
		}
	private:
		Mat* src;
	};
}//namesapce test
void parallelTestWithFor(InputArray _src)//'for' loop
{
	CV_Assert(_src.kind() == _InputArray::MAT);
	Mat src = _src.getMat();
	CV_Assert(src.isContinuous());
	int stepSrc = (int)(src.step/src.elemSize1());
	for (int x = 0; x < src.cols; ++x)
	{
		float* pData = (float*)src.col(x).data;
		for (int y = 0; y < src.rows; ++y)
			pData[y*stepSrc] = std::pow(pData[y*stepSrc], 3);
	}
}

void parallelTestWithParallel_for_(InputArray _src)//'parallel_for_' loop
{
	CV_Assert(_src.kind() == _InputArray::MAT);
	Mat src = _src.getMat();
	int totalCols = src.cols;
	typedef test::parallelTestBody parallelTestBody;
	parallel_for_(Range(0, totalCols), parallelTestBody(src));
}

int main(int argc, char* argv[])
{
	clock_t start, stop;
	Mat testInput = Mat::ones(40,400000, CV_32F);
	

	start = clock();
	parallelTestWithFor(testInput);
	stop = clock();
	cout<<"Running time using \'for\':"<<(double)(stop - start)/CLOCKS_PER_SEC*1000<<"ms"<<endl;

	start = clock();
	parallelTestWithParallel_for_(testInput);
	stop = clock();
	cout<<"Running time using \'parallel_for_\':"<<(double)(stop - start)/CLOCKS_PER_SEC*1000<<"ms"<<endl;
	return 1;
}
#else

namespace test
{
	int mandelbrot(const complex<float> &z0, const int max)
	{
		complex<float> z = z0;
		for (int t = 0; t < max; t++)
		{
			if (z.real()*z.real() + z.imag()*z.imag() > 4.0f) return t;
			z = z*z + z0;
		}
		return max;
	};

	int mandelbrotFormula(const complex<float> &z0, const int maxIter=500) {
		int value = mandelbrot(z0, maxIter);
		if(maxIter - value == 0)
		{
			return 0;
		}

		return cvRound(sqrt(value / (float) maxIter) * 255);
	};

	class ParallelMandelbrot : public ParallelLoopBody
	{
	public:
		ParallelMandelbrot (Mat &img, const float x1, const float y1, const float scaleX, const float scaleY)
			: m_img(img), m_x1(x1), m_y1(y1), m_scaleX(scaleX), m_scaleY(scaleY)
		{
		};

		virtual void operator ()(const Range& range) const// CV_OVERRIDE
		{
			for (int r = range.start; r < range.end; r++)
			{
				int i = r / m_img.cols;
				int j = r % m_img.cols;

				float x0 = j / m_scaleX + m_x1;
				float y0 = i / m_scaleY + m_y1;

				complex<float> z0(x0, y0);
				uchar value = (uchar) mandelbrotFormula(z0);
				m_img.ptr<uchar>(i)[j] = value;
			}
		};

		ParallelMandelbrot& operator=(const ParallelMandelbrot &) {
			return *this;
		};

	private:
		Mat &m_img;
		float m_x1;
		float m_y1;
		float m_scaleX;
		float m_scaleY;
	};

	void sequentialMandelbrot(Mat &img, const float x1, const float y1, const float scaleX, const float scaleY)
	{
		for (int i = 0; i < img.rows; i++)
		{
			for (int j = 0; j < img.cols; j++)
			{
				float x0 = j / scaleX + x1;
				float y0 = i / scaleY + y1;

				complex<float> z0(x0, y0);
				uchar value = (uchar) mandelbrotFormula(z0);
				img.ptr<uchar>(i)[j] = value;
			}
		}
	}
}

int main(void)
{
	Mat mandelbrotImg(4800, 5400, CV_8U);	
	float x1 = -2.1f, x2 = 0.6f;
	float y1 = -1.2f, y2 = 1.2f;
	float scaleX = mandelbrotImg.cols / (x2 - x1);
	float scaleY = mandelbrotImg.rows / (y2 - y1);

	double t1 = (double) getTickCount();
	test::ParallelMandelbrot parallelMandelbrot(mandelbrotImg, x1, y1, scaleX, scaleY);
	parallel_for_(Range(0, mandelbrotImg.rows*mandelbrotImg.cols), parallelMandelbrot);
	t1 = ((double) getTickCount() - t1) / getTickFrequency();
	cout << "Parallel Mandelbrot: " << t1 << " s" << endl;

	Mat mandelbrotImgSequential(4800, 5400, CV_8U);
	double t2 = (double) getTickCount();
	test::sequentialMandelbrot(mandelbrotImgSequential, x1, y1, scaleX, scaleY);
	t2 = ((double) getTickCount() - t2) / getTickFrequency();
	cout << "Sequential Mandelbrot: " << t2 << " s" << endl;
	cout << "Speed-up: " << t2/t1 << " times" << endl;

	imwrite("parallel.jpg", mandelbrotImg);
	imwrite("sequential.jpg", mandelbrotImgSequential);

	return 1;
}
#endif