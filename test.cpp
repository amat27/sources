#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"


using namespace std;
using namespace cv;


void showMat(cv::Mat matr)
{
	for(int i=0;i<matr.rows;++i)
		for (int j=0;j<matr.cols;++j)
		{
			std::cout<<matr.at<double>(i,j)<<" ";
		}
		std::cout<<std::endl;

}

int main()
{
	Mat A = Mat::ones(3,3,CV_64F);

	showMat(A);

	A=A.t();
	A.resize(4,2);
	A=A.t();

	showMat(A);

getchar();
}

