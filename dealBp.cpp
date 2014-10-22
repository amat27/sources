#include <fstream>
#include <iostream>
#include <fstream>
#include <string>
#include<cstring>
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

using namespace cv;
using namespace detail;
using namespace std;

//ofstream wmat("Transparent.txt");
//ofstream wmat1("Transparent1.txt");

void delete_coutours(string filename)
{
	Mat img = imread("E:\\stitching\\warped\\" + filename + ".jpg");
	cout << img.channels() << endl;
	Mat_<Vec3b> img_ = img;
	auto i = img.size();
	//Mat img2;
	//img2 = img.reshape(4,0);
	//cout << img2.channels() << endl;
	CvScalar color;
	color.val[0] = 0;
	color.val[1] = 0;
	color.val[2] = 0;
	color.val[3] = 255;
	Mat img1(i, CV_64FC4, color);
	Mat_<Vec4b> img1_ = img1;

	for (int i = 0; i < img.rows; ++i)
	for (int j = 0; j < img.cols; ++j){
		if (img_(i, j)[0] == 0 && img_(i, j)[1] == 0 && img_(i, j)[2] == 0)
		{
			img1_(i, j)[3] = 0;
		}
		else
		{
			img1_(i, j)[0] = img_(i, j)[0];
			img1_(i, j)[1] = img_(i, j)[1];
			img1_(i, j)[2] = img_(i, j)[2];
			img1_(i, j)[3] = 255;
		}
	}

	img1 = img1_;
	cout << img1.channels() << endl;


	// 	cout << img1.channels() << endl;
	//wmat << img << std::endl;
	// 	img.copyTo(img1);
	// 	cout << img1.channels() << endl;
	//wmat1 << img1 << endl;
	imwrite(filename + ".png", img1);

	//system("pause");

}

void delete_all()
{
	delete_coutours("G9PQ0282"); delete_coutours("G9PQ0283"); delete_coutours("G9PQ0284"); delete_coutours("G9PQ0285"); delete_coutours("G9PQ0286"); delete_coutours("G9PQ0287"); delete_coutours("G9PQ0288"); delete_coutours("G9PQ0302"); delete_coutours("G9PQ0303"); delete_coutours("G9PQ0304"); delete_coutours("G9PQ0305"); delete_coutours("G9PQ0306"); delete_coutours("G9PQ0307"); delete_coutours("G9PQ0308"); delete_coutours("G9PQ0319"); delete_coutours("G9PQ0320"); delete_coutours("G9PQ0321"); delete_coutours("G9PQ0322"); delete_coutours("G9PQ0323"); delete_coutours("G9PQ0324"); delete_coutours("G9PQ0325"); delete_coutours("G9PQ0335"); delete_coutours("G9PQ0336"); delete_coutours("G9PQ0337"); delete_coutours("G9PQ0338"); delete_coutours("G9PQ0339"); delete_coutours("G9PQ0340"); delete_coutours("G9PQ0341"); delete_coutours("G9PQ0354"); delete_coutours("G9PQ0355"); delete_coutours("G9PQ0356"); delete_coutours("G9PQ0357"); delete_coutours("G9PQ0358"); delete_coutours("G9PQ0359"); delete_coutours("G9PQ0360"); delete_coutours("G9PQ0380"); delete_coutours("G9PQ0381"); delete_coutours("G9PQ0382"); delete_coutours("G9PQ0383"); delete_coutours("G9PQ0384"); delete_coutours("G9PQ0385"); delete_coutours("G9PQ0386"); delete_coutours("G9PQ0418"); delete_coutours("G9PQ0419"); delete_coutours("G9PQ0420"); delete_coutours("G9PQ0421"); delete_coutours("G9PQ0422"); delete_coutours("G9PQ0423"); delete_coutours("G9PQ0424"); delete_coutours("G9PQ0442"); delete_coutours("G9PQ0443"); delete_coutours("G9PQ0444"); delete_coutours("G9PQ0445"); delete_coutours("G9PQ0446"); delete_coutours("G9PQ0447"); delete_coutours("G9PQ0448");
}


double distance(int i, int j, int center[2])
{
	double d;
	d = sqrt((i - center[0]) ^ 2 + (j - center[1]) ^ 2);
	return d;
}

void corp(string filename)
{
	string temp("E:\\stitching\\warped\\");
	Mat img = imread( temp+ filename + ".tif");
	//cout << img.channels() << endl;
	Mat_<Vec3b> img_ = img;

	int center[2] = { img.rows / 2, img.cols / 2 };
	int upperleft[3] = { img.rows / 2, img.cols / 2, 0 };
	int bottonleft[3] = { img.rows / 2, img.cols / 2, 0 };
	int upperright[3] = { img.rows / 2, img.cols / 2, 0 };
	int bottonright[3] = { img.rows / 2, img.cols / 2, 0 };

	
	for (int i = 0; i < img.rows; ++i)
	for (int j = 0; j < img.cols; ++j){
		double d = (i - center[0]) * (i - center[0]) + (j - center[1]) * (j - center[1]);
		//cout << d << ";";
		if (img_(i, j)[0] <= 30 && 
			img_(i, j)[1] <= 30 && 
			img_(i, j)[2] <= 30)
			continue;
		else
		{
			if (i<=center[0]&&j<=center[1]&&d > upperleft[2]) {
				upperleft[0] = i;
				upperleft[1] = j;
				upperleft[2] = d;
			}
			if (i<=center[0]&&j >= center[1]&& d > upperright[2]) {
				upperright[0] = i;
				upperright[1] = j;
				upperright[2] = d;
			}
			if (i >=center[0] &&j<=center[1] && d > bottonleft[2]) {
				bottonleft[0] = i;
				bottonleft[1] = j;
				bottonleft[2] = d;
			}
			if (i >= center[0] && j >= center[1] && d > bottonright[2]){
				bottonright[0] = i;
				bottonright[1] = j;
				bottonright[2] = d;
			}
		}
	}

	//cout << "img size: " << img.rows << "," << img.cols << endl << "center: " << center[0]<<","<<center[1]<<endl;
	//cout << "upperleft: "<< upperleft[0] << "," << upperleft[1] << endl << "upperright: "<< upperright[0] << "," << upperright[1] << endl << "bottonleft: " << bottonleft[0] << "," << bottonleft[1] << endl << "bottonright: " << bottonright[0] << "," << bottonright[1] << endl;

	//CvScalar color;
	//color.val[0] = 200; color.val[1] = 200; color.val[2] = 200;
	//circle(img, Point(upperleft[1], upperleft[0]), 20, color);
	//circle(img, Point(upperright[1], upperright[0]), 20, color);
	//circle(img, Point(bottonleft[1], bottonleft[0]), 20, color);
	//circle(img, Point(bottonright[1], bottonright[0]), 20, color);

	//cout << Point(upperleft[0], upperleft[1]).x << " " << Point(upperleft[0], upperleft[1]).y;


	//imwrite("test.png", img);


	Range Rrow, Rcol;
	Rrow.start = (upperleft[0] > upperright[0]) ? upperleft[0] : upperright[0];
	Rrow.end = (bottonleft[0] < bottonright[0]) ? bottonleft[0] : bottonright[0];
	Rcol.start = (upperleft[1] > bottonleft[1]) ? upperleft[1] : bottonleft[1];
	Rcol.end = (upperright[1] < bottonright[1]) ? upperright[1] : bottonright[1];

	// 	Rrow.start = 0;
	// 	Rrow.end = img.rows;
	// 	Rcol.start = 0;
	// 	Rcol.end = img.cols;


	//cout << "Row Range: " << Rrow.start << " " << Rrow.end << "" << endl << "Col Range: " << Rcol.start << " " << Rcol.end << endl;

	Mat img1(img, Rrow, Rcol);
	imwrite(filename + ".png", img1);

	//system("pause");
}



void corp_all()
{
	 corp("G9PQ0282");  	corp("G9PQ0283"); 	corp("G9PQ0284"); 	corp("G9PQ0285"); 	corp("G9PQ0286"); 	corp("G9PQ0287"); 	corp("G9PQ0288"); corp("G9PQ0302"); corp("G9PQ0303"); corp("G9PQ0304"); corp("G9PQ0305"); corp("G9PQ0306"); corp("G9PQ0307"); corp("G9PQ0308"); corp("G9PQ0319"); corp("G9PQ0320"); corp("G9PQ0321"); corp("G9PQ0322"); corp("G9PQ0323"); corp("G9PQ0324"); corp("G9PQ0325"); corp("G9PQ0335"); corp("G9PQ0336"); corp("G9PQ0337"); corp("G9PQ0338"); corp("G9PQ0339"); corp("G9PQ0340"); corp("G9PQ0341"); corp("G9PQ0354"); corp("G9PQ0355"); corp("G9PQ0356"); corp("G9PQ0357"); corp("G9PQ0358"); corp("G9PQ0359"); corp("G9PQ0360"); corp("G9PQ0380"); corp("G9PQ0381"); corp("G9PQ0382"); corp("G9PQ0383"); corp("G9PQ0384"); corp("G9PQ0385"); corp("G9PQ0386"); corp("G9PQ0418"); corp("G9PQ0419"); corp("G9PQ0420"); corp("G9PQ0421"); corp("G9PQ0422"); corp("G9PQ0423"); corp("G9PQ0424"); corp("G9PQ0442"); corp("G9PQ0443"); corp("G9PQ0444"); corp("G9PQ0445"); corp("G9PQ0446"); corp("G9PQ0447"); corp("G9PQ0448");
}


void main()
{
	//Mat img=imread("D:\\G9PQ0282.jpg");
	//string a("G9PQ0282");
//	cout << a;
//	corp(a);
	corp_all();


}