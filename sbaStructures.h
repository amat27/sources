#ifndef sbaStructures
#define sbaStructures

//#include "rotMatrixToQuternion.h"
#include <fstream>
#include <iostream>
#include <string>
#include "opencv2\stitching\detail\camera.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include <vector>
#include "modules/contrib/src/precomp.hpp"
#include "opencv2/calib3d/calib3d.hpp"


using namespace cv;
using namespace cv::detail;

enum ifExist
{
	BOTH_EXIST = 1,
	DST_EXISTS = 2,
	SRC_EXISTS = 3,
	NITHER_EXISTS = 4
};

class idx_target
{
public:
	idx_target(int i, Point3d p)
	{
		idx = i; target = p;
	}
	idx_target(){};

	int idx;
	Point3d target;
};

Point3d compute3Dpoints(Point2d origin, Mat cameraMatrix, Mat R, Mat T)
{
	
	//rodrigues

	
	cv::SVD svd;
	svd (R, SVD::FULL_UV);
	Mat tR = svd.u * svd.vt;
	if (determinant(R) < 0)
		tR *= -1;
	
	
	Mat rvec;
	Rodrigues(tR, rvec);
	CV_Assert(rvec.type() == CV_32F);
	
	Mat R1_;
	Rodrigues(rvec, R1_);
	
	/*cout << "R:" << endl;
	cout << R.at<double>(0, 0) << " " << R.at<double>(0, 1) << " " << R.at<double>(0, 2) << endl
		<< R.at<double>(1, 0) << " " << R.at<double>(1, 1) << " " << R.at<double>(1, 2) << endl
		<< R.at<double>(2, 0) << " " << R.at<double>(2, 1) << " " << R.at<double>(2, 2) << endl;

	cout << "R1_:" << endl;
	cout << R1_.at<double>(0, 0) << " " << R1_.at<double>(0, 1) << " " << R1_.at<double>(0, 2) << endl
		<< R1_.at<double>(1, 0) << " " << R1_.at<double>(1, 1) << " " << R1_.at<double>(1, 2) << endl
		<< R1_.at<double>(2, 0) << " " << R1_.at<double>(2, 1) << " " << R1_.at<double>(2, 2) << endl;*/
	
	
	
	
	Mat_<double> K_ = Mat_<double>(cameraMatrix).inv();
	Mat_<double> R_ = R1_;
	Mat_<double> T_ = T;
	
	double x = K_(0, 0)*origin.x + K_(0, 1)*origin.y + K_(0, 2) - T_(0,0);
	double y = K_(1, 0)*origin.x + K_(1, 1)*origin.y + K_(1, 2) - T_(1,0);
	double z = K_(2, 0)*origin.x + K_(2, 1)*origin.y + K_(2, 2) - T_(2,0);

	double x_ = R_(0, 0)*x + R_(0, 1)*y + R_(0, 2)*z;
	double y_ = R_(1, 0)*x + R_(1, 1)*y + R_(1, 2)*z;
	double z_ = R_(2, 0)*x + R_(2, 1)*y + R_(2, 2)*z;

	return Point3d(x_, y_, z_);
}


void readPointsInOnePic(const MatchesInfo matchesinfo, const vector<ImageFeatures> features, vector<vector<idx_target>>& mask, 
	vector<Point3d>& points, vector<vector<Point2d>>& imagepoints, vector<vector<int>>& visibility, int num_images, const vector<Mat>& cameraMatrix, const vector<Mat>& R, const vector<Mat>& T)

{
	cout << "Num of <Point3d> points: " << points.size() << endl;
	ifExist src_dst;
	int src = matchesinfo.src_img_idx;
	int dst = matchesinfo.dst_img_idx;
	//int num_matches = matchesinfo.matches.size();
	for (auto it = matchesinfo.matches.begin(); it != matchesinfo.matches.end(); ++it)
	{
		int src_idx = it->queryIdx;
		int dst_idx = it->trainIdx;
		bool src_exist = false;
		bool dst_exist = false;

		//check if queryIdx and trainIdx exist in mask
		if (!mask[src].empty())
		{
			for (auto idx_it = mask[src].begin(); idx_it != mask[src].end(); ++idx_it)
			{
				if (src_idx == idx_it->idx)
				{
					src_exist = true;
					break;
				}}}
		if (!mask[dst].empty())
		{
			for (auto idx_it = mask[dst].begin(); idx_it != mask[dst].end(); ++idx_it)
			{
				if (dst_idx == idx_it->idx)
				{
					dst_exist = true;
					break;
				}}}


		if (src_exist && dst_exist)			src_dst = BOTH_EXIST;
		else if (src_exist || dst_exist)
		{
			if (src_exist)			    	src_dst = SRC_EXISTS;
			else				            src_dst = DST_EXISTS;
		}
		else		                    	src_dst = NITHER_EXISTS;

		
		//temp vars
		Point3d tmp_target;
		int k = 0;

		//debug vars
		Point2d haha;
		
		switch (src_dst)
		{
		case NITHER_EXISTS:
		{
			Point2d guessFrom = features[src].keypoints[src_idx].pt;
			Point3d initialGuess = compute3Dpoints(guessFrom, cameraMatrix[src], R[src], T[src]);
			//TODO: compute point3d and push
			mask[src].push_back(idx_target(src_idx, initialGuess));
			mask[dst].push_back(idx_target(dst_idx, initialGuess));
			points.push_back(initialGuess);

			//push imagepoints
			vector<Point2d> proj_points;
			vector<int>     visib_points;
			for (int i = 0; i != num_images;++i)
			{
				if (i == src)
				{
					proj_points.push_back(features[src].keypoints[src_idx].pt);
					visib_points.push_back(1);
				}
				else if (i == dst)
				{
					proj_points.push_back(features[dst].keypoints[dst_idx].pt);
					visib_points.push_back(1);
				}
				else
				{
					proj_points.push_back(Point2d(-1, -1));
					visib_points.push_back(0);
				}
			}
			imagepoints.push_back(proj_points);
			visibility.push_back(visib_points);
		}
			break;
		case SRC_EXISTS:
			//Point3d tmp_target;
			//int k = 0;
			//for (auto it_target = mask[src].begin(); it_target != mask[src].end;++it_target)
			for (idx_target it_target : mask[src])
			{
				if (it_target.idx == src_idx)
				{
					tmp_target = it_target.target;
					break;
				}
			}
			mask[dst].push_back(idx_target(dst_idx, tmp_target));
			for (auto it = points.begin(); it != points.end();++it)
			{
				if (tmp_target.x == it->x && tmp_target.y == it->y && tmp_target.z == it->z)
					++k;
			}

			cout << k << " " << dst << " " << dst_idx << " " << endl;
			haha = features[dst].keypoints[dst_idx].pt;
			imagepoints[k][dst] = features[dst].keypoints[dst_idx].pt;
			visibility[k][dst] = 1;
			break; 
		case DST_EXISTS:
			//Point3d tmp_target;
			//int k = 0;
			for (idx_target it_target : mask[dst])
			{
				if (it_target.idx == dst_idx)
				{
					tmp_target = it_target.target;
					break;
				}
			}
			mask[src].push_back(idx_target(src_idx, tmp_target));
			for (auto it = points.begin(); it != points.end(); ++it)
			{
				if (tmp_target.x == it->x && tmp_target.y == it->y && tmp_target.z == it->z)
					++k;
			}
			imagepoints[k][src] = features[src].keypoints[src_idx].pt;
			visibility[k][src] = 1;
			break;
		case BOTH_EXIST:
			break;
		}


	}
	
			

}

void readCameraParams(CameraParams P, vector<Mat>& cameraMatrix, vector<Mat>& R, vector<Mat>& T, vector<Mat>& distCoeffs)
{
	Mat cameraMatrix_ = (Mat_<double>(3,3)<<
		            P.focal , 0 , P.ppx,
					0 , P.focal * P.aspect , P.ppy,
					0 , 0 , 1);
	Mat R_ = P.R;
	Mat T_ = P.t;
	
	distCoeffs.push_back(Mat_<double>(4, 1) << (0, 0, 0, 0));
	cameraMatrix.push_back(cameraMatrix_);
	R.push_back(R_);
	T.push_back(T_);
}


void readParams(const vector<ImageFeatures> features, const vector<MatchesInfo> pairwise_matches, const vector<CameraParams> cameras,
	vector<Point3d>& points, vector<vector<Point2d>>& imagepoints, vector<vector<int>>& visibility, int num_images,
	vector<Mat>& cameraMatrix, vector<Mat>& R, vector<Mat>& T, vector<Mat>& distCoeffs)
{
	//read camera params
	for (CameraParams Cp : cameras)
	{
		readCameraParams(Cp, cameraMatrix, R, T, distCoeffs);
	}

	//read points
	vector <vector<idx_target>> mask(num_images);
	for (MatchesInfo Mi : pairwise_matches)
	{
		readPointsInOnePic(Mi, features, mask, points, imagepoints, visibility, num_images, cameraMatrix, R, T);
	}
}


#endif 