#ifndef TJCAD
#define TJCAD

#include <vector>
#include "opencv2\stitching\detail\camera.hpp"
#include "opencv2\stitching\detail\matchers.hpp"
#include "iostream"


namespace TJCAD
{
	using namespace std;

	class match
	{
	public:
		int queryIdx = -1;//it should be the same as subscript of vector<matches> iff the match is valid
		int Pid = -1;//to which point the match is linking
		float x = -1, y = -1;

		match(){};
		match(cv::DMatch D); //do not update x,y. i.e. only Pid is really updated after this step.

	};

	//do not update x,y. i.e. only Pid is really updated after this step.
	match::match(cv::DMatch D)
	{
		queryIdx = D.queryIdx;
		Pid = D.trainIdx;

	}

	class keypoint
	{
	public:
		int Pid = -1;
		float x = 0, y = 0;
		vector<match> matches;
		bool is_inliar = false;
		float x3d = 0, y3d = 0, z3d = 0;

		keypoint();
		keypoint(int tpid, cv::KeyPoint K);//, cv::detail::MatchesInfo M);

	};




	class image
	{
	public:
		float focal = 0;                                                  //all zeros for params
		float ppx = 0, ppy = 0;
		float R[3][3];
		float t[3];
		vector<keypoint> keypoints;

		image();
		image(cv::detail::CameraParams P);
		image(cv::detail::CameraParams P, cv::detail::ImageFeatures I);   //under all circumstances this function should be prior uesd. it simultaneously initial image and keypoint parts

		void push_null_matches_to_keypoints();
	};

	void read_matches(vector<image> &images, vector<cv::detail::MatchesInfo> Ms, int num_images);
}

#endif // !TJCAD

