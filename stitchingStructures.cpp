#include "stitchingStructures.hpp"

keypoint::keypoint(int tpid, cv::KeyPoint K)//, cv::detail::MatchesInfo M)
{
	Pid = tpid;
	x = K.pt.x;
	y = K.pt.y;

	// 		for (auto iterator_matches : M.matches){
	// 			if (iterator_matches.queryIdx == Pid){
	// 				matches.push_back(match(iterator_matches));
	// 				return;
	// 			} 
	// 			else{
	// 				matches.push_back(match());
	// 			}			
	// 		}
}	image::image(cv::detail::CameraParams P)
{
	focal = P.focal;
	ppx = P.ppx;
	ppy = P.ppy;
	R[0][0] = P.R.at<double>(0, 0); R[0][1] = P.R.at<double>(0, 1); R[0][2] = P.R.at<double>(0, 2);
	R[1][0] = P.R.at<double>(1, 0); R[1][1] = P.R.at<double>(1, 1); R[1][2] = P.R.at<double>(1, 2);
	R[2][0] = P.R.at<double>(2, 0); R[2][1] = P.R.at<double>(2, 1); R[2][2] = P.R.at<double>(2, 2);
	t[0] = P.t.at<double>(0); t[1] = P.t.at<double>(1); t[2] = P.t.at<double>(2);
}


image::image(cv::detail::CameraParams P, cv::detail::ImageFeatures I)
{
	focal = P.focal;
	ppx = P.ppx;
	ppy = P.ppy;
	R[0][0] = P.R.at<double>(0, 0); R[0][1] = P.R.at<double>(0, 1); R[0][2] = P.R.at<double>(0, 2);
	R[1][0] = P.R.at<double>(1, 0); R[1][1] = P.R.at<double>(1, 1); R[1][2] = P.R.at<double>(1, 2);
	R[2][0] = P.R.at<double>(2, 0); R[2][1] = P.R.at<double>(2, 1); R[2][2] = P.R.at<double>(2, 2);
	t[0] = P.t.at<double>(0); t[1] = P.t.at<double>(1); t[2] = P.t.at<double>(2);

	int tpid = -1;
	for (auto f : I.keypoints){
		++tpid;
		keypoints.push_back(keypoint(tpid, f));
	}
}

inline void image::push_null_matches_to_keypoints()
{
	for (keypoint &tp : keypoints){
		tp.matches.push_back(match());
	}
}

//eye for rotation matrix, all zeros for trans
image::image()
{
	R[0][0] = 1; R[0][1] = 0; R[0][2] = 0;
	R[1][0] = 0; R[1][1] = 1; R[1][2] = 0;
	R[2][0] = 0; R[2][1] = 0; R[2][2] = 1;
	t[0] = 0; t[1] = 0; t[2] = 0;
}


void read_matches(vector<image> &images, vector<cv::detail::MatchesInfo> Ms, int num_images)
{
	for (int a = 0; a != num_images; ++a){
		for (int b = 0; b != num_images; ++b){
			cout << a << " " << b;
			if (a == b){
				images[a].push_null_matches_to_keypoints();
				// 					for (keypoint &tp : images[a].keypoints){
				// 						tp.matches.push_back(match());
				// 					}
				continue;
			}
			//int n = num_images * a + b;
			for (auto &tp : Ms[num_images * a + b].matches){
				images[a].keypoints[tp.queryIdx].matches.push_back(match(tp));
			}
			for (auto &tp : images[a].keypoints){
				//cout << tp.matches.size() << endl;
				//cout << a << " " << b << endl;
				//debug
				if (tp.matches.size() != b + 1){
					tp.matches.push_back(match());
				}
				else{
					//cout << a << " " << b << endl;
					//cout << tp.matches[b].Pid << " " << tp.matches[b].queryIdx << " " << tp.matches[b].x << "," << tp.matches[b].y << endl;
					//debug
					tp.matches[b].x = images[b].keypoints[tp.Pid].x;
					tp.matches[b].y = images[b].keypoints[tp.Pid].y;
				}
			}
		}
	}
}
