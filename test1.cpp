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

//headfile for using sba functions
//#include <sba.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "stitchingStructures.hpp"

//#include <sba.h>
//#include "compiler.h"
//#include "demo/eucsbademo.h"
//#include "demo/readparams.h"


using namespace std;
using namespace cv;
using namespace cv::detail;



static void printUsage()
{
	
}


// Default command line args
vector<string> img_names;
bool preview = false;
bool try_gpu = 0;
double work_megapix = 0.6;
double seam_megapix = 0.1;
double compose_megapix = -1;
float conf_thresh = 1.f;
string features_type = "surf";
string ba_cost_func = "reproj";
//Set refinement mask for bundle adjustment. It looks like 'x_xxx',
//where 'x' means refine respective parameter and '_' means don't
//refine one, and has the following format:\n"
//<fx><skew><ppx><aspect><ppy>. The default mask is 'xxxxx'. If bundle
//adjustment doesn't support estimation of selected parameter then
//the respective flag is ignored.
string ba_refine_mask = "x_x_x";
int baittimes = 200;
double bathresh = DBL_EPSILON; //should be DBL_EPSILON
bool do_wave_correct = 0;
WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;
bool save_graph = 0;
std::string save_graph_to = "matchp.txt";
string warp_type = "plane";
int expos_comp_type = ExposureCompensator::NO;
float match_conf = 0.65f;
string seam_find_type = "gc_color";
int blend_type = Blender::NO;
float blend_strength = 5;
string result_name = "result.jpg";
bool draw_matchs = 0;

ofstream writedown("output.txt");
set<pair<int, int> > span_tree_edges;

static int parseCmdArgs(int argc, char** argv)
{
	if (argc == 1)
	{
		//printUsage();
		return -1;
	}
	for (int i = 1; i < argc; ++i)
	{
		if (string(argv[i]) == "--help" || string(argv[i]) == "/?")
		{
			printUsage();
			return -1;
		}
		else if (string(argv[i]) == "--preview")
		{
			preview = true;
		}
		else if (string(argv[i]) == "--try_gpu")
		{
			if (string(argv[i + 1]) == "no")
				try_gpu = false;
			else if (string(argv[i + 1]) == "yes")
				try_gpu = true;
			else
			{
				cout << "Bad --try_gpu flag value\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--work_megapix")
		{
			work_megapix = atof(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--seam_megapix")
		{
			seam_megapix = atof(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--compose_megapix")
		{
			compose_megapix = atof(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--result")
		{
			result_name = argv[i + 1];
			i++;
		}
		else if (string(argv[i]) == "--features")
		{
			features_type = argv[i + 1];
			if (features_type == "orb")
				match_conf = 0.3f;
			i++;
		}
		else if (string(argv[i]) == "--match_conf")
		{
			match_conf = static_cast<float>(atof(argv[i + 1]));
			i++;
		}
		else if (string(argv[i]) == "--conf_thresh")
		{
			conf_thresh = static_cast<float>(atof(argv[i + 1]));
			i++;
		}
		else if (string(argv[i]) == "--ba")
		{
			ba_cost_func = argv[i + 1];
			i++;
		}
		else if (string(argv[i]) == "--ba_refine_mask")
		{
			ba_refine_mask = argv[i + 1];
			if (ba_refine_mask.size() != 5)
			{
				cout << "Incorrect refinement mask length.\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--wave_correct")
		{
			if (string(argv[i + 1]) == "no")
				do_wave_correct = false;
			else if (string(argv[i + 1]) == "horiz")
			{
				do_wave_correct = true;
				wave_correct = detail::WAVE_CORRECT_HORIZ;
			}
			else if (string(argv[i + 1]) == "vert")
			{
				do_wave_correct = true;
				wave_correct = detail::WAVE_CORRECT_VERT;
			}
			else
			{
				cout << "Bad --wave_correct flag value\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--save_graph")
		{
			save_graph = true;
			save_graph_to = argv[i + 1];
			i++;
		}
		else if (string(argv[i]) == "--warp")
		{
			warp_type = string(argv[i + 1]);
			i++;
		}
		else if (string(argv[i]) == "--expos_comp")
		{
			if (string(argv[i + 1]) == "no")
				expos_comp_type = ExposureCompensator::NO;
			else if (string(argv[i + 1]) == "gain")
				expos_comp_type = ExposureCompensator::GAIN;
			else if (string(argv[i + 1]) == "gain_blocks")
				expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
			else
			{
				cout << "Bad exposure compensation method\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--seam")
		{
			if (string(argv[i + 1]) == "no" ||
				string(argv[i + 1]) == "voronoi" ||
				string(argv[i + 1]) == "gc_color" ||
				string(argv[i + 1]) == "gc_colorgrad" ||
				string(argv[i + 1]) == "dp_color" ||
				string(argv[i + 1]) == "dp_colorgrad")
				seam_find_type = argv[i + 1];
			else
			{
				cout << "Bad seam finding method\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--blend")
		{
			if (string(argv[i + 1]) == "no")
				blend_type = Blender::NO;
			else if (string(argv[i + 1]) == "feather")
				blend_type = Blender::FEATHER;
			else if (string(argv[i + 1]) == "multiband")
				blend_type = Blender::MULTI_BAND;
			else
			{
				cout << "Bad blending method\n";
				return -1;
			}
			i++;
		}
		else if (string(argv[i]) == "--blend_strength")
		{
			blend_strength = static_cast<float>(atof(argv[i + 1]));
			i++;
		}
		else if (string(argv[i]) == "--output")
		{
			result_name = argv[i + 1];
			i++;
		}
		else
			img_names.push_back(argv[i]);
	}
	if (preview)
	{
		compose_megapix = 0.6;
	}
	return 0;
}


int main(int argc, char* argv[])
{
#if ENABLE_LOG
	int64 app_start_time = getTickCount();
#endif

	cv::setBreakOnError(true);

	int retval = parseCmdArgs(argc, argv);
	if (retval)
		return retval;

	// Check if have enough images
	int num_images = static_cast<int>(img_names.size());
	if (num_images < 2)
	{
		LOGLN("Need more images");
		writedown << "Need more images" << endl;
		return -1;
	}

	double work_scale = 1, seam_scale = 1, compose_scale = 1;
	bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;

	LOGLN("Finding features...");
	writedown << "Finding features..." << endl;
#if ENABLE_LOG
	int64 t = getTickCount();
#endif

	Ptr<FeaturesFinder> finder;
	if (features_type == "surf")
	{
#if defined(HAVE_OPENCV_NONFREE) && defined(HAVE_OPENCV_GPU) && !defined(ANDROID)
		if (try_gpu && gpu::getCudaEnabledDeviceCount() > 0)
			finder = new SurfFeaturesFinderGpu();
		else
#endif
			finder = new SurfFeaturesFinder();
	}
	else if (features_type == "orb")
	{
		finder = new OrbFeaturesFinder();
	}
	else
	{
		cout << "Unknown 2D features type: '" << features_type << "'.\n";
		return -1;
	}

	Mat full_img, img;
	vector<ImageFeatures> features(num_images);
	vector<Mat> images(num_images);
	vector<Size> full_img_sizes(num_images);
	//vector<Mat> need4drawmatch; //full size img 
	double seam_work_aspect = 1;

	for (int i = 0; i < num_images; ++i)
	{
		full_img = imread(img_names[i]);
		full_img_sizes[i] = full_img.size();

		if (full_img.empty())
		{
			LOGLN("Can't open image " << img_names[i]);
			writedown << "Can't open image " << img_names[i] << endl;
			return -1;
		}
		if (work_megapix < 0)
		{
			img = full_img;
			work_scale = 1;
			is_work_scale_set = true;
		}
		else
		{
			if (!is_work_scale_set)
			{
				work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
				is_work_scale_set = true;
			}
			resize(full_img, img, Size(), work_scale, work_scale);
		}
		if (!is_seam_scale_set)
		{
			seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
			seam_work_aspect = seam_scale / work_scale;
			is_seam_scale_set = true;
		}
		//need4drawmatch.push_back(img); //saving full size imgs
		(*finder)(img, features[i]);
		features[i].img_idx = i;
		LOGLN("Features in image #" << i + 1 << ": " << features[i].keypoints.size());
		writedown << "Features in image #" << i + 1 << ": " << features[i].keypoints.size() << endl;

		resize(full_img, img, Size(), seam_scale, seam_scale);
		images[i] = img.clone();
	}

	finder->collectGarbage();


	full_img.release();
	img.release();

	LOGLN("Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
	writedown << "Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;

	LOG("Pairwise matching");
#if ENABLE_LOG
	t = getTickCount();
#endif
	vector<MatchesInfo> pairwise_matches;
	BestOf2NearestMatcher matcher(try_gpu, match_conf);
	matcher(features, pairwise_matches);
	matcher.collectGarbage();
	LOGLN("Pairwise matching, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
	writedown << "Pairwise matching, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << endl;



	//=========================================================================================================
	//above is the sequence of first matching

	// Check if we should save matches graph


	LOGLN("Saving matches graph...");
	writedown << "Saving matches graph..." << endl;
	// 	if (0)
	// 	{
	// 		ofstream f(save_graph_to.c_str());
	// 		f << matchesGraphAsStringPowerUpVer(img_names, pairwise_matches, conf_thresh, span_tree_edges);
	// 	}

	//draw mathes and push numpts3D
	int numpts3D = 0;/* number of points */
	ofstream writeHomography("homographyRecord.txt");
	int id_of_pair = 0;
	// 	for (set<pair<int, int>>::iterator edgesit = span_tree_edges.begin(); edgesit != span_tree_edges.end(); ++edgesit)
	// 	{
	// 		++id_of_pair;
	// 		int i = (*edgesit).first;
	// 		int j = (*edgesit).second;
	// 		int k = i*num_images + j;
	// 		numpts3D += pairwise_matches[k].num_inliers;
	// 		cout << numpts3D << " ";
	// 		if (draw_matchs){
	// 			stringstream s;
	// 			s << id_of_pair;
	// 			string str = s.str();
	// 			Mat comeonbaby;
	// 			vector<char> maskwithsign;
	// 			for (vector<uchar>::iterator it = pairwise_matches[k].inliers_mask.begin(); it != pairwise_matches[k].inliers_mask.end() && !isspace(*it); ++it)
	// 				maskwithsign.push_back(*it);
	// 			drawMatches(need4drawmatch[i], features[i].keypoints, need4drawmatch[j], features[j].keypoints, pairwise_matches[k].matches, comeonbaby, Scalar(0, 255, 0), Scalar(255, 0, 0), maskwithsign);
	// 			imwrite("match" + str + ".jpg", comeonbaby);
	// 			Mat showHomography(pairwise_matches[k].H);
	// 			writeHomography << "Homography matrix which transfers pic " << i << " to pic " << j << " is:" << endl;
	// 			for (int a = 0; a != 3; ++a)
	// 			{
	// 				for (int b = 0; b != 3; ++b)
	// 				{
	// 					double fairlybaby = showHomography.at<double>(a, b);
	// 					writeHomography << fairlybaby << " ";
	// 				}
	// 				writeHomography << endl;
	// 			}
	// 		}
	//	}


	// Leave only images we are sure are from the same panorama
	vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
	vector<Mat> img_subset;
	vector<string> img_names_subset;
	vector<Size> full_img_sizes_subset;
	for (size_t i = 0; i < indices.size(); ++i)
	{
		img_names_subset.push_back(img_names[indices[i]]);
		img_subset.push_back(images[indices[i]]);
		full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
	}

	images = img_subset;
	img_names = img_names_subset;
	full_img_sizes = full_img_sizes_subset;



	// Check if we still have enough images
	num_images = static_cast<int>(img_names.size());
	if (num_images < 2)
	{
		LOGLN("Need more images");
		writedown << "Need more images" << endl;
		return -1;
	}

	HomographyBasedEstimator estimator;
	vector<CameraParams> cameras;
	estimator(features, pairwise_matches, cameras);

	for (size_t i = 0; i < cameras.size(); ++i)
	{
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R;
		LOGLN("Initial intrinsics #" << indices[i] + 1 << ":\n" << cameras[i].K());
		//writedown << "Initial intrinsics #" << indices[i] + 1 << ":\n" << cameras[i].K() << endl;
	}

	vector < TJCAD::image > image_data;
	
	for (int n = 0; n != 3; ++n)
	{
		image_data.push_back(TJCAD::image(cameras[n], features[n]));
	}

	TJCAD::read_matches(image_data, pairwise_matches, num_images);

	int a;
	cin >> a;

	//=======================================================================
	//to here is all the procedure needed before running bundle adjustment

	//here comes the first data tansfer






}