#include <fstream>
#include <iostream>
#include <string>
#include "rotMatrixToQuternion.h"
#include "opencv2\stitching\detail\camera.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include <vector>

using namespace std;

void writePoints(string writeName, vector<cv::Point3d> Points)
{
	int i = 0;
	ofstream writePoints(writeName);
	for (cv::Point3d pt:Points)
	{
		++i;
		writePoints <<"Point No. "<<i<< " = ( " << pt.x << " , " << pt.y << " , " << pt.z << " )" << endl;

	}
}


void writeCamParams(string writeName, int num_images, vector<cv::detail::CameraParams> cameras)
{
	ofstream writeCamParams(writeName);
	writeCamParams<<"# fu, u0, v0, ar, s   quaternion translation"<<endl;
	for (int i=0;i<num_images;++i)
	{
		int aspect=1;
		writeCamParams<<cameras[i].focal<<" "<<cameras[i].ppx<<" "<<cameras[i].ppy<<" "<<aspect<<" 0 ";
		
		//write rotations
		rotMatrix rotationMat; 
		rotationMat.m11=cameras[i].R.at<double>(0,0);
		rotationMat.m12=cameras[i].R.at<double>(0,1);
		rotationMat.m13=cameras[i].R.at<double>(0,2);
		rotationMat.m21=cameras[i].R.at<double>(1,0);
		rotationMat.m22=cameras[i].R.at<double>(1,1);
		rotationMat.m23=cameras[i].R.at<double>(1,2);
		rotationMat.m31=cameras[i].R.at<double>(2,0);
		rotationMat.m32=cameras[i].R.at<double>(2,1);
		rotationMat.m33=cameras[i].R.at<double>(2,2);
		Quaternion rotationQua;
		rotMatrixToQuternion(rotationQua,rotationMat);
		writeCamParams<<rotationQua.x<<" "<<rotationQua.y<<" "<<rotationQua.z<<" "<<rotationQua.w<<" ";

		//write translation
		writeCamParams<<cameras[i].t.at<double>(0)<<" "<<cameras[i].t.at<double>(1)<<" "<<cameras[i].t.at<double>(2)<<endl;
	}
}

using namespace cv::detail;
//record matches graph
string matchesGraphAsStringPowerUpVer(vector<string> &pathes, vector<cv::detail::MatchesInfo> &pairwise_matches,
                                float conf_threshold,set<pair<int,int> > &span_tree_edges)
{
	stringstream str;
    str << "graph matches_graph{\n";

    const int num_images = static_cast<int>(pathes.size());
    
    DisjointSets comps(num_images);

	    for (int i = 0; i < num_images; ++i)
    {
        for (int j = i; j < num_images; ++j)
        {
            if (pairwise_matches[i*num_images + j].confidence < conf_threshold)
                continue;
            int comp1 = comps.findSetByElem(i);
            int comp2 = comps.findSetByElem(j);
            //if (comp1 != comp2)
            {
                comps.mergeSets(comp1, comp2);
                span_tree_edges.insert(make_pair(i, j));
            }
			        }
    }
	    
		int id_of_pair=0;
		for (set<pair<int,int> >::const_iterator itr = span_tree_edges.begin();
         itr != span_tree_edges.end(); ++itr)
    {
		
		++id_of_pair;
        pair<int,int> edge = *itr;
        if (span_tree_edges.find(edge) != span_tree_edges.end())
        {
            string name_src = pathes[edge.first];
            size_t prefix_len = name_src.find_last_of("/\\");
            if (prefix_len != string::npos) prefix_len++; else prefix_len = 0;
            name_src = name_src.substr(prefix_len, name_src.size() - prefix_len);

            string name_dst = pathes[edge.second];
            prefix_len = name_dst.find_last_of("/\\");
            if (prefix_len != string::npos) prefix_len++; else prefix_len = 0;
            name_dst = name_dst.substr(prefix_len, name_dst.size() - prefix_len);

            int pos = edge.first*num_images + edge.second;
			str <<"No.:"<<id_of_pair
				<< "\"" << name_src << "\" -- \"" << name_dst << "\""
                << "[label=\"Nm=" << pairwise_matches[pos].matches.size()
                << ", Ni=" << pairwise_matches[pos].num_inliers
                << ", C=" << pairwise_matches[pos].confidence << "\"];\n";
			}
    }

    for (size_t i = 0; i < comps.size.size(); ++i)
    {
        if (comps.size[comps.findSetByElem((int)i)] == 1)
        {
            string name = pathes[i];
            size_t prefix_len = name.find_last_of("/\\");
            if (prefix_len != string::npos) prefix_len++; else prefix_len = 0;
            name = name.substr(prefix_len, name.size() - prefix_len);
            str << "\"" << name << "\";\n";
        }
    }

    str << "}";
    return str.str();
}