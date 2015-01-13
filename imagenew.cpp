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
#include<iostream>

using namespace cv;
using namespace std;

struct point
{
	int x;
	int y;
};

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		printf("useage: %s <imagefile>\n ", argv[0]);
		return -1;
	}
	string imageName = argv[1];

	Mat image;
	image = imread(imageName, CV_LOAD_IMAGE_COLOR);

	if (!image.data)
	{
		printf(" No image data \n ");
		return -1;
	}

	
	int row, col;
	int xleast, yleast, xlargest, ylargest;
	//row-least
	for (row = 0; row < image.rows; row++)
	{
		for (col = 0; col < image.cols; col++)
		{
			if (image.at<Vec3b>(row, col)[0]>10 || image.at<Vec3b>(row, col)[1]>10 || image.at<Vec3b>(row, col)[2] > 10)
			{
				yleast = row;
				break;
			}
		}
		if (col < image.cols)
			break;
	}
	//col-least
	for (col = 0; col< image.cols; col++)
	{
		for (row = 0; row < image.rows; row++)
		{
			if (image.at<Vec3b>(row, col)[0]>10 || image.at<Vec3b>(row, col)[1]>10 || image.at<Vec3b>(row, col)[2] > 10)
			{
				xleast = col;
				break;
			}
		}
		if (row < image.rows)
			break;
	}
	//row-largest
	for (row = image.rows-1; row >0; row--)
	{
		for (col = 0; col < image.cols; col++)
		{
			if (image.at<Vec3b>(row, col)[0]>10 || image.at<Vec3b>(row, col)[1]>10 || image.at<Vec3b>(row, col)[2] > 10)
			{
				ylargest = row;
				break;
			}
		}
		if (col < image.cols)
			break;
	}
	//col-largest
	for (col = image.cols-1; col>0; col--)
	{
		for (row = 0; row < image.rows; row++)
		{
			if (image.at<Vec3b>(row, col)[0]>10 || image.at<Vec3b>(row, col)[1]>10 || image.at<Vec3b>(row, col)[2] > 10)
			{
				xlargest = col;
				break;
			}
		}
		if (row < image.rows)
			break;
	}

	point p[5];
	p[0].x = (xleast + xlargest) / 2;
	p[0].y = (yleast + ylargest) / 2;

	cout << xleast << "," << xlargest << endl;
	cout << yleast << "," << ylargest << endl;
	cout << "p[0]:" << p[0].x << "," << p[0].y << endl;

	int area = 0;
	//upper left corner
	for (row = yleast; row < p[0].y; row++)
	{
		uchar* data = image.ptr<uchar>(row);
		int x1, x2, y1, y2;
		int i, j;
		//p(x1,y1)
		for (col = xleast; col < p[0].x; col++)
		{
			if (data[col * 3]>10 || data[col * 3 + 1]>10 || data[col * 3 + 2] > 10)
			{
				x1 = col;
				y1 = row;
				break;
			}
		}
		if (col == p[0].x)
			continue;//
		//p(x2,y1)
		for (i = xlargest; i> p[0].x; i--)
		{
			if (data[i * 3]>10 || data[i * 3 + 1]>10 || data[i * 3 + 2] > 10)
			{
				x2 = i;
				break;
			}
		}
		if (i == p[0].x)
			continue;//
		//p(x1,y2)
		for (j = ylargest; j > p[0].y; j--)
		{
			if (image.at<Vec3b>(j, x1)[0] > 10 || image.at<Vec3b>(j, x1)[1]>10 || image.at<Vec3b>(j, x1)[2]>10)
			{
				y2 = j;
				break;
			}
		}
		if (j == p[0].y)
			continue;//

		if (image.at<Vec3b>(y2, x2)[0] > 10 || image.at<Vec3b>(y2, x2)[1] > 10 || image.at<Vec3b>(y2, x2)[2] > 10)
		{
			int area1 = (y2 - y1)*(x2 - x1);
			if (area1 > area)
			{
				area = area1;
				p[1].x = x1; p[1].y = y1;
				p[2].x = x2; p[2].y = y1;
				p[3].x = x1; p[3].y = y2;
				p[4].x = x2; p[4].y = y2;
			}
		}
	}
	cout <<"1: "<< area << endl;
	cout << "p[1]:" << p[1].x << "," << p[1].y << endl;
	cout << "p[2]:" << p[2].x << "," << p[2].y << endl;
	cout << "p[3]:" << p[3].x << "," << p[3].y << endl;
	cout << "p[4]:" << p[4].x << "," << p[4].y << endl;

	//upper right corner
	for (row = yleast; row < p[0].y; row++)
	{
		uchar* data = image.ptr<uchar>(row);
		int x1, x2, y1, y2;
		int i, j;
		//p(x2,y1)
		for (col = xlargest; col > p[0].x; col--)
		{
			if (data[col * 3]>10 || data[col * 3 + 1]>10 || data[col * 3 + 2] > 10)
			{
				x2 = col;
				y1 = row;
				break;
			}
		}
		if (col == p[0].x)
			continue;//
		//p(x1,y1)
		for (i = xleast; i< p[0].x; i++)
		{
			if (data[i * 3]>10 || data[i * 3 + 1]>10 || data[i * 3 + 2] > 10)
			{
				x1 = i;
				break;
			}
		}
		if (i == p[0].x)
			continue;//
		//p(x2,y2)
		for (j = ylargest; j > p[0].y; j--)
		{
			if (image.at<Vec3b>(j, x2)[0] > 10 || image.at<Vec3b>(j, x2)[1]>10 || image.at<Vec3b>(j, x2)[2]>10)
			{
				y2 = j;
				break;
			}
		}
		if (j == p[0].y)
			continue;//

		if (image.at<Vec3b>(y2, x1)[0] > 10 || image.at<Vec3b>(y2, x1)[1] > 10 || image.at<Vec3b>(y2, x1)[2] > 10)
		{
			int area1 = (y2 - y1)*(x2 - x1);
			if (area1 > area)
			{
				area = area1;
				p[1].x = x1; p[1].y = y1;
				p[2].x = x2; p[2].y = y1;
				p[3].x = x1; p[3].y = y2;
				p[4].x = x2; p[4].y = y2;
			}
		}
	}
	cout << "2: " << area << endl;
	cout << "p[1]:" << p[1].x << "," << p[1].y << endl;
	cout << "p[2]:" << p[2].x << "," << p[2].y << endl;
	cout << "p[3]:" << p[3].x << "," << p[3].y << endl;
	cout << "p[4]:" << p[4].x << "," << p[4].y << endl;

	//lower left corner
	for (row = ylargest; row >p[0].y; row--)
	{
		uchar* data = image.ptr<uchar>(row);
		int x1, x2, y1, y2;
		int i, j;
		//p(x1,y2)
		for (col = xleast; col < p[0].x; col++)
		{
			if (data[col * 3]>10 || data[col * 3 + 1]>10 || data[col * 3 + 2] > 10)
			{
				x1 = col;
				y2 = row;
				break;
			}
		}
		if (col == p[0].x)
			continue;//
		//p(x2,y2)
		for (i = xlargest; i> p[0].x; i--)
		{
			if (data[i * 3]>10 || data[i * 3 + 1]>10 || data[i * 3 + 2] > 10)
			{
				x2 = i;
				break;
			}
		}
		if (i == p[0].x)
			continue;//
		//p(x1,y1)
		for (j = yleast; j <p[0].y; j++)
		{
			if (image.at<Vec3b>(j, x1)[0] > 10 || image.at<Vec3b>(j, x1)[1]>10 || image.at<Vec3b>(j, x1)[2]>10)
			{
				y1 = j;
				break;
			}
		}
		if (j == p[0].y)
			continue;//

		if (image.at<Vec3b>(y1, x2)[0] > 10 || image.at<Vec3b>(y1, x2)[1] > 10 || image.at<Vec3b>(y1, x2)[2] > 10)
		{
			int area1 = (y2 - y1)*(x2 - x1);
			if (area1 > area)
			{
				area = area1;
				p[1].x = x1; p[1].y = y1;
				p[2].x = x2; p[2].y = y1;
				p[3].x = x1; p[3].y = y2;
				p[4].x = x2; p[4].y = y2;
			}
		}
	}
	cout << "3: " << area << endl;
	cout << "p[1]:" << p[1].x << "," << p[1].y << endl;
	cout << "p[2]:" << p[2].x << "," << p[2].y << endl;
	cout << "p[3]:" << p[3].x << "," << p[3].y << endl;
	cout << "p[4]:" << p[4].x << "," << p[4].y << endl;

	//lower right corner
	for (row = ylargest; row >p[0].y; row--)
	{
		uchar* data = image.ptr<uchar>(row);
		int x1, x2, y1, y2;
		int i, j;
		//p(x2,y2)
		for (col = xlargest; col >p[0].x; col--)
		{
			if (data[col * 3]>10 || data[col * 3 + 1]>10 || data[col * 3 + 2] > 10)
			{
				x2 = col;
				y2 = row;
				break;
			}
		}
		if (col == p[0].x)
			continue;//
		//p(x1,y2)
		for (i = xleast; i<p[0].x; i++)
		{
			if (data[i * 3]>10 || data[i * 3 + 1]>10 || data[i * 3 + 2] > 10)
			{
				x1 = i;
				break;
			}
		}
		if (i == p[0].x)
			continue;//
		//p(x2,y1)
		for (j = yleast; j <p[0].y; j++)
		{
			if (image.at<Vec3b>(j, x2)[0] > 10 || image.at<Vec3b>(j, x2)[1]>10 || image.at<Vec3b>(j, x2)[2]>10)
			{
				y1 = j;
				break;
			}
		}
		if (j == p[0].y)
			continue;//

		if (image.at<Vec3b>(y1, x1)[0] > 10 || image.at<Vec3b>(y1, x1)[1] > 10 || image.at<Vec3b>(y1, x1)[2] > 10)
		{
			int area1 = (y2 - y1)*(x2 - x1);
			if (area1 > area)
			{
				area = area1;
				p[1].x = x1; p[1].y = y1;
				p[2].x = x2; p[2].y = y1;
				p[3].x = x1; p[3].y = y2;
				p[4].x = x2; p[4].y = y2;
			}
		}
	}
	cout << "4: " << area << endl;
	cout << "p[1]:" << p[1].x << "," << p[1].y << endl;
	cout << "p[2]:" << p[2].x << "," << p[2].y << endl;
	cout << "p[3]:" << p[3].x << "," << p[3].y << endl;
	cout << "p[4]:" << p[4].x << "," << p[4].y << endl;

	///////////////////////////////////////////////////////////12.11
	int row1, row2, col1, col2;
	for (row = p[1].y; row < p[3].y; row++)
	{
		uchar* data = image.ptr<uchar>(row);
		
		for (col = p[1].x; col < p[2].x; col++)
		{
			if (data[col * 3]<=10 && data[col * 3 + 1]<=10 && data[col * 3 + 2] <= 10)
			{
				break;
			}
		}
		if (col == p[2].x)
			break;//²»°¼
		else
			continue;//°¼		
	}
	row1 = row;
	for (row = p[3].y; row>p[1].y; row--)
	{
		uchar* data = image.ptr<uchar>(row);

		for (col = p[3].x; col < p[4].x; col++)
		{
			if (data[col * 3] <= 10 && data[col * 3 + 1] <= 10 && data[col * 3 + 2] <= 10)
			{
				break;
			}
		}
		if (col == p[4].x)
			break;//²»°¼
		else
			continue;//°¼		
	}
	row2 = row;
	for (col = p[1].x; col< p[2].x; col++)
	{
		for (row = row1; row < row2; row++)
		{
			if (image.at<Vec3b>(row, col)[0]<=10 && image.at<Vec3b>(row, col)[1]<=10 && image.at<Vec3b>(row, col)[2]<=10)
			{
				break;
			}
		}
		if (row == row2)
			break;//²»°¼
		else
			continue;//°¼
	}
	col1 = col;
	for (col = p[2].x; col>p[1].x; col--)
	{
		for (row = row1; row < row2; row++)
		{
			if (image.at<Vec3b>(row, col)[0] <= 10 && image.at<Vec3b>(row, col)[1] <= 10 && image.at<Vec3b>(row, col)[2] <= 10)
			{
				break;
			}
		}
		if (row == row2)
			break;//²»°¼
		else
			continue;//°¼
	}
	col2 = col;
	/////////////////////////////////////////////////////////////////////12.11
	cout << "row1:" << row1 << "; row2:" << row2 << "; col1:" << col1 << "; col2:" << col2 << endl;
	p[1].x = col1; p[1].y = row1;
	p[2].x = col2; p[2].y = row1;
	p[3].x = col1; p[3].y = row2;
	p[4].x = col2; p[4].y = row2;
	area = (row2 - row1)*(col2 - col1);
	cout << "area:" << area << endl;
	Rect rect(p[1].x, p[1].y, p[2].x - p[1].x, p[3].y - p[1].y);
	Mat newimage;
	newimage = image(rect);
	imwrite(imageName + ".png", newimage);
	//imshow("newimage", newimage);
	waitKey(0);

	return 0;
}