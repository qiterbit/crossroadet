#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <in2_msgs/ScanInfoV2.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define   PI              3.14159265
#define   GRIDWIDTHL     -30
#define   GRIDWIDTHR      30  
#define   GRIDHEIGHT      40
#define   PRECISION       0.5
#define   INVPRECISION    2
#define   HALFPRECISION   PRECISION/2   
#define   GRIDSIZE        GRIDHEIGHT*(GRIDWIDTHR-GRIDWIDTHL)*INVPRECISION*INVPRECISION
#define   NUMOFPOINTS     360

using namespace cv;

ros::Subscriber  scan_sub;
Point _min_loc, _max_loc;
double _min, _max;
Mat meanfilter = Mat::ones(3, 3, CV_32FC1) / 9;

int my_callback(const in2_msgs::ScanInfoV2::ConstPtr &scaninfo_msg)
{
	//data receive 
	Mat src(GRIDHEIGHT*INVPRECISION, GRIDWIDTHR*2*INVPRECISION, CV_8UC1, Scalar::all(255));
	//image process 
	Mat result_image(GRIDHEIGHT*INVPRECISION, GRIDWIDTHR*2*INVPRECISION, CV_8UC1, Scalar::all(255));

	int indx, indy;
	double x, y;
	for (int i = 0; i < NUMOFPOINTS; i++)
	{
		//handle out points out of border 
		x = double(scaninfo_msg->ScanInfoX[i+NUMOFPOINTS] - 1500) * 0.02;
		y = double(scaninfo_msg->ScanInfoY[i+NUMOFPOINTS] - 1000) * 0.02;
		indx = (int)((GRIDHEIGHT - y - PRECISION) * INVPRECISION);
		indy = (int)((x + GRIDWIDTHR) * INVPRECISION);
		indx = indx > src.rows ? src.rows : indx;
		indx = indx < 0 ? 0 : indx;
		indy = indy > src.cols ? src.cols : indy;
		indy = indy < 0 ? 0 : indy;
		src.at<uchar>(indx, indy) = 0;
		//locate the search range of every line 
		double block_startx, block_starty, block_endx, block_endy;
		if (x < 0)
		{
			block_startx = GRIDWIDTHL;
			double nx = x - (int)floor(x), ny = y - (int)floor(y);
			for (int ind = INVPRECISION-1; ind >= 0; ind --)
			{
				if (ny >= (double)(1.0*ind*PRECISION))
				{
					block_starty = (int)floor(y) + ind*PRECISION;
					break;
				}
			}
			for (int ind = INVPRECISION-1; ind >= 0; ind --)
			{
				if (nx >= (double)(1.0*ind*PRECISION))
				{
					block_endx = (int)floor(x) + (ind+1)*PRECISION;
					break;
				}
			}
			block_endy = GRIDHEIGHT;
		}
		else
		{
			double nx = x - (int)floor(x), ny = y - (int)floor(y);
			for (int ind = INVPRECISION-1; ind >= 0; ind --)
			{
				if (ny >= (double)(1.0*ind*PRECISION))
				{
					block_starty = (int)floor(y) + ind*PRECISION;
					break;
				}
			}
			for (int ind = INVPRECISION-1; ind >= 0; ind --)
			{
				if (nx >= (double)(1.0*ind*PRECISION))
				{
					block_startx = (int)floor(x) + ind*PRECISION;
					break;
				}
			}
			block_endx = GRIDWIDTHR;
			block_endy = GRIDHEIGHT;
		}
		//set the obstacle point and unseen block to zero
		double angle = atan2(y, x) * 180 / PI;
		double block_angle_begin, block_angle_end;
		for (double bx = block_startx; bx < block_endx; bx += (double)PRECISION)
		{
			for (double by = block_starty; by < block_endy; by += (double)PRECISION)
			{		
				if (bx < 0)
				{
					block_angle_begin = atan2(by+PRECISION, bx+PRECISION) * 180.0 / PI;
					block_angle_end   = atan2(by,bx) * 180.0 / PI;
				}
				else
				{
					block_angle_begin = atan2(by, bx+PRECISION) * 180.0 / PI;
					block_angle_end   = atan2(by+PRECISION, bx) * 180.0 / PI;
				}

				if (block_angle_begin < angle && block_angle_end > angle)
				{
					result_image.at<uchar>((GRIDHEIGHT-by-PRECISION)*INVPRECISION, (bx+GRIDWIDTHR)*INVPRECISION) = 0;
				}
			}
		}
	}
	//src
	namedWindow("pointcloud2", 0);
	imshow("pointcloud2", src);

	//obstacle image 	
	namedWindow("obstacle", 0);
	imshow("obstacle", result_image);

	//distance image
	distanceTransform(result_image, result_image, CV_DIST_L2, 3);
	normalize(result_image, result_image, 0.0, 1.0, NORM_MINMAX);
	namedWindow("distance", 0);
	imshow("distance", result_image);

	//locate crossroad center
	filter2D(result_image, result_image, result_image.depth(), meanfilter);
	minMaxLoc(result_image, &_min, &_max, &_min_loc, &_max_loc);
	circle(result_image, _max_loc, 5, Scalar(0,100,255));
	namedWindow("finalConv", 0);
	imshow("finalConv", result_image);

	waitKey(1);
	return 0;
}

int main(int argc, char** argv)
{
    	ros::init(argc, argv, "distmap");
    	ros::NodeHandle n;
    	scan_sub = n.subscribe<in2_msgs::ScanInfoV2>("/ScanInfoV2", 500, my_callback);

	ros::spin();

	return 0;
}
