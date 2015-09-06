#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <in2_msgs/ScanInfoV2.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ros::Subscriber  scan_sub;

#define PI 3.14159265

//width and height should be even and symmetry
//precision must be integer divided by GRID**
//3000 coordinate system
#define   GRIDWIDTHL     -30
#define   GRIDWIDTHR      30  
#define   GRIDHEIGHT      40
#define   PRECISION       0.5
#define   INVPRECISION    2
#define   HALFPRECISION   PRECISION/2   
#define   GRIDSIZE        GRIDHEIGHT*(GRIDWIDTHR-GRIDWIDTHL)*INVPRECISION*INVPRECISION

cv::Mat meanfilter = cv::Mat::ones(3, 3, CV_32FC1) / 9;
int my_callback(const in2_msgs::ScanInfoV2::ConstPtr &scaninfo_msg)
{
	//data receive 
	cv::Mat src = cv::Mat(GRIDHEIGHT*INVPRECISION, GRIDWIDTHR*2*INVPRECISION, CV_8UC1, cv::Scalar::all(255));
	for (int i = 360; i < 720; i++)
	{
		double bx, by;
		bx = (double)(scaninfo_msg->ScanInfoX[i] - 1500) * 0.02;
		by = (double)(scaninfo_msg->ScanInfoY[i] - 1000) * 0.02;
		//if (bx >= GRIDWIDTHR || bx <= GRIDWIDTHL || by >= GRIDHEIGHT || by <= -1*GRIDHEIGHT)
		//{
		//	bx = 5.0 * cos(atan2(by+0.00001, bx+0.00001)); 
		//	by = 5.0 * sin(atan2(by+0.00001, bx+0.00001));
		//}
		int indx = (int)((GRIDHEIGHT - by - PRECISION) * INVPRECISION);
		int indy = (int)((bx + GRIDWIDTHR) * INVPRECISION);
		src.at<uchar>(indx, indy) = 0;
	}
	cv::namedWindow("pointcloud2", 0);
	cv::imshow("pointcloud2", src);
	
	int num = 360;
	printf("sendTime: %d\n", scaninfo_msg->sendtime);	
	printf("\n");

	//image process 
	cv::Mat result_image(GRIDHEIGHT*INVPRECISION, GRIDWIDTHR*2*INVPRECISION, CV_8UC1, cv::Scalar::all(255));
	for (int i = 0; i < num; i++)
	{
		//handle out points out of border 
		float x, y;
		x = double(scaninfo_msg->ScanInfoX[i+360] - 1500) * 0.02;
		y = double(scaninfo_msg->ScanInfoY[i+360] - 1000) * 0.02;
		/*if (x >= GRIDWIDTHR || x <= GRIDWIDTHL || y >= GRIDHEIGHT || y <= -1*GRIDHEIGHT)
		{
			x = 5.0 * cos(atan2(y+0.00001, x+0.00001)); 
			y = 5.0 * sin(atan2(y+0.00001, x+0.00001));
		}*/
		//locate the search range of every line 
		double block_startx, block_starty, block_endx, block_endy;
		if (x < 0)
		{
			block_startx = GRIDWIDTHL;
			double nx = x - (int)floor(x), ny = y - (int)floor(y);
			for (int ind = INVPRECISION-1; ind >= 0; ind --)
			{
				if (ny >= ind*PRECISION)
				{
					block_starty = (int)floor(y) + ind*PRECISION;
					break;
				}
			}
			for (int ind = INVPRECISION-1; ind >= 0; ind --)
			{
				if (nx >= ind*PRECISION)
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
				if (ny >= ind*PRECISION)
				{
					block_starty = (int)floor(y) + ind*PRECISION;
					break;
				}
			}
			for (int ind = INVPRECISION-1; ind >= 0; ind --)
			{
				if (nx >= ind*PRECISION)
				{
					block_startx = (int)floor(x) + ind*PRECISION;
					break;
				}
			}
			block_endx   = GRIDWIDTHR;
			block_endy   = GRIDHEIGHT;
		}
		//set the obstacle point and unseen block to zero
		double angle = atan2(y, x) * 180 / PI;
		double block_angle_begin, block_angle_end;
		for (double bx = block_startx; bx < block_endx; bx += PRECISION)
		{
			for (double by = block_starty; by < block_endy; by += PRECISION)
			{		
				if (bx < 0)
				{
					block_angle_begin = atan2(by+PRECISION, bx+PRECISION) * 180 / PI;
					block_angle_end   = atan2(by,bx) * 180 / PI;
				}
				else
				{
					block_angle_begin = atan2(by, bx+PRECISION) * 180 / PI;
					block_angle_end   = atan2(by+PRECISION, bx) * 180 / PI;
				}

				if (block_angle_begin < angle && block_angle_end > angle)
				{
					result_image.at<uchar>((GRIDHEIGHT-by-PRECISION)*INVPRECISION, (bx+GRIDWIDTHR)*INVPRECISION) = 0;
				}
			}
		}
	}
	//obstacle image 	
	cv::namedWindow("obstacle", 0);
	cv::imshow("obstacle", result_image);
	//distance image
	cv::distanceTransform(result_image, result_image, CV_DIST_L2, 3);
	cv::normalize(result_image, result_image, 0.0, 1.0, cv::NORM_MINMAX);
	cv::namedWindow("distance", 0);
	cv::imshow("distance", result_image);
	//convolution distance image
	cv::filter2D(result_image, result_image, result_image.depth(), meanfilter);
	cv::Point min_loc, max_loc;
	double min, max;
	cv::minMaxLoc(result_image, &min, &max, &min_loc, &max_loc);
	cv::circle(result_image, max_loc, 5, cv::Scalar(0,100,255));
	cv::namedWindow("finalConv", 0);
	cv::imshow("finalConv", result_image);
	
	cv::waitKey(1);
	return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distmap");
    ros::NodeHandle n;
    scan_sub = n.subscribe<in2_msgs::ScanInfoV2>("/ScanInfoV2", 10, my_callback);

	ros::spin();

	return 0;
}
