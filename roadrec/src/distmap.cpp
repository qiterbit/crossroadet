#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ros::Subscriber  scan_sub;

#define PI 3.14159265

//width and height should be even and symmetry
//precision must be integer divided by GRID**
#define   GRIDWIDTHL     -10
#define   GRIDWIDTHR      10  
#define   GRIDHEIGHT      15
#define   PRECISION       0.25
#define   INVPRECISION    4
#define   HALFPRECISION   PRECISION/2   
#define   GRIDSIZE        GRIDHEIGHT*(GRIDWIDTHR-GRIDWIDTHL)*INVPRECISION*INVPRECISION

cv::Mat meanfilter = cv::Mat::ones(3, 3, CV_32FC1) / 9;

int my_callback(const sensor_msgs::LaserScan::ConstPtr& scan_rec)
{
	/* laser receive */
	sensor_msgs::LaserScan::ConstPtr scan;
	scan = scan_rec;
	double per_angle = PI / 180.0 / scan->angle_increment;
	int num = (scan->angle_max - scan->angle_min) / scan->angle_increment + 1;
	printf("seq: %d\n", scan->header.seq);
	printf("stamp: %d\n", scan->header.stamp.sec);
	printf("frame_id: %s\n", scan->header.frame_id.c_str());
	printf("angle_range: [%f, %f]deg\n", scan->angle_min / PI * 180, scan->angle_max / PI * 180);
	printf("angle_increment: %f\n", scan->angle_increment / PI * 180);	
	printf("time_increment: %f\n", scan->time_increment);
	printf("scan_time: %f\n", scan->scan_time);
	printf("distance_range: [%f, %f]m\n", scan->range_min, scan->range_max);
	printf("number of points: %d\n", num);
	printf("\n");

	/* image process */
	cv::Mat result_image(GRIDHEIGHT*INVPRECISION, GRIDWIDTHR*2*INVPRECISION, CV_8UC1, cv::Scalar::all(255));
	for (int i = 0; i < num; i++)
	{
		/* handle out points out of border */	
		float x, y;
		if (scan->ranges[i] > 2 * PRECISION && scan->ranges[i] < scan->range_max)
		{
			x = cos(i*PI/num) * scan->ranges[i];
			y = sin(i*PI/num) * scan->ranges[i];
		}
		else
		{
			x = cos(i*PI/num) * scan->range_max;
			y = sin(i*PI/num) * scan->range_max;
		}
		/* locate the search range of every line */
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
		/* set the obstacle point and unseen block to zero */ 
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
	cv::circle(result_image, max_loc, 2, cv::Scalar(0,100,255));
	cv::namedWindow("finalConv", 0);
	cv::imshow("finalConv", result_image);
	
	cv::waitKey(1);
	return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distmap");
    ros::NodeHandle n;
    scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, my_callback);

	ros::spin();

	return 0;
}
