#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ros::Publisher   marker_pub;
ros::Subscriber  scan_sub;

visualization_msgs::Marker points, line_list, cube_list;

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

cv::Mat mask = cv::Mat::ones(3, 3, CV_32FC1) / 9;

int my_callback(const sensor_msgs::LaserScan::ConstPtr& scan_rec)
{
	cv::Mat dist_image(GRIDHEIGHT*INVPRECISION, GRIDWIDTHR*2*INVPRECISION, CV_8UC1, cv::Scalar::all(255));
	cv::Mat result_image(GRIDHEIGHT*INVPRECISION, GRIDWIDTHR*2*INVPRECISION, CV_8UC1, cv::Scalar::all(255));
	
	/* laser receive */
	sensor_msgs::LaserScan::ConstPtr scan;
	scan = scan_rec;
	//printf("seq: %d\n", scan->header.seq);
	//printf("stamp: %d\n", scan->header.stamp.sec);
	printf("frame_id: %s\n", scan->header.frame_id.c_str());
	printf("angle_range: [%f, %f]\n", scan->angle_min, scan->angle_max);
	printf("angle_increment: %f\n", scan->angle_increment);	
	//printf("time_increment: %f\n", scan->time_increment);
	//printf("scan_time: %f\n", scan->scan_time);
	printf("distance_range: [%f, %f]\n", scan->range_min, scan->range_max);
	double per_angle = PI / 180.0 / scan->angle_increment;
	int num = (scan->angle_max - scan->angle_min) / scan->angle_increment + 1;
	printf("number of points: %d\n", num);

	/* laser publish */
	points.header.frame_id = line_list.header.frame_id = cube_list.header.frame_id = "/laserfake";
	points.header.stamp = line_list.header.stamp = cube_list.header.stamp = ros::Time::now();
   	points.ns = line_list.ns = "basic_shape";
	cube_list.ns = "obstacle";
   	points.action = line_list.action = cube_list.action = visualization_msgs::Marker::ADD;
   	points.pose.orientation.w = line_list.pose.orientation.w = cube_list.pose.orientation.w = 1.0;

   	points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
   	points.scale.x = 0.1;
   	points.scale.y = 0.1;
   	points.color.g = 1.0;
   	points.color.a = 1.0;    
	points.points.resize(num);

	line_list.id = 1;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.005;
	line_list.color.r = 1.0;
    line_list.color.a = 1.0;
	line_list.points.resize(num*2);

	cube_list.id = 2;
	cube_list.type = visualization_msgs::Marker::CUBE_LIST;
	cube_list.scale.x = PRECISION;
	cube_list.scale.y = PRECISION;
	cube_list.color.r = 1.0;
    cube_list.color.a = 1.0;
	cube_list.points.resize(GRIDSIZE);	
	cube_list.colors.resize(GRIDSIZE);

	/* points and line_list */
	float x, y;
   	for (uint32_t i = 0; i < num; ++i)
   	{
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
		points.points[i].x = x;
		points.points[i].y = y;
		points.points[i].z = 0.0;

		line_list.points[i*2].x = 0.0;
		line_list.points[i*2].y = 0.0;
		line_list.points[i*2].z = 0.0;
		line_list.points[i*2+1].x = x;
		line_list.points[i*2+1].y = y;
		line_list.points[i*2+1].z = 0.0;	
    }

	double angle_begin, angle_end;
	for (double x = GRIDWIDTHL; x < GRIDWIDTHR; x += PRECISION)
	{
		for (double y = 0.0; y < GRIDHEIGHT; y += PRECISION)
		{
			dist_image.at<uchar>((GRIDHEIGHT-y-PRECISION)*INVPRECISION, (x+GRIDWIDTHR)*INVPRECISION) = 255;
		
			int ind = (x+GRIDWIDTHR)*GRIDHEIGHT*INVPRECISION*INVPRECISION+y*INVPRECISION;
			cube_list.points[ind].x = x + HALFPRECISION;
			cube_list.points[ind].y = y + HALFPRECISION;						
			cube_list.colors[ind].a = 0;

			if (x < 0)
			{
				angle_begin = atan2(y+PRECISION, x+PRECISION) * 180 / PI;
				angle_end   = atan2(y, x) * 180 / PI;
			}
			else
			{
				angle_begin = atan2(y, x+PRECISION) * 180 / PI;
				angle_end   = atan2(y+PRECISION, x) * 180 / PI;
			}
			angle_begin *= per_angle;
			angle_end   *= per_angle;

			for (int index = (int)floor(angle_begin); index < (int)ceil(angle_end); index++)
			{
				if (points.points[index].x >= x && points.points[index].x <= x+PRECISION 
				 && points.points[index].y >= y && points.points[index].y <= y+PRECISION)
				{	
					dist_image.at<uchar>((GRIDHEIGHT-y-PRECISION)*INVPRECISION, (x+GRIDWIDTHR)*INVPRECISION) = 0;
					cube_list.colors[ind].b = 1.0;	
					cube_list.colors[ind].a = 1.0;
					break;
				}
			}
		}
	}
   
	for (int i = 0; i < num; i++)
	{
	
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
		//ROS_INFO("[%f, %f, %f, %f]\n", block_startx, block_endx, block_starty, block_endy);
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
	
	cv::namedWindow("binary", 0);
	cv::imshow("binary", dist_image);
	cv::namedWindow("result", 0);
	cv::imshow("result", result_image);
	cv::distanceTransform(result_image, result_image, CV_DIST_L2, 3);
	cv::normalize(result_image, result_image, 0.0, 1.0, cv::NORM_MINMAX);
	cv::Point min_loc, max_loc;
	double min, max;
	cv::minMaxLoc(result_image, &min, &max, &min_loc, &max_loc);
	cv::circle(result_image, max_loc, 2, cv::Scalar(0,100,255));
	cv::namedWindow("dist", 0);
	cv::imshow("dist", result_image);
	cv::filter2D(result_image, result_image, result_image.depth(), mask);
	cv::namedWindow("finalConv", 0);
	cv::imshow("finalConv", result_image);
	cv::waitKey(1);
	
	marker_pub.publish(points);
	marker_pub.publish(line_list);
	marker_pub.publish(cube_list);

	return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gridmap");
    ros::NodeHandle n;
	marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    scan_sub   = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, my_callback);
	
	ros::spin();

	return 0;
}
