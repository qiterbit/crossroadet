#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ros::Publisher   marker_pub;
ros::Subscriber  scan_sub;

visualization_msgs::Marker points, line_list;

#define PI 3.14159265

int my_callback(const sensor_msgs::LaserScan::ConstPtr& scan_rec)
{
	/* laser receive */
	sensor_msgs::LaserScan::ConstPtr scan;
	scan = scan_rec;

	printf("frame_id: %s\n", scan->header.frame_id.c_str());
	printf("angle_range: [%f, %f]\n", scan->angle_min, scan->angle_max);
	printf("angle_increment: %f\n", scan->angle_increment);	
	printf("distance_range: [%f, %f]\n", scan->range_min, scan->range_max);
	int num = (scan->angle_max - scan->angle_min) / scan->angle_increment + 1;
	printf("number of points: [%d]\n", num);

	/* laser publish */
	points.header.frame_id = line_list.header.frame_id = "/laser";
	points.header.stamp = line_list.header.stamp = ros::Time::now();
   	points.ns = line_list.ns = "basic_shape";
   	points.action = line_list.action = visualization_msgs::Marker::ADD;
   	points.pose.orientation.w = line_list.pose.orientation.w = 1.0;

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

	/* points and line_list */
	float x, y;
   	for (int i = 0; i < num; i++)
   	{
		if (scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max)
		{
			x = cos(i*PI/num) * scan->ranges[i];
			y = sin(i*PI/num) * scan->ranges[i];
		}
		else
		{
			x = .0;
			y = .0;
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

	marker_pub.publish(points);
	marker_pub.publish(line_list);

	return 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
	scan_sub   = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, my_callback);

	ros::spin();

	return 0;
}