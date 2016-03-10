import rospy
import time
from visualization_msgs import Marker
from geometry_msgs import Point

marker_publisher = rospy.Publisher('marker/lasers', Marker, queue_size=3)
    
def sendMarkers():
	xmarker = Marker();
	xmarker.type = 5 # line list
	xmarker.header.frame_id = "base_link";
	xmarker.header.stamp = ros::Time();
	xmarker.ns = "my_namespace";
	xmarker.id = 0;
	xmarker.type = visualization_msgs::Marker::SPHERE;
	xmarker.action = visualization_msgs::Marker::ADD;
	xmarker.pose.position.x = 1;
	xmarker.pose.position.y = 1;
	xmarker.pose.position.z = 1;
	xmarker.pose.orientation.x = 0.0;
	xmarker.pose.orientation.y = 0.0;
	xmarker.pose.orientation.z = 0.0;
	xmarker.pose.orientation.w = 1.0;
	xmarker.scale.x = 1; # Only X used
	xmarker.color.a = 1.0; 
	xmarker.color.r = 0.0;
	xmarker.color.g = 1.0;
	xmarker.color.b = 0.0;


	ymarker = Marker();
	ymarker.type = 5 # line list
	ymarker.header.frame_id = "base_link";
	ymarker.header.stamp = ros::Time();
	ymarker.ns = "my_namespace";
	ymarker.id = 0;
	ymarker.type = visualization_msgs::Marker::SPHERE;
	ymarker.action = visualization_msgs::Marker::ADD;
	ymarker.pose.position.x = 1;
	ymarker.pose.position.y = 1;
	ymarker.pose.position.z = 1;
	ymarker.pose.orientation.x = 0.0;
	ymarker.pose.orientation.y = 0.0;
	ymarker.pose.orientation.z = 0.0;
	ymarker.pose.orientation.w = 1.0;
	ymarker.scale.x = 1; # Only X used
	ymarker.color.a = 1.0; 
	ymarker.color.r = 0.0;
	ymarker.color.g = 1.0;
	ymarker.color.b = 0.0;


	zmarker = Marker();
	zmarker.type = 5 # line list
	zmarker.header.frame_id = "base_link";
	zmarker.header.stamp = ros::Time();
	zmarker.ns = "my_namespace";
	zmarker.id = 0;
	zmarker.type = visualization_msgs::Marker::SPHERE;
	zmarker.action = visualization_msgs::Marker::ADD;
	zmarker.pose.position.x = 1;
	zmarker.pose.position.y = 1;
	zmarker.pose.position.z = 1;
	zmarker.pose.orientation.x = 0.0;
	zmarker.pose.orientation.y = 0.0;
	zmarker.pose.orientation.z = 0.0;
	zmarker.pose.orientation.w = 1.0;
	zmarker.scale.x = 1; # Only X used
	zmarker.color.a = 1.0; 
	zmarker.color.r = 0.0;
	zmarker.color.g = 1.0;
	zmarker.color.b = 0.0;

	p = Point();
	p.x = 1.0
	p.y = 2.0
	p.z = 3.0
	zmarker.points.push_back(p);
	p.z = 4.0
	zmarker.points.push_back(p);