#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <ws_referee/custom.h>
#include <ws_referee/randomize.h>
#include <visualization_msgs/Marker.h>

std::string _name = "Paulo";
ros::Publisher chatter_pub;
ros::Publisher marker_pub;
double _pos_x;
double _pos_y;

void chatterCallback(const ws_referee::custom::ConstPtr& msg_in)
{
	bool finish = false;

  	ROS_INFO("%s: Received msg with dist %f", _name.c_str(), msg_in->dist);
	_pos_x += msg_in->dist;

	ws_referee::custom msg_out;
	msg_out.sender = _name;

	if(msg_in->winner != ""){
		ROS_INFO("%s: %s won the race!! :(", _name.c_str(), msg_in->winner.c_str());
		msg_out.winner = msg_in->winner;
		msg_out.dist = 0.0;
		finish = true;
	}else if(_pos_x > 5){
		ROS_WARN("\n\n%s: I won the race!! :DD\n", _name.c_str());
		msg_out.winner = _name.c_str();
		msg_out.dist = 0.0;
		finish = true;
	}else{
		msg_out.dist = get_random_num();
		msg_out.winner = "";
	}

	
	ROS_INFO("%s: Iam at %f. I will publish a message", _name.c_str(), _pos_x);
  	chatter_pub.publish(msg_out);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = _pos_x;
	marker.pose.position.y = _pos_y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.3;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker_pub.publish( marker );

	marker.id = 1;
	marker.pose.position.x = _pos_x - 0.3;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.text = "Paulo";
	marker_pub.publish(marker);

	if(finish){
		ROS_INFO("%s: I will shutdown",_name.c_str());
		ros::shutdown();
	}
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ws_p");
  ros::NodeHandle n;

init_randomization_seed();

  chatter_pub = n.advertise<ws_referee::custom>("player_out", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("paulo_marker", 1);
  ros::Subscriber sub = n.subscribe("player_in", 1, chatterCallback);

	_pos_x = 0;
	_pos_y = 5;

  ROS_INFO("%s: node started", _name.c_str());

  ros::Rate loop_rate(2);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

