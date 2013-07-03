#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <ws_referee/custom.h>

std::string _name = "Paulo";
ros::Publisher chatter_pub;
double _pos_x;
double _pos_y;

void chatterCallback(const ws_referee::custom::ConstPtr& msg_in)
{
  ROS_INFO("%s: Received msg with dist %f", _name.c_str(), msg_in->dist);
	_pos_x += msg_in->dist;
	
	ws_referee::custom msg_out;
	msg_out.dist = 0.8;
	msg_out.sender = _name;
	msg_out.winner = "";

	ROS_INFO("%s: Iam at %f. I will publish a message", _name.c_str(), _pos_x);
  chatter_pub.publish(msg_out);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ws_p");
  ros::NodeHandle n;
  chatter_pub = n.advertise<ws_referee::custom>("player_out", 1);
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

