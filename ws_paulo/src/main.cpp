#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

std::string _name = "Paulo";
ros::Publisher chatter_pub;

void chatterCallback(const std_msgs::String::ConstPtr& msg_in)
{
  ROS_INFO("%s: heard: [%s]",_name.c_str(),  msg_in->data.c_str());
	
  std_msgs::String msg_out;
  msg_out.data = "Hello world";

	ros::Rate loop_rate(0.5);
	ros::spinOnce();
  loop_rate.sleep();

	ROS_INFO("%s: will publish a message", _name.c_str());
  chatter_pub.publish(msg_out);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ws_p");
  ros::NodeHandle n;
  chatter_pub = n.advertise<std_msgs::String>("player_out", 1);
  ros::Subscriber sub = n.subscribe("player_in", 1, chatterCallback);

  ROS_INFO("%s: node started", _name.c_str());

  ros::Rate loop_rate(2);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

