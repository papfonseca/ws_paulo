#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <ws_referee/custom.h>
#include <ws_referee/MovePlayerTo.h>
#include <ws_referee/randomize.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//Global variable 
std::string _name="paulo";
std::string _policed_player="paulo";
ros::Publisher chatter_pub;
ros::Publisher marker_pub;

tf::Transform transform;
tf::TransformBroadcaster* br;
tf::TransformListener* listener;

ros::ServiceClient *client;

double _pos_x;
double _pos_y;

void chatterCallback(const ws_referee::custom::ConstPtr& msg_in)
{

	ROS_INFO("%s: Received msg with dist=%f",_name.c_str(), msg_in->dist);

	//Check for the policed player

	bool be_a_police = true;
	//query transform world to the policed player
	tf::StampedTransform tf_2;
	try{
		listener->lookupTransform("world", "tf_" + _policed_player, ros::Time(0), tf_2);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		be_a_police = false;
	}

	if (be_a_police)	
	{

		ROS_INFO("%s: I will police %s.",_name.c_str(), _policed_player.c_str());

		if (!is_in_field(tf_2.getOrigin().x(), tf_2.getOrigin().y()))
		{
			//send player to new_pos_x = -5 and new_pos_y=0
			ROS_INFO("%s: Policing ... I found that %s if out of the arena. Will send him to -5,0", _name.c_str(), _policed_player.c_str());

			ws_referee::MovePlayerTo srv;
			srv.request.new_pos_x = -5;
			srv.request.new_pos_y = 0;
			srv.request.player_that_requested = _name;

			if(client->call(srv)){

			}else{
				ROS_ERROR("failed to call service ...");
			}
		}
	}


	//Position update
	//Send transform from tf_mike to tf_tmp_mike
	tf::Transform tf_tmp;
	tf_tmp.setOrigin( tf::Vector3(msg_in->dist, 0.0, 0.0) );
	tf_tmp.setRotation( tf::Quaternion(0, 0, get_random_deg()*M_PI/180, 1) );
	br->sendTransform(tf::StampedTransform(tf_tmp, ros::Time::now(), "tf_" + _name, "tf_tmp" + _name));

	ros::Duration(0.1).sleep(); //wait a bit

	//query transform world to tf_tmp_mike
	tf::StampedTransform tf_1;
	try{
		listener->lookupTransform("world", "tf_tmp" + _name, ros::Time(0), tf_1);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}

	//send new transform
	transform = tf_1;
	br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "tf_" + _name));



	bool should_quit=false;
	//Publish new msg
	ws_referee::custom msg_out;
	msg_out.sender = _name;

	//Check race status
	if (msg_in->winner!="")//if someone else won (crap)
	{
		ROS_INFO("%s: Damn, %s has won the race. It was just luck",_name.c_str(), msg_in->winner.c_str());
		msg_out.winner = msg_in->winner;
		msg_out.dist = 0;
		should_quit=true;
	}
	else if (_pos_x > 5) //if I won
	{
		ROS_WARN("\n\n%s: I WON IUPIIII\n\n ",_name.c_str());
		msg_out.winner = "paulo";
		msg_out.dist = 0;
		should_quit=true;
	}
	else //if nobody won
	{
		msg_out.winner = "";
		msg_out.dist = get_random_num();
	}

	chatter_pub.publish(msg_out);
	ROS_INFO("%s: I am at %f. I will win for sure.  will publish a message",_name.c_str(), _pos_x);

	//The visualization markers
	visualization_msgs::Marker marker;
	marker.header.frame_id = "tf_" + _name;
	marker.header.stamp = ros::Time();
	marker.ns = ""; marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0; marker.pose.position.y = 0;
	marker.pose.position.z = 0; marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.3; marker.scale.y = 0.3;	marker.scale.z = 0.3;
	marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.3;
	marker.color.b = 0.0;
	marker_pub.publish( marker );

	//a new text marker
	marker.id = 1;
	marker.color.r = 0.4;
	marker.pose.position.x = - 0.3;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.text = "paulo";
	marker_pub.publish( marker );

	//Send the transformation
	br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "tf_" + _name));

	if (should_quit==true)
	{
		ROS_INFO("%s: I will shutdown",_name.c_str());
		ros::shutdown();
	}
}

bool serviceCallback(ws_referee::MovePlayerTo::Request  &req,
		ws_referee::MovePlayerTo::Response &res)
{
	ROS_INFO("%s: Damn %s sent me to x=%f, y=%f", _name.c_str(), req.player_that_requested.c_str(), req.new_pos_x, req.new_pos_y);

	//Send the transformation
	transform.setOrigin( tf::Vector3(req.new_pos_x, req.new_pos_y, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
	br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "tf_" + _name));

	//Add the code for moving our position to the requested new_pos
	res.reply = "I will have my revenge!";

	return true;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, _name);
	ros::NodeHandle n;

	//init the randomizer
	init_randomization_seed();

	chatter_pub = n.advertise<ws_referee::custom>("player_out", 1);
	marker_pub = n.advertise<visualization_msgs::Marker>("paulo_marker", 1);


	//Allocate the broadcaster and listener
	br = (tf::TransformBroadcaster*) new (tf::TransformBroadcaster);
	listener = (tf::TransformListener*) new (tf::TransformListener);

	_pos_x = -5;
	_pos_y = 6; //to be read from a param

	ros::Duration(0.1).sleep();


	ros::Time t = ros::Time::now();
	//Send the transformation
	transform.setOrigin( tf::Vector3(_pos_x, _pos_y, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
	br->sendTransform(tf::StampedTransform(transform, t, "world", "tf_" + _name));

	tf::Transform tf_tmp;
	tf_tmp.setOrigin( tf::Vector3(0, 0.0, 0.0) );
	tf_tmp.setRotation( tf::Quaternion(0, 0, 0, 1) );
	br->sendTransform(tf::StampedTransform(tf_tmp, t, "tf_" + _name, "tf_tmp" + _name));

	ros::spinOnce();
	ros::Duration(0.1).sleep();

	//at time now
	transform.setOrigin( tf::Vector3(_pos_x, _pos_y, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
	br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "tf_" + _name));

	tf_tmp.setOrigin( tf::Vector3(0, 0.0, 0.0) );
	tf_tmp.setRotation( tf::Quaternion(0, 0, 0, 1) );
	br->sendTransform(tf::StampedTransform(tf_tmp, ros::Time::now(), "tf_" + _name, "tf_tmp" + _name));

	client = (ros::ServiceClient*) new (ros::ServiceClient);
	*client = n.serviceClient<ws_referee::MovePlayerTo>("move_player_"+_policed_player);

	ros::ServiceServer service = n.advertiseService("move_player_" + _name, serviceCallback);
	ros::Subscriber sub = n.subscribe("player_in", 1, chatterCallback);
	ros::Rate loop_rate(2);

	ROS_INFO("%s: node started",_name.c_str());

	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();

	int count = 0;
	/*while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}*/


	return 0;
}
