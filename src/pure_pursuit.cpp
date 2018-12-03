// Pure Pursuit Boilerplate Exercise
//
// Copyright (c) 2018 Karl D. Hansen
// MIT License - see LICENSE file.

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


int points_initialized = 0;
double from_x, from_y, to_x, to_y;
ros::Publisher marker_pub;
bool lookaheadCalculated = false;
double lookahead_x;
double lookahead_y;

void send_markers()
{
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "markers";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0.7071;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0.7071;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        visualization_msgs::MarkerArray marker_array;

        if (points_initialized > 0)
        {
            marker.id = 0;
            marker.pose.position.x = from_x;
            marker.pose.position.y = from_y;
            marker.pose.position.z = 0;
            marker_array.markers.push_back(marker);
        }
        if (points_initialized > 1)
        {
            marker.id = 1;
            marker.pose.position.x = to_x;
            marker.pose.position.y = to_y;
            marker.pose.position.z = 0;
            marker_array.markers.push_back(marker);
        }
        if (lookaheadCalculated == true){
            marker.id = 2;
            marker.pose.position.x = lookahead_x;
            marker.pose.position.y = lookahead_y;
            marker.pose.position.z = 0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
        }
        marker_pub.publish(marker_array);
}

// Callback function to handle clicked points in RViz
// When more than two points have been clicked, the "to" point changes to the "from" point.
void clickCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  ROS_INFO("Clicked: %f, %f, %f", msg->point.x, msg->point.y, msg->point.z);
  switch (points_initialized)
  {
  case 0:
    from_x = msg->point.x;
    from_y = msg->point.y;
    break;
  case 1:
    to_x = msg->point.x;
    to_y = msg->point.y;
    break;
  default:
    from_x = to_x;
    from_y = to_y;
    to_x = msg->point.x;
    to_y = msg->point.y;
  }
  points_initialized += 1;
}

int main(int argc, char **argv)
{

  // Initialize the ROS node
  ros::init(argc, argv, "turtlebot_pure_pursuit");
  ros::NodeHandle node;

  // Subscribe the markers:
  marker_pub = node.advertise<visualization_msgs::MarkerArray>(
"goal_markers", 3);

  // Publish our output; the wanted speed of the robot.
  ros::Publisher turtle_vel =
      node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

  // Initialize the transform listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Subscribe to the clicked point from RViz
  ros::Subscriber click_sub = node.subscribe("clicked_point", 1000, clickCallback);

  // The business code
  // - Lookup the position of the robot in the map frame.
  // - Compute the lookahead point.
  // - Find the coresponding linear and angular speeds.
  // - Publish them.
  ros::Rate rate(10.0);
  while (node.ok())
  {
    // Execute any callbacks
    ros::spinOnce();

    send_markers();

    // Check that two points has been clicked
    if (points_initialized < 2)
    {
      rate.sleep();
      continue;
    }

    // Lookup position and orientation using the tf2 system.
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform( "map","base_footprint",
                                                  ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    double robot_pos_x = fabs(transformStamped.transform.translation.x);
    double robot_pos_y = fabs(transformStamped.transform.translation.y);
    double robot_rool, robot_pitch, robot_yaw;
    tf2::getEulerYPR(transformStamped.transform.rotation,
                     robot_yaw, robot_pitch, robot_rool);

    // Lookahead point
    // TODO: Compute the right values for the lookahead.
    // Use the values of: from_x, from_y, to_x, to_y, robot_pos_x, robot_pos_y
    double m = (to_y - from_y) / (to_x - from_x);
    double b = from_y - (m * from_x);
    double PLx = (m * robot_pos_y + robot_pos_x - m * b) / (m * m + 1);
    double PLy = (m * m * robot_pos_y + m * robot_pos_x + b) / (m * m + 1);

    // retningsvektor
    double Rx= to_x - from_x;
    double Ry= to_y - from_y;

    lookahead_x = PLx+0.2*Rx;
    lookahead_y = PLy+0.2*Ry;
    lookaheadCalculated = true;

    ROS_INFO("Lookahead X: %f, Y: %f", lookahead_x, lookahead_y);

    // Compute coresponding speeds and publish them
    geometry_msgs::Twist vel_msg;

    // Angular speed
    // TODO: Find angle between heading (yaw) and direction to the lookahead
    //       point. Use the atan2 function and the points lookahead_x/y and
    //       robot_pos_x/y.
    //       Consider using a scalar on the angle difference before sending the
    //       speed reference.
vel_msg.angular.z=  (atan2((lookahead_y-robot_pos_y),(lookahead_x -robot_pos_x))-robot_yaw)*0.5;;

    // Linear speed
    // TODO: Find the Euclidean distance between the lookahead point and the
    //       robot. Use the sqrt and pow functions, and the the points
    //       lookahead_x/y and robot_pos_x/y.
    //       Again, consider a scalar.

vel_msg.linear.x= sqrt(pow((lookahead_y - robot_pos_y),2)+pow((lookahead_x-robot_pos_x),2))*0.5;
    // Publish
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
