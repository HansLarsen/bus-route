#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>

#define ArraySize 10

using namespace std;
using namespace ros;
using namespace geometry_msgs;
bool trueOn = false;

void _processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
    << feedback->pose.position.x << ", " << feedback->pose.position.y
    << ", " << feedback->pose.position.z );
    trueOn = true;
}

class Route
{
private:
    uint stops_initialized;
    geometry_msgs::PointStamped points [ArraySize];
    int stops_order [ArraySize];
    uint size = 0;
    uint currentlyAt = 0;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
    ros::Publisher marker_pub;
    ros::Subscriber click_sub;

    void _init()
    {
        for (int i = 0; i < ArraySize; i++)
        {
            stops_order[i] = i;
        }
    }

    void _send_goal(const geometry_msgs::PointStamped& goal_point)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = goal_point.header.frame_id;
        goal.target_pose.pose.position = goal_point.point;
        goal.target_pose.pose.orientation.w = 1.0;
        client.sendGoal(goal, boost::bind(&Route::_target_reached_cb, this, _1, _2)  );
        _send_markers();
    }

    void _target_reached_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
    {
        _shift_array_by_one();
        _send_goal(points[stops_order[0]]);
    }

    void _shift_array_by_one() //Moves index 0 to the highest index in the array, so that the array is shifted arround.
    {
        uint savedPoint = stops_order[0];
        for(int i = 0; i < stops_initialized - 1; i++)
        {
            stops_order[i] = stops_order[i + 1];
        }
        stops_order[stops_initialized - 1] = savedPoint;
    }

    void _send_markers() //Sets the markers on the map.
    {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.ns = "bus_stops";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0.7071;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0.7071;
        marker.lifetime = ros::Duration();

        visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < stops_initialized; i++)
        {
            PointStamped from = points[i];
            marker.header.frame_id = from.header.frame_id;
            marker.id = i + 1;
            marker.pose.position = from.point;
            marker.pose.position.z += marker.scale.x;
            if (i == stops_order[0])
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
            }
            marker_array.markers.push_back(marker);
        }
        marker_pub.publish(marker_array);
    }

    void _clicked_point_cb(const PointStamped::ConstPtr& msg) //call back function for when a point is clicked in rviz.
    {
        //ROS_INFO("Received Point %f", (*msg).point.x );

        points[stops_initialized] = (*msg);

        if (stops_initialized <= ArraySize)
        {
            stops_initialized++;
        }

        /*for(int i = 0; i <= stops_initialized; i++ )
        {
            ROS_INFO("%d, %f", i, points[i].point.x);
        }*/
        _sort_routes_next_closest();
        _send_markers();

        if (trueOn == true)
        {
            _send_goal(points[0]);
        }
    }

    void _sort_routes_next_closest() //Finds the next closest point and adds it to an array and saves the array.
    {

        if (stops_initialized == 1) return;
        int temp_stops_order [ArraySize];
        for (int i = 0; i < ArraySize; i++) //Goes through the stops_order array. Point 1 is i.
        {
            temp_stops_order[i] = 0;
        }

        for (int i = 0; i < stops_initialized - 1; i++) //Goes through the stops_order array. Point 1 is i.
        {
            int pointNumber = temp_stops_order[i];
            PointStamped point1 = points[pointNumber];
            int number = 0;
            float oldLength = 100.0;
            for (int j = 0; j < stops_initialized; j++) //Goes through the stops_order array, and compares the length to the next vector.
            {
                if ( _containes_point(j, temp_stops_order) )
                {
                    continue;
                }
                PointStamped point2 = points[j];
                float lengths = _vector_length(point1, point2);
                if (lengths < oldLength)
                {
                    oldLength = lengths;
                    number = j;
                }
            }
            temp_stops_order[ i + 1 ] = number;
        }
        ROS_INFO("Second Array");
        for ( int i = 0; i < ArraySize; i++)
        {
            stops_order[i] = temp_stops_order[i];
            ROS_INFO("%d", stops_order[i]);
        }
    }

    bool _containes_point(int point, int pointArray[ArraySize] ) //Checks point array to see if the given point is in it.
    {
        for ( int i = 0; i < ArraySize; i++)
        {
            if (pointArray[i] == point)
            {
                return true;
            }
        }
        return false;
    }

    float _vector_total_length() //Finds the total length of the points_order array.
    {
        float length = 0.0;
        for (int i = 0; i < stops_initialized - 1; i++)
        {
            length += _vector_length(points[stops_order[i]], points[stops_order[i + 1]]);
        }
        return length;
    }

    float _vector_length(PointStamped point1, PointStamped point2 ) //finds the length of the vector between two points.
    {
        return sqrt( pow(point2.point.x - point1.point.x, 2) + pow(point2.point.y - point1.point.y, 2) );
    }

public:
    Route() : stops_initialized(0), client("move_base")
    {
        _init();
        ros::NodeHandle n;
        marker_pub = n.advertise<visualization_msgs::MarkerArray>("busroute_markers", 1);
        click_sub = n.subscribe( "clicked_point", 100, &Route::_clicked_point_cb, this);
    };
    ~Route(){};


};

// This is where we start
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "busroute");

    Route r;

        /*
    #############################################################################################
    ##                                      InterActive Marker                                 ##
    #############################################################################################
    */

    // create an interactive marker server on the topic namespace simple_marker
    interactive_markers::InteractiveMarkerServer server("simple_marker");

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = "my_marker";
    int_marker.description = "On Button";

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.45;
    box_marker.scale.y = 0.45;
    box_marker.scale.z = 0.45;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back( box_marker );

    // add the control to the interactive marker
    int_marker.controls.push_back( box_control );

    // create a control whice interactive markerh will move the box
    // this control does not  int_marker.controls.push_back(rotate_control); contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl rotate_control;
    rotate_control.name = "move_x";  // tell the server to call processFeedback() when feedback arrives for itove_x";
    rotate_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    // 'commit' changes and send to all clients
    // add the control to thserver.applyChanges();e interactive marker
    int_marker.controls.push_back(rotate_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, &_processFeedback );

    // 'commit' changes and send to all clients
    server.applyChanges();

    ros::spin();
    return 0;
}
