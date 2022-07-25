#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

using namespace visualization_msgs;

// variable
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

Marker makeText(InteractiveMarker &msg, std::string str)
{
    Marker marker;
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.pose.orientation.x =  marker.pose.orientation.y  = marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.8;
    marker.color.a = 1.0;
    marker.text = str;
    return marker;
}

void makeButtonMarker( const tf::Vector3& position, std::string str )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeText( int_marker, str );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "button_mode");
  ros::NodeHandle n;

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("button_mode","",false) );

  ros::Duration(0.1).sleep();

  position = tf::Vector3( 0, 0, 0);
  makeButtonMarker( position, "MTM Measurement" );
  position = tf::Vector3( 0, 3, 0);
  makeButtonMarker( position, "PSM Measurement" );

  server->applyChanges();

  ros::spin();

  server.reset();
}