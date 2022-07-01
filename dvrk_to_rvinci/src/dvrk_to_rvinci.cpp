#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <rvinci_input_msg/rvinci_input.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

class davinci_mtm
{
public:
    davinci_mtm();
  ros::Publisher  rvinci_pub_;
  rvinci_input_msg::rvinci_input rvmsg_;
  void getPose(const geometry_msgs::PoseStamped::ConstPtr&, int);
  void gripCallback(const std_msgs::Bool::ConstPtr& grip, int);
  void getaGrip();
  void cameraCallback(const sensor_msgs::Joy::ConstPtr& cam);
  void clutchCallback(const sensor_msgs::Joy::ConstPtr& cltch);
private:

  ros::NodeHandle n;
  ros::Subscriber left_pose_sub_;
  ros::Subscriber right_pose_sub_;
  ros::Subscriber left_grip_sub_;
  ros::Subscriber right_grip_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber clutch_sub_;

};

davinci_mtm::davinci_mtm()
{
  rvmsg_.header.frame_id = "/base_link";
  left_grip_sub_ = n.subscribe<std_msgs::Bool>("/MTML/gripper/closed",10,boost::bind(&davinci_mtm::gripCallback,this,_1,0));
  right_grip_sub_ = n.subscribe<std_msgs::Bool>("/MTMR/gripper/closed",10,boost::bind(&davinci_mtm::gripCallback,this,_1,1));
  left_pose_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/MTML/measured_cp",10,boost::bind(&davinci_mtm::getPose,this,_1,0));
  right_pose_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/MTMR/measured_cp",10,boost::bind(&davinci_mtm::getPose,this,_1,1));
  rvinci_pub_ = n.advertise<rvinci_input_msg::rvinci_input>("/rvinci_input_update",10);

  camera_sub_ = n.subscribe<sensor_msgs::Joy>("/footpedals/camera",10,&davinci_mtm::cameraCallback,this);
  clutch_sub_ = n.subscribe<sensor_msgs::Joy>("/footpedals/clutch",10,&davinci_mtm::clutchCallback,this);
}
//constructor creates marker that is used as a visualization for the 3D cursor and establishes
//pubs and subs for controller positon and button states.

void davinci_mtm::getPose(const geometry_msgs::PoseStamped::ConstPtr& pose, int i)
{
  ROS_INFO_STREAM("***** getPose *****");
  rvmsg_.gripper[i].pose = pose->pose;
  //Offsets to set davinci at 0 x and y, with an x offset for each gripper.
  rvmsg_.gripper[i].pose.position.x += (2*i-1)*0.15;
  rvmsg_.gripper[i].pose.position.y += 0.36;
  rvmsg_.gripper[i].pose.position.z += 0.135;
}
void davinci_mtm::gripCallback(const std_msgs::Bool::ConstPtr& grab, int i)
{
  ROS_INFO_STREAM("***** gripCallback *****");
 rvmsg_.gripper[i].grab = grab->data;
}
void davinci_mtm::cameraCallback(const sensor_msgs::Joy::ConstPtr& cam)
{
  rvmsg_.camera = cam->buttons[0];
}
void davinci_mtm::clutchCallback(const sensor_msgs::Joy::ConstPtr& cltch)
{
  ROS_INFO_STREAM("****** CLUTCH *******");
  rvmsg_.clutch = cltch->buttons[0];
  ROS_INFO_STREAM(cltch->buttons[0]);
}
int main(int argc, char** argv){
  ros::init(argc, argv, "dvrk_to_rvinci");
  davinci_mtm mtmlr;
  ros::Rate r(200);

  while(ros::ok())
  {
    ros::spinOnce();
    mtmlr.rvmsg_.header.stamp = ros::Time::now();
    mtmlr.rvinci_pub_.publish(mtmlr.rvmsg_);
    r.sleep();
  }
}

