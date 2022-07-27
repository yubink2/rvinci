/* * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <iostream>
#include <cmath>
#include <string>

#include <QWidget>
#include <QDesktopWidget>
#include <QApplication>

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>

#include <ros/package.h>
#include <ros/console.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>
#include <tf/transform_datatypes.h>

#include <interaction_cursor_msgs/InteractionCursorUpdate.h>
#include "interaction_cursor_rviz/interaction_cursor.h"
#include <rvinci_input_msg/rvinci_input.h>
#include <std_msgs/String.h>
#include "rvinci/rvinci_display.h"

#define _LEFT 0
#define _RIGHT 1

namespace rvinci
{
rvinciDisplay::rvinciDisplay()
  : render_widget_(0)
  , render_widget_R_(0)
  , camera_node_(0)
  , window_(0)
  , window_R_(0)
  , camera_offset_(0.0,-3.0,1.5)
{
  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);

  prop_ros_topic_ = new rviz::RosTopicProperty("Input Topic","/davinci_msg"
                                               ,ros::message_traits::datatype<rvinci_input_msg::rvinci_input>(),
                                               "Subscription topic (published by input controller node)"
                                               ,this,SLOT ( pubsubSetup()));
  prop_input_scalar_ = new rviz::VectorProperty("Input Scalar",Ogre::Vector3(5,5,5),
                                                "Scalar for X, Y, and Z of controller input motion",this);
  prop_cam_reset_ = new rviz::BoolProperty("Camera Reset",false,
                                           "Reset camera and cursor position", this, SLOT (cameraReset()));
  prop_gravity_comp_ = new rviz::BoolProperty("Release da Vinci",false,
                                           "Put da Vinci in Gravity Compensation mode", this, SLOT (gravityCompensation()));

//  prop_manual_coords_ = new rviz::BoolProperty("Use typed coordinates",false,
//                                               "Camera movement controlled by typed coordinates",this);

  prop_cam_focus_ = new rviz::VectorProperty("Camera Focus",Ogre::Vector3(0,0,0),
                                             "Focus Point of Camera",this);
  prop_camera_posit_ = new rviz::VectorProperty("Camera Position",camera_offset_,
                                                 "Position of scene node to world base frame",this);
  property_camrot_ = new rviz::QuaternionProperty("Camera Orientation",Ogre::Quaternion(0,0,0,1),
                                                  "Orientation of the camera",this);
  camera_[_LEFT] = 0;
  camera_[_RIGHT]= 0;

  camera_ipd_ = Ogre::Vector3(0.03,0.0,0.0);

  buffer_[0] = NULL;
  buffer_[1] = NULL;
  backgroundImage_[0] = NULL;
  backgroundImage_[1] = NULL;
  rect_[0] = NULL;
  rect_[1] = NULL;
  material_[0].setNull();
  material_[1].setNull();
  texture_[0].setNull();
  texture_[1].setNull();

  
}
rvinciDisplay::~rvinciDisplay()
{
  window_->removeViewport(0);
  window_R_->removeViewport(0);
  for(int i = 0; i<2; ++i)
  {
    if (viewport_[i])
    {
      viewport_[i] = 0;
    }

    if (camera_[i])
    {
      camera_[i]->getParentSceneNode()->detachObject(camera_[i]);
      scene_manager_->destroyCamera(camera_[i]);
      camera_[i] = 0;
    }
  }
  if (camera_node_)
  {
    camera_node_->getParentSceneNode()->removeChild(camera_node_);
    scene_manager_->destroySceneNode(camera_node_);
    camera_node_ = 0;
  }
  window_ = 0;
  window_R_ = 0;
  delete render_widget_;
  delete render_widget_R_;
//  delete prop_manual_coords_;
  delete prop_cam_focus_;
  delete prop_camera_posit_;
  delete prop_input_scalar_;
}
void rvinciDisplay::onInitialize()
{
  render_widget_ = new rviz::RenderWidget(rviz::RenderSystem::get());
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle("RVinci Left");
  render_widget_->resize(1280,1024);
  render_widget_->show();
  render_widget_->setWindowFlags(Qt::WindowSystemMenuHint | Qt::WindowTitleHint);

  render_widget_R_ = new rviz::RenderWidget(rviz::RenderSystem::get());
  render_widget_R_->setVisible(false);
  render_widget_R_->setWindowTitle("RVinci Right");
  render_widget_R_->resize(1280,1024);
  render_widget_R_->show();
  render_widget_R_->setWindowFlags(Qt::WindowSystemMenuHint | Qt::WindowTitleHint);

  window_ = render_widget_->getRenderWindow();
  window_->setVisible(false);
  window_->setAutoUpdated(false);
  window_->addListener(this);

  window_R_ = render_widget_R_->getRenderWindow();
  window_R_->setVisible(false);
  window_R_->setAutoUpdated(false);
  window_R_->addListener(this);

  camera_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  target_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  image_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode("Background");

  pubsubSetup();

  // publishWrenchGravity();

  start_measurement_PSM_[_LEFT] = false;
  start_measurement_PSM_[_RIGHT] = false;
  gravity_published_ = false;
  wrench_published_ = false;
  MTM_mm_ = false;
  coag_init_ = true;

  measurement_status_ = _BEGIN;
  measurement_status_PSM_ = _BEGIN;

  input_pos_[_LEFT].x = input_pos_[_LEFT].y = input_pos_[_LEFT].z = 0;
  input_pos_[_RIGHT].x = input_pos_[_RIGHT].y = input_pos_[_RIGHT].z = 0;
}
void rvinciDisplay::update(float wall_dt, float ros_dt)
{
  if( backgroundImage_[0] != NULL ){
    Ogre::Box b( 0, 0, 0,  
		 backgroundImage_[0]->getWidth(),
		 backgroundImage_[0]->getHeight(), 1 );
    texture_[0]->getBuffer()->blitFromMemory( backgroundImage_[0]->getPixelBox(), b );
  }

  if( backgroundImage_[1] != NULL ){
    Ogre::Box b( 0, 0, 0,
		 backgroundImage_[1]->getWidth(),
		 backgroundImage_[1]->getHeight(), 1 );
    texture_[1]->getBuffer()->blitFromMemory( backgroundImage_[1]->getPixelBox(), b );
  }

  cameraUpdate();
  window_ = render_widget_->getRenderWindow();
  window_->update(false);
  window_R_ = render_widget_R_->getRenderWindow();
  window_R_->update(false);

  rvmsg_.header.stamp = ros::Time::now();
  publisher_rvinci_.publish(rvmsg_);

  // switch (measurement_status_)
  // {
  //   case _BEGIN: ROS_INFO_STREAM("measurement status: _BEGIN"); break;
  //   case _START_MEASUREMENT: ROS_INFO_STREAM("measurement status: _START_MEASUREMENT"); break;
  //   case _MOVING: ROS_INFO_STREAM("measurement status: _MOVING"); break;
  //   case _END_MEASUREMENT: ROS_INFO_STREAM("measurement status: _END_MEASUREMENT"); break;
  // }

  publishMeasurementMarkers();

  if (!wrench_published_) 
  {
    publishWrench();
    wrench_published_ = true;
  }
  if (wrench_published_ && !gravity_published_)
  {
    publishGravity();
    gravity_published_ = true;
  }

  
}

//void rvinciDisplay::reset(){}
void rvinciDisplay::pubsubSetup()
{
  std::string subtopic = prop_ros_topic_->getStdString();
  rvmsg_.header.frame_id = "base_link";

  subscriber_input_ = nh_.subscribe<rvinci_input_msg::rvinci_input>(subtopic, 10, boost::bind(&rvinciDisplay::inputCallback,this,_1));
  subscriber_lcam_ = nh_.subscribe<sensor_msgs::Image>( "/jhu_daVinci/stereo_processed/left/image", 10, boost::bind(&rvinciDisplay::leftCallback,this,_1));
  subscriber_rcam_ = nh_.subscribe<sensor_msgs::Image>( "/jhu_daVinci/stereo_processed/right/image", 10, boost::bind(&rvinciDisplay::rightCallback,this,_1));
  subscriber_clutch_ = nh_.subscribe<sensor_msgs::Joy>( "/footpedals/clutch", 10, boost::bind(&rvinciDisplay::clutchCallback,this,_1));
  subscriber_camera_ = nh_.subscribe<sensor_msgs::Joy>( "/footpedals/camera", 10, boost::bind(&rvinciDisplay::cameraCallback,this,_1));
  subscriber_coag_ = nh_.subscribe<sensor_msgs::Joy>( "/footpedals/coag", 10, boost::bind(&rvinciDisplay::coagCallback,this,_1));
  subscriber_lgrip_ = nh_.subscribe<std_msgs::Bool>("/MTML/gripper/closed",10,boost::bind(&rvinciDisplay::gripCallback,this,_1,_LEFT));
  subscriber_rgrip_ = nh_.subscribe<std_msgs::Bool>("/MTMR/gripper/closed",10,boost::bind(&rvinciDisplay::gripCallback,this,_1,_RIGHT));
  
  //MTMR-PSM1, MTML-PSM2
  subscriber_MTML_ = nh_.subscribe<geometry_msgs::PoseStamped>("/MTML/measured_cp", 10, boost::bind(&rvinciDisplay::MTMCallback,this,_1, _LEFT));
  subscriber_MTMR_ = nh_.subscribe<geometry_msgs::PoseStamped>("/MTMR/measured_cp", 10, boost::bind(&rvinciDisplay::MTMCallback,this,_1, _RIGHT));
  subscriber_PSM1_ = nh_.subscribe<geometry_msgs::PoseStamped>("/PSM1/measured_cp", 10, boost::bind(&rvinciDisplay::PSMCallback,this,_1, _RIGHT));
  subscriber_PSM2_ = nh_.subscribe<geometry_msgs::PoseStamped>("/PSM2/measured_cp", 10, boost::bind(&rvinciDisplay::PSMCallback,this,_1, _LEFT));
  subscriber_mm_ = nh_.subscribe<std_msgs::Bool>("/rvinci_measurement_MTM", 10, boost::bind(&rvinciDisplay::measurementCallback,this,_1));
  subscriber_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>("/jhu_daVinci/stereo_processed/right/camera_info", 10, boost::bind(&rvinciDisplay::cameraInfoCallback,this,_1));

  publisher_rhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_right/update",10);
  publisher_lhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_left/update",10);
  // pub_robot_state_[_LEFT] = nh_.advertise<std_msgs::String>("/dvrk/MTML/set_robot_state",10);
  // pub_robot_state_[_RIGHT] = nh_.advertise<std_msgs::String>("/dvrk/MTMR/set_robot_state",10);
  
  publisher_markers = nh_.advertise<visualization_msgs::MarkerArray>("rvinci_markers", 10);
  publisher_rvinci_ = nh_.advertise<rvinci_input_msg::rvinci_input>("/rvinci_input_update",10);
  publisher_lwrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("/MTML/body/servo_cf", 10);
  publisher_rwrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("/MTMR/body/servo_cf", 10);
  publisher_lgravity_ = nh_.advertise<std_msgs::Bool>("/MTML/use_gravity_compensation", 10);
  publisher_rgravity_ = nh_.advertise<std_msgs::Bool>("/MTMR/use_gravity_compensation", 10);
}

void rvinciDisplay::leftCallback(const sensor_msgs::ImageConstPtr& img){

  if( buffer_[0] == NULL )
    { buffer_[0] = (unsigned char*)malloc( sizeof(unsigned char*)*img->height*img->step ); }

  if( backgroundImage_[0] == NULL ){
    backgroundImage_[0] = new Ogre::Image;
    backgroundImage_[0]->loadDynamicImage(buffer_[0], img->width, img->height, 1, Ogre::PF_BYTE_RGB);
  }

  if( texture_[0].isNull() ){
    texture_[0] = Ogre::TextureManager::getSingleton().createManual("BackgroundTextureLeft",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    Ogre::TEX_TYPE_2D,
                    img->width, img->height, 
                    0, 
                    Ogre::PF_BYTE_BGR,
                    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
    texture_[0]->loadImage( *(backgroundImage_[0]) );
  }

  if( material_[0].isNull() ){
    material_[0] = Ogre::MaterialManager::getSingleton().create("BackgroundMaterialLeft", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_[0]->getTechnique(0)->getPass(0)->createTextureUnitState("BackgroundTextureLeft");
    material_[0]->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
    material_[0]->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    material_[0]->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  }

  if( rect_[0] == NULL ){
    rect_[0] = new Ogre::Rectangle2D(true);
    rect_[0]->setCorners(-1.0, 1.0, 1.0, -1.0);
    rect_[0]->setMaterial("BackgroundMaterialLeft");
    rect_[0]->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
    rect_[0]->setVisibilityFlags( 0x0F );

    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    rect_[0]->setBoundingBox(aabInf);
    image_node_->attachObject(rect_[0]);

  }

  memcpy( (void*)buffer_[0], (void*)img->data.data(), img->step*img->height );
}

void rvinciDisplay::rightCallback(const sensor_msgs::ImageConstPtr& img){

  if( buffer_[1] == NULL )
    { buffer_[1] = (unsigned char*)malloc( sizeof(unsigned char*)*img->height*img->step ); }

  if( backgroundImage_[1] == NULL ){
    backgroundImage_[1] = new Ogre::Image;
    backgroundImage_[1]->loadDynamicImage(buffer_[1], img->width, img->height, 1, Ogre::PF_BYTE_RGB);
  }

  if( texture_[1].isNull() ){
    texture_[1] = Ogre::TextureManager::getSingleton().createManual("BackgroundTextureRight",
                    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                    Ogre::TEX_TYPE_2D,
                    img->width, img->height,
                    0, 
                    Ogre::PF_BYTE_BGR,
                    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
    texture_[1]->loadImage( *(backgroundImage_[1]) );
  }

  if( material_[1].isNull() ){
    material_[1] = Ogre::MaterialManager::getSingleton().create("BackgroundMaterialRight", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_[1]->getTechnique(0)->getPass(0)->createTextureUnitState("BackgroundTextureRight");
    material_[1]->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
    material_[1]->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    material_[1]->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  }

  if( rect_[1] == NULL ){
    rect_[1] = new Ogre::Rectangle2D(true);
    rect_[1]->setCorners(-1.0, 1.0, 1.0, -1.0);
    rect_[1]->setMaterial("BackgroundMaterialRight");
    rect_[1]->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
    rect_[1]->setVisibilityFlags( 0xF0 );

    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    rect_[1]->setBoundingBox(aabInf);
    image_node_->attachObject(rect_[1]);
  }

  memcpy( (void*)buffer_[1], (void*)img->data.data(), img->step*img->height );

}

void rvinciDisplay::gravityCompensation()
{
  std_msgs::String msg;
  if (prop_gravity_comp_->getBool())
  {
    msg.data = "DVRK_GRAVITY_COMPENSATION";
  }
  else
  {
    msg.data = "DVRK_READY";
  }

  // pub_robot_state_[_LEFT].publish(msg);
  // pub_robot_state_[_RIGHT].publish(msg);
}

void rvinciDisplay::inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input)
{
  // camera_mode_ = r_input->camera;
  clutch_mode_ = r_input->clutch;

  // if (MTM_mm_) {
  if (!clutch_mode_)  // be able to clutch cursors
  {  
    Ogre::Quaternion camor = camera_[_LEFT]->getRealOrientation();
    int grab[2];
    for (int i = 0; i<2; ++i)  //getting absolute and delta position of grippers, for use in cam and cursor.
    {
      Ogre::Vector3 old_input = input_pos_[i];
      geometry_msgs::Pose pose = r_input->gripper[i].pose;

      input_pos_[i] = Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z);// + cursor_offset_[i];
      input_pos_[i] *= prop_input_scalar_->getVector();
      input_change_[i] = (input_pos_[i] - old_input);
      input_change_[i].y *= 2.5;  //magnify yz cursor movements
      input_change_[i].z *= 2.5;
    }

    for (int i=0; i<2; ++i)
    {
      geometry_msgs::Pose pose = r_input->gripper[i].pose;
      cursor_[i].position.x += input_change_[i].x;
      cursor_[i].position.y -= input_change_[i].y;
      cursor_[i].position.z -= input_change_[i].z;
      cursor_[i].orientation.x = pose.orientation.x;
      cursor_[i].orientation.y = pose.orientation.y;
      cursor_[i].orientation.z = pose.orientation.z;
      cursor_[i].orientation.w = pose.orientation.w;
      grab[i] = getaGrip(r_input->gripper[i].grab, i);
    }

    prop_cam_focus_->setVector(input_pos_[_RIGHT]);
    publishCursorUpdate(grab);

    /*
      * inital_vect is constantly calculated, to set origin vector between grippers when
      * camera mode is triggered.
      */
    initial_cvect_ = (input_pos_[_LEFT] - input_pos_[_RIGHT]);
    initial_cvect_.normalise(); //normalise, otherwise issues when doing v1.getRotationto(v2);
  }
  else  //to avoid an erroneously large input_update_ following clutched movement
  {
    for(int i = 0; i<2; ++i)
    {
      geometry_msgs::Pose pose = r_input->gripper[i].pose;
      input_pos_[i] = Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z);// + cursor_offset_[i];
      input_pos_[i]*= prop_input_scalar_->getVector();
      initial_cvect_ = (input_pos_[_LEFT] - input_pos_[_RIGHT]);
      initial_cvect_.normalise();
    }
  }
  // }
}

void rvinciDisplay::publishCursorUpdate(int grab[2])
{
  //fixed frame is a parent member from RViz Display, pointing to selected world frame in rviz;
  std::string frame = context_->getFixedFrame().toStdString();
  interaction_cursor_msgs::InteractionCursorUpdate lhcursor;
  interaction_cursor_msgs::InteractionCursorUpdate rhcursor;

  lhcursor.pose.header.frame_id = frame;
  lhcursor.pose.header.stamp = ros::Time::now();
  lhcursor.pose.pose = cursor_[_LEFT];
  lhcursor.button_state = grab[_LEFT];

  rhcursor.pose.header.frame_id = frame;
  rhcursor.pose.header.stamp = ros::Time::now();
  rhcursor.pose.pose = cursor_[_RIGHT];
  rhcursor.button_state = grab[_RIGHT];

  // ROS_INFO_STREAM("left cursor: "<<cursor_[_LEFT].position.x<<" "<<cursor_[_LEFT].position.y<<" "<<cursor_[_LEFT].position.z);
  // ROS_INFO_STREAM("right cursor: "<<cursor_[_RIGHT].position.x<<" "<<cursor_[_RIGHT].position.y<<" "<<cursor_[_RIGHT].position.z);

  publisher_lhcursor_.publish(lhcursor);
  publisher_rhcursor_.publish(rhcursor);
}

int rvinciDisplay::getaGrip(bool grab, int i)
{
  //if "pinched" -> 0, if released -> 1
  if(!grab && prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 2;//Grab object
    }
  if(!grab && !prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 1;//hold object
    }
  if(grab && !prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 3;//Release object
    }
  if(grab && prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 0;//none
   }
}

void rvinciDisplay::cameraSetup()
{
  Ogre::ColourValue bg_color = context_->getViewManager()->getRenderPanel()->getViewport()->getBackgroundColour();
  window_ = render_widget_->getRenderWindow();
  window_R_ = render_widget_R_->getRenderWindow();

  camera_[_LEFT] = scene_manager_->createCamera("Left Camera");
  camera_[_RIGHT] = scene_manager_->createCamera("Right Camera");

  camera_node_->attachObject(camera_[_LEFT]);
  viewport_[_LEFT] = window_->addViewport(camera_[_LEFT]);
  viewport_[_LEFT]->setBackgroundColour(bg_color);

  camera_node_->attachObject(camera_[_RIGHT]);
  viewport_[_RIGHT] = window_R_->addViewport(camera_[_RIGHT]);
  viewport_[_RIGHT]->setBackgroundColour(bg_color);
  
  viewport_[_LEFT]->setVisibilityMask( 0x0F );
  viewport_[_RIGHT]->setVisibilityMask( 0xF0 );

  cameraReset();
}

void rvinciDisplay::cameraReset()
{
  camera_pos_= Ogre::Vector3(0.0f,0.0f,0.0f);
  camera_node_->setOrientation(1,0,0,0);
  camera_node_->setPosition(camera_pos_);
  for (int i=0; i<2; ++i)
  {
    camera_[i]->setNearClipDistance(0.01f);
    camera_[i]->setFarClipDistance(100.0f);
    camera_[i]->setFixedYawAxis(true, camera_node_->getOrientation() * Ogre::Vector3::UNIT_Z);
    camera_[i]->setPosition(camera_offset_ - camera_ipd_ + 2*i*camera_ipd_);
    camera_[i]->lookAt(camera_node_->getPosition());

    cursor_[i].position.x = (2*i - 1)*0.6;
    cursor_[i].position.y = 0;
    cursor_[i].position.z = 0;
  }

  prop_cam_reset_->setValue(QVariant(false));
}

void rvinciDisplay::cameraUpdate()
{
  bool getTransform_ret;
  getTransform_ret = frame_manager_.getTransform(cam_header_, camera_pos_, camera_ori_);
  if (img_width_ < 1 || img_height_ < 1) {
    return;
  }

  // ROS_INFO_STREAM("getTransform return value: "<<getTransform_ret);
  camera_ori_ = camera_ori_ * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  if (fx_ == 0) return;

  double baseline = -1 * tx_/fx_;
  Ogre::Vector3 right = camera_ori_ * Ogre::Vector3::UNIT_X;

  // ROS_INFO_STREAM("img width: "<<img_width_<<"img height: "<<img_height_);

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;
  proj_matrix[0][0] = 2.0 * fx_ / img_width_;
  proj_matrix[1][1] = 2.0 * fy_ / img_height_;
  proj_matrix[0][2] = 2.0 * (0.5 - cx_ / img_width_);
  proj_matrix[1][2] = 2.0 * (cy_ / img_height_ - 0.5);
  proj_matrix[2][2] = -(100 + 0.01) / (100 - 0.01);
  proj_matrix[2][3] = -2.0 * (100 * 0.01) / (100 - 0.01);
  proj_matrix[3][2] = -1;

  camera_[_LEFT]->setCustomProjectionMatrix(true, proj_matrix);
  camera_[_RIGHT]->setCustomProjectionMatrix(true, proj_matrix);

  camera_[_LEFT]->setPosition(camera_pos_);
  // camera_[_LEFT]->lookAt(0, 0, 0);
  Ogre::Vector3 baseline_offset = baseline * right;
  camera_[_LEFT]->setOrientation(camera_ori_);
  camera_[_RIGHT]->setPosition(camera_pos_ + baseline_offset);
  // camera_[_RIGHT]->lookAt(0, 0, 0);
  camera_[_RIGHT]->setOrientation(camera_ori_);

  frame_manager_.setFixedFrame("base_link");

  // ROS_INFO_STREAM(frame_manager_.getFixedFrame());
  // std::string errmsg;
  // frame_manager_.frameHasProblems("base_link", ros::Time(0), errmsg);
  // ROS_INFO_STREAM("frame has problem: "<<errmsg);
  // frame_manager_.transformHasProblems("base_link", ros::Time(0), errmsg);
  // ROS_INFO_STREAM("transform has problem: "<<errmsg);
  // frame_manager_.frameHasProblems("jhu_daVinci_stereo_frame", ros::Time(0), errmsg);
  // ROS_INFO_STREAM("frame has problem: "<<errmsg);
  // frame_manager_.transformHasProblems("jhu_daVinci_stereo_frame", ros::Time(0), errmsg);
  // ROS_INFO_STREAM("transform has problem: "<<errmsg);

  // ROS_INFO_STREAM("camera pos: "<<camera_pos_.x<<" "<<camera_pos_.y<<" "<<camera_pos_.z);
  // ROS_INFO_STREAM("camera offset: "<<baseline);
}

void rvinciDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  cameraUpdate();
}

void rvinciDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  window_ = render_widget_->getRenderWindow();
  window_->swapBuffers();
  window_R_ = render_widget_R_->getRenderWindow();
  window_R_->swapBuffers();
}

void rvinciDisplay::onEnable()
{
  if(!camera_[_LEFT])
  {
    cameraSetup();
  }
  render_widget_->setVisible(true);
  render_widget_R_->setVisible(true);
  cameraReset();
}

void rvinciDisplay::onDisable()
{
  render_widget_ ->setVisible(false);
  render_widget_R_ ->setVisible(false);
}

visualization_msgs::Marker rvinciDisplay::makeMarker(geometry_msgs::Pose p, int id)
{
  visualization_msgs::Marker marker;
  // marker.header.frame_id = "base_link";
  marker.header.frame_id = "jhu_daVinci_stereo_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = id;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose = p; 
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.7;

  marker.lifetime = ros::Duration();
  return marker;
}

visualization_msgs::Marker rvinciDisplay::makeTextMessage(geometry_msgs::Pose p, std::string msg, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "text_message";
  marker.id = id;

  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.pose.position = p.position;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.2;
  marker.color.r = 0.8;
  marker.color.g = 0.8;
  marker.color.b = 0.8;
  marker.color.a = 1.0;
  marker.text = msg;

  marker.lifetime = ros::Duration();
  return marker;
}

visualization_msgs::Marker rvinciDisplay::makeLineMarker(geometry_msgs::Point p1, geometry_msgs::Point p2, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  // marker.header.frame_id = "jhu_daVinci_stereo_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "line_strip";
  marker.id = id;

  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.scale.x = 0.02;
  marker.color.b = 0.8;
  marker.color.a = 0.7;

  return marker;
}

visualization_msgs::Marker rvinciDisplay::deleteMarker(int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = id;
  marker.action = visualization_msgs::Marker::DELETEALL;

  return marker;
}

void rvinciDisplay::publishMeasurementMarkers()
{
  visualization_msgs::MarkerArray marker_arr;
  geometry_msgs::Pose text_pose, distance_pose;
  text_pose.position.x = text_pose.position.y = text_pose.position.z = 0.0;
  text_pose.orientation.x = text_pose.orientation.y = text_pose.orientation.z = 0.0;
  text_pose.orientation.w = 1.0;
  distance_pose.position.x = distance_pose.position.y = 0.0;
  distance_pose.position.z = 1.0;
  distance_pose.orientation.x = distance_pose.orientation.y = distance_pose.orientation.z = 0.0;
  distance_pose.orientation.w = 1.0;

  // if (MTM_mm_) {  // MTM measurement
  //   // ROS_INFO_STREAM("MTM measurement");
  //   switch (measurement_status_)
  //   {
  //     case _BEGIN:
  //       marker_arr.markers.push_back( deleteMarker(_DELETE) );
  //       break;
  //     case _START_MEASUREMENT:
  //       marker_arr.markers.push_back( makeTextMessage(text_pose, "start measurement", _STATUS_TEXT) );
  //       marker_arr.markers.push_back( makeMarker(cursor_[marker_side_], _START_POINT) );
  //       measurement_start_ = cursor_[marker_side_];
  //       break;
  //     case _MOVING:
  //       marker_arr.markers.push_back( makeTextMessage(text_pose, "moving", _STATUS_TEXT) );
  //       marker_arr.markers.push_back( makeTextMessage(distance_pose, 
  //         std::to_string(calculateDistance(measurement_start_, cursor_[marker_side_])*1.45)+" cm", _DISTANCE_TEXT) );
  //       marker_arr.markers.push_back( makeMarker(measurement_start_, _START_POINT) );
  //       marker_arr.markers.push_back( makeMarker(cursor_[marker_side_], _END_POINT) );
  //       marker_arr.markers.push_back( makeLineMarker(measurement_start_.position, cursor_[marker_side_].position, _LINE) );
  //       measurement_end_ = cursor_[marker_side_];
  //       break;
  //     case _END_MEASUREMENT:
  //       marker_arr.markers.push_back( makeTextMessage(text_pose, "end measurement", _STATUS_TEXT) );
  //       marker_arr.markers.push_back( makeTextMessage(distance_pose, 
  //         std::to_string(calculateDistance(measurement_start_, measurement_end_)*1.45)+" cm", _DISTANCE_TEXT) );
  //       marker_arr.markers.push_back( makeMarker(measurement_start_, _START_POINT) );
  //       marker_arr.markers.push_back( makeMarker(measurement_end_, _END_POINT) );
  //       marker_arr.markers.push_back( makeLineMarker(measurement_start_.position, measurement_end_.position, _LINE) );
  //       break;
  //   }
  // } 
  // else {  // PSM measurement
  //   // ROS_INFO_STREAM("PSM measurement");
  //   switch(measurement_status_PSM_)
  //   {
  //     case _BEGIN:
  //       marker_arr.markers.push_back( deleteMarker(_DELETE) );
  //       break;
  //     case _START_MEASUREMENT:
  //       marker_arr.markers.push_back( makeTextMessage(text_pose, "start measurement", _STATUS_TEXT) );
  //       // marker_arr.markers.push_back( makeMarker(PSM_pose_start_, _START_POINT) );
  //       // marker_arr.markers.push_back( makeMarker(PSM_pose_end_, _END_POINT) );
  //       marker_arr.markers.push_back( makeTextMessage(distance_pose, 
  //         std::to_string( (calculateDistance(PSM_pose_start_, PSM_pose_end_)-.150)*100 )+" cm", _DISTANCE_TEXT) );
  //       // marker_arr.markers.push_back( makeLineMarker(PSM_pose_start_.position, PSM_pose_end_.position, _LINE) );
  //       break;
  //     case _END_MEASUREMENT:
  //       marker_arr.markers.push_back( makeTextMessage(text_pose, "end measurement", _STATUS_TEXT) );
  //       marker_arr.markers.push_back( makeTextMessage(distance_pose, 
  //         std::to_string( (calculateDistance(PSM_pose_start_, PSM_pose_end_)-.150)*100 )+" cm", _DISTANCE_TEXT) );
  //       // marker_arr.markers.push_back( makeLineMarker(PSM_pose_start_.position, PSM_pose_end_.position, _LINE) );
  //       break;
  //   }
  // }

  // PSM marker location test
  marker_arr.markers.push_back( makeMarker(PSM_pose_start_, _START_POINT) );
  marker_arr.markers.push_back( makeMarker(PSM_pose_end_, _END_POINT) );

  publisher_markers.publish(marker_arr);
}

void rvinciDisplay::clutchCallback(const sensor_msgs::Joy::ConstPtr& msg) 
{
  // buttons: 0 - released, 1 - pressed, 2 - quick tap
  rvmsg_.clutch = msg->buttons[0];
}

void rvinciDisplay::cameraCallback(const sensor_msgs::Joy::ConstPtr& msg) 
{
  // buttons: 0 - released, 1 - pressed, 2 - quick tap
  // rvmsg_.camera = msg->buttons[0]; 

  if (msg->buttons[0] == 2) camera_quick_tap_ = true;
  else camera_quick_tap_ = false;

  // MTM measurement - if camera quick tapped while one gripper closed, begin measurement
  if (camera_quick_tap_ && !rvmsg_.gripper[marker_side_].grab)
  {
    switch (measurement_status_)
    {
      case _BEGIN: 
        measurement_status_ = _START_MEASUREMENT; 
        break;
      case _START_MEASUREMENT: 
        measurement_status_ = _MOVING; 
        break;
      case _MOVING: 
        measurement_status_ = _END_MEASUREMENT; 
        break;
      case _END_MEASUREMENT: 
        measurement_status_ = _BEGIN; 
        break;
    }
  }
  else if (camera_quick_tap_ && measurement_status_ == _END_MEASUREMENT)
  {
    measurement_status_ = _BEGIN; 
  }

  // PSM measurement - if camera quick tapped while left & right gripper closed, end measurement
  if (camera_quick_tap_ && measurement_status_PSM_ == _END_MEASUREMENT)
  {
    measurement_status_PSM_ = _BEGIN;
  }
  else if (camera_quick_tap_ && start_measurement_PSM_[_LEFT] && start_measurement_PSM_[_RIGHT] && measurement_status_PSM_ == _START_MEASUREMENT)
  {
    measurement_status_PSM_ = _END_MEASUREMENT;
  }
}

void rvinciDisplay::MTMCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
{
  //Offsets to set davinci at 0 x and y, with an x offset for each gripper.
  rvmsg_.gripper[i].pose = msg->pose;
  rvmsg_.gripper[i].pose.position.x *= -1;
  rvmsg_.gripper[i].pose.position.y *= -1;
  rvmsg_.gripper[i].pose.position.z -= 0.4;
}

void rvinciDisplay::PSMCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
{
  // MTMR-PSM1, MTML-PSM2
  // if (start_measurement_PSM_[_LEFT] && start_measurement_PSM_[_RIGHT] && measurement_status_PSM_ == _START_MEASUREMENT)
  // {
  //   switch (i)
  //   {
  //     case _LEFT: 
  //       PSM_pose_start_ = msg->pose; 
  //       break;
  //     case _RIGHT: 
  //       PSM_pose_end_ = msg->pose; 
  //       break;
  //   }
  // }

  ROS_INFO_STREAM("actual PSM start: "<<msg->pose.position.x<<" "<<msg->pose.position.y<<" "<<msg->pose.position.z);
  // ROS_INFO_STREAM("PSM end: "<<PSM_pose_end_.position.x<<" "<<PSM_pose_end_.position.y<<" "<<PSM_pose_end_.position.z);

  // PSM marker location test
  // if (start_measurement_PSM_[_LEFT] && start_measurement_PSM_[_RIGHT] && measurement_status_PSM_ == _START_MEASUREMENT)
  // {
    switch (i)
    {
      case _LEFT: 
        PSM_pose_start_ = msg->pose; 
        // PSM_pose_start_.position.x -= 0.15;
        // PSM_pose_start_.position.y -= 0.15;
        // PSM_pose_start_.position.z -= 0.15;
        PSM_pose_start_.position.x *= -20;
        PSM_pose_start_.position.y *= -20;
        PSM_pose_start_.position.z *= 100;
        break;
      case _RIGHT: 
        PSM_pose_end_ = msg->pose; 
        // PSM_pose_end_.position.x -= 0.15;
        // PSM_pose_end_.position.y -= 0.15;
        // PSM_pose_end_.position.z -= 0.15;
        PSM_pose_end_.position.x *= -20;
        PSM_pose_end_.position.y *= -20;
        PSM_pose_end_.position.z *= 100;
        break;
    }
  // }

  ROS_INFO_STREAM("PSM start: "<<PSM_pose_start_.position.x<<" "<<PSM_pose_start_.position.y<<" "<<PSM_pose_start_.position.z);
  // ROS_INFO_STREAM("PSM end: "<<PSM_pose_end_.position.x<<" "<<PSM_pose_end_.position.y<<" "<<PSM_pose_end_.position.z);
}

void rvinciDisplay::gripCallback(const std_msgs::Bool::ConstPtr& grab, int i)
{
  // MTM measurement
  if (!rvmsg_.gripper[i].grab && grab->data && marker_side_ == i && measurement_status_ != _END_MEASUREMENT)
  {
    measurement_status_ = _BEGIN;  //if gripper released during measurement, status back to BEGIN
  }
  if (!grab->data && measurement_status_ == _BEGIN) 
  {
    marker_side_ = i;  //gripper closed from another arm shouldnt interrupt the measurement process
  }

  // PSM measurement
  start_measurement_PSM_[i] = !grab->data;
  if (start_measurement_PSM_[_LEFT] && start_measurement_PSM_[_RIGHT] && measurement_status_PSM_ == _BEGIN)
  {
    measurement_status_PSM_ = _START_MEASUREMENT;
  }
  if (!start_measurement_PSM_[_LEFT] && !start_measurement_PSM_[_RIGHT] && measurement_status_PSM_ != _END_MEASUREMENT)
  {
    measurement_status_PSM_ = _BEGIN;
  }

  //if "pinched" -> false, if released -> true
  rvmsg_.gripper[i].grab = grab->data;
}

void rvinciDisplay::coagCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  coag_mode_ = msg->buttons[0];

  if (coag_init_) {
    coag_init_ = false;
    return;
  }

  if (coag_mode_ == 1) {
    cursor_[_LEFT].position.x -= 10;
    cursor_[_RIGHT].position.x -= 10;
  }
  else if (coag_mode_ == 0){
    int grab[2];
    cursor_[_LEFT].position.x += 10;
    cursor_[_RIGHT].position.x += 10;
    grab[_LEFT] = 0;
    grab[_RIGHT] = 0;
    publishCursorUpdate(grab);
  }
}

void rvinciDisplay::measurementCallback(const std_msgs::Bool::ConstPtr& msg) 
{
  MTM_mm_ = msg->data;
}

void rvinciDisplay::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  fx_ = msg->P[0];
  tx_ = msg->P[3];
  cx_ = msg->P[2];
  cy_ = msg->P[6];
  fy_ = msg->P[5];
  cam_header_ = msg->header;
  img_width_ = msg->width;
  img_height_ = msg->height;
}

double rvinciDisplay::calculateDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return std::sqrt( std::pow(p1.position.x-p2.position.x, 2)
                    + std::pow(p1.position.y-p2.position.y, 2)
                    + std::pow(p1.position.z-p2.position.z, 2) );
}

void rvinciDisplay::publishWrench()
{
  ROS_INFO_STREAM("publishWrench");
  geometry_msgs::WrenchStamped wr;
  wr.header.stamp = ros::Time::now();
  wr.wrench.force.x = wr.wrench.force.y = wr.wrench.force.z = 0;
  wr.wrench.torque.x = wr.wrench.torque.y = wr.wrench.torque.z = 0;
  for (int i=0; i<5; i++) 
  {
    publisher_lwrench_.publish(wr);
    publisher_rwrench_.publish(wr);
  }
}

void rvinciDisplay::publishGravity()
{
  ROS_INFO_STREAM("publishGravity");
  std_msgs::Bool gravity;
  gravity.data = true;
  for (int i=0; i<5; i++) 
  {
    publisher_lgravity_.publish(gravity);
    publisher_rgravity_.publish(gravity);
  }
}

}//namespace rvinci
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rvinci::rvinciDisplay, rviz::Display )
