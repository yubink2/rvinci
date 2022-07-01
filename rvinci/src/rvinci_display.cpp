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

#include <interaction_cursor_msgs/InteractionCursorUpdate.h>
#include <rvinci_input_msg/rvinci_input.h>
#include <std_msgs/String.h>
#include "rvinci/rvinci_display.h"

#define _LEFT 0
#define _RIGHT 1


namespace rvinci
{
rvinciDisplay::rvinciDisplay()
  : render_widget_(0)
  , camera_node_(0)
  , window_(0)
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
  for(int i = 0; i<2; ++i)
  {
    if (viewport_[i])
   {
      window_->removeViewport(0);
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
  delete render_widget_;
//  delete prop_manual_coords_;
  delete prop_cam_focus_;
  delete prop_camera_posit_;
  delete prop_input_scalar_;
}
void rvinciDisplay::onInitialize()
{
  render_widget_ = new rviz::RenderWidget(rviz::RenderSystem::get());
  render_widget_->setVisible(false);
  render_widget_->setWindowTitle("RVinci");
  render_widget_->resize(3360,1050);
  render_widget_->show();
  render_widget_->setWindowFlags(Qt::WindowSystemMenuHint | Qt::WindowTitleHint);
  window_ = render_widget_->getRenderWindow();
  window_->setVisible(false);
  window_->setAutoUpdated(false);
  window_->addListener(this);
  camera_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  target_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  image_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode("Background");

  pubsubSetup();
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

  // cameraUpdate();
  window_ = render_widget_->getRenderWindow();
  window_->update(false);

  // makeMarker();
  rvmsg_.header.stamp = ros::Time::now();
  publisher_rvinci_.publish(rvmsg_);
}

//void rvinciDisplay::reset(){}
void rvinciDisplay::pubsubSetup()
{
  std::string subtopic = prop_ros_topic_->getStdString();
  // ROS_INFO_STREAM("*** SUBTOPIC: "<<subtopic);
  rvmsg_.header.frame_id = "base_link";

  subscriber_input_ = nh_.subscribe<rvinci_input_msg::rvinci_input>(subtopic, 10, boost::bind(&rvinciDisplay::inputCallback,this,_1));
  subscriber_lcam_ = nh_.subscribe<sensor_msgs::Image>( "/jhu_daVinci/left/decklink/jhu_daVinci_left/image_raw", 10, boost::bind(&rvinciDisplay::leftCallback,this,_1));
  subscriber_rcam_ = nh_.subscribe<sensor_msgs::Image>( "/jhu_daVinci/right/decklink/jhu_daVinci_right/image_raw", 10, boost::bind(&rvinciDisplay::rightCallback,this,_1));
  // subscriber_lcam_ = nh_.subscribe<sensor_msgs::Image>( "/jhu_daVinci/left/decklink/jhu_daVinci_left/image_raw", 10, boost::bind(&rvinciDisplay::leftCallback,this,_1));
  // subscriber_rcam_ = nh_.subscribe<sensor_msgs::Image>( "/jhu_daVinci/right/decklink/jhu_daVinci_right/image_raw", 10, boost::bind(&rvinciDisplay::rightCallback,this,_1));
  subscriber_clutch_ = nh_.subscribe<sensor_msgs::Joy>( "/footpedals/clutch", 10, boost::bind(&rvinciDisplay::clutchCallback,this,_1));
  subscriber_MTML_ = nh_.subscribe<geometry_msgs::PoseStamped>("/MTML/measured_cp", 10, boost::bind(&rvinciDisplay::MTMCallback,this,_1, _LEFT));
  subscriber_MTMR_ = nh_.subscribe<geometry_msgs::PoseStamped>("/MTMR/measured_cp", 10, boost::bind(&rvinciDisplay::MTMCallback,this,_1, _RIGHT));

  publisher_rhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_right/update",10);
  publisher_lhcursor_ = nh_.advertise<interaction_cursor_msgs::InteractionCursorUpdate>("rvinci_cursor_left/update",10);
  pub_robot_state_[_LEFT] = nh_.advertise<std_msgs::String>("/dvrk/MTML/set_robot_state",10);
  pub_robot_state_[_RIGHT] = nh_.advertise<std_msgs::String>("/dvrk/MTMR/set_robot_state",10);
  
  marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  publisher_rvinci_ = nh_.advertise<rvinci_input_msg::rvinci_input>("/rvinci_input_update",10);
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

  pub_robot_state_[_LEFT].publish(msg);
  pub_robot_state_[_RIGHT].publish(msg);
}

void rvinciDisplay::inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input)
{
   Ogre::Quaternion orshift(0,0,-sqrt(0.5),sqrt(0.5));  //shifts incoming davinci orientation into world frame
   orshift=orshift* Ogre::Quaternion(0,0,1,0);
   Ogre::Quaternion inori[2];

  camera_mode_ = r_input->camera;
  clutch_mode_ = r_input->clutch;

  // ROS_INFO_STREAM("*** CLUTCH MODE? "<<clutch_mode_);
  // ROS_INFO_STREAM("*** CAMERA MODE? "<<camera_mode_);

  if(!clutch_mode_)
  {
    // Ogre::Quaternion camor =camera_[_LEFT]->getRealOrientation();
    int grab[2];
    for (int i = 0; i<2; ++i)  //getting absolute and delta position of grippers, for use in cam and cursor.
    {
      Ogre::Vector3 old_input = input_pos_[i];
      geometry_msgs::Pose pose = r_input->gripper[i].pose;

      input_pos_[i] = Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z);// + cursor_offset_[i];
      input_pos_[i]*=prop_input_scalar_->getVector();
      inori[i] = Ogre::Quaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
      // inori[i]= camor*(orshift*inori[i]);

      input_change_[i] = (input_pos_[i] - old_input);
    }

    geometry_msgs::Pose curspose;

    for (int i = 0; i<2; ++i)
    {
      cursor_[i].position.x += input_change_[i].x;
      cursor_[i].position.y += input_change_[i].y;
      cursor_[i].position.z += input_change_[i].z;
      cursor_[i].orientation.x = inori[i].x;
      cursor_[i].orientation.y = inori[i].y;
      cursor_[i].orientation.z = inori[i].z;
      cursor_[i].orientation.w = inori[i].w;
      grab[i] = getaGrip(r_input->gripper[i].grab, i);
    }
    publishCursorUpdate(grab);
    /*
      * inital_vect is constantly calculated, to set origin vector between grippers when
      * camera mode is triggered.
      */
    initial_cvect_ = (input_pos_[_LEFT] - input_pos_[_RIGHT]);
    initial_cvect_.normalise(); //normalise, otherwise issues when doing v1.getRotationto(v2);
  }
  else//to avoid an erroneously large input_update_ following clutched movement
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
}

void rvinciDisplay::publishCursorUpdate(int grab[2])
{
  // ROS_INFO_STREAM("*** publishCursorUpdate called");
  //fixed frame is a parent member from RViz Display, pointing to selected world frame in rviz;
  std::string frame = context_->getFixedFrame().toStdString();
  interaction_cursor_msgs::InteractionCursorUpdate lhcursor;
  interaction_cursor_msgs::InteractionCursorUpdate rhcursor;

  rhcursor.pose.header.frame_id = frame;
  rhcursor.pose.header.stamp = ros::Time::now();
  rhcursor.pose.pose = cursor_[_RIGHT];
  rhcursor.button_state = grab[_RIGHT];

  lhcursor.pose.header.frame_id = frame;
  lhcursor.pose.header.stamp = ros::Time::now();
  lhcursor.pose.pose = cursor_[_LEFT];
  lhcursor.button_state = grab[_LEFT];

  ROS_INFO_STREAM("LEFT CURSOR POSE: "<<lhcursor.pose.pose.position.x<<" "<<lhcursor.pose.pose.position.y<<" "<<lhcursor.pose.pose.position.z);
  ROS_INFO_STREAM("RIGHT CURSOR POSE: "<<rhcursor.pose.pose.position.x<<" "<<rhcursor.pose.pose.position.y<<" "<<rhcursor.pose.pose.position.z);

  publisher_rhcursor_.publish(rhcursor);
  publisher_lhcursor_.publish(lhcursor);

}
int rvinciDisplay::getaGrip(bool grab, int i)
{
  if(grab && !prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 2;//Grab object
    }
  if(grab && prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 1;//hold object
    }
  if(!grab && prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 3;//Release object
    }
  if(!grab && !prev_grab_[i])
    {
    prev_grab_[i] = grab;
    return 0;//none
   }
}
void rvinciDisplay::cameraSetup()
{
  // ROS_INFO_STREAM("*** cameraSetup()");
  Ogre::ColourValue bg_color = context_->getViewManager()->getRenderPanel()->getViewport()->getBackgroundColour();
  window_ = render_widget_->getRenderWindow();
  camera_[_LEFT] = scene_manager_->createCamera("Left Camera");
  camera_[_RIGHT] = scene_manager_->createCamera("Right Camera");
  for(int i = 0; i<2; ++i)
    {
      camera_node_->attachObject(camera_[i]);
      viewport_[i] = window_->addViewport(camera_[i],i,0.5f*i,0.0f,0.5f,1.0f);//,0,0.5f,0,0.5f,1.0f);
      viewport_[i]->setBackgroundColour(bg_color);
    }
  
  viewport_[0]->setVisibilityMask( 0x0F );
  viewport_[1]->setVisibilityMask( 0xF0 );

  cameraReset();
}

void rvinciDisplay::cameraReset()
{
  // ROS_INFO_STREAM("*** cameraReset()");
  camera_pos_= Ogre::Vector3(0.0f,0.0f,0.0f);
  camera_node_->setOrientation(1,0,0,0);
  camera_node_->setPosition(camera_pos_);
  for (int i = 0; i<2; ++i)
  {
    camera_[i]->setNearClipDistance(0.01f);
    camera_[i]->setFarClipDistance(10000.0f);
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
/*Manual camera control doesn't work perfectly, but is deemed unnecessary.
 Code left for future use, if desired.

    if(prop_manual_coords_->getBool())
   {
     camera_pos_ = Ogre::Vector3(prop_camera_posit_->getVector());
     camera_node_->setPosition(camera_pos_ - camera_offset_);
     property_camrot_->setQuaternion(camera_[_LEFT]->getRealOrientation());
     camera_[_LEFT]->lookAt(prop_cam_focus_->getVector());
     camera_[_RIGHT]->lookAt(prop_cam_focus_->getVector());
    }*/
  // ROS_INFO_STREAM("*** cameraUpdate()");
  if(camera_mode_)
  {

 
      Ogre::Vector3 newvect = input_pos_[_LEFT] - input_pos_[_RIGHT];
      newvect.normalise();
      Ogre::Quaternion camrot  = initial_cvect_.getRotationTo(newvect);

      camera_pos_ = Ogre::Vector3(camera_pos_ - ((input_change_[_RIGHT] + input_change_[_LEFT])));
      camera_node_->setOrientation(camera_node_->getOrientation()*camrot.Inverse());
      camera_node_->setPosition(camera_pos_);

      initial_cvect_ = newvect;

      property_camrot_->setQuaternion(camera_[_LEFT]->getRealOrientation());
      prop_camera_posit_->setVector(camera_pos_ + property_camrot_->getQuaternion()*camera_[_LEFT]->getPosition());
      prop_cam_focus_->setVector(camera_node_->getPosition());
}
}
void rvinciDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  // cameraUpdate();
}
void rvinciDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  window_ = render_widget_->getRenderWindow();
  window_->swapBuffers();
}
void rvinciDisplay::onEnable()
{
  if(!camera_[_LEFT])
  {
  cameraSetup();
  }
  render_widget_->setVisible(true);
  cameraReset();
}
void rvinciDisplay::onDisable()
{
  render_widget_ ->setVisible(false);
}

//visualization
void rvinciDisplay::makeMarker()
{
  // std::cout<<marker_pub.getTopic()<<std::endl;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
}

void rvinciDisplay::deleteMarker()
{
  // std::cout<<marker_pub.getTopic()<<std::endl;
  // ROS_INFO_STREAM("*** DELETE MARKER");

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::DELETE;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
}

void rvinciDisplay::clutchCallback(const sensor_msgs::Joy::ConstPtr& msg) 
{
  // ROS_INFO_STREAM("*** CLUTCH CALLBACK");
  // buttons: 0 - released, 1 - pressed, 2 - quick tap
  rvmsg_.clutch = msg->buttons[0];
}

void rvinciDisplay::MTMCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
{
  //Offsets to set davinci at 0 x and y, with an x offset for each gripper.
  rvmsg_.gripper[i].pose = msg->pose;
  rvmsg_.gripper[i].pose.position.x *= -1;
  rvmsg_.gripper[i].pose.position.z -= 0.4;
}

}//namespace rvinci
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rvinci::rvinciDisplay, rviz::Display )
