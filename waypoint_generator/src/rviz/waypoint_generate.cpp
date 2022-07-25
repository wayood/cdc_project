 /*
  * Copyright (c) 2012, Willow Garage, Inc.
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
 
 #include <tf/transform_listener.h>
 
 #include <geometry_msgs/PoseWithCovarianceStamped.h>
 
 #include "rviz/display_context.h"
 #include "rviz/properties/string_property.h"
 #include "rviz/properties/float_property.h"

 #include "waypoint_generate.h"
 
 namespace rviz
 {
 Waypoint_generate::Waypoint_generate()
 {
   shortcut_key_ = 's';
   std_dev_x_ = new FloatProperty("X std deviation", 0.5, "X standard deviation for initial pose [m]", getPropertyContainer());
   std_dev_y_ = new FloatProperty("Y std deviation", 0.5, "Y standard deviation for initial pose [m]", getPropertyContainer());
   std_dev_theta_ = new FloatProperty("Theta std deviation", M_PI / 12.0, "Theta standard deviation for initial pose [rad]", getPropertyContainer());

   std_dev_x_->setMin(0);
   std_dev_y_->setMin(0);
   std_dev_theta_->setMin(0);
   QTimer* output_timer = new QTimer( this );
   connect( output_timer, SIGNAL( timeout() ), this, SLOT( Loop_publish() ));
   output_timer->start( 100 );

 }
 
 void Waypoint_generate::onInitialize()
 {
   PoseTool::onInitialize();
   setName("Waypoint");
   updateTopic();
   count = 0;
 }
 
 void Waypoint_generate::updateTopic()
 {
   try
   {
     pub_ = nh_.advertise<waypoint_generator::Waypoint_array>("waypoint", 1);
     marker_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_first", 1);
   }
   catch (const ros::Exception& e)
   {
     ROS_ERROR_STREAM_NAMED("Waypoint_generate", e.what());
   }
 }
 void Waypoint_generate::Loop_publish()
 {
    ros::Rate loop_rate(10);
    
    if(ros::ok()){
      pub_.publish(wp_array);
    }
 }
 void Waypoint_generate::onPoseSet(double x, double y, double theta)
 {
  
   std::string fixed_frame = context_->getFixedFrame().toStdString();
   wp.header.frame_id = fixed_frame;
   wp.header.stamp = ros::Time::now();
   wp.header.seq = count;
   wp.x = x;
   wp.y = y;
   wp_array.wp.push_back(wp);
   visualization_msgs::Marker marker;
   marker.header.frame_id = "map";
   marker.header.stamp = ros::Time::now();
   marker.ns = "waypoint_first";
   marker.id = count;
   marker.type = visualization_msgs::Marker::CYLINDER;
   marker.action = visualization_msgs::Marker::ADD;
   marker.lifetime = ros::Duration();

   marker.scale.x = 0.1;
   marker.scale.y = 0.1;
   marker.scale.z = 0.2;
   marker.pose.position.x = x;
   marker.pose.position.y = y;
   marker.pose.position.z = 0;
   marker.pose.orientation.x = 0;
   marker.pose.orientation.y = 0;
   marker.pose.orientation.z = 0;
   marker.pose.orientation.w = 1;
   marker.color.r = 0.0f;
   marker.color.g = 1.0f;
   marker.color.b = 0.0f;
   marker.color.a = 1.0f;
   marker_pub_.publish(marker);
   count += 1; 
   ROS_ERROR("wp: %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str());
   
 }

 
 
 } // end namespace rviz
 #include <pluginlib/class_list_macros.hpp>
 PLUGINLIB_EXPORT_CLASS(rviz::Waypoint_generate, rviz::Tool)