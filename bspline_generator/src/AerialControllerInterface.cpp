// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <bspline_generator/AerialControllerInterface.h>

namespace aerial_controller_interface{
  AerialControllerInterface::AerialControllerInterface(ros::NodeHandle nh, ros::NodeHandle nhp, int joint_num){
    nh_ = nh;
    nhp_ = nhp;
    joint_num_ = joint_num;

    nhp_.param("controller_freq", controller_freq_, 100.0);
    nhp_.param("state_dim", state_dim_, 7);

    init_param();

    robot_start_pub_ = nh_.advertise<std_msgs::Empty>("/teleop_command/start", 1);
    takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/teleop_command/takeoff", 1);
    joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("/hydrusx/joint_states", 1);
    flight_nav_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("/uav/nav", 1);;

    joints_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/hydrusx/joint_states", 1, &AerialControllerInterface::jointStatesCallback, this);
    baselink_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/uav/baselink/odom", 1, &AerialControllerInterface::baselinkOdomCallback, this);
    cog_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/uav/cog/odom", 1, &AerialControllerInterface::cogOdomCallback, this);
    move_start_flag_sub_ = nh_.subscribe<std_msgs::Empty>("/move_start", 1, &AerialControllerInterface::moveStartCallback, this);

    sleep(1.0);
  }

  AerialControllerInterface::~AerialControllerInterface(){
  }

  void AerialControllerInterface::init_param(){
    move_start_flag_ = false;

    for (int i = 0; i < joint_num_; ++i){
      joints_ang_vec_.push_back(0.0);
      joints_vel_vec_.push_back(0.0);
    }
  }

  void AerialControllerInterface::robot_start(){
    std_msgs::Empty msg; robot_start_pub_.publish(msg);
  }

  void AerialControllerInterface::takeoff(){
    std_msgs::Empty msg; takeoff_pub_.publish(msg);
  }

  void AerialControllerInterface::ff_controller(std::vector<double> &ff_term, std::vector<double> &target){


    // todo: yaw vel works?
    /* publish uav nav to control */
    // aerial_robot_base::FlightNav nav_msg;
    // nav_msg.header.frame_id = std::string("/world");
    // nav_msg.header.stamp = ros::Time::now();
    // nav_msg.header.seq = 1;
    // nav_msg.pos_xy_nav_mode = nav_msg.VEL_MODE;
    // nav_msg.target_vel_x = 0.0;
    // nav_msg.target_vel_y = 0.0;
    // nav_msg.psi_nav_mode = nav_msg.VEL_MODE;
    // nav_msg.target_psi = 0.0;
    // flight_nav_pub_.publish(nav_msg);

    // todo: joint vel works?
    /* publish joint states */
    // sensor_msgs::JointState joints_msg;
    // for (int i = 0; i < 3; ++i){
    //   joints_msg.position.push_back(des_joint_ang[i]);
    //   joints_msg.velocity.push_back(des_joint_vel[i]);
    // }
    // joints_ctrl_pub_.publish(joints_msg);
  }

  void AerialControllerInterface::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joints_msg){
    for (int i = 0; i < joints_msg->position.size(); ++i){
      joints_ang_vec_[i] = joints_msg->position[i];
      joints_vel_vec_[i] = joints_msg->velocity[i];
    }
  }

  void AerialControllerInterface::baselinkOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg){
    baselink_odom_ = *odom_msg;
    tf::Quaternion q(odom_msg->pose.pose.orientation.x,
                     odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z,
                     odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat;
    rot_mat.setRotation(q);
    tfScalar r,p,y;
    rot_mat.getRPY(r, p, y);
    baselink_ang_.setValue(r, p, y);
    baselink_pos_.setValue(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    baselink_vel_.setValue(odom_msg->twist.twist.linear.x,
                           odom_msg->twist.twist.linear.y,
                           odom_msg->twist.twist.linear.z);
    baselink_w_.setValue(odom_msg->twist.twist.angular.x,
                         odom_msg->twist.twist.angular.y,
                         odom_msg->twist.twist.angular.z);
  }

  void AerialControllerInterface::cogOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg){
    cog_odom_ = *odom_msg;
    tf::Quaternion q(odom_msg->pose.pose.orientation.x,
                     odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z,
                     odom_msg->pose.pose.orientation.w);
    tf::Matrix3x3 rot_mat;
    rot_mat.setRotation(q);
    tfScalar r,p,y;
    rot_mat.getRPY(r, p, y);
    cog_ang_.setValue(r, p, y);
    cog_pos_.setValue(odom_msg->pose.pose.position.x,
                      odom_msg->pose.pose.position.y,
                      odom_msg->pose.pose.position.z);
    cog_vel_.setValue(odom_msg->twist.twist.linear.x,
                      odom_msg->twist.twist.linear.y,
                      odom_msg->twist.twist.linear.z);
    cog_w_.setValue(odom_msg->twist.twist.angular.x,
                    odom_msg->twist.twist.angular.y,
                    odom_msg->twist.twist.angular.z);
  }

  void AerialControllerInterface::moveStartCallback(const std_msgs::Empty msg){
    move_start_flag_ = true;
  }
}
