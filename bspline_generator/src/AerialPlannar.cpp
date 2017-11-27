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

#include <bspline_generator/AerialPlannar.h>

namespace aerial_plannar{
  AerialPlannar::AerialPlannar(ros::NodeHandle nh, ros::NodeHandle nhp){
    nh_ = nh;
    nhp_ = nhp;
    joint_num_ = 3;

    aerial_controller_ = boost::shared_ptr<AerialControllerInterface>(new AerialControllerInterface(nh_, nhp_, joint_num_));
    endposes_server_ = nh_.advertiseService("endposes_server", &AerialPlannar::getEndposes, this);


    uav_takeoff_flag_ = false;
    aerial_controller_->robot_start();
    ROS_INFO("[AerialPlannar] Published robot start topic.");
    sleep(2.5);
    aerial_controller_->takeoff();
    sleep(18.0); // waiting for finishing taking off
    ROS_INFO("[AerialPlannar] Published takeoff topic.");
    uav_takeoff_flag_ = true;
  }

  AerialPlannar::~AerialPlannar(){
  }

  bool AerialPlannar::getEndposes(gap_passing::Endposes::Request &req, gap_passing::Endposes::Response &res){
    if (ros::ok && uav_takeoff_flag_){
      res.dim = 6;
      // todo: convert to cog frame
      res.start_pose.data.push_back(aerial_controller_->baselink_pos_.getX());
      res.start_pose.data.push_back(aerial_controller_->baselink_pos_.getY());
      res.start_pose.data.push_back(aerial_controller_->baselink_ang_.getZ());
      for (int i = 0; i < joint_num_; ++i)
        res.start_pose.data.push_back(aerial_controller_->joints_ang_vec_[i]);

      tf::Vector3 target_offset(4.0, 0.0, 0.0);
      res.end_pose.data.push_back(aerial_controller_->baselink_pos_.getX() + target_offset.getX());
      res.end_pose.data.push_back(aerial_controller_->baselink_pos_.getY() + target_offset.getY());
      res.end_pose.data.push_back(aerial_controller_->baselink_ang_.getZ() + target_offset.getZ());
      for (int i = 0; i < joint_num_; ++i)
        res.end_pose.data.push_back(aerial_controller_->joints_ang_vec_[i]);
    }
  }
}
