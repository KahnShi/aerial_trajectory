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

#include <bspline_generator/bsplineGenerator.h>

namespace bspline_generator{
  bsplineGenerator::bsplineGenerator(ros::NodeHandle nh, ros::NodeHandle nhp){
    nh_ = nh;
    nhp_ = nhp;
    sampling_plannar_client_ = nh_.serviceClient<gap_passing::Keyposes>("keyposes_server");
    bspline_ptr_ = boost::shared_ptr<bsplineGenerate>(new bsplineGenerate(nh_, nhp_, std::string("/path")));

    control_pts_ptr_ = new bspline_ros::ControlPoints();
    control_pts_ptr_->control_pts.layout.dim.push_back(std_msgs::MultiArrayDimension());
    control_pts_ptr_->control_pts.layout.dim.push_back(std_msgs::MultiArrayDimension());
    control_pts_ptr_->control_pts.layout.dim[0].label = "height";
    control_pts_ptr_->control_pts.layout.dim[1].label = "width";

    sleep(1);
    waitForKeyposes();
  }

  bsplineGenerator::~bsplineGenerator(){
  }

  void bsplineGenerator::waitForKeyposes(){
    gap_passing::Keyposes keyposes_srv;
    keyposes_srv.request.inquiry = true;
    while (!sampling_plannar_client_.call(keyposes_srv)){
      // waiting for keyposes
    }
    if (!keyposes_srv.response.available_flag){
      ROS_WARN("keyposes is not available, try again");
      waitForKeyposes();
    }
    else if (keyposes_srv.response.states_cnt == 0){
      ROS_WARN("keyposes size is 0, try again");
      waitForKeyposes();
    }
    else{
      control_pts_ptr_->num = keyposes_srv.response.states_cnt;
      control_pts_ptr_->dim = keyposes_srv.response.dim + 1; // keyposes do not have z axis info
      control_pts_ptr_->degree = 5; // todo
      control_pts_ptr_->is_uniform = true; // todo
      control_pts_ptr_->start_time = 10.0;
      control_pts_ptr_->end_time = 20.0;

      control_pts_ptr_->control_pts.layout.dim[0].size = control_pts_ptr_->num;
      control_pts_ptr_->control_pts.layout.dim[1].size = control_pts_ptr_->dim;
      control_pts_ptr_->control_pts.layout.dim[0].stride = control_pts_ptr_->num * control_pts_ptr_->dim;
      control_pts_ptr_->control_pts.layout.dim[1].stride = control_pts_ptr_->dim;
      control_pts_ptr_->control_pts.layout.data_offset = 0;

      control_pts_ptr_->control_pts.data.resize(0);
      for (int i = 0; i < control_pts_ptr_->num; ++i){
        int index_s = i * keyposes_srv.response.dim;
        for (int j = 0; j < 2; ++j) // add x, y axis data
          control_pts_ptr_->control_pts.data.push_back(keyposes_srv.response.data.data[index_s + j]);
        control_pts_ptr_->control_pts.data.push_back(0.0); // fixed data for z axis
        for (int j = 2; j < control_pts_ptr_->dim - 1; ++j) // add yaw and joint angles data
          control_pts_ptr_->control_pts.data.push_back(keyposes_srv.response.data.data[index_s + j]);
      }
      displayBspline();
    }
  }

  void bsplineGenerator::manuallySetControlPts(){
    control_pts_ptr_->num = 5;
    control_pts_ptr_->dim = 3;
    control_pts_ptr_->degree = 2;
    control_pts_ptr_->is_uniform = false;

    control_pts_ptr_->control_pts.layout.dim[0].size = control_pts_ptr_->num;
    control_pts_ptr_->control_pts.layout.dim[1].size = control_pts_ptr_->dim;
    control_pts_ptr_->control_pts.layout.dim[0].stride = control_pts_ptr_->num * control_pts_ptr_->dim;
    control_pts_ptr_->control_pts.layout.dim[1].stride = control_pts_ptr_->dim;
    control_pts_ptr_->control_pts.layout.data_offset = 0;

    // pt1
    control_pts_ptr_->control_pts.data.push_back(-2.0);
    control_pts_ptr_->control_pts.data.push_back(4.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    // pt2
    control_pts_ptr_->control_pts.data.push_back(0.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    // pt3
    control_pts_ptr_->control_pts.data.push_back(2.0);
    control_pts_ptr_->control_pts.data.push_back(4.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    // pt4
    control_pts_ptr_->control_pts.data.push_back(5.0);
    control_pts_ptr_->control_pts.data.push_back(3.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);
    // pt5
    control_pts_ptr_->control_pts.data.push_back(6.0);
    control_pts_ptr_->control_pts.data.push_back(5.0);
    control_pts_ptr_->control_pts.data.push_back(1.0);

    int knots_num = control_pts_ptr_->num + control_pts_ptr_->degree + 1;
    for (int i = 0; i <= control_pts_ptr_->degree; ++i)
      control_pts_ptr_->knots.data.push_back(0.0);
    // mid
    control_pts_ptr_->knots.data.push_back(0.4);
    control_pts_ptr_->knots.data.push_back(0.8);
    for (int i = 0; i <= control_pts_ptr_->degree; ++i)
      control_pts_ptr_->knots.data.push_back(1.0);
  }

  void bsplineGenerator::displayBspline(){
    bspline_ptr_->bsplineParamInput(control_pts_ptr_);
    bspline_ptr_->splinePathDisplay();
    bspline_ptr_->controlPolygonDisplay();
    ROS_INFO("Spline display finished.");
  }
}
