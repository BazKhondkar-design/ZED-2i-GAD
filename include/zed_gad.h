/*# The MIT License (MIT)

# Copyright (C) 2021 s7711
# support@oxts.com

# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#pragma once

#include <functional>
#include <iostream>
#include <string>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>


#define BOOST_BIND_NO_PLACEHOLDERS
using namespace std::placeholders;

#define RAD2DEG 57.295779513


//std::chrono::steady_clock::time_point packet_time;


#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iomanip>
#include <cmath>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "oxts/gal-cpp/gad.hpp"
#include "oxts/gal-cpp/gad_handler.hpp"

namespace zed_gad
{
    class ZedGadPoseOdom : public rclcpp::Node 
    {

        public:
            //Constructor
            ZedGadPoseOdom(const std::string av200_ip);
            //Destructor
            virtual ~ZedGadPoseOdom();

        protected:
            void odomCameraCallback(const nav_msgs::msg::Odometry::SharedPtr msg); //Camera odometry
            void odomINSCallback(const nav_msgs::msg::Odometry::SharedPtr msg);    //AV200 odometry

        private:

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSub; //Subscribe to the camera ROS2 odom topics
            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr  mVelPub; //Publish the camera velocity in body frame
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOxTSVelSub;

            double m_prevX;       //Camera pervious x position
            double m_prevY;       //Camera pervious y position
            double m_prevZ;       //Camera pervious z position
            double m_prevTime;    //Previous time at which camera position received 

            OxTS::GadHandler m_gh; //Gad handler

            

    };
}