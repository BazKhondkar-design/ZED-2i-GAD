/*
# The MIT License (MIT)

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

#include "zed_gad.h" //include the zed_gad.h 

namespace zed_gad
{
  //Constructor implementation
  ZedGadPoseOdom::ZedGadPoseOdom(const std::string av200_ip)  : Node("zed_gad"), m_prevX(0), m_prevY(0), m_prevZ(0), m_prevTime(0)
  {
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.durability_volatile();

    // Create odometry subscriber.
    mOdomSub = create_subscription<nav_msgs::msg::Odometry>(
      "/zed2i/zed_node/odom", qos, std::bind(&ZedGadPoseOdom::odomCameraCallback, this, _1));
    
    // Create camera body frame velocty Vx, Vy and Vz publisher.
    mVelPub = create_publisher<geometry_msgs::msg::Point>("zed_vel", qos);

     // Create odometry subscriber. INS
    mOxTSVelSub = create_subscription<nav_msgs::msg::Odometry>(
      "/ins/odometry", qos, std::bind(&ZedGadPoseOdom::odomINSCallback, this, _1));

    m_gh = OxTS::GadHandler();
	  m_gh.SetEncoderToBin();
	  m_gh.SetOutputModeToUdp(av200_ip);
    std::cout << "in constructor" << std::endl;
  }

  ZedGadPoseOdom::~ZedGadPoseOdom()
  {
    
  }

  //Topics received from NCOM to ROS2
  void ZedGadPoseOdom::odomINSCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // NOT used	  
    //std::cout << "INS odom callback" << std::endl;
    //msg->twist.linear.x;
    //msg->twist.linear.y;
    //msg->twist.linear.z;

  } 

  //odom received from camera
  void ZedGadPoseOdom::odomCameraCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  { 
    //std::cout << "Camera odom callback" << std::endl;
    if(m_prevTime != 0)
    {
      // Calculate time since last callback, this should reflect the fpublish rate of the ZED (i.e. at frame rate 30Hz = 0.033s)
      double time_delta = (msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9) - m_prevTime;
  
      // get orientation in quanternion complex number
      double oriX = msg->pose.pose.orientation.x;
      double oriY = msg->pose.pose.orientation.y;
      double oriZ = msg->pose.pose.orientation.z;
      double oriW = msg->pose.pose.orientation.w; //the scale of the x, y and z

      // Create quaternion object
      tf2::Quaternion q(oriX, oriY, oriZ, oriW);

      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);

      // Camera Roll Pitch and Heading (yaw) angles from rotation matrix
      double roll, pitch, yaw; //in radian
      m.getRPY(roll, pitch, yaw);

      // Convert angles to matrices
      Eigen::Matrix<double, 3, 3> Rh, Rp, Rr;

      //Heading rotation matrix
      Rh <<       cos(yaw),        -sin(yaw),        0,
                sin(yaw),        cos(yaw),         0,
                0,               0,                1;
      //Pitch rotation matrix
      Rp <<       cos(pitch),      0,                sin(pitch),
                0,               1,                0,
                -sin(pitch),     0,                cos(pitch);
      //Roll rotation matrix
      Rr <<       1.0,             0.0,              0,
                0,               cos(roll),        -sin(roll),
                0,               sin(roll),        cos(roll);

      // Apply all rotations to change FoR. https://support.oxts.com/hc/en-us/articles/115002859149-OxTS-Reference-Frames-and-ISO8855-Reference-Frames#C3
      Eigen::Matrix<double, 3, 3> C_lc = Rh * Rp * Rr;
      Eigen::Matrix<double, 3, 3> C_cl = C_lc.transpose();

      // Get velocities in the global frame
      double x_diff = msg->pose.pose.position.x - m_prevX;
      double y_diff = msg->pose.pose.position.y - m_prevY;
      double z_diff = msg->pose.pose.position.z - m_prevZ;

      double velX = time_delta != 0 ? x_diff / time_delta : 0;
      double velY = time_delta != 0 ? y_diff / time_delta : 0;
      double velZ = time_delta != 0 ? z_diff / time_delta : 0;

      Eigen::Vector3d cam_vel_global(velX, velY, velZ);

      // Convert to local frame
      Eigen::Vector3d cam_vel_body = C_cl*cam_vel_global;
      double Vx = cam_vel_body[0];
      double Vy = -cam_vel_body[1];
      double Vz = -cam_vel_body[2];

      // Publish to /zed_vel/
      geometry_msgs::msg::Point vel_msg = geometry_msgs::msg::Point();
      vel_msg.x = float( Vx );
      vel_msg.y = float( Vy );
      vel_msg.z = float( Vz );
      mVelPub->publish(vel_msg);

      // Set the following values ready for next callback
      m_prevX = msg->pose.pose.position.x;
      m_prevY = msg->pose.pose.position.y;
      m_prevZ = msg->pose.pose.position.z;

      // Send GAD packet
      OxTS::GadVelocity gv = OxTS::GadVelocity(130); //Gad packet ID 130 will be sent to the AV200

      gv.SetVelOdom(Vx, Vy, Vz);

      // Set position variance.
      double covarXX = msg->pose.covariance[0];
      double covarYY = msg->pose.covariance[7];
      double covarZZ = msg->pose.covariance[14];
      gv.SetVelOdomVar(fabs(covarXX), fabs(covarYY), fabs(covarZZ));

      gv.SetTimeVoid(); //set the gad packet in latency mode

      m_gh.SendPacket(gv);
      //std::cout << "Packet sent." << std::endl;
    }

     m_prevTime = (msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);
  }
}

