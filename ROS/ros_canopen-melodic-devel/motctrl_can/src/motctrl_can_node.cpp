/**
  ******************************************************************************
  * @file    motctrl_can_node.cpp
  * @author  LYH, CyberBeast
  * @brief   This file provides implementation for CyberBeast motor control over can
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 CyberBeast.
  * All rights reserved.</center></h2>
  *
  ******************************************************************************
  *
  */
#include <ros/ros.h>
#include "motctrl_can/motctrl_srvs_impl.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "motctrl_can");
  ros::NodeHandle n;

  ros::ServiceServer srvOpenCan = n.advertiseService("motctrl_can/open_can", OpenCan);
  ros::ServiceServer srvStartMotor = n.advertiseService("motctrl_can/start_motor", StartMotor);
  ros::ServiceServer srvStopMotor = n.advertiseService("motctrl_can/stop_motor", StopMotor);
  ros::ServiceServer srvTorqueControl = n.advertiseService("motctrl_can/torque_control", TorqueControl);
  ros::ServiceServer srvSpeedControl = n.advertiseService("motctrl_can/speed_control", SpeedControl);
  ros::ServiceServer srvPositionControl = n.advertiseService("motctrl_can/position_control", PositionControl);
  ROS_INFO("Ready to service incoming requests.");
  ros::spin();

  return 0;
}

