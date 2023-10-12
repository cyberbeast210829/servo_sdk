/**
  ******************************************************************************
  * @file    motctrl_srvs_impl.cpp
  * @author  LYH, CyberBeast
  * @brief   This file provides services implementation for CyberBeast motor control
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

#include "motctrl_can/motctrl_srvs_impl.h"
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/reader.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/vt.h>
#include <socketcan_interface/string.h>
#include <ros/ros.h>

using namespace can;

#include <iostream>

can::SocketCANInterfaceSharedPtr c(new can::SocketCANInterface());
can::BufferedReader canReader;
can::MsgHeader frameHeader;
boost::chrono::microseconds to = boost::chrono::microseconds(100);

VTsocket vt;
int num_frames = 1;
Frame *frames = new Frame[num_frames];

bool OpenCan(motctrl_can::open_can::Request & req, motctrl_can::open_can::Response & res)
{
  if (!vt.init(req.canPort.c_str())) {
    ROS_WARN("Unable to open port '%s'.", req.canPort.c_str());
    res.res = false;
    return true;
  }
  
  frames[0].id = req.canID;
  frames[0].is_rtr = 0;
  frames[0].is_extended = 0;
  frames[0].is_error = 0;
  frames[0].dlc = 8;

  canReader.flush();
  canReader.listen(c);

  res.res = true;
  return true;
}

bool StartMotor(motctrl_can::start_motor::Request & req, motctrl_can::start_motor::Response & res)
{
  res.res = (uint8_t)MOTCTRL_RES_SUCCESS;
  MCReqStartMotor(frames->data.data());

  for(int i = 0; i < num_frames; ++i){
	std::cout << frames[i] << std::endl;
  }
    
  vt.startTX(boost::chrono::duration<float>(atof("0")), num_frames, frames);
  
  return true;
}

bool StopMotor(motctrl_can::stop_motor::Request & req, motctrl_can::stop_motor::Response & res)
{
  res.res = (uint8_t)MOTCTRL_RES_SUCCESS;
  MCReqStopMotor(frames->data.data());

  for(int i = 0; i < num_frames; ++i){
	std::cout << frames[i] << std::endl;
  }
    
  vt.startTX(boost::chrono::duration<float>(atof("0")), num_frames, frames);
  
  return true;
}

bool TorqueControl(motctrl_can::torque_control::Request & req, motctrl_can::torque_control::Response & res)
{
  res.res = (uint8_t)MOTCTRL_RES_SUCCESS;
  MCReqTorqueControl(frames->data.data(), req.torque, req.duration);

  for(int i = 0; i < num_frames; ++i){
	std::cout << frames[i] << std::endl;
  }
    
  vt.startTX(boost::chrono::duration<float>(atof("0")), num_frames, frames);
  try {
    if (c->send(frames[0])) {
      if (canReader.read(&frames[0], to)) {
        float position, speed, torque;
        res.res = (uint8_t)MCResTorqueControl(frames->data.data(), &res.temp, &position, &speed, &torque);
        res.position = position;
        res.speed = speed;
        res.torque = torque;
      }
      else {
        res.res = (uint8_t)MOTCTRL_RES_FAIL;
      }
    }
    else {
      res.res = (uint8_t)MOTCTRL_RES_FAIL;
    }
  }
  catch (std::exception & e) {
    ROS_WARN("Error occurred: %s", e.what());
    res.res = (uint8_t)MOTCTRL_RES_FAIL;
  }

  return true;
}

bool SpeedControl(motctrl_can::speed_control::Request & req, motctrl_can::speed_control::Response & res)
{
  can::Frame frame(frameHeader, MOTCTRL_FRAME_SIZE);
  res.res = (uint8_t)MOTCTRL_RES_SUCCESS;
  MCReqSpeedControl(frames->data.data(), req.speed, req.duration);

  for(int i = 0; i < num_frames; ++i){
	std::cout << frames[i] << std::endl;
  }
    
  vt.startTX(boost::chrono::duration<float>(atof("0")), num_frames, frames);
  try {
    if (c->send(frame)) {
      if (canReader.read(&frame, to)) {
        float position, speed, torque;
        res.res = (uint8_t)MCResSpeedControl(frame.data.data(), &res.temp, &position, &speed, &torque);
        res.position = position;
        res.speed = speed;
        res.torque = torque;
      }
      else {
        res.res = (uint8_t)MOTCTRL_RES_FAIL;
      }
    }
    else {
      res.res = (uint8_t)MOTCTRL_RES_FAIL;
    }
  }
  catch (std::exception & e) {
    ROS_WARN("Error occurred: %s", e.what());
    res.res = (uint8_t)MOTCTRL_RES_FAIL;
  }

  return true;
}

bool PositionControl(motctrl_can::position_control::Request & req, motctrl_can::position_control::Response & res)
{
  can::Frame frame(frameHeader, MOTCTRL_FRAME_SIZE);
  res.res = (uint8_t)MOTCTRL_RES_SUCCESS;
  MCReqPositionControl(frames->data.data(), req.position, req.duration);
  for(int i = 0; i < num_frames; ++i){
	std::cout << frames[i] << std::endl;
  }
    
  vt.startTX(boost::chrono::duration<float>(atof("0")), num_frames, frames);
  try {
    if (c->send(frame)) {
      if (canReader.read(&frame, to)) {
        float position, speed, torque;
        res.res = (uint8_t)MCResPositionControl(frame.data.data(), &res.temp, &position, &speed, &torque);
        res.position = position;
        res.speed = speed;
        res.torque = torque;
      }
      else {
        res.res = (uint8_t)MOTCTRL_RES_FAIL;
      }
    }
    else {
      res.res = (uint8_t)MOTCTRL_RES_FAIL;
    }
  }
  catch (std::exception & e) {
    ROS_WARN("Error occurred: %s", e.what());
    res.res = (uint8_t)MOTCTRL_RES_FAIL;
  }

  return true;
}

