/**
  ******************************************************************************
  * @file    motctrl_srvs_impl.h
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
#ifndef _MOTCTRL_SRVS_IMPL_H__
#define _MOTCTRL_SRVS_IMPL_H__

#include "motctrl_can/motctrl_prot.h"
#include "motctrl_can/open_can.h"
#include "motctrl_can/reset_configuration.h"
#include "motctrl_can/refresh_configuration.h"
#include "motctrl_can/modify_configuration.h"
#include "motctrl_can/retrieve_configuration.h"
#include "motctrl_can/start_motor.h"
#include "motctrl_can/stop_motor.h"
#include "motctrl_can/torque_control.h"
#include "motctrl_can/speed_control.h"
#include "motctrl_can/position_control.h"
#include "motctrl_can/stop_control.h"
#include "motctrl_can/modify_parameter.h"
#include "motctrl_can/retrieve_parameter.h"
#include "motctrl_can/get_version.h"
#include "motctrl_can/get_fault.h"
#include "motctrl_can/ack_fault.h"
#include "motctrl_can/retrieve_indicator.h"

bool OpenCan(motctrl_can::open_can::Request & req, motctrl_can::open_can::Response & res);
bool StartMotor(motctrl_can::start_motor::Request & req, motctrl_can::start_motor::Response & res);
bool StopMotor(motctrl_can::stop_motor::Request & req, motctrl_can::stop_motor::Response & res);
bool TorqueControl(motctrl_can::torque_control::Request & req, motctrl_can::torque_control::Response & res);
bool SpeedControl(motctrl_can::speed_control::Request & req, motctrl_can::speed_control::Response & res);
bool PositionControl(motctrl_can::position_control::Request & req, motctrl_can::position_control::Response & res);

#endif
