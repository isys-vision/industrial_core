/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 * 	of its contributors may be used to endorse or promote products derived
 * 	from this software without specific prior written permission.
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

#ifndef FLATHEADERS
#include "simple_message/mikado_classes/mik_status.h"
#include "simple_message/shared_types.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "mik_status.h"
#include "shared_types.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;

namespace industrial
{
namespace mik_status
{

MikStatus::MikStatus(void)
{
  this->init();
}

MikStatus::~MikStatus(void)
{

}

void MikStatus::init()
{
  this->mik_state_ = MikStates::MS_UNKNOWN;
  this->camera_state_ = MikStates::MS_UNKNOWN;
  this->comm_state_ = MikStates::MS_UNKNOWN;
  this->robot_state_ = MikStates::MS_UNKNOWN;
  this->running_state_ = MikStates::MS_UNKNOWN;
}

void MikStatus::init(MikState mik_state, CameraState camera_state, CommState comm_state,
                     RobotState robot_state, RunningState running_state)
{
  this->setMikState(mik_state);
  this->setCameraState(camera_state);
  this->setCommState(comm_state);
  this->setRobotState(robot_state);
  this->setRunningState(running_state);
}

void MikStatus::copyFrom(MikStatus &src)
{
  this->setMikState(src.getMikState());
  this->setCameraState(src.getCameraState());
  this->setCommState(src.getCommState());
  this->setRobotState(src.getRobotState());
  this->setRunningState(src.getRunningState());
}

bool MikStatus::operator==(MikStatus &rhs)
{
  return this->mik_state_ == rhs.mik_state_ &&
         this->camera_state_ == rhs.camera_state_ &&
         this->comm_state_ == rhs.comm_state_ &&
         this->robot_state_ == rhs.robot_state_ &&
         this->running_state_ == rhs.running_state_;
}

bool MikStatus::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mik status load");
  if (buffer->load(this->mik_state_) &&
      buffer->load(this->camera_state_) &&
      buffer->load(this->comm_state_) &&
      buffer->load(this->robot_state_) &&
      buffer->load(this->running_state_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load mik status data");
  }
  return rtn;
}

bool MikStatus::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mik status unload");

  if (buffer->unload(this->running_state_) &&
      buffer->unload(this->robot_state_) &&
      buffer->unload(this->comm_state_) &&
      buffer->unload(this->camera_state_) &&
      buffer->unload(this->mik_state_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload mik status data");
  }
  return rtn;
}

}
}