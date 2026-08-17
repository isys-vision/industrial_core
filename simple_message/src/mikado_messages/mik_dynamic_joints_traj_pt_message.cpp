/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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
#include "simple_message/mikado_messages/mik_dynamic_joints_traj_pt_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "mikado_dynamic_joints_traj_pt_message.h"
#include "joint_data.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace industrial
{
namespace mik_dynamic_joint_traj_pt_msg
{

MikadoDynamicJointsTrajPtMessage::MikadoDynamicJointsTrajPtMessage(void)
{
  this->init();
}

MikadoDynamicJointsTrajPtMessage::~MikadoDynamicJointsTrajPtMessage(void)
{

}

bool MikadoDynamicJointsTrajPtMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());
  if (this->unload(&data))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload mik dynamic joint traj pt data");
    return false;
  }
  return true;
}

void MikadoDynamicJointsTrajPtMessage::init(MikadoDynamicJointsTrajPt & point)
{
  this->init();
  this->point_.copyFrom(point);
}

void MikadoDynamicJointsTrajPtMessage::init()
{
  this->setMessageType(mik_msg_type::TRAJ_PT);
  this->point_.init();
}


bool MikadoDynamicJointsTrajPtMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint traj. pt. message load");
  if (buffer->load(this->point_))
  {
    rtn = true;
    buffer->load(MIK_EOM);
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load joint traj. pt data");
  }
  return rtn;
}

bool MikadoDynamicJointsTrajPtMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint traj pt message unload");

  char c;
  buffer->unload(&c, 4);
  if (buffer->unload(this->point_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload joint traj pt data");
  }
  return rtn;
}

}
}
