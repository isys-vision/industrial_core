/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
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
#include "simple_message/mikado_messages/mik_action_trigger_message.h"
#include "simple_message/mikado_classes/mik_action_trigger.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#include "simple_message/mikado_messages/mikado_types.h"
#else
#include "mik_action_trigger_message.h"
#include "mik_action_trigger.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace industrial
{
namespace mik_action_trigger_message
{

MikActionTriggerMessage::MikActionTriggerMessage(void)
{
  this->init();
}

MikActionTriggerMessage::~MikActionTriggerMessage(void)
{
}

bool MikActionTriggerMessage::init(industrial::simple_message::SimpleMessage & msg)
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
    LOG_ERROR("Failed to unload mik action trigger data");
  }
  return rtn;
}

void MikActionTriggerMessage::init(industrial::mik_action_trigger::MikActionTrigger & action_trigger)
{
  this->init();
  this->action_trigger_.copyFrom(action_trigger);
}

void MikActionTriggerMessage::init()
{
  this->setMessageType(mik_msg_type::ACTION_TRIG);
  this->action_trigger_.init();
}

bool MikActionTriggerMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mik action trigger message load");
  if (buffer->load(this->action_trigger_))
  {
    if(buffer->load(MIK_EOM))
    {
      rtn = true;
    } else {
      rtn = false;
      LOG_ERROR("Failed to load mik action trigger data");
    }
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load mik action trigger data");
  }
  return rtn;
}

bool MikActionTriggerMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mik action trigger message unload");
  int EOM;

  if (buffer->unload(EOM))
  {
    if (buffer->unload(this->action_trigger_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
      LOG_ERROR("Failed to unload mik action trigger data");
    }
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload mik action trigger data");
  }
  return rtn;
}

}
}