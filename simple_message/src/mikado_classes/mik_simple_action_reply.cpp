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
#include "simple_message/mikado_classes/mik_simple_action_reply.h"
#include "simple_message/shared_types.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#include <algorithm>
#else
#include "mik_simple_action_reply.h"
#include "shared_types.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::mik_simple_action_reply;

namespace industrial
{

MikSimpleActionReply::MikSimpleActionReply(void)
{
  this->init();
}

MikSimpleActionReply::~MikSimpleActionReply(void)
{
}

void MikSimpleActionReply::init()
{
  this->action_id_ = 0;
  this->request_id_ = 0;
  this->action_status_ = 0;
  this->additional_int_args_.clear();
  this->additional_real_args_.clear();
  this->error_msg_ = "";
  this->int_args_count_ = 0;
  this->real_args_count_ = 0;
  this->error_msg_length_ = 0;
}

void MikSimpleActionReply::init(shared_types::shared_int action_id, shared_types::shared_int request_id, shared_types::shared_int action_status,
                            std::vector<shared_types::shared_int> additional_int_args,
                            std::vector<shared_types::shared_real> additional_real_args,
                            std::string error_msg)
{
  this->setActionId(action_id);
  this->setRequestId(request_id);
  this->setActionStatus(action_status);
  this->setAdditionalIntArgs(additional_int_args);
  this->setAdditionalRealArgs(additional_real_args);
  this->setErrorMsg(error_msg);
}

void MikSimpleActionReply::copyFrom(MikSimpleActionReply &src)
{
  this->action_id_ = src.action_id_;
  this->request_id_ = src.request_id_;
  this->action_status_ = src.action_status_;
  this->setAdditionalIntArgs(src.additional_int_args_);
  this->setAdditionalRealArgs(src.additional_real_args_);
  this->setErrorMsg(src.error_msg_);
}

bool MikSimpleActionReply::operator==(MikSimpleActionReply &rhs)
{
  if (this->action_id_ != rhs.action_id_ ||
      this->request_id_ != rhs.request_id_ ||
      this->action_status_ != rhs.action_status_ ||
      this->int_args_count_ != rhs.int_args_count_ ||
      this->real_args_count_ != rhs.real_args_count_ ||
      this->error_msg_length_ != rhs.error_msg_length_ ||
      this->error_msg_ != rhs.error_msg_)
  {
    return false;
  }
  
  // Compare additional int args
  if (this->additional_int_args_.size() != rhs.additional_int_args_.size())
  {
    return false;
  }
  for (size_t i = 0; i < this->additional_int_args_.size(); i++)
  {
    if (this->additional_int_args_[i] != rhs.additional_int_args_[i])
    {
      return false;
    }
  }
  
  // Compare additional real args
  if (this->additional_real_args_.size() != rhs.additional_real_args_.size())
  {
    return false;
  }
  for (size_t i = 0; i < this->additional_real_args_.size(); i++)
  {
    if (this->additional_real_args_[i] != rhs.additional_real_args_[i])
    {
      return false;
    }
  }


  
  return true;
}

bool MikSimpleActionReply::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mik simple action reply load");
  
  if (buffer->load(this->action_id_) &&
      buffer->load(this->request_id_) &&
      buffer->load(this->action_status_))
  {
    
    // Load additional int args
    for (shared_int i = 0; i < this->int_args_count_; i++)
    {
      shared_int val = this->additional_int_args_.at(i);
      if (!buffer->load(val))
      {
        rtn = false;
        LOG_ERROR("Failed to load additional int arg %d", i);
        return false;
      }
    }
    
    // Load additional real args
    for (shared_int i = 0; i < this->real_args_count_; i++)
    {
      shared_real val = this->additional_real_args_.at(i);
      if (!buffer->load(val))
      {
        rtn = false;
        LOG_ERROR("Failed to load additional real arg %d", i);
        return false;
      }
    }

    // Load error msg string
    for (shared_int i = 0; i < this->error_msg_length_; i++)
    {
      shared_char val = this->error_msg_.at(i);
      if (!buffer->load(val))
      {
        rtn = false;
        LOG_ERROR("Failed to error msg str char %d", i);
        return false;
      }
    }

    if(!buffer->load(this->int_args_count_) || !buffer->load(this->real_args_count_) || !buffer->load(this->error_msg_length_)){
        rtn = false;
        LOG_ERROR("Failed to load arg counts");
        return rtn;
    }
    
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load mik action trigger data");
  }
  return rtn;
}

bool MikSimpleActionReply::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mik simple action reply unload");
  
  // Unload all fixed fields first
  if(buffer->unload(this->error_msg_length_) && buffer->unload(this->real_args_count_) && buffer->unload(this->int_args_count_))
  { 
    // Clear vectors and add space for the expected number of elements
    this->additional_int_args_.clear();
    this->additional_real_args_.clear();
    this->error_msg_.clear();
    // Unload additional real args -> TODO: need to load backwards -> first element in vector is last as we read from behind
    for (shared_int i = this->error_msg_length_ - 1; i >= 0 ; i--)
    {
      shared_char val;
      if (!buffer->unload(val))
      {
        rtn = false;
        LOG_ERROR("Failed to unload error msg char %f", i);
        return false;
      }
      this->error_msg_.push_back(val);
    }
    std::reverse(this->error_msg_.begin(), this->error_msg_.end());

    // Unload additional real args -> TODO: need to load backwards -> first element in vector is last as we read from behind
    for (shared_int i = this->real_args_count_ - 1; i >= 0 ; i--)
    {
      shared_real val;
      if (!buffer->unload(val))
      {
        rtn = false;
        LOG_ERROR("Failed to unload additional real arg %f", i);
        return false;
      }
      this->additional_real_args_.push_back(val);
    }
    std::reverse(this->additional_real_args_.begin(),
             this->additional_real_args_.end());
    
    // Unload additional int args
    for (shared_int i = this->int_args_count_ - 1; i >= 0; i--)
    {
      shared_int val;
      if (!buffer->unload(val))
      {
        rtn = false;
        LOG_ERROR("Failed to unload additional int arg %d", i);
        return false;
      }
      this->additional_int_args_.push_back(val);
    }
    std::reverse(this->additional_int_args_.begin(),
             this->additional_int_args_.end());
    
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload mik simple action reply data");
    return rtn;
  }
  if (!(buffer->unload(this->action_status_) &&
      buffer->unload(this->request_id_) &&
      buffer->unload(this->action_id_))){
    rtn = false;
    LOG_ERROR("Failed to unload mik simple action reply data");
    return rtn;
  }
  
  return rtn;
}

unsigned int MikSimpleActionReply::byteLength()
{
  // Calculate size: 3 IDs + 3 counters + additional int args + additional real args
  
  return (3 + 3) * sizeof(shared_int) +
         this->getAdditionalIntCount() * sizeof(shared_int) +
         this->getAdditionalRealCount() * sizeof(shared_real) +
         this->getErrorMsgLength() * sizeof(shared_char);
}

}