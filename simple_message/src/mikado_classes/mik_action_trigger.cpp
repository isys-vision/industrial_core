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
#include "simple_message/mikado_classes/mik_action_trigger.h"
#include "simple_message/shared_types.h"
#include "simple_message/byte_array.h"
#include "simple_message/log_wrapper.h"
#else
#include "mik_action_trigger.h"
#include "shared_types.h"
#include "byte_array.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::mik_action_trigger;

namespace industrial
{

MikActionTrigger::MikActionTrigger(void)
{
  this->init();
}

MikActionTrigger::~MikActionTrigger(void)
{
}

void MikActionTrigger::init()
{
  this->action_id_ = 0;
  this->camera_id_ = 0;
  this->product_id_ = 0;
  this->gripper_id_ = 0;
  this->roi_id_ = 0;
  this->pickzone_id_ = 0;
  this->additional_int_args_.clear();
  this->additional_real_args_.clear();
  this->int_args_count_ = 0;
  this->real_args_count_ = 0;
}

void MikActionTrigger::init(shared_types::shared_int action_id, shared_types::shared_int camera_id,
                            shared_types::shared_int product_id, shared_types::shared_int gripper_id,
                            shared_types::shared_int roi_id, shared_types::shared_int pickzone_id,
                            std::vector<shared_types::shared_int> additional_int_args,
                            std::vector<shared_types::shared_real> additional_real_args)
{
  this->setActionId(action_id);
  this->setCameraId(camera_id);
  this->setProductId(product_id);
  this->setGripperId(gripper_id);
  this->setRoiId(roi_id);
  this->setPickzoneId(pickzone_id);
  this->setAdditionalIntArgs(additional_int_args);
  this->setAdditionalRealArgs(additional_real_args);
}

void MikActionTrigger::copyFrom(MikActionTrigger &src)
{
  this->action_id_ = src.action_id_;
  this->camera_id_ = src.camera_id_;
  this->product_id_ = src.product_id_;
  this->gripper_id_ = src.gripper_id_;
  this->roi_id_ = src.roi_id_;
  this->pickzone_id_ = src.pickzone_id_;
  this->additional_int_args_ = src.additional_int_args_;
  this->additional_real_args_ = src.additional_real_args_;
  this->int_args_count_ = src.int_args_count_;
  this->real_args_count_ = src.real_args_count_;
}

bool MikActionTrigger::operator==(MikActionTrigger &rhs)
{
  if (this->action_id_ != rhs.action_id_ ||
      this->camera_id_ != rhs.camera_id_ ||
      this->product_id_ != rhs.product_id_ ||
      this->gripper_id_ != rhs.gripper_id_ ||
      this->roi_id_ != rhs.roi_id_ ||
      this->pickzone_id_ != rhs.pickzone_id_ ||
      this->int_args_count_ != rhs.int_args_count_ ||
      this->real_args_count_ != rhs.real_args_count_)
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

bool MikActionTrigger::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mik action trigger load");
  
  shared_int int_count = 0;
  shared_int real_count = 0;
  
  if (buffer->load(this->action_id_) &&
      buffer->load(this->camera_id_) &&
      buffer->load(this->product_id_) &&
      buffer->load(this->gripper_id_) &&
      buffer->load(this->roi_id_) &&
      buffer->load(this->pickzone_id_))
  {
    // Extract counts from packed arguments
    int_count = this->getAdditionalIntArgs().size();
    real_count = this->getAdditionalRealArgs().size();
    
    // Load additional int args
    for (shared_int i = 0; i < int_count; i++)
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
    for (shared_int i = 0; i < real_count; i++)
    {
      shared_real val = this->additional_real_args_.at(i);
      if (!buffer->load(val))
      {
        rtn = false;
        LOG_ERROR("Failed to load additional real arg %d", i);
        return false;
      }
    }

    if(!buffer->load(this->int_args_count_) || !buffer->load(this->real_args_count_)){
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

bool MikActionTrigger::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing mik action trigger unload");
  
  // Unload all fixed fields first
  if(buffer->unload(this->real_args_count_) && buffer->unload(this->int_args_count_))
  { 
    // Clear vectors and add space for the expected number of elements
    this->additional_int_args_.clear();
    this->additional_real_args_.clear();
    
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
    
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload mik action trigger data");
    return rtn;
  }
  if (!( buffer->unload(this->pickzone_id_) &&
      buffer->unload(this->roi_id_) &&
      buffer->unload(this->gripper_id_) &&
      buffer->unload(this->product_id_) &&
      buffer->unload(this->camera_id_) &&
      buffer->unload(this->action_id_))){
    rtn = false;
    LOG_ERROR("Failed to unload mik action trigger data");
    return rtn;
  }
  
  return rtn;
}

unsigned int MikActionTrigger::byteLength()
{
  // Calculate size: 6 IDs + 2 * counts + additional int args + additional real args
  
  return (6 + 2) * sizeof(shared_int) +
         this->getAdditionalIntCount() * sizeof(shared_int) +
         this->getAdditionalRealCount() * sizeof(shared_real);
}

}