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
#include "simple_message/mikado_classes/mik_dynamic_joints_traj_pt.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include <algorithm>
#else
#include "mikado_dynamic_joints_traj_pt.h"
#include "mikado_dynamic_joints.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;

namespace industrial
{
namespace mik_dynamic_joint_traj_pt
{

MikadoDynamicJointsTrajPt::MikadoDynamicJointsTrajPt(void)
{
  this->init();
}
MikadoDynamicJointsTrajPt::~MikadoDynamicJointsTrajPt(void)
{

}

void MikadoDynamicJointsTrajPt::init()
{
  this->setTrajectoryId(0);
  this->setSequence(0);
  this->positions_.clear();
  this->setVelocity(0.0);
  this->setMotionType(mik_motion_type::INVALID);
  this->setTrajectoryPart(mik_traj_type::INVALID);
  this->setIsLastTrajPt(false);
}

void MikadoDynamicJointsTrajPt::init(shared_int trajectory_id,
          shared_int sequence,
          std::vector<shared_real> & positions,
          shared_real velocity,
          shared_int motion_type,
          shared_int trajectory_part,
          shared_bool is_last_traj_pt)
{
  this->setTrajectoryId(trajectory_id);
  this->setSequence(sequence);
  this->setPositions(positions);
  this->setVelocity(velocity);
  this->setMotionType(motion_type);
  this->setTrajectoryPart(trajectory_part);
  this->setIsLastTrajPt(is_last_traj_pt);
}

void MikadoDynamicJointsTrajPt::copyFrom(MikadoDynamicJointsTrajPt &src)
{
  this->setTrajectoryId(src.getTrajectoryId());
  this->setSequence(src.getSequence());
  this->setPositions(src.getPositions());
  this->setVelocity(src.getVelocity());
  this->setMotionType(src.getMotionType());
  this->setTrajectoryPart(src.getTrajectoryPart());
  this->setIsLastTrajPt(src.getIsLastTrajPt());
}

bool MikadoDynamicJointsTrajPt::operator==(MikadoDynamicJointsTrajPt &rhs)
{
  if (this->trajectory_id_ != rhs.trajectory_id_ ||
      this->sequence_ != rhs.sequence_ ||
      this->axis_count_ != rhs.axis_count_ ||
      this->velocity_ != rhs.velocity_ ||
      this->motion_type_ != rhs.motion_type_ ||
      this->trajectory_part_ != rhs.trajectory_part_ ||
      this->is_last_traj_pt_ != rhs.is_last_traj_pt_)
  {
    return false;
  }
  
  // Compare positions
  if (this->positions_.size() != rhs.positions_.size())
  {
    return false;
  }
  for (size_t i = 0; i < this->positions_.size(); i++)
  {
    if (this->positions_[i] != rhs.positions_[i])
    {
      return false;
    }
  }
  
  return true;

}

bool MikadoDynamicJointsTrajPt::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing mik dynamic traj pt load");
  
  if (buffer->load(this->trajectory_id_) &&
      buffer->load(this->sequence_)){
    
    // Load position data
    for (shared_int i = 0; i < this->getAxisCount(); i++)
    {
      shared_real val = this->positions_.at(i);
      if (!buffer->load(val))
      {
        LOG_ERROR("Failed to load position %d", i);
        return false;
      }
    }

    if (!(buffer->load(this->axis_count_) &&
          buffer->load(this->velocity_) &&
          buffer->load(this->motion_type_) &&
          buffer->load(this->trajectory_part_) &&
          buffer->load(this->is_last_traj_pt_))){
            LOG_ERROR("Failed to load additional pt info");
            return false;
    }
  } else {
    LOG_ERROR("Failed to load mik dynamic joint traj pt");
    return false;
  }
  return true;
}

bool MikadoDynamicJointsTrajPt::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing mik dynamic joint traj pt unload");
  
  // Clear vectors and add space for the expected number of elements. Clear() also resets axis count so we do this here
  this->positions_.clear();

  // Unload all fixed fields first
  if(buffer->unload(this->is_last_traj_pt_) && buffer->unload(this->trajectory_part_) && buffer->unload(this->motion_type_) && buffer->unload(this->velocity_) && buffer->unload(this->axis_count_))
  { 
    
    // Unload positions
    for (shared_int i = this->axis_count_ - 1; i >= 0 ; i--)
    {
      shared_real val;
      if (!buffer->unload(val))
      {
        LOG_ERROR("Failed to unload position %d", i);
        return false;
      }
      this->positions_.push_back(val);
    }
    std::reverse(this->positions_.begin(), this->positions_.end());
  }
  else
  {
    LOG_ERROR("Failed to unload mik dynamic traj pt");
    return false;
  }
  if (!( buffer->unload(this->sequence_) &&
      buffer->unload(this->trajectory_id_))){
    LOG_ERROR("Failed to unload mik dynamic traj pt");
    return false;
  }
  return true;
}

}
}

