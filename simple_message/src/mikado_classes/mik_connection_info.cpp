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
#include "simple_message/mikado_classes/mik_connection_info.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "mikado_connection_info.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif


using namespace industrial::shared_types;

namespace industrial
{
namespace mik_conn_info
{

MikadoConnectionInfo::MikadoConnectionInfo(void)
{
  this->init();
}
MikadoConnectionInfo::~MikadoConnectionInfo(void)
{

}

void MikadoConnectionInfo::init()
{
  int default_mikado_product = mik_product_type::INVALID;
  int default_rotation_convention = mik_rotation_convention::INVALID;
  int default_num_ax_traj = 6;
  int default_num_ax_state = 6;
  int default_num_eax_traj = 4;
  int default_num_eax_state = 4;

  this->init(default_mikado_product, default_rotation_convention, default_num_ax_traj, default_num_ax_state, default_num_eax_traj,
             default_num_eax_state, true, true, false);
}

void MikadoConnectionInfo::init(shared_int mikado_product, shared_int mikado_rotation_convention, shared_int num_ax_traj, shared_int num_ax_state, shared_int num_eax_traj,
                       shared_int num_eax_state, shared_bool is_traj_radian, shared_bool is_state_radian, shared_bool is_traj_is_moving)
{
  this->setMikadoProduct(mikado_product);
  this->setRotationConvention(mikado_rotation_convention);
  this->setNumAxTraj(num_ax_traj);
  this->setNumAxState(num_ax_state);
  this->setNumEaxTraj(num_eax_traj);
  this->setNumEaxState(num_eax_state);
  this->setIsTrajRadian(is_traj_radian);
  this->setIsStateRadian(is_state_radian);
  this->setIsTrajIsMoving(is_traj_is_moving);
}

void MikadoConnectionInfo::copyFrom(MikadoConnectionInfo &src)
{
  this->setMikadoProduct(src.getMikadoProduct());
  this->setRotationConvention(src.getRotationConvention());
  this->setNumAxTraj(src.getNumAxTraj());
  this->setNumAxState(src.getNumAxState());
  this->setNumEaxTraj(src.getNumEaxTraj());
  this->setNumEaxState(src.getNumEaxState());
  this->setIsTrajRadian(src.getIsTrajRadian());
  this->setIsStateRadian(src.getIsStateRadian());
  this->setIsTrajIsMoving(src.getIsTrajIsMoving());
}

bool MikadoConnectionInfo::operator==(MikadoConnectionInfo &rhs)
{
  return this->mikado_product_    == rhs.mikado_product_
      && this->rot_convention_    == rhs.rot_convention_
      && this->num_ax_traj_       == rhs.num_ax_traj_
      && this->num_ax_state_      == rhs.num_ax_state_
      && this->num_eax_traj_      == rhs.num_eax_traj_
      && this->num_eax_state_     == rhs.num_eax_state_
      && this->is_traj_radian_    == rhs.is_traj_radian_
      && this->is_state_radian_   == rhs.is_state_radian_
      && this->is_traj_is_moving_ == rhs.is_traj_is_moving_;
}

bool MikadoConnectionInfo::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  if (buffer->load(this->mikado_product_)
      && buffer->load(this->rot_convention_)
      && buffer->load(this->num_ax_traj_)
      && buffer->load(this->num_ax_state_)
      && buffer->load(this->num_eax_traj_)
      && buffer->load(this->num_eax_state_)
      && buffer->load(this->is_traj_radian_)
      && buffer->load(this->is_state_radian_)
      && buffer->load(this->is_traj_is_moving_))
  {
    LOG_COMM("Connection info successfully loaded");
    rtn = true;
  } else {
    LOG_COMM("Connection info not loaded");
    rtn = false;
  }

  return rtn;
}

bool MikadoConnectionInfo::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing connection info unload");
   if (buffer->unload(this->is_traj_is_moving_)
      && buffer->unload(this->is_state_radian_)
      && buffer->unload(this->is_traj_radian_)
      && buffer->unload(this->num_eax_state_)
      && buffer->unload(this->num_eax_traj_)
      && buffer->unload(this->num_ax_state_)
      && buffer->unload(this->num_ax_traj_)
      && buffer->unload(this->mikado_product_)
      && buffer->unload(this->rot_convention_))
  {

    rtn = true;
    LOG_COMM("Connection info successfully unloaded");
  }

  else
  {
    LOG_ERROR("Failed to unload connection info");
    rtn = false;
  }

  return rtn;
}

}
}

