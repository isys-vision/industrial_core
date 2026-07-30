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

#ifndef MIK_STATUS_H
#define MIK_STATUS_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include <stdio.h>
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif

namespace industrial
{
namespace mik_status
{

/**
 * \brief Contains all Enums for MikStates
 *
 */
namespace MikStates
{

enum MikState
{
  MS_UNKNOWN = -1,
  MS_TRUE = 1,
  MS_FALSE = 0
};

enum CameraState
{
  CS_UNKNOWN = -1,
  CS_TRUE = 1,
  CS_FALSE = 0
};

enum CommState
{
  CoS_UNKNOWN = -1,
  CoS_TRUE = 1,
  CoS_FALSE = 0
};

enum RobotState
{
  RS_UNKNOWN = -1,
  RS_TRUE = 1,
  RS_FALSE = 0
};

enum RunningState
{
  RuS_UNKNOWN = -1,
  RuS_TRUE = 1,
  RuS_FALSE = 0
};

}

typedef MikStates::MikState MikState;
typedef MikStates::CameraState CameraState;
typedef MikStates::CommState CommState;
typedef MikStates::RobotState RobotState;
typedef MikStates::RunningState RunningState;

/**
 * \brief Class encapsulated mik status data.
 * THIS CLASS IS NOT THREAD-SAFE
 */

class MikStatus : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  MikStatus(void);
  /**
   * \brief Destructor
   *
   */
  ~MikStatus(void);

  /**
   * \brief Initializes an empty mik status
   *
   */
  void init();

  /**
   * \brief Initializes a full mik status message
   *
   */
  void init(MikState mik_state, CameraState camera_state, CommState comm_state,
            RobotState robot_state, RunningState running_state);

  MikState getMikState()
  {
    return MikState(mik_state_);
  }

  CameraState getCameraState()
  {
    return CameraState(camera_state_);
  }

  CommState getCommState()
  {
    return CommState(comm_state_);
  }

  RobotState getRobotState()
  {
    return RobotState(robot_state_);
  }

  RunningState getRunningState()
  {
    return RunningState(running_state_);
  }

  void setMikState(MikState mik_state)
  {
    this->mik_state_ = mik_state;
  }

  void setCameraState(CameraState camera_state)
  {
    this->camera_state_ = camera_state;
  }

  void setCommState(CommState comm_state)
  {
    this->comm_state_ = comm_state;
  }

  void setRobotState(RobotState robot_state)
  {
    this->robot_state_ = robot_state;
  }

  void setRunningState(RunningState running_state)
  {
    this->running_state_ = running_state;
  }

  void print()
  {
    printf("Printing Mik Status");
    printf("  Mik State:      %d\n", this->getMikState());
    printf("  Camera State:   %d\n", this->getCameraState());
    printf("  Comm State:     %d\n", this->getCommState());
    printf("  Robot State:    %d\n", this->getRobotState());
    printf("  Running State:  %d\n", this->getRunningState());
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(MikStatus &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(MikStatus &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 5 * sizeof(industrial::shared_types::shared_int);
  }

private:
  /**
   * \brief mikado state (see MikStates:MikState)
   */
  industrial::shared_types::shared_int mik_state_;

  /**
   * \brief camera state (see MikStates:CameraState)
   */
  industrial::shared_types::shared_int camera_state_;

  /**
   * \brief communication state (see MikStates:CommState)
   */
  industrial::shared_types::shared_int comm_state_;

  /**
   * \brief robot state (see MikStates:RobotState)
   */
  industrial::shared_types::shared_int robot_state_;

  /**
   * \brief running state (see MikStates:RunningState)
   */
  industrial::shared_types::shared_int running_state_;

};

}
}

#endif /* MIK_STATUS_H */