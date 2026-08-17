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

#ifndef MIKADO_DYNAMIC_JOINT_TRAJ_PT
#define MIKADO_DYNAMIC_JOINT_TRAJ_PT

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include <stdio.h>
#else
#include "joint_data.h"
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using namespace industrial::shared_types;

namespace industrial
{
namespace mik_dynamic_joint_traj_pt
{


/**
 * \brief Class encapsulated joint trajectory point data.  The point data
 * serves as a waypoint along a trajectory and is meant to mirror the
 * JointTrajectoryPoint message.
 *
 * This class is similar to the simple_message joint_traj_pt class, but this
 * class provides the full message contents directly to the robot controller,
 * rather than simplifying the velocity duration.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type            size
 *   trajectory_id       (shared_int)    4  bytes
 *   sequence            (shared_int)    4  bytes
 *   positions           (shared_real)   4 * N bytes
 *   axis_count          (shared_int)    4  bytes
 *   velocity            (shared_real)   4  bytes
 *   motion_type         (shared_int)    4  bytes
 *   traj_part           (shared_int)    4  bytes
 *   is_last_traj_pt     (shared_bool)   1  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class MikadoDynamicJointsTrajPt : public industrial::simple_serialize::SimpleSerialize
{
public:

  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  MikadoDynamicJointsTrajPt(void);
  /**
   * \brief Destructor
   *
   */
  ~MikadoDynamicJointsTrajPt(void);

  /**
   * \brief Initializes a empty dynamic joint trajectory point
   *
   */
  void init();

  /**
   * \brief Initializes a dynamic joint trajectory point
   *
   */
  void init(shared_int trajectory_id,
            shared_int sequence,
            std::vector<shared_real> &positions,
            shared_real velocity,
            shared_int motion_type,
            shared_int trajectory_part,
            shared_bool is_last_traj_pt);


  // getters

  shared_int getTrajectoryId()
  {
    return this->trajectory_id_;
  }

  shared_int getSequence()
  {
    return this->sequence_;
  }

  const std::vector<shared_real> &getPositions() const
  {
    return this->positions_;
  }

  shared_int getAxisCount()
  {
    return this->axis_count_;
  }

  shared_real getVelocity()
  {
    return this->velocity_;
  }

  shared_int getMotionType()
  {
    return this->motion_type_;
  }

  shared_int getTrajectoryPart()
  {
    return this->trajectory_part_;
  }

  shared_bool getIsLastTrajPt()
  {
    return this->is_last_traj_pt_;
  }

  // setters
  void setTrajectoryId(shared_int trajectory_id)
  {
    this->trajectory_id_ = trajectory_id;
  }

  void setSequence(shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  void setPositions(const std::vector<shared_types::shared_real> &positions)
  {
    if(positions.size() > 10){
      printf("Could not set positions to traj pt. Max 10 position components allowed.");
      return;
    }
    this->positions_ = positions;
    setAxisCount(this->positions_.size());
  }

  void setAxisCount(shared_int axis_count)
  {
    this->axis_count_ = axis_count;
  }

  void setVelocity(shared_real velocity)
  {
    this->velocity_ = velocity;
  }

  void setMotionType(shared_int motion_type)
  {
    this->motion_type_ = motion_type;
  }

  void setTrajectoryPart(shared_int trajectory_part)
  {
    this->trajectory_part_ = trajectory_part;
  }

  void setIsLastTrajPt(shared_bool is_last_traj_pt)
  {
    this->is_last_traj_pt_ = is_last_traj_pt;
  }

  // other

  void clearPositions()
  {
    this->positions_.clear();
    this->setAxisCount(0);
  }


  void print()
  {
    printf("Printing Dynamic Joints Trajectory Point \n");
    printf("  Trajectory ID:   %d\n", this->getTrajectoryId());
    printf("  Sequence ID:     %d\n", this->getSequence());
    printf("  Axis Count:      %d\n", this->getAxisCount());
    printf("  Velocity  :      %f\n", this->getVelocity());
    printf("  Motion Type:     %d\n", this->getMotionType());
    printf("  Trajectory Part: %d\n", this->getTrajectoryPart());
    printf("  Is last traj pt: %d\n", this->getIsLastTrajPt());
    
    // Print additional real args
    const std::vector<shared_types::shared_real>& positions = this->getPositions();
    printf("  Positions (count=%ld): [", positions.size());
    for (size_t i = 0; i < positions.size(); i++)
    {
      printf("%f", positions[i]);
      if (i < positions.size() - 1) printf(", ");
    }
    printf("]\n");
  }


  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(MikadoDynamicJointsTrajPt &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(MikadoDynamicJointsTrajPt &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return 5 * sizeof(shared_int) +
           (this->axis_count_ + 1) * sizeof(shared_real) +
           2 * sizeof(shared_bool);
  }

private:
  /**
   * \brief trajectory id
   */
  shared_int trajectory_id_;

  /**
   * \brief trajectory sequence number
   */
  shared_int sequence_;

  /*
   * \brief additional real arguments
   */
  std::vector<shared_real> positions_;

  /**
   * \brief number of axis contained in position array
   */
  shared_int axis_count_;

  /**
   * \brief robot velocity when moving to this points
   */
  shared_real velocity_;

  /**
   * \brief motion type (joint, linear, etc.)
   */
  shared_int motion_type_;

  /**
   * \brief trajectory part (calibration, grasp, etc.)
   */
  shared_int trajectory_part_;

  /**
   * \brief is last trajectory point
   */
  shared_bool is_last_traj_pt_;



};

}
}

#endif /* MIKADO_DYNAMIC_JOINT_TRAJ_PT */
