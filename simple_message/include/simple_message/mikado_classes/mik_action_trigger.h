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

#ifndef MIK_ACTION_TRIGGER_H
#define MIK_ACTION_TRIGGER_H

#ifndef FLATHEADERS
#include <stdio.h>
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif

#include <vector>

namespace industrial
{
  namespace mik_action_trigger
{
/**
 * \brief Class encapsulates mik action trigger data.
 * THIS CLASS IS NOT THREAD-SAFE
 */

class MikActionTrigger : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  MikActionTrigger(void);
  /**
   * \brief Destructor
   *
   */
  ~MikActionTrigger(void);

  /**
   * \brief Initializes an empty mik action trigger
   *
   */
  void init();

  /**
   * \brief Initializes a full mik action trigger
   *
   * \param action_id Action identifier
   * \param request_id Request identifier
   * \param camera_id Camera identifier
   * \param product_id Product identifier
   * \param gripper_id Gripper identifier
   * \param roi_id Region of interest identifier
   * \param pickzone_id Pick zone identifier
   * \param additional_int_args Vector of additional integer arguments (max 10)
   * \param additional_real_args Vector of additional real arguments (max 10)
   */
  void init(shared_types::shared_int action_id, shared_types::shared_int request_id,
            shared_types::shared_int camera_id, shared_types::shared_int product_id, 
            shared_types::shared_int gripper_id, shared_types::shared_int roi_id, 
            shared_types::shared_int pickzone_id, std::vector<shared_types::shared_int> additional_int_args,
            std::vector<shared_types::shared_real> additional_real_args);

  // Getters for action IDs
  shared_types::shared_int getActionId() const
  {
    return this->action_id_;
  }

  shared_types::shared_int getRequestId() const
  {
    return this->request_id_;
  }

  shared_types::shared_int getCameraId() const
  {
    return this->camera_id_;
  }

  shared_types::shared_int getProductId() const
  {
    return this->product_id_;
  }

  shared_types::shared_int getGripperId() const
  {
    return this->gripper_id_;
  }

  shared_types::shared_int getRoiId() const
  {
    return this->roi_id_;
  }

  shared_types::shared_int getPickzoneId() const
  {
    return this->pickzone_id_;
  }

  // Getters for additional arguments
  const std::vector<shared_types::shared_int> &getAdditionalIntArgs() const
  {
    return this->additional_int_args_;
  }

  const std::vector<shared_types::shared_real> &getAdditionalRealArgs() const
  {
    return this->additional_real_args_;
  }

  // Setters for action IDs
  void setActionId(shared_types::shared_int action_id)
  {
    this->action_id_ = action_id;
  }

  void setRequestId(shared_types::shared_int request_id)
  {
    this->request_id_ = request_id;
  }

  void setCameraId(shared_types::shared_int camera_id)
  {
    this->camera_id_ = camera_id;
  }

  void setProductId(shared_types::shared_int product_id)
  {
    this->product_id_ = product_id;
  }

  void setGripperId(shared_types::shared_int gripper_id)
  {
    this->gripper_id_ = gripper_id;
  }

  void setRoiId(shared_types::shared_int roi_id)
  {
    this->roi_id_ = roi_id;
  }

  void setPickzoneId(shared_types::shared_int pickzone_id)
  {
    this->pickzone_id_ = pickzone_id;
  }

  // Setters for additional arguments
  void setAdditionalIntArgs(const std::vector<shared_types::shared_int> &args)
  {
    if(args.size() > 10){
      printf("Could not set additional int arguments to action trigger. Max 10 int args allowed.");
      return;
    }
    this->additional_int_args_ = args;
    setIntArgCount(this->additional_int_args_.size());
  }

  void setAdditionalRealArgs(const std::vector<shared_types::shared_real> &args)
  {
    if(args.size() > 10){
      printf("Could not set additional real arguments to action trigger. Max 10 real args allowed.");
      return;
    }
    this->additional_real_args_ = args;
    setRealArgCount(this->additional_real_args_.size());
  }


void setIntArgCount(int int_count)
{
  int_args_count_ = int_count;
}

void setRealArgCount(int real_count)
{
  real_args_count_ = real_count;
}

int getAdditionalIntCount()
{
  return this->additional_int_args_.size();
}

int getAdditionalRealCount()
{
  return this->additional_real_args_.size();
}

void print()
{
  printf("Printing Action Trigger\n");
  printf("  Action ID:      %d\n", this->getActionId());
  printf("  Request ID:      %d\n", this->getRequestId());
  printf("  Camera ID:      %d\n", this->getCameraId());
  printf("  Product ID:     %d\n", this->getProductId());
  printf("  Gripper ID:     %d\n", this->getGripperId());
  printf("  ROI ID:         %d\n", this->getRoiId());
  printf("  Pickzone ID:    %d\n", this->getPickzoneId());
  // Print additional int args
  const std::vector<shared_types::shared_int>& intArgs = this->getAdditionalIntArgs();
  printf("  Int Args (count=%d): [", intArgs.size());
  for (size_t i = 0; i < intArgs.size(); i++)
  {
    printf("%d", intArgs[i]);
    if (i < intArgs.size() - 1) printf(", ");
  }
  printf("]\n");
  
  // Print additional real args
  const std::vector<shared_types::shared_real>& realArgs = this->getAdditionalRealArgs();
  printf("  Real Args (count=%d): [", realArgs.size());
  for (size_t i = 0; i < realArgs.size(); i++)
  {
    printf("%f", realArgs[i]);
    if (i < realArgs.size() - 1) printf(", ");
  }
  printf("]\n");
}

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(MikActionTrigger &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(MikActionTrigger &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength();

private:
  /**
   * \brief action identifier
   */
  industrial::shared_types::shared_int action_id_;

  /**
   * \brief request identifier
   */
  industrial::shared_types::shared_int request_id_;

  /**
   * \brief camera identifier
   */
  industrial::shared_types::shared_int camera_id_;

  /**
   * \brief product identifier
   */
  industrial::shared_types::shared_int product_id_;

  /**
   * \brief gripper identifier
   */
  industrial::shared_types::shared_int gripper_id_;

  /**
   * \brief region of interest identifier
   */
  industrial::shared_types::shared_int roi_id_;

  /**
   * \brief pick zone identifier
   */
  industrial::shared_types::shared_int pickzone_id_;

  /**
   * \brief additional integer arguments
   */
  std::vector<industrial::shared_types::shared_int> additional_int_args_;

  /**
   * \brief additional real arguments
   */
  std::vector<industrial::shared_types::shared_real> additional_real_args_;

  /**
   * \brief int argument count
   */
  industrial::shared_types::shared_int int_args_count_;

  /**
   * \brief real argument count
   */
  industrial::shared_types::shared_int real_args_count_;

};
}

}

#endif /* MIK_ACTION_TRIGGER_H */