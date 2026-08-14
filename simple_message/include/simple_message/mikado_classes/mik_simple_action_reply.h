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

#ifndef MIK_SIMPLE_ACTION_REPLY_H
#define MIK_SIMPLE_ACTION_REPLY_H

#ifndef FLATHEADERS
#include <stdio.h>
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include <string>
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif

#include <vector>

namespace industrial
{
  namespace mik_simple_action_reply
{
/**
 * \brief Class encapsulates mik simple action reply data.
 * THIS CLASS IS NOT THREAD-SAFE
 */

class MikSimpleActionReply : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  MikSimpleActionReply(void);
  /**
   * \brief Destructor
   *
   */
  ~MikSimpleActionReply(void);

  /**
   * \brief Initializes an empty mik simple action reply
   *
   */
  void init();

  /**
   * \brief Initializes a full mik simple action reply
   *
   * \param action_id Action identifier
   * \param request_id Request identifier
   * \param action_status Status Code of the Action. > 0 means that an error occured
   * \param additional_int_args Vector of additional integer results (max 10)
   * \param additional_real_args Vector of additional real results (max 10)
   * \param error_msg String containing error msg (only present if error occured)
   */
  void init(shared_types::shared_int action_id, shared_types::shared_int request_id,
            shared_types::shared_int action_status, std::vector<shared_types::shared_int> additional_int_args,
            std::vector<shared_types::shared_real> additional_real_args, std::string error_msg);

  // Getters for action IDs
  shared_types::shared_int getActionId() const
  {
    return this->action_id_;
  }

  shared_types::shared_int getRequestId() const
  {
    return this->request_id_;
  }

  shared_types::shared_int getActionStatus() const
  {
    return this->action_status_;
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

  const std::string &getErrorMsg() const
  {
    return this->error_msg_;
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

  void setActionStatus(shared_types::shared_int action_status)
  {
    this->action_status_ = action_status;
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

  void setErrorMsg(std::string &error_msg)
  {
    if(error_msg.length() > 128){
      printf("Could not set error msg. Max 128 chars allowed.");
      return;
    }
    this->error_msg_ = error_msg;
    setErrorMsgLength(this->error_msg_.length());
  }


void setIntArgCount(int int_count)
{
  this->int_args_count_ = int_count;
}

void setRealArgCount(int real_count)
{
  this->real_args_count_ = real_count;
}

void setErrorMsgLength(int error_msg_length)
{
  this->error_msg_length_ = error_msg_length;
}

int getAdditionalIntCount()
{
  return this->additional_int_args_.size();
}

int getAdditionalRealCount()
{
  return this->additional_real_args_.size();
}

int getErrorMsgLength()
{
  return this->error_msg_.length();
}

void print()
{
  printf("Printing Simple Action Reply\n");
  printf("  Action ID:      %d\n", this->getActionId());
  printf("  Request ID:     %d\n", this->getRequestId());
  printf("  Action Status:  %d\n", this->getActionStatus());
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
  printf("  Error Message:  %s\n", this->getErrorMsg().c_str());
  printf("  Error no:  %d\n", this->getErrorMsgLength());
}

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(MikSimpleActionReply &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(MikSimpleActionReply &rhs);

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
   * \brief action status
   */
  industrial::shared_types::shared_int action_status_;

  /**
   * \brief additional integer arguments
   */
  std::vector<industrial::shared_types::shared_int> additional_int_args_;

  /**
   * \brief additional real arguments
   */
  std::vector<industrial::shared_types::shared_real> additional_real_args_;

  /**
   * \brief error msg
   */
  std::string error_msg_;

  /**
   * \brief int argument count
   */
  industrial::shared_types::shared_int int_args_count_;

  /**
   * \brief real argument count
   */
  industrial::shared_types::shared_int real_args_count_;

  /**
   * \brief real argument count
   */
  industrial::shared_types::shared_int error_msg_length_;

};
}

}

#endif /* MIK_SIMPLE_ACTION_REPLY_H */