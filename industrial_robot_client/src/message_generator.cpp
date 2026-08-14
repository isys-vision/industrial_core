/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024, Industrial Core
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
 * 	* Neither the name of the Industrial Core, nor the names
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

#include "industrial_robot_client/message_generator.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/mikado_messages/mik_status_message.h"
#include "simple_message/mikado_classes/mik_status.h"
#include "simple_message/mikado_messages/mik_action_trigger_message.h"
#include "simple_message/mikado_classes/mik_action_trigger.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/smpl_msg_connection.h"

using namespace industrial::simple_message;
using namespace industrial::mik_status_message;
using namespace industrial::mik_action_trigger_message;
using namespace industrial::mik_action_trigger;
using namespace industrial::mik_status;

namespace industrial_robot_client
{

void createMikStatusMessage(SimpleMessage& simple_message)
{
  MikStatus status;
  status.init(industrial::mik_status::MikStates::MS_FALSE,
            industrial::mik_status::MikStates::CS_TRUE,
            industrial::mik_status::MikStates::CoS_UNKNOWN,
            industrial::mik_status::MikStates::RS_TRUE,
            industrial::mik_status::MikStates::RuS_TRUE);
  MikStatusMessage msg;
  msg.init(status);
  msg.toTopic(simple_message);
  // print out message for debugging
  printf("\n[CLIENT] --- Created Mik Status Message ---\n");
  msg.status_.print();
}

void createActionTriggerMessage(SimpleMessage& simple_message)
{
  MikActionTrigger action_trigger;
  
  // Initialize with dummy action trigger data
  // Action ID: 1001 (find container action)
  // Camera ID: 1, Product ID: 100, Gripper ID: 200
  // ROI ID: 10, Pickzone ID: 5
  // Additional int args: [1, 2, 3]
  // Additional real args: [1.5, 2.5, 3.5]
  
  std::vector<industrial::shared_types::shared_int> int_args;
  int_args.push_back(1);
  int_args.push_back(2);
  int_args.push_back(3);
  
  std::vector<industrial::shared_types::shared_real> real_args;
  real_args.push_back(-113.5);
  real_args.push_back(1.5);
  real_args.push_back(2.5);
  real_args.push_back(3.5);

  printf("Sizes befre int: %d | real: %d", int_args.size(), real_args.size());
  
  action_trigger.init(1001,  // action_id (FIND_CONTAINER)
                      222,   // request_id
                      1,     // camera_id
                      100,   // product_id
                      200,   // gripper_id
                      10,    // roi_id
                      5,     // pickzone_id
                      int_args,
                      real_args);
  
  MikActionTriggerMessage msg;
  msg.init(action_trigger);
  msg.toTopic(simple_message);
  // print out message for debugging
  printf("\n[CLIENT] --- Created Action Trigger Message ---\n");
  msg.action_trigger_.print();
}

}