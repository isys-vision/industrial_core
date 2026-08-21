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
#include "simple_message/mikado_messages/mik_simple_action_reply_message.h"
#include "simple_message/mikado_classes/mik_simple_action_reply.h"
#include "simple_message/mikado_messages/mik_connection_info_message.h"
#include "simple_message/mikado_classes/mik_connection_info.h"
#include "simple_message/mikado_messages/mik_dynamic_joints_traj_pt_message.h"
#include "simple_message/mikado_classes/mik_dynamic_joints_traj_pt.h"
#include "simple_message/mikado_classes/mik_connection_info.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/smpl_msg_connection.h"

using namespace industrial::simple_message;
using namespace industrial::mik_status_message;
using namespace industrial::mik_action_trigger_message;
using namespace industrial::mik_action_trigger;
using namespace industrial::mik_status;
using namespace industrial::mik_simple_action_reply;
using namespace industrial::mik_simple_action_reply_message;
using namespace industrial::mik_conn_info;
using namespace industrial::mik_conn_info_message;
using namespace industrial::mik_dynamic_joint_traj_pt;
using namespace industrial::mik_dynamic_joint_traj_pt_msg;

namespace industrial_robot_client
{

void printMsg(SimpleMessage& simpleMsg){
  printf("\n--------------\n Printing msg:\n");
  switch(simpleMsg.getMessageType()){
    case mik_msg_type::MIK_STATUS:
    {
      MikStatusMessage reply_status_msg;
      if (reply_status_msg.init(simpleMsg)){
        reply_status_msg.status_.print();
      }
      break;
    }
    case mik_msg_type::ACTION_TRIG:
    {
      MikActionTriggerMessage action_trigger_msg;
      if (action_trigger_msg.init(simpleMsg)){
        action_trigger_msg.action_trigger_.print();
      }
      break;
    }
    case mik_msg_type::SIMPLE_REPLY:
    {
      MikSimpleActionReplyMessage simple_action_reply_msg;
      if (simple_action_reply_msg.init(simpleMsg)){
        simple_action_reply_msg.mik_simple_action_reply_.print();
      }
      break;
    }
    case mik_msg_type::CONN_INFO:
    {
      MikadoConnectionInfoMessage conn_info_msg;
      if (conn_info_msg.init(simpleMsg)){
        conn_info_msg.connection_info_.print();
      }
      break;
    }
    case mik_msg_type::TRAJ_PT:
    {
      MikadoDynamicJointsTrajPtMessage traj_pt_msg;
      if (traj_pt_msg.init(simpleMsg)){
        traj_pt_msg.point_.print();
      }
      break;
    }
    default:
    {
      printf("[CLIENT] Unknown message type, cannot print.");
    }
  }
  printf("\n--------------\n");
}

bool createMikMessage(SimpleMessage& simple_message, int msg_type){
  switch(msg_type){
    case mik_msg_type::ACTION_TRIG:
    {
      industrial_robot_client::createActionTriggerMessage(simple_message);
      break;
    }
    case mik_msg_type::MIK_STATUS:
    {
      industrial_robot_client::createMikStatusMessage(simple_message);
      break;
    }
    case mik_msg_type::SIMPLE_REPLY:
    {
      industrial_robot_client::createSimpleActionReplyMessage(simple_message);
      break;
    }
    case mik_msg_type::CONN_INFO:
    {
      industrial_robot_client::createConnectionInfoMessage(simple_message);
      break;
    }
     case mik_msg_type::TRAJ_PT:
    {
      industrial_robot_client::createTrajPtMessage(simple_message);
      break;
    }
    default:
    {
      printf("[Message Generator] Unkown message type.");
      return false;
    }
  }
  return true;
}

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

void createSimpleActionReplyMessage(SimpleMessage& simple_message)
{
  MikSimpleActionReply action_reply;
  
  std::vector<industrial::shared_types::shared_int> int_args;
  int_args.push_back(1);
  int_args.push_back(2);
  int_args.push_back(3);
  
  std::vector<industrial::shared_types::shared_real> real_args;
  real_args.push_back(-113.5);
  real_args.push_back(1.5);
  real_args.push_back(2.5);
  real_args.push_back(3.5);
  
  action_reply.init(1000,  // action_id (FIND_CONTAINER)
                      1,   // request_id
                      0,     // action_status
                      int_args,
                      real_args,
                    "Error 2: This action does not exists");
  
  MikSimpleActionReplyMessage msg;
  msg.init(action_reply);
  msg.toTopic(simple_message);
  // print out message for debugging
  printf("\n[CLIENT] --- Created Simple Action Reply Message ---\n");
  msg.mik_simple_action_reply_.print();
}

void createConnectionInfoMessage(SimpleMessage& simple_message)
{
  MikadoConnectionInfo conn_info;
  
  conn_info.init(mik_product_type::PICK, mik_rotation_convention::KUKA, 6, 1, 3, 4, true, false, true);
  
  MikadoConnectionInfoMessage msg;
  msg.init(conn_info);
  msg.toTopic(simple_message);
  // print out message for debugging
  printf("\n[CLIENT] --- Created Connection Info Message ---\n");
  msg.connection_info_.print();
}

void createTrajPtMessage(SimpleMessage& simple_message)
{
  MikadoDynamicJointsTrajPt traj_pt;

  std::vector<industrial::shared_types::shared_real> positions;
  positions.push_back(-113.5);
  positions.push_back(1.5);
  positions.push_back(2.5);
  positions.push_back(36.5);
  positions.push_back(33.5);
  positions.push_back(0.5);
  
  traj_pt.init(12, 2, positions, 13.6, mik_motion_type::JOINT, mik_traj_type::REGULAR, false);
  
  MikadoDynamicJointsTrajPtMessage msg;
  msg.init(traj_pt);
  msg.toTopic(simple_message);
  // print out message for debugging
  printf("\n[CLIENT] --- Created Trajectory Point Message ---\n");
  msg.point_.print();
}

}