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

#include "industrial_robot_client/message_decoder.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/mikado_messages/mik_status_message.h"
#include "simple_message/mikado_classes/mik_status.h"
#include "simple_message/mikado_messages/mik_action_trigger_message.h"
#include "simple_message/mikado_classes/mik_action_trigger.h"
#include "simple_message/mikado_messages/mik_simple_action_reply_message.h"
#include "simple_message/mikado_classes/mik_simple_action_reply.h"
#include "simple_message/mikado_messages/mik_connection_info_message.h"
#include "simple_message/mikado_classes/mik_connection_info.h"
#include "simple_message/mikado_classes/mik_simple_action_reply.h"
#include "simple_message/mikado_messages/mik_dynamic_joints_traj_pt_message.h"
#include "simple_message/mikado_classes/mik_dynamic_joints_traj_pt.h"

using namespace industrial::simple_message;
using namespace industrial::mik_status_message;
using namespace industrial::mik_action_trigger_message;
using namespace industrial::mik_action_trigger;
using namespace industrial::mik_simple_action_reply_message;
using namespace industrial::mik_simple_action_reply;
using namespace industrial::mik_conn_info_message;
using namespace industrial::mik_conn_info;
using namespace industrial::mik_status;
using namespace industrial::mik_dynamic_joint_traj_pt_msg;
using namespace industrial::mik_dynamic_joint_traj_pt;

namespace industrial_robot_client
{

SimpleMessage decodeAndRepackMessage(SimpleMessage& simple_message){
  switch(simple_message.getMessageType()){
    case mik_msg_type::MIK_STATUS:{
      return decodeAndRepackMikStatusMessage(simple_message);
    }
    case mik_msg_type::ACTION_TRIG:{
      return decodeAndRepackActionTriggerMessage(simple_message);
    }
    case mik_msg_type::SIMPLE_REPLY:{
      return decodeAndRepackMikSimpleActionReplyMessage(simple_message);
    }
    case mik_msg_type::CONN_INFO:{
      return decodeAndRepackMikConnInfoMessage(simple_message);
    }
    case mik_msg_type::TRAJ_PT:{
      return decodeAndRepackMikTrajPtMessage(simple_message);
    }
    default:{
      printf("[Message Decoder] Message Type with ID %d not known to decoder.", simple_message.getMessageType());
      exit(-1);
    }
  }
}

SimpleMessage decodeAndRepackMikStatusMessage(SimpleMessage& simple_message)
{
  MikStatusMessage status_msg;
  if (status_msg.init(simple_message)){
    MikStatus &status = status_msg.status_;

    printf("\n[SERVER] --- Mikado Status ---\n");
    status.print();

    MikStatus reply_status;
    reply_status.copyFrom(status);

    MikStatusMessage reply_msg;
    reply_msg.init(reply_status);

    SimpleMessage reply;
    reply_msg.toTopic(reply);
    return reply;
  }
  printf("[Message Decoder] Failed to decode Status Message");
  exit(-1);
}

SimpleMessage decodeAndRepackActionTriggerMessage(SimpleMessage& simple_message)
{
  MikActionTriggerMessage action_trigger_msg;
  if (action_trigger_msg.init(simple_message)){
    MikActionTrigger &action_trigger = action_trigger_msg.action_trigger_;

    printf("\n[SERVER] --- Mikado Action Trigger ---\n");
    action_trigger.print();

    MikActionTrigger reply_action_trigger;
    reply_action_trigger.copyFrom(action_trigger);

    MikActionTriggerMessage reply_msg;
    reply_msg.init(reply_action_trigger);

    SimpleMessage reply;
    reply_msg.toTopic(reply);
    return reply;
  }
  printf("[Message Decoder] Failed to decode Action Trigger Message");
  exit(-1);
}

SimpleMessage decodeAndReplyToActionTriggerMessage(SimpleMessage& simple_message)
{
  MikActionTriggerMessage action_trigger_msg;
  if (action_trigger_msg.init(simple_message)){
    MikActionTrigger &action_trigger = action_trigger_msg.action_trigger_;

    MikSimpleActionReply reply_simple_action_reply;
    reply_simple_action_reply.init();
    reply_simple_action_reply.setActionId(action_trigger.getActionId());
    reply_simple_action_reply.setRequestId(action_trigger.getRequestId());
    reply_simple_action_reply.setActionStatus(1500); // EKI interprest ints as unsigned, therefore use positive status range for errors
    std::string error_msg = "[ActionTrigger] Error: OH NO, AN ERROR OCCURED!"; 
    reply_simple_action_reply.setErrorMsg(error_msg);

    MikSimpleActionReplyMessage reply_msg;
    reply_msg.init(reply_simple_action_reply);

    SimpleMessage reply;
    reply_msg.toTopic(reply);
    return reply;
  }
  printf("[Message Decoder] Failed to decode Action Trigger Message");
  exit(-1);
}

SimpleMessage decodeAndRepackMikSimpleActionReplyMessage(SimpleMessage& simple_message)
{
  MikSimpleActionReplyMessage simple_action_reply_msg;
  if (simple_action_reply_msg.init(simple_message)){
    MikSimpleActionReply &simple_action_reply = simple_action_reply_msg.mik_simple_action_reply_;

    printf("\n[SERVER] --- Mikado Simple Action Reply ---\n");
    simple_action_reply.print();

    MikSimpleActionReply reply_simple_action_reply;
    reply_simple_action_reply.copyFrom(simple_action_reply);

    MikSimpleActionReplyMessage reply_msg;
    reply_msg.init(reply_simple_action_reply);

    SimpleMessage reply;
    reply_msg.toTopic(reply);
    return reply;
  }
  printf("[Message Decoder] Failed to decode Simple Action Reply Message");
  exit(-1);
}

SimpleMessage decodeAndRepackMikConnInfoMessage(SimpleMessage& simple_message)
{
  MikadoConnectionInfoMessage conn_info_msg;
  if (conn_info_msg.init(simple_message)){
    MikadoConnectionInfo &conn_info = conn_info_msg.connection_info_;

    printf("\n[SERVER] --- Mikado Connection Info ---\n");
    conn_info.print();

    MikadoConnectionInfo reply_conn_info;
    reply_conn_info.copyFrom(conn_info);

    MikadoConnectionInfoMessage reply_msg;
    reply_msg.init(reply_conn_info);

    SimpleMessage reply;
    reply_msg.toTopic(reply);
    return reply;
  }
  printf("[Message Decoder] Failed to decode Connection Info Message");
  exit(-1);
}

SimpleMessage decodeAndRepackMikTrajPtMessage(SimpleMessage& simple_message){
  MikadoDynamicJointsTrajPtMessage traj_pt_msg;
  if (traj_pt_msg.init(simple_message)){
    MikadoDynamicJointsTrajPt &traj_pt = traj_pt_msg.point_;

    printf("\n[SERVER] --- Mikado Trajectory Point ---\n");
    traj_pt.print();

    MikadoDynamicJointsTrajPt reply_traj_pt;
    reply_traj_pt.copyFrom(traj_pt);

    MikadoDynamicJointsTrajPtMessage reply_msg;
    reply_msg.init(reply_traj_pt);

    SimpleMessage reply;
    reply_msg.toTopic(reply);
    return reply;
  }
  printf("[Message Decoder] Failed to decode Trajectory Point Message");
  exit(-1);
}

}