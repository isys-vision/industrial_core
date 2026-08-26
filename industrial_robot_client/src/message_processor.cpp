#include "industrial_robot_client/message_processor.h"
#include "industrial_robot_client/mikado_utils.h"

#include <stdio.h>
#include <string>
#include <vector>

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

using namespace industrial::simple_message;
using namespace industrial::shared_types;
using namespace industrial::typed_message;
using namespace industrial::mik_status_message;
using namespace industrial::mik_status;
using namespace industrial::mik_action_trigger_message;
using namespace industrial::mik_action_trigger;
using namespace industrial::mik_simple_action_reply_message;
using namespace industrial::mik_simple_action_reply;
using namespace industrial::mik_conn_info_message;
using namespace industrial::mik_conn_info;
using namespace industrial::mik_dynamic_joint_traj_pt;
using namespace industrial::mik_dynamic_joint_traj_pt_msg;
using namespace industrial_robot_client::mikado_utils;

namespace industrial_robot_client
{

bool SimpleMessageProcessor::process(SimpleMessage& in_msg, SimpleMessage& reply)
{
  bool success = false;
  printf("\n[MikadoActionManager] --- Received Message ---\n");
  printf("Message Type: %d\n", in_msg.getMessageType());
  printf("Comm Type:    %d\n", in_msg.getCommType());
  printf("Reply Code:   %d\n", in_msg.getReplyCode());
  printf("Data Length:  %d bytes\n", in_msg.getDataLength());

  switch (in_msg.getMessageType())
  {
    case mik_msg_type::ROBOT_STATUS:
      success = handleStatus(in_msg, reply);
      break;
    case mik_msg_type::ROBOT_INFO:
      success = handleStatus(in_msg, reply);
      break;
    case mik_msg_type::ACTION_TRIG:
      success = handleActionTrigger(in_msg, reply);
      break;
    default:
      success = handleUnsupported(in_msg, reply);
      break;
  }
  return success;
}

bool SimpleMessageProcessor::handleStatus(SimpleMessage& in_msg, SimpleMessage& reply)
{
  MikStatusMessage status_msg;
  std::string error_msg;
  if (!status_msg.init(in_msg))
  {
    MikSimpleActionReply action_reply;
    error_msg = "Failed to decode MIK_STATUS message";
    printf("[Message Processor] %s\n", error_msg.c_str());
    createSimpleActionReplyError(action_reply, 0, 0, error_msg);
    return actionReplyToMsg(action_reply, reply);
  }

  printf("\n[MikadoActionManager] --- Mikado Status ---\n");
  status_msg.status_.print();

  // do whatever need to be done with status info (e.g. publish through ros topic)
  
  // return false, so we do not send a reply
  return false;
}

bool SimpleMessageProcessor::handleActionTrigger(SimpleMessage& in_msg, SimpleMessage& reply)
{
  MikActionTriggerMessage trigger_msg;
  MikSimpleActionReply action_reply;
  std::string error_msg;
  action_reply.init();
  if (!trigger_msg.init(in_msg))
  {
    error_msg = "Failed to decode ACTION_TRIG message";
    printf("[Message Processor] %s\n", error_msg.c_str());
    createSimpleActionReplyError(action_reply, 0, 0, error_msg);
    return actionReplyToMsg(action_reply, reply);
  }
  printf("\n[Message Processor] --- Mikado Action Trigger ---\n");
  trigger_msg.action_trigger_.print();
  const MikActionTrigger& trigger = trigger_msg.action_trigger_;

  // handle special case GET_CONNECTION_INFO
  if(trigger.getActionId() == mik_action_id::GET_CONNECTION_INFO){
    createConnectionInfoMsg(reply, getConnectionInfo());
    return true;
  }
  
  // do actual action handling
  if(!handleAction(trigger,action_reply)){
    createSimpleActionReplyError(trigger, action_reply);
  }

  return actionReplyToMsg(action_reply, reply);
}

bool SimpleMessageProcessor::handleAction(const MikActionTrigger& trigger, MikSimpleActionReply& action_reply){
  switch(trigger.getActionId()){
    case mik_action_id::CAPTURE_PC:{
      // a ros implementation could make a ros action call here and publish the result once ready
      printf("[Message Processor] Click click, an point cloud has been captured\n");
      createSimpleActionReplySuccess(trigger, action_reply);
      return true;
    }
    case mik_action_id::GET_CONNECTION_INFO:{
      // this is asked by the robot after a connection has been established
      printf("[Message Processor] Robot asks for connection info\n");
      createSimpleActionReplySuccess(trigger, action_reply);
      return true;
    }
    default:{
      std::string error_msg = "Cannot handle action with id " + std::to_string(trigger.getActionId()) + ". No handler implemented.";
      printf("[Message Processor] %s\n", error_msg.c_str());
      createSimpleActionReplyError(trigger, action_reply, error_msg);
      return false;
    }
  }
}


bool SimpleMessageProcessor::handleUnsupported(SimpleMessage& in_msg, SimpleMessage& reply)
{
  MikSimpleActionReply action_reply;
  std::string error_msg = "Received message of unknown type: " + std::to_string(in_msg.getMessageType());
  printf("[MikadoActionManager] %s\n", error_msg.c_str());
  createSimpleActionReplyError(action_reply, 0, 0, error_msg);
  return actionReplyToMsg(action_reply, reply);
}

}

