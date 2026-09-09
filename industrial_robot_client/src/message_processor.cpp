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

bool SimpleMessageProcessor::process(SimpleMessage& in_msg, SimpleMessage& reply, std::vector<SimpleMessage>& add_replies)
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
      success = handleStatus(in_msg, reply, add_replies);
      break;
    case mik_msg_type::ROBOT_INFO:
      success = handleStatus(in_msg, reply, add_replies);
      break;
    case mik_msg_type::ACTION_TRIG:
      success = handleActionTrigger(in_msg, reply, add_replies);
      break;
    default:
      success = handleUnsupported(in_msg, reply);
      break;
  }
  return success;
}

bool SimpleMessageProcessor::handleStatus(SimpleMessage& in_msg, SimpleMessage& reply, std::vector<SimpleMessage>& add_replies)
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

bool SimpleMessageProcessor::handleActionTrigger(SimpleMessage& in_msg, SimpleMessage& reply, std::vector<SimpleMessage>& add_replies)
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
  if(!handleAction(trigger,action_reply, add_replies)){
    createSimpleActionReplyError(trigger, action_reply);
  }

  return actionReplyToMsg(action_reply, reply);
}

bool SimpleMessageProcessor::handleAction(const MikActionTrigger& trigger, MikSimpleActionReply& action_reply, std::vector<SimpleMessage>& add_replies){
  switch(trigger.getActionId()){
    case mik_action_id::CAPTURE_PC:{
      // a ros implementation could make a ros action call here and publish the result once ready
      printf("[Message Processor] A point cloud has been captured\n");
      createSimpleActionReplySuccess(trigger, action_reply);
      return true;
    }
    case mik_action_id::GET_CONNECTION_INFO:{
      // this is asked by the robot after a connection has been established
      printf("[Message Processor] Robot asks for connection info\n");
      createSimpleActionReplySuccess(trigger, action_reply);
      return true;
    }
    case mik_action_id::FIND_PRODUCT:{
      printf("[Message Processor] Product search triggered by robot. Only confirming this to the robot, robot will ask for pose seperately.\n");
      createSimpleActionReplySuccess(trigger, action_reply);
      return true;
    }
    case mik_action_id::LOAD_RECIPE:{
      printf("[Message Processor] Loading recipe #%d\n", trigger.getAdditionalIntArgs()[0]);
      createSimpleActionReplySuccess(trigger, action_reply);
      return true;
    }
    case mik_action_id::FIND_CONTAINER:{
      printf("[Message Processor] Finding container and returning dummy pose\n");
      createSimpleActionReplySuccess(trigger, action_reply);
      std::vector<shared_real> pose = {123.3, 345.6, 789.0, 12.4, -41.7, 115.8};
      action_reply.setAdditionalRealArgs(pose);
      return true;
    }
    case mik_action_id::FIND_PICKS:{
      printf("[Message Processor] Starting pick search");
      createSimpleActionReplySuccess(trigger, action_reply);
      return true;
    }
    case mik_action_id::GET_NUM_DETECTS:{
      std::vector<int> detects = {37};
      printf("[Message Processor] Number of detected products: %d", detects[0]);
      createSimpleActionReplySuccess(trigger, action_reply);
      action_reply.setAdditionalIntArgs(detects);
      return true;
    }
    case mik_action_id::GET_NUM_PICKABLE:{
      std::vector<int> pickable = {9};
      printf("[Message Processor] Number of pickable products: %d", pickable[0]);
      createSimpleActionReplySuccess(trigger, action_reply);
      action_reply.setAdditionalIntArgs(pickable);
      return true;
    }
    case mik_action_id::GET_PROD_POSE:{
      printf("[Message Processor] Returning prod pose of prod with index %d\n", trigger.getAdditionalIntArgs()[0]);
      createSimpleActionReplySuccess(trigger, action_reply);
      std::vector<shared_real> pose = {2942.8, 419.6, -789.0, 0.3, -41.7, 115.8};
      action_reply.setAdditionalRealArgs(pose);
      return true;
    }
    case mik_action_id::GET_PICK_TRAJ:{
      printf("[Message Processor] Sending Pick Trajectory");
      int traj_length = 12;
      int traj_id = 1;
      int traj_type = mik_traj_type::APPROACH;
      MikadoDynamicJointsTrajPt traj_pt;
      MikadoDynamicJointsTrajPtMessage traj_pt_msg;
      SimpleMessage msg;
      // first message confirms that mikado received action
      // later we should return then and send traj info and traj pts later. For now this all happens at once.
      createSimpleActionReplySuccess(trigger, action_reply);

      // use action reply for traj info
      MikSimpleActionReply trajInfoReply;
      trajInfoReply.copyFrom(action_reply);
      std::vector<int> traj_info = {traj_id, traj_type, traj_length};
      trajInfoReply.setAdditionalIntArgs(traj_info);
      MikSimpleActionReplyMessage trajInfoReplyMsg;
      trajInfoReplyMsg.init(trajInfoReply);
      SimpleMessage trajInfoSimpleMsg;
      trajInfoReplyMsg.toTopic(trajInfoSimpleMsg);
      add_replies.push_back(trajInfoSimpleMsg);

      for(int i = 0; i < traj_length; i++){
        traj_pt.init();
        traj_pt.setTrajectoryId(traj_id);
        traj_pt.setSequence(i+1);
        traj_pt.setMotionType(mik_motion_type::JOINT);
        traj_pt.setTrajectoryPart(traj_type);
        traj_pt.setVelocity(100.0);
        std::vector<float> positions = {37.3, 321.2, -11.9, -286.0, 0.0, 14.6};
        traj_pt.setPositions(positions);
        if(i == traj_length-1){
          traj_pt.setIsLastTrajPt(true);
        }
        traj_pt_msg.init(traj_pt);
        traj_pt_msg.toTopic(msg);
        add_replies.push_back(msg);
      }

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

