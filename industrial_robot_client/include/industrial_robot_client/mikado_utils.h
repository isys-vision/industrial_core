#ifndef MIKADO_ACTION_MANAGER_H
#define MIKADO_ACTION_MANAGER_H

#include "simple_message/simple_message.h"
#include "simple_message/mikado_messages/mik_status_message.h"
#include "simple_message/mikado_classes/mik_status.h"
#include "simple_message/mikado_messages/mik_action_trigger_message.h"
#include "simple_message/mikado_classes/mik_action_trigger.h"
#include "simple_message/mikado_messages/mik_simple_action_reply_message.h"
#include "simple_message/mikado_classes/mik_simple_action_reply.h"
#include "simple_message/mikado_messages/mikado_types.h"


using namespace industrial::simple_message;
using namespace industrial::mik_simple_action_reply;
using namespace industrial::mik_simple_action_reply_message;
using namespace industrial::mik_action_trigger;
using namespace mik_connection_info;


namespace industrial_robot_client
{
namespace mikado_utils
{
  bool actionReplyToMsg(MikSimpleActionReply& action_reply, SimpleMessage& action_reply_msg);
  void createSimpleActionReplySuccess (const MikActionTrigger& trigger, MikSimpleActionReply& action_reply);
  void createSimpleActionReplyError (const MikActionTrigger& trigger, MikSimpleActionReply& action_reply, std::string error_msg = "An error occured");
  void createSimpleActionReplyError (MikSimpleActionReply& action_reply, int action_id, int request_id, std::string error_msg = "An error occured");
  void createConnectionInfoMsg(SimpleMessage& reply, ConnectionInfo connection_params);
}
}

#endif
