#include "industrial_robot_client/mikado_utils.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/mikado_classes/mik_connection_info.h"
#include "simple_message/mikado_messages/mik_connection_info_message.h"

using namespace industrial::simple_message;
using namespace industrial::mik_simple_action_reply;
using namespace industrial::mik_simple_action_reply_message;
using namespace industrial::mik_action_trigger;
using namespace industrial::mik_action_trigger_message;
using namespace industrial::mik_conn_info;
using namespace industrial::mik_conn_info_message;
using namespace mik_connection_info;



namespace industrial_robot_client
{
  namespace mikado_utils{

    bool actionReplyToMsg(MikSimpleActionReply& action_reply, SimpleMessage& action_reply_msg){
      MikSimpleActionReplyMessage msg;
      msg.init(action_reply);
      return msg.toTopic(action_reply_msg);
    }

    void createSimpleActionReplySuccess (const MikActionTrigger& trigger, MikSimpleActionReply& action_reply){
      action_reply.init();
      action_reply.setActionId(trigger.getActionId());
      action_reply.setRequestId(trigger.getRequestId());
      action_reply.setActionStatus(mik_action_status::success::OK);
    }

    void createSimpleActionReplyError (const MikActionTrigger& trigger, MikSimpleActionReply& action_reply, std::string error_msg){
      action_reply.init();
      action_reply.setActionId(trigger.getActionId());
      action_reply.setRequestId(trigger.getRequestId());
      action_reply.setActionStatus(mik_action_status::error::GENERAL_ERROR);
      action_reply.setErrorMsg(error_msg);
    }

    void createSimpleActionReplyError (MikSimpleActionReply& action_reply, int action_id, int request_id, std::string error_msg){
      action_reply.init();
      action_reply.setActionId(action_id);
      action_reply.setRequestId(request_id);
      action_reply.setActionStatus(mik_action_status::error::GENERAL_ERROR);
      action_reply.setErrorMsg(error_msg);
    }

    void createConnectionInfoMsg(SimpleMessage& reply, ConnectionInfo connection_params){
      MikadoConnectionInfo conn_info;
      conn_info.init(connection_params.mikado_product, connection_params.rotation_convention, 
        connection_params.number_of_axis_traj_pts, connection_params.number_of_axis_status, 
        connection_params.number_of_ext_axis_traj_pts, connection_params.number_of_ext_axis_status, 
        connection_params.traj_pt_is_radian, connection_params.status_is_radian, 
        connection_params.is_status_is_moving);
      MikadoConnectionInfoMessage conn_info_msg;
      conn_info_msg.init(conn_info);
      conn_info_msg.toTopic(reply);
    }

  }
}