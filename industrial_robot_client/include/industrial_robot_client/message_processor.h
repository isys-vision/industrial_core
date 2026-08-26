#ifndef MESSAGE_PROCESSOR_H
#define MESSAGE_PROCESSOR_H

#include <string>

#include "simple_message/simple_message.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/typed_message.h"
#include "simple_message/mikado_classes/mik_action_trigger.h"
#include "simple_message/mikado_classes/mik_simple_action_reply.h"

using namespace industrial::mik_action_trigger;
using namespace industrial::mik_simple_action_reply;
using namespace industrial::simple_message;
using namespace mik_connection_info;

namespace industrial_robot_client
{

class IMessageProcessor
{
public:
  virtual ~IMessageProcessor() {}

  /**
   * \brief called once by the manager before the message loop starts
   *
   * \return true on success, false otherwise
   */
  virtual bool init(ConnectionInfo connection_params)
  {
    connection_params_ = connection_params;
    return true;
  }

  /**
   * \brief called once by the manager after the message loop stops
   */
  virtual void shutdown()
  {
  }

  /**
   * \brief processes a single received message and produces the reply
   *
   * \param in_msg the received message
   * \param out_msg the reply message
   *
   * \return true on success, false otherwise
   */
  virtual bool process(SimpleMessage& in_msg, SimpleMessage& out_msg) = 0;

  virtual ConnectionInfo getConnectionInfo(){
    return connection_params_;
  }

  virtual void setConnectionInfo(ConnectionInfo connection_params){
    connection_params_ = connection_params;
  }

protected:
  ConnectionInfo connection_params_;
};

class SimpleMessageProcessor : public IMessageProcessor
{
public:
  SimpleMessageProcessor(ConnectionInfo connection_params) {connection_params_ = connection_params;}
  virtual ~SimpleMessageProcessor() {}

/**
   * \brief processes a single received message and produces the reply
   *
   * \param in_msg the received message
   * \param out_msg the reply message
   *
   * \return true on success, false otherwise
   */
  virtual bool process(SimpleMessage& in_msg, SimpleMessage& reply);

private:
  bool handleStatus(SimpleMessage& in_msg, SimpleMessage& reply);
  bool handleActionTrigger(SimpleMessage& in_msg, SimpleMessage& reply);
  bool handleAction(const MikActionTrigger& trigger, MikSimpleActionReply& action_reply);
  bool handleSimpleActionReply(SimpleMessage& in_msg, SimpleMessage& reply);
  bool handleUnsupported(SimpleMessage& in_msg, SimpleMessage& reply);

};
}

#endif

