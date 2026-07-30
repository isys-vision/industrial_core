#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "simple_message/simple_message.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/mikado_messages/mik_status_message.h"
#include "simple_message/mikado_classes/mik_status.h"
#include "simple_message/mikado_messages/mik_action_trigger_message.h"
#include "simple_message/mikado_classes/mik_action_trigger.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/smpl_msg_connection.h"
#include "industrial_robot_client/message_generator.h"

using namespace industrial::simple_message;
using namespace industrial::tcp_client;
using namespace industrial::smpl_msg_connection;
using namespace industrial::mik_status_message;
using namespace industrial::mik_status;
using namespace industrial::mik_action_trigger_message;
using namespace industrial::mik_action_trigger;
using namespace industrial_robot_client;


#define SERVER_PORT 11000


const int MSG_TYPE_TO_SEND = mik_msg_type::ACTION_TRIG; 


SimpleMessage getMessage(){
  SimpleMessage simple_msg;
  switch(MSG_TYPE_TO_SEND){
    case mik_msg_type::ACTION_TRIG:
    {
      industrial_robot_client::createActionTriggerMessage(simple_msg);
      break;
    }
    case mik_msg_type::MIK_STATUS:
    {
      industrial_robot_client::createMikStatusMessage(simple_msg);
      break;
    }
    default:
    {
      printf("[CLIENT] Unkown message type, aborting.");
      exit(EXIT_FAILURE);
    }
  }
  return simple_msg;
}

void printMsg(SimpleMessage& simpleMsg){
  printf("\n--------------\n[CLIENT] Printing msg:\n");
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
    default:
    {
      printf("[CLIENT] Unknown message type, cannot print.");
    }
  }
  printf("\n--------------\n");
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("[CLIENT] Usage: %s <server_ip>\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  TcpClient tcp_client;
  if (!tcp_client.init(argv[1], SERVER_PORT))
  {
    printf("[CLIENT] Failed to initialize TCP client\n");
    exit(EXIT_FAILURE);
  }

  printf("[CLIENT] Connecting to server %s:%d...\n", argv[1], SERVER_PORT);
  if (!tcp_client.makeConnect())
  {
    printf("[CLIENT] Failed to connect to server\n");
    exit(EXIT_FAILURE);
  }

  printf("[CLIENT] Connected to server!\n\n");

  int counter = 0;
  SimpleMessage simple_msg;
  simple_msg = getMessage();
  while (true)
  {
    printf("[CLIENT] --- Sending Message #%d ---\n", counter + 1);
    printf("Message Type: %d\n", simple_msg.getMessageType());
    printf("Comm Type: %d\n", simple_msg.getCommType());
    printf("Data Length: %d bytes\n", simple_msg.getDataLength());

    SimpleMessage reply_msg;

    if (tcp_client.sendAndReceiveMsg(simple_msg, reply_msg, false))
    {
      printf("\n[CLIENT] --- Received Reply ---\n");
      printf("Reply Message Type: %d\n", reply_msg.getMessageType());
      printf("Reply Comm Type: %d\n", reply_msg.getCommType());
      printf("Reply Reply Code: %d\n", reply_msg.getReplyCode());
      printMsg(reply_msg);
    }
    else
    {
      printf("[CLIENT] Failed to send/receive message\n");
    }

    counter++;
    sleep(2);
  }
  return 0;
}