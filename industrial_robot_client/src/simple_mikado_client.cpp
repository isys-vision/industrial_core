#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "simple_message/simple_message.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/smpl_msg_connection.h"
#include "industrial_robot_client/message_generator.h"
#include "industrial_robot_client/message_decoder.h"

using namespace industrial::simple_message;
using namespace industrial::tcp_client;
using namespace industrial::smpl_msg_connection;
using namespace industrial::mik_status_message;
using namespace industrial::mik_status;
using namespace industrial::mik_action_trigger_message;
using namespace industrial::mik_action_trigger;
using namespace industrial::mik_simple_action_reply_message;
using namespace industrial::mik_simple_action_reply;
using namespace industrial_robot_client;


//#define SERVER_PORT 11000
#define SERVER_PORT 54600
//const int MSG_TYPE_TO_SEND = mik_msg_type::TRAJ_PT; 
//const int MSG_TYPE_TO_SEND = mik_msg_type::CONN_INFO; 
const int MSG_TYPE_TO_SEND = mik_msg_type::SIMPLE_REPLY; 
//const int MSG_TYPE_TO_SEND = mik_msg_type::ACTION_TRIG; 


SimpleMessage getMessage(){
  SimpleMessage simple_msg;
  if(!createMikMessage(simple_msg, MSG_TYPE_TO_SEND)){
    printf("[CLIENT] Unkown message type, aborting.");
      exit(EXIT_FAILURE);
  }
  return simple_msg;
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
  while (!tcp_client.makeConnect())
  {
    printf("[CLIENT] Failed to connect to server\n");
    sleep(5);
  }

  printf("[CLIENT] Connected to server!\n\n");

  int counter = 0;
  SimpleMessage reply_msg;
  reply_msg = getMessage();
  while (true)
  {
    /*printf("[CLIENT] --- Sending Message #%d ---\n", counter + 1);
    printf("Message Type: %d\n", simple_msg.getMessageType());
    printf("Comm Type: %d\n", simple_msg.getCommType());
    printf("Data Length: %d bytes\n", simple_msg.getDataLength());
    */
    SimpleMessage recvieved_msg;

    //if (tcp_client.sendAndReceiveMsg(simple_msg, reply_msg, false))
    if (tcp_client.receiveMsg(recvieved_msg)) // (simple_msg, reply_msg, false))
    {
      printf("\n[CLIENT] --- Received Reply ---\n");
      printf("Reply Message Type: %d\n", recvieved_msg.getMessageType());
      printf("Reply Comm Type: %d\n", recvieved_msg.getCommType());
      printf("Reply Reply Code: %d\n", recvieved_msg.getReplyCode());
      printMsg(recvieved_msg);
      reply_msg = industrial_robot_client::decodeAndReplyToActionTriggerMessage(recvieved_msg);
      if (tcp_client.sendMsg(reply_msg)){
        printf("Sent reply");
      }
    }
    else
    {
      printf("[CLIENT] Failed to send/receive message\n");
      if (!tcp_client.init(argv[1], SERVER_PORT))
      {
        printf("[CLIENT] Failed to initialize TCP client\n");
        exit(EXIT_FAILURE);
      }

      printf("[CLIENT] Connecting to server %s:%d...\n", argv[1], SERVER_PORT);
      while (!tcp_client.makeConnect())
      {
        printf("[CLIENT] Failed to connect to server\n");
        sleep(5);
      }
    }

    counter++;
    sleep(2);
  }
  return 0;
}