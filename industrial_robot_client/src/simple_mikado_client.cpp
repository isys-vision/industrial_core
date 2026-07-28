#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "simple_message/simple_message.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/mikado_messages/mik_status_message.h"
#include "simple_message/mikado_classes/mik_status.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/smpl_msg_connection.h"

using namespace industrial::simple_message;
using namespace industrial::mik_status_message;
using namespace industrial::mik_status;
using namespace industrial::tcp_client;
using namespace industrial::smpl_msg_connection;

#define SERVER_PORT 11000

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
  while (true)
  {
    // Create MikStatus message
    MikStatus status;
    status.init(industrial::mik_status::MikStates::MS_FALSE,
                industrial::mik_status::MikStates::CS_TRUE,
                industrial::mik_status::MikStates::CoS_UNKNOWN,
                industrial::mik_status::MikStates::RS_TRUE,
                industrial::mik_status::MikStates::RuS_TRUE);

    MikStatusMessage msg;
    msg.init(status);

    SimpleMessage simple_msg;
    msg.toTopic(simple_msg);

    printf("[CLIENT] --- Sending Message #%d ---\n", counter + 1);
    printf("Message Type: %d\n", simple_msg.getMessageType());
    printf("Comm Type: %d\n", simple_msg.getCommType());
    printf("Data Length: %d bytes\n", simple_msg.getDataLength());

    printf("\n[CLIENT] --- Mikado Status ---\n");
    printf("  Mik State:      %d\n", msg.status_.getMikState());
    printf("  Camera State:   %d\n", msg.status_.getCameraState());
    printf("  Comm State:     %d\n", msg.status_.getCommState());
    printf("  Robot State:    %d\n", msg.status_.getRobotState());
    printf("  Running State:  %d\n", msg.status_.getRunningState());

    SimpleMessage reply_msg;

    if (tcp_client.sendAndReceiveMsg(simple_msg, reply_msg, false))
    {
      printf("\n[CLIENT] --- Received Reply ---\n");
      printf("Reply Message Type: %d\n", reply_msg.getMessageType());
      printf("Reply Comm Type: %d\n", reply_msg.getCommType());
      printf("Reply Reply Code: %d\n", reply_msg.getReplyCode());

      if (reply_msg.getMessageType() == mik_msg_type::MIK_STATUS)
      {
        MikStatusMessage reply_status_msg;
        if (reply_status_msg.init(reply_msg))
        {
          printf("\n[CLIENT] --- Received Reply Status ---\n");
          printf("  Mik State:      %d\n", reply_status_msg.status_.getMikState());
          printf("  Camera State:   %d\n", reply_status_msg.status_.getCameraState());
          printf("  Comm State:     %d\n", reply_status_msg.status_.getCommState());
          printf("  Robot State:    %d\n", reply_status_msg.status_.getRobotState());
          printf("  Running State:  %d\n", reply_status_msg.status_.getRunningState());
        }
      }
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