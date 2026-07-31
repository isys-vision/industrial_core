#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "simple_message/simple_message.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/mikado_messages/mik_status_message.h"
#include "simple_message/mikado_classes/mik_status.h"
#include "simple_message/socket/tcp_server.h"
#include "simple_message/smpl_msg_connection.h"
#include "industrial_robot_client/message_decoder.h"

using namespace industrial::simple_message;
using namespace industrial::mik_status_message;
using namespace industrial::mik_status;
using namespace industrial::tcp_server;
using namespace industrial::smpl_msg_connection;

#define PORT 11000

int main(int argc, char **argv)
{
  // Create TCP server
  TcpServer tcp_server;

  // Initialize server on port
  printf("[SERVER] Mikado Test Server listening on port %d...\n", PORT);
  if (!tcp_server.init(PORT))
  {
    printf("[SERVER] Failed to initialize TCP server\n");
    exit(EXIT_FAILURE);
  }

  // Wait for client connection
  printf("[SERVER] Waiting for client connection...\n");
  if (!tcp_server.makeConnect())
  {
    printf("[SERVER] Failed to accept client connection\n");
    exit(EXIT_FAILURE);
  }

  printf("[SERVER] Connection accepted!\n\n");

  // Message handling loop
  while (1)
  {
    SimpleMessage msg;

    // Receive message
    if (!tcp_server.receiveMsg(msg))
    {
      printf("[SERVER] Failed to receive message or connection lost\n");
      break;
    }

    printf("\n[SERVER] --- Received Message ---\n");
    printf("Message Type: %d\n", msg.getMessageType());
    printf("Comm Type: %d\n", msg.getCommType());
    printf("Reply Code: %d\n", msg.getReplyCode());
    printf("Data Length: %d bytes\n", msg.getDataLength());

    SimpleMessage reply = industrial_robot_client::decodeAndRepackMessage(msg);

    if (tcp_server.sendMsg(reply)){
      printf("\n[SERVER] --- Sent Reply ---\n");
      printf("Message Type: %d\n", reply.getMessageType());
      printf("Comm Type: %d\n", reply.getCommType());
      printf("Reply Code: %d\n", reply.getReplyCode());
      printf("Data Length: %d bytes\n", reply.getDataLength());
    }
    else {
      printf("[SERVER] Failed to send reply\n");
    }

    // Small delay before next message
    usleep(100000); // 100ms
  }

  // Cleanup (connection will be closed by TcpServer destructor)
  return 0;
}