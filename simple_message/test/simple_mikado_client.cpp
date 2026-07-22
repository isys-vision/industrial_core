/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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
 * 	* Neither the name of the Southwest Research Institute, nor the names
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/select.h>
#include <time.h>

// Simple message protocol includes
#include "simple_message/simple_message.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/mikado_messages/mik_status_message.h"
#include "simple_message/mikado_classes/mik_status.h"
#include "simple_message/byte_array.h"

using namespace industrial::simple_message;
using namespace industrial::mik_status_message;
using namespace industrial::mik_status;
using namespace industrial::byte_array;
using namespace industrial::shared_types;

#define SERVER_PORT 11000
#define BUFFER_SIZE 4096

int main(int argc, char **argv)
{
  int sock;
  struct sockaddr_in server_addr;
  char buffer[BUFFER_SIZE];
  
  // Check for server IP argument
  if (argc < 2)
  {
    printf("Usage: %s <server_ip>\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  
  // Create socket
  if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    perror("Socket creation failed");
    exit(EXIT_FAILURE);
  }
  
  // Set up server address
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(SERVER_PORT);
  
  if (inet_pton(AF_INET, argv[1], &server_addr.sin_addr) <= 0)
  {
    perror("Invalid server address");
    close(sock);
    exit(EXIT_FAILURE);
  }
  
  // Connect to server
  printf("Connecting to server %s:%d...\n", argv[1], SERVER_PORT);
  if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
  {
    perror("Connection failed");
    close(sock);
    exit(EXIT_FAILURE);
  }
  
  printf("Connected to server!\n\n");
  
  // Send messages every 2 seconds
  int counter = 0;
  while (1)
  {
    // Create MikStatus message
    MikStatus status;
    status.init(industrial::mik_status::MikStates::MS_TRUE,
                industrial::mik_status::MikStates::CS_TRUE,
                industrial::mik_status::MikStates::CoS_TRUE,
                industrial::mik_status::MikStates::RS_TRUE,
                industrial::mik_status::MikStates::RuS_TRUE);
    
    MikStatusMessage msg;
    msg.init(status);
    
    SimpleMessage simple_msg;
    msg.toTopic(simple_msg);
    
    // Convert to byte array
    ByteArray data;
    simple_msg.toByteArray(data);
    
    int msg_len = simple_msg.getMsgLength();
    
    printf("--- Sending Message #%d ---\n", counter + 1);
    printf("Message Type: %d\n", simple_msg.getMessageType());
    printf("Comm Type: %d\n", simple_msg.getCommType());
    printf("Data Length: %d bytes\n", msg_len);
    
    MikStatus &status_ref = msg.status_;
    printf("\n--- Mikado Status ---\n");
    printf("  Mik State:      %d\n", status_ref.getMikState());
    printf("  Camera State:   %d\n", status_ref.getCameraState());
    printf("  Comm State:     %d\n", status_ref.getCommState());
    printf("  Robot State:    %d\n", status_ref.getRobotState());
    printf("  Running State:  %d\n", status_ref.getRunningState());
    
    // Send message
    char *send_buf = new char[msg_len];
    memcpy(send_buf, &msg_len, 4);
    data.copyTo((std::vector<char> &)send_buf);
    
    write(sock, send_buf, msg_len);
    delete[] send_buf;
    
    // Wait for reply
    fd_set read_fds;
    struct timeval timeout;
    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);
    
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    
    int sel = select(sock + 1, &read_fds, NULL, NULL, &timeout);
    if (sel > 0 && FD_ISSET(sock, &read_fds))
    {
      // Read reply length
      int bytes_read = read(sock, buffer, 4);
      if (bytes_read == 4)
      {
        int reply_len = *(int *)buffer;
        printf("\n--- Received Reply (length: %d) ---\n", reply_len);
        
        // Read reply data
        if (reply_len > 4 && reply_len < BUFFER_SIZE)
        {
          int data_read = 0;
          while (data_read < reply_len - 4)
          {
            int bytes = read(sock, buffer + 4 + data_read, reply_len - 4 - data_read);
            if (bytes <= 0) break;
            data_read += bytes;
          }
          
          // Parse reply
          ByteArray reply_data;
          reply_data.init(buffer, reply_len);
          
          SimpleMessage reply_msg;
          if (reply_msg.init(reply_data))
          {
            printf("Reply Message Type: %d\n", reply_msg.getMessageType());
            printf("Reply Comm Type: %d\n", reply_msg.getCommType());
            printf("Reply Reply Code: %d\n", reply_msg.getReplyCode());
            
            if (reply_msg.getMessageType() == mik_msg_type::MIK_STATUS)
            {
              MikStatusMessage reply_status_msg;
              if (reply_status_msg.init(reply_msg))
              {
                MikStatus &reply_status = reply_status_msg.status_;
                printf("\n--- Received Reply Status ---\n");
                printf("  Mik State:      %d\n", reply_status.getMikState());
                printf("  Camera State:   %d\n", reply_status.getCameraState());
                printf("  Comm State:     %d\n", reply_status.getCommState());
                printf("  Robot State:    %d\n", reply_status.getRobotState());
                printf("  Running State:  %d\n", reply_status.getRunningState());
              }
            }
          }
        }
      }
    }
    else
    {
      printf("Timeout waiting for reply\n");
    }
    
    counter++;
    sleep(2);
  }
  
  close(sock);
  return 0;
}