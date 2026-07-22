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

#define PORT 11000
#define BUFFER_SIZE 4096

int main(int argc, char **argv)
{
  int server_fd, new_socket;
  struct sockaddr_in address;
  int addrlen = sizeof(address);
  
  // Create socket
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
    perror("Socket creation failed");
    exit(EXIT_FAILURE);
  }
  
  // Allow port reuse
  int opt = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  
  // Bind
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(PORT);
  
  if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
  {
    perror("Bind failed");
    close(server_fd);
    exit(EXIT_FAILURE);
  }
  
  // Listen
  if (listen(server_fd, 3) < 0)
  {
    perror("Listen failed");
    close(server_fd);
    exit(EXIT_FAILURE);
  }
  
  printf("Mikado Test Server listening on port %d...\n", PORT);
  
  // Accept connection
  if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
  {
    perror("Accept failed");
    close(server_fd);
    exit(EXIT_FAILURE);
  }
  
  printf("Connection accepted from %s:%d\n", 
         inet_ntoa(address.sin_addr), ntohs(address.sin_port));
  
  // Message handling loop
  while (1)
  {
    // Read message length (first 4 bytes)
    char length_buf[4];
    int bytes_read = read(new_socket, length_buf, 4);
    if (bytes_read <= 0)
    {
      if (errno == ECONNRESET)
      {
        printf("Client disconnected\n");
        break;
      }
      perror("Read length failed");
      break;
    }
    
    if (bytes_read != 4)
    {
      printf("Failed to read message length\n");
      break;
    }
    
    int msg_length = *(int *)length_buf;
    printf("Received message length: %d bytes\n", msg_length);
    
    // Read the complete message
    ByteArray msgBuffer;
    if (!msgBuffer.init(length_buf, 4))
    {
      printf("Failed to init length buffer\n");
      break;
    }
    
    // Read data bytes
    char data_buf[BUFFER_SIZE];
    int total_data_read = 0;
    int data_to_read = msg_length - 4;
    
    while (total_data_read < data_to_read)
    {
      int bytes = read(new_socket, data_buf + total_data_read, data_to_read - total_data_read);
      if (bytes <= 0)
      {
        perror("Read data failed");
        break;
      }
      total_data_read += bytes;
    }
    
    if (total_data_read < data_to_read)
    {
      printf("Failed to read complete message data\n");
      break;
    }
    
    // Load data into buffer
    msgBuffer.load(data_buf, data_to_read);
    
    // Parse message
    SimpleMessage msg;
    if (!msg.init(msgBuffer))
    {
      printf("Failed to parse message\n");
      continue;
    }
    
    printf("\n--- Received Message ---\n");
    printf("Message Type: %d\n", msg.getMessageType());
    printf("Comm Type: %d\n", msg.getCommType());
    printf("Reply Code: %d\n", msg.getReplyCode());
    printf("Data Length: %d bytes\n", msg.getDataLength());
    
    // Check if it's a Mikado Status message
    if (msg.getMessageType() == mik_msg_type::MIK_STATUS)
    {
      MikStatusMessage status_msg;
      if (status_msg.init(msg))
      {
        MikStatus &status = status_msg.status_;
        
        printf("\n--- Mikado Status ---\n");
        printf("  Mik State:      %d\n", status.getMikState());
        printf("  Camera State:   %d\n", status.getCameraState());
        printf("  Comm State:     %d\n", status.getCommState());
        printf("  Robot State:    %d\n", status.getRobotState());
        printf("  Running State:  %d\n", status.getRunningState());
        
        // Send reply with same status
        MikStatus reply_status;
        reply_status.init(status.getMikState(), status.getCameraState(),
                         status.getCommState(), status.getRobotState(),
                         status.getRunningState());
        
        MikStatusMessage reply_msg;
        reply_msg.init(reply_status);
        
        SimpleMessage reply;
        reply_msg.toReply(reply, ReplyTypes::SUCCESS);
        
        ByteArray reply_data;
        reply.toByteArray(reply_data);
        
        // Send reply
        int reply_len = reply.getMsgLength();
        std::vector<char> reply_vec;
        reply_data.copyTo(reply_vec);
        
        // Send length + data
        write(new_socket, &reply_len, 4);
        write(new_socket, reply_vec.data(), reply_vec.size());
        
        printf("\n--- Sent Reply ---\n");
        printf("  Mik State:      %d\n", status.getMikState());
        printf("  Camera State:   %d\n", status.getCameraState());
        printf("  Comm State:     %d\n", status.getCommState());
        printf("  Robot State:    %d\n", status.getRobotState());
        printf("  Running State:  %d\n", status.getRunningState());
      }
    }
    
    // Small delay before next message
    usleep(100000); // 100ms
  }
  
  close(new_socket);
  close(server_fd);
  
  return 0;
}