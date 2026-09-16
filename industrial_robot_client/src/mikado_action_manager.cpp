#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <queue>
#include <mutex>

#include "simple_message/simple_message.h"
#include "simple_message/mikado_messages/mikado_types.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/smpl_msg_connection.h"
#include "industrial_robot_client/message_generator.h"
#include "industrial_robot_client/message_decoder.h"
#include "industrial_robot_client/mikado_action_manager.h"

using namespace industrial::simple_message;
using namespace industrial::tcp_client;
using namespace industrial::smpl_msg_connection;
using namespace industrial_robot_client;
using namespace industrial_robot_client::mikado_action_manager;


// this class handles receiving all messages and replying to them. When receiving a message, header informations are extracted and then put into a buffer. Another thread goes into this buffer, parses the message and handles the action to take. Replies are written into another buffer which the main thread reads. If there is a reply in the buffer, the main thread takes it and sends it to the robot

namespace industrial_robot_client
{

MikadoActionManager::MikadoActionManager() 
: message_processor_(nullptr),
action_port_(0),
robot_ip_(""),
is_first_cycle_(true),
time_since_last_msg_(0.0),
keepalive_timeout_(5.0)
{
}

  MikadoActionManager::~MikadoActionManager() {}

void MikadoActionManager::init(const std::string& robot_ip, int action_port, IMessageProcessor& processor){
  robot_ip_ = robot_ip;
  action_port_ = action_port;
   message_processor_ = &processor;
  if (!tcp_client_.init(&robot_ip_[0], action_port_))
  {
    printf("[Action Manager] Failed to initialize TCP client\n");
    exit(EXIT_FAILURE);
  }
}

void MikadoActionManager::connect(){
  // blocks and retries connecting for ever
  if (!tcp_client_.init(&robot_ip_[0], action_port_))
  {
    printf("[Action Manager] Failed to initialize TCP client\n");
    exit(EXIT_FAILURE);
  }

  printf("[Action Manager] Connecting to server %s:%d...\n", robot_ip_.c_str(), action_port_);
  while (!tcp_client_.makeConnect())
  {
    printf("[Action Manager] Failed to connect to server\n");
    sleep(5);
  }
  time_since_last_msg_ = 0.0;
}

void MikadoActionManager::push_request(SimpleMessage& request_msg){
  std::lock_guard<std::mutex> lock(request_buffer_mutex_);
  request_buffer_.push(std::move(request_msg));
}

void MikadoActionManager::push_reply(SimpleMessage& reply_msg){
  std::lock_guard<std::mutex> lock(reply_buffer_mutex_);
  reply_buffer_.push(std::move(reply_msg));
}

bool MikadoActionManager::try_pop_request(SimpleMessage& request_msg){
  std::lock_guard<std::mutex> lock(request_buffer_mutex_);
  if(request_buffer_.empty()){
    return false;
  }
  printf("[RequestBuffer] Currently %d requests in buffer\n", request_buffer_.size());
  request_msg = std::move(request_buffer_.front());
  request_buffer_.pop();
  return true;
}

bool MikadoActionManager::try_pop_reply(SimpleMessage& reply_msg){
  std::lock_guard<std::mutex> lock(reply_buffer_mutex_);
  if(reply_buffer_.empty()){
    return false;
  }
  printf("[ReplyBuffer] Currently %d replies in buffer\n", reply_buffer_.size());
  reply_msg = std::move(reply_buffer_.front());
  reply_buffer_.pop();
  return true;
}

void MikadoActionManager::process_requests()
{
    SimpleMessage request;
    SimpleMessage reply;
    std::vector<SimpleMessage> add_replies;
    while(processor_running_)
    {
      if(try_pop_request(request)) {
        add_replies.clear();
        if(this->message_processor_->process(request, reply, add_replies)) {
          push_reply(reply);
          if(add_replies.size() > 0){
            for(SimpleMessage rep : add_replies){
              push_reply(rep);
            }
          }
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
}

void MikadoActionManager::cycle(){
  SimpleMessage request_msg;

  if(time_since_last_msg_ >= keepalive_timeout_){
    printf("[Action Manager] Did not receive a message for at least %f seconds. Trying to reconnect.\n", keepalive_timeout_);
    tcp_client_.setDisconnected();
    connect();
  }

  const auto cycle_start = std::chrono::steady_clock::now();
  bool message_sent = false;
  bool message_received = false;

  if(is_first_cycle_){
      is_first_cycle_ = false;
      processor_running_ = true;
      processor_thread_ = std::thread(&MikadoActionManager::process_requests, this);
  }
  while(try_pop_reply(reply_msg_)){
    if (tcp_client_.sendMsg(reply_msg_)){
      message_sent = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } else {
      printf("[Action Manager] Failed to send reply message\n");
      if(!tcp_client_.isConnected()){
        connect();
      }
    }
  }
   if (tcp_client_.receiveMsgWithTimeout(request_msg, 0.01, false)){
      printf("\n[Action Manager] Received Request. Msg Type: %d\n", request_msg.getMessageType());
      message_received = true;
      push_request(request_msg);
      // TODO: IF NOT TOPIC COMM TYPE -> ACKNOWLEDGE RECEIVING REQUEST 
    } else {
      //this->connect();
    }

  const double cycle_duration = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - cycle_start).count();
  if (message_sent || message_received){
    time_since_last_msg_ = 0.0;
  } else {
    time_since_last_msg_ += cycle_duration;
  }
}
};