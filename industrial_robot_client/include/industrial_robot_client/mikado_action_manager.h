#ifndef MIKADO_ACTION_MANAGER_H
#define MIKADO_ACTION_MANAGER_H

#include <queue>
#include <mutex>
#include <memory>
#include <thread>
#include <atomic>
#include <string>

#include "simple_message/simple_message.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/smpl_msg_connection.h"
#include "industrial_robot_client/message_processor.h"


using namespace industrial::simple_message;
using namespace industrial::tcp_client;
using namespace industrial::smpl_msg_connection;
using namespace industrial_robot_client;
using namespace std;


namespace industrial_robot_client
{
namespace mikado_action_manager
{
class MikadoActionManager
{
public:
  MikadoActionManager();

  ~MikadoActionManager();

  void init(const string &robot_ip, int action_port, IMessageProcessor& processor);

  void connect();

  void push_request(SimpleMessage& request_msg);

  void push_reply(SimpleMessage& reply_msg);

  bool try_pop_request(SimpleMessage& request_msg);

  bool try_pop_reply(SimpleMessage& reply_msg);

  void process_requests();

  void cycle();
  


private:
  string robot_ip_;
  int action_port_;
  bool is_first_cycle_;
  IMessageProcessor* message_processor_;
  TcpClient tcp_client_;
  SimpleMessage request_msg_;
  SimpleMessage reply_msg_;
  std::queue<SimpleMessage> reply_buffer_;
  std::mutex reply_buffer_mutex_;
  std::queue<SimpleMessage> request_buffer_;
  std::mutex request_buffer_mutex_;
  std::thread processor_thread_;
  std::atomic<bool> processor_running_{false};
};
}
}

#endif
