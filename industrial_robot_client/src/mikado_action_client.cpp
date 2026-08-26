#include <unistd.h>

#include "simple_message/mikado_messages/mikado_types.h"
#include "industrial_robot_client/mikado_action_manager.h"
#include "industrial_robot_client/message_processor.h"

using namespace industrial_robot_client;
using namespace industrial_robot_client::mikado_action_manager;

void getConnectionInfo(ConnectionInfo& connection_info){
  connection_info.mikado_product = mik_product_type::PICK;
  connection_info.rotation_convention = mik_rotation_convention::R_XYZ;
  connection_info.number_of_axis_traj_pts = 6;
  connection_info.number_of_axis_status = 6;
  connection_info.number_of_ext_axis_traj_pts = 0;
  connection_info.number_of_ext_axis_status = 0;
  connection_info.status_is_radian = false;
  connection_info.traj_pt_is_radian = false;
  connection_info.is_status_is_moving = false;
}


int main(int argc, char **argv)
{
  printf("--- STARTING MIKADO ACTION CLIENT ---\n");
  if (argc < 3)
  {
    printf("[CLIENT] Usage: %s <server_ip> <port>\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  std::string robot_ip = argv[1];
  int SERVER_PORT = std::stoi(argv[2]);
  ConnectionInfo connection_info;
  getConnectionInfo(connection_info);
  
  SimpleMessageProcessor processor(connection_info);

  MikadoActionManager action_manager;
  action_manager.init(robot_ip, SERVER_PORT, processor);

  action_manager.connect();

  while(true){
    action_manager.cycle();
    sleep(1);
  }
  return 0;
}