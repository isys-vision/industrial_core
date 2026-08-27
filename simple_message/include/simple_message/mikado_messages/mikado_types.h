#ifndef MIKADO_TYPES_H
#define MIKADO_TYPES_H

// ============================================================================
// MIKADO TYPES - Constant Definitions
// ============================================================================

// --- Communication Types ---
namespace mik_comm_type
{
  const int INVALID = 0;
  const int TOPIC = 1;
  const int SERVICE_REQUEST = 2;
  const int SERVICE_REPLY = 3;
}

// --- Product Types ---
namespace mik_product_type
{
  const int INVALID = 0;
  const int FIND = 1;
  const int SELECT = 2;
  const int PICK = 3;
  const int CONTROL = 4;
}

// --- Product Types ---
namespace mik_rotation_convention
{
  const int INVALID = 0;
  const int R_XYZ = 1; // default, used by Stäubli robots
    const int STAUBLI = R_XYZ;
  const int S_XYZ = 2; // used by Fanuc, Yaskawa, UR, Fruitcore, Mitsubishi and Techman robots
    const int FANUC = S_XYZ;
    const int YASKAWA = S_XYZ;
    const int UR_1 = S_XYZ;
    const int FRUITCORE = S_XYZ;
    const int MITSUBISHI = S_XYZ;
    const int TECHMAN = S_XYZ;
  const int R_ZYX = 3; // used by Kuka, ABB and Nachi robots
    const int KUKA = R_ZYX;
    const int ABB = R_ZYX;
    const int NACHI = R_ZYX;
  const int R_ZYZ = 4; // used by Doosan, Kawasaki and Omron robots
    const int DOOSAN = R_ZYZ;
    const int KAWASAKI = R_ZYZ;
    const int OMRON = R_ZYZ;
  const int UVW = 5; // used by UR robots
    const int UR_2 = UVW;

}

// --- Mikado Message Types ---
namespace mik_msg_type
{
  const int INVALID = 0;
  const int ACTION_TRIG = 65000; // action trigger
  const int MIK_STATUS = 65001; // mikado_status
  const int TRAJ_PT = 65010; // trajectory pt
  const int TRAJ = 65003; // set of trajectory pts
  const int CALIB_RES = 65004; // calibration result
  const int ROBOT_INFO = 65005; // robot info / robot details
  const int CALIB_PLAN = 65006; // calibration plan
  const int ROBOT_STATUS = 65007; // robot status 
  const int CONN_INFO = 65008; // connection info
  const int SIMPLE_REPLY = 65100; // simple reply
  const int REPLY_STATUS = 65101; // status reply
  const int REPLY_POSE = 65102; // pose reply
}

// --- Mikado Trajectory Part Types ---
namespace mik_traj_type
{
  const int INVALID = 0;
  const int PRE_CALIB = 1;
  const int CALIB = 2;
  const int REGULAR = 3;
  const int APPROACH = 4;
  const int GRASP = 5;
  const int EXTRACT = 6;
  const int RETRACT = 7;
}

// --- Mikado Motion Types ---
namespace mik_motion_type
{
  const int INVALID = 0;
  const int JOINT = 1;
  const int LINEAR = 2;
  const int SPLINE = 3;
  const int ARC = 4;
  const int APPROXIMATE = 5;
}

// --- Action IDs ---
namespace mik_action_id
{
  // Actions (set commands)
  const int CAPTURE_PC = 1000;
  const int FIND_CONTAINER = 1001;
  const int FIND_PRODUCT = 1002;
  const int FIND_PICKS = 1003;
  const int COUNT_PTS_IN_ROI = 1004;
  const int LOAD_RECIPE = 1005;
  const int START_CALIB = 1100;
  const int CAPTURE_CALIB = 1101;
  const int FINISH_CALIB = 1102;
  const int CREATE_ERR_REPORT = 1999;

  // Getters (get commands)
  const int GET_PICK_TRAJ = 2000;
  const int GET_ROI_ID = 2001;
  const int GET_PRODUCT_ID = 2002;
  const int GET_GRIPPER_ID = 2003;
  const int GET_CAMERA_ID = 2004;
  const int GET_GRASP_ID = 2005;
  const int GET_GRASP_PRD_POS = 2006;
  const int GET_GRASP_POSE = 2007;
  const int GET_CALIB_PLAN = 2100;
  const int GET_CALIB_RESULTS = 2101;
  const int GET_CONNECTION_INFO = 2102;
  const int GET_MIK_STATUS = 2999;

  // Setters (set commands)
  const int SET_ROI_BY_ID = 3000;
  const int SET_PRODUCT_BY_ID = 3001;
  const int SET_GRIPPER_BY_ID = 3003;
  const int SET_CAMERA_BY_ID = 3004;
  const int SET_EXTERNAL_AXIS = 3005;
  const int SET_GRASPED = 3006;
  const int SET_DETACHED_OBJ = 3007;
  const int SET_GRIPPER_OPEN = 3008;
  const int SET_GRIPPER_CLOSE = 3010;
  const int SET_CALIB_PLAN = 3100;
}

// here we can collect all possible action status
// should there be mapping to error msg as well?
namespace mik_action_status{
  namespace success{
  const int OK = 0;
  }
  namespace error{
  const int GENERAL_ERROR = 1500;
  const int NO_CAMERA_WITH_THIS_ID = 1501;
  }
}

namespace mik_connection_info{
  struct ConnectionInfo{
    int mikado_product;
    int rotation_convention;
    int number_of_axis_traj_pts;
    int number_of_ext_axis_traj_pts;
    int number_of_axis_status;
    int number_of_ext_axis_status;
    bool traj_pt_is_radian;
    bool status_is_radian;
    bool is_status_is_moving;
  };
}

const int MIK_EOM_LEN = 4;
const int MIK_EOM = -1; // by loading -1 into the buffer, we get 4 FF Bytes, that mark the end of message
#endif // MIKADO_TYPES_H