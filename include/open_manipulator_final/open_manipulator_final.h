#ifndef OPEN_MANIPULATOR_FINAL_H
#define OPEN_MANIPULATOR_FINAL_H

#include <ros/ros.h>
#include <termios.h>
#include <sys/ioctl.h>

#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/KinematicsPose.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "sensor_msgs/JointState.h"

#define NUM_OF_JOINT_AND_TOOL 5
#define HOME_POSE   'q'
#define DEMO_START  'w'
#define DEMO_STOP   'e'

typedef struct _ArMarker
{
  uint32_t id;
  double position[3];
} ArMarker;

class OpenManipulatorPickandPlace
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_tool_control_client_;
  ros::ServiceClient goal_task_space_path_client_;

  ros::Subscriber open_manipulator_states_sub_;
  ros::Subscriber open_manipulator_joint_states_sub_;
  ros::Subscriber open_manipulator_kinematics_pose_sub_;
  ros::Subscriber ar_pose_marker_sub_;

  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  std::vector<std::string> joint_name_;
  bool open_manipulator_is_moving_;
  std::vector<ArMarker> ar_marker_pose;

  uint8_t mode_state_;
  uint8_t demo_count_;
  uint8_t pick_ar_id_;
  uint8_t pick_marker_id_;   // 집을 마커 ID
  uint8_t place_marker_id_;  // 놓을 마커 ID

 public:
  OpenManipulatorPickandPlace();
  ~OpenManipulatorPickandPlace();

  void initServiceClient();
  void initSubscribe();

  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
  void processDigitInput(char first_input);
  void moveHomePose();


  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setToolControl(std::vector<double> joint_angle);
  bool setTaskSpacePath(std::vector<double> kinematics_pose, std::vector<double> kinematics_orientation, double path_time);


  void publishCallback(const ros::TimerEvent&);
  void setModeState(char ch);
  void demoSequence();


  void printText();
  bool kbhit();
};

#endif //OPEN_MANIPULATOR_final_H
