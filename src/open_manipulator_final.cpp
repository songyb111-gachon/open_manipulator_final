#include "open_manipulator_final/open_manipulator_final.h"

#define INPUT_WAIT_TIME 2  // 두 번째 입력 대기 시간 (초)

OpenManipulatorPickandPlace::OpenManipulatorPickandPlace()
    : node_handle_(""),
      priv_node_handle_("~"),
      mode_state_(0),
      demo_count_(0),
      pick_ar_id_(0),
      pick_marker_id_(-1),   // 초기값: 유효하지 않은 ID
      place_marker_id_(-1)   // 초기값: 유효하지 않은 ID
{
    present_joint_angle_.resize(NUM_OF_JOINT_AND_TOOL, 0.0);
    present_kinematic_position_.resize(3, 0.0);

    joint_name_.push_back("joint1");
    joint_name_.push_back("joint2");
    joint_name_.push_back("joint3");
    joint_name_.push_back("joint4");

    initServiceClient();
    initSubscribe();
}

OpenManipulatorPickandPlace::~OpenManipulatorPickandPlace()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
}

void OpenManipulatorPickandPlace::initServiceClient()
{
    goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
    goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
    goal_task_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
}

void OpenManipulatorPickandPlace::initSubscribe()
{
    open_manipulator_states_sub_ = node_handle_.subscribe("states", 10, &OpenManipulatorPickandPlace::manipulatorStatesCallback, this);
    open_manipulator_joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorPickandPlace::jointStatesCallback, this);
    open_manipulator_kinematics_pose_sub_ = node_handle_.subscribe("gripper/kinematics_pose", 10, &OpenManipulatorPickandPlace::kinematicsPoseCallback, this);
    ar_pose_marker_sub_ = node_handle_.subscribe("/ar_pose_marker", 10, &OpenManipulatorPickandPlace::arPoseMarkerCallback, this);
}

bool OpenManipulatorPickandPlace::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.joint_position.joint_name = joint_name;
    srv.request.joint_position.position = joint_angle;
    srv.request.path_time = path_time;

    if (goal_joint_space_path_client_.call(srv))
    {
        return srv.response.is_planned;
    }
    return false;
}

bool OpenManipulatorPickandPlace::setToolControl(std::vector<double> joint_angle)
{
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.joint_position.joint_name.push_back("gripper");
    srv.request.joint_position.position = joint_angle;

    if (goal_tool_control_client_.call(srv))
    {
        return srv.response.is_planned;
    }
    return false;
}

bool OpenManipulatorPickandPlace::setTaskSpacePath(std::vector<double> kinematics_pose, std::vector<double> kinematics_orientation, double path_time)
{
    open_manipulator_msgs::SetKinematicsPose srv;

    srv.request.end_effector_name = "gripper";

    srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
    srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
    srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

    srv.request.kinematics_pose.pose.orientation.w = kinematics_orientation.at(0);
    srv.request.kinematics_pose.pose.orientation.x = kinematics_orientation.at(1);
    srv.request.kinematics_pose.pose.orientation.y = kinematics_orientation.at(2);
    srv.request.kinematics_pose.pose.orientation.z = kinematics_orientation.at(3);

    srv.request.path_time = path_time;

    if (goal_task_space_path_client_.call(srv))
    {
        return srv.response.is_planned;
    }
    return false;
}

void OpenManipulatorPickandPlace::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
    open_manipulator_is_moving_ = (msg->open_manipulator_moving_state == msg->IS_MOVING);
}

void OpenManipulatorPickandPlace::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    std::vector<double> temp_angle(NUM_OF_JOINT_AND_TOOL, 0.0);
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name.at(i) == "joint1") temp_angle[0] = msg->position[i];
        else if (msg->name.at(i) == "joint2") temp_angle[1] = msg->position[i];
        else if (msg->name.at(i) == "joint3") temp_angle[2] = msg->position[i];
        else if (msg->name.at(i) == "joint4") temp_angle[3] = msg->position[i];
        else if (msg->name.at(i) == "gripper") temp_angle[4] = msg->position[i];
    }
    present_joint_angle_ = temp_angle;
}

void OpenManipulatorPickandPlace::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
    present_kinematic_position_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
}

void OpenManipulatorPickandPlace::arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    ar_marker_pose.clear();
    for (const auto &marker : msg->markers)
    {
        ar_marker_pose.push_back({marker.id, {marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z}});
    }
}

void OpenManipulatorPickandPlace::publishCallback(const ros::TimerEvent&)
{
    printText();

    if (kbhit()) // 키 입력이 있는 경우
    {
        char input = std::getchar();
        if (isdigit(input))
        {
            processDigitInput(input);
        }
        else
        {
            setModeState(input);
        }
    }

    if (mode_state_ == HOME_POSE)
    {
        moveHomePose();
    }
    else if (mode_state_ == DEMO_START)
    {
        if (!open_manipulator_is_moving_)
            demoSequence();
    }
    else if (mode_state_ == DEMO_STOP)
    {
        printf("[INFO] Demo stopped.\n");
    }
}

void OpenManipulatorPickandPlace::processDigitInput(char first_input)
{
    clock_t start_time = clock();
    char second_input = '\0';

    while ((clock() - start_time) / CLOCKS_PER_SEC < INPUT_WAIT_TIME)
    {
        if (kbhit())
        {
            second_input = std::getchar();
            break;
        }
    }

    int marker_id = (second_input != '\0') ? (first_input - '0') * 10 + (second_input - '0') : (first_input - '0');

    if (marker_id < 0 || marker_id > 17)
    {
        printf("[WARNING] Invalid marker ID. Enter a number between 0 and 17.\n");
        return;
    }

    if (mode_state_ == DEMO_START)
    {
        if (demo_count_ <= 3)
        {
            pick_marker_id_ = marker_id;
            printf("[INFO] Pick Marker ID set to: %d\n", pick_marker_id_);
        }
        else if (demo_count_ >= 5)
        {
            place_marker_id_ = marker_id;
            printf("[INFO] Place Marker ID set to: %d\n", place_marker_id_);
        }
    }
}

void OpenManipulatorPickandPlace::moveHomePose()
{
    std::vector<double> joint_angle = {0.01, -0.80, 0.00, 1.90};
    setJointSpacePath(joint_name_, joint_angle, 2.0);

    std::vector<double> gripper_value = {0.0};
    setToolControl(gripper_value);

    mode_state_ = 0;
}


void OpenManipulatorPickandPlace::setModeState(char ch)
{
  if (ch == 'q')
    mode_state_ = HOME_POSE;
  else if (ch == 'w')
  {
    mode_state_ = DEMO_START;
    demo_count_ = 0;
  }
  else if (ch == 'e')
    mode_state_ = DEMO_STOP;
}

void OpenManipulatorPickandPlace::demoSequence()
{
  std::vector<double> joint_angle;
  std::vector<double> kinematics_position;
  std::vector<double> kinematics_orientation;
  std::vector<double> gripper_value;

  switch (demo_count_)
  {
    case 0: // home pose
    joint_angle.push_back( 0.00);
    joint_angle.push_back(-1.05);
    joint_angle.push_back( 0.35);
    joint_angle.push_back( 0.70);
    setJointSpacePath(joint_name_, joint_angle, 2.0);
    demo_count_ ++;
    break;

    case 1: // initial pose
      joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 2.0);
    demo_count_ ++;
    break;

    case 2: // wait & open the gripper
      setJointSpacePath(joint_name_, present_joint_angle_, 3.0);
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_ ++;
    break;

    case 3: // pick the box 사용자가 입력한 번호의 마커를 집음
{
  bool marker_found = false;
  int search_attempts = 0; // 검색 시도 횟수

  while (!marker_found && search_attempts < 8) // 최대 8번 시도
  {
    for (int i = 0; i < ar_marker_pose.size(); i++)
    {
      if (ar_marker_pose.at(i).id == pick_marker_id_) // 사용자가 설정한 pick_marker_id_
      {
        marker_found = true;

        // X, Y, Z 값 설정
        kinematics_position.push_back(ar_marker_pose.at(i).position[0] + 0.005); // X 좌표
        kinematics_position.push_back(ar_marker_pose.at(i).position[1]);        // Y 좌표
        kinematics_position.push_back(0.033);                                  // Z 좌표 고정

        // 오리엔테이션 설정
        kinematics_orientation.push_back(0.74); // w 값
        kinematics_orientation.push_back(0.00); // x 값
        kinematics_orientation.push_back(0.66); // y 값
        kinematics_orientation.push_back(0.00); // z 값

        setTaskSpacePath(kinematics_position, kinematics_orientation, 3.0);
        demo_count_++; // 다음 단계로 진행
        break;
      }
    }

    if (!marker_found)
    {
      // 마커를 찾지 못했을 경우 Base joint 변경
      printf("Pick Marker ID %d not detected. Adjusting base joint... (Attempt %d)\n", pick_marker_id_, search_attempts + 1);

      // Base joint (joint1) 값을 회전하며 탐색
      std::vector<double> search_joint_angle;
      search_joint_angle.push_back(-1.60 + 0.4 * search_attempts); // Base joint 좌우로 회전
      search_joint_angle.push_back(-0.80);                        // Shoulder joint
      search_joint_angle.push_back(0.00);                         // Elbow joint
      search_joint_angle.push_back(1.90);                         // Wrist joint
      setJointSpacePath(joint_name_, search_joint_angle, 2.0);   // 카메라 위치 조정

      ros::Duration(3.0).sleep(); // 3초 대기
      search_attempts++;          // 시도 횟수 증가
    }
  }

  if (!marker_found) // 최대 시도 후에도 찾지 못했을 경우
  {
    printf("Pick Marker ID %d could not be found after multiple attempts.\n", pick_marker_id_);
    demo_count_ = 1; // 초기 단계로 돌아감
  }
}
break;


  case 4: // wait & grip
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(-0.008);
    setToolControl(gripper_value);
    demo_count_++;
    break;

  case 5: // initial pose
    joint_angle.clear();
    joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 2.0);
    demo_count_++;
    break;

    case 6: // place the box 사용자가 입력한 마커가 있는 곳에 놓음
{
  bool marker_found = false;
  int search_attempts = 0; // 검색 시도 횟수

  while (!marker_found && search_attempts < 8) // 최대 8번 시도
  {
    for (int i = 0; i < ar_marker_pose.size(); i++)
    {
      if (ar_marker_pose.at(i).id == place_marker_id_) // 사용자가 설정한 place_marker_id_
      {
        marker_found = true;

        // X, Y, Z 값 설정
        kinematics_position.push_back(ar_marker_pose.at(i).position[0] + 0.005); // X 좌표
        kinematics_position.push_back(ar_marker_pose.at(i).position[1]);        // Y 좌표
        kinematics_position.push_back(0.069);                                  // Z 좌표 고정

        // 오리엔테이션 설정
        kinematics_orientation.push_back(0.74); // w 값
        kinematics_orientation.push_back(0.00); // x 값
        kinematics_orientation.push_back(0.66); // y 값
        kinematics_orientation.push_back(0.00); // z 값

        setTaskSpacePath(kinematics_position, kinematics_orientation, 3.0);
        demo_count_++; // 다음 단계로 진행
        break;
      }
    }

    if (!marker_found)
    {
      // 마커를 찾지 못했을 경우 Base joint 변경
      printf("Place Marker ID %d not detected. Adjusting base joint... (Attempt %d)\n", place_marker_id_, search_attempts + 1);

      // Base joint (joint1) 값을 회전하며 탐색
      std::vector<double> search_joint_angle;
      search_joint_angle.push_back(-1.60 + 0.4 * search_attempts); // Base joint 좌우로 회전
      search_joint_angle.push_back(-0.80);                        // Shoulder joint
      search_joint_angle.push_back(0.00);                         // Elbow joint
      search_joint_angle.push_back(1.90);                         // Wrist joint
      setJointSpacePath(joint_name_, search_joint_angle, 2.0);   // 카메라 위치 조정

      ros::Duration(3.0).sleep(); // 1초 대기
      search_attempts++;          // 시도 횟수 증가
    }
  }

  if (!marker_found) // 최대 시도 후에도 찾지 못했을 경우
  {
    printf("Place Marker ID %d could not be found after multiple attempts.\n", place_marker_id_);
    demo_count_ = 6; // 놓기 단계로 돌아감
  }
}
break;



  case 7: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;


  case 8: // move up after place the box
    kinematics_position.clear();
    kinematics_orientation.clear();


    kinematics_position.push_back(ar_marker_pose.at(place_marker_id_).position[0] + 0.030); // X 좌표
    kinematics_position.push_back(ar_marker_pose.at(place_marker_id_).position[1] + 0.030);        // Y 좌표
    kinematics_position.push_back(0.170);                                                 // Z 좌표 (상승)

    // 기존 오리엔테이션 유지
    kinematics_orientation.push_back(0.74); // w 값
    kinematics_orientation.push_back(0.00); // x 값
    kinematics_orientation.push_back(0.66); // y 값
    kinematics_orientation.push_back(0.00); // z 값

    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0); // 위치로 이동
    demo_count_++;
	break;

case 9: // Prompt user to decide next action
{
    printf("\nWhat would you like to do next?\n");
    printf("Press 'p' to pick another object, or 'd' to proceed to demo termination.\n");

    char user_input = '\0';  // 초기화
    while (true) // 유효한 입력을 받을 때까지 무한 루프
    {
        if (kbhit()) // 키 입력 대기
        {
            user_input = std::getchar();
            if (user_input == 'p') // Pick another object
            {
                demo_count_ = 1; // Case 1로 설정하여 pick 과정으로 돌아감
                printf("[INFO] Returning to pick another object.\n");
                break; // 루프 종료
            }
            else if (user_input == 'd') // Enter demo termination process
            {
                demo_count_++; // 다음 단계(종료 과정)로 진행
                printf("[INFO] Proceeding to demo termination process.\n");
                break; // 루프 종료
            }
            else
            {
                printf("[WARNING] Invalid input. Please press 'p' or 'd'.\n");
            }
        }
        ros::Duration(0.1).sleep(); // ROS 노드가 응답을 유지하도록 100ms 대기
    }
    break;
}


    case 10: //I
    joint_angle.clear();
    joint_angle.push_back( -0.063);
    joint_angle.push_back( 0.061);
    joint_angle.push_back( -1.488);
    joint_angle.push_back( -0.012);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;

    case 11: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;

    case 12: //R
    joint_angle.clear();
    joint_angle.push_back( -0.015);
    joint_angle.push_back( 0.030);
    joint_angle.push_back( 0.779);
    joint_angle.push_back( 1.759);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;

    case 13: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;

    case 14: //임시
    joint_angle.clear();
    joint_angle.push_back( 0.00);
    joint_angle.push_back(-1.05);
    joint_angle.push_back( 0.35);
    joint_angle.push_back( 0.70);
    setJointSpacePath(joint_name_, joint_angle, 0.1);
    demo_count_++;
    break;

    case 15: //A
    joint_angle.clear();
    joint_angle.push_back( -0.032);
    joint_angle.push_back( 0.078);
    joint_angle.push_back( 0.894);
    joint_angle.push_back( 0.021);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;

    case 16: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;

    case 17: //S
    joint_angle.clear();
    joint_angle.push_back( 0.000);
    joint_angle.push_back( -1.085);
    joint_angle.push_back( 0.508);
    joint_angle.push_back( -0.341);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;

    case 18: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;

    case 19: //C
    joint_angle.clear();
    joint_angle.push_back( -0.031);
    joint_angle.push_back( -1.235);
    joint_angle.push_back( 0.032);
    joint_angle.push_back( 1.119);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;

    case 20: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;

    case 21: // home pose
    joint_angle.clear();
    joint_angle.push_back( 0.00);
    joint_angle.push_back(-1.05);
    joint_angle.push_back( 0.35);
    joint_angle.push_back( 0.70);
    setJointSpacePath(joint_name_, joint_angle, 0.01);
    demo_count_ = 1;
    mode_state_ = DEMO_STOP;
    break;


  default:
    demo_count_++;
    break;
  }
}



void OpenManipulatorPickandPlace::printText()
{
  system("clear");

  printf("\n");
  printf("-----------------------------\n");
  printf("Pick and Place demonstration!\n");
  printf("-----------------------------\n");

  printf("q : Home pose\n");
  printf("w : Pick and Place demo. start\n");
  printf("e : Pick and Place demo. Stop\n");

  printf("-----------------------------\n");

  if (mode_state_ == DEMO_START)
  {
    switch (demo_count_)
    {
    case 1: // home pose
      printf("Move home pose\n");
      break;
    case 2: // initial pose
      printf("Move initial pose\n");
      break;
    case 3:
      printf("Detecting AR marker for pick\n");
      break;
    case 4:
      printf("Grip preparation\n");
      break;
    case 5:
    case 6:
      printf("Pick the box\n");
      break;
    case 7:
    case 8:
      printf("Placing the box\n");
      break;
    case 9:
    case 10:
      printf("Moving up after placing\n");
      break;
    case 11:
      printf("I\n");
      break;
    case 12:
      printf("wait\n");
      break;
    case 13:
      printf("R\n");
      break;
    case 14:
      printf("wait\n");
      break;
    case 15:
      printf("wait\n");
      break;
    case 16:
      printf("A\n");
      break;
    case 17:
      printf("wait\n");
      break;
    case 18:
      printf("S\n");
      break;
    case 19:
      printf("wait\n");
      break;
    case 20:
      printf("C\n");
      break;
    case 21:
        printf("C\n");
    break;
    case 22:
      printf("Returning to home pose\n");
    break;
    default:
      printf("Unknown demo state\n");
    break;
    }
  }
  else if (mode_state_ == DEMO_STOP)
  {
    printf("The end of demo\n");
  }

  printf("-----------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         present_joint_angle_.at(0),
         present_joint_angle_.at(1),
         present_joint_angle_.at(2),
         present_joint_angle_.at(3));
  printf("Present Tool Position: %.3lf\n", present_joint_angle_.at(4));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         present_kinematic_position_.at(0),
         present_kinematic_position_.at(1),
         present_kinematic_position_.at(2));

  if (!ar_marker_pose.empty())
  {
    printf("AR marker detected.\n");
    for (int i = 0; i < ar_marker_pose.size(); i++)
    {
      printf("ID: %d --> X: %.3lf\tY: %.3lf\tZ: %.3lf\n",
             ar_marker_pose.at(i).id,
             ar_marker_pose.at(i).position[0],
             ar_marker_pose.at(i).position[1],
             ar_marker_pose.at(i).position[2]);
    }
  }
  else
  {
    printf("No AR marker detected. Waiting for marker input...\n");
  }
}



bool OpenManipulatorPickandPlace::kbhit()
{
  termios term;
  tcgetattr(0, &term);

  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_pick_and_place");
  ros::NodeHandle node_handle("");

  OpenManipulatorPickandPlace open_manipulator_pick_and_place;

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/, &OpenManipulatorPickandPlace::publishCallback, &open_manipulator_pick_and_place);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}