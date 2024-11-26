/**
 * OpenManipulator Final Demo Code
 * Author: 송영빈 (GitHub: https://github.com/songyb111-gachon)
 * Description: This code is developed for the final demonstration of
 *              OpenManipulator Pick and Place functionality with AR marker tracking.
 *              The code recognizes AR markers based on user input for picking and placing tasks.
 * Date: [2024-11-23 ~ 2024-11-26]
 *
 * Note: This code is part of the Drones and Robotics team project under the supervision
 *       of Prof. Andrew Jaeyong Choi at Gachon University.
 *
 * Repository: https://github.com/songyb111-gachon/open_manipulator_final
 *
 * Instructions:
 * - This project demonstrates the use of AR marker-based robotic arm manipulation.
 * - It includes picking and placing tasks using AR markers identified in a 3D space.
 * - The code integrates with ROS to control OpenManipulator hardware and process sensor data.
 * - User input is used to specify the AR markers for the robotic arm to pick and place.
 *
 * Usage:
 * 1. Clone the repository: `git clone https://github.com/songyb111-gachon/open_manipulator_final`
 * 2. Build the project: catkin build
 * 3. Run the node: `rosrun open_manipulator_final open_manipulator_final`.
 *
 * Disclaimer:
 * This code is for educational purposes as part of the team project. Use it with caution in real-world environments.
 */


#include "open_manipulator_final/open_manipulator_final.h"

#define INPUT_WAIT_TIME 1  // 두 번째 입력 대기 시간 (초)

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
    output_buffer_.str("");
    output_buffer_.clear();
    output_buffer_ << "[INFO] Starting demo\n ";
    std::cout << output_buffer_.str() << std::flush; // 즉시 출력
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


      case 3: // Request Pick Marker ID
      {
          // 버퍼 초기화
          output_buffer_.str(""); // 버퍼 내용 비우기
          output_buffer_.clear();

          // 메시지 출력
          output_buffer_ << "\n[INFO] Enter Pick Marker ID (0-17): ";
          std::cout << output_buffer_.str() << std::flush; // 즉시 출력

          char first_input = '\0';
          char second_input = '\0';
          int marker_id = -1; // 초기화된 ID 값

          while (true) // 유효한 입력을 받을 때까지 반복
          {
              if (kbhit()) // 키 입력 대기
              {
                  first_input = std::getchar(); // 첫 번째 입력
                  if (isdigit(first_input)) // 첫 번째 입력이 숫자인지 확인
                  {
                      clock_t start_time = clock(); // 두 번째 입력 대기 시간 측정
                      while ((clock() - start_time) / CLOCKS_PER_SEC < INPUT_WAIT_TIME)
                      {
                          if (kbhit()) // 두 번째 입력 대기
                          {
                              second_input = std::getchar(); // 두 번째 입력
                              if (isdigit(second_input)) // 두 번째 입력이 숫자인지 확인
                              {
                                  marker_id = (first_input - '0') * 10 + (second_input - '0'); // 두 글자 조합
                                  break;
                              }
                          }
                      }
                      if (second_input == '\0') // 두 번째 입력이 없으면 한 글자만 사용
                      {
                          marker_id = first_input - '0';
                      }

                      if (marker_id >= 0 && marker_id <= 17) // 유효한 범위 확인
                      {
                          // 버퍼 초기화 후 메시지 작성
                          output_buffer_.str(""); // 버퍼 내용 비우기
                          output_buffer_.clear();
                          pick_marker_id_ = marker_id;
                          output_buffer_ << "[INFO] Pick Marker ID set to: " << pick_marker_id_ << "\n";
                          std::cout << output_buffer_.str() << std::flush; // 즉시 출력
                          demo_count_++; // 다음 단계로 이동
                          break;
                      }
                      else
                      {
                          // 잘못된 입력 처리
                          output_buffer_.str(""); // 버퍼 내용 비우기
                          output_buffer_.clear();
                          output_buffer_ << "[WARNING] Invalid input. Please enter a number between 0 and 17.\n";
                          std::cout << output_buffer_.str() << std::flush; // 즉시 출력
                      }
                  }
                  else
                  {
                      // 잘못된 첫 번째 입력 처리
                      output_buffer_.str(""); // 버퍼 내용 비우기
                      output_buffer_.clear();
                      output_buffer_ << "[WARNING] Invalid input. Please enter a number between 0 and 17.\n";
                      std::cout << output_buffer_.str() << std::flush; // 즉시 출력
                  }
              }
              ros::Duration(0.1).sleep(); // ROS 노드가 응답을 유지하도록 100ms 대기
          }
          break;
      }


case 4: // pick the box 사용자가 입력한 번호의 마커를 감지
{
    // 버퍼 초기화
    output_buffer_.str("");
    output_buffer_.clear();

    // 디버깅 메시지: pick_marker_id_ 확인
    std::cout << "[DEBUG] Starting Case 4 with Pick Marker ID: " << pick_marker_id_ << std::endl;

    bool marker_found = false;
    int search_attempts = 0;

    while (!marker_found && search_attempts < 9) // 최대 9번 시도
    {
        // 탐색 동작 수행 메시지 출력
        output_buffer_.str("");
        output_buffer_.clear();
        output_buffer_ << "[WARNING] Pick Marker ID " << pick_marker_id_
                       << " not detected. Adjusting base joint... (Attempt "
                       << search_attempts + 1 << ")\n";
        std::cout << output_buffer_.str() << std::flush; // 즉시 출력

        // 베이스 조인트 조정
        std::vector<double> search_joint_angle = {-1.60 + 0.4 * search_attempts, -0.80, 0.00, 1.90};
        setJointSpacePath(joint_name_, search_joint_angle, 2.0);

        // 루프를 활용한 대기
        ros::Time start_time = ros::Time::now();
        ros::Duration wait_duration(2.0); // 이동 대기 시간: 2초

        while (ros::Time::now() - start_time < wait_duration)
        {
            ros::spinOnce(); // ROS 콜백 처리
            ros::Duration(0.1).sleep(); // 100ms 간격으로 짧게 대기
        }

        search_attempts++;

        // 탐색 시작
        ros::Time detection_start_time = ros::Time::now(); // 탐색 시작 시간
        ros::Duration detection_duration(6.0);  // 감지 시도 시간을 6초로 설정

        std::cout << "[DEBUG] Attempt " << search_attempts << ": Searching for Marker ID " << pick_marker_id_ << std::endl;

        while (ros::Time::now() - detection_start_time < detection_duration) // 6초 동안 감지 반복
        {
            ros::spinOnce(); // 콜백 강제 실행

            for (size_t i = 0; i < ar_marker_pose.size(); i++)
            {
                if (ar_marker_pose.at(i).id == pick_marker_id_)
                {
                    marker_found = true;

                    // 성공 메시지 출력
                    output_buffer_.str("");
                    output_buffer_.clear();
                    output_buffer_ << "[INFO] Pick Marker ID " << pick_marker_id_
                                   << " detected. Proceeding to next case.\n";
                    std::cout << output_buffer_.str() << std::flush; // 즉시 출력

                    // 작업에 필요한 데이터를 저장
                    target_kinematics_position_.clear();
                    target_kinematics_orientation_.clear();

                    target_kinematics_position_.push_back(ar_marker_pose.at(i).position[0] + 0.005);
                    target_kinematics_position_.push_back(ar_marker_pose.at(i).position[1]);
                    target_kinematics_position_.push_back(0.033);

                    target_kinematics_orientation_.push_back(0.74);
                    target_kinematics_orientation_.push_back(0.00);
                    target_kinematics_orientation_.push_back(0.66);
                    target_kinematics_orientation_.push_back(0.00);

                    break;
                }
            }

            if (marker_found)
            {
                break; // 마커를 찾으면 내부 감지 루프 종료
            }

            ros::Duration(0.1).sleep(); // 100ms 대기 후 다시 감지 시도
        }

        if (marker_found)
        {
            break; // 마커를 찾으면 전체 탐색 루프 종료
        }
    }

    if (!marker_found)
    {
        // 실패 메시지 출력
        output_buffer_.str("");
        output_buffer_.clear();
        output_buffer_ << "[ERROR] Pick Marker ID " << pick_marker_id_
                       << " could not be found after multiple attempts.\n";
        std::cout << output_buffer_.str() << std::flush; // 즉시 출력

        demo_count_ = 1; // 초기 단계로 돌아감
    }
    else
    {
        demo_count_++; // 다음 단계로 진행
    }

    break;
}


case 5: // 마커를 감지 후 작업 수행
{
    // target_kinematics_position_과 target_kinematics_orientation_을 사용
    if (!target_kinematics_position_.empty() && !target_kinematics_orientation_.empty())
    {
        setTaskSpacePath(target_kinematics_position_, target_kinematics_orientation_, 3.0);

        output_buffer_.str("");
        output_buffer_.clear();
        output_buffer_ << "[INFO] Pick task executed for Marker ID " << pick_marker_id_ << ".\n";
        std::cout << output_buffer_.str() << std::flush; // 즉시 출력

        demo_count_++; // 다음 단계로 진행
    }
    else
    {
        output_buffer_.str("");
        output_buffer_.clear();
        output_buffer_ << "[ERROR] No target position/orientation set. Returning to initial state.\n";
        std::cout << output_buffer_.str() << std::flush; // 즉시 출력

        demo_count_ = 1; // 초기 단계로 돌아감
    }

    break;
}


  case 6: // wait & grip
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(-0.008);
    setToolControl(gripper_value);
    demo_count_++;
    break;


  case 7: // initial pose
    joint_angle.clear();
    joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 2.0);
    demo_count_++;
    break;

    
case 8: // Request Place Marker ID
{
    // 버퍼 초기화
    output_buffer_.str(""); // 버퍼 내용 비우기
    output_buffer_.clear();

    // 메시지 출력
    output_buffer_ << "\n[INFO] Enter Place Marker ID (0-17): ";
    std::cout << output_buffer_.str() << std::flush; // 즉시 출력

    char first_input = '\0';
    char second_input = '\0';
    int marker_id = -1; // 초기화된 ID 값

    while (ros::ok()) // ROS가 실행 중일 때만 반복
    {
        if (kbhit()) // 키 입력 대기
        {
            first_input = std::getchar(); // 첫 번째 입력
            if (isdigit(first_input)) // 첫 번째 입력이 숫자인지 확인
            {
                ros::Time start_time = ros::Time::now(); // 두 번째 입력 대기 시간 측정
                ros::Duration wait_duration(INPUT_WAIT_TIME);

                while (ros::Time::now() - start_time < wait_duration)
                {
                    if (kbhit()) // 두 번째 입력 대기
                    {
                        second_input = std::getchar(); // 두 번째 입력
                        if (isdigit(second_input)) // 두 번째 입력이 숫자인지 확인
                        {
                            marker_id = (first_input - '0') * 10 + (second_input - '0'); // 두 글자 조합
                            break;
                        }
                    }
                    ros::spinOnce(); // ROS 콜백 처리
                    ros::Duration(0.1).sleep(); // 100ms 대기
                }

                if (second_input == '\0') // 두 번째 입력이 없으면 한 글자만 사용
                {
                    marker_id = first_input - '0';
                }

                if (marker_id >= 0 && marker_id <= 17) // 유효한 범위 확인
                {
                    // 버퍼 초기화 후 메시지 작성
                    output_buffer_.str(""); // 버퍼 내용 비우기
                    output_buffer_.clear();
                    place_marker_id_ = marker_id; // Place Marker ID 설정
                    output_buffer_ << "[INFO] Place Marker ID set to: " << place_marker_id_ << "\n";
                    std::cout << output_buffer_.str() << std::flush; // 즉시 출력
                    demo_count_++; // 다음 단계로 이동
                    break;
                }
                else
                {
                    // 잘못된 입력 처리
                    output_buffer_.str(""); // 버퍼 내용 비우기
                    output_buffer_.clear();
                    output_buffer_ << "[WARNING] Invalid input. Please enter a number between 0 and 17.\n";
                    std::cout << output_buffer_.str() << std::flush; // 즉시 출력
                }
            }
            else
            {
                // 잘못된 첫 번째 입력 처리
                output_buffer_.str(""); // 버퍼 내용 비우기
                output_buffer_.clear();
                output_buffer_ << "[WARNING] Invalid input. Please enter a number between 0 and 17.\n";
                std::cout << output_buffer_.str() << std::flush; // 즉시 출력
            }
        }
        ros::spinOnce(); // ROS 콜백 처리
        ros::Duration(0.1).sleep(); // 100ms 대기
    }
    break;
}


case 9: // place the box 사용자가 입력한 마커가 있는 곳에 감지만 수행
{
    // 버퍼 초기화
    output_buffer_.str("");
    output_buffer_.clear();

    // 디버깅 메시지: Place Marker ID 확인
    std::cout << "[DEBUG] Starting Case 9 with Place Marker ID: " << place_marker_id_ << std::endl;

    bool marker_found = false;
    int search_attempts = 0;

    while (!marker_found && search_attempts < 9) // 최대 9번 시도
    {
        // 탐색 동작 수행 메시지 출력
        output_buffer_.str("");
        output_buffer_.clear();
        output_buffer_ << "[WARNING] Place Marker ID " << place_marker_id_
                       << " not detected. Adjusting base joint... (Attempt "
                       << search_attempts + 1 << ")\n";
        std::cout << output_buffer_.str() << std::flush; // 즉시 출력

        // 베이스 조인트 조정
        std::vector<double> search_joint_angle = {-1.60 + 0.4 * search_attempts, -0.80, 0.00, 1.90};
        setJointSpacePath(joint_name_, search_joint_angle, 2.0);

        // 이동 완료를 루프를 통해 대기
        ros::Time start_time = ros::Time::now();
        ros::Duration wait_duration(2.0); // 이동 대기 시간: 2초

        while (ros::Time::now() - start_time < wait_duration)
        {
            ros::spinOnce(); // ROS 콜백 처리
            ros::Duration(0.1).sleep(); // 100ms 간격으로 짧게 대기
        }

        search_attempts++;

        // 탐색 시작
        ros::Time detection_start_time = ros::Time::now(); // 탐색 시작 시간
        ros::Duration detection_duration(6.0);  // 감지 시도 시간을 6초로 설정

        std::cout << "[DEBUG] Attempt " << search_attempts << ": Searching for Marker ID " << place_marker_id_ << std::endl;

        while (ros::Time::now() - detection_start_time < detection_duration) // 6초 동안 감지 반복
        {
            ros::spinOnce(); // 콜백 강제 실행

            for (size_t i = 0; i < ar_marker_pose.size(); i++)
            {
                if (ar_marker_pose.at(i).id == place_marker_id_)
                {
                    marker_found = true;

                    // 성공 메시지 출력
                    output_buffer_.str("");
                    output_buffer_.clear();
                    output_buffer_ << "[INFO] Place Marker ID " << place_marker_id_
                                   << " detected. Proceeding to next case.\n";
                    std::cout << output_buffer_.str() << std::flush; // 즉시 출력

                    // 작업에 필요한 데이터를 저장
                    target_place_position_.clear();
                    target_place_orientation_.clear();

                    target_place_position_.push_back(ar_marker_pose.at(i).position[0] + 0.005);
                    target_place_position_.push_back(ar_marker_pose.at(i).position[1]);
                    target_place_position_.push_back(0.079);

                    target_place_orientation_.push_back(0.74);
                    target_place_orientation_.push_back(0.00);
                    target_place_orientation_.push_back(0.66);
                    target_place_orientation_.push_back(0.00);

                    break;
                }
            }

            if (marker_found)
            {
                break; // 마커를 찾으면 내부 감지 루프 종료
            }

            ros::Duration(0.1).sleep(); // 100ms 대기 후 다시 감지 시도
        }

        if (marker_found)
        {
            break; // 마커를 찾으면 전체 탐색 루프 종료
        }
    }

    if (!marker_found)
    {
        // 실패 메시지 출력
        output_buffer_.str("");
        output_buffer_.clear();
        output_buffer_ << "[ERROR] Place Marker ID " << place_marker_id_
                       << " could not be found after multiple attempts.\n";
        std::cout << output_buffer_.str() << std::flush; // 즉시 출력

        demo_count_ = 6; // 이전 단계로 돌아감
    }
    else
    {
        demo_count_++; // 다음 단계로 진행
    }

    break;
}


case 10: // 감지한 위치에 물체 배치
{
    if (!target_place_position_.empty() && !target_place_orientation_.empty())
    {
        // 작업 공간 경로 설정
        setTaskSpacePath(target_place_position_, target_place_orientation_, 3.0);

        // 성공 메시지 출력
        output_buffer_.str("");
        output_buffer_.clear();
        output_buffer_ << "[INFO] Successfully placed the object at Marker ID: " << place_marker_id_ << ".\n";
        std::cout << output_buffer_.str() << std::flush; // 즉시 출력

        demo_count_++; // 다음 단계로 진행
    }
    else
    {
        // 실패 메시지 출력
        output_buffer_.str("");
        output_buffer_.clear();
        output_buffer_ << "[ERROR] No target position/orientation set. Returning to initial state.\n";
        std::cout << output_buffer_.str() << std::flush; // 즉시 출력

        demo_count_ = 6; // 이전 단계로 돌아감
    }

    break;
}


  case 11: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;


  case 12: // move up after place the box
    kinematics_position.clear();
    kinematics_orientation.clear();
    kinematics_position.push_back(present_kinematic_position_.at(0));
    kinematics_position.push_back(present_kinematic_position_.at(1));
    kinematics_position.push_back(0.180);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    demo_count_++;
    break;


case 13: // Prompt user to decide next action
{
    // 버퍼 초기화
    output_buffer_.str("");
    output_buffer_.clear();

    // 사용자 입력 요청 메시지 출력
    output_buffer_ << "\nWhat would you like to do next?\n";
    output_buffer_ << "Press 'p' to pick another object, or 'd' to proceed to demo termination.\n";
    std::cout << output_buffer_.str() << std::flush; // 즉시 출력

    char user_input = '\0'; // 초기화
    while (true) // 유효한 입력을 받을 때까지 반복
    {
        if (kbhit()) // 키 입력 대기
        {
            user_input = std::getchar();
            if (user_input == 'p') // Pick another object
            {
                // Pick과 Place ID를 초기화
                pick_marker_id_ = -1;  // 유효하지 않은 상태로 초기화
                place_marker_id_ = -1; // 유효하지 않은 상태로 초기화

                demo_count_ = 1; // Case 1로 설정하여 pick 과정으로 돌아감

                // 성공 메시지 출력
                output_buffer_.str("");
                output_buffer_.clear();
                output_buffer_ << "[INFO] Returning to pick another object. Marker IDs have been reset.\n";
                std::cout << output_buffer_.str() << std::flush; // 즉시 출력
                break;
            }
            else if (user_input == 'd') // Enter demo termination process
            {
                demo_count_++; // 다음 단계(종료 과정)로 진행

                // 성공 메시지 출력
                output_buffer_.str("");
                output_buffer_.clear();
                output_buffer_ << "[INFO] Proceeding to demo termination process.\n";
                std::cout << output_buffer_.str() << std::flush; // 즉시 출력
                break;
            }
            else
            {
                // 경고 메시지 출력
                output_buffer_.str("");
                output_buffer_.clear();
                output_buffer_ << "[WARNING] Invalid input. Please press 'p' or 'd'.\n";
                std::cout << output_buffer_.str() << std::flush; // 즉시 출력
            }
        }
        ros::Duration(0.1).sleep(); // ROS 노드가 응답을 유지하도록 100ms 대기
    }
    break;
}


    case 14: //I
    joint_angle.clear();
    joint_angle.push_back( -0.063);
    joint_angle.push_back( 0.061);
    joint_angle.push_back( -1.488);
    joint_angle.push_back( -0.012);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;


    case 15: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;


    case 16: //R
    joint_angle.clear();
    joint_angle.push_back( -0.015);
    joint_angle.push_back( 0.030);
    joint_angle.push_back( 0.779);
    joint_angle.push_back( 1.759);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;


    case 17: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;


    case 18: //임시
    joint_angle.clear();
    joint_angle.push_back( -0.015);
    joint_angle.push_back( -0.100);
    joint_angle.push_back( 0.579);
    joint_angle.push_back( 1.759);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;


    case 19: //A
    joint_angle.clear();
    joint_angle.push_back( -0.032);
    joint_angle.push_back( 0.078);
    joint_angle.push_back( 0.894);
    joint_angle.push_back( 0.021);
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


    case 21: //S
    joint_angle.clear();
    joint_angle.push_back( 0.000);
    joint_angle.push_back( -1.085);
    joint_angle.push_back( 0.508);
    joint_angle.push_back( -0.341);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;


    case 22: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;


    case 23: //C
    joint_angle.clear();
    joint_angle.push_back( -0.031);
    joint_angle.push_back( -1.235);
    joint_angle.push_back( 0.032);
    joint_angle.push_back( 1.119);
    setJointSpacePath(joint_name_, joint_angle, 1);
    demo_count_++;
    break;


    case 24: // wait & place
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.clear();
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_++;
    break;


    case 25: // home pose
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
    printf("HelloTello Final Demonstration!\n");
    printf("Drones and Robotics\n");
    printf("Andrew Jaeyong Choi Prof.\n");
    printf("-----------------------------\n");

    printf("q : Home Pose\n");
    printf("w : Start Pick and Place Demo\n");
    printf("e : Stop Pick and Place Demo\n");
    printf("-----------------------------\n");

if (mode_state_ == DEMO_START)
{
    switch (demo_count_)
    {
    case 0:
        printf("\033[32m[INFO] Moving to Home Pose...\033[0m\n");
        break;
    case 1:
        printf("\033[32m[INFO] Moving to Initial Pose...\033[0m\n");
        break;
    case 2:
        printf("\033[32m[INFO] Preparing Gripper (Opening)...\033[0m\n");
        break;
    case 3:
        printf("\033[32m[INPUT] Waiting for Pick Marker ID Input (0-17):\033[0m\n");
        break;
    case 4:
        printf("\033[32m[INFO] Searching for AR Marker ID: %d for Picking...\033[0m\n", pick_marker_id_);
        break;
    case 5:
        printf("\033[32m[INFO] Moving to AR Marker ID: %d for Picking...\033[0m\n", pick_marker_id_);
        break;
    case 6:
        printf("\033[32m[INFO] Gripping Object...\033[0m\n");
        break;
    case 7:
        printf("\033[32m[INFO] Returning to Initial Pose...\033[0m\n");
        break;
    case 8:
        printf("\033[32m[INPUT] Waiting for Place Marker ID Input (0-17):\033[0m\n");
        break;
    case 9:
        printf("\033[32m[INFO] Searching for AR Marker ID: %d for Placing...\033[0m\n", place_marker_id_);
        break;
    case 10:
        printf("\033[32m[INFO] Moving to AR Marker ID: %d for Placing...\033[0m\n", place_marker_id_);
        break;
    case 11:
        printf("\033[32m[INFO] Releasing Object (Opening Gripper)...\033[0m\n");
        break;
    case 12:
        printf("\033[32m[INFO] Moving Up After Placing the Object...\033[0m\n");
        break;
    case 13:
        printf("\033[32m[CHOICE] Press 'p' to Pick Another Object or 'd' to End Demo:\033[0m\n");
        break;
    case 14:
        printf("\033[32m[INFO] Moving to Pose I...\033[0m\n");
        break;
    case 15:
        printf("\033[32m[INFO] I\033[0m\n");
        break;
    case 16:
        printf("\033[32m[INFO] Moving to Pose R...\033[0m\n");
        break;
    case 17:
        printf("\033[32m[INFO] R\033[0m\n");
        break;
    case 18:
        printf("\033[32m[INFO] Moving to Pose A...\033[0m\n");
        break;
    case 19:
        printf("\033[32m[[INFO] Moving to Pose A...\033[0m\n");
        break;
    case 20:
        printf("\033[32mA\033[0m\n");
        break;
    case 21:
        printf("\033[32m[INFO] Moving to Pose S...\033[0m\n");
        break;
    case 22:
        printf("\033[32mS\033[0m\n");
        break;
    case 23:
        printf("\033[32m[INFO] Moving to Pose C...\033[0m\n");
        break;
    case 24:
        printf("\033[32mC\033[0m\n");
        break;
    case 25:
        printf("\033[32m[INFO] Finalizing Demo. Returning to Start Position...\033[0m\n");
        break;
    default:
        printf("\033[32m[WARNING] Unknown Demo State Detected...\033[0m\n");
        break;
    }
}

    else if (mode_state_ == DEMO_STOP)
    {
        printf("[INFO] Demo Stopped.\n");
    }

    else if (mode_state_ == HOME_POSE)
    {
        printf("[INFO] Moving to Home Pose...\n");
    }

    printf("-----------------------------\n");
    printf("Present Joint Angles: J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
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
        printf("Detected AR Markers:\n");
        for (const auto &marker : ar_marker_pose)
        {
            printf("ID: %d --> X: %.3lf\tY: %.3lf\tZ: %.3lf\n",
                   marker.id,
                   marker.position[0],
                   marker.position[1],
                   marker.position[2]);
        }
    }

    else
    {
        printf("[INFO] No AR Markers Detected. Waiting for Input...\n");
    }

    // demoSequence()의 출력 추가
    if (!output_buffer_.str().empty())
    {
    printf("-----------------------------\n");
    printf("Present Joint Angles: J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
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
        printf("Detected AR Markers:\n");
        for (const auto &marker : ar_marker_pose)
        {
            printf("ID: %d --> X: %.3lf\tY: %.3lf\tZ: %.3lf\n",
                   marker.id,
                   marker.position[0],
                   marker.position[1],
                   marker.position[2]);
        }
    }

        printf("\033[32m\n--- DemoSequence Output ---\033[0m\n"); // 초록색으로 출력
        printf("\033[32m%s\033[0m", output_buffer_.str().c_str()); // 초록색으로 출력
        output_buffer_.str("");  // 버퍼 초기화
        output_buffer_.clear();
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
  ros::init(argc, argv, "open_manipulator_final");
  ros::NodeHandle node_handle("");

  OpenManipulatorPickandPlace open_manipulator_pick_and_place;

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/, &OpenManipulatorPickandPlace::publishCallback, &open_manipulator_pick_and_place);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}