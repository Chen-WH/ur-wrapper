#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ur_client_library/example_robot_wrapper.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

class UrMpcNode : public rclcpp::Node
{
public:
  UrMpcNode()
  : Node("ur_mpc_node")
  {
    this->declare_parameter<std::string>("robot_ip", "192.168.125.6");
    std::string robot_ip = this->get_parameter("robot_ip").as_string();

    // ROS publishers/subscriptions
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_state", 10);
    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory", 10, std::bind(&UrMpcNode::trajectory_callback, this, std::placeholders::_1));
    // Resolve resource files from the installed package share directory so the
    std::string pkg_share = ament_index_cpp::get_package_share_directory("ur-wrapper");
    std::string script_file = pkg_share + "/resources/external_control.urscript";
    std::string output_recipe = pkg_share + "/resources/rtde_output_recipe.txt";
    std::string input_recipe = pkg_share + "/resources/rtde_input_recipe.txt";

    bool headless_mode = true;
    robot_ = std::make_unique<urcl::ExampleRobotWrapper>(robot_ip, output_recipe, input_recipe, headless_mode,
                               "external_control.urp", script_file);
    if (!robot_->isHealthy())
    {
      RCLCPP_ERROR(this->get_logger(), "Robot wrapper initialization failed");
      throw std::runtime_error("Robot init failed");
    }

    trajectory_received_ = false;
    current_trajectory_index_ = 0;

    // start RTDE comm and driver loop in a background thread
    driver_thread_ = std::thread(&UrMpcNode::driverLoop, this);
  }

  ~UrMpcNode()
  {
    if (robot_ && robot_->getUrDriver())
    {
      robot_->getUrDriver()->stopControl();
    }
    if (driver_thread_.joinable())
      driver_thread_.join();
  }

private:
  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    trajectory_ = *msg;
    current_trajectory_index_ = 0;

    // choose starting index similar to python logic: find first point close to current target
    double min_diff = std::numeric_limits<double>::infinity();
    size_t best_i = 0;
    for (size_t i = 0; i < trajectory_.points.size(); ++i)
    {
      double diff = 0.0;
      const auto &p = trajectory_.points[i];
      for (size_t j = 0; j < p.positions.size(); ++j)
        diff += std::abs(p.positions[j] - position_command_[j]);
      if (diff < min_diff)
      {
        min_diff = diff;
        best_i = i;
      }
    }
    current_trajectory_index_ = best_i + 1;
    trajectory_received_ = true;
  }

  void driverLoop()
  {
    // Start RTDE communication
    robot_->getUrDriver()->startRTDECommunication();

    // Write CSV header
    std::stringstream ss;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    ss << "data_" << std::put_time(std::localtime(&in_time_t), "%H%M%S") << ".csv";
    std::ofstream csv_file(ss.str());
    csv_file << "cmd_pos0,cmd_pos1,cmd_pos2,cmd_pos3,cmd_pos4,cmd_pos5,";
    csv_file << "cmd_vel0,cmd_vel1,cmd_vel2,cmd_vel3,cmd_vel4,cmd_vel5,";
    csv_file << "cmd_tor0,cmd_tor1,cmd_tor2,cmd_tor3,cmd_tor4,cmd_tor5,";
    csv_file << "state_pos0,state_pos1,state_pos2,state_pos3,state_pos4,state_pos5,";
    csv_file << "state_vel0,state_vel1,state_vel2,state_vel3,state_vel4,state_vel5,";
    csv_file << "state_cur0,state_cur1,state_cur2,state_cur3,state_cur4,state_cur5\n";

    while (rclcpp::ok())
    {
      // Get latest package (blocks with timeout inside driver)
      auto data_pkg = robot_->getUrDriver()->getDataPackage();
      if (!data_pkg)
      {
        RCLCPP_WARN(this->get_logger(), "Could not get fresh data package from robot");
        std::this_thread::sleep_for(10ms);
        continue;
      }

      urcl::vector6d_t actual_q;
      if (!data_pkg->getData("actual_q", actual_q))
      {
        RCLCPP_ERROR(this->get_logger(), "Received data package has no 'actual_q'");
        std::this_thread::sleep_for(8ms);
        continue;
      }

      // Get joint velocities
      urcl::vector6d_t actual_qd;
      if (!data_pkg->getData("actual_qd", actual_qd))
      {
        RCLCPP_ERROR(this->get_logger(), "Received data package has no 'actual_qd'");
        std::this_thread::sleep_for(8ms);
        continue;
      }

      // Get joint currents
      urcl::vector6d_t actual_current;
      if (!data_pkg->getData("actual_current", actual_current))
      {
        RCLCPP_ERROR(this->get_logger(), "Received data package has no 'actual_current'");
        std::this_thread::sleep_for(8ms);
        continue;
      }

      // publish joint state with position, velocity, and effort (current)
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.position.resize(6);
      js.velocity.resize(6);
      js.effort.resize(6);
      for (size_t i = 0; i < 6; ++i)
      {
        js.position[i] = actual_q[i];
        js.velocity[i] = actual_qd[i];
        js.effort[i] = actual_current[i];
      }
      joint_state_pub_->publish(js);

      // decide command to send
      urcl::vector6d_t joint_cmd;
      std::lock_guard<std::mutex> lk(mutex_);
      if (trajectory_received_)
      {
        if (current_trajectory_index_ < trajectory_.points.size()) {
          const auto &pt = trajectory_.points[current_trajectory_index_++];
          for (size_t i = 0; i < 6; ++i) position_command_[i] = pt.positions[i];

          for (size_t j = 0; j < 6; ++j) csv_file << pt.positions[j] << ",";
          for (size_t j = 0; j < 6; ++j) csv_file << pt.velocities[j] << ",";
          for (size_t j = 0; j < 6; ++j) csv_file << pt.effort[j] << ",";
          for (size_t j = 0; j < 6; ++j) csv_file << js.position[j] << ",";
          for (size_t j = 0; j < 6; ++j) csv_file << js.velocity[j] << ",";
          for (size_t j = 0; j < 6; ++j) csv_file << js.effort[j] << (j < 5 ? "," : "\n");
        } else {
          trajectory_received_ = false;
          for (size_t i = 0; i < 6; ++i) position_command_[i] = js.position[i];
          RCLCPP_ERROR(this->get_logger(), "Trajectory exhausted without new trajectory");
        }
      } else {
        for (size_t i = 0; i < 6; ++i) position_command_[i] = js.position[i];
      }

      // send joint command using UR client library
      for (size_t i = 0; i < 6; ++i) joint_cmd[i] = position_command_[i];
      bool ret = robot_->getUrDriver()->writeJointCommand(joint_cmd, urcl::comm::ControlMode::MODE_SERVOJ,
                                                          urcl::RobotReceiveTimeout::millisec(100));
      if (!ret)
      {
        RCLCPP_ERROR(this->get_logger(), "Could not send joint command. Is the robot in remote control?");
      }

      // small sleep to avoid busy loop
      std::this_thread::sleep_for(8ms);
    }

    csv_file.close();
  }

  // members
  std::unique_ptr<urcl::ExampleRobotWrapper> robot_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;

  std::thread driver_thread_;
  std::mutex mutex_;

  trajectory_msgs::msg::JointTrajectory trajectory_;
  bool trajectory_received_ = false;
  size_t current_trajectory_index_ = 0;

  std::array<double,6> position_command_ = {0, -M_PI_2, 0, -M_PI_2, 0, 0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<UrMpcNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception in UrMpcNode: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
