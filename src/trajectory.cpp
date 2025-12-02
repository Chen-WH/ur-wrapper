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
#include <vector>
#include <chrono>

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
    target_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_target", 10, std::bind(&UrMpcNode::target_callback, this, std::placeholders::_1));

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
    counter_ = 0;
    current_trajectory_index_ = 0;
    trajectory_received_ = true;

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
      else
      {
        break;
      }
    }
    current_trajectory_index_ = best_i + 1;
  }

  void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (msg->position.size() >= 6)
    {
      for (size_t i = 0; i < 6; ++i)
        position_target_[i] = msg->position[i];
    }
  }

  void driverLoop()
  {
    // Start RTDE communication
    robot_->getUrDriver()->startRTDECommunication();

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
        std::this_thread::sleep_for(10ms);
        continue;
      }

      // publish joint state
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.position.resize(6);
      for (size_t i = 0; i < 6; ++i)
        js.position[i] = actual_q[i];
      joint_state_pub_->publish(js);

      // decide command to send
      urcl::vector6d_t joint_cmd;
      {
        std::lock_guard<std::mutex> lk(mutex_);
        if (trajectory_received_)
        {
          if (current_trajectory_index_ < trajectory_.points.size())
          {
            const auto &pt = trajectory_.points[current_trajectory_index_++];
            for (size_t i = 0; i < 6; ++i) position_command_[i] = pt.positions[i];
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "Trajectory exhausted without new trajectory");
          }
        }
        else
        {
          for (size_t i = 0; i < 6; ++i) position_command_[i] = js.position[i];
        }
        for (size_t i = 0; i < 6; ++i) joint_cmd[i] = position_command_[i];
      }

      // send joint command using UR client library
      bool ret = robot_->getUrDriver()->writeJointCommand(joint_cmd, urcl::comm::ControlMode::MODE_SERVOJ,
                                                          urcl::RobotReceiveTimeout::millisec(100));
      if (!ret)
      {
        RCLCPP_ERROR(this->get_logger(), "Could not send joint command. Is the robot in remote control?");
      }

      // small sleep to avoid busy loop
      std::this_thread::sleep_for(8ms);
    }
  }

  // members
  std::unique_ptr<urcl::ExampleRobotWrapper> robot_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;

  std::thread driver_thread_;
  std::mutex mutex_;

  trajectory_msgs::msg::JointTrajectory trajectory_;
  bool trajectory_received_ = false;
  size_t current_trajectory_index_ = 0;
  int counter_ = 0;

  std::array<double,6> position_target_ = {0, -M_PI_2, 0, -M_PI_2, 0, 0};
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
