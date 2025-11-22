#include <ur_client_library/example_robot_wrapper.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <memory>
#include <unistd.h>
#include <cmath>
#include <limits>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "192.168.125.6";
std::unique_ptr<ExampleRobotWrapper> g_my_robot;
trajectory_msgs::msg::JointTrajectory traj_msg;
vector6d_t joint_state = {0, 0, 0, 0, 0, 0};
size_t current_trajectory_index = 0;
bool trajectory_received = false;

void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    traj_msg = *msg;
    current_trajectory_index = 0;
    trajectory_received = true; // 标记轨迹已接收

    // Find the point with the smallest joint angle difference
    double min_diff = std::numeric_limits<double>::max();
    for (size_t i = 0; i < traj_msg.points.size(); ++i) {
        double diff = 0.0;
        for (size_t j = 0; j < 6; ++j) {
            diff += std::abs(traj_msg.points[i].positions[j] - joint_state[j]);
        }
        if (diff < min_diff) {
            min_diff = diff;
            current_trajectory_index = i + 1;
        } else {
            break;
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mpc_publisher");

    auto trajectory_subscriber = node->create_subscription<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory", 1, trajectory_callback);
    auto joint_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>("/current_states", 1);

    urcl::setLogLevel(urcl::LogLevel::INFO);
    std::string robot_ip = DEFAULT_ROBOT_IP;

    bool headless_mode = true;
    // Resolve resource files from the installed package share directory so the
    // binary can find them regardless of working directory.
    std::string pkg_share = ament_index_cpp::get_package_share_directory("ur-wrapper");
    std::string script_file = pkg_share + "/resources/external_control.urscript";
    std::string output_recipe = pkg_share + "/resources/rtde_output_recipe.txt";
    std::string input_recipe = pkg_share + "/resources/rtde_input_recipe.txt";

    g_my_robot = std::make_unique<ExampleRobotWrapper>(robot_ip, output_recipe, input_recipe, headless_mode,
                                                     "external_control.urp", script_file);
    if (!g_my_robot->isHealthy())
    {
        URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
        return 1;
    }
    // --------------- INITIALIZATION END -------------------
    
    sensor_msgs::msg::JointState joint_state_msg;
    JointValue joint_state, joint_state_old, joint_velocity;
    robot.get_joint_position(&joint_state);
    joint_command = joint_state;
    robot.servo_move_enable(TRUE);
    sleep(2);

    std::stringstream ss;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    ss << "data_" << std::put_time(std::localtime(&in_time_t), "%H%M%S") << ".csv";
    std::ofstream csv_file(ss.str());

    // Write CSV header
    csv_file << "command_position,command_velocity,command_torque,state_position,state_velocity,state_current,\n";

    rclcpp::Rate loop_rate(62.5);
    while (rclcpp::ok()) {
        std::cout << "Main Loop :" << std::endl;
        joint_state_old = joint_state;
        robot.get_joint_position(&joint_state);
        robot.get_robot_status(&robstatus);

        // Calculate joint velocities
        for (int i = 0; i < 6; ++i) {
            joint_velocity.jVal[i] = (joint_state.jVal[i] - joint_state_old.jVal[i]) / 0.016;
        }

        // Publish joint states
        joint_state_msg.header.stamp = node->now();
        joint_state_msg.position = {joint_state.jVal[0], joint_state.jVal[1], joint_state.jVal[2], joint_state.jVal[3], joint_state.jVal[4], joint_state.jVal[5]};
        joint_state_msg.velocity = {joint_velocity.jVal[0], joint_velocity.jVal[1], joint_velocity.jVal[2], joint_velocity.jVal[3], joint_velocity.jVal[4], joint_velocity.jVal[5]};
        joint_state_publisher->publish(joint_state_msg);

        if (trajectory_received) {
            if (current_trajectory_index < traj_msg.points.size()) {
                for (int j = 0; j < 6; j++) {
                    joint_command.jVal[j] = traj_msg.points[current_trajectory_index].positions[j];
                }
                robot.servo_j(&joint_command, ABS, 2);

                // Record joint_command to CSV
                for (int j = 0; j < 6; j++) {
                    csv_file << traj_msg.points[current_trajectory_index].positions[j] << "," 
                             << traj_msg.points[current_trajectory_index].velocities[j] << "," 
                             << traj_msg.points[current_trajectory_index].effort[j] << "," 
                             << joint_state.jVal[j] << "," 
                             << joint_velocity.jVal[j] << "," 
                             << robstatus.robot_monitor_data.jointMonitorData[j].instCurrent;
                    if (j < 5) csv_file << ",";
                }
                csv_file << "\n";
                ++current_trajectory_index;
            } else {
                std::cout << "Trajectory completed" << std::endl;
                break;
            }
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    sleep(2);
    robot.servo_move_enable(FALSE);
    sleep(1);
    robot.login_out();

    csv_file.close();
    rclcpp::shutdown();
    return 0;
}
