#include <ur_client_library/example_robot_wrapper.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "192.168.125.6";

std::unique_ptr<ExampleRobotWrapper> g_my_robot;

static bool parse_joint_line(const std::string& line, vector6d_t& out)
{
  std::istringstream iss(line);
  double v;
  std::vector<double> vals;
  while (iss >> v)
  {
    vals.push_back(v);
  }
  if (vals.size() != 6)
    return false;
  for (size_t i = 0; i < 6; ++i)
    out[i] = vals[i];
  return true;
}

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::INFO);

  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

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

  std::cout << "请输入目标关节角 (6 个，弧度)，用空格分隔，例如: 0 -1.57 0 -1.57 0 0\n";
  std::cout << "直接回车则使用默认角: 0 -1.57 0 -1.57 0 0。输入 q 退出。\n> ";
  std::string line;
  if (!std::getline(std::cin, line))
    return 0;
  if (!line.empty() && (line[0] == 'q' || line[0] == 'Q'))
    return 0;

  vector6d_t target_joint = {0, 0, 0, 0, 0, 0};
  if (line.empty())
  {
    // 使用用户要求的默认初始关节角
    target_joint = {0, -1.57, 0, -1.57, 0, 0};
    std::cout << "使用默认目标关节角: 0 -1.57 0 -1.57 0 0\n";
  }
  else
  {
    if (!parse_joint_line(line, target_joint))
    {
      std::cerr << "解析失败：请确保输入 6 个数值（以空格分隔）。\n";
      return 1;
    }
  }

  std::cout << "请输入插补时长（秒），回车默认 5.0s: ";
  double duration = 5.0;
  if (std::getline(std::cin, line) && !line.empty())
  {
    try
    {
      duration = std::stod(line);
      if (duration <= 0)
        duration = 5.0;
    }
    catch (...) { duration = 5.0; }
  }

  // 启动 RTDE 通信（例子里要求先 startRTDECommunication）
  g_my_robot->getUrDriver()->startRTDECommunication();

  // 读取一次初始关节角
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_robot->getUrDriver()->getDataPackage();
  if (!data_pkg)
  {
    URCL_LOG_ERROR("无法获取 RTDE 数据包，请检查连接和外部控制脚本。");
    return 1;
  }

  vector6d_t start_joint;
  if (!data_pkg->getData("actual_q", start_joint))
  {
    URCL_LOG_ERROR("RTDE 返回的数据中没有 'actual_q' 字段。请检查 OUTPUT_RECIPE 配置。");
    return 1;
  }

  // 获取控制频率以决定步数
  double freq = static_cast<double>(g_my_robot->getUrDriver()->getControlFrequency());
  if (freq <= 0)
    freq = 125.0; // 兜底值
  size_t steps = static_cast<size_t>(std::ceil(freq * duration));
  if (steps < 1)
    steps = 1;

  URCL_LOG_INFO("开始插补: 时长 %.3f s, 频率 %.1f Hz, 步数 %zu", duration, freq, steps);

  // 线性插补（从 start_joint 到 target_joint）
  for (size_t i = 1; i <= steps; ++i)
  {
    // 阻塞读取 RTDE 包以跟随机器人周期
    data_pkg = g_my_robot->getUrDriver()->getDataPackage();
    if (!data_pkg)
    {
      URCL_LOG_WARN("无法获取数据包，提前退出插补。\n");
      break;
    }

    double alpha = static_cast<double>(i) / static_cast<double>(steps);
    vector6d_t cmd;
    for (size_t j = 0; j < 6; ++j)
      cmd[j] = start_joint[j] + alpha * (target_joint[j] - start_joint[j]);

    bool ret = g_my_robot->getUrDriver()->writeJointCommand(cmd, comm::ControlMode::MODE_SERVOJ,
                                                           RobotReceiveTimeout::millisec(100));
    if (!ret)
    {
      URCL_LOG_ERROR("发送关节命令失败，请检查机器人是否处于远程控制模式。\n");
      break;
    }
  }

  g_my_robot->getUrDriver()->stopControl();
  URCL_LOG_INFO("回零完成");
  return 0;
}