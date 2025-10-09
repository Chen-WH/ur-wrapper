#include <iostream>
#include <ur_client_library/ur/dashboard_client.h>

int main(int argc, char* argv[])
{
  urcl::DashboardClient my_client("192.168.125.6");
  bool connected = my_client.connect();
  if (connected)
  {
    std::string answer = my_client.sendAndReceive("PolyscopeVersion\n");
    std::cout << answer << std::endl;
    my_client.disconnect();
  }
  return 0;
}