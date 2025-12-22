import rtde_receive
import rtde_control
from math import pi

ROBOT_IP = "192.168.125.6"

rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)

actual_q = rtde_r.getActualQ()
print(actual_q)
rtde_c.moveJ([-0.5, 0.5, -pi/2-0.5, -pi/2, 0, 0], 0.5, 0.5)
actual_q = rtde_r.getActualQ()
print(actual_q)