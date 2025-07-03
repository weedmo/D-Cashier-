# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import sys
from robot_api_control.constants import *
# ───── Doosan API ────────────────────────────────────────────────────
# Doosan API
import DR_init
from robot_api_control.onrobot import RG

# ─── DSR Gripper ───────────────────────────
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("main_controller_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        movej,
        movel,
        mwait,
        get_tool,
        get_tcp,
        DR_BASE,
        get_current_posx,
        get_external_torque,
        
        # force control
        task_compliance_ctrl,
        release_force,
        check_force_condition,
        DR_FC_MOD_REL,
        set_desired_force,
        release_compliance_ctrl,
        DR_AXIS_Z
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()
    
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("move_block", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            # grip
            set_digital_output,
            get_tool,
            get_tcp,
            wait,
            movej,

        )

        from DR_common2 import posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    home = posj(0, 0, 90.0, 0, 90, 0)
    
    # ====================
    # grip
    ON, OFF = 1, 0

    def grip():
        set_digital_output(1, OFF)
        wait(1.0)

    def release():
        set_digital_output(0, OFF)
        wait(1.0)

    while rclpy.ok():
        tool_name = get_tool()
        tcp_name = get_tcp()

        print(f"Tool: {tool_name}, TCP: {tcp_name}")
        if tool_name == "" or tcp_name == "":
            rclpy.shutdown()
            return
        release()
        movej(HOME, vel=VELOCITY, acc=ACC)
        break  # ← 반복을 원한다면 이 줄을 제거

if __name__ == "__main__":
    main()