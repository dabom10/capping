import rclpy
import DR_init
from std_msgs.msg import String, Int32
import time

# ===============================
# Global Parameters
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY = 500
ACC = 500
SAFE_Z_OFFSET = 100

J_READY = [0, 0, 90, 0, 90, 0]
J_MIX_1 = [0, 10, 80,  45,  45,  90]
J_MIX_2 = [0, 10, 80, -45, -45, -90]

POS_PICK = [322.7, 8.10, 87.41, 19.83, -179.47, 19.28]
POS_AIR  = [261.31, -343.97, 400.04, 114.58, -179.02, 115.44]

POS_PLACE = [
    [251.1, -386.71, 215.94, 92.46, 162.31, 92.86],
    [399.69, -381.66, 186.38, 91.56, 162.08, 91.85]
]

# ===============================
# Publish Helper
# ===============================
def publish_status(text: str, pub):
    msg = String()
    msg.data = text
    pub.publish(msg)
    print(f"[STATUS] {text}")

def publish_progress(percent: int, pub):
    msg = Int32()
    msg.data = percent
    pub.publish(msg)
    print(f"[PROGRESS] {percent}%")

# ===============================
# Main
# ===============================
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("shake_pick_place_node", namespace=ROBOT_ID)

    DR_init.__dsr__node  = node
    DR_init.__dsr__id    = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    from DSR_ROBOT2 import (
        posx,
        movej, movel, move_periodic,
        set_tool, set_tcp, set_digital_output,
        wait, DR_BASE, DR_TOOL, DR_MV_MOD_REL,
        ON, OFF
    )

    # ===============================
    # Publisher
    # ===============================
    status_pub  = node.create_publisher(String, "status", 10)
    process_pub = node.create_publisher(Int32, "process", 10)

    # ===============================
    # IO Control
    # ===============================
    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(1.0)

    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.0)

    # ===============================
    # Pick
    # ===============================
    def pick(p):
        print("[STEP] Pick")

        release()
        movej(J_READY, vel=120, acc=120)

        pick_up = posx(0, 0, SAFE_Z_OFFSET, 0, 0, 0)
        pick_target = posx(*p)

        movel(pick_target, vel=80, acc=80)
        grip()
        movel(pick_up, vel=80, acc=80, mod=DR_MV_MOD_REL)

    # ===============================
    # Shaking
    # ===============================
    def shaking(cycle=2):
        print("[STEP] Shaking")

        for i in range(cycle):
            print(f"  - Cycle {i+1}/{cycle}")

            movej(J_READY, vel=VELOCITY, acc=ACC)
            movej(J_MIX_1, vel=VELOCITY, acc=ACC)
            movej(J_MIX_2, vel=VELOCITY, acc=ACC)
            movej(J_READY, vel=VELOCITY, acc=ACC)

            move_periodic(
                amp=[0, 0, 40, 0, 0, 120],
                period=0.4,
                atime=0.05,
                repeat=1,
                ref=DR_BASE
            )

            move_periodic(
                amp=[0, 0, 0, 40, 40, 0],
                period=0.35,
                atime=0.05,
                repeat=1,
                ref=DR_TOOL
            )

    # ===============================
    # Place
    # ===============================
    def place(p):
        print("[STEP] Place")

        place_up = posx(0, 0, SAFE_Z_OFFSET, 0, 0, 0)
        place_target = posx(*p)

        movel(place_up, vel=80, acc=80, mod=DR_MV_MOD_REL)
        movel(POS_AIR, vel=80, acc=80)
        movel(place_target, vel=80, acc=80)

        release()
        movel(place_up, vel=80, acc=80, mod=DR_MV_MOD_REL)

    # ===============================
    # Process Flow (Progress 관리)
    # ===============================
    try:
        set_tool("Tool Weight")
        set_tcp("GripperDA_v2")

        publish_status("start", status_pub)
        publish_progress(0, process_pub)

        # Pick (30%)
        pick(POS_PICK)
        publish_status("pick_done", status_pub)
        publish_progress(30, process_pub)

        # Shaking (60%)
        shaking(cycle=2)
        publish_status("shake_done", status_pub)
        publish_progress(60, process_pub)

        # Place (90%)
        place(POS_PLACE[1])
        publish_status("place_done", status_pub)
        publish_progress(90, process_pub)

        # Finish (100%)
        movel(POS_AIR, vel=120, acc=120)
        movej(J_READY, vel=120, acc=120)

        publish_status("completed", status_pub)
        publish_progress(100, process_pub)

        print("[DONE] All Process Completed")

    except Exception as e:
        publish_status(f"error: {e}", status_pub)
        print(f"[ERROR] {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()