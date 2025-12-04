#!/usr/bin/env python3
import rospy
import sys
import termios
import tty
import select
import time
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Int8

msg = """
x: enter setting x wrench mode
y: enter setting y wrench mode
z: enter setting z wrench mode
q: reset mode
[: +0.1
]: -0.1
p: publish
@: publish flags (true, pos->vel)
,: publish send_feedforward_switch_flag (false)
.: publish attaching_flag (false)
/: publish flags (false, vel->pos)
"""

def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

def main():
    if not sys.stdin.isatty():
        print("!run in terminal!")
        return

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    is_flag_published = False

    rospy.init_node("set_desire_wrench_node")
    pub = rospy.Publisher("/gimbalrotor/desire_wrench", WrenchStamped, queue_size=1)
    pub_flag_1 = rospy.Publisher("/gimbalrotor/send_feedforward_switch_flag", Bool, queue_size=1)
    pub_flag_2 = rospy.Publisher("/gimbalrotor/attaching_flag", Bool, queue_size=1)
    pub_flag_3 = rospy.Publisher("/gimbalrotor/xyz_wrench_control_flag", Bool, queue_size=1)
    pub_x_control_mode = rospy.Publisher("/gimbalrotor/teleop_command/x_ctrl_mode", Int8, queue_size=1)
    pub_y_control_mode = rospy.Publisher("/gimbalrotor/teleop_command/y_ctrl_mode", Int8, queue_size=1)
    pub_z_control_mode = rospy.Publisher("/gimbalrotor/teleop_command/z_ctrl_mode", Int8, queue_size=1)
    
    print(msg)
    print("waiting key input... (Ctrl-C to quit)")

    mode = 0
    msg_w = WrenchStamped()
    msg_f1 = Bool()
    msg_f2 = Bool()
    msg_f3 = Bool()
    msg_x_ctrl = Int8()
    msg_y_ctrl = Int8()
    msg_z_ctrl = Int8()

    rate = rospy.Rate(50)

    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key is None:
                rate.sleep()
                continue

            if key == '\x03':
                print("exiting.")
                break

            if key == "@":
                if mode == 0:
                    msg_f1.data = True
                    msg_f2.data = True
                    msg_f3.data = True
                    # pub_flag_1.publish(msg_f1)
                    # pub_flag_2.publish(msg_f2)
                    # print("publish: send_feedforward_switch_flag, attaching_flag")
                    pub_flag_3.publish(msg_f3)
                    print("publish: xyz_wrench_control_flag")
                if mode == 1:
                    msg_x_ctrl.data = 1
                    pub_x_control_mode.publish(msg_x_ctrl)
                    print("x control -> vel")
                if mode == 2:
                    msg_y_ctrl.data = 1
                    pub_y_control_mode.publish(msg_y_ctrl)
                    print("y control -> vel")
                if mode == 3:
                    msg_z_ctrl.data = 1
                    pub_z_control_mode.publish(msg_z_ctrl)
                    print("z control -> vel")

            if key == ",":
                msg_f1.data = False
                pub_flag_1.publish(msg_f1)
                print("publish: send_feedforward_switch_flag->False")

            if key == ".":
                msg_f2.data = False
                pub_flag_2.publish(msg_f2)
                print("publish: attaching_flag->False")

            if key == "/":
                if mode == 0:
                    msg_f3.data = False
                    pub_flag_3.publish(msg_f3)
                    print("publish: xyz_wrench_control_flag->False")
                if mode == 1:
                    msg_x_ctrl.data = 0
                    pub_x_control_mode.publish(msg_x_ctrl)
                    print("x control -> pos")
                if mode == 2:
                    msg_y_ctrl.data = 0
                    pub_y_control_mode.publish(msg_y_ctrl)
                    print("y control -> pos")
                if mode == 3:
                    msg_z_ctrl.data = 1
                    pub_z_control_mode.publish(msg_z_ctrl)
                    print("z control -> pos")
                    
            if key == "x":
                mode = 1
                print("mode -> x")
            elif key == "y":
                mode = 2
                print("mode -> y")
            elif key == "z":
                mode = 3
                print("mode -> z")
            elif key == "q":
                mode = 0
                print("mode -> None")

            elif key == "[":
                if mode == 1:
                    msg_w.wrench.force.x += 0.1
                elif mode == 2:
                    msg_w.wrench.force.y += 0.1
                elif mode == 3:
                    msg_w.wrench.force.z += 0.1
                print(f"{msg_w.wrench.force.x:.1f}, {msg_w.wrench.force.y:.1f}, {msg_w.wrench.force.z:.1f}")
                    
            elif key == "]":
                if mode == 1:
                    msg_w.wrench.force.x -= 0.1
                elif mode == 2:
                    msg_w.wrench.force.y -= 0.1
                elif mode == 3:
                    msg_w.wrench.force.z -= 0.1
                print(f"{msg_w.wrench.force.x:.1f}, {msg_w.wrench.force.y:.1f}, {msg_w.wrench.force.z:.1f}")

            # publish
            elif key == "p":
                pub.publish(msg_w)
                rospy.loginfo("published: %s", msg_w.wrench.force)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("terminal restored. bye.")

if __name__ == "__main__":
    main()
