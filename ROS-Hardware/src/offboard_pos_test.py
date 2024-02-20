# import rospy
# from geometry_msgs.msg import PoseStamped
# from mavros_msgs.msg import State
# from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
# #from pynput import keyboard as kb
# from std_msgs.msg import String
# import signal
# import sys

# current_state = State()

# def state_cb(msg):
#     global current_state
#     current_state = msg

# def position_cb(msg):
#     global current_position
#     current_position = msg

# def set_manual_mode():
#     manual_set_mode = SetModeRequest()
#     manual_set_mode.custom_mode = 'MANUAL'
#     try:
#         resp = set_mode_client.call(manual_set_mode)
#         if resp.mode_sent:
#             rospy.loginfo("Manual mode enabled")
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s", e)

# def signal_handler(sig, frame):
#     rospy.loginfo('KeyboardInterrupt (ID: {}) has been caught. Changing to manual mode...'.format(sig))
#     set_manual_mode()
#     sys.exit(0)

# def increase_altitude():
#     global pose
#     pose.pose.position.z += 1
#     local_pos_pub.publish(pose)

# def decrease_altitude():
#     global pose
#     pose.pose.position.z -= 1
#     local_pos_pub.publish(pose)

# def move_left():
#     global pose
#     pose.pose.position.x -= 1
#     local_pos_pub.publish(pose)

# def move_right():
#     global pose
#     pose.pose.position.x += 1
#     local_pos_pub.publish(pose)

# def on_press(key):
    
#     try:
#         if key.data == 'w':
#             print("key pressed")
#             increase_altitude()
#         elif key.data == 's':
#             decrease_altitude()
#         elif key.data == 'a':
#             move_left()
#         elif key.data == 'd':
#             move_right()
#     except AttributeError:
#         pass  # Handle special keys here if necessary

# if __name__ == "__main__":
#     rospy.init_node("offb_node_py")
#     signal.signal(signal.SIGINT, signal_handler)  # Register the signal handler

#     state_sub = rospy.Subscriber("/mavros/state", State, callback=state_cb)
#     local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
#     local_pos_sub = rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, callback=position_cb)
    
#     rospy.wait_for_service("/mavros/cmd/arming")
#     arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
#     rospy.wait_for_service("/mavros/set_mode")
#     set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
#     middleware_sub = rospy.Subscriber("/middleware/control", String, on_press)

#     rate = rospy.Rate(20)  # Setpoint publishing MUST be faster than 2Hz

#     # Wait for FCU connection
#     while not rospy.is_shutdown() and not current_state.connected:
#         rate.sleep()

#     rospy.loginfo("Connected to FCU")

#     pose = PoseStamped()
#     pose.pose.position.x = 0
#     pose.pose.position.y = 0
#     pose.pose.position.z = 0

#     # Send a few setpoints before starting
#     for i in range(100):
#         if rospy.is_shutdown():
#             break
#         local_pos_pub.publish(pose)
#         rate.sleep()

#     offb_set_mode = SetModeRequest()
#     offb_set_mode.custom_mode = 'OFFBOARD'
#     arm_cmd = CommandBoolRequest()
#     arm_cmd.value = True
#     last_req = rospy.Time.now()

#     # Create a listener for keyboard input
#   #  listener = kb.Listener(on_press=on_press)
#  #   listener.start()

#     while not rospy.is_shutdown():
#         if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
#             if set_mode_client.call(offb_set_mode).mode_sent:
#                 rospy.loginfo("OFFBOARD enabled")
#             last_req = rospy.Time.now()
#         else:
#             if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
#                 if arming_client.call(arm_cmd).success:
#                     rospy.loginfo("Vehicle armed")
#                 last_req = rospy.Time.now()
#         local_pos_pub.publish(pose)
#         rate.sleep()


# coding=UTF-8

# import rospy
# import mavros
# import mavros.command as mc
# from mavros_msgs.msg import State
# from geometry_msgs.msg import PoseStamped, Twist, Quaternion
# from mavros_msgs.srv import CommandBool
# from mavros_msgs.srv import SetMode
# import tf.transformations as t
# import math
# import sys
# import signal
# import curses

# current_state = State()
# current_pose = PoseStamped()
# current_vel = Twist()

# def localpose_callback(data):
#     global current_pose
#     current_pose = data

# def publish_setvel(event):
#     global current_pose, setvel_pub, setvel, setvel_forward
#     q = current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w
#     roll, pitch, yaw = t.euler_from_quaternion(q)
#     setvel.linear.x = setvel_forward * math.cos(yaw)
#     setvel.linear.y = setvel_forward * math.sin(yaw)
#     setvel_pub.publish(setvel)

# def signal_handler(sig, frame):
#     global set_mode
#     print('Switching to MANUAL mode...')
#     set_mode(0, "MANUAL")
#     curses.endwin()
#     sys.exit(0)

# def main():
#     global current_pose, setvel, setvel_pub, setvel_forward, set_mode

#     signal.signal(signal.SIGINT, signal_handler)
#     rospy.init_node("offbrd", anonymous=True)
#     rate = rospy.Rate(20)
#     pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, localpose_callback)
#     setvel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
#     arming_s = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
#     set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
#     setvel = Twist()
#     setvel_forward = 0

#     arming_s(True)
#     set_mode(0, "AUTO.TAKEOFF")
#     print('Taking off.....\r')
#     rospy.sleep(5)
    
#     stdscr = curses.initscr()
#     curses.noecho()
#     stdscr.nodelay(1)
#     stdscr.keypad(1)

#     for i in range(0, 100):
#         setvel_pub.publish(setvel)
#         rate.sleep()
#     set_mode(0, "OFFBOARD")
#     setvel_timer = rospy.Timer(rospy.Duration(0.05), publish_setvel)


#     while not rospy.is_shutdown():
#         rate.sleep()
#         c = stdscr.getch()
#         if c == ord('q'):
#             break  # Exit the while()
#         elif c == ord('u'):
#             setvel.linear.z += 0.25
#         elif c == ord('d'):
#             setvel.linear.z -= 0.25
#         elif c == curses.KEY_LEFT:
#             setvel.angular.z += 0.25
#         elif c == curses.KEY_RIGHT:
#             setvel.angular.z -= 0.25
#         elif c == curses.KEY_UP:
#             setvel_forward += 0.25 
#         elif c == curses.KEY_DOWN:
#             setvel_forward -= 0.25
#         elif c == ord('s'):
#             setvel_forward = setvel.linear.z = setvel.angular.z = 0
#         if c != curses.ERR:
#             print(setvel, '\r')
    
#     curses.endwin()
#     set_mode(0, "AUTO.LAND")
#     print('Landing.......\r')

# if __name__ == "__main__":
#     main()


#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
#from pynput import keyboard as kb
from std_msgs.msg import String
import signal
import sys
#sets current state to the state of the drone
global pose
current_state = State()

#call back to set state
def state_cb(msg):
    global current_state
    current_state = msg

#call back to set position
def position_cb(msg):
    global current_position
    current_position = msg
#function created to set manual mode upon keyboard interrupt
def set_manual_mode():
    manual_set_mode = SetModeRequest()
    manual_set_mode.custom_mode = 'MANUAL'
    try:
        resp = set_mode_client.call(manual_set_mode)
        if resp.mode_sent:
            rospy.loginfo("Manual mode enabled")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
#function created to handle keyboard interrupt
def signal_handler(sig, frame):
    rospy.loginfo('KeyboardInterrupt (ID: {}) has been caught. Changing to manual mode...'.format(sig))
    set_manual_mode()
    sys.exit(0)

def increase_altitude():
    pose.pose.position.z += 1
    local_pos_pub.publish(pose)

def decrease_altitude():
    # global pose
    pose.pose.position.z -= 1
    local_pos_pub.publish(pose)

def move_left():
    # global pose
    pose.pose.position.x -= 1
    local_pos_pub.publish(pose)

def move_right():
    # global pose
    pose.pose.position.x += 1
    local_pos_pub.publish(pose)

def on_press(key):
    
    try:
        if key.data == 'w':
            print("key pressed")
            increase_altitude()
        elif key.data == 's':
            decrease_altitude()
        elif key.data == 'a':
            move_left()
        elif key.data == 'd':
            move_right()
    except AttributeError:
        pass  # Handle special keys here if necessary

if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    signal.signal(signal.SIGINT, signal_handler)  # Register the signal handler

    state_sub = rospy.Subscriber("/mavros/state", State, callback=state_cb)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_pos_sub = rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, callback=position_cb)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    middleware_sub = rospy.Subscriber("/middleware/control", String, on_press)

    rate = rospy.Rate(20)  # Setpoint publishing MUST be faster than 2Hz

    # Wait for FCU connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    rospy.loginfo("Connected to FCU")

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    last_req = rospy.Time.now()

    # Create a listener for keyboard input
  #  listener = kb.Listener(on_press=on_press)
 #   listener.start()
    if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if arming_client.call(arm_cmd).success:
                rospy.loginfo("Vehicle armed")
                # set_mode_client.call(offb_set_mode)
            last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
    local_pos_pub.publish(pose)
    rate.sleep()
