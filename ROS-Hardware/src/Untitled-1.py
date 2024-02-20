# #! /usr/bin/env python3

# # import rospy
# # from geometry_msgs.msg import PoseStamped
# # from mavros_msgs.msg import State
# # from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
# # import keyboard
# # from std_msgs.msg import String

# # current_state = State()
# # pose = PoseStamped()

# # def state_cb(msg):
# #     global current_state
# #     current_state = msg

# # def position_cb(msg):
# #     global current_position
# #     current_position = msg


# # #if __name__ == "__main__":


# # def control_Drone(key):
# #     print(key)
# #     global current_position
# #     current_position = key.data
# #     # Setpoint publishing MUST be faster than 2Hz
# #     rate = rospy.Rate(20)
# #     # Wait for Flight Controller connection
# #     # keys_pressed = keyboard.read_event(suppress=True).name.split(' ')

# #     while(not rospy.is_shutdown() and not current_state.connected):
# #         rate.sleep()
# #         print('Not Connected')
    
# #     print('Connected')
    
    

# #     #set the pos.position.x to current position.x


# #     pose.pose.position.x = current_position.pose.position.x
# #     pose.pose.position.y = current_position.pose.position.y
# #     pose.pose.position.z = current_position.pose.position.z
# #     # Send a few setpoints before starting
# #     for i in range(100):
# #         if(rospy.is_shutdown()):
# #             break
# #         local_pos_pub.publish(pose)
# #         rate.sleep()
    
# #     offb_set_mode = SetModeRequest()
# #     offb_set_mode.custom_mode = 'OFFBOARD'
    
# #     arm_cmd = CommandBoolRequest()
# #     arm_cmd.value = True
# #     last_req = rospy.Time.now()
    
# #     try:
# #         while not rospy.is_shutdown():
# #             if key == 'w':
# #                 pose.pose.position.z += 5
# #                 print(pose.pose.position.z)
# #     except Exception as e:
# #         print(e)
    

# #     while(not rospy.is_shutdown()):
# #         print('Connected')
# #         if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
# #             if(set_mode_client.call(offb_set_mode).mode_sent == True):
# #                 rospy.loginfo("OFFBOARD enabled")
# #             last_req = rospy.Time.now()
# #         else:
# #             if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
# #                 if(arming_client.call(arm_cmd).success == True):
# #                     rospy.loginfo("Vehicle armed")
# #                 last_req = rospy.Time.now()
# #         print(f"The position is {pose.pose.position.z}")
# #         local_pos_pub.publish(pose)
# #         rate.sleep()




# # rospy.init_node("offb_node_py")

# # state_sub = rospy.Subscriber("/mavros/state", State, callback = state_cb)
# # local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
# # local_pos_sub = rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, callback = position_cb)

# # middleware_sub = rospy.Subscriber("/middleware/control", String, control_Drone)
# # rospy.wait_for_service("/mavros/cmd/arming")
# # arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

# # rospy.wait_for_service("/mavros/set_mode")
# # set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


# # rospy.spin()
# # import airsim #pip install airsim

# # #for car use CarClient() 
# # client = airsim.MultirotorClient()


# # responses = client.simGetImages([
# # airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
# # airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
# # airsim.ImageRequest("1", airsim.ImageType.Scene), #scene vision image in png format
# # airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
# # print('Retrieved images: %d' % len(responses[0].image_data_uint8))
# # print(responses[0])

# # import airsim

# # import pprint
# # import tempfile
# # import os
# # import time

# # import cv2
# # import numpy as np

# # client = airsim.VehicleClient()

# # responses = client.simGetImages([
# #     airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
# #     airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
# #     airsim.ImageRequest("1", airsim.ImageType.Scene), #scene vision image in png format
# #     airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
# # print('Retrieved images: %d' % len(responses))


# # for idx, response in enumerate(responses):



# #     if response.pixels_as_float:
# #         print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
# #         #airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
# #     elif response.compress: #png format
# #         print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
# #         #airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
# #     else: #uncompressed array
# #         print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
# #         img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
# #         img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3

# #         #test run ay 7aga#!/usr/bin/env python3


# # import rospy
# # from geometry_msgs.msg import PoseStamped
# # from mavros_msgs.msg import State
# # from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
# # from pynput import keyboard as kb
# # from std_msgs.msg import String


# # current_state = State()

# # def state_cb(msg):
# #     global current_state
# #     current_state = msg

# # def position_cb(msg):
# #     global current_position
# #     current_position = msg

# # def increase_altitude():
# #     global pose
# #     pose.pose.position.z += 1
# #     local_pos_pub.publish(pose)


# # def decrease_altitude():
# #     global pose
# #     pose.pose.position.z -= 1
# #     local_pos_pub.publish(pose)

# # def move_left():
# #     global pose
# #     pose.pose.position.x -= 1
# #     local_pos_pub.publish(pose)

# # def move_right():
# #     global pose
# #     pose.pose.position.x += 1
# #     local_pos_pub.publish(pose)


# # def on_press(key):
# #     print(key)
# #     if key.data == 'w':
# #         increase_altitude()
# #     elif key.data == 's':
# #         decrease_altitude()
# #     elif key.data == 'a':
# #         move_left()
# #     elif key.data == 'd':
# #         move_right()

# # if __name__ == "__main__":
# #     rospy.init_node("offb_node_py")
# #     state_sub = rospy.Subscriber("/mavros/state", State, callback=state_cb)
# #     local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
# #     local_pos_sub = rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, callback = position_cb)

# #     rospy.wait_for_service("/mavros/cmd/arming")
# #     arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
# #     rospy.wait_for_service("/mavros/set_mode")
# #     set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
# #     middleware_sub = rospy.Subscriber("/middleware/control", String, on_press)



# #     rate = rospy.Rate(20)

# #     while not rospy.is_shutdown() and not current_state.connected:
# #         rate.sleep()
# #         print('Not Connected')
# #     print('Connected')

# #     pose = PoseStamped()
# #     pose.pose.position.x = 0
# #     pose.pose.position.y = 0
# #     pose.pose.position.z = 12

# #     for i in range(100):
# #         if rospy.is_shutdown():
# #             break
# #         local_pos_pub.publish(pose)
# #         rate.sleep()

# #     offb_set_mode = SetModeRequest()
# #     offb_set_mode.custom_mode = 'OFFBOARD'
# #     arm_cmd = CommandBoolRequest()
# #     arm_cmd.value = True
# #     last_req = rospy.Time.now()

# #     # Create a listener for keyboard input
# #     listener = kb.Listener(on_press=on_press)
# #     listener.start()

# #     while not rospy.is_shutdown():
# #         print('Connected')

# #         if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
# #             if set_mode_client.call(offb_set_mode).mode_sent:
# #                 rospy.loginfo("OFFBOARD enabled")
# #             last_req = rospy.Time.now()
# #         else:
# #             if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
# #                 if arming_client.call(arm_cmd).success:
# #                     rospy.loginfo("Vehicle armed")
# #                 last_req = rospy.Time.now()
        
        
# #         rate.sleep()

# #!/usr/bin/env python3


# import rospy
# from geometry_msgs.msg import PoseStamped
# from mavros_msgs.msg import State
# from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
# #from pynput import keyboard as kb
# from std_msgs.msg import String
# import signal
# import sys


# current_state = State()
# current_position = PoseStamped()
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
#             rospy.loginfo("Manual Mode enabled")
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s",e)


# def signal_handler(sig, frame):
#     rospy.loginfo("KeyboardInterrupt (ID: {}) has been caught. Changing to manual mode".format(sig))
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
#     print(key)
#     try:
#         if key.data == 'w':
#             print('w')
#             increase_altitude()
#         elif key.data == 's':
#             decrease_altitude()
#         elif key.data == 'a':
#             move_left()
#         elif key.data == 'd':
#             move_right()
#     except AttributeError:
#         pass #Handle special keys here if necessary

# if __name__ == "__main__":
#     rospy.init_node("offb_node_py")
#     signal.signal(signal.SIGINT, signal_handler)

#     state_sub = rospy.Subscriber("/mavros/state", State, callback=state_cb)
#     local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
#     local_pos_sub = rospy.Subscriber("/mavros/setpoint_position/local", PoseStamped, callback = position_cb)

#     rospy.wait_for_service("/mavros/cmd/arming")
#     arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
#     rospy.wait_for_service("/mavros/set_mode")
#     set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
#     middleware_sub = rospy.Subscriber("/middleware/control", String, on_press)



#     rate = rospy.Rate(20)

#     while not rospy.is_shutdown() and not current_state.connected:
#         rate.sleep()
#         print('Not Connected')
#     print('Connected')

#     # arming_client(True)
#     arm_cmd = CommandBoolRequest()
#     arm_cmd.value = True
#     arming_client.call(arm_cmd)
#     set_mode_client(0,"AUTO.TAKEOFF")
#     print ('Taking off.....\r')
#     rospy.sleep(5)

#     #current_position = PoseStamped()
#     pose = PoseStamped()
#     pose.pose.position.x = current_position.pose.position.x
#     pose.pose.position.y = current_position.pose.position.y
#     pose.pose.position.z = current_position.pose.position.z

    
#     for i in range(100):
#         if rospy.is_shutdown():
#             break
#         local_pos_pub.publish(pose)
#         rate.sleep()
#     # set_mode_client(0,"OFFBOARD")
#     # offb_set_mode = SetModeRequest()
#     # offb_set_mode.custom_mode = 'OFFBOARD'
#     # set_mode_client.call(offb_set_mode)


#     # # Create a listener for keyboard input
#     # listener = kb.Listener(on_press=on_press)
#     # listener.start()

#     offb_set_mode = SetModeRequest()
#     offb_set_mode.custom_mode = 'OFFBOARD'
#     arm_cmd = CommandBoolRequest()
#     arm_cmd.value = True
#     last_req = rospy.Time.now()
    
#     while not rospy.is_shutdown():
#         print('Connected')
#         while 
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
#     #rospy.spin()

# # #!/usr/bin/env python
# # # coding=UTF-8
# # import rospy
# # import mavros
# # import mavros.command as mc
# # from mavros_msgs.msg import State
# # from geometry_msgs.msg import PoseStamped, Twist, Quaternion
# # from mavros_msgs.srv import CommandBool
# # from mavros_msgs.srv import SetMode
# # import tf.transformations as t
# # import math
# # from std_msgs.msg import String

# # current_state=State()
# # current_pose = PoseStamped()
# # current_vel = Twist()

# # def localpose_callback(data):
# #     global current_pose
# #     current_pose = data

# # def publish_setvel(event):
# #     global current_pose, setvel_pub, setvel, setvel_forward
# #     q=current_pose.pose.orientation.x, current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w
# #     roll, pitch, yaw = t.euler_from_quaternion(q)
# #     setvel.linear.x = setvel_forward * math.cos(yaw)
# #     setvel.linear.y = setvel_forward * math.sin(yaw)
# #     setvel_pub.publish(setvel)

# # def main():
# #     global current_pose, setvel, setvel_pub, setvel_forward

# #     rospy.init_node("offbrd",anonymous=True)
# #     rate=rospy.Rate(10)
# #     pose_sub=rospy.Subscriber("/mavros/local_position/pose",PoseStamped,localpose_callback)
# #     setvel_pub=rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=1)
# #     arming_s=rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
# #     set_mode=rospy.ServiceProxy("/mavros/set_mode",SetMode)
# #     middleware_sub = rospy.Subscriber("/middleware/control", String, c)

# #     setvel=Twist()
# #     setvel_forward = 0

# #     arming_s(True)
# #     set_mode(0,"AUTO.TAKEOFF")
# #     print ('Taking off.....\r')
# #     rospy.sleep(5)
    
# #     # keyboard manipulation
# #     import curses
# #     stdscr = curses.initscr()
# #     curses.noecho()
# #     stdscr.nodelay(1)
# #     stdscr.keypad(1)

# #     for i in range (0,10):
# #         setvel_pub.publish(setvel)
# #         rate.sleep()
# #     set_mode(0,"OFFBOARD")
# #     setvel_timer = rospy.Timer(rospy.Duration(0.05), publish_setvel)
# #     while (rospy.is_shutdown()==False):
# #         rate.sleep()
# #         # keyboard  hcommands handling    
# #         c = stdscr.getch()
# #         if c == ord('q'): break  # Exit the while()
# #         elif c == ord('u'): setvel.linear.z += 0.25
# #         elif c == ord('d'): setvel.linear.z -= 0.25
# #         elif c == curses.KEY_LEFT: setvel.angular.z += 0.25
# #         elif c == curses.KEY_RIGHT: setvel.angular.z -= 0.25
# #         elif c == curses.KEY_UP: setvel_forward += 0.25 
# #         elif c == curses.KEY_DOWN: setvel_forward -= 0.25
# #         elif c == ord('s'): setvel_forward=setvel.linear.z=setvel.angular.z=0
# #         if  c!=curses.ERR: 
# #           print (setvel,'\r')
# #     curses.endwin()
# #     set_mode(0,"AUTO.LAND")
# #     print ('Landing.......\r')

# # if __name__=="__main__":
# #     main()