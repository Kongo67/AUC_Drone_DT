#! /usr/bin/env python
# import rospy
# from mavros_msgs.msg import PositionTarget



# # from turtle import position
# from json.encoder import INFINITY
# from pickle import NONE
# import rospy
# from std_msgs.msg import String
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# import json
# import time

# last_speed=0
# last_acceleration=0

# def send_dictionary(dictionary,topic):
#     pub = rospy.Publisher(topic, String, queue_size=50)
#     # rospy.init_node('publisher', anonymous=True)
#     rate = rospy.Rate(10) # 10hz

#     encoded_data_string = json.dumps(dictionary)

#     # while not rospy.is_shutdown():
#         # hello_str = "hello world %s" % rospy.get_time()
#         # rospy.loginfo(encoded_data_string)
#         #timestamp here -----------------------------------------
#     pub.publish(encoded_data_string)
#     # time.sleep(1)
    
#     rate.sleep()

# def cmd_vel_callback(msg):
#     speed_dict = {}
#     global last_speed
#     speed_dict['sensor_name'] = 'Speed_Digital1'
#     # speed_dict['linear_x'] = msg.linear.x
#     # speed_dict['linear_y'] = msg.linear.y
#     # speed_dict['linear_z'] = msg.linear.z
#     # speed_dict['angular_x'] = msg.angular.x
#     # speed_dict['angular_y'] = msg.angular.y
#     # speed_dict['angular_z'] = msg.angular.z
#     speed_dict['magnitude'] = (((msg.linear.x)**2 + (msg.linear.y)**2)**0.5)*10
#     if(speed_dict['magnitude']==last_speed):
#         return
#     else:
#         last_speed=speed_dict['magnitude']
#         # print (speed_dict['magnitude'])

#         send_dictionary(speed_dict,'metric')

# def pose_callback(msg):
#     position_dict = {}
#     position_dict['sensor_name'] = 'Position_Digital_1'
#     position_dict['position_x'] = msg.pose.pose.position.x
#     position_dict['position_y'] = msg.pose.pose.position.y
#     position_dict['position_z'] = msg.pose.pose.position.z
#     print (position_dict['position_x'])
#     # encoded_data_string = json.dumps(position_dict)
#     # print('json',encoded_data_string)
#     send_dictionary(position_dict,'metric')
#     # loaded_dictionary = json.loads(encoded_data_string)
#     # print('dict',loaded_dictionary)



# n = 0
# linear_acc_x = 0
# linear_acc_y = 0
# linear_acc_z = 0
# def acc_callback(msg):
#     acc_dict = {}
#     global linear_acc_x,linear_acc_y,linear_acc_z,n,last_acceleration
#     linear_acc_x += msg.linear_acceleration.x
#     linear_acc_y += msg.linear_acceleration.y
#     linear_acc_z += msg.linear_acceleration.z
#     n+=1
#     acc_dict['sensor_name'] = 'Accelerometer_Digital1'
#     # acc_dict['linear_acceleration_x'] = linear_acc_x / n
#     # acc_dict['linear_acceleration_y'] = linear_acc_y / n
#     # acc_dict['linear_acceleration_z'] = linear_acc_z / n
#     acc_dict['magnitude'] = int((linear_acc_x*10000))
    
#     dif = abs(acc_dict['magnitude'] - last_acceleration)
#     #print(dif)
#     # acc_dict['magnitude'] = int((((linear_acc_x/n)**2 + (linear_acc_y/n)**2)**0.5)*100000)
#     if(acc_dict['magnitude']==last_acceleration or dif<100 or n<40): 
#         return
    
#     else:
#         last_acceleration=acc_dict['magnitude']
#         send_dictionary(acc_dict,'metric')
#         # print (acc_dict['magnitude'])
    

#     if n == 30:
#         linear_acc_x = linear_acc_y = linear_acc_z = n = 0
    


# rospy.init_node('Sensor_data')
# vel_sub = rospy.Subscriber('/cmd_vel',Twist , cmd_vel_callback)
# acc_sub = rospy.Subscriber('/imu',Imu , acc_callback)
# #acc_sub = rospy.Subscriber('/odom',Odometry , pose_callback)
# rospy.spin()



import rospy
from geometry_msgs.msg import TwistStamped  # Or another message type depending on the topic
from sensor_msgs.msg import Imu

def velocity_callback(data):
    # rospy.loginfo("Velocity: Linear: [%f, %f, %f] Angular: [%f, %f, %f]",
    #               data.twist.linear.x, data.twist.linear.y, data.twist.linear.z,
    #               data.twist.angular.x, data.twist.angular.y, data.twist.angular.z)
    #display the linearacceleration
    rospy.loginfo("Acceleration: Linear: [%f, %f, %f]",
                  data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)


def listener():
    rospy.init_node('velocity_listener', anonymous=True)
    rospy.Subscriber("/mavros/imu/data_raw", Imu, velocity_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
