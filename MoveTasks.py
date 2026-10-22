#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

g_robotPosition = Odometry()
g_positionReceived = False


def odom_callback(data):
    global g_robotPosition
    global g_positionReceived
    global roll, pitch, yaw
    quaternion = data.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    
    rospy.loginfo("Robot position: [%f, %f]", 
             data.pose.pose.position.x, 
             data.pose.pose.position.y)
    
    g_robotPosition = data
    g_positionReceived = True

rospy.init_node("move_robot")
rospy.Subscriber('odom', Odometry, odom_callback)
cmdvel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
rate = rospy.Rate(10)


def goto(xGoal:int, yGoal:int):
    global g_robotPosition
    global g_positionReceived
    
    # Wait receiving robot Odometry at least one time
    while (not rospy.is_shutdown()) and (not g_positionReceived):
        rospy.loginfo("Waiting Odometry data")
        rospy.loginfo("position received = %d", g_positionReceived)
        rate.sleep()
    
    #publish velocity of robot 0.1m/s forward
    velocity_data = Twist()
    actual_quaternion = g_robotPosition.pose.pose.orientation
    yaw = euler_from_quaternion([
        actual_quaternion.x,
        actual_quaternion.y,
        actual_quaternion.z,
        actual_quaternion.w
    ])[2]
    x_error = xGoal - g_robotPosition.pose.pose.position.x
    y_error = yGoal - g_robotPosition.pose.pose.position.y
    robot_x_error = math.cos(yaw) * x_error + math.sin(yaw) * y_error
    robot_y_error = -math.sin(yaw) * x_error + math.cos(yaw) * y_error
    velocity_data.linear.x = 0.1 * math.copysign(1, robot_x_error)
    velocity_data.linear.y = 0.1 * math.copysign(1, robot_y_error)
    cmdvel_pub.publish(velocity_data)
    rospy.loginfo("move the robot")
    
    positionReached = [0, 0]
    while (not rospy.is_shutdown()) and (positionReached != [1, 1]):
        cmdvel_pub.publish(velocity_data)
        
        if abs(g_robotPosition.pose.pose.position.x - xGoal) < 0.01:
            rospy.loginfo("Robot reach final x position")
            velocity_data.linear.x = 0
            cmdvel_pub.publish(velocity_data)
            positionReached[0] = 1

        if abs(g_robotPosition.pose.pose.position.y - yGoal) < 0.01:
            rospy.loginfo("Robot reach final y position")
            velocity_data.linear.y = 0
            cmdvel_pub.publish(velocity_data)
            positionReached[1] = 1
        
        rate.sleep()

def yawto(yawGoal:int):
    global g_robotPosition
    global g_positionReceived
    
    # Wait receiving robot Odometry at least one time
    while (not rospy.is_shutdown()) and (not g_positionReceived):
        rospy.loginfo("Waiting Odometry data")
        rospy.loginfo("position received = %d", g_positionReceived)
        rate.sleep()
    
    velocity_data = Twist()
    actual_qaternion = g_robotPosition.pose.pose.orientation
    yaw = euler_from_quaternion([actual_qaternion.x, actual_qaternion.y, actual_qaternion.z, actual_qaternion.w])[2]
    velocity_data.angular.z = 0.1*math.copysign(1, yawGoal - yaw)
    cmdvel_pub.publish(velocity_data)
    rospy.loginfo("rotate the robot")
    
    positionReached = 0
    while (not rospy.is_shutdown()) and (positionReached != 1):
        cmdvel_pub.publish(velocity_data)
        actual_qaternion = g_robotPosition.pose.pose.orientation
        if abs(euler_from_quaternion([actual_qaternion.x, actual_qaternion.y, actual_qaternion.z, actual_qaternion.w])[2] - yawGoal) < 0.01:
            rospy.loginfo("Robot reach final yaw position")
            velocity_data.angular.z = 0
            cmdvel_pub.publish(velocity_data)
            positionReached = 1
        
        rate.sleep()

def moveto(xGoal: float, yGoal: float):
    global g_robotPosition
    global g_positionReceived
    
    # Attendre de recevoir l'odométrie au moins une fois
    while (not rospy.is_shutdown()) and (not g_positionReceived):
        rospy.loginfo("Waiting Odometry data")
        rate.sleep()
    
    velocity_data = Twist()
    rospy.loginfo("Déplacement vers [%f, %f]", xGoal, yGoal)
    
    while not rospy.is_shutdown():

        current_x = g_robotPosition.pose.pose.position.x
        current_y = g_robotPosition.pose.pose.position.y
        
        quaternion = g_robotPosition.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        x_error = xGoal - current_x
        y_error = yGoal - current_y
        distance = math.sqrt(x_error**2 + y_error**2)
        
        if distance < 0.05:
            rospy.loginfo("Cible atteinte !")
            velocity_data.linear.x = 0.0
            velocity_data.angular.z = 0.0
            cmdvel_pub.publish(velocity_data)
            break
            
        target_angle = math.atan2(y_error, x_error)
        angle_error = target_angle - yaw
        
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_error) > 0.25:
            velocity_data.linear.x = 0.0
            velocity_data.angular.z = 0.3 * math.copysign(1, angle_error)
        else:
            velocity_data.linear.x = 0.15  
            velocity_data.angular.z = 0.8 * angle_error 
        cmdvel_pub.publish(velocity_data)
        rate.sleep()

def Task1():
    goto(0, 0)
    rate.sleep()
    goto(1, 0)
    rate.sleep()
    goto(1, 1)
    rate.sleep()
    goto(0, 1)
    rate.sleep()
    goto(0, 0)

def Task2():
    yawto(0)
    goto(0, 0)
    goto(1, 0)
    yawto(1.57)
    goto(1, 1)
    yawto(3.14)
    goto(0, 1)
    yawto(-1.57)
    goto(0, 0)

def Task2V2():
    yawto(0)
    goto(0, 0)
    moveto(1, 0)
    yawto(1.57)
    moveto(1, 1)
    yawto(3.14)
    moveto(0, 1)
    yawto(-1.57)
    moveto(0, 0)
    

if __name__ == '__main__':
    try:
        Task2V2()
    except rospy.ROSInterruptException:
        pass
            
    
    