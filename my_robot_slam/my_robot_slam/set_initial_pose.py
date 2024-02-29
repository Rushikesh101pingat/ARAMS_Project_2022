#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

#flag = 1

def main():
    global mypub
    rclpy.init()
    myfirstpublisher = rclpy.create_node('publisher')
    mypub = myfirstpublisher.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    myfirstpublisher.create_timer(0.1, mytimercallback)
    try:
        rclpy.spin_once(myfirstpublisher)
    except KeyboardInterrupt:
        pass

    myfirstpublisher.destroy_node()
    rclpy.shutdown()

def mytimercallback():
    global mypub
    global flag
    mymsg = PoseWithCovarianceStamped()
    mymsg.header.frame_id = 'map'
    mymsg.pose.pose.position.x = 0.0
    mymsg.pose.pose.position.y = 0.0
    mymsg.pose.pose.position.z = 0.0
    mymsg.pose.pose.orientation.x = 0.0
    mymsg.pose.pose.orientation.y = 0.0
    mymsg.pose.pose.orientation.z = 1.0 #previously 1.57
    mymsg.pose.pose.orientation.w = 1.0 #1.0

    #if flag:
    mypub.publish(mymsg)
        #flag = 0
    exit()
    
if __name__ == '__main__':
    main()
