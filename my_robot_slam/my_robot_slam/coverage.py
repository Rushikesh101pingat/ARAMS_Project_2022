
#!/usr/bin/env python3
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, Twist, Point
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import math
from math import pow, atan2, sqrt
import numpy as np
from scipy.spatial import distance

class MinimalActionClient(Node):

    def __init__(self):
        super().__init__('minimal_action_client')
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.send_points(mgoal)
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_waypoint))
        #euclidean_distance(waypoint)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.missed_waypoints))
        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_points(self, points):
        # self.get_logger().info('Waiting for action server...')

        msg = FollowWaypoints.Goal()
        msg.poses = points

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Sending goal request...')



def genpoints(x1, y1, x2, y2): #Send 2 diagonal cornerpoints of the room
    w = 0.5 #distance between 2 lanes
    nr_of_lanes = math.trunc((x2-x1)/w + 1)
    if y1 > 0:
        nr_of_waypoints = 4*nr_of_lanes
    else:
        nr_of_waypoints = 2*nr_of_lanes
    points = np.empty([nr_of_waypoints, 3])

    j = 1 #auxiliary loop index
    k = 0 #auxiliary loop index
    l = 0 #auxiliary loop index

    for i in range(nr_of_waypoints):

        if y1 > 0:
            if j == 1:
                points[i,0] = x1 + (k*w)
                points[i,1] = y1 - l*(y1-y2)
                points[i,2] = -0.706915
                l = l + 0.25
                if l > 1:
                    i = i+1
                    k = k + 1
                    j = 2
                    l = 0

            if j == 2:
                points[i,0] = x1 + (k*w)
                points[i,1] = y2 + l*(y1-y2)
                points[i,2] = 0.706915
                l = l + 0.25
                if l > 1:
                    i = i+1
                    j = 1
                    k = k + 1
                    l = 0
        else:
            if j == 1:
                points[i,0] = x1 + (k*w)
                points[i,1] = y1 - l*(y1-y2)
                points[i,2] = -0.706915
                l = l + 1
                if l > 1:
                    i = i+1
                    k = k + 1
                    l = 0

    return(points)

def main(args = None):
    global mgoal,waypoint,self
    rclpy.init(args=args)
    action_client = MinimalActionClient()

    # points1 = genpoints(-7.135, 0.476, -5.322, -3.730) # Bottom right room
    # points2 = genpoints(-7.278, 4.330, -5.341, 1.034) # Bottom left room
    # points3 = genpoints(-4.868, 4.330, -0.44, 0.069) # Center room
    # points4 = genpoints(0.130, 4.857, 1.883, 1.328) # Center small room
    # points5 = genpoints(2.431, 4.604, 6.874, 0.307) # Top left room
    # points6 = genpoints(5.164, -0.205, 6.920, -4.618) # Top right room
    # points7 = genpoints(0.638, -1.934, 0.638, -1.934) # Outside the house

    # waypoints = np.append(points1, points2, axis = 0)
    # waypoints = np.append(waypoints, points3, axis = 0)
    # waypoints = np.append(waypoints, points4, axis = 0)
    # waypoints = np.append(waypoints, points5, axis = 0)
    # waypoints = np.append(waypoints, points6, axis = 0)
    # waypoints = np.append(waypoints, points7, axis = 0)
    mgoal = []

    # for i in range(np.size(waypoints, 0)):
    #     waypoint = PoseStamped()
    #     waypoint.header.frame_id = "map"
    #     waypoint.pose.position.x =  waypoints[i,0]
    #     waypoint.pose.position.y =  waypoints[i,1]
    #     waypoint.pose.orientation.w = 0.706915
    #     waypoint.pose.orientation.x = 0.0
    #     waypoint.pose.orientation.y = 0.0
    #     waypoint.pose.orientation.z = waypoints[i,2]

    #for i in range(0, 4, 1):
    ## 1st point
    waypoint = PoseStamped()
    waypoint.header.frame_id = "map"
    waypoint.pose.position.x =  0.75 #+ i 
    waypoint.pose.position.y =  1.75 #+ i 
    waypoint.pose.orientation.w = 0.706915
    waypoint.pose.orientation.x = 0.0 #roll
    waypoint.pose.orientation.y = 0.0 #pitch
    waypoint.pose.orientation.z = 1.57 #yaw
    mgoal.append(waypoint)
    print(waypoint)
    action_client.send_points(mgoal)
    # speed = Twist()
    # speed.linear.x = 0.0
    # speed.angular.z = 0.2
    ## 2nd point
    waypoint.pose.position.x =  1.75 #+ i 
    waypoint.pose.position.y =  1.25 #+ i 
    waypoint.pose.orientation.w = 0.706915
    waypoint.pose.orientation.x = 0.0 #roll
    waypoint.pose.orientation.y = 0.0 #pitch
    waypoint.pose.orientation.z = 1.57 #yaw
    mgoal.append(waypoint)
    print(waypoint)
    action_client.send_points(mgoal)
    
    rclpy.spin(action_client)
    # speed = Twist()
    # speed.angular.z = 0.2
    # p1 = (waypoint.pose.position.x, waypoint.pose.position.y)
    # p2 = (self.pose.x,self.pose.y)
    # dist = distance.euclidean(p1,p2)
    # print(dist)




if __name__ == '__main__':
    main()
