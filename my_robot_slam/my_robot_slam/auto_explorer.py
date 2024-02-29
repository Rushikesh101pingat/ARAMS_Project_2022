#!/usr/bin/env python3
from operator import imod
import sys
import tf2_ros
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, Twist
from bboxes_ex_msgs.msg import BoundingBoxes
from apriltag_msgs.msg import AprilTagDetectionArray
###
print("Scanning...")
op = open('OutputR.txt', 'w')
marker_detected = False
previous_id = []
previous_class_id = []
###
global position, orientation
global nh, pub_cmd, tfBuffer
global counterr 

# Map Bounds
map_min_x = -6.1
map_max_x = 3.3
map_min_y = -0.6
map_max_y = 8.6

success = True

def publish_cmd_vel(twist):
    """Publish movement values to the topic"""
    global pub_cmd
    pub_cmd.publish(twist)

def stop():
    """Publish velocities for yawing"""
    twist = Twist()
    twist.angular.z = -0.5
    twist.linear.x = 0.0
    publish_cmd_vel(twist)
    
def stop2():
    """Publish velocities for braking"""
    twist = Twist()
    twist.angular.z = 0.0
    twist.linear.x = 0.0
    publish_cmd_vel(twist)   
#################################################
def image_callback(msg):
    """ Use a flag that stays high as long as the topic shows that a marker is detected"""
    global marker_detected, previous_class_id, tfBuffer, out
    if len(msg.bounding_boxes) > 0 : 
        if not msg.bounding_boxes[0].class_id in previous_class_id : 
            previous_class_id.append(msg.bounding_boxes[0].class_id)
            print("IMAGEs detected : ")
            print(previous_class_id)
            tags_str = []
            for element in previous_id:
              tags_str.append(str(element))
            out = list(zip(tags_str, previous_class_id))
            print(f"The tags and associated images are: {out}")
            op.write("\n" + "The tags and associated images are:" + str(out))
            
def marker_callback(msg):
    """ Use a flag that stays high as long as the topic shows that a marker is detected"""
    global marker_detected, previous_id, tfBuffer, out
    if len(msg.detections) > 0 and msg.detections[0].decision_margin > 30: 
        if not msg.detections[0].id in previous_id :
            marker_detected = True 
            previous_id.append(msg.detections[0].id)
            print("TAGs detected : ")
            print(previous_id)
            #op.write("\n" + "The tags and associated images are:" + str(out))
#################################################

def main():
  rclpy.init()
  global auto_chaos,pub_cmd
  global nav_to_pose_client
  global nh, tfBuffer

  counterr = 0
  status = 4

  auto_chaos = rclpy.create_node('auto_explorer')
  pub_cmd = auto_chaos.create_publisher(Twist, '/cmd_vel', 10)
  nav_to_pose_client = ActionClient(auto_chaos, NavigateToPose, 'navigate_to_pose')
  ###################################
  tfBuffer = tf2_ros.Buffer()
  tf2_ros.TransformListener(tfBuffer, auto_chaos)

  auto_chaos.create_subscription(AprilTagDetectionArray, "/apriltag/detections", marker_callback, 10)

  auto_chaos.create_subscription(BoundingBoxes, "/bounding_boxes", image_callback, 10)
######################################


  while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
    print("World still not available; waiting...")

  while rclpy.ok():
    
    try:
      if counterr == 0 :
        position = generatePosition1()
        #orientation = generateOrientation()
      elif counterr == 1 :
        position = generatePosition2()
      elif counterr == 2 :
        position = generatePosition3()
      elif counterr == 3 :
        position = generatePosition4()
      elif counterr == 4 :
        position = generatePosition5()
      elif counterr == 5 :
        position = generatePosition6()
      elif counterr == 6 :
        position = generatePosition7()
      elif counterr == 7 :
        position = generatePosition8()
      elif counterr == 8 :
        position = generatePosition9()
      elif counterr == 9 :
        position = generatePosition10()
      elif counterr == 10 :
        position = generatePosition11()
      elif counterr == 11 :
        position = generatePosition12()
      elif counterr == 12 :
        position = generatePosition13()
      elif counterr == 13 :
        position = generatePosition14()
      else :
        break

      #goal_handle = sendGoal(position,orientation)
      goal_handle = sendGoal(position)
      status = checkResult(goal_handle)
      if status == GoalStatus.STATUS_SUCCEEDED:
          print("Scanning Tags and Images...")
          stop()
          time.sleep(20) 
          stop2()
          print("stopping")
          time.sleep(2) 
          counterr += 1
          #print(counterr)
    except KeyboardInterrupt:
      print("Shutdown requested... complying...")
      break
  # op.write(str(previous_id[0]) + ":"  + str(previous_class_id[0]) + "\n" )
  # op.write(str(previous_id[1]) + ":"  + str(previous_class_id[1]) + "\n" )
  # op.write(str(previous_id[2]) + ":"  + str(previous_class_id[2]))
  #######op.write("\n Output = "+ str(previous_id) + ":" + str(previous_class_id))
 
  nav_to_pose_client.destroy()
  auto_chaos.destroy_node()
  #rclpy.shutdown()

#def sendGoal(position, orientation):
def sendGoal(position):
  global auto_chaos
  global nav_to_pose_client
  #Prev_position = Point()

  goal = NavigateToPose.Goal()
  goal.pose.header.frame_id = "map" 
  #An important factor here is the frame_id of the goal pose. 
  #this allows for sending either relative or absolute goals. 
  #To send a relative goal one has to send the goal relative to base_link. 
  #For absolute goals one has to send the goal relative to the map frame.
  
  goal.pose.header.stamp = auto_chaos.get_clock().now().to_msg()
  #if not Prev_position == position :
  goal.pose.pose.position = position
      #goal.pose.pose.orientation = orientation
  print("Received new goal => X: " + str(goal.pose.pose.position.x) + " Y: " + str(goal.pose.pose.position.y))
  send_goal_future = nav_to_pose_client.send_goal_async(goal)
  rclpy.spin_until_future_complete(auto_chaos, send_goal_future)
  goal_handle = send_goal_future.result()
  #Prev_position = position


  if not goal_handle.accepted:
    print("Goal was rejected")
    nav_to_pose_client.destroy()
    auto_chaos.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

  print("Goal Accepted!")

  return goal_handle

def checkResult(goal_handle):
  get_result_future = goal_handle.get_result_async()
  rclpy.spin_until_future_complete(auto_chaos, get_result_future)
  status = get_result_future.result().status
  if status == GoalStatus.STATUS_SUCCEEDED:
    print("Reached Goal !")
  return status

def generatePosition1():
  position = Point()
  position.x = 0.5
  position.y = 1.5
  position.z = 0.0
  return position
def generatePosition2():
  position = Point()
  position.x = 1.5 
  position.y = 1.0 
  position.z = 0.0
  return position
def generatePosition3():
  position = Point()
  position.x = 2.5 
  position.y = 2.5 
  position.z = 0.0
  return position
def generatePosition4():
  position = Point()
  position.x = 1.5 
  position.y = 4.0
  position.z = 0.0
  return position
def generatePosition5():
  position = Point()
  position.x = -0.5 
  position.y = 3.5 
  position.z = 0.0
  return position  
def generatePosition6():
  position = Point()
  position.x = -1.0 
  position.y = 2.0 
  position.z = 0.0
  return position
def generatePosition7():
  position = Point()
  position.x = -3.0 
  position.y = 3.0 
  position.z = 0.0
  return position
def generatePosition8():
  position = Point()
  position.x = -3.5
  position.y = 1.5
  position.z = 0.0
  return position
def generatePosition9():
  position = Point()
  position.x = -4.5
  position.y = 5.0 
  position.z = 0.0
  return position
def generatePosition10():
  position = Point()
  position.x = -4.5
  position.y = 7.0 
  position.z = 0.0
  return position
def generatePosition11():
  position = Point()
  position.x = -3.0 
  position.y = 7.0 
  position.z = 0.0
  return position
def generatePosition12():
  position = Point()
  position.x = -2.0 
  position.y = 5.0 
  position.z = 0.0
  return position
def generatePosition13():
  position = Point()
  position.x = 0.5
  position.y = 6.2
  position.z = 0.0
  return position
def generatePosition14():
  position = Point()
  position.x = 1.25
  position.y = 6.6
  position.z = 0.0
  return position
  
# def generateOrientation():
#   quat = Quaternion()
#   quat.w = 1.0
#   quat.x = 0.0
#   quat.y = 0.0
#   quat.z = 0.0 #sqrt(1 - quat.w*quat.w)
#   return quat

if __name__ == '__main__':
    main()