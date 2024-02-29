
#!/usr/bin/env python3
from numbers import Number
import numpy as np
import rclpy, array
import tf2_ros
from geometry_msgs.msg import Twist,TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from apriltag_msgs.msg import AprilTagDetection
from bboxes_ex_msgs.msg import BoundingBoxes
#from bboxes_ex_msgs.msg import BoundingBox

print("Scanning...")
op = open('OutputR.txt', 'w')

marker_detected = False
previous_id = []
previous_class_id = []



def image_callback(msg):
    """ Use a flag that stays high as long as the topic shows that a image is detected"""
    global marker_detected, previous_class_id, tfBuffer
    if len(msg.bounding_boxes) > 0 : 
        if not msg.bounding_boxes[0].class_id in previous_class_id : 
            previous_class_id.append(msg.bounding_boxes[0].class_id)
            print("IMAGEs detected : ")
            print(previous_class_id)
            # op.write(previous_class_id)
        op.write("Output : "+ str(previous_class_id))


def marker_callback(msg):
    """ Use a flag that stays high as long as the topic shows that a marker is detected"""
    global marker_detected, previous_id, tfBuffer
    
    if len(msg.detections) > 0 : 
        #marker_detected = True
        #previous_id.append(msg.detections[0].id)
        if not msg.detections[0].id in previous_id : 
            previous_id.append(msg.detections[0].id)
            print("TAGs detected : ")
            print(previous_id)
        op.write("Output : " + str(previous_id))

    tags_str = []
    for element in previous_id:
        tags_str.append(str(element))
        
    out = list(zip(tags_str, previous_class_id))
    print(f"The tags and associated images are: {out}")

    #op.write(str(previous_id[0]) + ":" + str(previous_class_id[0]) + "\n")
    #op.write(str(previous_id[1]) + ":" + str(previous_class_id[1]) + "\n")
    #op.write(str(previous_id[2]) + ":" + str(previous_class_id[2]) + "\n")
    #op.write(str(previous_id[3]) + ":" + str(previous_class_id[3]) + "\n")
    #op.write(str(previous_id[4]) + ":" + str(previous_class_id[4]))

            #previous_id = msg.detections[0].id
            #trans = tfBuffer.lookup_transform("map", "marker", rclpy.duration.Duration())
            #printer(trans)
            
        # if marker_detected == True:
        #  
        # else:
        #  marker_detected = False
        
        
        # # twist = trans_to_twist(trans)
        # # publish_cmd_vel(twist)
        # for x in msg.detections() : 
        #     print(x.id)
        # print("Length:"+ str(len(msg.detections)))
        # print("Tag ID:" + str(msg.id))
    

#the cmd_vel and braking and all part was here
# def printer(trans: TransformStamped):
#     print("AprilTag detected")
#     print("----------------------------------------")
#     print("Position: ")
#     print("  X: " + str(trans.transform.translation.x))
#     print("  Y: " + str(trans.transform.translation.y))
#     print("  Z: " + str(trans.transform.translation.z))  
#     print("----------------------------------------")
    

def main():
    """Initialize ROS related setup"""
    global nh, pub_cmd, tfBuffer
    rclpy.init()
    nh = rclpy.create_node('follower')
    #pub_cmd = nh.create_publisher(Twist, '/cmd_vel', 1)
    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer, nh)
    nh.create_subscription(AprilTagDetectionArray, "/apriltag/detections", marker_callback, 10)
    nh.create_subscription(BoundingBoxes, "/bounding_boxes", image_callback, 10)
    #nh.create_timer(0.1, timercallback)
    try:
        rclpy.spin(nh)
    except KeyboardInterrupt:
        #stop()
        nh.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()