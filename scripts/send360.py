#!/usr/bin/env python

import rospy
import sys
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Quaternion
import math

if __name__ == '__main__':

    # expected arguments are X,Y,Z,YAW
    if len(sys.argv) < 5:
        print("Arguments needed: X Y Z YAW")
        exit()

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    yaw = float(sys.argv[4])

    rospy.init_node('waypoint_publisher')

    pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=1)

    msg = MultiDOFJointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"

    angle_inc = 2*math.pi/20.0
    end_yaw = yaw + 2*math.pi

    while yaw <= end_yaw:
        point = MultiDOFJointTrajectoryPoint()
        transform = Transform()
        transform.translation.x = x
        transform.translation.y = y
        transform.translation.z = z
        transform.rotation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        point.transforms.append(transform)
        msg.points.append(point)
        yaw += angle_inc

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        #rospy.loginfo('Connections: %d', connections)
        if connections > 0:
            pub.publish(msg)
            rospy.loginfo("Waypoint published!")
            break
        rate.sleep()
