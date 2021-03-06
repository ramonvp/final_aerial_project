#!/usr/bin/env python

import rospy
import sys
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Quaternion

if __name__ == '__main__':

    # expected arguments are X,Y,Z,YAW
    if len(sys.argv) < 5:
        print("Arguments needed: X Y Z YAW")
        exit()

    rospy.init_node('waypoint_publisher')

    pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=1)

    msg = MultiDOFJointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"

    d = len(sys.argv) - 1
    i = 0
    while d >= 4:
        x = float(sys.argv[i+1])
        y = float(sys.argv[i+2])
        z = float(sys.argv[i+3])
        yaw = float(sys.argv[i+4])
        i = i + 4
        d = d - 4

        point = MultiDOFJointTrajectoryPoint()
        transform = Transform()
        transform.translation.x = x
        transform.translation.y = y
        transform.translation.z = z
        transform.rotation = Quaternion(*quaternion_from_euler(0, 0, yaw))
        point.transforms.append(transform)
        msg.points.append(point)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        #rospy.loginfo('Connections: %d', connections)
        if connections > 0:
            pub.publish(msg)
            rospy.loginfo("Waypoint published!")
            break
        rate.sleep()
