#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Point


def VelocityPublisher_talker():
    pub = rospy.Publisher('setAngularVelocity', Point, queue_size=10)
    # rospy.init_node('setpoint_node', anonymous=True)
    rospy.init_node('Velocity_Publisher')
    rate = rospy.Rate(1) # 10hz
    velocity=100.00
    while not rospy.is_shutdown():
        velocity -= 0.10
        rospy.loginfo(velocity)
        pub.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
        VelocityPublisher_talker()
    except rospy.ROSInterruptException:
        pass
