#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Point


def VelocityPublisher_talker():
    pub = rospy.Publisher('setStepMotorVelocity_topic', Point, queue_size=10)
    # rospy.init_node('setpoint_node', anonymous=True)
    rospy.init_node('MotorVelocity_Publisher')
    rate = rospy.Rate(1) # 10hz
    velocity=100.00
    while not rospy.is_shutdown():
        velocity -= 1.0
        rospy.loginfo([velocity,velocity,velocity])
        pub.publish(velocity,velocity,velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
        VelocityPublisher_talker()
    except rospy.ROSInterruptException:
        pass
