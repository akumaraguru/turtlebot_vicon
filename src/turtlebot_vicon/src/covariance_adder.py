#!/usr/bin/env python
from copy import deepcopy

import rospy
from nav_msgs.msg import Odometry


COV_VAL = 1e-2
COV_IDX = [0, 7, 14, 21, 28, 35]

def odom_cb(pub, data):
    odom_w_cov = deepcopy(data)
    pose_cov = list(odom_w_cov.pose.covariance)
    for i in COV_IDX:
        pose_cov[i] += COV_VAL
    odom_w_cov.pose.covariance = tuple(pose_cov)
    pub.publish(odom_w_cov)
    # rospy.loginfo(
    #     "retransmit msg @ {}".format(data.header.stamp))


def main():
    rospy.init_node('cov_adder', anonymous=True)
    pub = rospy.Publisher(
        'odom_w_cov', Odometry, queue_size=10)
    rospy.Subscriber(
        'odom', Odometry, lambda data: odom_cb(pub, data))
    rospy.loginfo(
        "Initialized covariance adder: odom -> odom_w_cov")
    rospy.spin()


if __name__ == '__main__':
    main()
