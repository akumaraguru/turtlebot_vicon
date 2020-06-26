#!/usr/bin/env python
import argparse
import csv

import numpy as np

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf


EKF_FIELDNAMES = [
    'ros_time',
    'position',
    'orientation',
    'covariance',
]

VIC_FIELDNAMES = [
    'ros_time',
    'position',
    'orientation',
]

def ekf_cb(writer, data):
    row = dict(
        ros_time=data.header.stamp.secs + 1e-9 * data.header.stamp.nsecs,
        position=[data.pose.pose.position.x,
                  data.pose.pose.position.y,
                  data.pose.pose.position.z],
        orientation=[data.pose.pose.orientation.x,
                     data.pose.pose.orientation.y,
                     data.pose.pose.orientation.z],
        covariance=list(data.pose.covariance),
    )
    writer.writerow(row)

def main(ekf_topic, vic_frame, ekf_path, vic_path):
    with open(ekf_path, mode='w') as ekf_file:
        with open(vic_path, mode='w') as vic_file:
            rospy.init_node('pose_recorder', anonymous=True)

            ekf_writer = csv.DictWriter(ekf_file, fieldnames=EKF_FIELDNAMES)
            ekf_writer.writeheader()
            rospy.Subscriber(
                ekf_topic,
                PoseWithCovarianceStamped,
                lambda data: ekf_cb(ekf_writer, data))

            vic_writer = csv.DictWriter(vic_file, fieldnames=VIC_FIELDNAMES)
            vic_writer.writeheader()
            listener = tf.TransformListener()
            last_ts = 0
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                try:
                    ts = listener.getLatestCommonTime('/odom', vic_frame)
                    if ts != last_ts:
                        trans, rot = listener.lookupTransform(
                            '/odom', vic_frame, ts)
                        row = dict(
                            ros_time=ts.to_sec(),
                            position=list(trans),
                            orientation=list(rot),
                        )
                        vic_writer.writerow(row)
                        last_ts = ts
                        rospy.loginfo(ts)
                    rate.sleep()
                except (tf.LookupException,
                        tf.ConnectivityException,
                        tf.ExtrapolationException):
                    continue


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Record a trajectory to csv.")
    parser.add_argument(
        "--ekf-topic",
        dest="ekf_topic",
        default="/robot_pose_ekf/odom_combined",
        help="EKF topic to subscribe to")
    parser.add_argument(
        "--vicon-frame",
        dest="vicon_frame",
        default="/vicon/tb3_three_resl/tb3_three_resl",
        help="tf frame of vicon tracker")
    parser.add_argument(
        "--ekf-csv",
        dest="ekf_csv",
        default="ekf_pose.csv",
        help="output csv file path for EKF pose")
    parser.add_argument(
        "--vicon-csv",
        dest="vicon_csv",
        default="vicon_pose.csv",
        help="output csv file path for Vicon pose")
    args = parser.parse_args(rospy.myargv()[1:])

    main(args.ekf_topic, args.vicon_frame, args.ekf_csv, args.vicon_csv)
