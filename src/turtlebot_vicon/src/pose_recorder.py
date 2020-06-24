#!/usr/bin/env python
import argparse
import csv

import numpy as np

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf.transformations import quaternion_matrix, rotation_matrix


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

def vic_cb(writer, data):
    trans = [data.transform.translation.x,
             data.transform.translation.y,
             data.transform.translation.z]
    rot = [data.transform.rotation.x,
           data.transform.rotation.y,
           data.transform.rotation.z,
           data.transform.rotation.w]

    T1 = np.eye(4)
    T1[:3, 3] = trans
    T2 = quaternion_matrix(rot)
    Rz = rotation_matrix(-np.pi/2, [0, 0, 1])
    TT = Rz.dot(T2.dot(T1))
    rospy.loginfo("\ntrans={}\n".format(trans))
    rospy.loginfo("\nT1={}\n".format(T1))
    rospy.loginfo("\nT2={}\n".format(T2))
    rospy.loginfo("\nTT={}\n".format(TT))

    row = dict(
        ros_time=data.header.stamp.secs + 1e-9 * data.header.stamp.nsecs,
        position=TT[:3, 3].flatten().tolist(),
        orientation=TT[:3, :3].flatten().tolist(),
    )
    writer.writerow(row)

def main(ekf_topic, vic_topic, ekf_path, vic_path):
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
            rospy.Subscriber(
                vic_topic,
                TransformStamped,
                lambda data: vic_cb(vic_writer, data))

            rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Record a trajectory to csv.")
    parser.add_argument(
        "--ekf-topic",
        dest="ekf_topic",
        default="/robot_pose_ekf/odom_combined",
        help="EKF topic to subscribe to")
    parser.add_argument(
        "--vicon-topic",
        dest="vicon_topic",
        default="/vicon/tb3_three_resl/tb3_three_resl",
        help="Vicon topic to subscribe to")
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

    main(args.ekf_topic, args.vicon_topic, args.ekf_csv, args.vicon_csv)
