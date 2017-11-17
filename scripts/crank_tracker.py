#!/usr/bin/env python


import rospy
import tf2_ros

from visualization_msgs.msg import Marker


def crank_path_visualizer():

    # Node initialization for the turntable player
    rospy.init_node('crank_path_visualizer')
    crank_marker = rospy.Publisher('path', Marker, queue_size=10)

    update_rate = rospy.Rate(10)

    # Initialize TF buffer and TF2 listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Create Marker message structure and initialize any sequential header variables
    icon = Marker()

    # One-time settings for icons (markers)
    # Marker controls
    icon.type = Marker.SPHERE
    icon.action = Marker.ADD

    # Marker scaling settings
    icon.scale.x = 0.1
    icon.scale.y = 0.1
    icon.scale.z = 0.1

    # Marker color settings
    icon.color.r = 1.0
    icon.color.g = 1.0
    icon.color.b = 1.0
    icon.color.a = 1.0

    # Marker lifetime setting
    icon.lifetime = rospy.Duration(1.25)

    while not rospy.is_shutdown():

        # Wait for the listener to find valid TF data - BLOCKS if it doesn't find valid TF data
        try:
            crank_pos = tf_buffer.lookup_transform('world','EE',rospy.Time(),rospy.Duration(2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Warning: TF2 Lookup Error")
            update_rate.sleep()
            continue

        # Set header information for markers based on sourced transformation data (NOTE: Time is pulled straight from
        # transform information
        icon.header.stamp = crank_pos.header.stamp
        icon.header.frame_id = crank_pos.header.frame_id

        # Marker position information for last transform info
        icon.pose.position.x = crank_pos.transform.translation.x
        icon.pose.position.y = crank_pos.transform.translation.y
        icon.pose.position.z = crank_pos.transform.translation.z

        crank_marker.publish(icon)
        update_rate.sleep()


def main():

    try:
        crank_path_visualizer()
    except KeyboardInterrupt:
        print "Shutting down"


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
