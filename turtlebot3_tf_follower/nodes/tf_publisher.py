#!/usr/bin/env python  
import roslib
roslib.load_manifest('turtlebot3_tf_follower')
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import tf
#import turtlesim.msg

def handle_tb3_pose(msg, tb3name):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     #tf.transformations.quaternion_from_euler(0, 0, msg.rotation.theta),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     tb3name,
                     "/world")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    tb3name = rospy.get_param('~turtle')
    #parent_name = rospy.get_param('~parent')
    rospy.Subscriber('/%s/pose' % tb3name,
                     nav_msgs.msg.Odometry,
                     handle_tb3_pose,
                     tb3name)
    rospy.spin()