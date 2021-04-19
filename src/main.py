import time
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String

def load_data(data_types_list):
    for dt in data_types_list:
        filepath = '../data/train/{}.npy'.format(dt)
        if dt == "timestamp":
            vars()[dt] = np.load(filepath)[:, np.newaxis]
        else:
            vars()[dt] = np.load(filepath)
    return vars()

if __name__=="__main__":
    # Load data
    data_types_list = ["action", "angular_vel", "position", "rotation", "timestamp", "vel"]
    vars = load_data(data_types_list)
    print("Loaded data, starting ros")

    # Start ros
    node = rospy.init_node("ros_npy_visualizer")

    # Make topic publishers
    action_publisher = rospy.Publisher("npy_joystick_action", TwistStamped, queue_size=10)
    vel_publisher = rospy.Publisher("npy_vel", TwistStamped, queue_size=10)
    position_publisher = rospy.Publisher("npy_pose", PoseStamped, queue_size=10)

    loop_period = 0.01
    d_len = len(vars["action"])

    for i in range(d_len):
        t1 = time.time()

        # Action
        action_msg = TwistStamped()
        action_msg.header.stamp = rospy.Time.now()
        action_msg.twist.linear.x = vars["action"][0]
        action_msg.twist.angular.z = vars["action"][1]
        action_publisher.publish(action_msg)

        # Velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.twist.linear.x = vars["vel"][0]
        vel_msg.twist.linear.y = vars["vel"][1]
        vel_msg.twist.linear.z = vars["vel"][2]
        vel_msg.twist.angular.x = vars["action"][0]
        vel_msg.twist.angular.y = vars["action"][1]
        vel_msg.twist.angular.z = vars["action"][2]
        vel_publisher.publish(vel_msg)

        # Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = vars["position"][0]
        pose_msg.pose.position.y = vars["position"][1]
        pose_msg.pose.position.z = vars["position"][2]
        pose_msg.pose.orientation.x = vars["rotation"][0]
        pose_msg.pose.orientation.y = vars["rotation"][1]
        pose_msg.pose.orientation.z = vars["rotation"][2]
        pose_msg.pose.orientation.w = vars["rotation"][3]
        position_publisher.publish(pose_msg)

        t2 = time.time()
        while (t2 - t1) < loop_period: pass

    print("Done")



