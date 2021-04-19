import time
import numpy as np
import rospy
import tf2_ros
import tf
import tf2_geometry_msgs
import tf2_msgs.msg
from geometry_msgs.msg import Twist, TwistStamped, Pose, PoseStamped, Vector3Stamped, TransformStamped, Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import tf.transformations
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
    rate = rospy.Rate(100)

    # Make topic publishers
    action_publisher = rospy.Publisher("npy_joystick_action", Imu, queue_size=10)
    vel_publisher = rospy.Publisher("npy_vel", Imu, queue_size=10)
    position_publisher = rospy.Publisher("npy_pose", PoseStamped, queue_size=10)

    br = tf2_ros.TransformBroadcaster()
    d_len = len(vars["action"])

    for i in range(d_len):
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "origin"
        t.child_frame_id = "buggy"
        t.transform.translation.x = float(vars["position"][i][0])
        t.transform.translation.y = float(vars["position"][i][1])
        t.transform.translation.z = float(vars["position"][i][2])

        t.transform.rotation.w = float(vars["rotation"][i][0])
        t.transform.rotation.x = float(vars["rotation"][i][1])
        t.transform.rotation.y = float(vars["rotation"][i][2])
        t.transform.rotation.z = float(vars["rotation"][i][3])

        br.sendTransform(t)

        # Action
        action_msg = Imu()
        action_msg.header.frame_id = "buggy"
        action_msg.header.stamp = rospy.Time.now()
        action_msg.linear_acceleration.x = float(vars["action"][i][0])
        action_msg.angular_velocity.z = float(vars["action"][i][1])
        action_publisher.publish(action_msg)

        # Velocity_corrected
        velocity = [float(vars["vel"][i][0]), float(vars["vel"][i][1]), float(vars["vel"][i][2])]
        orientation = [float(vars["rotation"][i][1]), float(vars["rotation"][i][2]), float(vars["rotation"][i][3]), float(vars["rotation"][i][0])]
        orientation_matrix = tf.transformations.quaternion_matrix(orientation)
        velocity_corrected = np.matmul(orientation_matrix[:3,:3].T, velocity)

        vel_msg = Imu()
        vel_msg.header.frame_id = "buggy"
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.linear_acceleration.x = velocity_corrected[0]
        vel_msg.linear_acceleration.y = velocity_corrected[1]
        vel_msg.linear_acceleration.z = velocity_corrected[2]
        vel_msg.angular_velocity.x = float(vars["angular_vel"][i][0])
        vel_msg.angular_velocity.y = float(vars["angular_vel"][i][1])
        vel_msg.angular_velocity.z = float(vars["angular_vel"][i][2])
        vel_publisher.publish(vel_msg)

        # Pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "origin"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = float(vars["position"][i][0])
        pose_msg.pose.position.y = float(vars["position"][i][1])
        pose_msg.pose.position.z = float(vars["position"][i][2])
        pose_msg.pose.orientation.w = float(vars["rotation"][i][0])
        pose_msg.pose.orientation.x = float(vars["rotation"][i][1])
        pose_msg.pose.orientation.y = float(vars["rotation"][i][2])
        pose_msg.pose.orientation.z = float(vars["rotation"][i][3])
        position_publisher.publish(pose_msg)

        if i % 100 == 0:
            print(i)
        rate.sleep()

    print("Done")



