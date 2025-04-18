#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
import socket
import json

HOST = "0.0.0.0"
PORT = 5000

def joint_goal_tcp_server():
    rospy.init_node('examplePub', anonymous=False)
    pub = rospy.Publisher('/JointMatchingController/joint_goal', Vector3, queue_size=10)

    # TCP Server Setup
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    rospy.loginfo(f"Server listening on {HOST}:{PORT}... Waiting for OpenPose data.")

    conn, addr = server_socket.accept()
    rospy.loginfo(f"Connected to {addr}")

    buffer = ""

    with conn:
        try:
            while not rospy.is_shutdown():
                data = conn.recv(1024).decode("utf-8")
                if not data:
                    rospy.logwarn("Connection closed by client.")
                    break

                buffer += data

                # Handle partial data/messages
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    angles = json.loads(line)

                    shoulder_angle = angles["shoulder_angle"]
                    elbow_angle = angles["elbow_angle"]

                    # Z remains unused here (but available if needed)
                    vector = Vector3(x=shoulder_angle, y=elbow_angle, z=0.0)
                    rospy.loginfo(f"Publishing angles: Shoulder={shoulder_angle:.3f} rad, Elbow={elbow_angle:.3f} rad")
                    pub.publish(vector)

        except rospy.ROSInterruptException:
            rospy.loginfo("ROS Interrupt detected. Shutting down gracefully.")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
        finally:
            server_socket.close()
            rospy.loginfo("Server socket closed.")

if __name__ == '__main__':
    joint_goal_tcp_server()
