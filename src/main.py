import sys
from pathlib import Path

sys.path.append("src")

from devices.realsense import RealSenseCamera
from networking.TCP_client import Client
from pose_estimator.pose_estimator_final import PoseEstimator

def convert_deg_to_rad(deg):
    """
    Convert degrees to radians.

    :param deg: Angle in degrees
    :return: Angle in radians
    """
    return deg * (3.14159 / 180.0)

if __name__ == "__main__":

    client = Client(host="206.87.198.42", port=5000)
    client.connect()

    camera = RealSenseCamera()
    pose_estimator = PoseEstimator()

    while True:
        color_frame = camera.get_color_image()

        shoulder_angle = pose_estimator.get_shoulder_angle(color_frame)
        elbow_angle = pose_estimator.get_elbow_angle(color_frame)

        shoulder_angle = convert_deg_to_rad(shoulder_angle)
        elbow_angle = convert_deg_to_rad(elbow_angle)
        
        # Send angle dict to robot via TCP
        client.send_data(shoulder_angle, elbow_angle)
        print(f"Shoulder Angle: {shoulder_angle}, Elbow Angle: {elbow_angle}")

    




    
    

    