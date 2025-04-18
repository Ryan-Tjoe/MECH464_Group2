#!/usr/bin/env python3

import socket
import json
import time
import random


class Client:
    def __init__(self, host="127.0.0.1", port=5000):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        try:
            self.sock = socket.create_connection((self.host, self.port))
            print(f"Connected to {self.host}:{self.port}")
        except ConnectionRefusedError:
            print("Could not connect to server. Make sure the server is running.")
            self.sock = None

    def disconnect(self):
        if self.sock:
            self.sock.close()
            print("Connection closed.")

    def generate_mock_angles(self):
        """
        Simulates generation of shoulder and elbow angles in radians.
        Replace this method with real sensor or model input as needed.
        """
        shoulder_angle = random.uniform(-1.5, 1.5)
        elbow_angle = random.uniform(0.0, 2.5)
        return shoulder_angle, elbow_angle

    def send_data(self, shoulder_angle, elbow_angle):
        if not self.sock:
            print("Not connected to server.")
            return

        data = {
            "shoulder_angle": shoulder_angle,
            "elbow_angle": elbow_angle
        }

        try:
            json_str = json.dumps(data) + "\n"
            self.sock.sendall(json_str.encode('utf-8'))
            print(f"Sent: {data}")
        except Exception as e:
            print(f"Failed to send data: {e}")


if __name__ == "__main__":
    client = Client(host="127.0.0.1", port=5000)
    client.connect()

    client.send_data(0.5, 1.0)  # Example data
    time.sleep(1)  # Wait for a second before sending more data

    client.disconnect()
