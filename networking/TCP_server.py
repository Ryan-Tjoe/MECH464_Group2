import socket
import time
import json

class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)  # Allow only one client (robot)
        print(f"Server listening on {self.host}:{self.port}...")

    def accept_client(self):
        conn, addr = self.server_socket.accept()
        print(f"Connected to {addr}")
        return conn
    
    def send_data(self, conn: socket, data):
        print(f"Sending data: {data}")
        json_data = json.dumps(data)
        conn.sendall(json_data.encode("utf-8"))
        conn.sendall(b"\n")
        print("Data sent successfully.")

    def close(self):
        self.server_socket.close()
        print("Server socket closed.")

if __name__ == "__main__":
    HOST = "0.0.0.0"  # Listen on all available network interfaces
    PORT = 5000        # Port number (can be changed)

    server = Server(HOST, PORT)
    conn = server.accept_client()

    while True:
        try:
            server.send_data(conn, {"x": 10.5, "y": -3.2, "z": 1.7})
            time.sleep(0.1)  # Simulate real-time transmission
        except KeyboardInterrupt:
            print("Server shutting down.")
            break

# HOST = "0.0.0.0"  # Listen on all available network interfaces
# PORT = 5000        # Port number (can be changed)

# def start_server():
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_socket.bind((HOST, PORT))
#     server_socket.listen(1)  # Allow only one client (robot)
#     print(f"Server listening on {HOST}:{PORT}...")

#     conn, addr = server_socket.accept()
#     print(f"Connected to {addr}")

#     # TODO: Initialize the pose estimator here and realsense camera

#     try:
#         while True:

#             # TODO: Camera captures image and passes it to the pose estimator. Pose estimator will output some data (coordiantes or numbers)

#             # We will also use the coordinates of the joint found by the pose estimator to find the depth at each joint

#             # 

#             # 
#             # 
#             data = {"x": 10.5, "y": -3.2, "z": 1.7}
#             print(f"Sending data: {data}")
#             json_data = json.dumps(data)

#             conn.sendall(json_data.encode("utf-8"))  # Send data
#             # send new line
#             conn.sendall(b"\n") # Must be sent to indicate end of message
#             time.sleep(0.1)  # Simulate real-time transmission
#     except KeyboardInterrupt:
#         print("Server shutting down.")
#     finally:
#         conn.close()
#         server_socket.close()

# def convert_pixels_to_m():
#     # TODO: we need to find a way to convert the pixel values in x and y to meters. 
#     pass



# if __name__ == "__main__":
#     start_server()
