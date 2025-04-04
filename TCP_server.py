import socket
import time
import json

HOST = "0.0.0.0"  # Listen on all available network interfaces
PORT = 5000        # Port number (can be changed)

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)  # Allow only one client (robot)
    print(f"Server listening on {HOST}:{PORT}...")

    conn, addr = server_socket.accept()
    print(f"Connected to {addr}")

    try:
        while True:
            # Generate fake XYZ data (Replace with real ML estimation)
            data = {"x": 10.5, "y": -3.2, "z": 1.7}
            print(f"Sending data: {data}")
            json_data = json.dumps(data)

            conn.sendall(json_data.encode("utf-8"))  # Send data
            # send new line
            conn.sendall(b"\n") # Must be sent to indicate end of message
            time.sleep(0.1)  # Simulate real-time transmission
    except KeyboardInterrupt:
        print("Server shutting down.")
    finally:
        conn.close()
        server_socket.close()

if __name__ == "__main__":
    start_server()
