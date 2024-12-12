import socket
import cv2
import threading
from api.util import util

"""THIS IS A DUMMY SERVER AND NOT COMPATIBLE WITH CURRENT DONKEY IMPLEMENTATION DUE TO DATA STRUCTURE"""

def handle_client(conn, addr, cap):
    """Handles communication with a single client."""
    print(f"Connection from: {addr}")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame.")
                break
            # Encode frame and send to client
            conn.sendall(util.encode_data(frame))

    except Exception as e:
        print(f"Error: {e}")
    finally:
        conn.close()
        print(f"Connection from {addr} closed.")


def start_server(host='0.0.0.0', port=3233):
    """Starts the server, accepts multiple client connections."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)  # Allows 5 pending connections
    print(f"Server listening on {host}:{port}")

    cap = cv2.VideoCapture(2)

    try:
        while True:
            conn, addr = server_socket.accept()
            # Start a new thread for each client connection
            client_thread = threading.Thread(target=handle_client, args=(conn, addr, cap))
            client_thread.daemon = True  # Ensure the thread exits when the program exits
            client_thread.start()

    except KeyboardInterrupt:
        print("Server shutting down...")
    finally:
        cap.release()
        server_socket.close()
        print("Server closed.")


if __name__ == "__main__":
    start_server()