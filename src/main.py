import threading
import api.tcp.server
import api.tcp.client
import src.yolo

if __name__ == "__main__":
    # Starting YOLO in separate thread
    yolo_thread = threading.Thread(target=src.yolo.start)
    yolo_thread.daemon = True  # Ensure the thread exits when the program exits
    yolo_thread.start()
    print("#2")

    # Starting client, initializing connection
    api.tcp.client.start_client("127.0.0.1", 3233)

