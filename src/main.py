from api.tcp.client import VideoStreamClient
from src.yolo import Yolo

# todo: config
host = "127.0.0.1"
port = 3233

if __name__ == "__main__":
    print(f"Trying to connect to server at {host}:{port}..")
    client = VideoStreamClient(host, port)
    client.start()

    print("Starting YOLO service..")
    Yolo()

    cmd_line = ""
    while cmd_line != "exit" and cmd_line != "quit":
        cmd_line = input("\n\nType exit to exit\n")

    print("Closing applications...")
    client.stop()

# Todo: create application (system part) object with start and stop functionality
