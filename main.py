from api.tcp.client import VideoStreamClient
from src.yolo import Yolo

import yaml

if __name__ == "__main__":
    with open('config.yml', 'r') as file:
        config = yaml.safe_load(file)

    # Todo: handle errors in config
    hostname = config['donkey-server']['hostname']
    port = config['donkey-server']['port']
    yolo_width = config['yolo']['width']
    yolo_height = config['yolo']['height']
    yolo_verbose = config['yolo']['console-logging']

    # Todo: create application (system part) object with start and stop functionality
    print(f"Trying to connect to server at {hostname}:{port}..")
    client = VideoStreamClient(hostname, port)
    client.start()

    print(f"Starting YOLO service with console-logging == {yolo_verbose}")
    Yolo(yolo_width, yolo_height, yolo_verbose)

    cmd_line = ""
    while cmd_line != "exit" and cmd_line != "quit":
        cmd_line = input("\n\nType exit to exit\n")

    print("Closing applications...")
    client.stop()
