import socket
import struct

import cv2

import api.listener.listener
from api.listener.event import FrameReceivedEvent
from api.util import util


class VideoStreamClient:
    def __init__(self, host, port):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((host, port))
        self.data = b""
        self.payload_size = struct.calcsize("Q")

    def read(self):
        # Read the frame size
        while len(self.data) < self.payload_size:
            packet = self.client_socket.recv(4096)
            if not packet:
                return
            self.data += packet

        packed_msg_size = self.data[:self.payload_size]
        self.data = self.data[self.payload_size:]
        msg_size = struct.unpack("Q", packed_msg_size)[0]

        # Read the frame data
        while len(self.data) < msg_size:
            self.data += self.client_socket.recv(4096)

        # Decode frame and broadcast via ListenerRegistry
        self.data, frame = util.decode_data(self.data, msg_size)
        api.listener.listener.get_registry().notify_listeners(FrameReceivedEvent(frame))
        # frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

    def release(self):
        self.client_socket.close()


def start_client(host='127.0.0.1', port=3233):
    stream = VideoStreamClient(host, port)

    try:
        while True:
            stream.read()
            if cv2.waitKey(1) & 0xFF == 27:  # Exit on ESC key
                break
    finally:
        stream.release()

# if __name__ == "__main__":
#     from src.yolo import Yolo
#
#     Yolo()
#     stream = VideoStreamClient("127.0.0.1", 3233)
#
#     try:
#         while True:
#             stream.read()
#             if cv2.waitKey(1) & 0xFF == 27:  # Exit on ESC key
#                 break
#     finally:
#         stream.release()
#         cv2.destroyAllWindows()
