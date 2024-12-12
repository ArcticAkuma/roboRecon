import socket
import struct
import threading
import time

import cv2

import api.listener.listener
from api.listener.event import SocketDataReceivedEvent, SocketStateChangeEvent
from api.util import util
import api.util.util

online = True

class VideoStreamClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_socket = None
        self.data = b""
        self.payload_size = struct.calcsize("Q")
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True
        self.connected = False

    def connect(self):
        """Attempts to connect to the server."""
        while online and not self.connected:
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((self.host, self.port))
                self.connected = True
                print("Client connected!")

                api.listener.listener.get_registry().notify_listeners(
                    SocketStateChangeEvent(util.ConnectionState.CONNECTED))
            except socket.error:
                print("Failed to connect. Retrying in 5 seconds...")
                time.sleep(5)

    def read(self):
        """Reads and processes data from the server."""
        try:
            while len(self.data) < self.payload_size:
                packet = self.client_socket.recv(4096)
                if not packet:
                    self.handle_disconnect()
                    return
                self.data += packet

            packed_msg_size = self.data[:self.payload_size]
            self.data = self.data[self.payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]

            while len(self.data) < msg_size:
                self.data += self.client_socket.recv(4096)

            self.data, decoded = util.decode_data(self.data, msg_size)
            if len(decoded) > 0:
                api.listener.listener.get_registry().notify_listeners(
                    SocketDataReceivedEvent(decoded['name'], decoded['val']))
        except socket.error:
            self.handle_disconnect()

    def handle_disconnect(self):
        """Handles disconnection from the server."""
        print("Disconnected from server.")
        self.connected = False
        self.client_socket.close()

        api.listener.listener.get_registry().notify_listeners(
            SocketStateChangeEvent(util.ConnectionState.DISCONNECTED))
        self.connect()

    def release(self):
        """Closes the socket connection."""
        if self.client_socket:
            self.client_socket.close()
            api.listener.listener.get_registry().notify_listeners(
                SocketStateChangeEvent(util.ConnectionState.DISCONNECTED))
        print("Connection closed.")

    def run(self):
        """Main loop for reading data from the server."""
        self.connect()
        try:
            global online
            while online:
                if self.connected:
                    self.read()
                if cv2.waitKey(1) & 0xFF == 27:  # Exit on ESC key
                    break
        finally:
            self.release()

    def start(self):
        """Starts the client in a separate thread."""
        self.thread.start()

    def stop(self):
        """Stops the client."""
        global online
        online = False
        self.release()