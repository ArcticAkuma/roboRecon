import pickle
import struct
import zlib
from enum import Enum


def encode_data(data):
    """Encodes data to be sent via TCP socket"""
    raw_data = pickle.dumps(data)
    compressed = zlib.compress(raw_data)
    message = struct.pack("Q", len(compressed)) + compressed
    return message


def decode_data(data, msg_size):
    """Decodes data after being received through TCP socket"""
    raw_data = data[:msg_size]
    data = data[msg_size:]
    decompressed = zlib.decompress(raw_data)
    return data, pickle.loads(decompressed)

class ConnectionState(Enum):
    DISCONNECTED = 0
    CONNECTED = 1