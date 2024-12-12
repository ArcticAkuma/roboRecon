import pickle
import struct
import zlib
from io import BytesIO

import numpy as np
from PIL import Image


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

def img_to_binary(img, format='jpeg'):
    '''
    accepts: PIL image
    returns: binary stream (used to save to database)
    '''
    f = BytesIO()
    try:
        img.save(f, format=format)
    except Exception as e:
        raise e
    return f.getvalue()


def arr_to_binary(arr):
    '''
    accepts: numpy array with shape (Hight, Width, Channels)
    returns: binary stream (used to save to database)
    '''
    img = arr_to_img(arr)
    return img_to_binary(img)


def arr_to_img(arr):
    '''
    accepts: numpy array with shape (Height, Width, Channels)
    returns: binary stream (used to save to database)
    '''
    arr = np.uint8(arr)
    img = Image.fromarray(arr)
    return img


def img_to_arr(img):
    '''
    accepts: PIL image
    returns: a numpy uint8 image
    '''
    return np.array(img)


def binary_to_img(binary):
    '''
    accepts: binary file object from BytesIO
    returns: PIL image
    '''
    if binary is None or len(binary) == 0:
        return None

    img = BytesIO(binary)
    try:
        img = Image.open(img)
        return img
    except:
        return None
