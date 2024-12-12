from donkey.network import TCPServeValue
from donkey.image import ImgArrToJpg

pub = TCPServeValue("camera")
V.add(ImgArrToJpg(), inputs=['cam/image_array'], outputs=['jpg/bin'])
V.add(pub, inputs=['jpg/bin'])