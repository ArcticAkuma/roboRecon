from api.listener.event import FrameReceivedEvent
from api.listener.listener import ListenerRegistry, Listener
import api.listener
from ultralytics import YOLO
import cv2


class Yolo(Listener):
    def __init__(self):
        super().__init__()

        # Registration of this class as a listener
        api.listener.listener.get_registry().register_listener(self)

        # Preparation of YOLO model
        self.model = YOLO("../../tests/yolo-Weights/yolov8n.pt")
        self.classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                           "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                           "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
                           "umbrella",
                           "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
                           "baseball bat",
                           "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
                           "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                           "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                           "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                           "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                           "teddy bear", "hair drier", "toothbrush"
                           ]

    def handleEvent(self, event):
        if not isinstance(event, FrameReceivedEvent):
            return

        # Frame resizing, analyzing with YOLO and displaying
        resized_frame = cv2.resize(event.frame, (640, 480))
        results = self.model(resized_frame)
        annotated_frame = results[0].plot()
        cv2.imshow("YOLO Detection on TCP Stream", annotated_frame)


def start():
    Yolo()
