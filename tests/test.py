from api.listener.listener import CameraFrameReceivedEvent
import api.listener.listener

def handle_frame(event):
    if isinstance(event, CameraFrameReceivedEvent):
        print(event.name)

def reg():
    print("Registering test listener...")
    api.listener.listener.get_registry().register_listener(handle_frame)