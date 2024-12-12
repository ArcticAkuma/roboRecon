from api.listener.event import SocketDataReceivedEvent
import api.listener.listener

def handle_frame(event):
    if isinstance(event, SocketDataReceivedEvent):
        print(event.name)

def reg():
    print("Registering test listener...")
    api.listener.listener.get_registry().register_listener(handle_frame)