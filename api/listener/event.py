class Event:
    """Basic event class every event extends from."""

    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return f"Event(name={self.name})"


class FrameReceivedEvent(Event):
    """Event called when client receives a new frame by the server."""

    def __init__(self, frame):
        super().__init__("CameraFrameReceivedEvent")
        self.frame = frame
