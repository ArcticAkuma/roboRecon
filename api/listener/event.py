from matplotlib.pyplot import connect

from api.util.util import ConnectionState

class Event:
    """Basic event class every event extends from."""

    def __init__(self, name: str):
        self.name = name

    def __repr__(self):
        return f"Event(name={self.name})"


class SocketStateChangeEvent(Event):
    def __init__(self, state: ConnectionState):
        super().__init__("SocketStateChangeEvent")
        self.state = state


class SocketDataReceivedEvent(Event):
    """Event called when client receives a new frame by the server."""

    def __init__(self, channel: str, data):
        super().__init__("SocketDataReceivedEvent")
        self.channel = channel
        self.data = data
