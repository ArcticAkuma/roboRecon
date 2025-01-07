from abc import ABC, abstractmethod

from api.listener.event import Event


# Todo: Add priority
class Listener(ABC):
    """Abstraction to ensure proper listener functionality."""
    @abstractmethod
    def handleEvent(self, event):
        pass


class ListenerRegistry:
    def __init__(self):
        self._listeners = []

    def register_listener(self, listener: Listener):
        self._listeners.append(listener)

    def notify_listeners(self, event: Event):
        for listener in self._listeners:
            listener.handleEvent(event)

    def unregister_listener(self, listener: Listener):
        self._listeners.remove(listener)


registry = None


def get_registry():
    """Ensures only a single ListenerRegistry instance can exist simultaneous."""
    global registry
    if registry is None:
        registry = ListenerRegistry()
    return registry
