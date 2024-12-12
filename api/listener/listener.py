from abc import ABC, abstractmethod


# Todo: Add priority
class Listener(ABC):
    """Abstraction to ensure proper listener functionality."""
    @abstractmethod
    def handleEvent(self, event):
        pass


class ListenerRegistry:
    def __init__(self):
        self._listeners = []

    def register_listener(self, listener):
        assert isinstance(listener, Listener), "Can not register non-listener object as listener."
        self._listeners.append(listener)

    def notify_listeners(self, event):
        for listener in self._listeners:
            listener.handleEvent(event)

    def unregister_listener(self, listener):
        self._listeners.remove(listener)


registry = None


def get_registry():
    """Ensures only a single ListenerRegistry instance can exist simultaneous."""
    global registry
    if registry is None:
        registry = ListenerRegistry()
    return registry
