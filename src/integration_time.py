from dynamic_reconfigure.client import Client


class IntegrationTime:

    MIN = 30
    MAX = 2000

    def __init__(self):
        self.client = Client('camera/driver', timeout=1)

    def set(self, time):
        time = min(self.MAX, max(self.MIN, time))
        self.client.update_configuration({'integration_time': time})

    def get(self):
        return self.client.get_configuration()['integration_time']

    def add(self, time):
        current = self.get()
        self.set(current + time)
