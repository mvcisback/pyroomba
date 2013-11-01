import socket
from .controller import Controller, PyRobotControllerError

class BluetoothController(Controller):
    """
    """
    def __init__(self, mac, port=1):
        Controller.__init__(self)
        try:
            self.conn = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                      socket.BTPROTO_RFCOMM)
            self.conn.connect((mac, port))
        except socket.error.ConnectionResetError:
            raise PyRobotControllerError('Failed to connect via bluetooth')

        self._recv = self.conn.recv
        self._send = self.conn.send

