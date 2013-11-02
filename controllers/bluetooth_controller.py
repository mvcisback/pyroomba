"""
License
"""

import socket
from .controller import Controller, PyRobotControllerError

class BluetoothController(Controller):
    """"A higher-level wrapper around Bluetooth sockets specifically designed
    for use with iRobot's SCI."""
    def __init__(self, mac, port=1):
        Controller.__init__(self)
        try:
            self.conn = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                      socket.BTPROTO_RFCOMM)
            self.conn.connect((mac, port))
        except socket.error.ConnectionResetError:
            raise PyRobotControllerError('Failed to connect via bluetooth')

    def _send(self, msg):
        return self.conn.send(msg)

    def _recv(self, num):
        return self.conn.recv(num)

    def wake(self):
        pass

    def flush_input(self):
        pass
