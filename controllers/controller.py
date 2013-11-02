"""
License
"""

import struct
import threading
import logging

class PyRobotControllerError(Exception):
    """Thrown if the controller runs into an error"""
    pass

class Controller(object):
    """Base clase for high-level wrapper for use withiRobot's SCI."""
    def __init__(self, ):
        self.opcodes = {}
        self.lock = threading.RLock()

    def wake(self):
        """Wake up robot."""
        raise NotImplementedError()

    def add_opcodes(self, opcodes):
        """Add available opcodes to the SCI."""
        self.opcodes.update(opcodes)

    def _send(self, msg):
        """Sends a message to the SCI"""
        raise NotImplementedError()

    def send(self, byte_str):
        """Send a string of bytes to the robot."""
        with self.lock:
            self._send(struct.pack('B' * len(byte_str), *byte_str))

    def _recv(self, num_bytes):
        """Recives a message from the SCI"""
        raise NotImplementedError()

    def read(self, num_bytes):
        """Read a string of 'num_bytes' bytes from the robot."""
        logging.debug('Attempting to read %d bytes from SCI port.', num_bytes)
        with self.lock:
            data = self._recv(num_bytes)
        logging.debug('Read %d bytes from SCI port.', len(data))
        if not data:
            raise PyRobotControllerError(
                'Error reading from SCI port. No data.')
        if len(data) != num_bytes:
            raise PyRobotControllerError(
                'Error reading from SCI port. Wrong data length.')
        return data

    def flush_input(self):
        """Flush input buffer, discarding all its contents."""
        raise NotImplementedError()
