"""
License
"""
import time
import struct
import threading
import logging
import serial
import socket

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

SERIAL_TIMEOUT = 2  # Number of seconds to wait for reads. 2 is generous.

class SerialController(Controller):
    """A higher-level wrapper around PySerial specifically designed for use with
    iRobot's SCI."""
    def __init__(self, tty, baudrate):
        Controller.__init__(self)
        self.ser = serial.Serial(tty, baudrate=baudrate, timeout=SERIAL_TIMEOUT)
        self.ser.open()

    def _send(self, msg):
        return self.ser.write(msg)

    def _recv(self, num):
        return self.ser.read(num)

    def wake(self):
        """Wake up robot."""
        self.ser.setRTS(0)
        time.sleep(0.25)
        self.ser.setRTS(1)
        time.sleep(1)  # Technically it should wake after 500ms.

    def flush_input(self):
        """Flush input buffer, discarding all its contents."""
        logging.debug('Flushing serial input buffer.')
        self.ser.flushInput()
