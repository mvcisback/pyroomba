"""

"""
import serial
import logging
import time

from .controller import Controller

SERIAL_TIMEOUT = 2  # Number of seconds to wait for reads. 2 is generous.

class SerialCommandInterface(Controller):
    """A higher-level wrapper around PySerial specifically designed for use with
    iRobot's SCI.

    """
    def __init__(self, tty, baudrate):
        Controller.__init__(self)
        self.ser = serial.Serial(tty, baudrate=baudrate, timeout=SERIAL_TIMEOUT)
        self.ser.open()
        self._send = self.ser.write
        self._recv = self.ser.read

    def Wake(self):
        """Wake up robot."""
        self.ser.setRTS(0)
        time.sleep(0.25)
        self.ser.setRTS(1)
        time.sleep(1)  # Technically it should wake after 500ms.

    def FlushInput(self):
        """Flush input buffer, discarding all its contents."""
        logging.debug('Flushing serial input buffer.')
        self.ser.flushInput()
