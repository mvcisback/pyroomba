"""

"""

class SerialCommandInterface(object):
    """A higher-level wrapper around PySerial specifically designed for use with
    iRobot's SCI.

    """
    def __init__(self, tty, baudrate):
        self.ser = serial.Serial(tty, baudrate=baudrate, timeout=SERIAL_TIMEOUT)
        try:
            self.ser.open()
        except Exception:
            pass
        self.opcodes = {}
        self.lock = threading.RLock()

    def Wake(self):
        """Wake up robot."""
        self.ser.setRTS(0)
        time.sleep(0.25)
        self.ser.setRTS(1)
        time.sleep(1)  # Technically it should wake after 500ms.

    def AddOpcodes(self, opcodes):
        """Add available opcodes to the SCI."""
        self.opcodes.update(opcodes)

    def Send(self, bytes):
        """Send a string of bytes to the robot."""
        with self.lock:
            self.ser.write(struct.pack('B' * len(bytes), *bytes))

    def Read(self, num_bytes):
        """Read a string of 'num_bytes' bytes from the robot."""
        logging.debug('Attempting to read %d bytes from SCI port.' % num_bytes)
        with self.lock:
            data = self.ser.read(num_bytes)
        logging.debug('Read %d bytes from SCI port.' % len(data))
        if not data:
            raise PyRobotError('Error reading from SCI port. No data.')
        if len(data) != num_bytes:
            raise PyRobotError('Error reading from SCI port. Wrong data length.')
        return data

    def FlushInput(self):
        """Flush input buffer, discarding all its contents."""
        logging.debug('Flushing serial input buffer.')
        self.ser.flushInput()

    def __getattr__(self, name):
        """Creates methods for opcodes on the fly.

        Each opcode method sends the opcode optionally followed by a string of
        bytes.

        """
        if name in self.opcodes:
            def SendOpcode(*bytes):
                logging.debug('Sending opcode %s.' % name)
                self.Send([self.opcodes[name]] + list(bytes))
            return SendOpcode
        raise AttributeError
