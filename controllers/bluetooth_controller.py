class BluetoothController(object):
    """
    """
    def __init__(self, mac, port=1):
        try:
            self.conn = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                      socket.BTPROTO_RFCOMM)
            self.conn.connect((mac, port))
        except ConnectionResetError:
            raise PyRobotError('Failed to connect via bluetooth')
        self.opcodes = {}
        self.lock = threading.RLock()

    def Wake(self):
        """Wake up robot."""
        pass

    def AddOpcodes(self, opcodes):
        """Add available opcodes to the SCI."""
        self.opcodes.update(opcodes)

    def Send(self, bytes):
        """Send a string of bytes to the robot."""
        with self.lock:
            self.conn.send(struct.pack('B' * len(bytes), *bytes))

    def Read(self, num_bytes):
        """Read a string of 'num_bytes' bytes from the robot."""
        logging.debug('Attempting to read %d bytes from SCI port.' % num_bytes)
        with self.lock:
            data = self.conn.recv(num_bytes)
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
