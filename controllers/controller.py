class PyRobotControllerError(Exception):
    pass

class Controller(object):
    """
    """
    
    def __init__(self, ):
        self.opcodes = {}
        self.lock = threading.RLock()

    def Wake(self):
        """Wake up robot."""
        raise NotImplementedError()

    def AddOpcodes(self, opcodes):
        """Add available opcodes to the SCI."""
        self.opcodes.update(opcodes)

    def _send(self, msg):
        raise NotImplementedError()
        
    def Send(self, byte_str):
        """Send a string of bytes to the robot."""
        with self.lock:
            self._send(struct.pack('B' * len(byte_str), *byte_str))

    def _recv(self, num_bytes):
        raise NotImplementedError()

    def Read(self, num_bytes):
        """Read a string of 'num_bytes' bytes from the robot."""
        logging.debug('Attempting to read %d bytes from SCI port.' % num_bytes)
        with self.lock:
            data = self._recv(num_bytes)
        logging.debug('Read %d bytes from SCI port.' % len(data))
        if not data:
            raise PyRobotControllerError(
                'Error reading from SCI port. No data.')
        if len(data) != num_bytes:
            raise PyRobotControllerError(
                'Error reading from SCI port. Wrong data length.')
        return data

    def FlushInput(self):
        """Flush input buffer, discarding all its contents."""
        raise NotImplementedError()

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
        

        
