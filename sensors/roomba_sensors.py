import struct
import logging
import traceback

CHARGING_STATES = (
    'not-charging',
    'charging-recovery',
    'charging',
    'trickle-charging',
    'waiting',
    'charging-error')

REMOTE_OPCODES = {
    # Remote control.
    129: 'left',
    130: 'forward',
    131: 'right',
    132: 'spot',
    133: 'max',
    134: 'small',
    135: 'medium',
    136: 'large/clean',
    137: 'pause',
    138: 'power',
    139: 'arc-left',
    140: 'arc-right',
    141: 'drive-stop',
    # Scheduling remote.
    142: 'send-all',
    143: 'seek-dock',
    # Home base.
    240: 'reserved',
    242: 'force-field',
    244: 'green-buoy',
    246: 'green-buoy-and-force-field',
    248: 'red-buoy',
    250: 'red-buoy-and-force-field',
    252: 'red-buoy-and-green-buoy',
    254: 'red-buoy-and-green-buoy-and-force-field',
    255: 'none',
}


SENSOR_GROUP_PACKET_LENGTHS = (26, 10, 6, 10, 14, 12, 52)

class RoombaSensorError(Exception):
    pass

class RoombaSensors(dict):

    """Retrive and decode the Roomba's sensor data.

    Some of the specification is included in the docstrings. For a complete
    description, see the Roomba SCI Sepc Manual.

    """
    def __init__(self, robot):
        dict.__init__(self)
        self.robot = robot
        self.data = {}  # Last sensor readings.

    def Clear(self):
        """Clear out old sensor data."""
        self.clear()

    def _MakeHumanReadable(self, sensor, mapping):
        """Change a sensor value to it's human readable form."""
        try:
            self[sensor] = mapping[self[sensor]]
        except (KeyError, IndexError):
            logging.debug(traceback.format_exc())
            raise RoombaSensorError('Invalid sensor data.')

    def _DecodeGroupPacket0(self, bytes):
        """Decode sensord data from a request for group 0 (all data)."""
        # NOTE(damonkohler): We decode sensor data in reverse order for better pop
        # performance.
        self.DecodeUnsignedShort('capacity', bytes.pop(), bytes.pop())  # mAh
        self.DecodeUnsignedShort('charge', bytes.pop(), bytes.pop())  # mAh
        self.DecodeByte('temperature', bytes.pop())  # C
        self.DecodeShort('current', bytes.pop(), bytes.pop())  # mA
        self.DecodeUnsignedShort('voltage', bytes.pop(), bytes.pop())  # mV
        self.DecodeUnsignedByte('charging-state', bytes.pop())
        self._MakeHumanReadable('charging-state', CHARGING_STATES)
        self.Angle(bytes.pop(), bytes.pop(), 'degrees')
        self.DecodeShort('distance', bytes.pop(), bytes.pop())  # mm
        self.Buttons(bytes.pop())
        self.DecodeUnsignedByte('remote-opcode', bytes.pop())
        self._MakeHumanReadable('remote-opcode', REMOTE_OPCODES)
        self.DecodeUnsignedByte('dirt-detector-right', bytes.pop())
        self.DecodeUnsignedByte('dirt-detector-left', bytes.pop())
        self.MotorOvercurrents(bytes.pop())
        self.DecodeBool('virtual-wall', bytes.pop())
        self.DecodeBool('cliff-right', bytes.pop())
        self.DecodeBool('cliff-front-right', bytes.pop())
        self.DecodeBool('cliff-front-left', bytes.pop())
        self.DecodeBool('cliff-left', bytes.pop())
        self.DecodeBool('wall', bytes.pop())
        self.BumpsWheeldrops(bytes.pop())

    def RequestPacket(self, packet_id):
        """Reqeust a sesnor packet."""
        with self.robot.sci.lock:
            logging.debug('Requesting sensor packet %d.' % packet_id)
            self.robot.sci.FlushInput()
            self.robot.sci.sensors(packet_id)
            length = SENSOR_GROUP_PACKET_LENGTHS[packet_id]
            data = list(self.robot.sci.Read(length))
            return data

    def GetAll(self):
        """Request and decode all available sensor data."""
        bytes = self.RequestPacket(0)
        if bytes is not None:
            self._DecodeGroupPacket0(bytes)

    def Angle(self, low, high, unit=None):
        """The angle that Roomba has turned through since the angle was last
        requested. The angle is expressed as the difference in the distance
        traveled by Roomba's two wheels in millimeters, specifically the right
        wheel distance minus the left wheel distance, divided by two. This makes
        counter-clockwise angles positive and clockwise angles negative. This can
        be used to directly calculate the angle that Roomba has turned through
        since the last request. Since the distance between Roomba's wheels is
        258mm, the equations for calculating the angles in familiar units are:

        Angle in radians = (2 * difference) / 258
        Angle in degrees = (360 * difference) / (258 * Pi).

        If the value is not polled frequently enough, it will be capped at its
        minimum or maximum.

        Note: Reported angle and distance may not be accurate. Roomba measures
        these by detecting its wheel revolutions. If for example, the wheels slip
        on the floor, the reported angle of distance will be greater than the
        actual angle or distance.

        """
        if unit not in (None, 'radians', 'degrees'):
            raise RoombaSensorError('Invalid angle unit specified.')
        self.DecodeShort('angle', low, high)
        if unit == 'radians':
            self['angle'] = (2 * self['angle']) / 258
        if unit == 'degrees':
            self['angle'] #/= math.pi

    def BumpsWheeldrops(self, byte):
        """The state of the bump (0 = no bump, 1 = bump) and wheeldrop sensors
        (0 = wheel up, 1 = wheel dropped) are sent as individual bits.

        Note: Some robots do not report the three wheel drops separately. Instead,
        if any of the three wheels drops, all three wheel-drop bits will be set.
        You can tell which kind of robot you have by examining the serial number
        inside the battery compartment. Wheel drops are separate only if there
        is an 'E' in the serial number.

        """
        byte = struct.unpack('B', byte)[0]
        self.update({
            'wheel-drop-caster': bool(byte & 0x10),
            'wheel-drop-left': bool(byte & 0x08),
            'wheel-drop-right': bool(byte & 0x04),
            'bump-left': bool(byte & 0x02),
            'bump-right': bool(byte & 0x01)})

    def MotorOvercurrents(self, byte):
        """The state of the five motors overcurrent sensors are sent as individual
        bits (0 = no overcurrent, 1 = overcurrent).

        """
        byte = struct.unpack('B', byte)[0]
        self.update({
            'drive-left': bool(byte & 0x10),
            'drive-right': bool(byte & 0x08),
            'main-brush': bool(byte & 0x04),
            'vacuum': bool(byte & 0x02),
            'side-brush': bool(byte & 0x01)})

    def Buttons(self, byte):
        """The state of the four Roomba buttons are sent as individual bits
        (0 = button not pressed, 1 = button pressed).

        """
        byte = struct.unpack('B', byte)[0]
        self.update({
            'power': bool(byte & 0x08),
            'spot': bool(byte & 0x04),
            'clean': bool(byte & 0x02),

            'max': bool(byte & 0x01)})

    def DecodeBool(self, name, byte):
        """Decode 'byte' as a bool and map it to 'name'."""
        self[name] = bool(struct.unpack('B', byte)[0])

    # NOTE(damonkohler): We specify the low byte first to make it easier when
    # popping bytes off a list.
    def DecodeUnsignedShort(self, name, low, high):
        """Map an unsigned short from a 'high' and 'low' bytes to 'name'."""
        self[name] = struct.unpack('>H', high + low)[0]

    def DecodeShort(self, name, low, high):
        """Map a short from a 'high' and 'low' bytes to 'name'."""
        self[name] = struct.unpack('>h', high + low)[0]

    def DecodeByte(self, name, byte):
        """Map signed 'byte' to 'name'."""
        self[name] = struct.unpack('b', byte)[0]

    def DecodeUnsignedByte(self, name, byte):
        """Map unsigned 'byte' to 'name'."""
        self[name] = struct.unpack('B', byte)[0]
