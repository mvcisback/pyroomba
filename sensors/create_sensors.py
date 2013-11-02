import logging, struct
from functools import wraps
from .roomba_sensors import RoombaSensors

# From: http://www.harmony-central.com/MIDI/Doc/table2.html
MIDI_TABLE = {
    'rest': 0, 'R': 0, 'pause': 0,
    'G1': 31, 'G#1': 32, 'A1': 33,
    'A#1': 34, 'B1': 35,

    'C2': 36, 'C#2': 37, 'D2': 38,
    'D#2': 39, 'E2': 40, 'F2': 41,
    'F#2': 42, 'G2': 43, 'G#2': 44,
    'A2': 45, 'A#2': 46, 'B2': 47,

    'C3': 48, 'C#3': 49, 'D3': 50,
    'D#3': 51, 'E3': 52, 'F3': 53,
    'F#3': 54, 'G3': 55, 'G#3': 56,
    'A3': 57, 'A#3': 58, 'B3': 59,

    'C4': 60, 'C#4': 61, 'D4': 62,
    'D#4': 63, 'E4': 64, 'F4': 65,
    'F#4': 66, 'G4': 67, 'G#4': 68,
    'A4': 69, 'A#4': 70, 'B4': 71,

    'C5': 72, 'C#5': 73, 'D5': 74,
    'D#5': 75, 'E5': 76, 'F5': 77,
    'F#5': 78, 'G5': 79, 'G#5': 80,
    'A5': 81, 'A#5': 82, 'B5': 83,

    'C6': 84, 'C#6': 85, 'D6': 86,
    'D#6': 87, 'E6': 88, 'F6': 89,
    'F#6': 90, 'G6': 91, 'G#6': 92,
    'A6': 93, 'A#6': 94, 'B6': 95,

    'C7': 96, 'C#7': 97, 'D7': 98,
    'D#7': 99, 'E7': 100, 'F7': 101,
    'F#7': 102, 'G7': 103, 'G#7': 104,
    'A7': 105, 'A#7': 106, 'B7': 107,

    'C8': 108, 'C#8': 109, 'D8': 110,
    'D#8': 111, 'E8': 112, 'F8': 113,
    'F#8': 114, 'G8': 115, 'G#8': 116,
    'A8': 117, 'A#8': 118, 'B8': 119,

    'C9': 120, 'C#9': 121, 'D9': 122,
    'D#9': 123, 'E9': 124, 'F9': 125,
    'F#9': 126, 'G9': 127
}

OI_MODES = (
    'off',
    'passive',
    'safe',
    'full')

def _get_sensor(sensor, size, byte_t):
    def __get_sensor(func):
        @wraps(func)
        def ___get_sensor(self):
            data = struct.unpack(byte_t, self.RequestSensor(sensor, size))[0]
            return func(self, data)
        return ___get_sensor
    return __get_sensor


class CreateSensors(RoombaSensors):
    """Handles retrieving and decoding the Create's sensor data."""
    def RequestSensor(self, sensor_id, length):
        """Reqeust a sesnor packet."""
        with self.robot.sci.lock:
            logging.debug('Requesting sensor value from %d.' % sensor_id)
            self.robot.sci.FlushInput()
            self.robot.sci.sensors(sensor_id)
            data = self.robot.sci.Read(length)
            return data

    @_get_sensor(19, 2, '>h')
    def GetDistance(self, dist):
        return dist

    @_get_sensor(20, 2, '>h')
    def GetAngle(self, angle):
        return angle

    @_get_sensor(7, 1, 'B')
    def GetBump(self, bumps):
        return bool(bumps & 0x03)

    @_get_sensor(27, 2, '>H')
    def GetWall(self, wall):
        return wall

    @_get_sensor(25, 2, '>H')
    def GetBatteryCharge(self, charge):
        return charge

    @_get_sensor(33, 2, '>H')
    def GetAnalogInput(self, analog_in):
        return analog_in

    def _DecodeGroupPacket6(self, buff):
        """Decode sensor group packet 6."""
        self.DecodeShort('left-velocity', buff.pop(), buff.pop())  # mm/s
        self.DecodeShort('right-velocity', buff.pop(), buff.pop())  # mm/s
        self.DecodeShort('radius', buff.pop(), buff.pop())  # mm
        self.DecodeShort('velocity', buff.pop(), buff.pop())  # mm/s
        self.DecodeUnsignedByte('number-of-stream-packets', buff.pop())
        self.DecodeBool('song-playing', buff.pop())
        self.DecodeUnsignedByte('song-number', buff.pop())
        self.DecodeUnsignedByte('oi-mode', buff.pop())
        self._MakeHumanReadable('oi-mode', OI_MODES)
        self.DecodeUnsignedByte('charging-sources-available', buff.pop())
        self.DecodeUnsignedShort('user-analog-input', buff.pop(), buff.pop())
        self.DecodeUnsignedByte('user-digital-inputs', buff.pop())
        self.DecodeUnsignedShort('cliff-right-signal', buff.pop(), buff.pop())
        self.DecodeUnsignedShort(
            'cliff-front-right-signal', buff.pop(), buff.pop())
        self.DecodeUnsignedShort(
            'cliff-front-left-signal', buff.pop(), buff.pop())
        self.DecodeUnsignedShort('cliff-left-signal', buff.pop(), buff.pop())
        self.DecodeUnsignedShort('wall-signal', buff.pop(), buff.pop())
        self._DecodeGroupPacket0(buff)

    def GetAll(self):
        """Request and decode all available sensor data."""
        bytes = self.RequestPacket(6)
        if bytes is not None:
            self._DecodeGroupPacket6(bytes)
