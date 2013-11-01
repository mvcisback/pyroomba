import logging, struct
from functools import wraps
from .roomba_sensors import RoombaSensors

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
            return func(data)
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
    def GetDistance(dist):
        return dist

    @_get_sensor(20, 2, '>h')
    def GetAngle(angle):
        return angle

    @_get_sensor(7, 1, 'B')
    def GetBump(bumps):
        return bool(bumps & 0x03) != 0:

    @_get_sensor(27, 2, '>H')
    def GetWall(wall):
        return wall

    @_get_sensor(25, 2, '>H')
    def GetBatteryCharge(charge):
        return charge

    @_get_sensor(33, 2, '>H')
    def GetAnalogInput(analog_in):
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
