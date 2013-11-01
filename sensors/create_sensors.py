import logging, struct
from .roomba_sensors import RoombaSensors

OI_MODES = (
    'off',
    'passive',
    'safe',
    'full')

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

    def GetDistance(self):
        bytes = self.RequestSensor(19, 2)
        distance = struct.unpack('>h', bytes)[0]
        return distance

    def GetAngle(self):
        bytes = self.RequestSensor(20, 2)
        angle = struct.unpack('>h', bytes)[0]
        return angle

    def GetBump(self):
        bytes = self.RequestSensor(7, 1)
        bumps = struct.unpack('B', bytes)[0]
        if bool(bumps & 0x03) != 0:
            return True
        else:
            return False

    def GetWall(self):
        bytes = self.RequestSensor(27, 2)
        wall = struct.unpack('>H', bytes)[0]
        return wall

    def GetBatteryCharge(self):
        bytes = self.RequestSensor(25, 2)
        charge = struct.unpack('>H', bytes)[0]
        return charge

    def GetAnalogInput(self):
        bytes = self.RequestSensor(33, 2)
        analog_input = struct.unpack('>H', bytes)[0]
        return analog_input

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
