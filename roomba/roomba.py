import time
import struct

ROOMBA_OPCODES = {
    'start': 128,
    'baud': 129,
    'control': 130,
    'safe': 131,
    'full': 132,
    'power': 133,
    'spot': 134,
    'clean': 135,
    'max': 136,
    'drive': 137,
    'motors': 138,
    'leds': 139,
    'song': 140,
    'play': 141,
    'sensors': 142,
    'force_seeking_dock': 143,
}

# Drive constants.
RADIUS_TURN_IN_PLACE_CW = -1
RADIUS_TURN_IN_PLACE_CCW = 1
RADIUS_STRAIGHT = 32768
RADIUS_MAX = 2000

VELOCITY_MAX = 500  # mm/s
VELOCITY_SLOW = int(VELOCITY_MAX * 0.33)
VELOCITY_FAST = int(VELOCITY_MAX * 0.66)

WHEEL_SEPARATION = 298  # mm

class RoombaError(Exception):
    pass

class Roomba(object):

    """Represents a Roomba robot."""

    def __init__(self, controller):
        self.sci = controller
        self.sci.AddOpcodes(ROOMBA_OPCODES)
        self.sensors = RoombaSensors(self)
        self.safe = True

    # TODO move into controller
    # def ChangeBaudRate(self, baud_rate):
    #    """Sets the baud rate in bits per second (bps) at which SCI commands and
    #    data are sent according to the baud code sent in the data byte.
    #
    #    The default baud rate at power up is 57600 bps. (See Serial Port Settings,
    #    above.) Once the baud rate is changed, it will persist until Roomba is
    #    power cycled by removing the battery (or until the battery voltage falls
    #    below the minimum required for processor operation). You must wait 100ms
    #    after sending this command before sending additional commands at the new
    #    baud rate. The SCI must be in passive, safe, or full mode to accept this
    #    command. This command puts the SCI in passive mode.
    #
    #    """
    #    if baud_rate not in BAUD_RATES:
    #        raise RoombaError('Invalid baud rate specified.')
    #    self.sci.baud(baud_rate)
    #    self.sci = SerialCommandInterface(self.tty, baud_rate)

    def Passive(self):
        """Put the robot in passive mode."""
        self.sci.start()
        time.sleep(0.5)

    def Control(self):
        """Start the robot's SCI interface and place it in safe mode."""
        self.Passive()
        self.sci.control()  # Also puts the Roomba in to safe mode.
        if not self.safe:
            self.sci.full()
        time.sleep(0.5)

    # TODO: Should check if currently in the correct mode
    def Drive(self, velocity, radius):
        """Controls Roomba's drive wheels.

        NOTE(damonkohler): The following specification applies to both the Roomba
        and the Create.

        The Roomba takes four data bytes, interpreted as two 16-bit signed values
        using two's complement. The first two bytes specify the average velocity
        of the drive wheels in millimeters per second (mm/s), with the high byte
        being sent first. The next two bytes specify the radius in millimeters at
        which Roomba will turn. The longer radii make Roomba drive straighter,
        while the shorter radii make Roomba turn more. The radius is measured from
        the center of the turning circle to the center of Roomba.

        A Drive command with a positive velocity and a positive radius makes
        Roomba drive forward while turning toward the left. A negative radius
        makes Roomba turn toward the right. Special cases for the radius make
        Roomba turn in place or drive straight, as specified below. A negative
        velocity makes Roomba drive backward.

        Also see DriveStraight and TurnInPlace convenience methods.

        """
        # Mask integers to 2 bytes.
        velocity = int(velocity) & 0xffff
        radius = int(radius) & 0xffff
        # Pack as shorts to get 2 x 2 byte integers. Unpack as 4 bytes to send.
        # TODO(damonkohler): The 4 unpacked bytes will just be repacked later,
        # that seems dumb to me.
        bytes = struct.unpack('4B', struct.pack('>2H', velocity, radius))
        self.sci.drive(*bytes)

    def Stop(self):
        """Set velocity and radius to 0 to stop movement."""
        self.Drive(0, 0)

    def SlowStop(self, velocity):
        """Slowly reduce the velocity to 0 to stop movement."""
        velocities = range(velocity, VELOCITY_SLOW, -25)
        if velocity < 0:
            velocities = range(velocity, -VELOCITY_SLOW, 25)
        for velocity in velocities:
            self.Drive(velocity, RADIUS_STRAIGHT)
            time.sleep(0.05)
        self.Stop()

    def DriveStraight(self, velocity):
        """Drive in a straight line."""
        self.Drive(velocity, RADIUS_STRAIGHT)

    def TurnInPlace(self, velocity, direction):
        """Turn in place either clockwise or counter-clockwise."""
        valid_directions = {'cw': RADIUS_TURN_IN_PLACE_CW,
                            'ccw': RADIUS_TURN_IN_PLACE_CCW}
        self.Drive(velocity, valid_directions[direction])

    def Dock(self):
        """Start looking for the dock and then dock."""
        # NOTE(damonkohler): We should be able to call dock from any mode, however
        # it only seems to work from passive.
        self.sci.start()
        time.sleep(0.5)
        self.sci.force_seeking_dock()
