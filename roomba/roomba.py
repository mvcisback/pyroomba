"""
License
"""
import logging
import time
import struct

from math import floor

from .sensors.roomba_sensors import RoombaSensors
from .sensors.create_sensors import CreateSensors

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
VELOCITY_SLOW = floor(VELOCITY_MAX * 0.33)
VELOCITY_FAST = floor(VELOCITY_MAX * 0.66)

WHEEL_SEPARATION = 298  # mm

class RoombaError(Exception):
    """Raise if roomba wrapper has an issue"""

class Roomba(object):

    """Represents a Roomba robot."""

    def __init__(self, controller):
        self.sci = controller
        self.sci.add_opcodes(ROOMBA_OPCODES)
        self.sensors = RoombaSensors(self)
        self.safe = True

    def passive(self):
        """Put the robot in passive mode."""
        self.sci.start()
        time.sleep(0.5)

    def control(self):
        """Start the robot's SCI interface and place it in safe mode."""
        self.passive()
        self.sci.control()  # Also puts the Roomba in to safe mode.
        if not self.safe:
            self.sci.full()
        time.sleep(0.5)

    # TODO: Should check if currently in the correct mode
    def drive(self, velocity, radius):
        """Controls Roomba's drive wheels.

        NOTE(damonkohler): The following specification applies to both the
        Roomba and the Create.

        The Roomba takes four data bytes, interpreted as two 16-bit signed
        values using two's complement. The first two bytes specify the average
        velocity of the drive wheels in millimeters per second (mm/s), with the
        high byte being sent first. The next two bytes specify the radius in
        millimeters at which Roomba will turn. The longer radii make Roomba
        drive straighter, while the shorter radii make Roomba turn more. The
        radius is measured from the center of the turning circle to the center
        of Roomba.

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
        byte_str = struct.unpack('4B', struct.pack('>2H', velocity, radius))
        self.sci.drive(*byte_str)

    def stop(self):
        """Set velocity and radius to 0 to stop movement."""
        self.drive(0, 0)

    def slow_stop(self, velocity):
        """Slowly reduce the velocity to 0 to stop movement."""
        velocities = range(velocity, VELOCITY_SLOW, -25)
        if velocity < 0:
            velocities = range(velocity, -VELOCITY_SLOW, 25)
        for velocity in velocities:
            self.drive(velocity, RADIUS_STRAIGHT)
            time.sleep(0.05)
        self.stop()

    def drive_straight(self, velocity):
        """Drive in a straight line."""
        self.drive(velocity, RADIUS_STRAIGHT)

    def turn_in_place(self, velocity, direction):
        """Turn in place either clockwise or counter-clockwise."""
        valid_directions = {'cw': RADIUS_TURN_IN_PLACE_CW,
                            'ccw': RADIUS_TURN_IN_PLACE_CCW}
        self.drive(velocity, valid_directions[direction])

    def dock(self):
        """Start looking for the dock and then dock."""
        # NOTE(damonkohler): We should be able to call dock from any mode,
        # however it only seems to work from passive.
        self.sci.start()
        time.sleep(0.5)
        self.sci.force_seeking_dock()

CREATE_OPCODES = {
    'soft_reset': 7,  # Where is this documented?
    'low_side_drivers': 138,
    'pwm_low_side_drivers': 144,
    'direct_drive': 145,
    'digital_outputs': 147,
    'stream': 148,
    'query_list': 149,
    'pause_resume_stream': 150,
    'send_ir': 151,
    'script': 152,
    'play_script': 153,
    'show_script': 154,
    'wait_time': 155,
    'wait_distance': 156,
    'wait_angle': 157,
    'wait_event': 158,
}

START_DELAY = 5  # Time it takes the Roomba/Create to boot.

class Create(Roomba):
    """Represents a Create robot."""

    def __init__(self, controller):
        Roomba.__init__(self, controller)
        self.sci.add_opcodes(CREATE_OPCODES)
        self.sensors = CreateSensors(self)

    def control(self):
        """Start the robot's SCI interface and place it in safe or full mode."""
        logging.info('Sending control opcodes.')
        self.passive()
        if self.safe:
            self.sci.safe()
        else:
            self.sci.full()
        time.sleep(0.5)

    def power_low_side_drivers(self, drivers):
        """Enable or disable power to low side drivers.

        'drivers' should be a list of booleans indicating which low side drivers
        should be powered.
        """
        if len(drivers) != 3:
            raise RoombaError('Expecting 3 low side driver power settings.')
        byte = sum((2**driver * int(pwr) for driver, pwr in enumerate(drivers)))
        self.sci.low_side_drivers(byte)

    def soft_reset(self):
        """Do a soft reset of the Create."""
        logging.info('Sending soft reset.')
        self.sci.soft_reset()
        time.sleep(START_DELAY)
        self.passive()

    # FIXME I think this is broken in the original
    def led_control(self, leds):
        """Turn or or off Advance or Play leds.

        'leds' should be a list of booleans indicating which led
        should be powered, in this sequence: Advance, Play, Power
        """
        fst_byte = (2 ** 3) * int(leds[0])
        fst_byte += 2 * int(leds[1])
        snd_byte = 0    #green
        trd_byte = 0xFF * int(leds[2])
        byte_str = (fst_byte, snd_byte, trd_byte)
        self.sci.leds(*byte_str)
