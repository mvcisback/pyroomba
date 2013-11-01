import logging
import time

from .roomba import Roomba, RoombaError
from ..sensors.create_sensors import CreateSensors

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
        self.sci.AddOpcodes(CREATE_OPCODES)
        self.sensors = CreateSensors(self)

    def Control(self):
        """Start the robot's SCI interface and place it in safe or full mode."""
        logging.info('Sending control opcodes.')
        self.Passive()
        if self.safe:
            self.sci.safe()
        else:
            self.sci.full()
        time.sleep(0.5)

    def PowerLowSideDrivers(self, drivers):
        """Enable or disable power to low side drivers.

        'drivers' should be a list of booleans indicating which low side drivers
        should be powered.
        """
        if len(drivers) != 3:
            raise RoombaError('Expecting 3 low side driver power settings.')
        byte = sum((2**driver * int(pwr) for driver, pwr in enumerate(drivers)))
        self.sci.low_side_drivers(byte)

    def SoftReset(self):
        """Do a soft reset of the Create."""
        logging.info('Sending soft reset.')
        self.sci.soft_reset()
        time.sleep(START_DELAY)
        self.Passive()

    # I think this is broken in the original
    def LedControl(self, leds):
        """Turn or or off Advance or Play leds.

        'leds' should be a list of booleans indicating which led
        should be powered, in this sequence: Advance, Play, Power

        """
        fst_byte = (2 ** 3) * int(leds[0])
        fst_byte += 2 * int(leds[1])
        snd_byte = 0    #green
        trd_byte = 0xFF * int(leds[2])
        bytes = (fst_byte,snd_byte,trd_byte)
        self.sci.leds(*bytes)
