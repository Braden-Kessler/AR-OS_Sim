import math
from abc import ABC
from enum import Enum
from queue import Queue
from threading import Lock
import time  # Only used for getting local time to save files


DEFAULT_VOLTAGE = 12.0
DEFAULT_TEMP = 30.0

HOST = "127.0.0.1"


class system(ABC):
    """
    Abstract class for each system, contains the basic variables for every system such as their allowed port number, the
    basic state of health data, and the reference to the main controller to check when the simulation closes.

    Also contains the generic network code for each system to listen on a port and accept TCP requests.
    """
    def __init__(self, name, controller, port=0):
        self.name = name
        self.controller = controller
        self.port = port
        self.socket = None

        self.voltage = DEFAULT_VOLTAGE
        self.temp = DEFAULT_TEMP

        self.interface = None

    def add_interface(self, interfaceABC):
        self.interface = interfaceABC(self.port, self.controller, self)

    def run(self, _):
        self.interface.runInterface(_)


# Electrical Power System
class EPS(system):
    """
    System for simulating the Electrical Power System in Audimus
    """
    def __init__(self, name, controller, port=0):
        system.__init__(self, name, controller, port)
        self.charge = 50
        self.power_saving = False
        self.status = EPSState.SIMULATED

    def set_ps_on(self):
        if not self.power_saving:
            self.power_saving = True
            return True
        return False

    def set_ps_off(self):
        if self.power_saving:
            self.power_saving = False
            return True
        return False

class EPSState(Enum):
    """
    Enum to record the state of the EPS
    """
    MANUAL = 0
    SIMULATED = 1
    DECREASING = 2
    CHARGING = 3


# Electro-Spray Propulsion
class ESP(system):
    """
    System for simulating the Electro-Spray Propulsion in Audimus
    """
    def __init__(self, name, controller, port=0):
        system.__init__(self, name, controller, port)
        self.fuel = 100
        self.engine_temp = 0
        self.status = ESPState.OFF

    def set_warmup(self):
        if self.status == ESPState.OFF:
            self.status = ESPState.WARMING
            return True
        return False

    def set_burn(self):
        if self.status == ESPState.READY:
            self.status = ESPState.BURNING
            return True
        return False

    def set_off(self):
        if self.status == ESPState.BURNING:
            self.status = ESPState.COOLDOWN
            return True
        return False


class ESPState(Enum):
    """
    Enum to record the state of the ESP
    """
    OFF = 0
    WARMING = 1
    READY = 2
    BURNING = 3
    COOLDOWN = 4


# Drag Sail
class dragSail(system):
    """
    System for simulating the Drag Sail in Audimus
    """
    def __init__(self, name, controller, port=0):
        system.__init__(self, name, controller, port)
        self.deployed = False

    def deploy_drag(self):
        if not self.deployed:
            self.deployed = True
            return True
        return False


# Attitude Direction Control System
class ADCS(system):
    """
    System for simulating the Attitude Direction Control System in Audimus
    """
    def __init__(self, name, controller, port=0):
        system.__init__(self, name, controller, port)
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.status = GNSS_ADCSState.SIMULATED
        self.mode = ADCS_mode.OFF

    def simulate(self, angel):
        if self.status == GNSS_ADCSState.MANUAL:
            return
        self.pitch = angel[0]
        self.roll = angel[1]
        self.yaw = angel[2]

    def set_off(self):
        if self.mode != ADCS_mode.OFF:
            self.mode = ADCS_mode.OFF
            return True
        return False

    def set_de_tumbling(self):
        if self.mode != ADCS_mode.DETUMBLING:
            self.mode = ADCS_mode.DETUMBLING
            return True
        return False

    def set_sun_pointing(self):
        if self.mode != ADCS_mode.SUN_POINTING:
            self.mode = ADCS_mode.SUN_POINTING
            return True
        return False

class ADCS_mode(Enum):
    """
    Enum to record the operational mode of the ADCS
    """
    OFF = 0
    DETUMBLING = 1
    SUN_POINTING = 2


GNSS_TRAIL_SIZE = 3000


# Global Navigation Satellite System
class GNSS(system):
    """
    System for simulating the Global Navigation Satellite System in Audimus, also tracks the previous locations of
    audimus for the GUI's rendering

    """
    def __init__(self, name, controller, port=0):
        system.__init__(self, name, controller, port)
        self.latitude = 0.00
        self.longitude = 0.00
        self. elevation = 2000
        self.status = GNSS_ADCSState.SIMULATED

        # For image tracking purposes
        self.trail = Queue(maxsize=GNSS_TRAIL_SIZE)
        self.lastSaved = [0, 0]
        self.trailSize = -1

        # Mutex to avoid read write errors for trail between GNSS and GNSS Display
        self.lock = Lock()

    def simulate(self, lat, long, alt):
        """
        takes in simulated latitude longitude and altitude to update GNSS if in simulated mode.
        Updates the trail if the new point is far enough away from the last point.
        """
        if self.status == GNSS_ADCSState.SIMULATED:
            # Only save points and trail if GNSS in simulated mode
            if math.hypot(long - self.lastSaved[0], lat - self.lastSaved[1]) > 1:
                # Only save point to trail if distance between last point is greater then 1
                self.lock.acquire()
                # Acquire lock to avoid write-write race condition

                if self.trail.full():
                    # If full pop one off list to make space
                    self.trail.get()
                else:
                    # Else increment length
                    self.trailSize += 1

                point = (self.longitude, self.latitude)

                if self.trailSize > 0:
                    # Does not save the first point, so it does not save the jump when initializing
                    self.trail.put(point)
                # save current point as last saved point
                self.lastSaved = point

                self.lock.release()

            # Save calculated lat and long for system
            self.latitude = lat
            self.longitude = long
            self.elevation = alt



    def clear(self):
        """
        Empties all items in trail queue and sets trail size to -1.
        """
        self.lock.acquire()

        while not self.trail.empty():
            self.trail.get()

        self.lock.release()

        self.trailSize = -1


class GNSS_ADCSState(Enum):
    """
    Enum to record the state of the GNSS or the ADCS
    """
    MANUAL = 0
    SIMULATED = 1


# Raspberry Pi and VHF Radio
class Pi_VHF(system):
    """
    System for simulating the Raspberry Pi and VHF Radio System in Audimus
    """

    def __init__(self, name, controller, port=0):
        system.__init__(self, name, controller, port)
        # Pi status
        self.enabled = False
        self.connected = False
        # Audio Status
        self.audio_filepath = ""
        self.audio_status = audioState.NO_DATA
        self.byte_to_send = b''
        self.PI_MSG_LENGTH = 100
        # Sonar Bouy
        self.connection_radius = 500.0
        self.latitude = 73.0
        self.longitude = -96.0

    def load_file(self):
        try:
            f = open(self.audio_filepath, 'rb')
            self.byte_to_send = f.read()
            f.close()
            self.audio_status = audioState.UNSENT
            print(len(self.byte_to_send))
        except:
            self.audio_status = audioState.ERROR_LOADING

    def get_audio(self):
        if self.connected is False:
            # If Pi not connected, then return empty string
            return b''

        self.audio_status = audioState.SENDING

        # gets section of message
        msg = self.byte_to_send[0:self.PI_MSG_LENGTH]

        if len(self.byte_to_send) > self.PI_MSG_LENGTH:
            self.byte_to_send = self.byte_to_send[self.PI_MSG_LENGTH:]
        else:
            self.byte_to_send = b''
            self.audio_status = audioState.SENT

        return msg

    def set_on(self):
        if not self.enabled:
            self.enabled = True
            return True
        return False

    def set_off(self):
        if self.enabled:
            self.enabled = False
            return True
        return False


class audioState(Enum):
    NO_DATA = 0
    UNSENT = 1
    SENDING = 2
    SENT = 3
    ERROR_LOADING = 4



# On Board Computer
class OBC(system):
    """
    System for simulating the TTC in Audimus
    """
    def __init__(self, name, controller, port=0):
        system.__init__(self, name, controller, port)


class TTC(system):

    def __init__(self, name, controller, port=0):
        system.__init__(self, name, controller, port)
        # TTC and GS status variable
        self.mode = TTC_mode.OFF
        self.gs_status = TTC_GS_status.NO_RESPONSE
        self.connection_radius = 500
        self.connected = False
        # Message queues for sending and receiving and printing
        self.console_output = ''
        self.gs_to_aros = ''
        self.audio_to_save = b''

    def gs_send_command(self, msg):
        if msg == '':
            # If message is empty, do nothing with it
            return
        # Add message sent to console output so that it can be seen
        if self.console_output != '':
            self.console_output += '\n'
        self.console_output += f'GS[{"C" if self.connected else "X"}] > {msg}'

        # Adds message to queue for ar-os if in right state for sending commands
        if self.mode == TTC_mode.ESTABLISHED_CONT:
            # State for sending commands to AR-OS from GS
            self.gs_to_aros += msg + '\n'


    def get_msg(self):
        # If in right state for connection for receiving commands, read from gs_to_aros
        if self.mode == TTC_mode.ESTABLISHED_CONT:
            # Save and clear message queue
            msg = self.gs_to_aros
            self.gs_to_aros = ""
            # encode msg in bytes to be sent to AR-OS
            msg = msg.encode(encoding='utf-8')
            return msg
        # If not in right mode, return empty byte string
        return b''

    def recv_msg(self, msg=b'TEST'):
        if self.mode == TTC_mode.OFF or self.mode == TTC_mode.BEACONING or self.mode == TTC_mode.CONNECTING or self.mode == TTC_mode.DISCONNECTED:
            # If not in right mode to send byte then TTC will return an error
            return False
        elif self.connected == False:
            # If TTC not connected but in right mode, return true but do nothing with it, data has just been lost
            return True

        # Add message sent to console output so that it can be seen
        if self.console_output != '':
            self.console_output += '\n'
        self.console_output += f'AR-OS > {msg.decode(encoding="utf-8")}'

        return True

    def recv_health(self, msg=b'TEST'):
        if self.mode == TTC_mode.OFF or self.mode == TTC_mode.BEACONING or self.mode == TTC_mode.CONNECTING or self.mode == TTC_mode.DISCONNECTED:
            # If not in right mode to send byte then TTC will return an error
            return False
        elif self.connected == False:
            # If TTC not connected but in right mode, return true but do nothing with it, data has just been lost
            return True

        health_data = msg.decode(encoding="utf-8")

        # Save sent health data to log file, assumes health data is single message
        f = open("TTC_output/health_log.txt", 'at')
        f.write(health_data + '\n')
        f.close()

        if self.console_output != '':
            self.console_output += '\n'
        self.console_output += f'AR-OS > Logged Health data: {health_data[:80]}{"..." if len(health_data) > 70 else ""}'

        return True

    def recv_audio(self, msg=b'TEST'):
        if self.mode == TTC_mode.OFF or self.mode == TTC_mode.BEACONING or self.mode == TTC_mode.CONNECTING or self.mode == TTC_mode.DISCONNECTED:
            # If not in right mode to send byte then TTC will return an error
            return False
        elif self.connected == False:
            # If TTC not connected but in right mode, return true but do nothing with it, data has just been lost
            return True

        # Append audio data recevived to total audio data, done assuming most audio files will be too big for one message
        self.audio_to_save += msg

        if self.audio_to_save != b'' and msg == b'':
            # If audio data to save not empty but an empty message is sent, then end of message and should save it
            # generate unique name from hash
            audio_name = f'Audio_data_{time.strftime("%d-%m-%y_%H-%M-%S")}.wav'

            # Save data in output folder with name
            f = open(f'TTC_output/{audio_name}', 'wb')
            f.write(self.audio_to_save)
            f.close()

            # Clear Audio to save after saving it
            self.audio_to_save = b''

            # Print to console
            if self.console_output != '':
                self.console_output += '\n'
            self.console_output += f'AR-OS > Saved audio data in file {audio_name}'

        return True

    def set_off(self):
        if self.mode != TTC_mode.OFF:
            self.mode = TTC_mode.OFF
            return True
        return False

    def set_beaconing(self):
        if self.mode != TTC_mode.BEACONING:
            self.mode = TTC_mode.BEACONING
            return True
        return False

    def set_connecting(self):
        if self.mode == TTC_mode.OFF or self.mode == TTC_mode.DISCONNECTED:
            self.mode = TTC_mode.CONNECTING
            return True
        return False

    def set_broadcast_no_con(self):
        if self.mode == TTC_mode.CONNECTING:
            self.mode = TTC_mode.BROADCAST_NO_CON
            return True
        return False

class TTC_mode(Enum):
    """
    Enum to record the operational mode of the TTC
    """
    OFF = 0
    BEACONING = 1
    CONNECTING = 2
    ESTABLISHED_DATA = 3
    ESTABLISHED_CONT = 4
    BROADCAST_NO_CON = 5
    DISCONNECTED = 6

class TTC_GS_status(Enum):
    """
    Enum to record the operational mode of the Ground Station, mainly weather to respond to TTC or not
    """
    NO_RESPONSE = 0
    CONNECTION_DATA = 1
    CONNECTION_CONTROL = 2


