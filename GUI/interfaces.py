from abc import ABC, abstractmethod
import socket
import AR_OS_pb2 as pb
from systems import ESPState, ADCS_mode, TTC_mode

HOST = "127.0.0.1"

class interface(ABC):
    """
    Generic interface code for connecting the simulator to AR-OS.

    Made modular to work with other interface types without having to rewrite any other system or display code,
    but was originally coded to work with LAN using local ports for communication.
    """

    def __init__(self, address, controller, system):
        """
        Saves references to other objects in simulator and the address to respond to when listening (In LAN used as port)
        """
        self.system = system
        self.controller = controller
        self.address = address
        self.connected = False

    @abstractmethod
    def connect(self):
        """
        Either connects to the desired system or opens a local interface for system to connect to it.
        returns when connection has been established or controller.close = True
        """
        pass

    @abstractmethod
    def handle_communication(self):
        """
        Handles one interaction (receive and send) between system and AR-OS, is system specific
        """
        pass

    def runInterface(self, _):
        """
        Loop for the interface thread, connects to a system then continuously handles communication
        until controller.close = True.
        """
        print(f"Thread for {self.system.name} running")

        self.connect()

        if not self.connected:
            return

        while not self.controller.close:
            self.handle_communication()

        return

    @abstractmethod
    def sendTo(self, msg: bytes):
        """
        Abstract method for sending byte string to AR-OS from the simulator
        """
        pass

    @abstractmethod
    def recvFrom(self):
        """
        Abstract method for receiving byte string from AR-OS to the simulator
        """
        msg = b''
        return msg


class interfaceLAN(interface, ABC):
    """
    Generic system interface code for LAN. Listens on a TCP socket on a given port to simulate listening on a serial
    bus, each system will have it own inherited version of this to handle their specific message requirements.
    """

    def __init__(self, address, controller, system):
        super().__init__(address, controller, system)
        """
        Creates empty variables to store the socket and connection when generated
        """
        self.socket = None
        self.conn = None

    def __del__(self):
        """
        Closes the socket and connection when the object is deleted, to avoid memory leak.
        """
        if self.socket:
            self.socket.close()
        if self.conn:
            self.conn.close()

    def connect(self):
        """
        Creates new TCP socket at port 'address' and waits for connection, will return either when successfully connected
        or when self.controller.close = True. Will set self.connected if connection established
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Address is used as port number, since it is the 'address' for the system in the simulator
        self.socket.bind((HOST, self.address))

        self.socket.settimeout(1)

        while not self.conn:
            # Well not connected, continue listening and try to connect
            try:
                self.socket.listen(1)
                # If connection is successful save connection in variable then break loop
                self.conn, _ = self.socket.accept()
                break
            except TimeoutError:
                # If timeout then check controller.close status
                if self.controller.close:
                    # If true return so thread can close
                    print(f"Thread {self.system.name} closing before connection")
                    return
                else:
                    # Else do nothing and listen again
                    pass
        # If successful at making connection, update boolean and print message
        self.connected = True
        print(f"Thread {self.system.name} connection established")

    def sendTo(self, msg: bytes):
        """
        Sends generated message through the desired interface (LAN)
        packet is length of message in 4 bytes + message given by system
        """
        length = len(msg)
        lengthBytes = length.to_bytes(4, 'little')
        packet = lengthBytes + msg
        self.conn.send(packet)
        return

    def recvFrom(self):
        """
        Receives packet from AR-OS and returns the message to the simulator from desired interface (LAN)
        """
        # sets timeout to check if simulation still running
        self.conn.settimeout(1)

        # Recevives first 4 bytes of message, which is the length of the message to come
        lengthBytes = b''
        while len(lengthBytes) < 4:
            try:
                lengthBytes += self.conn.recv(4-len(lengthBytes))
            except TimeoutError:
                #print("TIMEOUT")
                # If timeout then check controller.close status
                if self.controller.close:
                    # If true exit thread
                    print(f"Thread {self.system.name} closing after connection")
                    exit()
                else:
                    # Else do nothing and listen again
                    pass
        length = int.from_bytes(lengthBytes, 'little')

        msg = b''
        while len(msg) < length:
            try:
                msg += self.conn.recv(length-len(msg))
            except TimeoutError:
                # If timeout then check controller.close status
                if self.controller.close:
                    # If true exit thread
                    print(f"Thread {self.system.name} closing after connection")
                    exit()
                else:
                    # Else do nothing and listen again
                    pass
        return msg

    @abstractmethod
    def handle_communication(self):
        """
        Handles one interaction (receive and send) between system and AR-OS, is system specific
        """
        pass


class interfaceLAN_EPS(interfaceLAN):
    """
    interface of EPS
    """

    def handle_communication(self):
        msg = self.recvFrom()
        aros_com = pb.AROS_Command()
        aros_com.ParseFromString(msg)

        sim_resp = pb.Simulator_Response()

        if aros_com.command == pb.COMMAND.GEN_PING:
            sim_resp.response = pb.RESPONSE.GEN_PONG
        elif aros_com.command == pb.COMMAND.GEN_GET_VOLTAGE:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_VOLTAGE
            sim_resp.single = self.system.voltage
        elif aros_com.command == pb.COMMAND.GEN_GET_TEMP:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_TEMP
            sim_resp.single = self.system.temp

        elif aros_com.command == pb.COMMAND.EPS_GET_CHARGE:
            sim_resp.response = pb.RESPONSE.EPS_RETURN_CHARGE
            sim_resp.single = self.system.charge
        elif aros_com.command == pb.COMMAND.EPS_GET_PS:
            if self.system.power_saving:
                sim_resp.response = pb.RESPONSE.EPS_PS_ON
            else:
                sim_resp.response = pb.RESPONSE.EPS_PS_OFF
        elif aros_com.command == pb.COMMAND.EPS_SET_PS_ON:
            if self.system.set_ps_on():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.EPS_SET_PS_OFF:
            if self.system.set_ps_off():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        else:
            sim_resp.response = pb.RESPONSE.GEN_ERROR

        msg = sim_resp.SerializeToString()
        self.sendTo(msg)

class interfaceLAN_ESP(interfaceLAN):
    """
    interface of ESP
    """

    def handle_communication(self):
        msg = self.recvFrom()
        aros_com = pb.AROS_Command()
        aros_com.ParseFromString(msg)

        sim_resp = pb.Simulator_Response()

        if aros_com.command == pb.COMMAND.GEN_PING:
            sim_resp.response = pb.RESPONSE.GEN_PONG
        elif aros_com.command == pb.COMMAND.GEN_GET_VOLTAGE:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_VOLTAGE
            sim_resp.single = self.system.voltage
        elif aros_com.command == pb.COMMAND.GEN_GET_TEMP:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_TEMP
            sim_resp.single = self.system.temp

        elif aros_com.command == pb.COMMAND.ESP_GET_FUEL:
            sim_resp.response = pb.RESPONSE.ESP_RETURN_FUEL
            sim_resp.single = self.system.fuel
        elif aros_com.command == pb.COMMAND.ESP_GET_MODE:
            if self.system.status == ESPState.OFF:
                sim_resp.response = pb.RESPONSE.ESP_OFF
            elif self.system.status == ESPState.WARMING:
                sim_resp.response = pb.RESPONSE.ESP_WARMING
            elif self.system.status == ESPState.READY:
                sim_resp.response = pb.RESPONSE.ESP_READY
            elif self.system.status == ESPState.BURNING:
                sim_resp.response = pb.RESPONSE.ESP_BURNING
            elif self.system.status == ESPState.COOLDOWN:
                sim_resp.response = pb.RESPONSE.ESP_COOL_DOWN
        elif aros_com.command == pb.COMMAND.ESP_SET_WARMUP:
            if self.system.set_warmup():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.ESP_SET_BURNING:
            if self.system.set_burn():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.ESP_SET_OFF:
            if self.system.set_off():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        else:
            sim_resp.response = pb.RESPONSE.GEN_ERROR

        msg = sim_resp.SerializeToString()
        self.sendTo(msg)


class interfaceLAN_dragSail(interfaceLAN):
    """
    interface of dragSail
    """

    def handle_communication(self):
        msg = self.recvFrom()
        aros_com = pb.AROS_Command()
        aros_com.ParseFromString(msg)

        sim_resp = pb.Simulator_Response()

        if aros_com.command == pb.COMMAND.GEN_PING:
            sim_resp.response = pb.RESPONSE.GEN_PONG
        elif aros_com.command == pb.COMMAND.GEN_GET_VOLTAGE:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_VOLTAGE
            sim_resp.single = self.system.voltage
        elif aros_com.command == pb.COMMAND.GEN_GET_TEMP:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_TEMP
            sim_resp.single = self.system.temp

        elif aros_com.command == pb.COMMAND.DRAG_GET_MODE:
            if not self.system.deployed:
                sim_resp.response = pb.RESPONSE.DRAG_RETRACTED
            else:
                sim_resp.response = pb.RESPONSE.DRAG_DEPLOYED
        elif aros_com.command == pb.COMMAND.DRAG_SET_DEPLOY:
            if self.system.deploy_drag():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        else:
            sim_resp.response = pb.RESPONSE.GEN_ERROR

        msg = sim_resp.SerializeToString()
        self.sendTo(msg)


class interfaceLAN_ADCS(interfaceLAN):
    """
    interface of ADCS
    """

    def handle_communication(self):
        msg = self.recvFrom()
        aros_com = pb.AROS_Command()
        aros_com.ParseFromString(msg)

        sim_resp = pb.Simulator_Response()

        if aros_com.command == pb.COMMAND.GEN_PING:
            sim_resp.response = pb.RESPONSE.GEN_PONG
        elif aros_com.command == pb.COMMAND.GEN_GET_VOLTAGE:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_VOLTAGE
            sim_resp.single = self.system.voltage
        elif aros_com.command == pb.COMMAND.GEN_GET_TEMP:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_TEMP
            sim_resp.single = self.system.temp

        elif aros_com.command == pb.COMMAND.ADCS_GET_PRY:
            sim_resp.response = pb.RESPONSE.ADCS_RETURN_PRY
            sim_resp.vector.x = self.system.pitch
            sim_resp.vector.y = self.system.roll
            sim_resp.vector.z = self.system.yaw
        elif aros_com.command == pb.COMMAND.ADCS_GET_AV:
            sim_resp.response = pb.RESPONSE.ADCS_RETURN_AV
            sim_resp.vector.x = self.system.pitch_av
            sim_resp.vector.y = self.system.roll_av
            sim_resp.vector.z = self.system.yaw_av
        elif aros_com.command == pb.COMMAND.ADCS_GET_MODE:
            if self.system.mode == ADCS_mode.OFF:
                sim_resp.response = pb.RESPONSE.ADCS_OFF
            elif self.system.mode == ADCS_mode.DETUMBLING:
                sim_resp.response = pb.RESPONSE.ADCS_DE_TUMBLE
            elif self.system.mode == ADCS_mode.SUN_POINTING:
                sim_resp.response = pb.RESPONSE.ADCS_SUN_POINT
        elif aros_com.command == pb.COMMAND.ADCS_SET_OFF:
            if self.system.set_off():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.ADCS_SET_DE_TUMBLE:
            if self.system.set_de_tumbling():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.ADCS_SET_SUN_POINT:
            if self.system.set_sun_pointing():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        else:
            sim_resp.response = pb.RESPONSE.GEN_ERROR

        msg = sim_resp.SerializeToString()
        self.sendTo(msg)


class interfaceLAN_GNSS(interfaceLAN):
    """
    interface of GNSS
    """

    def handle_communication(self):
        msg = self.recvFrom()
        aros_com = pb.AROS_Command()
        aros_com.ParseFromString(msg)

        sim_resp = pb.Simulator_Response()

        if aros_com.command == pb.COMMAND.GEN_PING:
            sim_resp.response = pb.RESPONSE.GEN_PONG
        elif aros_com.command == pb.COMMAND.GEN_GET_VOLTAGE:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_VOLTAGE
            sim_resp.single = self.system.voltage
        elif aros_com.command == pb.COMMAND.GEN_GET_TEMP:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_TEMP
            sim_resp.single = self.system.temp

        elif aros_com.command == pb.COMMAND.GNSS_GET_POSI:
            sim_resp.response = pb.RESPONSE.GNSS_RETURN_POSI
            sim_resp.vector.x = self.system.latitude
            sim_resp.vector.y = self.system.longitude
            sim_resp.vector.z = self.system.elevation
        else:
            sim_resp.response = pb.RESPONSE.GEN_ERROR

        msg = sim_resp.SerializeToString()
        self.sendTo(msg)


class interfaceLAN_Pi_VHF(interfaceLAN):
    """
    interface of Pi_VHF
    """

    def handle_communication(self):
        msg = self.recvFrom()
        aros_com = pb.AROS_Command()
        aros_com.ParseFromString(msg)

        sim_resp = pb.Simulator_Response()

        if aros_com.command == pb.COMMAND.GEN_PING:
            sim_resp.response = pb.RESPONSE.GEN_PONG
        elif aros_com.command == pb.COMMAND.GEN_GET_VOLTAGE:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_VOLTAGE
            sim_resp.single = self.system.voltage
        elif aros_com.command == pb.COMMAND.GEN_GET_TEMP:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_TEMP
            sim_resp.single = self.system.temp

        elif aros_com.command == pb.COMMAND.PI_GET_MODE:
            if self.system.enabled:
                sim_resp.response = pb.RESPONSE.PI_ON
            else:
                sim_resp.response = pb.RESPONSE.PI_OFF
        elif aros_com.command == pb.COMMAND.PI_GET_AUDIO:
            if not self.system.enabled:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
            else:
                sim_resp.response = pb.RESPONSE.PI_RETURN_AUDIO
                sim_resp.byte_string = self.system.get_audio()
        elif aros_com.command == pb.COMMAND.PI_SET_ON:
            if self.system.set_on():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.PI_SET_OFF:
            if self.system.set_off():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        else:
            sim_resp.response = pb.RESPONSE.GEN_ERROR

        msg = sim_resp.SerializeToString()
        self.sendTo(msg)


class interfaceLAN_OBC(interfaceLAN):
    """
    interface of OBC
    """

    def handle_communication(self):
        msg = self.recvFrom()
        aros_com = pb.AROS_Command()
        aros_com.ParseFromString(msg)

        sim_resp = pb.Simulator_Response()

        if aros_com.command == pb.COMMAND.GEN_PING:
            sim_resp.response = pb.RESPONSE.GEN_PONG
        elif aros_com.command == pb.COMMAND.GEN_GET_VOLTAGE:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_VOLTAGE
            sim_resp.single = self.system.voltage
        elif aros_com.command == pb.COMMAND.GEN_GET_TEMP:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_TEMP
            sim_resp.single = self.system.temp
        else:
            sim_resp.response = pb.RESPONSE.GEN_ERROR

        msg = sim_resp.SerializeToString()
        self.sendTo(msg)


class interfaceLAN_TTC(interfaceLAN):
    """
    interface of TTC
    """

    def handle_communication(self):
        msg = self.recvFrom()
        aros_com = pb.AROS_Command()
        aros_com.ParseFromString(msg)

        sim_resp = pb.Simulator_Response()

        if aros_com.command == pb.COMMAND.GEN_PING:
            sim_resp.response = pb.RESPONSE.GEN_PONG
        elif aros_com.command == pb.COMMAND.GEN_GET_VOLTAGE:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_VOLTAGE
            sim_resp.single = self.system.voltage
        elif aros_com.command == pb.COMMAND.GEN_GET_TEMP:
            sim_resp.response = pb.RESPONSE.GEN_RETURN_TEMP
            sim_resp.single = self.system.temp
        elif aros_com.command == pb.COMMAND.TTC_GET_MODE:
            if self.system.mode == TTC_mode.OFF:
                sim_resp.response = pb.RESPONSE.TTC_OFF
            elif self.system.mode == TTC_mode.BEACONING:
                sim_resp.response = pb.RESPONSE.TTC_BEACONING
            elif self.system.mode == TTC_mode.CONNECTING:
                sim_resp.response = pb.RESPONSE.TTC_CONNECTING
            elif self.system.mode == TTC_mode.ESTABLISHED_DATA:
                sim_resp.response = pb.RESPONSE.TTC_ESTABLISHED_DATA
            elif self.system.mode == TTC_mode.ESTABLISHED_CONT:
                sim_resp.response = pb.RESPONSE.TTC_ESTABLISHED_CONT
            elif self.system.mode == TTC_mode.BROADCAST_NO_CON:
                sim_resp.response = pb.RESPONSE.TTC_BROADCAST_NO_CON
            elif self.system.mode == TTC_mode.DISCONNECTED:
                sim_resp.response = pb.RESPONSE.TTC_DISCONNECTED
        elif aros_com.command == pb.COMMAND.TTC_GET_COMMAND:
            if self.system.mode == TTC_mode.ESTABLISHED_DATA or self.system.mode == TTC_mode.ESTABLISHED_CONT:
                msg = self.system.get_msg()
                if msg == b'':
                    sim_resp.response = pb.RESPONSE.GEN_ERROR
                else:
                    sim_resp.response = pb.RESPONSE.TTC_RETURN_COMMAND
                    sim_resp.byte_string = msg
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.TTC_SET_OFF:
            if self.system.set_off():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.TTC_SET_BEACONING:
            if self.system.set_beaconing():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.TTC_SET_CONNECTING:
            if self.system.set_connecting():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.TTC_SET_BROADCAST_NO_CON:
            if self.system.set_broadcast_no_con():
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.TTC_SEND_BYTE_STRING:
            if aros_com.HasField('byte_string'):
                msg = aros_com.byte_string
            else:
                msg = b''
            if self.system.recv_msg(msg=msg):
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.TTC_SEND_HEALTH:
            if aros_com.HasField('byte_string'):
                msg = aros_com.byte_string
            else:
                msg = b''
            if self.system.recv_health(msg=msg):
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        elif aros_com.command == pb.COMMAND.TTC_SEND_AUDIO:
            if aros_com.HasField('byte_string'):
                msg = aros_com.byte_string
            else:
                msg = b''
            if self.system.recv_audio(msg=msg):
                sim_resp.response = pb.RESPONSE.GEN_SUCCESS
            else:
                sim_resp.response = pb.RESPONSE.GEN_ERROR
        else:
            sim_resp.response = pb.RESPONSE.GEN_ERROR

        msg = sim_resp.SerializeToString()
        self.sendTo(msg)

