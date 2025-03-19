import socket
import AR_OS_pb2 as pb
import random

HOST = "127.0.0.1"

class interface_testerLAN:

    def __init__(self, port):
        """
        Creates a tester object to test the generic functionality of a interfaceLAN object
        """
        self.port = port
        self.connected = False
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __del__(self):
        """
        Destructor ensures socket is correctly closed when shut down
        """
        self.socket.close()

    def connect(self):
        """
        Attempts to connect to the given port at local host IP
        """
        print(f"{self.port}: Connecting to port ")
        try:
            self.socket.connect((HOST, self.port))
            self.connected = True
            print(f"{self.port}: Connected to port ")
        except:
            print(f"{self.port}: Failed to connect to port")

    def send(self, msg: bytes):
        """
        Sends a byte string message using the socket
        """
        # Adds message length as first four bytes of packet encoded in little endian
        length = len(msg)
        lengthBytes = length.to_bytes(4, 'little')
        packet = lengthBytes + msg
        self.socket.send(packet)
        return

    def recv(self):
        """
        receives a byte string message using the socket
        """
        # Recevives first 4 bytes of message, which is the length of the message to come
        lengthBytes = b''
        while len(lengthBytes) < 4:
            lengthBytes += self.socket.recv(4-len(lengthBytes))
        length = int.from_bytes(lengthBytes, 'little')

        msg = b''
        while len(msg) < length:
            msg += self.socket.recv(length-len(msg))
        return msg

    def ping(self):
        """
        Sends a ping and receives a response using the socket, checks response for correctness
        """
        print(f"{self.port}: Trying to ping port")
        if not self.connected:
            # Return if connection not established first
            print(f"{self.port}: Could not ping port, not connected to in first place")
            return

        # Creates both protobuf objects
        msg = pb.AROS_Command()
        rsp = pb.Simulator_Response()

        # Sets desired flag and creates the message string
        msg.command = pb.COMMAND.GEN_PING
        msgString = msg.SerializeToString()

        try:
            # Tries to send message, if exception print fail and return
            self.send(msgString)
        except:
            print(f"{self.port}: Could not ping port, failed to send")
            return

        try:
            # Tries to recv message, if exception print fail and return
            rspString = self.recv()
        except:
            print(f"{self.port}: Could not ping port, failed to receive")
            return

        # parse the message from the received string
        rsp.ParseFromString(rspString)
        #print(rsp)

        try:
            # Assert the correct response, else return error
            assert rsp.response == pb.RESPONSE.GEN_PONG
            print(f"{self.port}: Successfully pinged port")
        except:
            print(f"{self.port}: Could not ping port, received wrong response")

    def get_health(self):
        """
        Sends for and then receives both the health and voltage data of the component, asserts correct fields given
        """
        print(f"{self.port}: Trying to get health data")
        if not self.connected:
            # Return if connection not established first
            print(f"{self.port}: Could not get health data, not connected to in first place")
            return

        # Creates both protobuf objects
        msg = pb.AROS_Command()
        rsp = pb.Simulator_Response()

        # Creates message string to request voltage
        msg.command = pb.COMMAND.GEN_GET_VOLTAGE
        msgString1 = msg.SerializeToString()
        # Creates message string to request temperature
        msg.command = pb.COMMAND.GEN_GET_TEMP
        msgString2 = msg.SerializeToString()

        try:
            # Try both sends and receives, if error report error
            self.send(msgString1)
            rspString1 = self.recv()
            self.send(msgString2)
            rspString2 = self.recv()
        except:
            print(f"{self.port}: Could not get health data, failed to send or receive")
            return

        try:
            # Parse contents of each response, assert correct fields and save desired data
            rsp.ParseFromString(rspString1)
            assert rsp.response == pb.RESPONSE.GEN_RETURN_VOLTAGE and rsp.HasField('single')
            voltage = rsp.single
            rsp.ParseFromString(rspString2)
            assert rsp.response == pb.RESPONSE.GEN_RETURN_TEMP and rsp.HasField('single')
            temp = rsp.single
            # Print positive response as well as the received voltage and temp data
            print(f"{self.port}: Successfully got health data {voltage}V and {temp}°C")
        except:
            print(f"{self.port}: Could not get health data, received wrong responses")

    def test_eps(self):
        """
        Tests all functionality of the EPS module interface, works as proof of concept for all other modules
        """
        print(f"{self.port}: Testing full scope of EPS")
        if not self.connected:
            # Return if connection not established first
            print(f"{self.port}: Could not test full scope of EPS, not connected to in first place")
            return

        # Creates both protobuf objects
        msg = pb.AROS_Command()
        rsp = pb.Simulator_Response()

        try:
            msg.command = pb.COMMAND.DRAG_GET_MODE
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.GEN_ERROR
            print(f"{self.port}: Successfully got error from asking EPS for drag sail data")
        except:
            print(f"{self.port}: Failed to get error from asking EPS for drag sail data")

        try:
            msg.command = pb.COMMAND.EPS_GET_CHARGE
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.EPS_RETURN_CHARGE and rsp.HasField('single')
            charge = rsp.single
            print(f"{self.port}: Successfully checked EPS has charge and it's at {charge}%")
        except:
            print(f"{self.port}: Failed to get charge from EPS")

        try:
            msg.command = pb.COMMAND.EPS_GET_PS
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.EPS_PS_OFF
            print(f"{self.port}: Successfully checked EPS power saving status, and it was OFF")
        except:
            print(f"{self.port}: Failed to check the power saving status of EPS")

        try:
            msg.command = pb.COMMAND.EPS_SET_PS_ON
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.GEN_SUCCESS
            print(f"{self.port}: Successfully set EPS power saving to on")
        except:
            print(f"{self.port}: Failed to set the power saving status of EPS")

        try:
            msg.command = pb.COMMAND.EPS_GET_PS
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.EPS_PS_ON
            print(f"{self.port}: Successfully checked EPS power saving status, and it was ON")
        except:
            print(f"{self.port}: Failed to check the power saving status of EPS")

        try:
            msg.command = pb.COMMAND.EPS_SET_PS_OFF
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.GEN_SUCCESS
            print(f"{self.port}: Successfully set EPS power saving to off")
        except:
            print(f"{self.port}: Failed to set the power saving status of EPS")

    def test_pi_VHF_file(self):
        """
        Test the downloading of a file from the Pi/VHF

        Steps to set up in GUI.

        1) Select audio file in Pi_VHF
        2) Click "Set File"
        3) Type 999999 into "Connection Range"
        4) Click "Set Range"
        5) In Simulator panel, if not already running simulation,
            5.1) Initialize simulation
            5.2) Do at least one time step

        """
        print(f"{self.port}: Testing Download of file from Pi")
        if not self.connected:
            # Return if connection not established first
            print(f"{self.port}: Could not test download from Pi, not connected to in first place")
            return

        # Creates both protobuf objects
        msg = pb.AROS_Command()
        rsp = pb.Simulator_Response()

        try:
            msg.command = pb.COMMAND.PI_GET_MODE
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.PI_OFF
            print(f"{self.port}: Successfully got response that Pi is disabled")
        except:
            print(f"{self.port}: Failed to check that Pi is disabled")

        try:
            msg.command = pb.COMMAND.PI_SET_ON
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.GEN_SUCCESS
            print(f"{self.port}: Successfully set Pi to enabled for receiving")
        except:
            print(f"{self.port}: Failed to set Pi to enabled for receiving, aborting")
            return

        f = open('test_output/test.wav', 'wb')
        test_file = b''
        receiving = False
        done = False

        while not done:
            msg.command = pb.COMMAND.PI_GET_AUDIO
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()
            rsp.ParseFromString(rspString)

            if rsp.HasField('byte_string'):
                test_file += rsp.byte_string

                if rsp.byte_string != b'' and not receiving:
                    print(f"{self.port}: Receiving file from Pi")
                    receiving = True

                if rsp.byte_string == b'' and receiving:
                    print(f"{self.port}: Finished receiving File from Pi")
                    done = True

        f.write(test_file)
        f.close()
        print(f"{self.port}: Successfully saved file from Pi, finished testing download ")

    def test_adcs_vectors(self):
        """
        Test retriving vectors of data using the ADCS' PRY and AV commands
        """
        print(f"{self.port}: Testing retrieving vectors from simulator")
        if not self.connected:
            # Return if connection not established first
            print(f"{self.port}: Could not test retrieving vectors, not connected to in first place")
            return

        # Creates both protobuf objects
        msg = pb.AROS_Command()
        rsp = pb.Simulator_Response()

        # Tries to retrieve the Pitch Roll and Yaw
        try:
            msg.command = pb.COMMAND.ADCS_GET_PRY
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.ADCS_RETURN_PRY and rsp.HasField('vector')
            pitch = rsp.vector.x
            roll = rsp.vector.y
            yaw = rsp.vector.z
            print(f"{self.port}: Successfully got response that ADCS has Pitch {pitch}, Roll {roll}, and Yaw {yaw}.")
        except Exception as e:
            print(f"{self.port}: Failed to get Pitch Roll and Yaw from ADCS: {e}")

        # Tries to retrieve the Angular Velocities
        try:
            msg.command = pb.COMMAND.ADCS_GET_AV
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.ADCS_RETURN_AV and rsp.HasField('vector')
            pitch_av = rsp.vector.x
            roll_av = rsp.vector.y
            yaw_av = rsp.vector.z
            print(f"{self.port}: Successfully got response that ADCS has angualr velocities [{pitch_av}, {roll_av}, {yaw_av}].")
        except Exception as e:
            print(f"{self.port}: Failed to get angular velocities from ADCS: {e}")


    def test_ttc_gc_comms(self):
        """
        Test the different comms method of TTC and GS

        Steps to set up in GUI.
        1) In Simulator panel, if not already running simulation,
            1.1) Initialize simulation
            1.2) Enable real time
        2) Open TTC/GS
        3) Type 999999 into "Connection Range"
        4) Click "Set Range"
        5) When promoted for a command, enter anything into input box and click "send"

        """
        print(f"{self.port}: Testing Download of file from Pi")
        if not self.connected:
            # Return if connection not established first
            print(f"{self.port}: Could not test download from Pi, not connected to in first place")
            return

        # Creates both protobuf objects
        msg = pb.AROS_Command()
        rsp = pb.Simulator_Response()

        try:
            msg.command = pb.COMMAND.TTC_GET_MODE
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.TTC_OFF
            print(f"{self.port}: Successfully got response that TTC is off")
        except:
            print(f"{self.port}: Failed to check that TTC is off")

        try:
            msg.command = pb.COMMAND.TTC_SET_CONNECTING
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.GEN_SUCCESS
            print(f"{self.port}: Successfully set TTC to connecting")
        except:
            print(f"{self.port}: Failed to set TTC to connecting")
            return

        connected = False

        while not connected:
            try:
                msg.command = pb.COMMAND.TTC_GET_MODE
                msgString = msg.SerializeToString()
                self.send(msgString)
                rspString = self.recv()

                rsp.ParseFromString(rspString)

                if rsp.response == pb.RESPONSE.TTC_ESTABLISHED_CONT:
                    connected = True
            except:
                print(f"{self.port}: Failed to check TTC mode when waiting for connection")
                return

        try:
            msg.command = pb.COMMAND.TTC_SEND_BYTE_STRING
            msg.byte_string = b'This is a test of send byte string function\nThis would be used for testing\nor responses to command'
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.GEN_SUCCESS
            print(f"{self.port}: Successfully sent byte string to TTC")
        except:
            print(f"{self.port}: Failed to send byte string to TTC:")
            return

        try:
            msg.command = pb.COMMAND.TTC_SEND_HEALTH
            test_health_data = '[This is a test health string, generated by interface_test.py'
            for i in range(0,50):
                test_health_data += f', {i*random.randint(1,10)}'
            test_health_data += ']'
            msg.byte_string = test_health_data.encode('utf-8')

            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.GEN_SUCCESS
            print(f"{self.port}: Successfully sent health data to TTC")
        except:
            print(f"{self.port}: Failed to send health data to TTC:")
            return



        try:
            sending = True
            f = open('test_input/StarWars3.wav', 'rb')
            file = f.read()
            f.close()

            print(f"{self.port}: Successfully opened file to send")

            while sending:
                msg.command = pb.COMMAND.TTC_SEND_AUDIO
                if file == b'':
                    # print(f"Last {len(file)}")
                    file_part = b''
                    sending = False
                elif len(file) < 100:
                    # print(f"Second last {len(file)}")
                    file_part = file
                    file = b''
                else:
                    #print(f"Regular {len(file)}")
                    file_part = file[:100]
                    file = file[100:]

                msg.byte_string = file_part

                msgString = msg.SerializeToString()
                self.send(msgString)
                rspString = self.recv()

                rsp.ParseFromString(rspString)

                assert rsp.response == pb.RESPONSE.GEN_SUCCESS
            print(f"{self.port}: Successfully sent file to to TTC")
        except:
            print(f"{self.port}: Failed to send file to TTC:")
            return

        try:
            msg.command = pb.COMMAND.TTC_SEND_BYTE_STRING
            msg.byte_string = b'Please send a command to test'
            msgString = msg.SerializeToString()
            self.send(msgString)
            rspString = self.recv()

            rsp.ParseFromString(rspString)

            assert rsp.response == pb.RESPONSE.GEN_SUCCESS
            # print(f"{self.port}: Successfully sent second byte string to TTC")
        except:
            # print(f"{self.port}: Failed to send second byte string to TTC:")
            return

        try:
            waiting = True
            while waiting:
                msg.command = pb.COMMAND.TTC_GET_COMMAND
                msgString = msg.SerializeToString()
                self.send(msgString)
                rspString = self.recv()

                rsp.ParseFromString(rspString)

                assert rsp.response == pb.RESPONSE.TTC_RETURN_COMMAND or rsp.response == pb.RESPONSE.GEN_ERROR
                if rsp.response == pb.RESPONSE.TTC_RETURN_COMMAND:
                    assert rsp.HasField('byte_string')
                    command = rsp.byte_string
                    waiting = False
                    break

            command = command.decode('utf-8')
            print(f"{self.port}: Successfully recevived command from TTC: \"{command}\"")

        except:
            print(f"{self.port}: Failed to receive command form TTC:")
            return



if __name__ == "__main__":
    """
    Tests the generic functionality of the interfaceLAN objects from interfaces.py.
    Run this test code when simulation is already running.
    """
    test_systems = []
    # list of ports used by systems
    ports = (8001, 8002, 8003, 8004, 8005, 8006, 8007, 8008)

    print("Beginning Regular testing")

    for port in ports:
        # For each system, create a tester with its port and connect to it
        system_tester = interface_testerLAN(port)
        test_systems.append(system_tester)
        system_tester.connect()

    for system_tester in test_systems:
        # For each system, ping it and check correct response
        system_tester.ping()

    for system_tester in test_systems:
        # For each system, request and receive all health data
        system_tester.get_health()

    test_systems[0].test_eps()

    #test_systems[4].test_pi_VHF_file()

    #test_systems[7].test_ttc_gc_comms()

    test_systems[6].test_adcs_vectors()

    print("Finished Regular testing, Beginning sporadic Pinging")

    cnt = 0
    while True:
        # Randomly ping one system ever 1000000000 cycles (~every minute).
        # This is needed since test code shouldn't close until the simulator closes,
        # since it will cause crashed threads in the simulator.
        cnt += 1
        if cnt == 1000000000:
            # Ping is used since its good for continued testing of connectivity
            random.choice(test_systems).ping()
            cnt = 0

