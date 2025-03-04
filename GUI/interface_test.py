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
            assert rsp.response == pb.RESPONSE.GEN_RETURN_SINGLE and rsp.HasField('single')
            voltage = rsp.single
            rsp.ParseFromString(rspString2)
            assert rsp.response == pb.RESPONSE.GEN_RETURN_SINGLE and rsp.HasField('single')
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

            assert rsp.response == pb.RESPONSE.GEN_RETURN_SINGLE and rsp.HasField('single')
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


if __name__ == "__main__":
    """
    Tests the generic functionality of the interfaceLAN objects from interfaces.py.
    Run this test code when simulation is already running.
    """
    test_systems = []
    # list of ports used by systems
    ports = (8001, 8002, 8003, 8004, 8005, 8006, 8007, 8008)

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

    test_systems[4].test_pi_VHF_file()

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

