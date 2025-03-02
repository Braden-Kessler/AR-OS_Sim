import socket
import AR_OS_pb2 as pb
import random

HOST = "127.0.0.1"

class interface_testerLAN:

    def __init__(self, port):
        self.port = port
        self.connected = False
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __del__(self):
        self.socket.close()

    def connect(self):
        print(f"{self.port}: Connecting to port ")
        try:
            self.socket.connect((HOST, self.port))
            self.connected = True
            print(f"{self.port}: Connected to port ")
        except:
            print(f"{self.port}: Failed to connect to port")

    def send(self, msg: bytes):
        length = len(msg)
        lengthBytes = length.to_bytes(4, 'little')
        packet = lengthBytes + msg
        self.socket.send(packet)
        return

    def recv(self):
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
        print(f"{self.port}: Trying to ping port")
        if not self.connected:
            print(f"{self.port}: Could not ping port, not connected to in first place")
            return

        msg = pb.AROS_Command()
        rsp = pb.Simulator_Response()

        msg.command = pb.COMMAND.GEN_PING
        msgString = msg.SerializeToString()

        try:
            self.send(msgString)
        except:
            print(f"{self.port}: Could not ping port, failed to send")
            return

        try:
            rspString = self.recv()
        except:
            print(f"{self.port}: Could not ping port, failed to receive")
            return

        rsp.ParseFromString(rspString)
        #print(rsp)

        try:
            assert rsp.response == pb.RESPONSE.GEN_PONG
            print(f"{self.port}: Successfully pinged port")
        except:
            print(f"{self.port}: Could not ping port, received wrong response")

    def get_health(self):
        print(f"{self.port}: Trying to get health data")
        if not self.connected:
            print(f"{self.port}: Could not get health data, not connected to in first place")
            return

        msg = pb.AROS_Command()
        rsp = pb.Simulator_Response()

        msg.command = pb.COMMAND.GEN_GET_VOLTAGE
        msgString1 = msg.SerializeToString()
        msg.command = pb.COMMAND.GEN_GET_TEMP
        msgString2 = msg.SerializeToString()

        try:
            self.send(msgString1)
            rspString1 = self.recv()
            self.send(msgString2)
            rspString2 = self.recv()
        except:
            print(f"{self.port}: Could not get health data, failed to send or receive")
            return



        try:
            rsp.ParseFromString(rspString1)
            assert rsp.response == pb.RESPONSE.GEN_RETURN_SINGLE and rsp.HasField('single')
            voltage = rsp.single
            rsp.ParseFromString(rspString2)
            assert rsp.response == pb.RESPONSE.GEN_RETURN_SINGLE and rsp.HasField('single')
            temp = rsp.single
            print(f"{self.port}: Successfully got health data {voltage}V and {temp}°C")
        except:
            print(f"{self.port}: Could not get health data, received wrong responses")

if __name__ == "__main__":

    test_systems = []
    for port in (8001, 8002, 8003, 8004, 8005, 8006, 8007):
        system_tester = interface_testerLAN(port)
        test_systems.append(system_tester)
        system_tester.connect()

    for system_tester in test_systems:
        system_tester.ping()

    for system_tester in test_systems:
        system_tester.get_health()

    cnt = 0
    while True:
        cnt += 1
        if cnt == 1000000000:
            random.choice(test_systems).ping()
            cnt = 0

