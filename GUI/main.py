from threading import Thread
from display import displayController
from systems import EPS, ESP, dragSail, GNSS, Pi_VHF, OBC, ADCS, TTC
from display import epsDisplay, espDisplay, dragSailDisplay, gnssDisplay, pi_vhfDisplay, obcDisplay, adcsDisplay, ttcDisplay
from interfaces import interfaceLAN_EPS, interfaceLAN_ESP, interfaceLAN_dragSail, interfaceLAN_GNSS, interfaceLAN_Pi_VHF, interfaceLAN_OBC, interfaceLAN_ADCS, interfaceLAN_TTC
from simulator import simulator

import PySimpleGUI as sg


class controller:
    """
    Primary Controller class for the application, creates all systems and passes their objects to the display controller
    to create the appropriate display. When run will create a new thread for the display controller and for each
    systems network code.
    """

    def __init__(self):
        self.close = False
        self.systems = []
        self.threads = []

        print("Controller Started")
        self.displayController = displayController(self)

        # Global Navigation Satellite System
        self.GNSS = GNSS("GNSS", self, 8004)
        self.GNSS.add_interface(interfaceLAN_EPS)
        self.displayController.addSystem(self.GNSS, gnssDisplay)
        self.systems.append(self.GNSS)

        # Attitude Direction Control System
        self.ADCS = ADCS("ADCS", self, 8007)
        self.ADCS.add_interface(interfaceLAN_ADCS)
        self.displayController.addSystem(self.ADCS, adcsDisplay)
        self.systems.append(self.ADCS)

        # Electrical Power System
        self.EPS = EPS("EPS", self, 8001)
        self.EPS.add_interface(interfaceLAN_EPS)
        self.displayController.addSystem(self.EPS, epsDisplay)
        self.systems.append(self.EPS)

        # Raspberry Pi and VHF Radio
        self.Pi_VHF = Pi_VHF("Pi VHF", self, 8005)
        self.Pi_VHF.add_interface(interfaceLAN_Pi_VHF)
        self.displayController.addSystem(self.Pi_VHF, pi_vhfDisplay)
        self.systems.append(self.Pi_VHF)
        # Adds reference of Pi VHF to GNSS sub display for map drawing purposes
        self.displayController.systemDisplays[0].add_Pi_VHF_ref(self.Pi_VHF)

        # Electro-Spray Propulsion
        self.ESP = ESP("ESP", self, 8002)
        self.ESP.add_interface(interfaceLAN_ESP)
        self.displayController.addSystem(self.ESP, espDisplay)
        self.systems.append(self.ESP)

        # Drag Sail
        self.dragSail = dragSail("Drag Sail", self, 8003)
        self.dragSail.add_interface(interfaceLAN_dragSail)
        self.displayController.addSystem(self.dragSail, dragSailDisplay)
        self.systems.append(self.dragSail)

        # On Board Computer
        self.OBC = OBC("OBC", self, 8006)
        self.OBC.add_interface(interfaceLAN_OBC)
        self.displayController.addSystem(self.OBC, obcDisplay)
        self.systems.append(self.OBC)

        # Telemetry Tracking and Command
        self.TTC = TTC("TTC/GS", self, 8008)
        self.TTC.add_interface(interfaceLAN_TTC)
        self.displayController.addSystem(self.TTC, ttcDisplay)
        self.systems.append(self.TTC)

        # Simulator
        self.simulator = simulator(self)
        self.displayController.addSimulator(self.simulator)

    def run(self):
        print("Controller Running")

        print("Controller creating display")
        tempThread = Thread(target=self.displayController.run, args=(1,))
        tempThread.start()
        self.threads.append(tempThread)

        print("Controller creating Refresher")
        tempThread = Thread(target=self.displayController.autoRefresher, args=(1,))
        tempThread.start()
        self.threads.append(tempThread)

        print("Controller starting port threads")
        for system in self.systems:
            tempThread = Thread(target=system.run, args=(1,))
            tempThread.start()
            self.threads.append(tempThread)

        print("Controller running simulator thread")
        tempThread = Thread(target=self.simulator.run, args=(1,))
        tempThread.start()
        self.threads.append(tempThread)

        for thread in self.threads:
            thread.join()
        print("Controller Closed")
        return


if __name__ == "__main__":
    # sg.preview_all_look_and_feel_themes()

    SimController = controller()
    SimController.run()
