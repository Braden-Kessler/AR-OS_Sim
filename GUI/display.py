from abc import ABC, abstractmethod
import PySimpleGUI as sg
import numpy as np
import time
from systems import EPSState, ESPState, GNSS_ADCSState, GNSS_TRAIL_SIZE, ADCS_mode, TTC_mode, TTC_GS_status
from globe import GLOBE

KINGSTON = (-76.4930, 44.2334)

class displayController:
    """
    Main GUI controller class which generates each GUI window and processes all GUI events. Uses the PySimpleGUI for all
    GUI elements.
    """

    def __init__(self, controller):
        print("Display Controller Started")
        # sg.theme('DarkPurple2')
        sg.theme('DarkGrey12')
        self.controller = controller
        self.simulator = None

        self.systemDisplays = []
        self.layoutControl = [[sg.Text('Systems')],
                              [sg.Button('Exit', key='-CLOSE-', size=(10, 1)), sg.Button('Simulator', key='-SIM-', size=(10, 1))],
                              [sg.Radio('Static Refresh', group_id=1, default=True, enable_events=True, key='-STATIC-'), sg.Radio('Auto Refresh', group_id=1, enable_events=True, key='-AUTO-')],
                              [sg.HorizontalSeparator()]
                              ]
        self.windowControl = None

        self.autoRefresh = False

        self.sohPopup = sohPopup()

    def run(self, _):
        print("Display Controller Running")
        self.windowControl = sg.Window('Control', self.layoutControl, finalize=True)

        while True:  # The Event Loop
            window, event, values = sg.read_all_windows()

            # Main check for if the primary window has been closed
            if window == self.windowControl and (event == sg.WIN_CLOSED or event == '-CLOSE-'):
                for display in self.systemDisplays:
                    display.close()
                self.windowControl.close()
                self.controller.close = True
                return
            # Check if main window clicked, if yes check for Radio button or generate appropriate sub window
            elif window == self.windowControl:
                if event == '-AUTO-':
                    self.autoRefresh = True
                elif event == '-STATIC-':
                    self.autoRefresh = False
                else:
                    for display in self.systemDisplays:
                        if event == f'-{display.name}-':
                            display.generateWindow()
                            break
            # Check if event was for an SoH popup window, if so enter the popup which is a blocking call
            elif event == '-SOH-':
                for display in self.systemDisplays:
                    if display.window and display.window == window:
                        self.sohPopup.generatePopup(display.system)
                        break
            # Else check each display if the event is theirs, if yes hand event and value to sub-windows to handle
            else:
                for display in self.systemDisplays:
                    if display.window and display.window == window:
                        display.handleEvent(event, values)
                        break

    def addSystem(self, system, sysDisplay):
        # print(f"{system.name} added to Display")
        tempDisplay = sysDisplay(system)
        self.layoutControl.append([sg.Button(f'{system.name}', key=f'-{system.name}-', size=(10, 1))])
        self.systemDisplays.append(tempDisplay)

    def addSimulator(self, simulator):
        self.simulator = simulator
        tempDisplay = simulatorDisplay(self.simulator)
        self.systemDisplays.append(tempDisplay)

    def autoRefresher(self, _):
        print("Thread Refresher running")
        while not self.controller.close:
            if self.windowControl is not None and self.autoRefresh:
                for display in self.systemDisplays:
                    display.refresh()
            time.sleep(1)
        print("Thread Refresher Closed")


class simulatorDisplay:

    def __init__(self, simulator):
        self.name = "SIM"
        self.simulator = simulator
        self.layout = None
        self.generateLayout()
        self.window = None
        self.debug = False
        self.realTime = False

    def generateLayout(self):

        self.layout = [[sg.Button('Refresh', key='-REFRESH-'), sg.Button('Close', key='-CLOSE-')],
                       [sg.HorizontalSeparator()],
                       [sg.Text('Simulation Settings:\t'), sg.Button('Initialize', key='-INIT-', size=(16, 1))],
                       [sg.Text("Time of simulation\t\t"), sg.Text(f"{self.simulator.time}s\t", key='-TIME-')],
                       [sg.Text("Time step\t\t"), sg.Text(f"{self.simulator.dt}s\t", key='-DT-'),
                        sg.Input(size=(16, 1), key='-INPUT_DT-'),
                        sg.Button('Set Time step', key='-SET_DT-', size=(16, 1))],
                       [sg.Text("Semi-major axis\t\t"), sg.Text(f"{self.simulator.semiMajor}km\t", key='-SEMI-MAJOR-'),
                        sg.Input(size=(16, 1), key='-INPUT_SEMI-MAJOR-'),
                        sg.Button('Set Semi-major', key='-SET_SEMI-MAJOR-', size=(16, 1))],
                       [sg.Text("Eccentricity\t\t"), sg.Text(f"{self.simulator.eccentricity}\t", key='-ECCENT-'),
                        sg.Input(size=(16, 1), key='-INPUT_ECCENT-'),
                        sg.Button('Set Eccentricity', key='-SET_ECCENT-', size=(16, 1))],
                       [sg.Text("Inclination\t\t"), sg.Text(f"{np.round(np.degrees(self.simulator.inclination), decimals=5)}°\t", key='-INCLI-'),
                        sg.Input(size=(16, 1), key='-INPUT_INCLI-'),
                        sg.Button('Set Inclination', key='-SET_INCLI-', size=(16, 1))],
                       [sg.Text("Right Ascension of Ascending Node\t", size=(20, 2)),
                        sg.Text(f"{np.round(np.degrees(self.simulator.raan), decimals=5)}°\t", key='-RAAN-'),
                        sg.Input(size=(16, 1), key='-INPUT_RAAN-'),
                        sg.Button('Set RAAN', key='-SET_RAAN-', size=(16, 1))],
                       [sg.Text("Argument of Periapsis\t"),
                        sg.Text(f"{np.round(np.degrees(self.simulator.arg_periapsis), decimals=5)}°\t", key='-ARGP-'),
                        sg.Input(size=(16, 1), key='-INPUT_ARGP-'),
                        sg.Button('Set Arg of Periapsis', key='-SET_ARGP-', size=(16, 1))],
                       [sg.Text("True Anomaly at epoch\t"),
                        sg.Text(f"{np.round(np.degrees(self.simulator.true_anomaly), decimals=5)}°\t", key='-TRA-'),
                        sg.Input(size=(16, 1), key='-INPUT_TRA-'),
                        sg.Button('Set True Anomaly', key='-SET_TRA-', size=(16, 1))],
                       [sg.HorizontalSeparator()],
                       [sg.Text('Simulation Running:\t'), sg.Button("Do Time Step", key='-TIMESTEP-', size=(16, 1))],
                       [sg.Button('Run until', key='-UNTIL-', size=(16, 1)),
                        sg.Input(size=(16, 1), key='-INPUT_UNTIL-')],
                       [sg.Button('Run for', key='-FOR-', size=(16, 1)),
                        sg.Input(size=(16, 1), key='-INPUT_FOR-')],
                       [sg.Radio('Enable Real Time', group_id=1, enable_events=True, key='-RT_ON-'),
                        sg.Radio('Disable Real Time', group_id=1, default=True, enable_events=True, key='-RT_OFF-')],
                       [sg.HorizontalSeparator()],
                       [sg.Text('Simulation Debugger:\t'), sg.Button('Show Debug', key='-DEBUG-', size=(16, 1))],
                       [sg.Text('Position:', key='-P-', visible=False),
                        sg.Text(f'X: {self.simulator.position[0]}', key='-PX-', visible=False),
                        sg.Text(f'Y: {self.simulator.position[1]}', key='-PY-', visible=False),
                        sg.Text(f'Z: {self.simulator.position[2]}', key='-PZ-', visible=False)],
                       [sg.Text('Velocity:', key='-V-', visible=False),
                        sg.Text(f'X: {self.simulator.velocity[0]}', key='-VX-', visible=False),
                        sg.Text(f'Y: {self.simulator.velocity[1]}', key='-VY-', visible=False),
                        sg.Text(f'Z: {self.simulator.velocity[2]}', key='-VZ-', visible=False)]
                       ]

    def generateWindow(self):
        if self.window is None:
            # print(f"Display {self.name}")
            self.window = sg.Window("Simulator", self.layout, finalize=True, relative_location=(400, 0))
            self.refresh()
        else:
            # print(f"{self.name} Already Displayed")
            pass

    def close(self):
        if self.window:
            self.window.close()
            self.generateLayout()
            self.window = None

    def refresh(self):
        if self.window:
            if self.debug:
                self.window['-PX-'].update(f'X: {self.simulator.position[0]}')
                self.window['-PY-'].update(f'Y: {self.simulator.position[1]}')
                self.window['-PZ-'].update(f'Z: {self.simulator.position[2]}')
                self.window['-VX-'].update(f'X: {self.simulator.velocity[0]}')
                self.window['-VY-'].update(f'Y: {self.simulator.velocity[1]}')
                self.window['-VZ-'].update(f'Z: {self.simulator.velocity[2]}')
            self.window['-TIME-'].update(f'{self.simulator.time}s\t')
            self.window['-DT-'].update(f"{self.simulator.dt}s\t")
            self.window['-SEMI-MAJOR-'].update(f"{self.simulator.semiMajor}km\t")
            self.window['-ECCENT-'].update(f"{self.simulator.eccentricity}\t")
            self.window['-INCLI-'].update(f"{np.round(np.degrees(self.simulator.inclination), decimals=5)}°\t")
            self.window['-RAAN-'].update(f"{np.round(np.degrees(self.simulator.raan), decimals=5)}°\t")
            self.window['-ARGP-'].update(f"{np.round(np.degrees(self.simulator.arg_periapsis), decimals=5)}°\t")
            self.window['-TRA-'].update(f"{np.round(np.degrees(self.simulator.true_anomaly), decimals=5)}°\t")

    def handleEvent(self, event, values):
        if event == sg.WIN_CLOSED or event == '-CLOSE-':
            self.close()
            return
        elif event == '-REFRESH-':
            pass
        elif event == '-SET_DT-':
            try:
                self.simulator.dt = int(values['-INPUT_DT-'])
            except ValueError:
                pass
        elif event == '-SET_SEMI-MAJOR-':
            try:
                self.simulator.semiMajor = int(values['-INPUT_SEMI-MAJOR-'])
            except ValueError:
                pass
        elif event == '-SET_ECCENT-':
            try:
                self.simulator.eccentricity = float(values['-INPUT_ECCENT-'])
            except ValueError:
                pass
        elif event == '-SET_INCLI-':
            try:
                self.simulator.inclination = np.radians(float(values['-INPUT_INCLI-']))
            except ValueError:
                pass
        elif event == '-SET_RAAN-':
            try:
                self.simulator.raan = np.radians(float(values['-INPUT_RAAN-']))
            except ValueError:
                pass
        elif event == '-SET_ARGP-':
            try:
                self.simulator.arg_periapsis = np.radians(float(values['-INPUT_ARGP-']))
            except ValueError:
                pass
        elif event == '-SET_TRA-':
            try:
                self.simulator.true_anomaly = np.radians(float(values['-INPUT_TRA-']))
            except ValueError:
                pass
        elif event == '-INIT-':
            self.simulator.orbital_elements_to_state_vectors()
            print(self.simulator.position)
            print(self.simulator.velocity)
        elif event == '-DEBUG-':
            if self.debug:
                self.debug = False
            else:
                self.debug = True
            self.window['-P-'].update(visible=self.debug)
            self.window['-PX-'].update(visible=self.debug)
            self.window['-PY-'].update(visible=self.debug)
            self.window['-PZ-'].update(visible=self.debug)
            self.window['-V-'].update(visible=self.debug)
            self.window['-VX-'].update(visible=self.debug)
            self.window['-VY-'].update(visible=self.debug)
            self.window['-VZ-'].update(visible=self.debug)
        elif event == '-TIMESTEP-':
            self.simulator.doTimeStep()
        elif event == '-UNTIL-':
            try:
                self.simulator.desiredTime = int(values['-INPUT_UNTIL-'])
            except ValueError:
                pass
        elif event == '-FOR-':
            try:
                val = int(values['-INPUT_FOR-'])
                self.simulator.desiredTime = self.simulator.desiredTime + val
            except ValueError:
                pass
        elif event == '-RT_ON-':
            self.simulator.realTime = True
        elif event == '-RT_OFF-':
            self.simulator.realTime = False
        self.refresh()


class subDisplay(ABC):
    """
    Abstract class for each system's dedicated window, contains all basic functions to interact with the display
    controller. Only functions specific to each system are left as abstract
    """
    def __init__(self, system):
        self.system = system
        self.name = self.system.name
        self.layout = None
        self.generateLayout()
        self.window = None

    def generateLayout(self):
        self.generateLayoutHeader()
        self.generateLayoutBody()

    @abstractmethod
    def refresh(self):
        pass

    @abstractmethod
    def handleEvent(self, event, value):
        pass

    def generateLayoutHeader(self):
        self.layout = [[sg.Button('Refresh', key='-REFRESH-'), sg.Button('Close', key='-CLOSE-'), sg.Button('Edit SoH', key='-SOH-')],
                       [sg.Text('State of Health (SoH):\t'),
                        sg.Text(f'{self.system.voltage} V\t{self.system.temp}°C\t Port Status: {"CONNECTED" if self.system.interface.connected else "NOT CONNECTED"}', key='-HEALTH-'), ],
                       [sg.HorizontalSeparator()]
                       ]

    @abstractmethod
    def generateLayoutBody(self):
        pass

    def generateWindow(self):
        if self.window is None:
            # print(f"Display {self.name}")
            self.window = sg.Window(self.name, self.layout, finalize=True, relative_location=(400, 0))
            self.refresh()
        else:
            # print(f"{self.name} Already Displayed")
            pass

    def close(self):
        if self.window:
            self.window.close()
            self.generateLayout()
            self.window = None


class epsDisplay(subDisplay):

    def generateLayoutBody(self):
        options = ('Manual', 'Simulated', 'Decreasing', 'Charging')

        self.layout.append([[sg.Text('EPS Charge:\t'), sg.Text(f'{round(self.system.charge,2)}%\t', key='-CHARGE-'),
                            sg.Input(size=(4, 1), key='-INPUT_CHARGE-'), sg.Button('Set Charge', key='-SET_CHARGE-')],
                            [sg.Text('Power Saving:\t'), sg.Text(f'{"ON" if self.system.power_saving else "OFF"}\t', key='-PS-'), sg.Button('Enable', key='-PS_ON-'), sg.Button('Disable', key='-PS_OFF-')],
                            [sg.Text('Sim Status:\t'), sg.Text(f'{self.system.status.name}\t', key='-STATUS-'),
                            sg.Listbox(options, size=(10, len(options)), key='-STATUS_OPTIONS-'), sg.Button('Set Status', key='-SET_STATUS-')]
                            ]
                           )

    def refresh(self):
        if self.window:
            self.window['-HEALTH-'].update(f'{self.system.voltage} V\t{self.system.temp}°C\t Port Status: {"CONNECTED" if self.system.interface.connected else "NOT CONNECTED"}')
            self.window['-CHARGE-'].update(f'{round(self.system.charge, 2)}%\t')
            self.window['-STATUS-'].update(f'{self.system.status.name}\t')
            self.window['-PS-'].update(f'{"ON" if self.system.power_saving else "OFF"}\t')

    def handleEvent(self, event, values):
        if event == sg.WIN_CLOSED or event == '-CLOSE-':
            self.close()
            return
        elif event == '-REFRESH-':
            pass
        elif event == '-SET_CHARGE-':
            try:
                val = int(values['-INPUT_CHARGE-'])
                if val > 100:
                    val = 100
                elif val < 0:
                    val = 0
                self.system.charge = val
            except ValueError:
                pass
        elif event == '-SET_STATUS-':
            if values['-STATUS_OPTIONS-'][0] == 'Manual':
                self.system.status = EPSState.MANUAL
            elif values['-STATUS_OPTIONS-'][0] == 'Simulated':
                self.system.status = EPSState.SIMULATED
            elif values['-STATUS_OPTIONS-'][0] == 'Decreasing':
                self.system.status = EPSState.DECREASING
            elif values['-STATUS_OPTIONS-'][0] == 'Charging':
                self.system.status = EPSState.CHARGING
        elif event == '-PS_ON-':
            self.system.power_saving = True
        elif event == '-PS_OFF-':
            self.system.power_saving = False
        self.refresh()


class espDisplay(subDisplay):

    def generateLayoutBody(self):
        options = ('Off', 'Warming', 'Ready', 'Burning', 'Cooldown')

        self.layout.append([[sg.Text('Fuel level:\t'), sg.Text(f'{round(self.system.fuel, 2)}%\t', key='-FUEL-'),
                            sg.Input(size=(4, 1), key='-INPUT_FUEL-'), sg.Button('Set Fuel Level', key='-SET_FUEL-')],
                            [sg.Text('Engine Status:\t'), sg.Text(f'{self.system.status.name}\t', key='-STATUS-'),
                            sg.Listbox(options, size=(10, len(options)), key='-STATUS_OPTIONS-'), sg.Button('Set Status', key='-SET_STATUS-')],
                            [sg.Text('Engine Temperature Percent:\t'), sg.Text(f'{round(self.system.engine_temp, 2)}%', key='-ENG_TEMP-')]
                            ]
                           )

    def refresh(self):
        if self.window:
            self.window['-HEALTH-'].update(f'{self.system.voltage} V\t{self.system.temp}°C\t Port Status: {"CONNECTED" if self.system.interface.connected else "NOT CONNECTED"}')
            self.window['-FUEL-'].update(f'{round(self.system.fuel, 2)}%\t')
            self.window['-STATUS-'].update(f'{self.system.status.name}\t')
            self.window['-ENG_TEMP-'].update(f'{round(self.system.engine_temp, 2)}%')

    def handleEvent(self, event, values):
        if event == sg.WIN_CLOSED or event == '-CLOSE-':
            self.close()
            return
        elif event == '-REFRESH-':
            pass
        elif event == '-SET_FUEL-':
            try:
                val = int(values['-INPUT_FUEL-'])
                if val > 100:
                    val = 100
                elif val < 0:
                    val = 0
                self.system.fuel = val
            except ValueError:
                pass
        elif event == '-SET_STATUS-':
            if values['-STATUS_OPTIONS-'][0] == 'Off':
                self.system.status = ESPState.OFF
            elif values['-STATUS_OPTIONS-'][0] == 'Warming':
                self.system.status = ESPState.WARMING
            elif values['-STATUS_OPTIONS-'][0] == 'Ready':
                self.system.status = ESPState.READY
            elif values['-STATUS_OPTIONS-'][0] == 'Burning':
                self.system.status = ESPState.BURNING
            elif values['-STATUS_OPTIONS-'][0] == 'Cooldown':
                self.system.status = ESPState.COOLDOWN
        self.refresh()


class dragSailDisplay(subDisplay):

    def generateLayoutBody(self):
        options = ('Off', 'Warming', 'Ready', 'Burning', 'Cooldown')

        self.layout.append([[sg.Text('Drag Sail Status:\t'), sg.Text(f'{"ARMED" if not self.system.deployed else "DEPLOYED"}\t', key='-STATUS-')],
                            [sg.Button('Deploy', key='-DEPLOY-', size=(7, 1)), sg.Button('Retract', key='-RETRACT-', size=(7, 1))]
                            ]
                           )

    def refresh(self):
        if self.window:
            self.window['-HEALTH-'].update(f'{self.system.voltage} V\t{self.system.temp}°C\t Port Status: {"CONNECTED" if self.system.interface.connected else "NOT CONNECTED"}')
            self.window['-STATUS-'].update(f'{"ARMED" if not self.system.deployed else "DEPLOYED"}\t')

    def handleEvent(self, event, values):
        if event == sg.WIN_CLOSED or event == '-CLOSE-':
            self.close()
            return
        elif event == '-REFRESH-':
            pass
        elif event == '-DEPLOY-':
            self.system.deployed = True
        elif event == '-RETRACT-':
            self.system.deployed = False
        self.refresh()


class adcsDisplay(subDisplay):

    def generateLayoutBody(self):
        options_status = ('Manual', 'Simulated')
        options_mode = ('Off', 'Detumbling', 'Sun pointing')

        self.layout.append([[sg.Text('Pitch:\t\t'), sg.Text(f'{round(self.system.pitch,2)}°\t', key='-PITCH-'),
                             sg.Input(size=(6, 1), key='-INPUT_PITCH-'), sg.Button('Set Pitch', key='-SET_PITCH-')],
                            [sg.Text('Roll:\t\t'), sg.Text(f'{round(self.system.roll,2)}°\t', key='-ROLL-'),
                             sg.Input(size=(6, 1), key='-INPUT_ROLL-'), sg.Button('Set Roll', key='-SET_ROLL-')],
                            [sg.Text('Yaw:\t\t'), sg.Text(f'{round(self.system.yaw,2)}°\t', key='-YAW-'),
                             sg.Input(size=(6, 1), key='-INPUT_YAW-'), sg.Button('Set Yaw', key='-SET_YAW-')],
                            [sg.Text('ADCS Mode:\t'), sg.Text(f'{self.system.mode.name}\t', key='-MODE-'),
                             sg.Listbox(options_mode, size=(12, len(options_mode)), key='-MODE_OPTIONS-'),
                             sg.Button('Set Mode', key='-SET_MODE-')],
                            [sg.Text('Sim Status:\t'), sg.Text(f'{self.system.status.name}\t', key='-STATUS-'),
                             sg.Listbox(options_status, size=(10, len(options_status)), key='-STATUS_OPTIONS-'),
                             sg.Button('Set Status', key='-SET_STATUS-')]
                            ])

    def refresh(self):
        if self.window:
            self.window['-HEALTH-'].update(f'{self.system.voltage} V\t{self.system.temp}°C\t Port Status: {"CONNECTED" if self.system.interface.connected else "NOT CONNECTED"}')
            self.window['-PITCH-'].update(f'{round(self.system.pitch,2)}°\t')
            self.window['-ROLL-'].update(f'{round(self.system.roll,2)}°\t')
            self.window['-YAW-'].update(f'{round(self.system.yaw,2)}°\t')
            self.window['-STATUS-'].update(f'{self.system.status.name}\t')
            self.window['-MODE-'].update(f'{self.system.mode.name}\t')

    def handleEvent(self, event, values):
        if event == sg.WIN_CLOSED or event == '-CLOSE-':
            self.close()
            return
        elif event == '-REFRESH-':
            pass
        elif event == '-SET_PITCH-':
            try:
                val = int(values['-INPUT_PITCH-'])
                if val > 90.0:
                    val = 90.0
                elif val < -90.0:
                    val = -90.0
                self.system.pitch = val
            except ValueError:
                pass
        elif event == '-SET_ROLL-':
            try:
                val = int(values['-INPUT_ROLL-'])
                if val > 180.0:
                    val = 180.0
                elif val < -180.0:
                    val = -180.0
                self.system.roll = val
            except ValueError:
                pass
        elif event == '-SET_YAW-':
            try:
                val = int(values['-INPUT_YAW-'])
                if val > 180.0:
                    val = 180.0
                elif val < -180.0:
                    val = -180.0
                self.system.yaw = val
            except ValueError:
                pass
        elif event == '-SET_STATUS-':
            if values['-STATUS_OPTIONS-'][0] == 'Manual':
                self.system.status = GNSS_ADCSState.MANUAL
            elif values['-STATUS_OPTIONS-'][0] == 'Simulated':
                self.system.status = GNSS_ADCSState.SIMULATED
        elif event == '-SET_MODE-':
            if values['-MODE_OPTIONS-'][0] == 'Off':
                self.system.mode = ADCS_mode.OFF
            elif values['-MODE_OPTIONS-'][0] == 'Detumbling':
                self.system.mode = ADCS_mode.DETUMBLING
            elif values['-MODE_OPTIONS-'][0] == 'Sun pointing':
                self.system.mode = ADCS_mode.SUN_POINTING
        self.refresh()


class gnssDisplay(subDisplay):

    def __init__(self, system):
        super().__init__(system)
        self.Pi_VHF = None

    def add_Pi_VHF_ref(self, Pi_VHF):
        self.Pi_VHF = Pi_VHF

    def generateLayoutBody(self):
        options = ('Manual', 'Simulated')

        self.layout.append([[sg.Text('Latitude:\t\t'), sg.Text(f'{abs(self.system.latitude)} {"N" if self.system.latitude >= 0 else "S"}\t', key='-LAT-'),
                            sg.Input(size=(10, 1), key='-INPUT_LAT-'), sg.Button('Set Latitude', key='-SET_LAT-')],
                            [sg.Text('Longitude:\t'),
                             sg.Text(f'{abs(self.system.longitude)} {"E" if self.system.longitude >= 0 else "W"}\t',
                                     key='-LONG-'),
                             sg.Input(size=(10, 1), key='-INPUT_LONG-'), sg.Button('Set Latitude', key='-SET_LONG-')],
                            [sg.Text('Elevation:\t'),
                             sg.Text(f'{self.system.elevation}Km\t',
                                     key='-ELA-'),
                             sg.Input(size=(10, 1), key='-INPUT_ELA-'), sg.Button('Set Elevation', key='-SET_ELA-')],
                            [sg.Text('Sim Status:\t'), sg.Text(f'{self.system.status.name}\t', key='-STATUS-'),
                            sg.Listbox(options, size=(10, len(options)), key='-STATUS_OPTIONS-'), sg.Button('Set Status', key='-SET_STATUS-')],
                            [sg.HorizontalSeparator()],
                            [sg.Graph(key='-MAP-', canvas_size=(360*2, 180*2), graph_bottom_left=(-180, -90), graph_top_right=(180, 90), background_color='lightblue')]
                            ]
                           )

    def refresh(self):
        if self.window:
            self.window['-HEALTH-'].update(f'{self.system.voltage} V\t{self.system.temp}°C\t Port Status: {"CONNECTED" if self.system.interface.connected else "NOT CONNECTED"}')
            self.window['-LAT-'].update(f'{abs(self.system.latitude)} {"N" if self.system.latitude >= 0 else "S"}\t')
            self.window['-LONG-'].update(f'{abs(self.system.longitude)} {"E" if self.system.longitude >= 0 else "W"}\t')
            self.window['-ELA-'].update(f'{self.system.elevation}Km\t')
            self.window['-STATUS-'].update(f'{self.system.status.name}\t')
            self.updateMap()

    def updateMap(self):
        draw = self.window['-MAP-']

        draw.Erase()
        # Draws Globe
        self.drawGlobe(draw)

        # Draw kingston at 44.2334 N and 76.4930 W
        draw.DrawPoint(KINGSTON, size=3, color='green')

        # Draw sonarbouy at set coordinates
        draw.DrawPoint((self.Pi_VHF.longitude, self.Pi_VHF.latitude), size=3, color='blue')

        # Draw Audimus trail
        if self.system.trailSize > 0:
            oldPoint = None
            i = 0

            self.system.lock.acquire()

            for point in self.system.trail.queue:
                # Sets colour based on present through GNSS_TRAIL_SIZE, black at end and red at beggning
                red = int(((i+GNSS_TRAIL_SIZE-self.system.trailSize)*255)/GNSS_TRAIL_SIZE)

                if oldPoint is not None:
                    # Stops line jumping when initializing
                    if not self.wrap_around(point[0], oldPoint[0]) and not self.wrap_around(point[1], oldPoint[1]):
                        # Stops line jumping when wrapping around the screen vertically or horizontally
                        draw.draw_line(point, oldPoint, color=("#%02x0000" % red))

                oldPoint = point
                i += 1

            self.system.lock.release()

        # Draw Audimus Position
        draw.DrawPoint((self.system.longitude, self.system.latitude), size=5, color='red')

    def wrap_around(self, i, j):
        if (i > 0 > j) or (i < 0 < j) and (abs(i) + abs(j) > 100):
            return True
        return False

    def drawGlobe(self, draw):
        for island in GLOBE:
            oldPoint = None
            for point in island:
                if oldPoint is not None:
                    draw.draw_line(point, oldPoint, color="black")
                oldPoint = point

    def handleEvent(self, event, values):
        if event == sg.WIN_CLOSED or event == '-CLOSE-':
            self.close()
            return
        elif event == '-REFRESH-':
            pass
        elif event == '-SET_LAT-':
            try:
                val = float(values['-INPUT_LAT-'])
                if val > 90.0:
                    val = 90.0
                elif val < -90.0:
                    val = -90.0
                self.system.latitude = val
            except ValueError:
                pass
        elif event == '-SET_LONG-':
            try:
                val = float(values['-INPUT_LONG-'])
                if val > 180.0:
                    val = 180.0
                elif val < -180.0:
                    val = -180.0
                self.system.longitude = val
            except ValueError:
                pass
        elif event == '-SET_ELA-':
            try:
                val = float(values['-INPUT_ELA-'])
                if val < 200.0:
                    val = 200.0
                self.system.elevation = val
            except ValueError:
                pass
        elif event == '-SET_STATUS-':
            if values['-STATUS_OPTIONS-'][0] == 'Manual':
                self.system.status = GNSS_ADCSState.MANUAL
            elif values['-STATUS_OPTIONS-'][0] == 'Simulated':
                self.system.status = GNSS_ADCSState.SIMULATED
        self.refresh()


class pi_vhfDisplay(subDisplay):

    def generateLayoutBody(self):

        self.layout.append([[sg.Text('Raspberry Pi:')],
                            [sg.Text('VHS Radio Status:\t'), sg.Text(f'{"LISTENING" if self.system.enabled else "OFF"}\t', key='-RADIO_STATUS-')],
                            [sg.Button('Enable', key='-ENABLE-', size=(7, 1)), sg.Button('Disable', key='-DISABLE-', size=(7, 1))],
                            [sg.HorizontalSeparator()],
                            [sg.Text('Sonarbuoy:')],
                            [sg.Text('Connection Range\t'),
                             sg.Text(f'{self.system.connection_radius} km\t', key='-RANGE-'),
                             sg.Input(size=(10, 1), key='-INPUT_RANGE-'), sg.Button('Set Range', key='-SET_RANGE-')],
                            [sg.Text('Latitude:\t\t'),
                             sg.Text(f'{abs(self.system.latitude)} {"N" if self.system.latitude >= 0 else "S"}\t',
                                     key='-LAT-'),
                             sg.Input(size=(10, 1), key='-INPUT_LAT-'), sg.Button('Set Latitude', key='-SET_LAT-')],
                            [sg.Text('Longitude:\t'),
                             sg.Text(f'{abs(self.system.longitude)} {"E" if self.system.longitude >= 0 else "W"}\t',
                                     key='-LONG-'),
                             sg.Input(size=(10, 1), key='-INPUT_LONG-'), sg.Button('Set Latitude', key='-SET_LONG-')],
                            [sg.HorizontalSeparator()],
                            [sg.Text('Enter Audio File to Send:')],
                            [sg.Text('File Path:\t'), sg.Text(f'{self.system.audio_filepath if self.system.audio_filepath != "" else "No File Selected"}', key='-AUDIO_FILEPATH_OUTPUT-')],
                            [sg.Input(key='-AUDIO_FILEPATH-', size=(70, 1)), sg.FileBrowse(key='-BROWSE-', enable_events=True)], #file_types=['typeName {wav},{txt}']
                            [sg.Button('Set File', key='-SET_FILE-')],
                            [sg.Text('Audio Status:\t'), sg.Text(f'{self.system.audio_status.name}\t', key='-AUDIO_STATUS-')]
                            ]
                           )

    def refresh(self):
        if self.window:
            self.window['-HEALTH-'].update(f'{self.system.voltage} V\t{self.system.temp}°C\t Port Status: {"CONNECTED" if self.system.interface.connected else "NOT CONNECTED"}')
            self.window['-RADIO_STATUS-'].update(f'{"LISTENING" if self.system.enabled else "OFF"}\t')
            self.window['-AUDIO_STATUS-'].update(f'{self.system.audio_status.name}\t')
            # self.window['-AUDIO_FILEPATH-'].update(f'{self.system.audio_filepath}')
            self.window['-AUDIO_FILEPATH_OUTPUT-'].update(f'{self.system.audio_filepath if self.system.audio_filepath != "" else "No File Selected"}')
            self.window['-RANGE-'].update(f'{self.system.connection_radius} km\t')
            self.window['-LAT-'].update(f'{abs(self.system.latitude)} {"N" if self.system.latitude >= 0 else "S"}\t')
            self.window['-LONG-'].update(f'{abs(self.system.longitude)} {"E" if self.system.longitude >= 0 else "W"}\t')

    def handleEvent(self, event, values):
        if event == sg.WIN_CLOSED or event == '-CLOSE-':
            self.close()
            return
        elif event == '-REFRESH-':
            pass
        elif event == '-ENABLE-':
            self.system.enabled = True
        elif event == '-DISABLE-':
            self.system.enabled = False
        elif event == '-SET_FILE-' or event == '-BROWSE-':
            self.system.audio_filepath = values['-AUDIO_FILEPATH-']
            print(self.system.audio_filepath)
            self.system.load_file()
        elif event == '-SET_RANGE-':
            try:
                val = float(values['-INPUT_RANGE-'])
                if val < 0:
                    val = 0
                self.system.connection_radius = val
            except ValueError:
                pass
        elif event == '-SET_LAT-':
            try:
                val = float(values['-INPUT_LAT-'])
                if val > 90.0:
                    val = 90.0
                elif val < -90.0:
                    val = -90.0
                self.system.latitude = val
            except ValueError:
                pass
        elif event == '-SET_LONG-':
            try:
                val = float(values['-INPUT_LONG-'])
                if val > 180.0:
                    val = 180.0
                elif val < -180.0:
                    val = -180.0
                self.system.longitude = val
            except ValueError:
                pass
        self.refresh()


class obcDisplay(subDisplay):

    def generateLayoutBody(self):
        pass

    def refresh(self):
        if self.window:
            self.window['-HEALTH-'].update(f'{self.system.voltage} V\t{self.system.temp}°C\t Port Status: {"CONNECTED" if self.system.interface.connected else "NOT CONNECTED"}')

    def handleEvent(self, event, values):
        if event == sg.WIN_CLOSED or event == '-CLOSE-':
            self.close()
            return
        elif event == '-REFRESH-':
            pass
        self.refresh()

class ttcDisplay(subDisplay):
    """
    Link for determing when to connect or not https://www.askpython.com/python/examples/find-distance-between-two-geo-locations
    """
    def generateLayoutBody(self):
        options_mode = ('Off', 'Beaconing', 'Connecting', 'Established Data', 'Established Control', 'Broadcast No Connection', 'Disconnected')

        self.layout.append([[sg.Text('TTC:')],
                            [sg.Text('TTC Mode:\t'), sg.Text(f'{self.system.mode.name}\t', key='-MODE-'),
                             sg.Listbox(options_mode, size=(len('Broadcast No Connection'), len(options_mode)), key='-MODE_OPTIONS-'),
                             sg.Button('Set Mode', key='-SET_MODE-')],
                            [sg.HorizontalSeparator()],
                            [sg.Text('Ground Station:')],
                            [sg.Text('Connection Range\t'), sg.Text(f'{self.system.connection_radius} km\t', key='-RANGE-'),
                            sg.Input(size=(10, 1), key='-INPUT_RANGE-'), sg.Button('Set Range', key='-SET_RANGE-')],
                            [sg.Radio('No Response', group_id=1, default=True, enable_events=True, key='-NO_RESP-'), sg.Radio('Connect Data', group_id=1, enable_events=True, key='-CON_DATA-'), sg.Radio('Connect Control', group_id=1, enable_events=True, key='-CON_CONT-')],
                            [sg.HorizontalSeparator()],
                            [sg.Text('Communication Channel:\t Output:')],
                            [sg.Multiline(write_only=True, size=(100, 20), key='-OUTPUT-')],
                            [sg.Text('Input:')],
                            [sg.Input(size=(90, 1), key='-INPUT_COMMAND-'), sg.Button('Send', key='-SEND_COMMAND-', size=(10, 1))]
                            ])

    def refresh(self):
        if self.window:
            self.window['-HEALTH-'].update(f'{self.system.voltage} V\t{self.system.temp}°C\t Port Status: {"CONNECTED" if self.system.interface.connected else "NOT CONNECTED"}')
            self.window['-MODE-'].update(f'{self.system.mode.name}\t')
            self.window['-RANGE-'].update(f'{self.system.connection_radius} km\t')
            if self.system.console_output != "":
                self.window['-OUTPUT-'].print(self.system.console_output)
                self.system.console_output = ""

    def handleEvent(self, event, values):
        if event == sg.WIN_CLOSED or event == '-CLOSE-':
            self.close()
            return
        elif event == '-REFRESH-':
            pass
        elif event == '-SET_MODE-':
            if values['-MODE_OPTIONS-'][0] == 'Off':
                self.system.mode = TTC_mode.OFF
            elif values['-MODE_OPTIONS-'][0] == 'Beaconing':
                self.system.mode = TTC_mode.BEACONING
            elif values['-MODE_OPTIONS-'][0] == 'Connecting':
                self.system.mode = TTC_mode.CONNECTING
            elif values['-MODE_OPTIONS-'][0] == 'Established Data':
                self.system.mode = TTC_mode.ESTABLISHED_DATA
            elif values['-MODE_OPTIONS-'][0] == 'Established Control':
                self.system.mode = TTC_mode.ESTABLISHED_CONT
            elif values['-MODE_OPTIONS-'][0] == 'Broadcast No Connection':
                self.system.mode = TTC_mode.BROADCAST_NO_CON
            elif values['-MODE_OPTIONS-'][0] == 'Disconnected':
                self.system.mode = TTC_mode.DISCONNECTED
        elif event == '-SET_RANGE-':
            try:
                val = float(values['-INPUT_RANGE-'])
                if val < 0:
                    val = 0
                self.system.connection_radius = val
            except ValueError:
                pass
        elif event == '-NO_RESP-':
            self.system.gs_status = TTC_GS_status.NO_RESPONSE
        elif event == '-CON_DATA-':
            self.system.gs_status = TTC_GS_status.CONNECTION_DATA
        elif event == '-CON_CONT-':
            self.system.gs_status = TTC_GS_status.CONNECTION_CONTROL
        elif event == '-SEND_COMMAND-':
            try:
                command = str(values['-INPUT_COMMAND-'])
                self.system.gs_send_command(command)
            except ValueError:
                pass
        self.refresh()

    def print_to_console(self, msg):
        self.window['-OUTPUT-'].print(msg)


class sohPopup:

    def __init__(self):
        self.system = None
        self.name = None
        self.layout = None
        self.window = None

    def generateLayout(self):
        self.layout = [[sg.Text(f"State of Health for {self.name}")],
                       [sg.Text("Voltage(V):\t"), sg.Input(size=(10, 1), key='-INPUT_VOL-')],
                       [sg.Text("Temperature(°C):\t"), sg.Input(size=(10, 1), key='-INPUT_TEMP-')],
                       [sg.Button('Set', key='-SET-', size=(6, 1)), sg.Button('Cancel', key='-CANCEL-', size=(6, 1))]
                       ]

    def generatePopup(self, system):
        self.system = system
        self.name = system.name
        self.generateLayout()
        self.window = sg.Window(self.name, self.layout, finalize=True, relative_location=(600, 0), force_toplevel=True, keep_on_top=True)

        event, values = self.window.read()

        self.window.close()

        if event == '-SET-':
            if values['-INPUT_VOL-'] != "":
                try:
                    val = float(values['-INPUT_VOL-'])
                    if val < 0:
                        val = 0
                    self.system.voltage = val
                except ValueError:
                    pass
            if values['-INPUT_TEMP-'] != "":
                try:
                    val = float(values['-INPUT_TEMP-'])
                    self.system.temp = val
                except ValueError:
                    pass
        return
