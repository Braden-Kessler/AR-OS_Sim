import numpy as np
import random
import time
from haversine import haversine, Unit
from systems import ADCS_mode, EPSState, TTC_mode, TTC_GS_status, ESPState
from display import KINGSTON

# Constants
mu = 398600.4418  # Earth's gravitational parameter, km^3/s^2
Re = 6378.1  # Earth's radius, km
omega_earth = 7.2921159e-5  # Earth's angular velocity, rad/s

# Rotational constants
AV_RANGE = 50  # Angular velocity ranges (divided by 10)

# Charging constants for Power Supply
EPS_RATE = 0.01  # Charging and Discharging rate of battery

# Heating and burn constants for ESP
ESP_HEAT_RATE = 1
ESP_FUEL_RATE = 0.5

class simulator:
    """
    Simulates the movement of the satellite in orbit by stepping through it in small increments of time. Are able to set
    with initial conditions and changed in the GUI.

    The core code was generated using ChatGPT since orbital mechanics are above my pay grade.
    """

    def __init__(self, controller):

        self.running = False
        self.controller = controller
        self.GNSS = self.controller.GNSS
        self.ADCS = self.controller.ADCS
        self.EPS = self.controller.EPS
        self.ESP = self.controller.ESP
        self.Pi_VHF = self.controller.Pi_VHF
        self.TTC = self.controller.TTC

        # Orbital elements (default values)
        self.semiMajor = 7000  # Semi-major axis, km
        self.eccentricity = 0.001  # Eccentricity
        self.inclination = np.radians(45)  # Inclination, radians
        self.raan = np.radians(30)  # Right Ascension of Ascending Node, radians
        self.arg_periapsis = np.radians(60)  # Argument of Periapsis, radians
        self.true_anomaly = np.radians(0)  # True Anomaly at epoch, radians

        # Time
        self.dt = 10  # seconds
        self.time = 0
        self.desiredTime = 0

        # Real time simulation variables
        self.realTime = False
        self.startTime = 0

        # Position and velocity vectors, used once initialized
        self.position = [0, 0, 0]
        self.velocity = [0, 0, 0]

        # Pitch Roll and Yaw (and their angular velocity) for ADCS
        self.angel = [0, 0, 0]
        self.angular_velocity = [random.randint(-AV_RANGE, AV_RANGE) / 10, random.randint(-AV_RANGE, AV_RANGE) / 10, random.randint(-AV_RANGE, AV_RANGE) / 10]
        self.tumbling = True


    def orbital_elements_to_state_vectors(self):
        """
        Initialize a new orbit based on parameters, clears trail, and resets all time variables
        """
        # Calculate position and velocity vectors in the orbital plane
        r = self.semiMajor * (1 - self.eccentricity ** 2) / (1 + self.eccentricity * np.cos(self.true_anomaly))
        h = np.sqrt(mu * self.semiMajor * (1 - self.eccentricity ** 2))

        # Position in orbital plane
        rx_orbit = r * np.cos(self.true_anomaly)
        ry_orbit = r * np.sin(self.true_anomaly)
        rz_orbit = 0

        # Velocity in orbital plane
        vx_orbit = -np.sqrt(mu / self.semiMajor) * np.sin(self.true_anomaly)
        vy_orbit = np.sqrt(mu / self.semiMajor) * (self.eccentricity + np.cos(self.true_anomaly))
        vz_orbit = 0

        # Rotation matrices
        R3_W = np.array([[np.cos(-self.raan), np.sin(-self.raan), 0],
                         [-np.sin(-self.raan), np.cos(-self.raan), 0],
                         [0, 0, 1]])
        R1_i = np.array([[1, 0, 0],
                         [0, np.cos(-self.inclination), np.sin(-self.inclination)],
                         [0, -np.sin(-self.inclination), np.cos(-self.inclination)]])
        R3_w = np.array([[np.cos(-self.arg_periapsis), np.sin(-self.arg_periapsis), 0],
                         [-np.sin(-self.arg_periapsis), np.cos(-self.arg_periapsis), 0],
                         [0, 0, 1]])

        # Combined rotation
        R = R3_W @ R1_i @ R3_w

        # Position and velocity in ECI frame
        self.position = R @ np.array([rx_orbit, ry_orbit, rz_orbit])
        self.velocity = R @ np.array([vx_orbit, vy_orbit, vz_orbit])
        self.GNSS.clear()
        self.time = 0
        self.desiredTime = 0
        self.startTime = 0

    def update_orbit(self):
        """
        Propagate using simple numerical integration (Euler's method for simplicity)
        """
        r_new = self.position + self.velocity * self.dt
        v_new = self.velocity - (mu / np.linalg.norm(self.position) ** 3) * self.position * self.dt

        self.position = r_new
        self.velocity = v_new

    def cartesian_to_geodetic(self):
        """
        Convert from ECI to latitude, longitude, and elevation
        """

        x, y, z = self.position
        theta = omega_earth * self.time
        long = np.degrees(np.arctan2(y * np.cos(theta) - x * np.sin(theta),
                                x * np.cos(theta) + y * np.sin(theta)))
        lat = np.degrees(np.arctan2(z, np.sqrt(x ** 2 + y ** 2)))
        alt = np.linalg.norm(self.position) - Re
        return lat, long, alt

    def update_angular_velocity(self):
        """
        Propagates angular velocities to current orientation, apply ADCS state to correct tumbling and error.
        """
        for i in range(0, 3):
            # For each of the three angels of rotation, propagate the initialized velocity using time step
            self.angel[i] += self.angular_velocity[i] * self.dt
            # Loop around each value so stays between -180 and 180
            if self.angel[i] > 180:
                self.angel[i] -= 360
            elif self.angel[i] < -180:
                self.angel[i] += 360

        if self.ADCS.mode == ADCS_mode.DETUMBLING and self.tumbling:
            # detumbling decrees angular velocity
            for i in range(0, 3):
                if abs(self.angular_velocity[i]) < 0.2 * self.dt:
                    # if close enough to zero, set to zero
                    self.angular_velocity[i] = 0
                # else slowly move towards zero
                elif self.angular_velocity[i] > 0:
                    self.angular_velocity[i] -= 0.1 * self.dt
                elif self.angular_velocity[i] < 0:
                    self.angular_velocity[i] += 0.1 * self.dt
            if self.angular_velocity == [0, 0, 0]:
                self.tumbling = False

        if self.ADCS.mode == ADCS_mode.SUN_POINTING and not self.tumbling:
            # detumbling angel to point to sun
            for i in range(0, 3):
                if abs(self.angel[i]) < 2 * self.dt:
                    # if close enough to zero, set to zero
                    self.angel[i] = 0
                # else slowly move towards zero
                elif self.angel[i] > 0:
                    self.angel[i] -= 1 * self.dt
                elif self.angel[i] < 0:
                    self.angel[i] += 1 * self.dt

    def update_charge(self):
        """
        Updates the charge of ESP based on current state and position of Audimus
        """
        if self.EPS.status == EPSState.MANUAL:
            # If manual return, since no change to charge
            return

        # Decrease if set to DECREASING, or simulated and x < 0
        elif self.EPS.status == EPSState.DECREASING or (self.position[0] < 0 and self.EPS.status == EPSState.SIMULATED):
            # If decreasing then decrease charge with every time step, don't go below 0
            if self.EPS.charge > 0:
                self.EPS.charge -= (EPS_RATE / 2 if self.EPS.power_saving else EPS_RATE) * self.dt
            elif self.EPS.charge < 0:
                self.EPS.charge = 0

        # Increase if set to INCREASING, or simulated and x > 0
        elif self.EPS.status == EPSState.CHARGING or (self.position[0] > 0 and self.EPS.status == EPSState.SIMULATED):
            # If charging then increase charge with every time step, dont go above 100
            if self.EPS.charge < 100:
                self.EPS.charge += (EPS_RATE * 2 if self.EPS.power_saving else EPS_RATE) * self.dt
            elif self.EPS.charge > 100:
                self.EPS.charge = 100

    def check_connectivity(self):
        """
        Checks and updates TTC and Pi if within range of points during simulation
        """
        # Determines distance from current location to sonarbouy and ground station
        current = (self.GNSS.latitude, self.GNSS.longitude)
        sonarbouy = (self.Pi_VHF.latitude, self.Pi_VHF.longitude)
        groundStation = (KINGSTON[1], KINGSTON[0]) # inverts order of lat and long since map uses it as x and y

        distance_to_sonarbouy = haversine(current, sonarbouy, unit=Unit.KILOMETERS)
        distance_to_groundStation = haversine(current, groundStation, unit=Unit.KILOMETERS)
        # print(f"distance to sonarbout is {distance_to_sonarbouy} and distance to GS is {distance_to_groundStation}")

        # If within range on sonarbouy, then they are connected for internal purposes, and data is allowed to be sent
        if distance_to_sonarbouy <= self.Pi_VHF.connection_radius:
            self.Pi_VHF.connected = True
        else:
            self.Pi_VHF.connected = False

        if distance_to_groundStation <= self.TTC.connection_radius:
            self.TTC.connected = True
            if self.TTC.gs_status == TTC_GS_status.NO_RESPONSE:
                pass
            elif self.TTC.gs_status == TTC_GS_status.CONNECTION_DATA and (self.TTC.mode == TTC_mode.BEACONING or self.TTC.mode == TTC_mode.CONNECTING or self.TTC.mode == TTC_mode.BROADCAST_NO_CON):
                self.TTC.mode = TTC_mode.ESTABLISHED_DATA
            elif self.TTC.gs_status == TTC_GS_status.CONNECTION_CONTROL and (self.TTC.mode == TTC_mode.BEACONING or self.TTC.mode == TTC_mode.CONNECTING or self.TTC.mode == TTC_mode.BROADCAST_NO_CON):
                self.TTC.mode = TTC_mode.ESTABLISHED_CONT
        else:
            self.TTC.connected = False
            if self.TTC.mode == TTC_mode.ESTABLISHED_DATA or self.TTC.mode == TTC_mode.ESTABLISHED_CONT:
                self.TTC.mode = TTC_mode.DISCONNECTED

    def update_engine(self):
        """
        Updates the heat and fuel levels of the ESP based on ESP state
        """
        if self.ESP.status == ESPState.WARMING:
            # If in warm up phase, increase internal temperature until 100%
            self.ESP.engine_temp += ESP_HEAT_RATE * self.dt
            if self.ESP.engine_temp >= 100:
                # If over 100% temp, then transition to ready state
                self.ESP.engine_temp = 100
                self.ESP.status = ESPState.READY

        elif self.ESP.status == ESPState.BURNING:
            # If in burning phase, decrease fuel level
            self.ESP.fuel -= ESP_FUEL_RATE * self.dt
            if self.ESP.fuel <= 0:
                # If fuel level falls below zero, set to zero and force cooldown
                self.ESP.fuel = 0.0
                self.ESP.status = ESPState.COOLDOWN

        if self.ESP.status == ESPState.COOLDOWN:
            # If in cooldown, decrease internal temperature until back at 0
            self.ESP.engine_temp -= ESP_HEAT_RATE * self.dt
            if self.ESP.engine_temp <= 0:
                # If below or equal to 0% temp, then transition to off state
                self.ESP.engine_temp = 0
                self.ESP.status = ESPState.OFF


    def doTimeStep(self):
        """
        Advacne time forward one timestep, calculate new position and send it to GNSS, calculate new orientation and send
        it to ADCS, update charge in ESP, and connect/disconnect radio systems.
        """
        #
        # Update orbit and angular parameters
        self.update_orbit()
        self.update_angular_velocity()
        # Updates systems that have values tied to time
        self.update_charge()
        self.update_engine()
        # Update simulation time based on time step
        self.time = self.time + self.dt

        # Calculate new coordinates and send to GNSS
        lat, long, alt = (self.cartesian_to_geodetic())
        self.GNSS.simulate(lat, long, alt)

        # Update adcs with calculated angels
        self.ADCS.simulate(self.angel)

        # Update pi and TTCs connectivity based on current location
        self.check_connectivity()




    def run(self, _):
        """
        Main loop for the thread that runs the simulation, only advances time steps when time is less then desired.
        """
        while not self.controller.close:
            # If controller not closed, continue loop
            if self.time < self.desiredTime:
                # If desired greater than current time, advance current time by doing a time step
                self.doTimeStep()
            else:
                time.sleep(1)

            # if real time, advance desired time based on current time from OS
            if self.realTime:
                if self.startTime == 0:
                    # If start time 0, then first frame of real time, set start time to not loose any desired time
                    self.startTime = int(time.time()) - self.desiredTime
                # calculate desired time using current time
                self.desiredTime = time.time() - self.startTime
            elif not self.realTime and self.startTime != 0:
                # If real time off and start time not 0, reset start time to 0.
                self.startTime = 0

        print("Thread for Simulator Closing")
