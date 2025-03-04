import numpy as np
import random
import time
from systems import ADCS_mode, EPSState

# Constants
mu = 398600.4418  # Earth's gravitational parameter, km^3/s^2
Re = 6378.1  # Earth's radius, km
omega_earth = 7.2921159e-5  # Earth's angular velocity, rad/s

# Rotational constants
AV = 50  # Angular velocity ranges (divided by 10)

# Charging constants
rate = 0.01  # Charging and Discharging rate of battery

class simulator:
    """
    Simulates the movement of the satellite in orbit by stepping through it in small increments of time. Are able to set
    with initial conditions and changed in the GUI.

    The core code was generated using ChatGPT since orbital mechanics are above my pay grade.
    """

    def __init__(self, GNSS, ADCS, EPS, controller):

        self.running = False
        self.GNSS = GNSS
        self.ADCS = ADCS
        self.EPS = EPS
        self.controller = controller

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
        self.angular_velocity = [random.randint(-AV, AV)/10, random.randint(-AV, AV)/10, random.randint(-AV, AV)/10]
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

    def propagate_orbit(self):
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

    def propagate_angular_velocity(self):
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

    def propagate_charge(self):
        if self.EPS.status == EPSState.MANUAL:
            # If manual return
            return
        elif self.EPS.status == EPSState.DECREASING or (self.position[0] < 0 and self.EPS.status == EPSState.SIMULATED):
            # If decreasing then decrease charge with every time step
            if self.EPS.charge > 0:
                self.EPS.charge -= (rate/2 if self.EPS.power_saving else rate) * self.dt
            elif self.EPS.charge < 0:
                self.EPS.charge = 0

        elif self.EPS.status == EPSState.CHARGING or (self.position[0] > 0 and self.EPS.status == EPSState.SIMULATED):
            # If charging then increase charge with every time step
            if self.EPS.charge < 100:
                self.EPS.charge += (rate * 2 if self.EPS.power_saving else rate) * self.dt
            elif self.EPS.charge > 100:
                self.EPS.charge = 100

    def doTimeStep(self):
        """
        Advacne time forward one timestep, calculate new position and send it to GNSS
        """
        self.propagate_orbit()
        self.propagate_angular_velocity()
        self.propagate_charge()
        self.time = self.time + self.dt

        lat, long, alt = self.cartesian_to_geodetic()
        self.GNSS.simulate(lat, long, alt)

        self.ADCS.simulate(self.angel)


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
