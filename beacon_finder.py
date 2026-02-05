import gtsam
import random
import math
import multiprocessing as mp
import numpy as np

""" Project to generate ESM measurements captured from a drone flying over the ocean and gathering hits off a 400m tall beacon. """

class LLA_Point:
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt

def lla_to_ecef(lla_point: LLA_Point):
    """ Convert lla to ecef. """
    a = 6378137.0
    f = 1 / 298.257223563
    lat_rad = math.radians(lla_point.lat)
    lon_rad = math.radians(lla_point.lon)
    alt_rad = lla_point.alt
    e2 = f * (2 - f)
    n = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
    x = (n + alt_rad) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (n + alt_rad) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (n * (1 - e2) + alt_rad) * math.sin(lat_rad)
    return x, y, z

class ECEF_Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def to_llapoint(self):
        """ Convert ecef to lla. """
        a = 6378137.0
        f = 1 / 298.257223563
        e2 = f * (2 - f)
        lon = math.atan2(self.y, self.x)
        p = math.sqrt(self.x**2 + self.y**2)
        lat = math.atan2(self.z, p * (1 - e2))
        n = a / math.sqrt(1 - e2 * math.sin(lat)**2)
        alt = p / math.cos(lat) - n
        return LLA_Point(lat, lon, alt)

class State:
    def __init__(self, lat, lon, alt, vx, vy, vz):
        """ Initialize the state of the drone with position in lla and velocity in m/s. """
        self.lla_init = LLA_Point(lat, lon, alt)

        # Convert lla to ecef
        self.ecef_init = lla_to_ecef(self.lla_init)

        self.vx = vx
        self.vy = vy
        self.vz = vz

    def update(self, dt):
        """ Move the drone according to its velocity. """
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt


class FactorGraph:
    def __init__(self):
        self.graph =  gtsam.NonlinearFactorGraph()
        self.radar_measurement_prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))

    def add_pose(self, robot_position: ECEF_Point):
        """ Add a pose to the factor graph. """
        robot_pose = gtsam.Pose3({0, 0, 0}, robot_position) # convert to gtsam pose todo
        prior = gtsam.PriorFactorPose3(1, gtsam.Pose3(robot_pose), self.radar_measurement_prior_noise)
        self.graph.add(prior)

    def add_measurement(self, measurement: tuple):
        """ Add a radar measurement to the factor graph. """
        radar_measurement = gtsam.RadarFactor(1, 2, 3, measurement[0], measurement[1], measurement[2], measurement[3], self.radar_measurement_prior_noise)
        self.graph.add(radar_measurement)


class Drone:
    def __init__(self, initial_state):
        self.state = initial_state
        self.drone_takeoff_velocity = 0.5  # m/s
        self.drone_flight_velocity = 20 # m/s
        self.factor_graph = FactorGraph()

    def plan_path(self, start_location: ECEF_Point, end_location: ECEF_Point, total_states=100):
        # Plan a path from the current position to the beacon - straight line path for now
        self.states = []
        for i in range(total_states):
            x = start_location.x + (end_location.x - start_location.x) * i / total_states
            y = start_location.y + (end_location.y - start_location.y) * i / total_states
            z = start_location.z + (end_location.z - start_location.z) * i / total_states
            self.states.append(ECEF_Point(x, y, z))

    def fly_to(self):
        angle = random.uniform(0, 2 * math.pi)
        dx = distance * math.cos(angle)
        dy = distance * math.sin(angle)
        self.position = (self.position[0] + dx, self.position[1] + dy)

    def takeoff(self):
        self.state.vz = 10

    # def spiral(self, location, theta):
    #     """ Fly in a spiral pattern around a location with a constant angular velocity. """
    #     # generate the ecef points of the spiral
    #     r = 


class Beacon:
    def __init__(self, height):
        self.height = height
        self.position = (random.uniform(-1000, 1000), random.uniform(-1000, 1000))

    def get_signal_strength(self, drone_position):
        distance = math.sqrt((self.position[0] - drone_position[0])**2 + (self.position[1] - drone_position[1])**2)
        return self.height / (distance)

    def get_azel_from_positions(self, drone_position):
        dx = self.position[0] - drone_position[0]
        dy = self.position[1] - drone_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        azimuth = math.atan2(dy, dx)
        elevation = math.atan2(self.height, distance)
        return azimuth, elevation


class Simulation:
    def __init__(self):
        # mission parameters - fly drone from 3022 penn court to thelma g spencer park
        self.beacon_location = LLA_Point(42.628356390253714, -83.10454313558542, 400)
        self.drone_start_location = LLA_Point(42.63562993734775, -83.12015983135913, 400)

        self.drone = Drone(self.drone_start_location)
        self.beacon = Beacon(self.beacon_location)


    def step(self, dt):
        self.drone.update(dt)

    def generate_measurements(self, num_measurements):
        measurements = []
        for _ in range(num_measurements):
            self.drone.fly(drone.speed)
            signal_strength = beacon.get_signal_strength(drone.position)
            az, el = beacon.get_azel_from_positions(drone.position)
            measurements.append((drone.position, az, el, signal_strength))
            self.drone.factor_graph.add_measurement((drone.position, az, el, signal_strength))
        return measurements

if __name__ == "__main__":
    simulation = Simulation()
    drone = Drone(altitude=100, speed=10)
    beacon = Beacon(height=400)
    measurements = simulation.generate_measurements(drone, beacon, num_measurements=100)
    for position, signal in measurements:
        print(f"Position: {position}, Signal Strength: {signal}")
