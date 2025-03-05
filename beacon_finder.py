import gtsam
import random
import math
import multiprocessing as mp
import numpy as np

""" Project to generate ESM measurements captured from a drone flying over the ocean and gathering hits off a 400m tall beacon. """

class llapoint:
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class ecefpoint:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def to_llapoint(self):
        """ Convert ecef to lla. """
        a = 6378137.0
        f = 1 / 298.257223563
        


class State:
    def __init__(self, x, y, z, vx, vy, vz):
        """ Initialize the state of the drone with position in ecef and velocity in m/s."""
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz

    def __init__(self, lat, lon, alt, vx, vy, vz):
        """ Initialize the state of the drone with position in lla and velocity in m/s. """
        self.lat = lat
        self.lon = lon
        self.alt = alt

        # Convert lla to ecef
        self.x, self.y, self.z = lla_to_ecef(lat, lon, alt)

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

    def add_pose(self, robot_pose):
        """ Add a pose to the factor graph. """
        prior = gtsam.PriorFactorPose3(1, gtsam.Pose3(robot_pose), self.radar_measurement_prior_noise)
        self.graph.add(prior)

    def add_measurement(self, measurement):
        """ Add a radar measurement to the factor graph. """
        radar_measurement = gtsam.RadarFactor(1, 2, 3, measurement[0], measurement[1], measurement[2], measurement[3], self.radar_measurement_prior_noise)
        self.graph.add(radar_measurement)


    


class Drone:
    def __init__(self, initial_state):
        self.state = initial_state
        self.drone_takeoff_velocity = 0.5  # m/s
        self.drone_flight_velocity = 20 # m/s

    def plan_path(self, start_location, end_location):
        # Plan a path from the current position to the beacon - straight line path for now
        

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
        self.beacon_location = llapoint(42.628356390253714, -83.10454313558542, 400)
        self.drone_start_location = llapoint(42.63562993734775, -83.12015983135913, 400)

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
        return measurements

if __name__ == "__main__":
    simulation = Simulation()
    drone = Drone(altitude=100, speed=10)
    beacon = Beacon(height=400)
    measurements = simulation.generate_measurements(drone, beacon, num_measurements=100)
    for position, signal in measurements:
        print(f"Position: {position}, Signal Strength: {signal}")
