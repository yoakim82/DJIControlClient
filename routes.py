import csv
import json

from typing import Optional
from dataclasses import dataclass

import numpy as np

import math

def dump_poses_to_csv(poses, output_path: str):
    with open(output_path, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header
        writer.writerow(['x', 'y', 'z', 'yaw', 'pitch'])
        # Write pose values
        for pose in poses:
            writer.writerow([pose.x, pose.y, pose.z, pose.yaw, pose.pitch])
    print(f"Wrote {len(poses)} poses to {output_path}")



@dataclass
class Drone:
    name: str
    speed: float  # m/s
    turn_speed: float  # rad/s
    flight_time: int  # seconds


@dataclass
class Pose:
    x: float  # m
    y: float  # m
    z: float  # m
    yaw: float  # rad
    pitch: float  # rad
    time: str

    def __eq__(self, o: object) -> bool:
        if not isinstance(o, Pose):
            return False
        return (o.x == self.x
                and o.y == self.y
                and o.z == self.z
                and o.yaw == self.yaw
                and o.pitch == self.pitch)


@dataclass
class ActionPoint:
    id: int
    pose: Pose
    time: tuple[float, float] # s
    transit_time: float # s

@dataclass
class Velocity:
    x: float  # m/s
    y: float  # m/s
    z: float  # m/s

    def normalize(self):
        length = math.sqrt(self.x**2 + self.y**2 + self.z**2)
        self.x = self.x / length
        self.y = self.y / length
        self.z = self.z / length

    def setLength(self, length):
        self.normalize()
        self.x = self.x * length
        self.y = self.y * length
        self.z = self.z * length

    def length(self):

        return math.sqrt(self.x**2 + self.y**2 + self.z**2)


def calculate_bearing(pos1, pos2):
    lon1 = pos1.x
    lat1 = pos1.y
    lon2 = pos2.x
    lat2 = pos2.y
    # Convert from degrees to radians
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon_rad = math.radians(lon2 - lon1)

    x = math.sin(delta_lon_rad) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon_rad)

    initial_bearing_rad = math.atan2(x, y)

    # Convert bearing from radians to degrees and normalize to 0–360°
    initial_bearing_deg = (math.degrees(initial_bearing_rad) + 360) % 360

    return initial_bearing_deg


def shortest_rotation(h, b):
    # Normalize the difference to the range [-180, 180)
    rotation = (b - h + 180) % 360 - 180
    return rotation


def degrees_to_meters(delta_lat, delta_lon, at_latitude):
    # Convert latitude to radians
    lat_rad = math.radians(at_latitude)

    # Approximate conversions
    meters_per_deg_lat = 111_320  # meters
    meters_per_deg_lon = 111_320 * math.cos(lat_rad)

    # Convert differences to meters
    dy = delta_lat * meters_per_deg_lat
    dx = delta_lon * meters_per_deg_lon

    # Use Pythagorean theorem for total distance
    distance = math.sqrt(dx**2 + dy**2)
    return dx, dy


def degrees_to_meters_2d(delta_lat, delta_lon, at_latitude):
    # Convert latitude to radians
    lat_rad = math.radians(at_latitude)

    # Approximate conversions
    meters_per_deg_lat = 111_320  # meters
    meters_per_deg_lon = 111_320 * math.cos(lat_rad)

    # Convert differences to meters
    dy = delta_lat * meters_per_deg_lat
    dx = delta_lon * meters_per_deg_lon

    # Use Pythagorean theorem for total distance
    distance = math.sqrt(dx**2 + dy**2)
    return distance

def load_json_file(file_path):
    with open(file_path, 'r') as f:
        return json.load(f)


def generate_routes(route_plan_file, slow=False):

    retval = []
    routes_json = load_json_file(route_plan_file)
    routes = []
    drones = []
    for rte in routes_json:

        drone_name = rte["name"]
        drone_speed = rte["speed"]/100.0
        turn_speed = rte["turn_speed"]
        flight_time = rte["flight_time"]
        poses = rte["poses"]
        drones.append((drone_name, drone_speed, turn_speed, flight_time))
        routes.append(poses)

    #print(drones)
    #print(routes)
    return drones, routes

