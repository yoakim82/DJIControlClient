import csv
import json
import math
import sys
from pathlib import Path

# Constants
METERS_PER_DEG_LAT = 111_320

def meters_to_wgs84(origin_lat, origin_lon, origin_alt, x, y, z):
    """
    Converts local ENU x, y, z to lat, lon, alt.
    x = East, y = North, z = Up
    """
    lat_rad = math.radians(origin_lat)
    delta_lat = y / METERS_PER_DEG_LAT
    delta_lon = x / (METERS_PER_DEG_LAT * math.cos(lat_rad))

    new_lat = origin_lat + delta_lat
    new_lon = origin_lon + delta_lon
    new_alt = origin_alt + z

    return new_lon, new_lat, new_alt

def convert_file(input_path, origin_lat, origin_lon, origin_alt):
    input_path = Path(input_path)
    with open(input_path, "r") as f:
        data = json.load(f)

    for route in data:
        for pose in route["poses"]:
            x = pose["pos"]["x"]
            y = pose["pos"]["y"]
            z = pose["pos"]["z"]

            lon, lat, alt = meters_to_wgs84(origin_lat, origin_lon, origin_alt, x, y, z)

            pose["pos"] = {
                "longitude": lon,
                "latitude": lat,
                "altitude": alt
            }

    output_path = input_path.with_name(input_path.stem + "_wgs_hagby.json")
    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"Converted file saved to {output_path}")


def convert_csv_to_route_json(csv_path, drone_name="static.prop.drone_mavic_air"):
    poses = []
    with open(csv_path, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for i, row in enumerate(reader):
            lat = float(row["Latitude"])
            lon = float(row["Longitude"])
            yaw = float(row["Yaw"])
            pitch = float(row.get("Pitch", 0))  # Default pitch to 0 if missing
            alt = float(row.get("Height", 10.0))

            pose = {
                "time_arrive": 0.0,
                "time_depart": 0.0,
                "pos": {
                    "longitude": lon,
                    "latitude": lat,
                    "altitude": alt
                },
                "rot": {
                    "roll": 0.0,
                    "pitch": pitch,
                    "yaw": yaw
                }
            }
            poses.append(pose)

    route_json = [{
        "name": drone_name,
        "speed": 10,
        "turn_speed": 1.5707963,
        "flight_time": 600,
        "poses": poses
    }]

    output_path = Path(csv_path).with_suffix(".json")
    with open(output_path, "w") as f:
        json.dump(route_json, f, indent=2)

    return output_path


def conv_to_wgs84():
    # convert from m to wgs84
    file = "test_route.json"
    # granso_origin
    # lat = 57.76528976764431
    # lon = 16.676588299360205
    (lat, lon) = (59.46731557248635, 18.019252923720437)
    alt = 1.0
    convert_file(input_path=file, origin_lat=lat, origin_lon=lon, origin_alt=alt)


if __name__ == "__main__":

    #conv_to_wgs84()

    convert_csv_to_route_json(csv_path="route.csv")

