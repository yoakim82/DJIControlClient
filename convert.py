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

    output_path = input_path.with_name(input_path.stem + "_wgs2.json")
    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"Converted file saved to {output_path}")

if __name__ == "__main__":
    # if len(sys.argv) != 5:
    #     print("Usage: python convert.py <input_file.json> <origin_lat> <origin_lon> <origin_alt>")
    #     sys.exit(1)
    file = "test_route.json"
    lat = 57.76528976764431
    lon = 16.676588299360205
    alt = 1.0
    convert_file(input_path=file, origin_lat=lat, origin_lon=lon, origin_alt=alt)