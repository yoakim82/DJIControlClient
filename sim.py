import time

from flask import Flask, jsonify, Response, render_template_string
import math
import io
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

app = Flask(__name__)

# Simulated state
state = {
    "latitude": 57.763570,
    "longitude": 16.680859,
    "altitude": 1.0,
    "yaw": 0.0,  # heading in degrees
    "control_mode": "POSITION",
    "speed": 1.0
}

# Constants for converting degrees to meters
METERS_PER_DEG_LAT = 111_320

# Store position history for visualization
position_history = []
position_history.append((state["latitude"], state["longitude"], state["altitude"]))

def meters_to_degrees(lat, dx, dy):
    lat_rad = math.radians(lat)
    deg_lat = dy / METERS_PER_DEG_LAT
    deg_lon = dx / (METERS_PER_DEG_LAT * math.cos(lat_rad))
    return deg_lat, deg_lon

@app.route('/', methods=['GET'])
def root():
    return b"Connected"

@app.route('/setControlMode/<mode>', methods=['GET'])
def set_control_mode(mode):
    if mode.upper() not in ["POSITION", "VELOCITY"]:
        return jsonify({"completed": False, "errorDescription": "Invalid mode"})
    state["control_mode"] = mode.upper()
    return jsonify({"completed": True})


@app.route('/setMaxSpeed/<speed>', methods=['GET'])
def set_max_speed(speed):
    state["speed"] = speed
    return jsonify({"completed": True})


@app.route('/getHeading', methods=['GET'])
def get_heading():
    return jsonify({"yaw": state["yaw"]})

@app.route('/getAltitude', methods=['GET'])
def get_altitude():
    return jsonify({"altitude": state["altitude"]})

@app.route('/getCurrentPos', methods=['GET'])
def get_current_pos():
    print(f'state latitude: {state["latitude"]}, longitude: {state["longitude"]}, alt: {state["altitude"]}, yaw: {state["yaw"]}')
    return jsonify({
        "latitude": state["latitude"],
        "longitude": state["longitude"],
        "altitude": state["altitude"],
        "yaw": state["yaw"]
    })

@app.route('/setCurrentPos/<lat>/<lon>/<alt>/<yaw>', methods=['GET'])
def set_current_pos(lat, lon, alt, yaw):
    try:
        state["latitude"] = float(lat)
        state["longitude"] = float(lon)
        state["altitude"] = float(alt)
        state["yaw"] = float(yaw)
        position_history.append((state["latitude"], state["longitude"], state["altitude"]))
        return jsonify({"completed": True})
    except Exception as e:
        return jsonify({"completed": False, "errorDescription": str(e)})

@app.route('/capturePhoto', methods=['GET'])
def capture_photo():
    return jsonify({"completed": True})  # Simulate photo capture

@app.route('/pitchGimbal/<angle>', methods=['GET'])
def pitch_gimbal(angle):
    return jsonify({"completed": True})  # Simulate gimbal movement


# --- Movement helpers ---
def move(dx=0.0, dy=0.0, dz=0.0):
    try:
        time_to_traverse = math.sqrt(dx**2 + dy**2 + dz**2) / state["speed"]
        lat = state["latitude"]
        deg_lat, deg_lon = meters_to_degrees(lat, dx, dy)
        # time.sleep(time_to_traverse)
        state["latitude"] += deg_lat
        state["longitude"] += deg_lon
        state["altitude"] += dz
        position_history.append((state["latitude"], state["longitude"], state["altitude"]))
        return jsonify({"completed": True})
    except Exception as e:
        return jsonify({"completed": False, "errorDescription": str(e)})

# def move(dx=0.0, dy=0.0, dz=0.0):
#     try:
#         total_distance = math.sqrt(dx**2 + dy**2 + dz**2)
#         time_to_traverse = total_distance / state["speed"]
#
#         # Number of updates (every 0.2 seconds)
#         update_interval = 0.2  # 200 ms
#         steps = max(1, int(time_to_traverse / update_interval))
#
#         # Step size
#         step_dx = dx / steps
#         step_dy = dy / steps
#         step_dz = dz / steps
#
#         print(f"steps: {steps}, dx: {dx}, dy: {dy}, dz: {dz}")
#         print(f"steps: {steps}, step_dx: {step_dx}, step_dy: {step_dy}, step_dz: {step_dz}")
#         for _ in range(steps):
#             # Convert step distance to degrees
#             lat = state["latitude"]
#             deg_lat_step, deg_lon_step = meters_to_degrees(lat, step_dx, step_dy)
#
#             state["latitude"] += deg_lat_step
#             state["longitude"] += deg_lon_step
#             state["altitude"] += step_dz
#
#             time.sleep(update_interval)
#
#         position_history.append((state["latitude"], state["longitude"], state["altitude"]))
#         return jsonify({"completed": True})
#     except Exception as e:
#         return jsonify({"completed": False, "errorDescription": str(e)})


@app.route('/moveForward/<float:dist>', methods=['GET'])
def move_forward(dist):
    yaw_rad = math.radians(state["yaw"])
    dy = dist * math.cos(yaw_rad)
    dx = dist * math.sin(yaw_rad)
    return move(dx=dx, dy=dy)

@app.route('/moveBackward/<float:dist>', methods=['GET'])
def move_backward(dist):
    yaw_rad = math.radians(state["yaw"])
    dy = -dist * math.cos(yaw_rad)
    dx = -dist * math.sin(yaw_rad)
    return move(dx=dx, dy=dy)

@app.route('/moveLeft/<float:dist>', methods=['GET'])
def move_left(dist):
    yaw_rad = math.radians(state["yaw"] - 90)
    dy = dist * math.cos(yaw_rad)
    dx = dist * math.sin(yaw_rad)
    return move(dx=dx, dy=dy)

@app.route('/moveRight/<float:dist>', methods=['GET'])
def move_right(dist):
    yaw_rad = math.radians(state["yaw"] + 90)
    dy = dist * math.cos(yaw_rad)
    dx = dist * math.sin(yaw_rad)
    return move(dx=dx, dy=dy)

@app.route('/moveUp/<float:dist>', methods=['GET'])
def move_up(dist):
    return move(dz=dist)

@app.route('/moveDown/<float:dist>', methods=['GET'])
def move_down(dist):
    return move(dz=-dist)

@app.route('/rotateClockwise/<float:angle>', methods=['GET'])
def rotate_clockwise(angle):
    state["yaw"] = (state["yaw"] + angle) % 360
    return jsonify({"completed": True})

@app.route('/rotateCounterClockwise/<float:angle>', methods=['GET'])
def rotate_counter_clockwise(angle):
    state["yaw"] = (state["yaw"] - angle) % 360
    return jsonify({"completed": True})

@app.route('/showState', methods=['GET'])
def show_state():
    html = '''
    <html>
    <head>
        <title>Position State Visualization</title>
        <meta http-equiv="refresh" content="1">
    </head>
    <body>
        <h1>Position State History</h1>
        <img src="/plot.png" alt="Position plot">
    </body>
    </html>
    '''
    return render_template_string(html)

@app.route('/plot.png')
def plot_png():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    lats, lons, alts = zip(*position_history)
    ax.plot(lons, lats, alts, marker='o')
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.set_zlabel("Altitude")
    ax.set_title("Position State History")

    buf = io.BytesIO()
    plt.tight_layout()
    plt.savefig(buf, format='png')
    buf.seek(0)
    plt.close(fig)
    return Response(buf.getvalue(), mimetype='image/png')

if __name__ == '__main__':
    app.run(debug=True, port=5000)  # Change port if needed
