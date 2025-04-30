import threading
import time

from flask import Flask, jsonify, Response, render_template_string
import math
import io
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from DJIControlClient import VelocityProfile
from routes import Velocity

app = Flask(__name__)

# Simulated state
state = {
    #"latitude": 57.763570, #gränsö
    #"longitude": 16.680859,
    "altitude": 1.0,
    "latitude": 59.46536932843077, #hagby
    "longitude": 18.02518065894246,
    "yaw": 0.0,  # heading in degrees
    "control_mode": "POSITION",
    "speed": 1.0,
    "angular_speed":30.0,
    "velocity_control_active": False,
    "vel": Velocity(0.0,0.0,0.0),
    "yawRate": 0.0,
    "velocity_profile": VelocityProfile.CONSTANT
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

@app.route('/takeoff', methods=['GET'])
def takeoff():
    return jsonify({"completed": True})

@app.route('/land', methods=['GET'])
def land():
    return jsonify({"completed": True})

@app.route('/confirmLanding', methods=['GET'])
def confirm_landing():
    return jsonify({"completed": True})

@app.route('/setMaxAngularSpeed/<speed>', methods=['GET'])
def set_max_angular_speed(speed):
    state["angular_speed"] = speed
    return jsonify({"completed": True})

@app.route('/startVelocityControl', methods=['GET'])
def start_velocity_control():
    state["velocity_control_active"] = True
    max_duration = 20.0
    threading.Thread(target=velocity_control_loop, args=(max_duration,)).start()
    return jsonify({"completed": True})


@app.route('/stopVelocityControl', methods=['GET'])
def stop_velocity_control():
    state["velocity_control_active"] = False
    return jsonify({"completed": True})


@app.route('/setVelocityProfile/<profilename>', methods=['GET'])
def set_velocity_profile(profilename):
    if profilename == 'CONSTANT':
        state["velocity_profile"] = VelocityProfile.CONSTANT
    elif profilename == 'TRAPEZOIDAL':
        state["velocity_profile"] = VelocityProfile.TRAPEZOIDAL
    elif profilename == 'S_CURVE':
        state["velocity_profile"] = VelocityProfile.S_CURVE

    return jsonify({"completed": True})


@app.route('/setVelocityCommand/<xVel>/<yVel>/<zVel>/<yawRate>', methods=['GET'])
def set_velocity_command(xVel, yVel, zVel, yawRate):
    # Velocity vector is expressed in NED system
    state["vel"] =  Velocity(x=float(xVel), y=float(yVel), z=float(zVel))
    print(state["vel"])
    state["yawRate"] = float(yawRate)

    return jsonify({"completed": True})

@app.route('/getHeading', methods=['GET'])
def get_heading():
    return jsonify({"yaw": state["yaw"]})

@app.route('/getAltitude', methods=['GET'])
def get_altitude():
    return jsonify({"altitude": state["altitude"]})

@app.route('/getCurrentPos', methods=['GET'])
def get_current_pos():
    #print(f'state latitude: {state["latitude"]}, longitude: {state["longitude"]}, alt: {state["altitude"]}, yaw: {state["yaw"]}')
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
def move(dx, dy, dz):
    lat = state["latitude"]
    deg_lat, deg_lon = meters_to_degrees(lat, dx, dy)
    state["latitude"] += deg_lat
    state["longitude"] += deg_lon
    state["altitude"] += dz


def move_return(dx=0.0, dy=0.0, dz=0.0):
    try:
        move(dx, dy, dz)
        position_history.append((state["latitude"], state["longitude"], state["altitude"]))
        return jsonify({"completed": True})
    except Exception as e:
        return jsonify({"completed": False, "errorDescription": str(e)})


def velocity_control_loop(max_duration=20.0):
    start_time = time.time()
    update_interval = 1.0  # seconds

    i = 0

    while time.time() - start_time < max_duration and state["velocity_control_active"]:
        dy = state["vel"].x * update_interval
        dx = state["vel"].y * update_interval
        dz = -state["vel"].z * update_interval

        i += 1
        print(f"control loop: {i}, dx: {dx}, dy: {dy}, dz: {dz}")
        # (dx, dy, dz) is the movement in m along velocity vector during time interval update_interval

        # Reuse movement logic
        move(dx, dy, dz)
        time.sleep(update_interval)

    # Stop velocities after finishing
    state["velocity_control_active"] = False
    state["vel"] = Velocity(0.0, 0.0, 0.0)
    print(f"control loop: {i}")

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
    return move_return(dx=dx, dy=dy)

@app.route('/moveBackward/<float:dist>', methods=['GET'])
def move_backward(dist):
    yaw_rad = math.radians(state["yaw"])
    dy = -dist * math.cos(yaw_rad)
    dx = -dist * math.sin(yaw_rad)
    return move_return(dx=dx, dy=dy)

@app.route('/moveLeft/<float:dist>', methods=['GET'])
def move_left(dist):
    yaw_rad = math.radians(state["yaw"] - 90)
    dy = dist * math.cos(yaw_rad)
    dx = dist * math.sin(yaw_rad)
    return move_return(dx=dx, dy=dy)

@app.route('/moveRight/<float:dist>', methods=['GET'])
def move_right(dist):
    yaw_rad = math.radians(state["yaw"] + 90)
    dy = dist * math.cos(yaw_rad)
    dx = dist * math.sin(yaw_rad)
    return move_return(dx=dx, dy=dy)

@app.route('/moveUp/<float:dist>', methods=['GET'])
def move_up(dist):
    return move_return(dz=dist)

@app.route('/moveDown/<float:dist>', methods=['GET'])
def move_down(dist):
    return move_return(dz=-dist)

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
