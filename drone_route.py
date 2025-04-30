import time
from datetime import timedelta, datetime

from DJIControlClient import *
from routes import *


class DroneRoute(DJIControlClient):

    def __init__(self, ip, port, route_filename, selected_route=0, sim=False):
        super().__init__(ip=ip, port=port)
        #self.ip = ip
        #self.port = port
        drones, routes = generate_routes(route_filename)
        self.route_filename = route_filename
        self.routes = routes
        self.drones = drones
        self.selected_route = routes[selected_route]
        self.selected_drone = drones[selected_route]
        self.origin = None
        self.activeWP = None
        self.prevWP = None
        self.currentPose = None
        self.actionPoses = []
        self.pause = False
        self.abort = False
        self.gpsPoses = []

        self.hagby = (59.46536932843077, 18.02518065894246)
        self.gränsö = (57.763570, 16.680859)


        if sim:
            self.setCurrentPos(lat=self.hagby[0], lon=self.hagby[1], alt=1.5, yaw=0.0)

        #self.setMaxSpeed(4.0) # only in positional mode

        self.setControlMode(ControlMode.VELOCITY)
        self.setVelocityProfile(VelocityProfile.S_CURVE)


    def selectActiveRoute(self, activeRoute):
        self.selected_route = self.routes[activeRoute]
        self.selected_drone = self.drones[activeRoute]

    def checkCommand(self, command, res):
        time.sleep(0.2)

        if not res["completed"]:
            e = res["errorDescription"]
            print(f"Command {command}: {e}")
            #raise AssertionError(
            #    "stop program")
        else:
            print(f"Command {command} completed successfully")

        return res["completed"]


    def setOrigin(self):
        pos = self.getPosition()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        self.origin = Pose(x=pos['longitude'], y=pos['latitude'], z=pos['altitude'], yaw=pos['yaw'], pitch=0.0, time=timestamp)


    def gotoWP(self, pose, num):

        self.activeWP = pose
        pos = self.getPosition()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        self.currentPose = Pose(x=pos['longitude'], y=pos['latitude'], z=pos['altitude'], yaw=pos['yaw'], pitch=0.0, time=timestamp)
        #print(f'current pose latitude: {self.currentPose.y}, longitude: {self.currentPose.x}, alt: {self.currentPose.z}, yaw: {self.currentPose.yaw}')
        #print(f'active  pose latitude: {self.activeWP.y}, longitude: {self.activeWP.x}, alt: {self.activeWP.z}, yaw: {self.activeWP.yaw}')

        dist_lon = self.activeWP.x - self.currentPose.x
        dist_lat = self.activeWP.y - self.currentPose.y
        xy_dist_meters = degrees_to_meters_2d(delta_lat=dist_lat, delta_lon=dist_lon, at_latitude=self.currentPose.y)
        dist_z = self.activeWP.z - self.currentPose.z


        # do vertical movement first
        if dist_z > 0.1:
            self.moveUp(dist_z)
        elif dist_z < -0.1:
            self.moveDown(abs(dist_z))

        #find direction to WP
        if xy_dist_meters > 0.1:
            bearing_to_WP = calculate_bearing(self.currentPose, self.activeWP)
            r = shortest_rotation(self.currentPose.yaw, bearing_to_WP)
            # print(f'dist: lat{dist_lat}, lon: {dist_lon}, xy_meters: {xy_dist_meters}, dist_z: {dist_z}, bearing: {bearing_to_WP}, current heading: {self.currentPose.yaw}, rot: {r}')
            if r > 0.1:
                self.rotateClockwise(r)
            elif r < -0.1:
                self.rotateCounterClockwise(abs(r))

            # now, all we should need is to go fwd until we reach the next WP
            self.moveForward(distance=xy_dist_meters)


    def gotoWP_velmode(self, timedPose, num):

        self.activeWP = timedPose.pose
        (start_time, end_time) = timedPose.time


        pos = self.getPosition()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        self.currentPose = Pose(x=pos['longitude'], y=pos['latitude'], z=pos['altitude'], yaw=pos['yaw'], pitch=0.0, time=timestamp)
        print(f'current pose latitude: {self.currentPose.y}, longitude: {self.currentPose.x}, alt: {self.currentPose.z}, yaw: {self.currentPose.yaw}')
        #print(f'active  pose latitude: {self.activeWP.y}, longitude: {self.activeWP.x}, alt: {self.activeWP.z}, yaw: {self.activeWP.yaw}')

        self.setControlMode(ControlMode.VELOCITY)

        # VELOCITY mode operates in XYZ NED system (X = North, Y = East, Z = Down)

        # we work with coordinates ENU (x = East, y = North, z = Up)
        dist_lon = self.activeWP.x - self.currentPose.x
        dist_lat = self.activeWP.y - self.currentPose.y
        (x_dist_meters, y_dist_meters) = degrees_to_meters(delta_lat=dist_lat, delta_lon=dist_lon, at_latitude=self.currentPose.y)
        #y_dist_meters = degrees_to_meters(delta_lat=dist_lat, delta_lon=0.0, at_latitude=self.currentPose.y)

        dist_z = self.activeWP.z - self.currentPose.z

        print(f"dist x: {x_dist_meters}, y: {y_dist_meters}, z: {dist_z}")
        #t = timedPose.transit_time
        #if t < 1.0:
        t = 5.0
        # set velocity vector to go shortest path to next WP
        # vel is expressed in NED system!
        vel = Velocity(x=y_dist_meters/t, y=x_dist_meters/t, z=-dist_z/t)

        print(f"Velocity: {vel}, t: {t} s")

        if vel.length() > 0.2:
            self.startVelocityControl()
            self.setVelocityCommand(xVel=vel.x, yVel=vel.y, zVel=vel.z, yawRate=0.0)
            start_time = time.time()
            update_interval = 0.2  # seconds

            while time.time() - start_time < t:
                p = self.getPosition()
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                self.currentPose = Pose(x=p['longitude'], y=p['latitude'], z=p['altitude'], yaw=p['yaw'], pitch=0.0, time=timestamp)
                #print(f"time: {time.time()}, {self.currentPose}")
                time.sleep(update_interval)

            self.stopVelocityControl()
            time_wp = time.time() - start_time
            print(f"time spent: {time_wp}, {self.currentPose}")

        p = self.getPosition()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.currentPose = Pose(x=p['longitude'], y=p['latitude'], z=p['altitude'], yaw=p['yaw'], pitch=0.0, time=timestamp)
        dist_lon = self.activeWP.x - self.currentPose.x
        dist_lat = self.activeWP.y - self.currentPose.y
        (x_dist_meters, y_dist_meters) = degrees_to_meters(delta_lat=dist_lat, delta_lon=dist_lon, at_latitude=self.currentPose.y)
        dist_z = self.activeWP.z - self.currentPose.z
        print(f"Arrived at {self.currentPose} after {t} s, deviation: dx: {x_dist_meters:.2f}, dy: {y_dist_meters:.2f}, dz: {dist_z:.2f}")

        # time.sleep(1.0)
        # p = self.getPosition()
        # self.currentPose = Pose(x=p['longitude'], y=p['latitude'], z=p['altitude'], yaw=p['yaw'], pitch=0.0)
        # dist_lon = self.activeWP.x - self.currentPose.x
        # dist_lat = self.activeWP.y - self.currentPose.y
        # x_dist_meters = degrees_to_meters(delta_lat=0.0, delta_lon=dist_lon, at_latitude=self.currentPose.y)
        # y_dist_meters = degrees_to_meters(delta_lat=dist_lat, delta_lon=0.0, at_latitude=self.currentPose.y)
        # dist_z = self.activeWP.z - self.currentPose.z
        # print(
        #     f"Arrived at {self.currentPose} after {t} s, deviation: dx: {x_dist_meters:.2f}, dy: {y_dist_meters:.2f}, dz: {dist_z:.2f}")

    def do_action(self, timedPose, wpName):

        pose = timedPose.pose

        self.setControlMode(ControlMode.POSITION)
        pos = self.getPosition()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.currentPose = Pose(x=pos['longitude'], y=pos['latitude'], z=pos['altitude'], yaw=pos['yaw'], pitch=pose.pitch, time=timestamp)
        self.actionPoses.append(self.currentPose)
        r = shortest_rotation(self.currentPose.yaw, pose.yaw)
        if r > 0.1:
            self.rotateClockwise(r)
        elif r < -0.1:
            self.rotateCounterClockwise(abs(r))

        self.pitchGimbal(angle=pose.pitch)
        self.capturePhoto()
        print(f"Photo {wpName} captured at {self.currentPose}")

    def execute_route(self):

        self.start_flight()
        #self.check_pos_accuracy()
        last_wp_idx = 0
        prev_time_depart = 0.0
        for i, wp in enumerate(self.selected_route):

            if self.abort:
                print("Aborted route execution.")
                break
            while self.pause:

                time.sleep(0.1)
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            pt = Pose(x=wp['pos']['longitude'], y=wp['pos']['latitude'], z=wp['pos']['altitude']+self.origin.z, yaw=wp['rot']['yaw'],
                      pitch=wp['rot']['pitch'], time=timestamp)

            transit_time = float(wp["time_arrive"]) - prev_time_depart
            timedPt = ActionPoint(id=i, pose=pt, time=tuple([wp["time_arrive"], wp["time_depart"]]), transit_time=transit_time)
            self.gotoWP_velmode(timedPt, num=i)

            # this keeps track of previous WP's departure time, which allows us to calc the transit time b/w WPs
            prev_time_depart = timedPt.time[1]

            #self.gotoWP(pt, num=i)
            # print(f"Waypoint {i} reached.")
            self.do_action(timedPt, i)
            time.sleep(1)
            last_wp_idx = i

        # for i, wp in enumerate(self.selected_route):
        #     pt = Pose(x=wp['pos']['longitude'], y=wp['pos']['latitude'], z=wp['pos']['altitude'], yaw=wp['rot']['yaw'], pitch=wp['rot']['pitch'])
        #     self.gotoWP(pt, num=i)
        #     #print(f"Waypoint {i} reached.")
        #     self.do_action(pt, i)
        #     time.sleep(0.1)

        dump_poses_to_csv(self.actionPoses, "actionposes.csv")

        # return to home
        self.return_home(id=100)
        time.sleep(1.0)

        self.land()
        time.sleep(1.0)
        self.confirmLanding()

    def start_flight(self):

        self.takeOff()
        time.sleep(5.0)
        self.moveUp(distance=1.0)
        self.setOrigin()
        print(f"Setting Origin at {self.origin}")
        time.sleep(1.0)

    def check_pos_accuracy(self):
        self.setControlMode(ControlMode.POSITION)
        for i in range(20):
            pos = self.getPosition()
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.currentPose = Pose(x=pos['longitude'], y=pos['latitude'], z=pos['altitude'], yaw=pos['yaw'], pitch=0.0, time=timestamp)
            self.gpsPoses.append(self.currentPose)
            time.sleep(6.0)

        dump_poses_to_csv(self.gpsPoses, "gpsposes.csv")

    def return_home(self, id):
        timedPt = ActionPoint(id=id, pose=self.origin, time=tuple([0.0, 1.0]), transit_time=5.0)
        self.gotoWP_velmode(timedPt, num=id)


def main():

    #client = DJIControlClient(ip="192.168.100.167", port=8080)


    #client.pitchGimbal(angle=45.0)

    #ret = client.captureShot()

    #print(ret

    testRoute = f"test_route_simple_hagby.json"

    rte = DroneRoute(ip="192.168.133.25", port=8080, route_filename=testRoute)

    #rte = DroneRoute(ip="217.214.18.119", port=8080, route=None)
    #rte = DroneRoute(ip="127.0.0.1", port=5000, route_filename=testRoute)
    rte.checkCommand("gimbal pitch", rte.pitchGimbal(angle=05.0))

    #rte.capturePhoto()

    #rte.startVideoRecording()
    #time.sleep(0.5)
    #rte.stopVideoRecording()

    #pos = rte.getPosition()
    #print(pos)

    #rte.checkCommand("check getLandingProtectionState()", rte.getLandingProtectionState())
    #time.sleep(1.0)

    rte.checkCommand("Set Control Mode", rte.setControlMode(mode=ControlMode.POSITION))
    #time.sleep(3.0)

    rte.start_flight()
    time.sleep(1.0)

    rte.check_pos_accuracy()
    #time.sleep(1.0)

    rte.moveForward(2.5)
    print("Move Fwd Done.")
    #rte.execute_route()

    rte.return_home(id=100)
    print("Return home done.")
    time.sleep(1.0)

    rte.land()
    print("init landing.")
    time.sleep(1.0)
    rte.confirmLanding()
    print("landing confirmed.")
    #captureShot

if __name__ == "__main__":
    main()