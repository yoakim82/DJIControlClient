import time

from DJIControlClient import *
from routes import *


class DroneRoute(DJIControlClient):

    def __init__(self, ip, port, route_filename, selected_route=0):
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

        self.setMaxSpeed(4.0)

        #self.client = DJIControlClient(ip=self.ip, port=self.port)


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

        self.origin = Pose(x=pos['longitude'], y=pos['latitude'], z=pos['altitude'], yaw=pos['yaw'], pitch=0.0)

    def gotoWP(self, pose, num):

        self.activeWP = pose
        pos = self.getPosition()
        self.currentPose = Pose(x=pos['longitude'], y=pos['latitude'], z=pos['altitude'], yaw=pos['yaw'], pitch=0.0)
        #print(f'current pose latitude: {self.currentPose.y}, longitude: {self.currentPose.x}, alt: {self.currentPose.z}, yaw: {self.currentPose.yaw}')
        #print(f'active  pose latitude: {self.activeWP.y}, longitude: {self.activeWP.x}, alt: {self.activeWP.z}, yaw: {self.activeWP.yaw}')

        dist_lon = self.activeWP.x - self.currentPose.x
        dist_lat = self.activeWP.y - self.currentPose.y
        xy_dist_meters = degrees_to_meters(delta_lat=dist_lat, delta_lon=dist_lon, at_latitude=self.currentPose.y)
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


    def do_action(self, pose, wpName):

        pos = self.getPosition()
        self.currentPose = Pose(x=pos['longitude'], y=pos['latitude'], z=pos['altitude'], yaw=pos['yaw'], pitch=pose.pitch)
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

        for i, wp in enumerate(self.selected_route):
            if self.abort:
                print("Aborted route execution.")
                break
            while self.pause:
                time.sleep(0.1)

            pt = Pose(x=wp['pos']['longitude'], y=wp['pos']['latitude'], z=wp['pos']['altitude'], yaw=wp['rot']['yaw'],
                      pitch=wp['rot']['pitch'])
            self.gotoWP(pt, num=i)
            # print(f"Waypoint {i} reached.")
            self.do_action(pt, i)
            time.sleep(0.1)

        # for i, wp in enumerate(self.selected_route):
        #     pt = Pose(x=wp['pos']['longitude'], y=wp['pos']['latitude'], z=wp['pos']['altitude'], yaw=wp['rot']['yaw'], pitch=wp['rot']['pitch'])
        #     self.gotoWP(pt, num=i)
        #     #print(f"Waypoint {i} reached.")
        #     self.do_action(pt, i)
        #     time.sleep(0.1)

        dump_poses_to_csv(self.actionPoses, "actionposes.csv")

        # self.land()
        # time.sleep(1.0)
        # self.confirmLanding()


def main():

    #client = DJIControlClient(ip="192.168.100.167", port=8080)


    #client.pitchGimbal(angle=45.0)

    #ret = client.captureShot()

    #print(ret

    testRoute = f"test_route_wgs2.json"

    #rte = DroneRoute(ip="217.214.18.119", port=8080, route=None)
    rte = DroneRoute(ip="127.0.0.1", port=5000, route_filename=testRoute)
    rte.checkCommand("gimbal pitch", rte.pitchGimbal(angle=05.0))

    rte.capturePhoto()

    #rte.startVideoRecording()
    #time.sleep(0.5)
    #rte.stopVideoRecording()

    pos = rte.getPosition()
    print(pos)

    #rte.checkCommand("check getLandingProtectionState()", rte.getLandingProtectionState())
    #time.sleep(1.0)

    rte.checkCommand("Set Control Mode", rte.setControlMode(mode=ControlMode.POSITION))
    #time.sleep(3.0)

    #rte.checkCommand("takeOff", rte.takeOff())
    #time.sleep(5.0)

    rte.checkCommand("Move Up", rte.moveUp(distance=1.3))
    #time.sleep(2.0)

    imgs = []

    for i in range(0,4):
        rte.rotateClockwise(angle=90.0)
        #time.sleep(1.0)
        imgs.append(rte.capturePhoto())


    rte.checkCommand("Move down", rte.moveDown(distance=1.3))
    #time.sleep(2.0)


    #rte.checkCommand("Land", rte.land())
    #time.sleep(1.0)
    #rte.checkCommand("confirmLanding", rte.confirmLanding())

    #rte.checkCommand("Set Control Mode", rte.setControlMode(mode=ControlMode.POSITION))
    #time.sleep(1.0)

    #rte.checkCommand("Move Up", rte.moveUp(distance=0.1))
    #time.sleep(1.0)

    #rte.checkCommand("getAltitude", rte.getAltitude())

    rte.setOrigin()
    time.sleep(1.0)

    rte.execute_route()


    #captureShot

if __name__ == "__main__":
    main()