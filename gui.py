import math
import sys
import threading

import folium
import os
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QHBoxLayout, QPushButton, QMessageBox
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QTimer, QUrl
from drone_route import DroneRoute  # Assumes this is your class

from PyQt5.QtWebChannel import QWebChannel


class DroneRouteMapViewer(QMainWindow):
    def __init__(self, drone_route: DroneRoute):
        super().__init__()
        self.execution_thread = None
        self.drone_route = drone_route
        self.setWindowTitle("Drone Route Map Viewer")
        self.setGeometry(100, 100, 1000, 700)
        self.route_plan_mode = False
        self.planned_waypoints = []
        self.k = 2
        self.alt_min = 2.0
        self.alt_max = 15.0
        self.n = 8

        self.web_view = QWebEngineView()
        # Buttons
        self.execute_button = QPushButton("Execute Route")
        self.pause_button = QPushButton("Pause")
        self.abort_button = QPushButton("Abort")
        self.plan_button = QPushButton("Enter Route Plan Mode")
        self.save_plan_button = QPushButton("Save Planned Route")


        # Button states
        self.pause_button.setEnabled(False)
        self.abort_button.setEnabled(False)
        self.plan_button.setCheckable(True)

        # Layout
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.execute_button)
        button_layout.addWidget(self.pause_button)
        button_layout.addWidget(self.abort_button)
        button_layout.addWidget(self.plan_button)
        button_layout.addWidget(self.save_plan_button)

        layout = QVBoxLayout()
        layout.addLayout(button_layout)
        layout.addWidget(self.web_view)
        self.setLayout(layout)


        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.map_file = "drone_map.html"
        #self.update_map()

        self.execute_button.clicked.connect(self.start_execution)
        self.pause_button.clicked.connect(self.toggle_pause)
        self.abort_button.clicked.connect(self.abort_execution)
        self.plan_button.clicked.connect(self.toggle_plan_mode)
        self.save_plan_button.clicked.connect(self.save_planned_route)

        # Refresh the map every second to reflect actionPoses dynamically
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_map)
        self.timer.start(1000)

        #self.start_route_thread()  # 🔥 new: start route execution in parallel


        # self.web_view.page().runJavaScript("""
        #     if (!window.clickedCoords) {
        #         window.clickedCoords = null;
        #         map.on('click', function(e) {
        #             window.clickedCoords = [e.latlng.lat, e.latlng.lng];
        #         });
        #     }
        # """)
        self.web_view.page().runJavaScript("""
            map.on('click', function(e) {
                L.circle([e.latlng.lat, e.latlng.lng], {
                    radius: 1.0,
                    color: 'red'
                }).addTo(map);
            });
        """)


        # QTimer.singleShot(1000, self.start_click_timer)


    def start_click_timer(self):
        self.click_timer = QTimer()
        self.click_timer.timeout.connect(self.check_map_click)
        self.click_timer.start(500)


    def check_map_click(self):
        if not self.route_plan_mode:
            return

        self.web_view.page().runJavaScript("window.clickedCoords", self.handle_map_click)

    def add_planned_waypoints(self, lat, lon):
        for i in range(self.k):
            alt = self.alt_min + i * (self.alt_max - self.alt_min) / max(self.k - 1, 1)
            for j in range(self.n):
                yaw_deg = j * 360 / self.n
                pose = {
                    "pos": {"latitude": lat, "longitude": lon, "altitude": alt},
                    "rot": {"roll": 0.0, "pitch": 0.0, "yaw": yaw_deg},
                    "time_arrive": 0.0,
                    "time_depart": 0.0
                }
                self.planned_waypoints.append(pose)



    def handle_map_click(self, coords):

        print("Clicked coords from JS:", coords)
        if coords is None:
            return

        lat, lon = coords
        self.add_planned_waypoints(lat, lon)
        self.update_map()
        self.web_view.page().runJavaScript("window.clickedCoords = null;")


    def start_execution(self):
        self.drone_route.pause = False
        self.drone_route.abort = False
        self.execution_thread = threading.Thread(target=self.drone_route.execute_route)
        self.execution_thread.start()

        self.execute_button.setEnabled(False)
        self.pause_button.setEnabled(True)
        self.abort_button.setEnabled(True)

    def toggle_pause(self):
        self.drone_route.pause = not self.drone_route.pause
        if self.drone_route.pause:
            self.pause_button.setText("Resume")
        else:
            self.pause_button.setText("Pause")

    def abort_execution(self):
        self.drone_route.abort = True
        self.pause_button.setEnabled(False)
        self.abort_button.setEnabled(False)
        self.execute_button.setEnabled(True)
        self.pause_button.setText("Pause")

    def start_route_thread(self):
        # Start the drone route execution in its own thread
        route_thread = threading.Thread(
            target=self.drone_route.execute_route,
            daemon=True  # dies with the main app
        )
        route_thread.start()

    def toggle_plan_mode(self):
        self.route_plan_mode = self.plan_button.isChecked()
        self.plan_button.setText("Exit Route Plan Mode" if self.route_plan_mode else "Enter Route Plan Mode")

    def save_planned_route(self):
        import os, json
        if not self.planned_waypoints:
            QMessageBox.information(self, "Save Planned Route", "No waypoints to save.")
            return

        filename = os.path.splitext(self.drone_route.route_filename)[0] + "_planned.json"
        data = {
            "name": self.drone_route.selected_drone.get("name", "planned_drone"),
            "speed": self.drone_route.selected_drone.get("speed", 10),
            "turn_speed": self.drone_route.selected_drone.get("turn_speed", 1.57),
            "flight_time": self.drone_route.selected_drone.get("flight_time", 600),
            "poses": self.planned_waypoints
        }

        with open(filename, "w") as f:
            json.dump(data, f, indent=2)

        QMessageBox.information(self, "Route Saved", f"Saved to {filename}")

    def update_map(self):
        # Use origin if available, else first waypoint
        if self.drone_route.origin:
            lat = self.drone_route.origin["latitude"]
            lon = self.drone_route.origin["longitude"]
        else:
            first_wp = self.drone_route.selected_route[0]
            lat = first_wp["pos"]["latitude"]
            lon = first_wp["pos"]["longitude"]

        fmap = folium.Map(
            location=[lat, lon],
            zoom_start=18,
            tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
            attr='Google Satellite',
            control_scale=True
        )

        # Plot blue waypoints
        for wp in self.drone_route.selected_route:
            pos = wp["pos"]
            folium.CircleMarker(
                location=[pos["latitude"], pos["longitude"]],
                radius=4,
                color='blue',
                fill=True,
                fill_opacity=0.7
            ).add_to(fmap)

        # Plot green visited action poses
        action_coords = [(pose.y, pose.x) for pose in self.drone_route.actionPoses]
        for lat, lon in action_coords:
            folium.CircleMarker(
                location=[lat, lon],
                radius=4,
                color='green',
                fill=True,
                fill_opacity=0.9
            ).add_to(fmap)

        if len(action_coords) >= 2:
            folium.PolyLine(action_coords, color="green", weight=2.5).add_to(fmap)

        # Add currentPose as red heading arrow
        if self.drone_route.currentPose:
            cp = self.drone_route.currentPose
            arrow_icon = folium.features.CustomIcon(
                icon_image="assets/Red_Arrow_Down.svg.png", #icon_image="https://upload.wikimedia.org/wikipedia/commons/thumb/0/04/Red_Arrow_Down.svg/120px-Red_Arrow_Down.svg.png",
                icon_size=(20, 20),
                icon_anchor=(15, 15)
            )
            folium.Marker(
                location=[cp.y, cp.x],
                icon=arrow_icon,
                popup=f"Yaw: {round(math.degrees(cp.yaw), 1)}°"
            ).add_to(fmap)

        for wp in self.planned_waypoints:
            lat = wp["pos"]["latitude"]
            lon = wp["pos"]["longitude"]
            js_commands.append(f"""
                L.circle([{lat}, {lon}], {{
                    color: 'orange',
                    radius: 0.3
                }}).addTo(map);
            """)

        # Save and reload
        fmap.save(self.map_file)
        self.web_view.load(QUrl.fromLocalFile(os.path.abspath(self.map_file)))


if __name__ == "__main__":
    # Replace with real initialization

    testRoute = f"test_route_wgs2.json"

    drone = DroneRoute("127.0.0.1", 5000, testRoute)
    app = QApplication(sys.argv)
    viewer = DroneRouteMapViewer(drone)
    viewer.show()
    sys.exit(app.exec_())