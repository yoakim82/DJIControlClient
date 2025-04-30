import sys
import os
from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QSplitter, QListWidget, QWidget
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage, QWebEngineProfile
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtCore import pyqtSlot, QObject, pyqtSignal
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWebEngineWidgets import QWebEngineProfile


class Bridge(QObject):
    # Define a signal that will be triggered when coordinates are received from JS
    map_clicked_signal = pyqtSignal(float, float)
    wp_removed_signal = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent

    @pyqtSlot(float, float)
    def map_clicked(self, lat, lon):
        print(f"Python received: {lat}, {lon}")
        # Example action: Update list of waypoints (this could be more complex)
        self.map_clicked_signal.emit(lat, lon)  # Emit signal if you want to notify other parts of the app
        self.parent.wp_list.addItem(f"WP: {lat:.5f}, {lon:.5f}")

    @pyqtSlot(int)
    def wp_removed(self, index):
        print(f"Removed waypoint index {index}")
        # You can use a signal or directly call a JS function here
        # This would need a JS function that adds the marker
        self.wp_removed_signal.emit(index)


class MyWebEnginePage(QWebEnginePage):
    def __init__(self, parent=None):
        super().__init__(parent)

    def javaScriptConsoleMessage(self, level, message, line, sourceID):
        """Capture console messages from JavaScript."""
        print(f"JS Console Log: {message} (Line: {line}, Source: {sourceID})")


class MyWebEngineView(QWebEngineView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setPage(MyWebEnginePage(self))  # Use custom page that captures logs


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Drone Route Planner")

        self.map_widget = MyWebEngineView()  # Use custom view that captures console logs
        self.wp_list = QListWidget()

        profile = self.map_widget.page().profile()
        profile.downloadRequested.connect(self.handle_download)

        splitter = QSplitter()
        splitter.addWidget(self.wp_list)
        splitter.addWidget(self.map_widget)
        splitter.setSizes([100, 900])

        container = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(splitter)
        container.setLayout(layout)

        self.setCentralWidget(container)

        # Setup WebChannel
        self.channel = QWebChannel()
        self.bridge = Bridge(parent=self)
        self.channel.registerObject("bridge", self.bridge)
        self.map_widget.page().setWebChannel(self.channel)
        self.bridge.map_clicked_signal.connect(self.on_map_click)
        self.bridge.wp_removed_signal.connect(self.remove_wp_from_list)

        # Load the route planner map
        self.map_widget.setUrl(QUrl.fromLocalFile(os.path.abspath('route_planner_map.html')))

    def handle_download(self, download):
        download.setPath("route.csv")  # Set destination
        download.accept()

    def on_map_click(self, lat, lon):
        print(f"Map click signal received: Latitude: {lat}, Longitude: {lon}")

    def remove_wp_from_list(self, index):
        print(f"removing {index} from list")
        if 0 <= index < self.wp_list.count():
            self.wp_list.takeItem(index)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
