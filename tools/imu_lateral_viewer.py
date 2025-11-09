# imu_lateral_viewer.py
# Visualizes lateral displacement from mpu6050_lateral_streamer.c
# 
# Usage: python imu_lateral_viewer.py
# Then enter Pi IP , port 9010, click connect.

import sys, json, socket, threading
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np

class NetThread(QtCore.QObject):
    connected = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal(str)
    gotPacket = QtCore.pyqtSignal(dict)

    def __init__(self, ip, port):
        super().__init__()
        self.ip = ip
        self.port = port
        self._stop = False

    def stop(self):
        self._stop = True

    @QtCore.pyqtSlot()
    def run(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((self.ip, self.port))
            s.settimeout(1.0)
            self.connected.emit()
        except Exception as e:
            self.disconnected.emit(f"Connection error: {e}")
            return
        
        buf = b""
        try:
            while not self._stop:
                try:
                    chunk = s.recv(4096)
                    if not chunk:
                        raise Exception("Socket closed")
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        if not line.strip():
                            continue
                        try:
                            pkt = json.loads(line.decode('utf-8'))
                            self.gotPacket.emit(pkt)
                        except json.JSONDecodeError:
                            pass
                except socket.timeout:
                    continue
        except Exception as e:
            self.disconnected.emit(f"Connection lost: {e}")
        finally:
            try:
                s.close()
            except:
                pass

class LateralViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Lateral Viewer")
        self.resize(1100, 700)

        centralWidget = QtWidgets.QWidget()
        self.setCentralWidget(centralWidget)
        grid = QtWidgets.QGridLayout(centralWidget)

        # top controls
        self.ipEdit = QtWidgets.QLineEdit("192.168.1.100")
        self.portEdit = QtWidgets.QLineEdit("9010")
        self.btn = QtWidgets.QPushButton("Connect")
        self.statusLbl = QtWidgets.QLabel("Disconnected")
        self.statusLbl.setStyleSheet("color:#c00; font-weight:bold;")

        ctr = QtWidgets.QHBoxLayout()
        ctr.addWidget(QtWidgets.QLabel("Pi IP:"))
        ctr.addWidget(self.ipEdit)
        ctr.addWidget(QtWidgets.QLabel("Port:"))
        ctr.addWidget(self.portEdit)
        ctr.addWidget(self.btn)
        ctr.addStretch(1)
        ctr.addWidget(self.statusLbl)
        grid.addLayout(ctr, 0, 0, 1, 2)

        # instructions
        self.banner = QtWidgets.QLabel(
            "Place IMU flat and still while Pi Calibrates.\n"
            "Then move IMU laterally; the cube below shows X, Y, Z displacement (m)."
        )

        self.banner.setAlignment(QtCore.Qt.AlignCenter)
        self.banner.setStyleSheet("font: 14pt 'Segoe UI'; padding:10px; color:#333;")
        grid.addWidget(self.banner, 1, 0, 1, 2)

        # 3D cube view
        self.view = gl.GLViewWidget()
        self.view.setMinimumHeight(600)
        self.view.opts['fov'] = 45
        self.view.setCameraPosition(distance=10, azimuth=45, elevation=25)
        grid.addWidget(self.view, 2, 0, 1, 2)

        # ground grid
        g = gl.GLGridItem()
        g.setSize(20, 20)
        g.setSpacing(1, 1)
        g.translate(0, 0, -3)
        self.view.addItem(g)

        # static world axis RGB
        axis_len = 2.0
        pts = np.array([
            [0, 0, 0], [axis_len, 0, 0],
            [0, 0, 0], [0, axis_len, 0],
            [0, 0, 0], [0, 0, axis_len]
        ], dtype=np.float32)
        cols = np.array([
            [1, 1, 0, 1], [1, 1, 0, 1], # X - yellow
            [0, 1, 0, 1], [0, 1, 0, 1], # Y - green
            [0, 0, 1, 1], [0, 0, 1, 1]  # Z - blue
        ], dtype=np.float32)

        axes = gl.GLLinePlotItem(pos=pts, color=cols, width=2, antialias=True, mode='lines')
        self.view.addItem(axes)

        # moving cube representing IMU
        self.cube = self._make_cube(size=0.3)
        self.view.addItem(self.cube)

        # numeric readouts
        self.lblPos = QtWidgets.QLabel("Position (m): X=0.000 Y=0.000 Z=0.000")
        self.lblPos.setStyleSheet("font: 12pt 'Consolas'; padding:5px;")
        grid.addWidget(self.lblPos, 3, 0, 1, 2)

        # net thread
        self.thread = None
        self.worker = None
        self.btn.clicked.connect(self.toggle)

        self.last_pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # tiemr: smooth transition if needed
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.onFrame)
        self.timer.start(16)  # ~60 FPS

    def _make_cube(self, size=0.3):
        s = size
        verts = np.array([
            [-s, -s, -s],
            [ s, -s, -s],
            [ s,  s, -s],
            [-s,  s, -s],
            [-s, -s,  s],
            [ s, -s,  s],
            [ s,  s,  s],
            [-s,  s,  s],
        ], dtype=np.float32)
        faces = np.array([
            [0, 1, 2], [0, 2, 3],
            [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4],
            [2, 3, 7], [2, 7, 6],
            [1, 2, 6], [1, 6, 5],
            [3, 0, 4], [3, 4, 7],
        ], dtype=np.uint32)

        colors = np.ones((faces.shape[0], 4), dtype=np.float32)
        colors[:, 3] = 0.6  # set alpha
        cube = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False, drawEdges=True, edgeColor=(0,0,0,1))
        return cube
        
    def toggle(self):
        if self.thread and self.thread.isRunning():
            self.worker.stop()
            self.thread.quit()
            self.thread.wait()
            self.thread = None
            self.worker = None
            self.statusLbl.setText("Disconnected")
            self.statusLbl.setStyleSheet("color:#c00; font-weight:bold;")
            self.btn.setText("Connect")
            return
            
        host = self.ipEdit.text().strip()
        port = int(self.portEdit.text().strip())
        self.worker = NetThread(host, port)
        self.thread = QtCore.QThread()
        self.worker.moveToThread(self.thread)
        self.worker.connected.connect(self.onConnected)
        self.worker.disconnected.connect(self.onDisconnected)
        self.worker.gotPacket.connect(self.onPacket)
        self.thread.started.connect(self.worker.run)
        self.thread.start()

    def onConnected(self):
        self.statusLbl.setText("Connected")
        self.statusLbl.setStyleSheet("color:#0a0; font-weight:bold;")
        self.btn.setText("Disconnect")
        self.banner.setText("Move the IMU; cube show the lateral displacement (m).")

    def onDisconnected(self, reason):
        self.statusLbl.setText("Disconnected")
        self.statusLbl.setStyleSheet("color:#c00; font-weight:bold;")
        self.btn.setText("Connect")
        self.banner.setText(
            "Place IMU flat and still while Pi Calibrates.\n"
            "Then move IMU laterally; the cube below shows X, Y, Z displacement (m)."
        )
        if self.thread:
            self.worker.stop()
            self.thread.quit()
            self.thread.wait()
            self.thread = None
            self.worker = None

    @QtCore.pyqtSlot(dict)
    def onPacket(self, pkt):
        try:
            pos = np.array(pkt.get("pos_m",[0.0,0.0,0.0]), dtype=float)
            self.last_pos = pos
            print(pos)
            self.lblPos.setText(f"X={pos[0]:.3f} m Y={pos[1]:.3f} m Z={pos[2]:.3f} m")
        except Exception as e:
            pass

    def onFrame(self):
        # Apply translation to cube based on last received position
        x, y, z = self.last_pos
        # Build transformation matrix
        tr = QtGui.QMatrix4x4()
        tr.translate(x, y, z)
        self.cube.setTransform(tr)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    viewer = LateralViewer()
    viewer.show()
    sys.exit(app.exec_())


                