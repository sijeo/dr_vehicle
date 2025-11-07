# imu_viewer.py
# pip install pyqt5 pyqtgraph numpy
# UI uses PyQt5 + pyqtgraph.opengl, connects via TCP to Pi and renders orientation.

import sys, json, socket, threading, queue, time
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np
import pyqtgraph as pg

class StaticAxesWidget(QtWidgets.QWidget):
    """ Small static 3- axis indicator drawn in screen corner."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.setFixedSize(100, 100)
        self.margin = 10

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        center = QtCore.QPointF(self.width()/2, self.height() - self.margin*2)
        scale = 30.0

        # Draw X axis
        p.setPen(QtGui.QPen(QtGui.QColor(255,255,0), 2))
        p.drawLine(center, center + QtCore.QPointF(scale, 0))
        p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 0), 1))
        p.drawText(center + QtCore.QPointF(scale + 5, 5), "X")

        # Draw Y axis
        p.setPen(QtGui.QPen(QtGui.QColor(0,255,0), 2))
        p.drawLine(center, center + QtCore.QPointF(0, -scale))
        p.setPen(QtGui.QPen(QtGui.QColor(0, 255, 0)))
        p.drawText(center + QtCore.QPointF(5, -scale - 5), "Y")

        # Draw Z axis
        p.setPen(QtGui.QPen(QtGui.QColor(0,0,255), 2))
        p.drawLine(center, center + QtCore.QPointF(scale*0.7, -scale*0.7))
        p.setPen(QtGui.QPen(QtGui.QColor(0, 0, 255)))
        p.drawText(center + QtCore.QPointF(scale*0.7 + 5, -scale*0.7 - 5), "Z")

        p.end()




def quat_to_R( qw, qx, qy, qz ):
    # Normalize
    n = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = qw/n, qx/n, qy/n, qz/n
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z,     2*x*z + 2*w*y],
        [2*x*y + 2*w*z,     1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y,     2*y*z + 2*w*x,     1 - 2*x*x - 2*y*y]
    ], dtype=np.float32)

class NetThread(QtCore.QObject):
    connected = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal(str)
    gotPacket = QtCore.pyqtSignal(dict)

    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self._stop = False

    def stop(self):
        self._stop = True

    @QtCore.pyqtSlot()
    def run(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((self.host, self.port))
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
                        raise ConnectionError( "Peer Closed ")
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if not line.strip():
                            continue
                        try:
                            pkt = json.loads(line.decode('utf-8'))
                            self.gotPacket.emit(pkt)
                        except Exception as e:
                            print(f"JSON parse error: {e}")
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

class Viewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MPU6050 Orientation Viewer")
        self.resize(1100, 700)

        cw = QtWidgets.QWidget()
        self.setCentralWidget(cw)
        grid = QtWidgets.QGridLayout(cw)

        # Controls
        self.ipEdit = QtWidgets.QLineEdit("192.168.1.100")
        self.portEdit = QtWidgets.QLineEdit("9009")
        self.btn = QtWidgets.QPushButton("Connect")
        self.statusLbl = QtWidgets.QLabel("Disconnected")
        self.statusLbl.setStyleSheet("color:#c00; font-weight:bold;")
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Pi IP:"))
        controls.addWidget(self.ipEdit)
        controls.addWidget(QtWidgets.QLabel("Port:"))
        controls.addWidget(self.portEdit)
        controls.addWidget(self.btn)
        controls.addStretch(1)
        controls.addWidget(self.statusLbl)
        grid.addLayout(controls, 0, 0, 1, 2)

        # Instruction Banner
        self.banner = QtWidgets.QLabel("Place the IMU flat and still. Then click Connect.")
        self.banner.setAlignment(QtCore.Qt.AlignCenter)
        self.banner.setStyleSheet("font:18pt 'Segoe UI'; padding:10px; color:#333;")
        grid.addWidget(self.banner, 1, 0, 1, 2)

        # 3D View
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=8, azimuth=45, elevation=25)
        self.view.setMinimumHeight(450)
        grid.addWidget(self.view, 2, 0, 1, 2)
        grid.setRowStretch(2, 1)
        grid.setColumnStretch(0, 1)

        # Axes (Length 2)
        ax = gl.GLAxisItem(size=QtGui.QVector3D(2,2,2))
        ax.xColor = (1.0 , 0.0, 0.0, 1.0)
        ax.yColor = (0.0, 1.0, 0.0, 1.0)
        ax.zColor = (0.0, 0.0, 1.0, 1.0)
        ax.setGLOptions('opaque')
        self.view.addItem(ax)
        self.axes = ax



        # Add static screen corner axis indicator bottom-left
        self.staticAxes = StaticAxesWidget(self.view)
        self.staticAxes.move(10, self.view.height() - self.staticAxes.height() - 10)
        self.staticAxes.show()


        # Grid as Ground
        g = gl.GLGridItem()
        g.scale(1,1,1)
        g.setSize(8,8)
        g.translate(0,0,-1)
        self.view.addItem(g)

        # Cube (unit) - build mesh
        verts = np.array([
            [-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
            [-1,-1, 1], [1,-1, 1], [1,1, 1], [-1,1, 1]
            ], dtype=float)*0.8
        faces = np.array([
            [0,1,2], [0,2,3], [4,5,6], [4,6,7],
            [0,1,5], [0,5,4], [2,3,7], [2,7,6],
            [1,2,6], [1,6,5], [3,0,4], [3,4,7]
            ], dtype=int)
        colors = np.ones((faces.shape[0], 4), dtype=float)
        colors[:,3] = 0.4 # alpha
        mesh = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False, drawEdges=True)
        self.mesh = mesh
        self.view.addItem(mesh)

        # Readouts
        self.lblEuler = QtWidgets.QLabel("Yaw=- Pitch=- Roll=-")
        self.lblTime = QtWidgets.QLabel("t_ns=-")
        info = QtWidgets.QHBoxLayout()
        info.addWidget(self.lblEuler)
        info.addStretch(1)
        info.addWidget(self.lblTime)
        grid.addLayout(info, 3, 0, 1, 2)

        # net thread handle
        self.worker = None
        self.thread = None
        self.btn.clicked.connect(self.toggle)

        # initial idle rotation so user sees axes
        self.last_R = np.eye(3)

        # timer to update mesh transform smoothly
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.apply_last_R)
        self.timer.start(16)

    def apply_last_R(self):
            R = self.last_R
            M = np.eye(4)
            M[:3,:3] = R
            self.mesh.setTransform(QtGui.QMatrix4x4(*M.T.flatten()))

    def resizeEvent(self, a0):
        super().resizeEvent(a0)

        if hasattr(self, 'staticAxes'):
            self.staticAxes.move(10, self.view.height() - self.staticAxes.height() - 10)

    



    def toggle(self):
        if self.thread and self.thread.isRunning():
            # disconnect
            self.worker.stop()
            self.thread.quit(); self.thread.wait()
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
        self.statusLbl.setStyleSheet("color:#090; font-weight:bold;")
        self.btn.setText("Disconnect")
        self.banner.setText("Streaming... Move the IMU to see orientation.")

    def onDisconnected(self, reason):
        self.statusLbl.setText("Disconnected")
        self.statusLbl.setStyleSheet("color:#c00; font-weight:bold;")
        self.btn.setText("Connect")
        self.banner.setText(f"Place the IMU flat and still. Then click Connect.\n{reason}")
        if self.thread:
            self.worker.stop()
            self.thread.quit(); self.thread.wait()
            self.thread = None
            self.worker = None

    @QtCore.pyqtSlot(dict)
    def onPacket(self, pkt):
        try:
            w, x, y, z = pkt["q"]
            R = quat_to_R(w, x, y, z)
            self.last_R = R
            yaw, pitch, roll = pkt.get("euler_deg", [0, 0, 0])
            print( yaw, pitch, roll )
            self.lblEuler.setText(f"Yaw={yaw:6.2f}° Pitch={pitch:6.2f}° Roll={roll:6.2f}°")
            self.lblTime.setText(f"t_ns={pkt.get('t_ns', '-')}")
        except Exception as e:
            print(f"Packet handling error: {e}")
            pass


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    viewer = Viewer()
    viewer.show()
    sys.exit(app.exec_())




