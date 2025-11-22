# imu_lateral_viewer.py
# Visualizes lateral displacement + orientation from mpu6050_lateral_streamer.c

import sys, json, socket
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl

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
                        raise ConnectionError(" peer closed ")
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            pkt = json.loads(line.decode("utf-8"))
                            self.gotPacket.emit(pkt)
                        except Exception as e:
                            print("JSON Parse Error:", e, "line = ", line)
                except socket.timeout:
                    continue
        except Exception as e:
            self.disconnected.emit(str(e))
        finally:
            try:
                s.close()
            except:
                pass
                 

class LateralViewer(QtWidgets.QMainWindow):
     def __init__(self):
          super().__init__()
          self.setWindowTitle("MPU6050 Lateral Motion + Orientation Viewer")
          self.resize(1200, 900)

          cw = QtWidgets.QWidget()
          self.setCentralWidget(cw)
          grid = QtWidgets.QGridLayout(cw)

          #-------- top controls ------------
          self.ipEdit = QtWidgets.QLineEdit("192.168.1.10")
          self.portEdit = QtWidgets.QLineEdit("9010")
          self.btn = QtWidgets.QPushButton("Connect")
          self.statusLbl = QtWidgets.QLabel("Disconnected")
          self.statusLbl.setStyleSheet("color:#c00; font-weight:bold")

          ctr = QtWidgets.QHBoxLayout()
          ctr.addWidget(QtWidgets.QLabel("Pi IP:"))
          ctr.addWidget(self.ipEdit)
          ctr.addWidget(QtWidgets.QLabel("Port:"))
          ctr.addWidget(self.portEdit)
          ctr.addWidget(self.btn)
          ctr.addStretch(1)
          ctr.addWidget(self.statusLbl)
          grid.addLayout(ctr, 0, 0, 1, 2)

          self.banner = QtWidgets.QLabel(
               "Place IMU still while Pi Calibrates.\n"
               " Then move it: The cube will translate (position) and tilt(orientation)."
          )

          self.banner.setAlignment(QtCore.Qt.AlignCenter)
          self.banner.setStyleSheet("font: 14pt 'Segoe UI'; padding:8px; color:#333;")
          grid.addWidget(self.banner, 1, 0, 1, 2)

          # ------------3D View ----------------
          self.view = gl.GLViewWidget()
          self.view.setCameraPosition(distance=10, azimuth=45, elevation=40)
          self.view.opts['fov'] = 45
          self.view.setMinimumHeight(600)
          grid.addWidget(self.view, 2, 0, 1, 2)

          grid.setRowStretch(0, 0)
          grid.setRowStretch(1, 0)
          grid.setRowStretch(2, 1)
          grid.setRowStretch(3, 0)

          # ground grid
          g = gl.GLGridItem()
          g.setSize(20, 20)
          #g.getSpacing(1, 1)
          g.translate(0, 0, -3)
          self.view.addItem(g)

          # static world axes (X yellow, Y green, Z blue)
          axis_lem =5.0
          pts = np.array([
                [0, 0, 0], [axis_lem, 0, 0],
                [0, 0, 0], [0, axis_lem, 0],
                [0, 0, 0], [0, 0, axis_lem],
             ], dtype=np.float32)
          cols = np.array([
                [1, 1, 0, 1], [1, 1, 0, 1],
                [0, 1, 0, 1], [0, 1, 0, 1],
                [0, 0, 1, 1], [0, 0, 1, 1],
             ], dtype=np.float32)
          axis = gl.GLLinePlotItem(pos=pts, color=cols, width=3, antialias=True, mode='lines')
          self.view.addItem(axis)

          # Cube
          self.cube = self._make_cube(size = 0.3)
          self.view.addItem(self.cube)

          # Numeric Readouts
          self.lblPos = QtWidgets.QLabel("Position: X=0.000 Y=0.000 Z=0.000(m)")
          self.lblPos.setStyleSheet("font: 12pt 'Consolas'; color:#222; padding:4px;")
          self.lblEuler = QtWidgets.QLabel("Orientation: Yaw=0.00 Pitch=0.00 Roll=0.00(deg)")
          self.lblEuler.setStyleSheet("font: 12pt 'Consolas'; color:#222; padding:4px;")

          grid.addWidget(self.lblPos, 3, 0, 1, 1)
          grid.addWidget(self.lblEuler, 3, 1, 1, 1)

            # Network Thread
          self.thread = None
          self.worker = None
          self.btn.clicked.connect(self.toggle)

          # state from IMU
          self.last_pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
          self.last_euler = np.array([0.0, 0.0, 0.0], dtype=np.float32) # yaw, pitch, roll

          # animation timer
          self.motion_scale = 1.0 # visualize 1:1; increase to exaggerate motion
          self.timer = QtCore.QTimer()
          self.timer.timeout.connect(self.onFrame)
          self.timer.start(16) # ~60fps

     def _make_cube(self, size=1.0):
          s = size
          verts = np.array([
               [-s, -s, -s], [ s, -s, -s], [ s,  s, -s], [-s,  s, -s], # bottom
               [-s, -s,  s], [ s, -s,  s], [ s,  s,  s], [-s,  s,  s], # top
          ], dtype=np.float32)
          faces = np.array([
               [0, 1, 2], [0, 2, 3], # bottom
               [4, 5, 6], [4, 6, 7], # top
               [0, 1, 5], [0, 5, 4], # front
               [2, 3, 7], [2, 7, 6], # back
               [1, 2, 6], [1, 6, 5], # right
               [3, 0, 4], [3, 4, 7], # left
          ], dtype=np.uint32)
          colors = np.ones((faces.shape[0], 4), dtype=np.float32) 
          colors[:, 3] = 0.6
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
               self.statusLbl.setStyleSheet("color:#c00; font-weight:bold")
               self.btn.setText("Connect")
               return 
          
          host = self.ipEdit.text().strip()
          port = int(self.portEdit.text().strip())
          self.worker = NetThread(host, port)
          self.thread = QtCore.QThread()
          self.worker.moveToThread(self.thread)
          self.worker.connected.connect(self.onConnected)
          self.worker.disconnected.connect(self.onDisconnected)
          self.worker.gotPacket.connect(self.onGotPacket)
          self.thread.started.connect(self.worker.run)
          self.thread.start()

     def onConnected(self):
          self.statusLbl.setText("Connected")
          self.statusLbl.setStyleSheet("color:#0a0; font-weight:bold")
          self.btn.setText("Disconnect")
          self.banner.setText("Move the IMU: The cube will translate (position) and tilt(orientation) according to fused pose.")

     def onDisconnected(self, reason):
          self.statusLbl.setText(f"Disconnected: {reason}")
          self.statusLbl.setStyleSheet("color:#c00; font-weight:bold")
          self.btn.setText("Connect")
          self.banner.setText("Place IMU still while Pi Calibrates.\n Then move it: The cube will translate (position) and tilt(orientation).")
          print("Disconnected:", reason)
          if self.thread:
               self.worker.stop()
               self.thread.quit()
               self.thread.wait()
               self.thread = None
               self.worker = None

     @QtCore.pyqtSlot(dict)
     def onGotPacket(self, pkt):
          try:
               print("Received packet:", pkt)
               pos = np.array(pkt.get("pos_m", [0.0, 0.0, 0.0]), dtype=np.float32)
               euler = np.array(pkt.get("euler_deg", [0.0, 0.0, 0.0]), dtype=np.float32)
               self.last_pos = pos
               self.last_euler = euler
               #print(f"Packet: pos={pos.tolist()} euler={euler.tolist()}")

               self.lblPos.setText(
                    f"pos: X={pos[0]:.3f} Y={pos[1]:.3f} Z={pos[2]:.3f} (m)" )
               self.lblEuler.setText(
                    f"Yaw={euler[0]:.2f} Pitch={euler[1]:.2f} Roll={euler[2]:.2f} (deg)" )
          except Exception as e:
               print("Packet handling error:", e)

     def onFrame(self):
          # Translation
          x, y, z = (self.last_pos * self.motion_scale).tolist()
          yaw, pitch, roll = self.last_euler.tolist()

          M = QtGui.QMatrix4x4()
          M.translate(x, y, z)

          # Rotation order: yaw (Z), pitch (Y), roll (X)
          M.rotate(yaw, 0, 0, 1)   # yaw
          M.rotate(pitch, 0, 1, 0) # pitch  
          M.rotate(roll, 1, 0, 0)  # roll   
          self.cube.setTransform(M)


if __name__ == "__main__":
        app = QtWidgets.QApplication(sys.argv)
        w = LateralViewer()
        w.show()
        sys.exit(app.exec_())









