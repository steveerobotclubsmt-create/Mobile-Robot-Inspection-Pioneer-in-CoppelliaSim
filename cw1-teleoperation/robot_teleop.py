from PyQt5 import QtWidgets, QtGui, QtCore
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import sys

class RobotTeleop(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Industrial Inspection Robot - Tele-operation Interface')
        self.setMinimumSize(1100, 750)
        self.setStyleSheet('background-color: #1a1a2e; color: #eaeaea;')
        self.sim = None
        self.scriptHandle = None
        self.visionHandle = None
        self.topCamHandle = None
        self.robotHandle = None
        self.connected = False

        # Track which WASD keys are currently held
        self._keys = set()

        self.camTimer = QtCore.QTimer()
        self.camTimer.timeout.connect(self.refreshCameras)
        self.camTimer.setInterval(300)

        # Drive update timer — re-sends drive command while keys held
        self.driveTimer = QtCore.QTimer()
        self.driveTimer.timeout.connect(self._applyKeyDrive)
        self.driveTimer.setInterval(100)
        self.driveTimer.start()

        self.initUI()

    # ------------------------------------------------------------------ UI --
    def initUI(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        mainLayout = QtWidgets.QVBoxLayout(central)
        mainLayout.setSpacing(6)
        mainLayout.setContentsMargins(12, 12, 12, 12)

        # Title
        titleLabel = QtWidgets.QLabel('INDUSTRIAL INSPECTION ROBOT — TELE-OPERATION SYSTEM')
        titleLabel.setStyleSheet('font-size: 13px; font-weight: bold; color: #00d4ff; letter-spacing: 2px;')
        titleLabel.setAlignment(QtCore.Qt.AlignCenter)
        mainLayout.addWidget(titleLabel)

        # Status bar
        statusLayout = QtWidgets.QHBoxLayout()
        statusLayout.addWidget(self._label('STATUS:'))
        self.statusLabel = QtWidgets.QLabel('NOT CONNECTED')
        self.statusLabel.setStyleSheet('color: #ff4444; font-weight: bold; font-size: 11px;')
        statusLayout.addWidget(self.statusLabel)
        statusLayout.addStretch()
        statusLayout.addWidget(self._label('SPEED:'))
        self.speedLabel = QtWidgets.QLabel('0.0 m/s')
        self.speedLabel.setStyleSheet('color: #00d4ff; font-weight: bold;')
        statusLayout.addWidget(self.speedLabel)
        mainLayout.addLayout(statusLayout)

        # Content: cameras (left) + controls (right), proportional via stretch
        contentLayout = QtWidgets.QHBoxLayout()
        contentLayout.setSpacing(10)

        # ---- Left: cameras (stretch = 6) ----
        cameraLayout = QtWidgets.QVBoxLayout()
        cameraLayout.setSpacing(5)

        cam1Title = QtWidgets.QLabel('ROBOT CAMERA (FRONT VIEW)')
        cam1Title.setStyleSheet('color: #00d4ff; font-size: 10px; font-weight: bold;')
        cameraLayout.addWidget(cam1Title)

        self.cameraView = QtWidgets.QLabel()
        self.cameraView.setStyleSheet('background-color: #0a0a1a; border: 2px solid #00d4ff;')
        self.cameraView.setAlignment(QtCore.Qt.AlignCenter)
        self.cameraView.setText('[ Connect and start simulation ]')
        self.cameraView.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.cameraView.setMinimumHeight(200)
        cameraLayout.addWidget(self.cameraView, stretch=4)

        cam2Title = QtWidgets.QLabel('OVERHEAD VIEW (TOP CAMERA)')
        cam2Title.setStyleSheet('color: #ffaa00; font-size: 10px; font-weight: bold;')
        cameraLayout.addWidget(cam2Title)

        self.topCamView = QtWidgets.QLabel()
        self.topCamView.setStyleSheet('background-color: #0a0a1a; border: 2px solid #ffaa00;')
        self.topCamView.setAlignment(QtCore.Qt.AlignCenter)
        self.topCamView.setText('[ Add a vision sensor named "/topCamera" in the scene ]')
        self.topCamView.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.topCamView.setMinimumHeight(150)
        cameraLayout.addWidget(self.topCamView, stretch=3)

        self.snapshotBtn = self._button('📷  SAVE SNAPSHOT', '#005f73')
        self.snapshotBtn.clicked.connect(self.saveSnapshot)
        cameraLayout.addWidget(self.snapshotBtn)

        contentLayout.addLayout(cameraLayout, stretch=6)

        # ---- Right: controls (stretch = 4) ----
        rightLayout = QtWidgets.QVBoxLayout()
        rightLayout.setSpacing(5)

        # Sim control group
        menuGroup = QtWidgets.QGroupBox('SIMULATION CONTROL')
        menuGroup.setStyleSheet(self._groupStyle())
        menuLayout = QtWidgets.QVBoxLayout(menuGroup)
        menuLayout.setSpacing(4)

        self.connectBtn = self._button('🔌  CONNECT TO COPPELIA', '#006400')
        self.connectBtn.clicked.connect(self.connectSim)
        menuLayout.addWidget(self.connectBtn)

        self.startBtn = self._button('▶   START SIMULATION', '#004d00')
        self.startBtn.clicked.connect(self.startSim)
        menuLayout.addWidget(self.startBtn)

        self.stopSimBtn = self._button('⏹   STOP SIMULATION', '#4d0000')
        self.stopSimBtn.clicked.connect(self.stopSim)
        menuLayout.addWidget(self.stopSimBtn)

        self.resetBtn = self._button('🔄  RESET EMERGENCY STOP', '#4d3800')
        self.resetBtn.clicked.connect(self.resetEmergency)
        menuLayout.addWidget(self.resetBtn)

        rightLayout.addWidget(menuGroup)

        # Telemetry group
        infoGroup = QtWidgets.QGroupBox('ROBOT TELEMETRY')
        infoGroup.setStyleSheet(self._groupStyle())
        infoLayout = QtWidgets.QGridLayout(infoGroup)
        infoLayout.addWidget(self._label('X POS:'), 0, 0)
        self.xPosLabel = QtWidgets.QLabel('—')
        self.xPosLabel.setStyleSheet('color: #00ff88; font-size: 10px;')
        infoLayout.addWidget(self.xPosLabel, 0, 1)
        infoLayout.addWidget(self._label('Y POS:'), 1, 0)
        self.yPosLabel = QtWidgets.QLabel('—')
        self.yPosLabel.setStyleSheet('color: #00ff88; font-size: 10px;')
        infoLayout.addWidget(self.yPosLabel, 1, 1)
        infoLayout.addWidget(self._label('SENSORS:'), 2, 0)
        self.sensorReadout = QtWidgets.QLabel('16 ultrasonic active')
        self.sensorReadout.setStyleSheet('color: #00ff88; font-size: 10px;')
        infoLayout.addWidget(self.sensorReadout, 2, 1)
        rightLayout.addWidget(infoGroup)

        # Drive controls group
        driveGroup = QtWidgets.QGroupBox('DRIVE CONTROLS  [ W A S D ]')
        driveGroup.setStyleSheet(self._groupStyle())
        driveLayout = QtWidgets.QVBoxLayout(driveGroup)

        arrowLayout = QtWidgets.QGridLayout()
        arrowLayout.setSpacing(4)

        self.fwdBtn = self._button('▲\nFWD  [W]', '#003566')
        self.fwdBtn.setFixedHeight(55)
        self.fwdBtn.pressed.connect(lambda: self._setButtonDrive(2.0, 0))
        self.fwdBtn.released.connect(lambda: self._setButtonDrive(0, 0))

        self.bwdBtn = self._button('▼\nBWD  [S]', '#003566')
        self.bwdBtn.setFixedHeight(55)
        self.bwdBtn.pressed.connect(lambda: self._setButtonDrive(-2.0, 0))
        self.bwdBtn.released.connect(lambda: self._setButtonDrive(0, 0))

        self.leftBtn = self._button('◄\nLEFT  [A]', '#003566')
        self.leftBtn.setFixedHeight(55)
        self.leftBtn.pressed.connect(lambda: self._setButtonDrive(0, 2.0))
        self.leftBtn.released.connect(lambda: self._setButtonDrive(0, 0))

        self.rightBtn = self._button('►\nRIGHT  [D]', '#003566')
        self.rightBtn.setFixedHeight(55)
        self.rightBtn.pressed.connect(lambda: self._setButtonDrive(0, -2.0))
        self.rightBtn.released.connect(lambda: self._setButtonDrive(0, 0))

        self.stopDriveBtn = self._button('⏺\nSTOP', '#7d0000')
        self.stopDriveBtn.setFixedHeight(55)
        self.stopDriveBtn.clicked.connect(lambda: self.drive(0, 0))

        arrowLayout.addWidget(self.fwdBtn, 0, 1)
        arrowLayout.addWidget(self.leftBtn, 1, 0)
        arrowLayout.addWidget(self.stopDriveBtn, 1, 1)
        arrowLayout.addWidget(self.rightBtn, 1, 2)
        arrowLayout.addWidget(self.bwdBtn, 2, 1)
        driveLayout.addLayout(arrowLayout)

        driveLayout.addWidget(self._label('SPEED MULTIPLIER:'))
        self.speedSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speedSlider.setMinimum(1)
        self.speedSlider.setMaximum(3)
        self.speedSlider.setValue(1)
        self.speedSlider.setStyleSheet('QSlider::handle:horizontal { background: #00d4ff; width: 14px; }')
        driveLayout.addWidget(self.speedSlider)

        rightLayout.addWidget(driveGroup)
        rightLayout.addStretch()

        contentLayout.addLayout(rightLayout, stretch=4)
        mainLayout.addLayout(contentLayout)

    # --------------------------------------------------------------- Helpers --
    def _label(self, text):
        lbl = QtWidgets.QLabel(text)
        lbl.setStyleSheet('color: #aaaaaa; font-size: 10px;')
        return lbl

    def _button(self, text, color):
        btn = QtWidgets.QPushButton(text)
        btn.setStyleSheet(f'''
            QPushButton {{ background-color: {color}; color: white;
                font-weight: bold; font-size: 10px;
                border: 1px solid #00d4ff; border-radius: 4px; padding: 6px; }}
            QPushButton:hover {{ background-color: #00d4ff; color: #1a1a2e; }}
            QPushButton:pressed {{ background-color: #005f8a; }}
        ''')
        return btn

    def _groupStyle(self):
        return ('QGroupBox { color: #00d4ff; border: 1px solid #00d4ff; border-radius: 4px; '
                'margin-top: 6px; padding-top: 6px; font-weight: bold; font-size: 10px; }')

    # -------------------------------------------------------- WASD keyboard --
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        self._keys.add(event.key())

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        self._keys.discard(event.key())

    # Button-held state (separate from keyboard so they don't conflict)
    _btnSpeed = 0.0
    _btnTurn  = 0.0

    def _setButtonDrive(self, speed, turn):
        self._btnSpeed = speed
        self._btnTurn  = turn

    def _applyKeyDrive(self):
        """Called every 100 ms — resolves WASD keys into a drive command."""
        W = QtCore.Qt.Key_W
        A = QtCore.Qt.Key_A
        S = QtCore.Qt.Key_S
        D = QtCore.Qt.Key_D

        speed   = 0.0
        turning = 0.0

        if W in self._keys:
            speed   += 2.0
        if S in self._keys:
            speed   -= 2.0
        if A in self._keys:
            turning += 2.0   # positive = turn left
        if D in self._keys:
            turning -= 2.0   # negative = turn right

        # Keyboard takes priority; fall back to button state when no keys held
        if speed == 0.0 and turning == 0.0 and not self._keys:
            speed   = self._btnSpeed
            turning = self._btnTurn

        self.drive(speed, turning)

    # ---------------------------------------------------- Simulation control --
    def connectSim(self):
        self.statusLabel.setText('CONNECTING...')
        self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')
        QtWidgets.QApplication.processEvents()
        try:
            client = RemoteAPIClient()
            self.sim = client.require('sim')
            self.visionHandle = self.sim.getObject('/PioneerP3DX/visionSensor')
            self.scriptHandle = self.sim.getScript(
                self.sim.scripttype_childscript, '/PioneerP3DX')
            self.robotHandle = self.sim.getObject('/PioneerP3DX')

            try:
                self.topCamHandle = self.sim.getObject('/topCamera')
                self.topCamView.setText('[ Top camera found — start simulation ]')
            except:
                self.topCamHandle = None
                self.topCamView.setText('[ /topCamera not found in scene ]')

            self.connected = True
            self.statusLabel.setText('CONNECTED ✓')
            self.statusLabel.setStyleSheet('color: #00ff88; font-weight: bold; font-size: 11px;')
        except Exception as e:
            self.statusLabel.setText(f'FAILED: {str(e)[:50]}')
            self.statusLabel.setStyleSheet('color: #ff4444; font-weight: bold; font-size: 11px;')

    def startSim(self):
        if self.sim and self.connected:
            self.sim.startSimulation()
            self.statusLabel.setText('SIMULATION RUNNING ▶')
            self.statusLabel.setStyleSheet('color: #00ff88; font-weight: bold; font-size: 11px;')
            self.camTimer.start()
        else:
            self.statusLabel.setText('Connect first!')

    def stopSim(self):
        if self.sim and self.connected:
            self.camTimer.stop()
            self.drive(0, 0)
            self.sim.stopSimulation()
            self.statusLabel.setText('SIMULATION STOPPED ⏹')
            self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')
            self.cameraView.setText('[ Simulation stopped ]')
            self.topCamView.setText('[ Simulation stopped ]')

    def resetEmergency(self):
        if self.sim and self.scriptHandle and self.connected:
            try:
                self.sim.callScriptFunction('resetEmergency', self.scriptHandle, [])
                self._keys.clear()
                self._btnSpeed = 0.0
                self._btnTurn  = 0.0
                self.statusLabel.setText('EMERGENCY RESET — READY ✓')
                self.statusLabel.setStyleSheet('color: #00ff88; font-weight: bold; font-size: 11px;')
            except Exception as e:
                self.statusLabel.setText(f'Reset failed: {str(e)[:40]}')
        else:
            self.statusLabel.setText('Start simulation first')

    def drive(self, speed, turning):
        if self.sim and self.scriptHandle and self.connected:
            try:
                multiplier = self.speedSlider.value()
                actual_speed   = speed   * multiplier / 2
                actual_turning = turning * multiplier / 2
                # Pass multiplier so Lua can scale the e-stop threshold with speed
                self.sim.callScriptFunction('driveRobot', self.scriptHandle,
                                            [actual_speed, actual_turning, multiplier])
                self.speedLabel.setText(f'{actual_speed:.1f} m/s')
                if self.robotHandle:
                    pos = self.sim.getObjectPosition(self.robotHandle, -1)
                    self.xPosLabel.setText(f'{pos[0]:.2f} m')
                    self.yPosLabel.setText(f'{pos[1]:.2f} m')
            except:
                pass

    # --------------------------------------------------------- Camera refresh --
    def refreshCameras(self):
        # Poll emergency state — update status if Lua auto-stopped the robot
        if self.sim and self.scriptHandle and self.connected:
            try:
                result = self.sim.callScriptFunction('getEmergencyState', self.scriptHandle, [])
                if result and result[0]:
                    self.statusLabel.setText('⚠ AUTO EMERGENCY STOP — OBSTACLE DETECTED')
                    self.statusLabel.setStyleSheet('color: #ff4444; font-weight: bold; font-size: 11px;')
                    self.speedLabel.setText('0.0 m/s')
            except:
                pass

        # Front camera
        # CoppeliaSim getVisionSensorImg returns RGB — no cvtColor needed,
        # just flip vertically and pass straight to QImage Format_RGB888
        if self.sim and self.visionHandle and self.connected:
            try:
                img, [resX, resY] = self.sim.getVisionSensorImg(self.visionHandle)
                img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
                img = cv2.flip(img, 0)  # vertical flip only — no channel swap
                h, w, ch = img.shape
                qImg = QtGui.QImage(img.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
                pw = self.cameraView.width()
                ph = self.cameraView.height()
                pixmap = QtGui.QPixmap.fromImage(qImg).scaled(
                    pw, ph, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
                self.cameraView.setPixmap(pixmap)
            except Exception as e:
                self.cameraView.setText(f'Camera error: {str(e)[:50]}')

        # Top camera
        if self.sim and self.topCamHandle and self.connected:
            try:
                img, [resX, resY] = self.sim.getVisionSensorImg(self.topCamHandle)
                img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
                img = cv2.flip(img, 0)  # vertical flip only — no channel swap
                h, w, ch = img.shape
                qImg = QtGui.QImage(img.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
                pw = self.topCamView.width()
                ph = self.topCamView.height()
                pixmap = QtGui.QPixmap.fromImage(qImg).scaled(
                    pw, ph, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
                self.topCamView.setPixmap(pixmap)
            except Exception as e:
                self.topCamView.setText(f'Top cam error: {str(e)[:50]}')

    def saveSnapshot(self):
        if self.sim and self.visionHandle and self.connected:
            try:
                img, [resX, resY] = self.sim.getVisionSensorImg(self.visionHandle)
                img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
                img = cv2.flip(img, 0)
                # cv2.imwrite expects BGR, so convert RGB → BGR for correct saved colors
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                cv2.imwrite('snapshot.png', img_bgr)
                self.statusLabel.setText('SNAPSHOT SAVED → snapshot.png ✓')
            except Exception as e:
                self.statusLabel.setText(f'Snapshot error: {str(e)[:40]}')

    def closeEvent(self, event):
        self.camTimer.stop()
        self.driveTimer.stop()
        event.accept()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = RobotTeleop()
    window.show()
    sys.exit(app.exec_())