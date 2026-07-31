from PyQt5 import QtWidgets, QtGui, QtCore
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import cv2
import math
import sys
import heapq

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
        self.targetDummyHandle = None
        
        self.connected = False
        self.simRunning = False 

        self.topCamWorldCenter = (0.0, 0.0)
        self.topCamWorldWidth = 5.77
        self.topCamWorldHeight = 5.77
        self.latestTopCamImg = None

        self._keys = set()

        # ---- Navigation (A* Waypoint state) ----
        self.navTarget = None
        self.navTargetWorld = None
        self.waypoints = []             
        self.navMaxSpeed = 1.5
        self.navKLinear = 1.2
        self.navKAngular = 1.5

        # ---- Simple Recovery State ----
        self.recoveryPhase = 0   
        self.recoveryTicks = 0
        self.recoveryTurnSpeed = 0.0
        self.stuckCheckPos = None
        self.stuckTicks = 0

        self.robotPath = []
        self.pathMaxPoints = 400
        self.pathSampleMinDist = 0.03

        self.camTimer = QtCore.QTimer()
        self.camTimer.timeout.connect(self.refreshCameras)
        self.camTimer.setInterval(300)

        self.driveTimer = QtCore.QTimer()
        self.driveTimer.timeout.connect(self._applyKeyDrive)
        self.driveTimer.setInterval(100)
        self.driveTimer.start()

        self.sonarAngles = [
            -1.5708, -1.309, -0.9163, -0.5236, -0.1745, 0.1745, 0.5236, 0.9163,
            1.309, 1.5708, 1.9199, 2.2689, 2.618, -2.618, -2.2689, -1.9199,
        ]
        self.avoidRadius = 0.30   

        self.initUI()

    # ------------------------------------------------------------------ UI ---
    def initUI(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        mainLayout = QtWidgets.QVBoxLayout(central)
        mainLayout.setSpacing(6)
        mainLayout.setContentsMargins(12, 12, 12, 12)

        titleLabel = QtWidgets.QLabel('INDUSTRIAL INSPECTION ROBOT — TELE-OPERATION SYSTEM')
        titleLabel.setStyleSheet('font-size: 13px; font-weight: bold; color: #00d4ff; letter-spacing: 2px;')
        titleLabel.setAlignment(QtCore.Qt.AlignCenter)
        mainLayout.addWidget(titleLabel)

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

        contentLayout = QtWidgets.QHBoxLayout()
        contentLayout.setSpacing(10)

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

        cam2Title = QtWidgets.QLabel('OVERHEAD VIEW (TOP CAMERA) — CLICK TO SEND ROBOT THERE')
        cam2Title.setStyleSheet('color: #ffaa00; font-size: 10px; font-weight: bold;')
        cameraLayout.addWidget(cam2Title)

        self.topCamView = QtWidgets.QLabel()
        self.topCamView.setStyleSheet('background-color: #0a0a1a; border: 2px solid #ffaa00;')
        self.topCamView.setAlignment(QtCore.Qt.AlignCenter)
        self.topCamView.setText('[ Add a vision sensor named "/topCamera" in the scene ]')
        self.topCamView.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.topCamView.setMinimumHeight(150)
        self.topCamView.setCursor(QtCore.Qt.PointingHandCursor)
        self.topCamView.mousePressEvent = self._onTopCamClicked
        cameraLayout.addWidget(self.topCamView, stretch=3)

        self.snapshotBtn = self._button('📷  SAVE SNAPSHOT', '#005f73')
        self.snapshotBtn.clicked.connect(self.saveSnapshot)
        cameraLayout.addWidget(self.snapshotBtn)

        contentLayout.addLayout(cameraLayout, stretch=6)

        rightLayout = QtWidgets.QVBoxLayout()
        rightLayout.setSpacing(5)

        menuGroup = QtWidgets.QGroupBox('SIMULATION CONTROL')
        menuGroup.setStyleSheet(self._groupStyle())
        menuLayout = QtWidgets.QVBoxLayout(menuGroup)
        menuLayout.setSpacing(4)

        self.connectBtn = self._button('🔌  CONNECT TO COPPELIA', '#006400')
        self.connectBtn.clicked.connect(self.connectSim)
        menuLayout.addWidget(self.connectBtn)

        self.startBtn = self._button('▶️   START SIMULATION', '#004d00')
        self.startBtn.clicked.connect(self.startSim)
        menuLayout.addWidget(self.startBtn)

        self.stopSimBtn = self._button('⏹   STOP SIMULATION', '#4d0000')
        self.stopSimBtn.clicked.connect(self.stopSim)
        menuLayout.addWidget(self.stopSimBtn)

        rightLayout.addWidget(menuGroup)

        pathGroup = QtWidgets.QGroupBox('ROBOT PATH')
        pathGroup.setStyleSheet(self._groupStyle())
        pathLayout = QtWidgets.QVBoxLayout(pathGroup)
        self.pathInfoLabel = QtWidgets.QLabel('No path recorded yet')
        self.pathInfoLabel.setStyleSheet('color: #00ff88; font-size: 10px;')
        pathLayout.addWidget(self.pathInfoLabel)
        self.clearPathBtn = self._button('🧹  CLEAR PATH', '#4d3800')
        self.clearPathBtn.clicked.connect(self.clearPath)
        pathLayout.addWidget(self.clearPathBtn)
        rightLayout.addWidget(pathGroup)

        navGroup = QtWidgets.QGroupBox('AUTONOMOUS NAVIGATION')
        navGroup.setStyleSheet(self._groupStyle())
        navLayout = QtWidgets.QVBoxLayout(navGroup)
        self.navTargetLabel = QtWidgets.QLabel('No target set')
        self.navTargetLabel.setStyleSheet('color: #00d4ff; font-size: 10px;')
        navLayout.addWidget(self.navTargetLabel)
        self.cancelNavBtn = self._button('✖  CANCEL NAVIGATION', '#4d3800')
        self.cancelNavBtn.clicked.connect(self.cancelNavigation)
        navLayout.addWidget(self.cancelNavBtn)
        rightLayout.addWidget(navGroup)

        driveGroup = QtWidgets.QGroupBox('DRIVE CONTROLS  [ W A S D ]')
        driveGroup.setStyleSheet(self._groupStyle())
        driveLayout = QtWidgets.QVBoxLayout(driveGroup)

        arrowLayout = QtWidgets.QGridLayout()
        arrowLayout.setSpacing(4)

        # FIXED: Lowered instant button speed from 2.0 to 0.8
        self.fwdBtn = self._button('▲\nFWD  [W]', '#003566')
        self.fwdBtn.setFixedHeight(55)
        self.fwdBtn.pressed.connect(lambda: self._setButtonDrive(0.8, 0))
        self.fwdBtn.released.connect(lambda: self._setButtonDrive(0, 0))

        self.bwdBtn = self._button('▼\nBWD  [S]', '#003566')
        self.bwdBtn.setFixedHeight(55)
        self.bwdBtn.pressed.connect(lambda: self._setButtonDrive(-0.8, 0))
        self.bwdBtn.released.connect(lambda: self._setButtonDrive(0, 0))

        self.leftBtn = self._button('◄\nLEFT  [A]', '#003566')
        self.leftBtn.setFixedHeight(55)
        self.leftBtn.pressed.connect(lambda: self._setButtonDrive(0, 0.8))
        self.leftBtn.released.connect(lambda: self._setButtonDrive(0, 0))

        self.rightBtn = self._button('►\nRIGHT  [D]', '#003566')
        self.rightBtn.setFixedHeight(55)
        self.rightBtn.pressed.connect(lambda: self._setButtonDrive(0, -0.8))
        self.rightBtn.released.connect(lambda: self._setButtonDrive(0, 0))

        self.stopDriveBtn = self._button('⏺\nSTOP', '#7d0000')
        self.stopDriveBtn.setFixedHeight(55)
        self.stopDriveBtn.clicked.connect(lambda: (self.cancelNavigation(), self.drive(0, 0)))

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

    def keyPressEvent(self, event):
        if event.isAutoRepeat(): return
        if self.navTarget is not None: self.cancelNavigation()
        self._keys.add(event.key())

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat(): return
        self._keys.discard(event.key())

    _btnSpeed = 0.0
    _btnTurn  = 0.0

    def _setButtonDrive(self, speed, turn):
        if self.navTarget is not None: self.cancelNavigation()
        self._btnSpeed = speed
        self._btnTurn  = turn

    def _applyKeyDrive(self):
        if not self.simRunning: 
            return

        if self.navTarget is not None:
            self._navigateStep()
            return

        W, A, S, D = QtCore.Qt.Key_W, QtCore.Qt.Key_A, QtCore.Qt.Key_S, QtCore.Qt.Key_D
        speed, turning = 0.0, 0.0

        # FIXED: Lowered raw keyboard speed from 2.0 to a smooth 0.8
        if W in self._keys: speed += 0.8
        if S in self._keys: speed -= 0.8
        if A in self._keys: turning += 0.8
        if D in self._keys: turning -= 0.8

        if speed == 0.0 and turning == 0.0 and not self._keys:
            speed, turning = self._btnSpeed, self._btnTurn

        self.drive(speed, turning)

    def _pixelToWorld(self, u, v):
        cx, cy = self.topCamWorldCenter
        return cx - (u - 0.5) * self.topCamWorldWidth, cy + (v - 0.5) * self.topCamWorldHeight

    def _worldToPixel(self, wx, wy, pixW, pixH):
        cx, cy = self.topCamWorldCenter
        return (0.5 - (wx - cx) / self.topCamWorldWidth) * pixW, ((wy - cy) / self.topCamWorldHeight + 0.5) * pixH

    def _ensureDummyHandle(self):
        if self.targetDummyHandle is not None:
            try:
                self.sim.getObjectPosition(self.targetDummyHandle, -1)
                return
            except Exception:
                self.targetDummyHandle = None

        try:
            self.targetDummyHandle = self._findObject(['/NavTarget', '/NavTarget#0'])
            return
        except Exception: pass

        try:
            self.targetDummyHandle = self.sim.createDummy(0.1)
            self.sim.setObjectAlias(self.targetDummyHandle, 'NavTarget')
            self.sim.setObjectPosition(self.targetDummyHandle, -1, [0.0, 0.0, 0.05])
        except Exception:
            self.targetDummyHandle = None

    # =========================================================================
    # VISION A* SEARCH PATH PLANNING
    # =========================================================================
    def _astar(self, grid, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        
        oheap = []
        heapq.heappush(oheap, (math.hypot(goal[0]-start[0], goal[1]-start[1]), start))
        h, w = grid.shape
        
        while oheap:
            current = heapq.heappop(oheap)[1]
            
            if current in close_set:
                continue
                
            if math.hypot(current[0]-goal[0], current[1]-goal[1]) < 4:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.reverse()
                return data
                
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                if 0 <= neighbor[0] < w and 0 <= neighbor[1] < h:
                    if grid[neighbor[1]][neighbor[0]] == 255:
                        continue
                else:
                    continue
                    
                cost = 1.414 if i != 0 and j != 0 else 1.0
                tentative_g_score = gscore[current] + cost
                
                if tentative_g_score < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore = tentative_g_score + math.hypot(neighbor[0]-goal[0], neighbor[1]-goal[1])
                    heapq.heappush(oheap, (fscore, neighbor))
        return []

    def _planPathVision(self, startWorld, goalWorld):
        if self.latestTopCamImg is None:
            return [(goalWorld[0], goalWorld[1])]

        pixW, pixH = self.latestTopCamImg.shape[1], self.latestTopCamImg.shape[0]

        sx, sy = self._worldToPixel(startWorld[0], startWorld[1], pixW, pixH)
        gx, gy = self._worldToPixel(goalWorld[0], goalWorld[1], pixW, pixH)
        sx, sy, gx, gy = int(sx), int(sy), int(gx), int(gy)

        gx = max(0, min(pixW-1, gx))
        gy = max(0, min(pixH-1, gy))

        gray = cv2.cvtColor(self.latestTopCamImg, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY_INV)

        cv2.circle(thresh, (sx, sy), 22, 0, -1)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
        grid = cv2.dilate(thresh, kernel)

        cv2.circle(grid, (sx, sy), 18, 0, -1)
        cv2.circle(grid, (gx, gy), 18, 0, -1)

        path_pixels = self._astar(grid, (sx, sy), (gx, gy))

        if not path_pixels:
            return []

        waypoints = []
        for px, py in path_pixels[::5]:
            wx, wy = self._pixelToWorld(px / pixW, py / pixH)
            waypoints.append((wx, wy))
            
        waypoints.append((goalWorld[0], goalWorld[1]))
        return waypoints

    def _onTopCamClicked(self, event):
        if not (self.sim and self.connected and self.simRunning):
            self.statusLabel.setText('Start simulation first!')
            return

        pixmap = self.topCamView.pixmap()
        if pixmap is None or pixmap.isNull(): return

        pixW, pixH = pixmap.width(), pixmap.height()
        clickX = event.pos().x() - (self.topCamView.width() - pixW) / 2.0
        clickY = event.pos().y() - (self.topCamView.height() - pixH) / 2.0

        if not (0 <= clickX <= pixW and 0 <= clickY <= pixH): return

        worldX, worldY = self._pixelToWorld(clickX / pixW, clickY / pixH)
        self._ensureDummyHandle()

        if self.targetDummyHandle is not None:
            try:
                z = self.sim.getObjectPosition(self.targetDummyHandle, -1)[2]
                self.sim.setObjectPosition(self.targetDummyHandle, -1, [worldX, worldY, z])
            except Exception:
                self.targetDummyHandle = None

        try:
            pos = self.sim.getObjectPosition(self.robotHandle, -1)
            robotWorld = (pos[0], pos[1])
        except Exception:
            return

        # RESET FLAGS ON NEW CLICK
        self.recoveryPhase = 0
        self.recoveryTicks = 0
        self.recoveryTurnSpeed = 0.0
        self.stuckCheckPos = None
        self.stuckTicks = 0

        self.statusLabel.setText('CALCULATING PATH (VISION A*)...')
        QtWidgets.QApplication.processEvents()

        self.waypoints = self._planPathVision(robotWorld, (worldX, worldY))

        if self.waypoints:
            self.robotPath = []
            self.navTargetWorld = (worldX, worldY)
            self.navTarget = True
            self.navTargetLabel.setText(f'Target: ({worldX:.2f}, {worldY:.2f}) m')
            self.statusLabel.setText(f'FOLLOWING A* PATH TO ({worldX:.2f}, {worldY:.2f})')
            self.statusLabel.setStyleSheet('color: #00ff88; font-weight: bold; font-size: 11px;')
        else:
            self.statusLabel.setText('PLANNING FAILED: UNREACHABLE LOCATION')
            self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')
            self.cancelNavigation()

    def cancelNavigation(self):
        if self.navTarget is not None:
            self.navTarget = None
            self.waypoints = []
            self.recoveryPhase = 0
            self.recoveryTicks = 0
            self.recoveryTurnSpeed = 0.0
            self.stuckCheckPos = None
            self.stuckTicks = 0
            self.navTargetLabel.setText('No target set')
            self.drive(0, 0)

    def clearPath(self):
        self.robotPath = []
        self.pathInfoLabel.setText('No path recorded yet')

    def _recordPathPoint(self, x, y):
        if self.robotPath:
            lastX, lastY = self.robotPath[-1]
            if math.hypot(x - lastX, y - lastY) < self.pathSampleMinDist: return
        self.robotPath.append((x, y))
        if len(self.robotPath) > self.pathMaxPoints: self.robotPath.pop(0)
        self.pathInfoLabel.setText(f'{len(self.robotPath)} points recorded')

    def _decideTurnDirection(self):
        """
        Uses a long-range threshold (up to 5.0m) to evaluate the deep volume 
        of open space to the left versus the right side of the robot. 
        Returns turn direction multiplier (1.0 for Left, -1.0 for Right).
        """
        if not (self.sim and self.scriptHandle and self.simRunning): return 1.0
        try:
            distances = self.sim.callScriptFunction('getSonarDistances', self.scriptHandle, [])
        except Exception: return 1.0

        if not distances: return 1.0
        
        FAR_THRESHOLD = 5.0 
        
        # Uses ALL 8 sensors on the left hemisphere
        left_space = sum([min(distances[i], FAR_THRESHOLD) for i in [0, 1, 2, 3, 12, 13, 14, 15] if i < len(distances)])
        # Uses ALL 8 sensors on the right hemisphere
        right_space = sum([min(distances[i], FAR_THRESHOLD) for i in [4, 5, 6, 7, 8, 9, 10, 11] if i < len(distances)])
        
        # Determine safest side
        if left_space > right_space:
            return 1.0  # Turn Left towards more open space
        else:
            return -1.0 # Turn Right towards more open space

    def _navigateStep(self):
        """Pure Pursuit Waypoint Follower w/ Smart Escaping"""
        if not (self.sim and self.robotHandle and self.connected and self.simRunning): return

        try:
            pos = self.sim.getObjectPosition(self.robotHandle, -1)
            ori = self.sim.getObjectOrientation(self.robotHandle, -1)
        except Exception: return

        dist_to_final_target = math.hypot(self.navTargetWorld[0] - pos[0], self.navTargetWorld[1] - pos[1])

        # -------------------------------------------------------------
        # 1. "CLOSE ENOUGH" & JAM DETECTION
        # -------------------------------------------------------------
        if self.stuckCheckPos is None:
            self.stuckCheckPos = (pos[0], pos[1])
            self.stuckTicks = 0
        else:
            moved = math.hypot(pos[0] - self.stuckCheckPos[0], pos[1] - self.stuckCheckPos[1])
            if moved < 0.015:
                self.stuckTicks += 1
            else:
                self.stuckTicks = 0
                self.stuckCheckPos = (pos[0], pos[1])

        if self.stuckTicks > 8: 
            if dist_to_final_target < 0.45:
                self.drive(0, 0)
                self.statusLabel.setText('TARGET REACHED (NEARBY) ✓')
                self.statusLabel.setStyleSheet('color: #00ff88; font-weight: bold; font-size: 11px;')
                self.cancelNavigation()
                return
            elif self.recoveryPhase == 0:
                # Physically jammed! Start the 4-step sequence
                self.recoveryPhase = 1  
                self.recoveryTicks = 10  # Backup first
                self.recoveryTurnSpeed = self._decideTurnDirection() * 0.785 # 45 Degree Pivot Turn
                self.waypoints = [] 
                self.stuckTicks = 0
                self.drive(0, 0, autonomous=True) 
                return

        # -------------------------------------------------------------
        # 2. SMART RECOVERY (BACKUP -> PIVOT -> ESCAPE DRIVE -> REPLAN)
        # -------------------------------------------------------------
        if self.recoveryPhase == 1:
            self.recoveryTicks -= 1
            rear_dist = self._getRearSonarDistance()
            
            # SENSE BACKWARD MOTION: Stop instantly if an obstacle is directly behind!
            if rear_dist < 0.15:
                self.recoveryTicks = 0
            
            if self.recoveryTicks > 0:
                self.statusLabel.setText('⚠ OBSTACLE: REVERSING')
                self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')
                self.drive(-0.4, 0.0, autonomous=True) # STRAIGHT BACKUP
                return
            else:
                self.recoveryPhase = 2
                self.recoveryTicks = 10 # Proceed to pivot for 1.0 seconds
                self.drive(0.0, 0.0, autonomous=True) # Brake backward momentum safely
                return

        elif self.recoveryPhase == 2:
            self.recoveryTicks -= 1
            if self.recoveryTicks > 0:
                dir_str = "LEFT" if self.recoveryTurnSpeed > 0 else "RIGHT"
                self.statusLabel.setText(f'⚠ OBSTACLE: PIVOTING {dir_str} (FAR SPACE)')
                self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')
                self.drive(0.0, self.recoveryTurnSpeed, autonomous=True) # FIXED PIVOT
                return
            else:
                self.recoveryPhase = 3
                self.recoveryTicks = 10 # 1.0 seconds to drive forward/backward to escape
                self.drive(0.0, 0.0, autonomous=True) # BRAKE
                return

        elif self.recoveryPhase == 3:
            self.recoveryTicks -= 1
            front_dist = self._getFrontSonarDistance()
            rear_dist = self._getRearSonarDistance()
            
            # Decide whether to escape by driving forward or backward
            if front_dist > 0.25:
                speed = 0.4
                direction_str = "FORWARD"
            elif rear_dist > 0.15:
                speed = -0.4
                direction_str = "BACKWARD"
            else:
                speed = 0.0
                self.recoveryTicks = 0 # Cannot escape safely, abort
                direction_str = "BLOCKED"
                
            if self.recoveryTicks > 0 and speed != 0.0:
                self.statusLabel.setText(f'⚠ OBSTACLE: ESCAPING {direction_str}')
                self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')
                self.drive(speed, 0.0, autonomous=True)
                return
            else:
                self.recoveryPhase = 4
                self.recoveryTicks = 5 # Short pause before replanning
                self.drive(0.0, 0.0, autonomous=True) # BRAKE
                return

        elif self.recoveryPhase == 4:
            self.recoveryTicks -= 1
            self.drive(0.0, 0.0, autonomous=True) 
            if self.recoveryTicks > 0:
                self.statusLabel.setText('⚠ OBSTACLE: BRAKING')
                self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')
                return
            else:
                # Sequence complete. Replan from the newly escaped location.
                self.recoveryPhase = 0
                try:
                    pos = self.sim.getObjectPosition(self.robotHandle, -1)
                    robotWorld = (pos[0], pos[1])
                except Exception:
                    self.cancelNavigation()
                    return
                
                self.statusLabel.setText('RE-CALCULATING PATH...')
                QtWidgets.QApplication.processEvents()
                
                self.waypoints = self._planPathVision(robotWorld, self.navTargetWorld)
                if not self.waypoints:
                    self.statusLabel.setText('RE-PLAN FAILED: UNREACHABLE LOCATION')
                    self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')
                    self.cancelNavigation()
                else:
                    self.statusLabel.setText('FOLLOWING RE-PLANNED PATH')
                    self.statusLabel.setStyleSheet('color: #00ff88; font-weight: bold; font-size: 11px;')
                return

        # -------------------------------------------------------------
        # 3. NORMAL WAYPOINT FOLLOWING
        # -------------------------------------------------------------
        if not self.waypoints:
            self.cancelNavigation()
            return

        heading = ori[2]
        self._recordPathPoint(pos[0], pos[1])

        while len(self.waypoints) > 1:
            tx, ty = self.waypoints[0]
            if math.hypot(tx - pos[0], ty - pos[1]) < 0.15: 
                self.waypoints.pop(0)
            else:
                break

        if not self.waypoints:
            self.cancelNavigation()
            return

        tx, ty = self.waypoints[0]
        dx = tx - pos[0]
        dy = ty - pos[1]
        distance = math.hypot(dx, dy)

        tolerance = 0.20 if len(self.waypoints) > 1 else 0.10
        if distance < tolerance:
            self.waypoints.pop(0)
            if not self.waypoints:
                self.drive(0, 0) 
                self.statusLabel.setText('TARGET REACHED ✓')
                self.statusLabel.setStyleSheet('color: #00ff88; font-weight: bold; font-size: 11px;')
                self.cancelNavigation()
                return
            else:
                tx, ty = self.waypoints[0]
                dx = tx - pos[0]
                dy = ty - pos[1]
                distance = math.hypot(dx, dy)

        desiredHeading = math.atan2(dy, dx)
        headingError = math.atan2(math.sin(desiredHeading - heading), math.cos(desiredHeading - heading))

        turn = self.navKAngular * headingError
        
        if abs(headingError) > 0.4:
            speed = 0.0
        else:
            speed = min(self.navMaxSpeed, self.navKLinear * distance)
            speed *= max(0.1, 1.0 - (abs(headingError) / 0.4))

        # -------------------------------------------------------------
        # 4. COLLISION DETECTION TRIGGER
        # -------------------------------------------------------------
        if self._checkImminentCollision() and dist_to_final_target > 0.45: 
            self.recoveryPhase = 1  
            self.recoveryTicks = 10 
            self.recoveryTurnSpeed = self._decideTurnDirection() * 0.785 
            self.waypoints = [] 
            self.drive(0, 0, autonomous=True) 
            return

        speedScale = self._computeAvoidance()
        speed *= speedScale 

        self.drive(speed, turn, autonomous=True)

    def _computeAvoidance(self):
        """Returns speedScale (Emergency braking only)"""
        if not (self.sim and self.scriptHandle and self.simRunning): return 1.0
        try:
            distances = self.sim.callScriptFunction('getSonarDistances', self.scriptHandle, [])
        except Exception: return 1.0

        if not distances: return 1.0

        closestDistance = self.avoidRadius
        
        for i in [1, 2, 3, 4, 5, 6]:
            if i < len(distances) and distances[i] < self.avoidRadius:
                closestDistance = min(closestDistance, distances[i])

        return max(0.0, closestDistance / self.avoidRadius)
    
    def _checkImminentCollision(self):
        """
        Determines if a collision is truly imminent based on graduated thresholds.
        """
        if not (self.sim and self.scriptHandle and self.simRunning): return False
        try:
            distances = self.sim.callScriptFunction('getSonarDistances', self.scriptHandle, [])
        except Exception: return False

        if not distances: return False
        
        # Front sensors (direct block)
        if min([distances[i] for i in [3, 4]] + [5.0]) < 0.15: return True
        
        # Diagonal sensors (catches corners and rack legs)
        if min([distances[i] for i in [1, 2]] + [5.0]) < 0.10: return True
        if min([distances[i] for i in [5, 6]] + [5.0]) < 0.10: return True
        
        # Pure side sensors (kept very low to survive narrow hallways)
        if min([distances[i] for i in [0, 15]] + [5.0]) < 0.04: return True
        if min([distances[i] for i in [7, 8]] + [5.0]) < 0.04: return True
            
        return False

    def _getFrontSonarDistance(self):
        """Returns the minimum distance strictly from the forward-facing sonars."""
        if not (self.sim and self.scriptHandle and self.simRunning): return 5.0
        try:
            distances = self.sim.callScriptFunction('getSonarDistances', self.scriptHandle, [])
        except Exception: return 5.0

        if not distances: return 5.0

        min_dist = 5.0
        for i in [3, 4]:
            if i < len(distances):
                min_dist = min(min_dist, distances[i])
        return min_dist

    def _getRearSonarDistance(self):
        """Returns the minimum distance strictly from the true rear-facing sonars (11, 12)."""
        if not (self.sim and self.scriptHandle and self.simRunning): return 5.0
        try:
            distances = self.sim.callScriptFunction('getSonarDistances', self.scriptHandle, [])
        except Exception: return 5.0

        if not distances: return 5.0

        min_dist = 5.0
        # FIXED: Only check the true rear sensors (11, 12) to avoid false positives from parallel side walls
        for i in [11, 12]:
            if i < len(distances):
                min_dist = min(min_dist, distances[i])
        return min_dist

    def _findObject(self, candidates):
        for path in candidates:
            try: return self.sim.getObject(path)
            except Exception: continue
        raise Exception(f'None of these paths resolved: {candidates}')

    def connectSim(self):
        self.statusLabel.setText('CONNECTING...')
        self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')
        QtWidgets.QApplication.processEvents()

        self.targetDummyHandle, self.visionHandle, self.topCamHandle = None, None, None
        self.robotHandle, self.scriptHandle = None, None

        try:
            client = RemoteAPIClient()
            self.sim = client.require('sim')
            self.visionHandle = self._findObject(['/PioneerP3DX/visionSensor', '/PioneerP3DX/visionSensor#0'])
            self.scriptHandle = self.sim.getScript(self.sim.scripttype_childscript, '/PioneerP3DX')
            self.robotHandle = self._findObject(['/PioneerP3DX', '/PioneerP3DX#0'])
            self._ensureDummyHandle()

            try:
                self.topCamHandle = self._findObject(['/topCamera', '/topCamera#0', '/PioneerP3DX/topCamera'])
                self.topCamView.setText('[ Top camera found — start simulation ]')
            except Exception: pass

            self.connected = True
            self.statusLabel.setText('CONNECTED ✓')
            self.statusLabel.setStyleSheet('color: #00ff88; font-weight: bold; font-size: 11px;')
        except Exception as e:
            self.statusLabel.setText(f'FAILED: {str(e)[:50]}')
            self.statusLabel.setStyleSheet('color: #ff4444; font-weight: bold; font-size: 11px;')

    def startSim(self):
        if self.sim and self.connected:
            self.sim.startSimulation()
            self.simRunning = True 
            self.targetDummyHandle = None
            self.robotPath, self.waypoints = [], []
            self.pathInfoLabel.setText('No path recorded yet')
            self.statusLabel.setText('SIMULATION RUNNING ▶')
            self.statusLabel.setStyleSheet('color: #00ff88; font-weight: bold; font-size: 11px;')
            self.camTimer.start()
        else: self.statusLabel.setText('Connect first!')

    def stopSim(self):
        if self.sim and self.connected:
            self.simRunning = False 
            self.camTimer.stop()
            self.cancelNavigation()
            self.drive(0, 0)
            self.sim.stopSimulation()
            self.targetDummyHandle = None
            self.statusLabel.setText('SIMULATION STOPPED ⏹')
            self.statusLabel.setStyleSheet('color: #ffaa00; font-weight: bold; font-size: 11px;')

    def drive(self, speed, turning, autonomous=False):
        if self.sim and self.scriptHandle and self.connected and self.simRunning:
            try:
                multiplier = 1 if autonomous else self.speedSlider.value()
                actual_speed = speed if autonomous else speed * multiplier / 2
                actual_turning = turning if autonomous else turning * multiplier / 2
                
                # GLOBAL SAFETY OVERRIDE: Stop physically reversing if obstacle detected
                if actual_speed < 0:
                    rear_dist = self._getRearSonarDistance()
                    if rear_dist < 0.15:
                        actual_speed = 0.0
                        
                self.sim.callScriptFunction('driveRobot', self.scriptHandle, [actual_speed, actual_turning, multiplier])
                self.speedLabel.setText(f'{actual_speed:.1f} m/s')
            except Exception: pass

    def refreshCameras(self):
        if not self.simRunning:
            return

        if self.sim and self.visionHandle and self.connected:
            try:
                img, [resX, resY] = self.sim.getVisionSensorImg(self.visionHandle)
                img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
                img = cv2.flip(img, 0)
                h, w, ch = img.shape
                qImg = QtGui.QImage(img.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
                self.cameraView.setPixmap(QtGui.QPixmap.fromImage(qImg).scaled(
                    self.cameraView.width(), self.cameraView.height(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
            except Exception: pass

        if self.sim and self.topCamHandle and self.connected:
            try:
                img, [resX, resY] = self.sim.getVisionSensorImg(self.topCamHandle)
                img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
                img = cv2.flip(img, 0)
                
                self.latestTopCamImg = img.copy() 

                h, w, ch = img.shape
                qImg = QtGui.QImage(img.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
                pixmap = QtGui.QPixmap.fromImage(qImg).scaled(
                    self.topCamView.width(), self.topCamView.height(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)

                pixW, pixH = pixmap.width(), pixmap.height()
                painter = QtGui.QPainter(pixmap)
                painter.setRenderHint(QtGui.QPainter.Antialiasing)

                if len(self.robotPath) >= 2:
                    pen = QtGui.QPen(QtGui.QColor('#00d4ff'))
                    pen.setWidth(2)
                    painter.setPen(pen)
                    points = [QtCore.QPointF(*self._worldToPixel(x, y, pixW, pixH)) for (x, y) in self.robotPath]
                    painter.drawPolyline(QtGui.QPolygonF(points))

                if len(self.waypoints) >= 2:
                    pen = QtGui.QPen(QtGui.QColor('#ffaa00'))
                    pen.setWidth(2)
                    pen.setStyle(QtCore.Qt.DashLine)
                    painter.setPen(pen)
                    planned_points = [QtCore.QPointF(*self._worldToPixel(x, y, pixW, pixH)) for (x, y) in self.waypoints]
                    painter.drawPolyline(QtGui.QPolygonF(planned_points))

                if self.navTargetWorld is not None:
                    mx, my = self._worldToPixel(self.navTargetWorld[0], self.navTargetWorld[1], pixW, pixH)
                    markerColor = QtGui.QColor('#ff3333') if self.navTarget else QtGui.QColor('#00ff88')
                    pen = QtGui.QPen(markerColor)
                    pen.setWidth(2)
                    painter.setPen(pen)
                    painter.setBrush(QtGui.QBrush(markerColor))
                    r = 5
                    painter.drawEllipse(QtCore.QPointF(mx, my), r, r)
                    painter.drawLine(QtCore.QPointF(mx - 8, my), QtCore.QPointF(mx + 8, my))
                    painter.drawLine(QtCore.QPointF(mx, my - 8), QtCore.QPointF(mx, my + 8))

                painter.end()
                self.topCamView.setPixmap(pixmap)
            except Exception: pass

    def saveSnapshot(self):
        if self.sim and self.visionHandle and self.connected and self.simRunning:
            try:
                img, [resX, resY] = self.sim.getVisionSensorImg(self.visionHandle)
                img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
                img = cv2.flip(img, 0)
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                cv2.imwrite('snapshot.png', img_bgr)
                self.statusLabel.setText('SNAPSHOT SAVED → snapshot.png ✓')
            except Exception as e:
                self.statusLabel.setText(f'Snapshot error: {str(e)[:40]}')

    def closeEvent(self, event):
        self.simRunning = False
        self.camTimer.stop()
        self.driveTimer.stop()
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = RobotTeleop()
    window.show()
    sys.exit(app.exec_())