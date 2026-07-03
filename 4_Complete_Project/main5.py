import math

import sys
import time
import threading

from PyQt5 import QtCore, QtWidgets, QtGui
import cv2
    
from gcode_manager2 import GcodeManager
from vition_manager3 import VitionManager
from espManager2 import ESPManager

PORT          = "COM9"
BAUD          = 115200

PORT_ESP      = "COM13"
BAUD_ESP      = 115200

SCALE_MM      = (139.3, 142.8, 141)
MODEL_PATH    = "Modelito_v11_best.pt"
CAM_INDEX     = 1
SLEEP_LOOP_S  = 0.05

WORKSPACE     = (27.6, 11)
CORNER        = (WORKSPACE[0], 0)

ARUCO_CARD_POS =  {0: [28+1.85, -11.5], 1: [1.85, -11.5+24], 2: [28+1.85, -11.5+24], 3: [1.85, -11.5]}


class WorkspaceView(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.egg_pos = None
        self.cards_pos = []   # List of detected card positions (x, y)
        self.robot_pos = []  # Robot position in cm, updated continuously
        self.setMinimumSize(300, 200)
        self.posEgg = None

    @QtCore.pyqtSlot(object)
    def set_egg_position(self, pos):
        self.egg_pos = pos
        self.update()

    def set_cards_positions(self, cards):
        self.cards_pos = cards
        self.update()

    def set_robot_position(self, pos):
        self.robot_pos = pos
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor(240, 240, 240))

        width = self.width()
        height = self.height()

        # Draw the border of the graphical representation area
        painter.setPen(QtGui.QPen(QtCore.Qt.black, 2))
        painter.drawRect(0, 0, width, height)

        # Draw green border for workspace
        factor = 0.3
        marginW = int(width*factor)
        marginH = int(height*factor)
        painter.setPen(QtGui.QPen(QtCore.Qt.green, 2))
        painter.drawRect(marginW, marginH, width - marginW * 2, height - marginH * 2)

        scale_x = (width - marginW * 2) / WORKSPACE[0]
        scale_y = (height - marginH * 2) / WORKSPACE[1]

        # Function to convert cm coordinates to pixels (Y is inverted so origin is at bottom)
        def to_canvas_coords(x_cm, y_cm):
            cx = int(x_cm * scale_x + marginW)
            cy = int(height - marginH - y_cm * scale_y)
            return cx, cy
        
        # Draw point at workspace origin using to_canvas_coords
        p = (0,0)
        painter.setPen(QtGui.QPen(QtCore.Qt.red, 2))
        painter.drawEllipse(QtCore.QPointF(*to_canvas_coords(*p)), 5, 5)

        # Draw all ARUCO_CARD_POS cards
        painter.setPen(QtGui.QPen(QtCore.Qt.blue, 2))
        for idx, (x_exp, y_exp) in enumerate(ARUCO_CARD_POS.values()):
            cx, cy = to_canvas_coords(x_exp, y_exp)
            painter.drawRect(cx - 5, cy - 5, 10, 10)
            painter.drawText(cx + 6, cy + 3, f"A_{idx} ({x_exp:.1f},{y_exp:.1f})")      
            
        # Paint detected ArUco cards in green and undetected ones in red
        painter.setBrush(QtGui.QBrush(QtCore.Qt.green))
        painter.setPen(QtGui.QPen(QtCore.Qt.darkGreen))
        size = 10  # square size
        card_det = self.cards_pos
        for idx, (x_exp, y_exp) in enumerate(ARUCO_CARD_POS.values()):
            cx, cy = to_canvas_coords(x_exp, y_exp)
            # Paint detected cards in green
            if idx in card_det: 
                painter.setBrush(QtGui.QBrush(QtCore.Qt.green))
                painter.setPen(QtGui.QPen(QtCore.Qt.darkGreen))
            # Paint undetected cards in red
            else: 
                painter.setBrush(QtGui.QBrush(QtCore.Qt.red))
                painter.setPen(QtGui.QPen(QtCore.Qt.red))
            painter.drawRect(int(cx - size/2), int(cy - size/2), int(size), int(size))

        # Draw the egg (if it exists)
        if self.egg_pos:
            cx, cy = to_canvas_coords(*self.egg_pos)
            painter.setBrush(QtGui.QBrush(QtCore.Qt.red))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(QtCore.QPointF(cx, cy), 8, 8)

        # Draw robot position as a yellow triangle
        if self.robot_pos:
            x,y = self.robot_pos
            cx, cy = to_canvas_coords(x, y)
            painter.setBrush(QtGui.QBrush(QtCore.Qt.yellow))
            painter.setPen(QtGui.QPen(QtCore.Qt.darkYellow))

        # Triangle pointing upward
        points = [
            QtCore.QPointF(cx, cy - 10),
            QtCore.QPointF(cx - 8, cy + 8),
            QtCore.QPointF(cx + 8, cy + 8),
        ]
        painter.drawPolygon(QtGui.QPolygonF(points))

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Egg Robot Control")
        self.resize(1024, 768)

        self.esp = ESPManager(port=PORT_ESP, baud=BAUD_ESP)
        self.gm = GcodeManager(PORT, BAUD, SCALE_MM)
        self.vm = VitionManager(
            MODEL_PATH,
            (0, 0, WORKSPACE[0], WORKSPACE[1]),
            CAM_INDEX
        )

        self.video_label = QtWidgets.QLabel(); self.video_label.setFixedSize(640, 480)
        self.workspace = WorkspaceView()
        self.workspace.setFixedSize(720, 480)

        self.run_btn     = QtWidgets.QPushButton("Run")
        self.stop_btn    = QtWidgets.QPushButton("Stop")
        self.terminal    = QtWidgets.QTextEdit(); self.terminal.setReadOnly(True)

        top = QtWidgets.QHBoxLayout(); top.addWidget(self.video_label); top.addWidget(self.workspace)
        btn = QtWidgets.QHBoxLayout(); btn.addWidget(self.run_btn); btn.addWidget(self.stop_btn)
        main= QtWidgets.QVBoxLayout(); main.addLayout(top); main.addLayout(btn); main.addWidget(self.terminal)
        w = QtWidgets.QWidget(); w.setLayout(main); self.setCentralWidget(w)

        self.run_btn.clicked.connect(self.start_all)
        self.stop_btn.clicked.connect(self.stop_all)

        self.v_timer = QtCore.QTimer()
        self.v_timer.timeout.connect(self.update_frame)
        self.v_timer.start(50)

        self.running = False

    def log(self, msg):
        QtCore.QMetaObject.invokeMethod(self.terminal, "append",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, msg))


    def start_all(self):
        if self.running: return
        self.log("Starting vision...")
        self.vm.start()
        self.log("Vision started.")
        self.log("Connecting robot...")
        self.log("Connecting to ARDUINO...")
        self.gm.start()
        self.log("Connection with ARDUINO established.")
        self.log("Calibrating robot...")
        self.gm.calibrate()
        self.log("Robot calibrated.")

        self.log("Connecting to ESP32...")
        self.esp.start()
        self.esp.turn_off()
        self.log("Connection with ESP32 established.")

        self.running = True
        threading.Thread(target=self.control_loop, daemon=True).start()

    def stop_all(self):
        if not self.running: return
        self.vm.stop()
        self.gm.stop()         # Set flag so update() terminates
        self.running = False
        self.log("Everything stopped.")
        self.workspace.set_egg_position(None)

    def update_frame(self):
            # Update vision
            self.vm.update()

            # Update robot state (read serial) ONLY if running
            if self.running:
                self.gm.update()

            # Update ESP32 state
            self.esp.update()

            # Display camera frame
            frame = self.vm.get_last_frame()
            if frame is None:
                return
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            qimg = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888)
            self.video_label.setPixmap(QtGui.QPixmap.fromImage(qimg))

            # Get all detected card positions (must be implemented in VitionManager)
            cards_pos = self.vm.get_all_cards_positions()  # Expects list of (x,y) in cm
            self.workspace.set_cards_positions(cards_pos)

            # Get egg position and update visualization
            if self.vm.is_egg_in_target():
                xr,yr = self.gm.get_mm_position()
                xr = xr/10
                yr = yr/10
                egg_pos = self.vm.get_egg_position(xr,yr)
                self.workspace.set_egg_position(egg_pos)
            else:
                self.workspace.set_egg_position(None)

            # Robot position (can be fixed or dynamic; here set to CORNER)
            # self.workspace.set_robot_position(CORNER)

            # Robot position in cm
            robot_pos = self.gm.get_mm_position()
            # Divide all components by 10
            robot_pos = [x/10 for x in robot_pos]
            if robot_pos != self.workspace.robot_pos:
                self.workspace.set_robot_position(robot_pos)

    def update_egg_position(self, pos):
        QtCore.QMetaObject.invokeMethod(
            self.workspace, "set_egg_position",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(object, pos)
        )


    def control_loop(self):
        while self.running:
            # If egg is not detected, skip
            if not self.vm.is_egg_in_target():
            #if not self.vm.is_targets():
                self.workspace.set_egg_position(None)
                time.sleep(SLEEP_LOOP_S)
                continue

            # Egg detection inside the card
            xr,yr = self.gm.get_mm_position()
            xr = xr/10
            yr = yr/10
            pos = self.vm.get_egg_position(xr,yr)

            print(pos)
            self.update_egg_position(pos)

            if pos != -1:
                self.posEgg =pos


            self.log(f"Egg at {self.posEgg [0]:.1f},{self.posEgg [1]:.1f} cm")
            ex,ey = self.posEgg 
            print(self.posEgg )
            print(xr)
            print(yr)
            if ((xr-ex)**2+(yr-ey)**2) < 1.5**2:

                # Move robot in Z (lower gripper)
                self.gm.moveZ_cm(-7.8)
                while self.gm.isMoving(): time.sleep(SLEEP_LOOP_S)

                # Grab
                self.esp.turn_on()
                time.sleep(1.5)  # Wait for the mechanism to grab the egg (adjust as needed)

                # Move robot in Z (raise gripper)
                self.gm.moveZ_cm(0)
                while self.gm.isMoving(): time.sleep(SLEEP_LOOP_S)

                    # Move robot to origin
                self.gm.move_cm(*CORNER)
                while self.gm.isMoving(): time.sleep(SLEEP_LOOP_S)

                # Move robot in Z (lower gripper)
                self.gm.moveZ_cm(-7.8)
                while self.gm.isMoving(): time.sleep(SLEEP_LOOP_S)

                # Release
                self.esp.turn_off()
                time.sleep(1.5)  # Wait for the mechanism to release the egg (adjust as needed)

                # Move robot in Z (raise gripper)
                self.gm.moveZ_cm(0)
                while self.gm.isMoving(): time.sleep(SLEEP_LOOP_S)       

            
            if self.gm.isMoving():
                time.sleep(SLEEP_LOOP_S/50)

                if ((ex_old-ex)**2+(ey_old-ey)**2) > 1**2:
                    ex_old,ey_old = ex,ey
                    self.gm.move_cm(ex, ey)
                    continue             
                
            else:
                ex_old,ey_old = ex,ey
                self.gm.move_cm(ex, ey)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())