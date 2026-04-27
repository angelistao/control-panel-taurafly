import sys
import cv2
import numpy as np
import urllib.request
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from mavros_msgs.msg import State, StatusText
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import os
import time
from datetime import datetime
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QPlainTextEdit, 
                             QFrame, QRadioButton, QButtonGroup)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QImage, QPixmap, QIcon

# --- CONFIGURAÇÃO --- 
NOME_DO_SISTEMA = "CONTROL PANEL" 
IP_BASE = "192.168.0" 
URL_ESP32_REAL = f'http://{IP_BASE}.159:81/stream' 
# --------------------

class TauraControlPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.path_logo = os.path.join(self.base_dir, "file.jpeg")
        self.setWindowTitle(f"{NOME_DO_SISTEMA} - TauraBots")
        self.setGeometry(50, 50, 1300, 900)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        
        # Estados iniciais
        self.mode = "HARDWARE" 
        self.stream_active = False
        self.ros_image = None
        self.last_mavros_msg_time = 0
        self.current_flight_mode = "---"
        self.is_armed = False 
        self.prearm_status = "WAITING" 
        self.bytes_data = bytes()
        self.stream = None
        self.pos_x = 0.0; self.pos_y = 0.0; self.pos_z = 0.0; self.alt_rel = 0.0

        # ROS2 Setup
        if not rclpy.ok(): rclpy.init()
        self.node = Node('gui_node')
        self.bridge = CvBridge()
        
        self.node.create_subscription(Image, '/drone/camera', self.callback_gazebo_image, 10)
        self.node.create_subscription(State, '/mavros/state', self.callback_mavros_state, 10)
        self.node.create_subscription(StatusText, '/mavros/statustext', self.callback_status_text, 10)
        self.node.create_subscription(PoseStamped, '/mavros/local_position/pose', self.callback_pose, 10)

        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters()
        )

        self.setup_ui()
        
        self.timer_video = QTimer(); self.timer_video.timeout.connect(self.update_frame); self.timer_video.start(30)
        self.timer_infra = QTimer(); self.timer_infra.timeout.connect(self.update_infrastructure); self.timer_infra.start(500)

    def setup_ui(self):
        if os.path.exists(self.path_logo):
            self.setWindowIcon(QIcon(self.path_logo))

        self.setStyleSheet("""
            QMainWindow { background-color: rgba(14, 14, 14, 180); }
            QWidget#centralWidget { background-color: rgba(14, 14, 14, 180); border-radius: 15px; border: 1px solid #333; }
            QLabel { color: white; font-family: 'Ubuntu'; background: transparent; }
            QPlainTextEdit { background-color: rgba(0, 0, 0, 220); color: #00ff41; font-family: 'Monospace'; font-size: 11px; border: 1px solid #444; }
            QPushButton { background-color: rgba(45, 45, 45, 200); color: white; border: 1px solid #555; border-radius: 6px; padding: 10px; font-weight: bold; }
            QPushButton:hover { background-color: #ff5500; border-color: #ff5500; }
            QRadioButton { color: white; font-weight: bold; }
            QFrame.panel_box { background-color: rgba(30, 30, 30, 160); border: 1px solid #444; border-radius: 8px; }
        """)

        self.central_widget = QWidget(); self.central_widget.setObjectName("centralWidget")
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)

        # Coluna de Vídeo e Logs
        self.left_col = QVBoxLayout()
        self.label_video = QLabel("AGUARDANDO SINAL..."); self.label_video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label_video.setStyleSheet("background-color: black; border: 1px solid #333; border-radius: 10px;")
        self.label_video.setMinimumSize(800, 500)
        self.console_logs = QPlainTextEdit(); self.console_logs.setReadOnly(True)
        self.left_col.addWidget(self.label_video, stretch=4)
        self.left_col.addWidget(self.console_logs, stretch=1)
        self.main_layout.addLayout(self.left_col, stretch=3)

        # Sidebar
        self.sidebar = QVBoxLayout()
        
        # Logo
        self.label_logo = QLabel()
        pix = QPixmap(self.path_logo)
        if not pix.isNull(): self.label_logo.setPixmap(pix.scaled(100, 100, Qt.AspectRatioMode.KeepAspectRatio))
        self.label_logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.sidebar.addWidget(self.label_logo)

        # --- SEÇÃO SUPERIOR: STATUS DO DRONE (MOVIDO PARA CIMA) ---
        self.label_status_drone = QLabel("MODO: ---\nDRONE: DISARMED")
        self.label_status_drone.setStyleSheet("padding: 10px; background-color: rgba(0, 0, 0, 150); border-left: 4px solid #ff5500; font-weight: bold;")
        self.sidebar.addWidget(self.label_status_drone)

        # Escolha do Modo
        mode_frame = QFrame(); mode_frame.setProperty("class", "panel_box")
        mode_lay = QVBoxLayout(mode_frame)
        self.radio_hw = QRadioButton("HARDWARE"); self.radio_hw.setChecked(True)
        self.radio_sim = QRadioButton("SIMULAÇÃO")
        self.btn_group = QButtonGroup(); self.btn_group.addButton(self.radio_hw); self.btn_group.addButton(self.radio_sim)
        
        # CONEXÃO DOS RADIOS
        self.radio_hw.toggled.connect(self.on_mode_toggled)
        self.radio_sim.toggled.connect(self.on_mode_toggled)
        
        mode_lay.addWidget(QLabel("Modo de Operação"))
        mode_lay.addWidget(self.radio_hw); mode_lay.addWidget(self.radio_sim)
        self.sidebar.addWidget(mode_frame)

        # Telemetria XYZ
        telemetry_frame = QFrame(); telemetry_frame.setProperty("class", "panel_box")
        tele_lay = QVBoxLayout(telemetry_frame)
        tele_lay.addWidget(QLabel("TELEMETRIA LOCAL (m)"))
        self.label_xyz = QLabel("X: 0.00 | Y: 0.00\nZ: 0.00"); self.label_xyz.setStyleSheet("font-family: 'Monospace'; color: #00d4ff;")
        self.label_alt = QLabel("ALT: 0.00 m"); self.label_alt.setStyleSheet("font-family: 'Monospace'; color: #00d4ff;")
        tele_lay.addWidget(self.label_xyz); tele_lay.addWidget(self.label_alt)
        self.sidebar.addWidget(telemetry_frame)

        # Infra MAVROS
        infra_frame = QFrame(); infra_frame.setProperty("class", "panel_box")
        infra_lay = QVBoxLayout(infra_frame)
        self.label_stat_1 = QLabel("MAVROS: ---")
        prearm_box = QHBoxLayout(); prearm_box.addWidget(QLabel("PRE-ARM:"))
        self.led_prearm = QLabel(); self.led_prearm.setFixedSize(16, 16); self.led_prearm.setStyleSheet("background-color: #333; border-radius: 8px;")
        prearm_box.addWidget(self.led_prearm); prearm_box.addStretch()
        infra_lay.addWidget(QLabel("STATUS DE VOO"))
        infra_lay.addWidget(self.label_stat_1); infra_lay.addLayout(prearm_box)
        self.sidebar.addWidget(infra_frame)

        self.btn_connect = QPushButton("CONECTAR VÍDEO"); self.btn_connect.clicked.connect(self.toggle_stream)
        self.sidebar.addWidget(self.btn_connect)
        
        self.sidebar.addStretch()
        self.main_layout.addLayout(self.sidebar, stretch=1)

    def on_mode_toggled(self):
        """Detecta qual rádio foi selecionado e altera o modo"""
        if self.radio_hw.isChecked():
            self.change_mode("HARDWARE")
        else:
            self.change_mode("SIM")

    def change_mode(self, mode):
        if self.stream_active: self.toggle_stream()
        self.mode = mode
        self.prearm_status = "WAITING"
        self.add_log("SISTEMA", f"MODO ALTERADO: {mode}")
        # Reset visual imediato
        if mode == "HARDWARE":
            self.label_status_drone.setText("MODO: HARDWARE\nSINAL: ESP32 ATIVA")
            self.label_stat_1.setText("MAVROS: ---")
            self.led_prearm.setStyleSheet("background-color: #333; border-radius: 8px;")

    def add_log(self, source, message):
        time_str = datetime.now().strftime("%H:%M:%S")
        self.console_logs.appendPlainText(f"[{time_str}] {source}: {message}")
        self.console_logs.verticalScrollBar().setValue(self.console_logs.verticalScrollBar().maximum())

    def callback_status_text(self, msg):
        text_up = msg.text.upper()
        if "PREARM" in text_up or "PRE-ARM" in text_up:
            if "GOOD" in text_up or "PASSED" in text_up:
                self.prearm_status = "GOOD"; self.add_log("ARDUPILOT", "PRE-ARM GOOD")
            else:
                self.prearm_status = "FAIL"; self.add_log("PREARM_ERR", msg.text)
        else:
            self.add_log("ARDUPILOT", msg.text)

    def callback_mavros_state(self, msg):
        self.last_mavros_msg_time = time.time()
        self.current_flight_mode = msg.mode
        self.is_armed = msg.armed

    def callback_gazebo_image(self, msg):
        if self.mode == "SIM": self.ros_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def callback_pose(self, msg):
        self.pos_x = msg.pose.position.x; self.pos_y = msg.pose.position.y; self.pos_z = msg.pose.position.z
        self.alt_rel = self.pos_z

    def update_infrastructure(self):
        try:
            if self.mode == "SIM":
                online = (time.time() - self.last_mavros_msg_time) < 5.0
                self.label_stat_1.setText(f"MAVROS: {'CONECTADO' if online else 'OFFLINE'}")
                self.label_stat_1.setStyleSheet(f"color: {'#00ff00' if online else '#ff4444'};")
                
                self.label_xyz.setText(f"X: {self.pos_x:.2f} | Y: {self.pos_y:.2f}\nZ: {self.pos_z:.2f}")
                self.label_alt.setText(f"ALTITUDE: {self.alt_rel:.2f} m")
                
                if not online:
                    self.led_prearm.setStyleSheet("background-color: #333; border-radius: 8px;")
                elif self.prearm_status == "GOOD":
                    self.led_prearm.setStyleSheet("background-color: #00ff00; border-radius: 8px; border: 1px solid white;")
                elif self.prearm_status == "FAIL":
                    self.led_prearm.setStyleSheet("background-color: #ff0000; border-radius: 8px; border: 1px solid white;")
                else:
                    self.led_prearm.setStyleSheet("background-color: #f1c40f; border-radius: 8px;")

                status = "ARMED" if self.is_armed else "DISARMED"
                self.label_status_drone.setText(f"DRONE: {status}\nMODO: {self.current_flight_mode}")
            else:
                # Mantém o texto fixo do modo Hardware se não estiver em SIM
                self.label_status_drone.setText("MODO: HARDWARE\nSINAL: ESP32 ATIVA")
                self.label_stat_1.setText("MAVROS: ---")
                self.led_prearm.setStyleSheet("background-color: #333; border-radius: 8px;")
        except: pass

    def toggle_stream(self):
        if not self.stream_active:
            if self.mode == "HARDWARE":
                try:
                    self.stream = urllib.request.urlopen(URL_ESP32_REAL, timeout=5)
                    self.bytes_data = bytes(); self.stream_active = True
                except Exception as e: self.add_log("ERRO", f"ESP32: {e}")
            else: self.stream_active = True
            self.btn_connect.setText("PARAR VÍDEO")
        else:
            self.stream_active = False; self.stream = None; self.btn_connect.setText("CONECTAR VÍDEO")

    def update_frame(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if not self.stream_active: return
        img = None
        try:
            if self.mode == "HARDWARE" and self.stream:
                self.bytes_data += self.stream.read(4096)
                a, b = self.bytes_data.find(b'\xff\xd8'), self.bytes_data.find(b'\xff\xd9')
                if a != -1 and b != -1:
                    jpg = self.bytes_data[a:b+2]; self.bytes_data = self.bytes_data[b+2:]
                    img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            elif self.mode == "SIM":
                img = self.ros_image

            if img is not None:
                img = cv2.resize(img, (640, 480))
                corners, ids, _ = self.detector.detectMarkers(img)
                if ids is not None: cv2.aruco.drawDetectedMarkers(img, corners, ids)
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                qimg = QImage(img_rgb.data, 640, 480, 640*3, QImage.Format.Format_RGB888)
                self.label_video.setPixmap(QPixmap.fromImage(qimg).scaled(self.label_video.size(), Qt.AspectRatioMode.KeepAspectRatio))
        except: pass

if __name__ == "__main__":
    app = QApplication(sys.argv); panel = TauraControlPanel(); panel.show(); sys.exit(app.exec())