import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QGridLayout, QMessageBox
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QTimer, pyqtSignal, QThread

COM_PORT = "COM10"
BAUD_RATE = 115200

class SerialReadThread(QThread):
    data_received = pyqtSignal(str)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self._running = True

    def run(self):
        while self._running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        data = self.ser.readline().decode(errors='ignore').strip()
                        if data:
                            self.data_received.emit(data)
                except Exception as e:
                    print("Serial read error:", e)
                    self._running = False
            self.msleep(10)  # sleep 10 ms

    def stop(self):
        self._running = False
        self.wait()


class PlotterGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.ser = None
        self.serial_thread = None
        self.initUI()
        self.connectSerial()

    def initUI(self):
        self.setWindowTitle("Futuristic Pen Plotter Control")
        self.setStyleSheet("background-color: #1e1e2f; color: white;")
        font = QFont("Consolas", 12)

        self.statusLabel = QLabel("Disconnected")
        self.statusLabel.setFont(font)
        self.statusLabel.setStyleSheet("color: red; font-weight: bold;")
        self.portLabel = QLabel(f"Port: {COM_PORT}")
        self.portLabel.setFont(font)

        topBar = QHBoxLayout()
        topBar.addWidget(self.portLabel)
        topBar.addStretch()
        topBar.addWidget(self.statusLabel)

        gamepadLayout = QGridLayout()
        gamepadLayout.setSpacing(15)

        self.addButton(gamepadLayout, "↖", 0, 0, lambda: self.sendCommand("DIAG_UL"))
        self.addButton(gamepadLayout, "↑", 0, 1, lambda: self.sendCommand("Y+"))
        self.addButton(gamepadLayout, "↗", 0, 2, lambda: self.sendCommand("DIAG_UR"))

        self.addButton(gamepadLayout, "←", 1, 0, lambda: self.sendCommand("X-"))
        self.addButton(gamepadLayout, "→", 1, 2, lambda: self.sendCommand("X+"))

        self.addButton(gamepadLayout, "↙", 2, 0, lambda: self.sendCommand("DIAG_DL"))
        self.addButton(gamepadLayout, "↓", 2, 1, lambda: self.sendCommand("Y-"))
        self.addButton(gamepadLayout, "↘", 2, 2, lambda: self.sendCommand("DIAG_DR"))

        servoLayout = QVBoxLayout()
        servoUpBtn = QPushButton("Pen Up")
        servoDownBtn = QPushButton("Pen Down")
        self.styleButton(servoUpBtn, "#0055ff", big=True)
        self.styleButton(servoDownBtn, "#ff5500", big=True)
        servoUpBtn.clicked.connect(lambda: self.sendCommand("PEN_UP"))
        servoDownBtn.clicked.connect(lambda: self.sendCommand("PEN_DOWN"))
        servoLayout.addWidget(servoUpBtn)
        servoLayout.addWidget(servoDownBtn)
        servoLayout.setSpacing(25)

        leftSide = QVBoxLayout()
        leftSide.addStretch()
        leftSide.addLayout(gamepadLayout)
        leftSide.addSpacing(30)
        leftSide.addLayout(servoLayout)
        leftSide.addStretch()
        leftSide.setContentsMargins(400, 0, 100, 0)

        rightLayout = QVBoxLayout()
        nikolausBtn = QPushButton("Nikolaus House")
        homeBtn = QPushButton("Homing")
        emergencyBtn = QPushButton("Emergency Stop")
        nameBtn = QPushButton("Name")

        self.styleButton(nikolausBtn, "#00cc88", big=True)
        self.styleButton(homeBtn, "#ffaa00", big=True)
        self.styleButton(emergencyBtn, "#cc0000", big=True)
        self.styleButton(nameBtn, "#8888ff", big=True)

        nikolausBtn.clicked.connect(lambda: self.sendCommand("NIKOLAUS"))
        homeBtn.clicked.connect(lambda: self.sendCommand("HOME"))
        emergencyBtn.clicked.connect(lambda: self.sendCommand("EMERGENCY"))
        nameBtn.clicked.connect(lambda: self.sendCommand("NAME"))

        rightLayout.addStretch()
        rightLayout.addWidget(nikolausBtn)
        rightLayout.addWidget(homeBtn)
        rightLayout.addWidget(emergencyBtn)
        rightLayout.addWidget(nameBtn)
        rightLayout.setSpacing(25)
        rightLayout.addStretch()
        rightLayout.setContentsMargins(100, 0, 400, 0)

        mainLayout = QVBoxLayout()
        mainLayout.addLayout(topBar)

        midLayout = QHBoxLayout()
        midLayout.addLayout(leftSide)
        midLayout.addStretch()
        midLayout.addLayout(rightLayout)

        mainLayout.addLayout(midLayout)
        self.setLayout(mainLayout)
        self.resize(800, 500)

    def styleButton(self, btn, color, big=False):
        btn.setFont(QFont("Consolas", 14 if big else 12, QFont.Bold))
        btn.setMinimumSize(120, 60 if big else 50)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                padding: 10px;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {color}AA;
            }}
        """)

    def addButton(self, layout, text, row, col, callback):
        btn = QPushButton(text)
        self.styleButton(btn, "#4444aa", big=True)
        btn.clicked.connect(callback)
        layout.addWidget(btn, row, col)

    def connectSerial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
            self.statusLabel.setText("Connected")
            self.statusLabel.setStyleSheet("color: green; font-weight: bold;")
            print(f"Connected to {COM_PORT}")

            # Start serial reading thread
            if self.serial_thread:
                self.serial_thread.stop()
            self.serial_thread = SerialReadThread(self.ser)
            self.serial_thread.data_received.connect(self.onSerialData)
            self.serial_thread.start()
        except Exception as e:
            self.ser = None
            self.statusLabel.setText("Disconnected")
            self.statusLabel.setStyleSheet("color: red; font-weight: bold;")
            print("Serial connection failed:", e)

    def onSerialData(self, data):
        # Debug print incoming serial data from ESP32
        print(f"Received: {data}")

    def sendCommand(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                full_cmd = cmd + "\n"
                self.ser.write(full_cmd.encode())
                self.ser.flush()  # Force send immediately
                print(f"Sent: {cmd}")
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Failed to send command: {e}")
        else:
            QMessageBox.warning(self, "Error", "ESP32 not connected!")

    def closeEvent(self, event):
        # Cleanup on exit
        if self.serial_thread:
            self.serial_thread.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PlotterGUI()
    window.show()
    sys.exit(app.exec_())
