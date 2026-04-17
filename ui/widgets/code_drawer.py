from PyQt5 import QtWidgets, QtCore, QtGui
import os
import subprocess
import threading
import time

class CodeDrawer(QtWidgets.QWidget):
    upload_status_signal = QtCore.pyqtSignal(str, bool) # status message, is_error

    def __init__(self, main_window):
        super().__init__(main_window)
        self.mw = main_window
        
        # UI Setup
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.layout.setSpacing(10)
        
        self.setStyleSheet("""
            QWidget {
                background-color: #1e1e1e;
                border-left: 1px solid #333;
            }
            QLabel {
                color: #4a90e2;
                font-weight: bold;
            }
        """)
        
        # Header
        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("ESP32 CODE GENERATOR")
        title.setStyleSheet("font-size: 13px;")
        header.addWidget(title)
        
        self.close_btn = QtWidgets.QPushButton("✕")
        self.close_btn.setFixedSize(24, 24)
        self.close_btn.setToolTip("Close Code Panel")
        self.close_btn.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                color: #888;
                border: none;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                color: #ff5555;
            }
        """)
        self.close_btn.clicked.connect(self.hide_panel)
        header.addWidget(self.close_btn)
        self.layout.addLayout(header)
        
        # Code Editor
        self.code_edit = QtWidgets.QPlainTextEdit()
        self.code_edit.setReadOnly(True)
        self.code_edit.setFont(QtGui.QFont("Consolas", 9))
        self.code_edit.setStyleSheet("""
            QPlainTextEdit {
                background-color: #121212;
                color: #a9b7c6;
                border: 1px solid #333;
                border-radius: 4px;
            }
        """)
        self.layout.addWidget(self.code_edit)
        
        # Action Buttons Layout
        btn_layout = QtWidgets.QHBoxLayout()
        
        # Copy Button
        self.copy_btn = QtWidgets.QPushButton("📋 COPY")
        self.copy_btn.setToolTip("Copy code to clipboard")
        self.copy_btn.setStyleSheet("""
            QPushButton {
                background-color: #e0e0e0;
                color: black;
                font-weight: bold;
                padding: 10px;
                border-radius: 8px;
                border: 1px solid #bbb;
            }
            QPushButton:hover {
                background-color: #4d4d4d;
            }
        """)
        self.copy_btn.clicked.connect(self.copy_code)
        btn_layout.addWidget(self.copy_btn)
        
        # Upload Button
        self.upload_btn = QtWidgets.QPushButton("🚀 UPLOAD")
        self.upload_btn.setToolTip("Compile and Upload to ESP32")
        self.upload_btn.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: black;
                font-weight: bold;
                padding: 10px;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #2ecc71;
            }
            QPushButton:disabled {
                background-color: #2c3e50;
                color: #7f8c8d;
            }
        """)
        self.upload_btn.clicked.connect(self.upload_code)
        btn_layout.addWidget(self.upload_btn)
        
        self.layout.addLayout(btn_layout)
        
        # Status Label
        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #888; font-size: 10px; font-weight: normal;")
        self.layout.addWidget(self.status_label)
        
        self.upload_status_signal.connect(self.on_upload_status)
        
        self.hide() # Hidden by default

    def set_code(self, code):
        self.code_edit.setPlainText(code)

    def open_drawer(self):
        """Shows the panel in the main layout."""
        self.show()
        # MainWindow will handle splitter sizes

    def hide_panel(self):
        """Hides the panel and informs MainWindow."""
        self.hide()
        if hasattr(self.mw, 'main_splitter'):
            # Shrink splitter to hide this section
            self.mw.main_splitter.setSizes([350, 850, 0])

    def copy_code(self):
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(self.code_edit.toPlainText())
        self.copy_btn.setText("✓ COPIED")
        QtCore.QTimer.singleShot(2000, lambda: self.copy_btn.setText("📋 COPY"))

    def upload_code(self):
        """Handles the upload logic using arduino-cli."""
        if not self.mw.serial_mgr.is_connected and not self.mw.serial_mgr.port_name:
            self.mw.log("⚠️ ERROR: No COM Port selected! Select a port first.")
            self.on_upload_status("Select COM port first!", True)
            return

        port = self.mw.serial_mgr.port_name or self.mw.port_combo.currentText()
        if port == "No Ports found":
            self.on_upload_status("No COM port detected!", True)
            return

        self.upload_btn.setEnabled(False)
        self.status_label.setText("Preparing Upload...")
        self.mw.log(f"🚀 Starting Firmware Upload to {port}...")

        # Run in thread to keep UI alive
        thread = threading.Thread(target=self._run_upload_process, args=(port,), daemon=True)
        thread.start()

    def _run_upload_process(self, port):
        """Background compilation and upload."""
        arduino_cli = r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
        
        if not os.path.exists(arduino_cli):
            self.upload_status_signal.emit("arduino-cli not found!", True)
            return

        # 1. Create a temporary sketch directory
        sketch_dir = os.path.join(os.getcwd(), "firmware", "torotron_esp32")
        os.makedirs(sketch_dir, exist_ok=True)
        ino_file = os.path.join(sketch_dir, "torotron_esp32.ino")
        
        with open(ino_file, "w") as f:
            f.write(self.code_edit.toPlainText())

        # 2. Release Serial Port so arduino-cli can use it
        was_connected = self.mw.serial_mgr.is_connected
        if was_connected:
            self.upload_status_signal.emit("Releasing Serial Port...", False)
            self.mw.serial_mgr.disconnect()
            # Increase wait to 3s to ensure Windows releases the handle
            time.sleep(3) 

        try:
            # 3. Compile & Upload
            self.upload_status_signal.emit("Compiling & Uploading (May take 30s)...", False)
            
            # Extract raw port (e.g., "COM6" from "COM6 (USB-Serial...)")
            raw_port = port.split(" ")[0]
            
            # Use shell=True for windows to handle potential path issues
            # Ensure the port is quoted in case of any weirdness
            cmd = [
                f'"{arduino_cli}"', "compile", "--upload",
                "-p", f'"{raw_port}"',
                "--fqbn", "esp32:esp32:esp32",
                f'"{sketch_dir}"'
            ]
            
            full_cmd = " ".join(cmd)
            self.mw.log(f"Executing: {full_cmd}")
            
            process = subprocess.Popen(full_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)
            stdout, stderr = process.communicate()

            if process.returncode == 0:
                self.upload_status_signal.emit("✅ UPLOAD SUCCESSFUL!", False)
            else:
                self.upload_status_signal.emit(f"❌ ERROR: {stderr[:100]}...", True)
                print(f"UPLOAD FAIL:\n{stderr}")

        except Exception as e:
            self.upload_status_signal.emit(f"Process Error: {str(e)}", True)

        finally:
            # 4. Reconnect Serial Port
            if was_connected:
                # Wait for ESP32 to finish rebooting (increase to 4s)
                time.sleep(4) 
                self.upload_status_signal.emit("Reconnecting serial...", False)
                self.mw.serial_mgr.connect(port)
            
            self.upload_status_signal.emit("Ready.", False)

    def on_upload_status(self, msg, is_error):
        self.status_label.setText(msg)
        if is_error:
            self.status_label.setStyleSheet("color: #ff5555; font-size: 10px;")
            self.mw.log(f"❌ Upload Failed: {msg}")
        else:
            self.status_label.setStyleSheet("color: #4a90e2; font-size: 10px;")
            if "SUCCESS" in msg:
                self.mw.log("✅ Firmware Upload Complete!")
        
        if "Ready" in msg:
            self.upload_btn.setEnabled(True)
