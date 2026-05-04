from PyQt5 import QtWidgets, QtCore, QtGui
import time
import os
import re
import numpy as np


class RobotSyntaxHighlighter(QtGui.QSyntaxHighlighter):
    """Syntax highlighter for robot programming languages (Command, Python, Matlab)."""

    def __init__(self, document, lang="command"):
        super().__init__(document)
        self.lang = lang
        self._build_rules()

    def set_language(self, lang):
        self.lang = lang
        self._build_rules()
        self.rehighlight()

    def _build_rules(self):
        self.rules = []

        # --- FORMATS ---
        keyword_fmt = QtGui.QTextCharFormat()
        keyword_fmt.setForeground(QtGui.QColor("#1976d2"))
        keyword_fmt.setFontWeight(QtGui.QFont.Bold)

        builtin_fmt = QtGui.QTextCharFormat()
        builtin_fmt.setForeground(QtGui.QColor("#1565c0"))
        builtin_fmt.setFontWeight(QtGui.QFont.Bold)

        number_fmt = QtGui.QTextCharFormat()
        number_fmt.setForeground(QtGui.QColor("#0d47a1"))

        string_fmt = QtGui.QTextCharFormat()
        string_fmt.setForeground(QtGui.QColor("#00796b"))

        comment_fmt = QtGui.QTextCharFormat()
        comment_fmt.setForeground(QtGui.QColor("#9e9e9e"))
        comment_fmt.setFontItalic(True)

        func_fmt = QtGui.QTextCharFormat()
        func_fmt.setForeground(QtGui.QColor("#0d47a1"))

        if self.lang == "command":
            # Robot command keywords
            for kw in [r'\bJOINT\b', r'\bWAIT\b', r'\bMOVE\b', r'\bSPEED\b', r'\bHOME\b', r'\bLOOP\b']:
                self.rules.append((re.compile(kw, re.IGNORECASE), keyword_fmt))
            # Comments
            self.rules.append((re.compile(r'#.*$', re.MULTILINE), comment_fmt))

        elif self.lang == "python":
            # Python keywords
            py_keywords = [
                r'\bdef\b', r'\bclass\b', r'\bimport\b', r'\bfrom\b', r'\breturn\b',
                r'\bif\b', r'\belif\b', r'\belse\b', r'\bfor\b', r'\bwhile\b',
                r'\bin\b', r'\bnot\b', r'\band\b', r'\bor\b', r'\bTrue\b',
                r'\bFalse\b', r'\bNone\b', r'\btry\b', r'\bexcept\b', r'\bwith\b',
                r'\bas\b', r'\blambda\b', r'\byield\b', r'\bpass\b', r'\bbreak\b',
                r'\bcontinue\b', r'\braise\b', r'\bglobal\b', r'\bnonlocal\b',
                r'\bassert\b', r'\bdel\b', r'\bimport\b', r'\bwith\b'
            ]
            for kw in py_keywords:
                self.rules.append((re.compile(kw), keyword_fmt))
            # Builtins & Robot API
            for bi in [r'\bprint\b', r'\brange\b', r'\blen\b', r'\bint\b', r'\bfloat\b', r'\bstr\b',
                       r'\brobot\.move\b', r'\brobot\.move_xyz\b', r'\brobot\.wait\b', 
                       r'\brobot\.home\b', r'\brobot\.get_joint\b', r'\brobot\.get_tcp\b',
                       r'\brobot\.set_speed\b', r'\brobot\.log\b']:
                self.rules.append((re.compile(bi), builtin_fmt))
            # Function calls
            self.rules.append((re.compile(r'\b[a-zA-Z_]\w*(?=\s*\()'), func_fmt))
            # Strings
            self.rules.append((re.compile(r"'[^']*'"), string_fmt))
            self.rules.append((re.compile(r'"[^"]*"'), string_fmt))
            # Comments
            self.rules.append((re.compile(r'#.*$', re.MULTILINE), comment_fmt))

        elif self.lang == "matlab":
            # Matlab keywords
            for kw in [r'\bfunction\b', r'\bend\b', r'\bif\b', r'\belse\b', r'\bfor\b',
                        r'\bwhile\b', r'\breturn\b', r'\bpause\b', r'\bglobal\b', r'\bpersistent\b']:
                self.rules.append((re.compile(kw, re.IGNORECASE), keyword_fmt))
            # Builtin commands
            for bi in [r'\bjoint\b', r'\bmove_xyz\b', r'\bhome\b', r'\bset_speed\b', r'\bclc\b']:
                self.rules.append((re.compile(bi, re.IGNORECASE), builtin_fmt))
            # Strings
            self.rules.append((re.compile(r"'[^']*'"), string_fmt))
            # Comments
            self.rules.append((re.compile(r'%.*$', re.MULTILINE), comment_fmt))

        # Numbers (universal)
        self.rules.append((re.compile(r'\b-?\d+\.?\d*\b'), number_fmt))

    def highlightBlock(self, text):
        for pattern, fmt in self.rules:
            for match in pattern.finditer(text):
                start = match.start()
                length = match.end() - start
                self.setFormat(start, length, fmt)


class LineNumberArea(QtWidgets.QWidget):
    """Line number gutter for the code editor."""

    def __init__(self, editor):
        super().__init__(editor)
        self.editor = editor

    def sizeHint(self):
        return QtCore.QSize(self.editor.line_number_area_width(), 0)

    def paintEvent(self, event):
        self.editor.line_number_area_paint_event(event)


class CodeEditor(QtWidgets.QPlainTextEdit):
    """Professional code editor with line numbers and current-line highlight."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.line_number_area = LineNumberArea(self)

        self.blockCountChanged.connect(self.update_line_number_area_width)
        self.updateRequest.connect(self.update_line_number_area)
        self.cursorPositionChanged.connect(self.highlight_current_line)

        self.update_line_number_area_width(0)
        self.highlight_current_line()

        # Editor font
        font = QtGui.QFont("Consolas", 11)
        font.setStyleHint(QtGui.QFont.Monospace)
        self.setFont(font)

        # Tab width
        metrics = QtGui.QFontMetrics(font)
        self.setTabStopDistance(4 * metrics.horizontalAdvance(' '))

        # Editor style
        self.setStyleSheet("""
            QPlainTextEdit {
                background-color: #fafafa;
                color: #212121;
                border: 1px solid #e0e0e0;
                selection-background-color: #bbdefb;
                selection-color: #212121;
                padding-left: 5px;
            }
        """)

    def line_number_area_width(self):
        digits = max(1, len(str(self.blockCount())))
        space = 10 + self.fontMetrics().horizontalAdvance('9') * digits
        return space

    def update_line_number_area_width(self, _):
        self.setViewportMargins(self.line_number_area_width(), 0, 0, 0)

    def update_line_number_area(self, rect, dy):
        if dy:
            self.line_number_area.scroll(0, dy)
        else:
            self.line_number_area.update(0, rect.y(), self.line_number_area.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self.update_line_number_area_width(0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self.line_number_area.setGeometry(
            QtCore.QRect(cr.left(), cr.top(), self.line_number_area_width(), cr.height())
        )

    def line_number_area_paint_event(self, event):
        painter = QtGui.QPainter(self.line_number_area)
        painter.fillRect(event.rect(), QtGui.QColor("#f5f5f5"))

        block = self.firstVisibleBlock()
        block_number = block.blockNumber()
        top = round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top())
        bottom = top + round(self.blockBoundingRect(block).height())

        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                number = str(block_number + 1)
                painter.setPen(QtGui.QColor("#bdbdbd"))
                painter.setFont(self.font())
                painter.drawText(
                    0, top,
                    self.line_number_area.width() - 5,
                    self.fontMetrics().height(),
                    QtCore.Qt.AlignRight, number
                )
            block = block.next()
            top = bottom
            bottom = top + round(self.blockBoundingRect(block).height())
            block_number += 1

        painter.end()

    def highlight_current_line(self):
        extra_selections = []
        if not self.isReadOnly():
            selection = QtWidgets.QTextEdit.ExtraSelection()
            line_color = QtGui.QColor("#e3f2fd")
            selection.format.setBackground(line_color)
            selection.format.setProperty(QtGui.QTextFormat.FullWidthSelection, True)
            selection.cursor = self.textCursor()
            selection.cursor.clearSelection()
            extra_selections.append(selection)
        self.setExtraSelections(extra_selections)


class ProgramPanel(QtWidgets.QWidget):
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.is_running = False
        self.current_lang = "command"  # Default language

        # Example templates for each language
        self.templates = {
            "command": (
                "# ROBOT COMMAND SEQUENCE\n"
                "SPEED 60          # Set speed to 60%\n"
                "HOME              # Move to home position\n"
                "WAIT 1.0          # Wait 1s\n"
                "\n"
                "# Move to Cartesian Target\n"
                "MOVE 20.0 15.0 10.0\n"
                "WAIT 0.5\n"
                "\n"
                "# Direct Joint Control\n"
                "JOINT Shoulder 45\n"
                "JOINT Elbow -30\n"
                "WAIT 1.0\n"
            ),
            "python": (
                "# ============================================================\n"
                "# E-labs Python Robot API - Professional Scripting\n"
                "# Use 'robot' object to control the 3D simulation.\n"
                "# \n"
                "# MOVEMENT:\n"
                "#   robot.move('JointName', angle)  - Smooth joint rotation\n"
                "#   robot.move_xyz(x, y, z)         - Smooth Cartesian movement (IK)\n"
                "#   robot.home()                    - Reset to zero position\n"
                "#   robot.wait(seconds)             - Non-blocking delay\n"
                "# \n"
                "# QUERY STATE:\n"
                "#   val = robot.get_joint('Name')   - Get current angle\n"
                "#   pos = robot.get_tcp()           - Get end-effector [x,y,z]\n"
                "#   names = robot.get_joint_names() - List of all joints\n"
                "# \n"
                "# UTILS:\n"
                "#   robot.set_speed(%)              - Set animation speed\n"
                "#   robot.log('message')            - Print to system console\n"
                "# ============================================================\n"
                "import math\n"
                "\n"
                "robot.set_speed(80)\n"
                "robot.log('Starting pick-and-place sequence...')\n"
                "robot.home()\n"
                "robot.wait(0.5)\n"
                "\n"
                "# Move to specific coordinates\n"
                "robot.move_xyz(10, 15, 20)\n"
                "robot.wait(0.5)\n"
                "\n"
                "# Rotate specific joint\n"
                "joints = robot.get_joint_names()\n"
                "if joints:\n"
                "    robot.move(joints[0], 45)\n"
                "\n"
                "robot.log('Sequence finished.')\n"
                "robot.home()\n"
            ),
            "matlab": (
                "% ============================================================\n"
                "% E-labs MATLAB/Octave Notation Script\n"
                "% \n"
                "% Available commands:\n"
                "%   joint('Name', angle);     - Smooth joint rotation\n"
                "%   move_xyz(x, y, z);        - Smooth Cartesian IK\n"
                "%   home();                   - Reset robot\n"
                "%   pause(seconds);           - Delay\n"
                "%   set_speed(percent);       - Set speed\n"
                "%   clc;                      - Clear console\n"
                "% ============================================================\n"
                "\n"
                "set_speed(70);\n"
                "home();\n"
                "pause(1.0);\n"
                "\n"
                "% Move End-Effector to Point\n"
                "move_xyz(15, 0, 25);\n"
                "pause(0.5);\n"
                "\n"
                "% Manual joint override\n"
                "joint('Joint1', -30);\n"
                "pause(0.5);\n"
                "\n"
                "home();\n"
            )
        }

        self.init_ui()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # --- TOP TOOLBAR ---
        toolbar = QtWidgets.QHBoxLayout()
        toolbar.setSpacing(6)

        # Icon-based action buttons — blue/white/black theme
        btn_style = """
            QPushButton {
                background-color: white;
                color: #212121;
                border: 1px solid #e0e0e0;
                border-radius: 6px;
                padding: 6px 14px;
                font-weight: bold;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #1976d2;
                color: white;
                border-color: #1976d2;
            }
            QPushButton:pressed {
                background-color: #1565c0;
                color: white;
            }
            QPushButton:disabled {
                background-color: #f5f5f5;
                color: #bdbdbd;
                border-color: #e0e0e0;
            }
        """

        self.run_btn = QtWidgets.QPushButton("  Run")
        self.run_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        self.run_btn.setToolTip("Run simulation")
        self.run_btn.setAccessibleName("Run Simulation")
        self.run_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.run_btn.setStyleSheet(btn_style)
        self.run_btn.clicked.connect(self.run_program)
        toolbar.addWidget(self.run_btn)

        self.stop_btn = QtWidgets.QPushButton("  Stop")
        self.stop_btn.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaStop))
        self.stop_btn.setToolTip("Stop execution")
        self.stop_btn.setAccessibleName("Stop Execution")
        self.stop_btn.setCursor(QtCore.Qt.PointingHandCursor)
        self.stop_btn.setStyleSheet(btn_style)
        self.stop_btn.clicked.connect(self.stop_program)
        toolbar.addWidget(self.stop_btn)

        toolbar.addStretch()

        layout.addLayout(toolbar)

        # --- Thin separator ---
        sep = QtWidgets.QFrame()
        sep.setFrameShape(QtWidgets.QFrame.HLine)
        sep.setStyleSheet("color: #e0e0e0;")
        layout.addWidget(sep)

        # --- CODE EDITOR ---
        self.code_edit = CodeEditor()
        self.code_edit.setPlainText(self.templates["command"])

        # Syntax highlighter
        self.highlighter = RobotSyntaxHighlighter(self.code_edit.document(), "command")

        # Editor takes all available space
        layout.addWidget(self.code_edit, 1)

        # --- LANGUAGE SELECTION (Bottom) ---
        lang_layout = QtWidgets.QHBoxLayout()
        lang_layout.setSpacing(8)

        lang_label = QtWidgets.QLabel("Language:")
        lang_label.setStyleSheet("color: #757575; font-size: 15px; font-weight: bold;")
        lang_layout.addWidget(lang_label)

        self.lang_btns = {}
        for lang_key, display_name in [("command", "Command"), ("python", "Python"), ("matlab", "Matlab")]:
            btn = QtWidgets.QPushButton(display_name)
            btn.setCheckable(True)
            btn.setCursor(QtCore.Qt.PointingHandCursor)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: white;
                    color: #424242;
                    border: 1px solid #e0e0e0;
                    border-radius: 6px;
                    padding: 8px 18px;
                    font-weight: bold;
                    font-size: 15px;
                }
                QPushButton:checked {
                    background-color: #1976d2;
                    color: white;
                    border-color: #1976d2;
                }
                QPushButton:hover:!checked {
                    background-color: #f5f5f5;
                    border-color: #1976d2;
                    color: #1976d2;
                }
                QPushButton:disabled {
                    background-color: #f5f5f5;
                    color: #9e9e9e;
                    border: 2px solid #e0e0e0;
                }
            """)
            btn.clicked.connect(lambda checked, lk=lang_key: self.set_language(lk))
            lang_layout.addWidget(btn)
            self.lang_btns[lang_key] = btn

        lang_layout.addStretch()
        self.lang_btns["command"].setChecked(True)
        layout.addLayout(lang_layout)

    def set_language(self, lang_key):
        """Switches the editor template and parsing mode."""
        self.current_lang = lang_key

        # Uncheck others
        for key, btn in self.lang_btns.items():
            btn.blockSignals(True)
            btn.setChecked(key == lang_key)
            btn.blockSignals(False)

        # Set template if editor is empty or just has another template
        current_text = self.code_edit.toPlainText().strip()
        is_default = any(current_text == t.strip() for t in self.templates.values())
        if not current_text or is_default:
            self.code_edit.setPlainText(self.templates[self.current_lang])

        # Update syntax highlighter
        self.highlighter.set_language(lang_key)

        self.mw.log(f"Language set to: {lang_key.capitalize()}")

    def run_program(self):
        """Pure simulation execution of the editor's code."""
        if self.is_running: return

        code = self.code_edit.toPlainText()
        lines = code.splitlines()

        self.is_running = True
        self.run_btn.setEnabled(False)

        self.mw.log(f"🧪 RUNNING {self.current_lang.upper()} SIMULATION...")

        if self.current_lang == "python":
            self.run_python_code(code)
        elif self.current_lang == "matlab":
            self.run_matlab_code(code)
        else:
            # Standard "command" parsing
            for line in lines:
                if not self.is_running: break
                line = line.strip()
                if not line or line.startswith("#"): continue
                self.execute_line(line)

        self.is_running = False
        self.run_btn.setEnabled(True)
        self.mw.log(f"{self.current_lang.capitalize()} Finished.")

    def run_python_code(self, code):
        """Executes Python code with a safe robot API."""
        class RobotAPI:
            def __init__(self, panel):
                self.panel = panel
            
            def move(self, joint_name, angle):
                """Moves a joint smoothly and waits for completion."""
                if not self.panel.is_running: return
                self.panel.mw.move_joint_animated(joint_name, angle, blocking=True)
            
            def move_xyz(self, x, y, z):
                """Moves End-Effector to [x,y,z] via IK smoothly and waits."""
                if not self.panel.is_running: return
                tcp_link = self.panel.mw._get_preferred_tcp_link()
                if tcp_link:
                    self.panel.mw._move_tcp_to_xyz(x, y, z, tcp_link, blocking=True)
            
            def wait(self, seconds):
                """Waits while keeping UI alive."""
                if not self.panel.is_running: return
                start = time.time()
                while time.time() - start < seconds and self.panel.is_running:
                    QtWidgets.QApplication.processEvents()
                    time.sleep(0.01)
            
            def home(self):
                """Smoothly returns all joints to zero."""
                if not self.panel.is_running: return
                joint_ids = list(self.panel.mw.robot.joints.keys())
                child_names = [j.child_link.name for j in self.panel.mw.robot.joints.values() if j.child_link]
                targets = [0.0] * len(joint_ids)
                self.panel.mw._start_joint_animation(joint_ids, child_names, targets, blocking=True)
            
            def get_joint(self, name):
                """Returns current joint angle."""
                if name in self.panel.mw.robot.joints:
                    return float(self.panel.mw.robot.joints[name].current_value)
                return 0.0
            
            def get_joint_names(self):
                """Returns list of all joint names."""
                return list(self.panel.mw.robot.joints.keys())

            def get_tcp(self):
                """Returns end-effector [x,y,z] in CM."""
                ratio = getattr(self.panel.mw.canvas, "grid_units_per_cm", 1.0)
                tcp_link = self.panel.mw._get_preferred_tcp_link()
                if tcp_link:
                    pos, _, _ = self.panel.mw.get_link_tool_point(tcp_link)
                    return (pos / ratio).tolist()
                return [0.0, 0.0, 0.0]
            
            def set_speed(self, percent):
                """Sets the animation speed (0-100)."""
                self.panel.mw.on_speed_change(float(np.clip(percent, 0, 100)))
            
            def log(self, msg):
                self.panel.mw.log(str(msg))
            
            def clear(self):
                self.panel.mw.console.clear()


        api = RobotAPI(self)
        try:
            # Execute with robot api available as 'robot'
            exec(code, {"robot": api, "print": self.mw.log})
        except Exception as e:
            self.mw.log(f"Python Error: {e}")

    def run_matlab_code(self, code):
        """Simulates Matlab syntax execution."""
        lines = code.splitlines()
        for line in lines:
            if not self.is_running: break
            line = line.strip()
            if not line or line.startswith("%"): continue

            joint_match = re.match(r"joint\s*\(['\"](.+?)['\"]\s*,\s*(-?\d+\.?\d*)\s*\);?", line, re.IGNORECASE)
            move_xyz_match = re.match(r"move_xyz\s*\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\);?", line, re.IGNORECASE)
            pause_match = re.match(r"pause\s*\((-?\d+\.?\d*)\s*\);?", line, re.IGNORECASE)

            if joint_match:
                name = joint_match.group(1)
                val = float(joint_match.group(2))
                self.mw.move_joint_animated(name, val, blocking=True)
            elif move_xyz_match:
                x = float(move_xyz_match.group(1))
                y = float(move_xyz_match.group(2))
                z = float(move_xyz_match.group(3))
                tcp_link = self.mw._get_preferred_tcp_link()
                if tcp_link:
                    self.mw._move_tcp_to_xyz(x, y, z, tcp_link, blocking=True)
            elif pause_match:
                val = float(pause_match.group(1))
                start = time.time()
                while time.time() - start < val and self.is_running:
                    QtWidgets.QApplication.processEvents()
                    time.sleep(0.01)
            elif re.match(r"home\s*\(\s*\);?", line, re.IGNORECASE):
                joint_ids = list(self.mw.robot.joints.keys())
                child_names = [j.child_link.name for j in self.mw.robot.joints.values() if j.child_link]
                targets = [0.0] * len(joint_ids)
                self.mw._start_joint_animation(joint_ids, child_names, targets, blocking=True)
            elif re.match(r"set_speed\s*\(\s*(-?\d+\.?\d*)\s*\);?", line, re.IGNORECASE):
                speed_match = re.match(r"set_speed\s*\(\s*(-?\d+\.?\d*)\s*\);?", line, re.IGNORECASE)
                self.mw.on_speed_change(float(speed_match.group(1)))
            elif re.match(r"clc\s*\(\s*\);?", line, re.IGNORECASE) or line.lower() == "clc":
                self.mw.console.clear()
            else:
                self.mw.log(f"Matlab Parser: Skipping unknown line: {line}")


    def _move_tcp_to_xyz(self, x_cm, y_cm, z_cm):
        """Moves the current TCP to an XYZ target in centimeters using the robot IK solver."""
        if not self.mw.robot.joints:
            self.mw.log("⚠️ No robot joints defined for MOVE!")
            return
        
        # Get TCP link
        tcp_link = self.mw._get_preferred_tcp_link()
        if not tcp_link:
            self.mw.log("⚠️ No TCP link found for MOVE!")
            return

        success, info = self.mw._move_tcp_to_xyz(x_cm, y_cm, z_cm, tcp_link)
        if not success:
            self.mw.log(f"❌ MOVE failed to reach ({x_cm}, {y_cm}, {z_cm})")

    def stop_program(self):
        """Stops the current execution."""
        if self.is_running:
            self.is_running = False
            self.mw.log("🛑 SIMULATION STOPPED BY USER.")

    def execute_line(self, line):
        """
        Parses and executes a single line of robot command.
        Commands:
          JOINT Name Angle
          MOVE X Y Z
          WAIT Seconds
          SPEED Percent
          HOME
        """
        if not self.is_running: return
        
        parts = line.split()
        if not parts: return
        
        cmd = parts[0].upper()
        
        try:
            if cmd == "JOINT" and len(parts) >= 3:
                name = parts[1]
                angle = float(parts[2])
                self.mw.joint_tab.apply_joint_rotation(name, angle)
                # Small delay for visual smoothiness in loop
                QtWidgets.QApplication.processEvents()
                time.sleep(0.01)
                
            elif cmd == "MOVE" and len(parts) >= 4:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                self._move_tcp_to_xyz(x, y, z)
                QtWidgets.QApplication.processEvents()
                
            elif cmd == "WAIT" and len(parts) >= 2:
                sec = float(parts[1])
                # Non-blocking wait to keep UI responsive
                start = time.time()
                while time.time() - start < sec and self.is_running:
                    QtWidgets.QApplication.processEvents()
                    time.sleep(0.05)
            
            elif cmd == "SPEED" and len(parts) >= 2:
                speed = float(parts[1])
                self.mw.current_speed = np.clip(speed, 0, 100)
                if hasattr(self.mw, 'speed_slider'):
                    self.mw.speed_slider.setValue(int(self.mw.current_speed))
                self.mw.log(f"Speed set to {self.mw.current_speed}%")

            elif cmd == "HOME":
                self.mw.reset_to_home()
                QtWidgets.QApplication.processEvents()
                
        except Exception as e:
            self.mw.log(f"Execution Error in line '{line}': {e}")
