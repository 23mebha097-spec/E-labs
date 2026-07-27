from PyQt5 import QtWidgets, QtCore, QtGui
import time
import os
import re
import numpy as np

from elabs import Robot
from elabs.runtime import simulation_context


class RobotSyntaxHighlighter(QtGui.QSyntaxHighlighter):
    """Syntax highlighter for robot programming languages (Command, Python)."""

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
                       r'\bRobot\b', r'\brobot\.move\b', r'\brobot\.move_joint\b',
                       r'\brobot\.move_xyz\b', r'\brobot\.move_tcp\b', r'\brobot\.wait\b',
                       r'\brobot\.home\b', r'\brobot\.get_joint\b', r'\brobot\.get_tcp\b',
                       r'\brobot\.set_speed\b', r'\brobot\.log\b',
                       r'\brobot\.gripper\.open\b', r'\brobot\.gripper\.close\b',
                       r'\bplan\.line\b', r'\bplan\.circle\b', r'\bplan\.arc\b',
                       r'\bplan\.rectangle\b', r'\bplan\.rect\b', r'\bplan\.clear\b']:
                self.rules.append((re.compile(bi), builtin_fmt))
            # Function calls
            self.rules.append((re.compile(r'\b[a-zA-Z_]\w*(?=\s*\()'), func_fmt))
            # Strings
            self.rules.append((re.compile(r"'[^']*'"), string_fmt))
            self.rules.append((re.compile(r'"[^"]*"'), string_fmt))
            # Comments
            self.rules.append((re.compile(r'#.*$', re.MULTILINE), comment_fmt))

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


class PlanAPI:
    """
    Exposes CAD drawing tools to Python scripts, enabling 2D shape drawing on the
    45-degree inclined reachable workspace plane using robot live-point tracing.
    """
    def __init__(self, panel):
        self.panel = panel
        self._current_pos_local = np.array([0.0, 0.0, 0.0]) # Pen location local to inclined workspace plane
        self._current_live_point_world = None

    def _get_workspace_plan(self):
        """Helper to get the current auto-calculated workspace plan from the canvas."""
        canvas = self.panel.mw.canvas
        if canvas and hasattr(canvas, 'current_workspace_plan'):
            return canvas.current_workspace_plan
        return None

    def _get_base_world_transform(self):
        """Helper to get the robot base world transformation matrix."""
        try:
            base_world = np.array(self.panel.mw.robot.base_link.t_world, dtype=float).copy()
            ratio = getattr(self.panel.mw.canvas, "grid_units_per_cm", 1.0) or 1.0
            base_world[:3, 3] = base_world[:3, 3] / ratio
            return base_world
        except Exception:
            return np.eye(4)

    def _get_tcp_link(self):
        """Helper to get the active robot toolpoint TCP link."""
        return self.panel.mw._get_preferred_tcp_link()

    def _get_live_point_world_cm(self, tcp_link):
        """Return the current live point/TCP in world coordinates, expressed in cm."""
        ratio = getattr(self.panel.mw.canvas, "grid_units_per_cm", 1.0) or 1.0
        pos_world_internal, _, _ = self.panel.mw.get_link_tool_point(tcp_link)
        return np.array(pos_world_internal, dtype=float) / ratio

    def _move_local_pen(self, x_local, y_local, is_drawing=True):
        """
        Moves the robot's TCP toolpoint to a 2D local point on the inclined workspace plane.
        If is_drawing is True, a permanent visual line actor is rendered between successive locations.
        """
        if not self.panel.is_running:
            return False

        ws_plan = self._get_workspace_plan()
        if ws_plan is None:
            self.panel.mw.log("⚠️ Plan warning: Workspace plane not calculated. Please assemble the robot and click 'Make Robo' first.")
            return False

        # Target coordinate on workspace plane (local Z is always 0.0)
        target_local = np.array([x_local, y_local, 0.0])
        if not ws_plan.validate_workspace_bounds(target_local):
            self.panel.mw.log(
                f"⚠️ Plan warning: CAD point ({x_local:.2f}, {y_local:.2f}) is outside the verified drawing plane."
            )
            return False

        target_world = ws_plan.convert_local_to_world(target_local, self._get_base_world_transform())

        tcp_link = self._get_tcp_link()
        if not tcp_link:
            self.panel.mw.log("⚠️ Plan warning: No active TCP toolpoint link found for drawing.")
            return False

        # Perform smooth synchronous IK movement to target world point
        success, info = self.panel.mw._move_tcp_to_xyz(target_world[0], target_world[1], target_world[2], tcp_link, blocking=True)
        
        if success:
            live_point_world = self._get_live_point_world_cm(tcp_link)
            if is_drawing:
                prev_world = self._current_live_point_world
                if prev_world is None:
                    prev_world = ws_plan.convert_local_to_world(self._current_pos_local, self._get_base_world_transform())
                self.panel.mw.canvas.add_cad_line(prev_world, live_point_world)
            self._current_live_point_world = live_point_world
            self._current_pos_local = target_local
            return True
        else:
            self.panel.mw.log(f"⚠️ Plan warning: Local coordinates ({x_local:.2f}, {y_local:.2f}) are outside robot's reach.")
            return False

    def line(self, x1, y1, x2, y2, steps=20):
        """Draws a straight line from (x1, y1) to (x2, y2) local coordinates on the plan."""
        if not self.panel.is_running:
            return

        ws_plan = self._get_workspace_plan()
        if ws_plan is None:
            self.panel.mw.log("⚠️ Plan warning: Workspace plane not calculated. Please assemble the robot and click 'Make Robo' first.")
            return

        # 1. Approach starting position without drawing
        self._current_pos_local = np.array([x1, y1, 0.0])
        self._move_local_pen(x1, y1, is_drawing=False)

        # 2. Linear interpolate waypoints to draw
        xs = np.linspace(x1, x2, steps)
        ys = np.linspace(y1, y2, steps)
        for x, y in zip(xs[1:], ys[1:]):
            if not self.panel.is_running:
                break
            self._move_local_pen(x, y, is_drawing=True)

    def circle(self, cx, cy, r, steps=40):
        """Draws a complete circle with center (cx, cy) and radius r on the plan."""
        if not self.panel.is_running:
            return

        ws_plan = self._get_workspace_plan()
        if ws_plan is None:
            self.panel.mw.log("⚠️ Plan warning: Workspace plane not calculated. Please assemble the robot and click 'Make Robo' first.")
            return

        # 1. Approach circle start coordinate
        x0 = cx + r
        y0 = cy
        self._current_pos_local = np.array([x0, y0, 0.0])
        self._move_local_pen(x0, y0, is_drawing=False)

        # 2. Tracing the circular path
        angles = np.linspace(0, 2 * np.pi, steps + 1)
        for angle in angles[1:]:
            if not self.panel.is_running:
                break
            x = cx + r * np.cos(angle)
            y = cy + r * np.sin(angle)
            self._move_local_pen(x, y, is_drawing=True)

    def arc(self, cx, cy, r, start_ang, end_ang, steps=30):
        """Draws a circular arc centered at (cx, cy) with radius r between start_ang and end_ang degrees."""
        if not self.panel.is_running:
            return

        ws_plan = self._get_workspace_plan()
        if ws_plan is None:
            self.panel.mw.log("⚠️ Plan warning: Workspace plane not calculated. Please assemble the robot and click 'Make Robo' first.")
            return

        start_rad = np.radians(start_ang)
        end_rad = np.radians(end_ang)

        # 1. Approach arc start coordinate
        x0 = cx + r * np.cos(start_rad)
        y0 = cy + r * np.sin(start_rad)
        self._current_pos_local = np.array([x0, y0, 0.0])
        self._move_local_pen(x0, y0, is_drawing=False)

        # 2. Tracing the arc
        angles = np.linspace(start_rad, end_rad, steps + 1)
        for angle in angles[1:]:
            if not self.panel.is_running:
                break
            x = cx + r * np.cos(angle)
            y = cy + r * np.sin(angle)
            self._move_local_pen(x, y, is_drawing=True)

    def rectangle(self, x, y, w, h):
        """Draws a rectangular frame from bottom-left corner (x, y) with width w and height h."""
        if not self.panel.is_running:
            return

        ws_plan = self._get_workspace_plan()
        if ws_plan is None:
            self.panel.mw.log("⚠️ Plan warning: Workspace plane not calculated. Please assemble the robot and click 'Make Robo' first.")
            return

        # Sequential tracing of 4 edges
        self.line(x, y, x + w, y, steps=15)
        self.line(x + w, y, x + w, y + h, steps=15)
        self.line(x + w, y + h, x, y + h, steps=15)
        self.line(x, y + h, x, y, steps=15)

    def rect(self, x, y, w, h):
        """Alias for rectangle."""
        self.rectangle(x, y, w, h)

    def clear(self):
        """Clears all drawn CAD trajectories from the 3D canvas plotter."""
        canvas = self.panel.mw.canvas
        if canvas and hasattr(canvas, 'clear_cad_drawings'):
            canvas.clear_cad_drawings()
        self._current_live_point_world = None
        self.panel.mw.log("🧹 CAD drawings cleared from inclined workspace plane.")


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
                "# E-Labs Python Robotics API\n"
                "# The built-in elabs package controls the live simulation.\n"
                "# \n"
                "# ROBOT MOVEMENT CONTROLS:\n"
                "#   from elabs import Robot\n"
                "#   robot.set_speed(percent)        - Set speed (0-100)\n"
                "#   robot.home()                    - Reset robot to zero pose\n"
                "#   robot.move_joint(name, angle)   - Move joint in degrees\n"
                "#   robot.move_tcp(x, y, z)         - Move TCP/live point in cm\n"
                "#   robot.gripper.open()/close()    - Control configured gripper\n"
                "# ============================================================\n"
                "\n"
                "from elabs import Robot\n"
                "\n"
                "robot = Robot()\n"
                "robot.set_speed(50)\n"
                "robot.home()\n"
                "robot.wait(1)\n"
                "robot.move_joint('Shoulder', 45)\n"
                "robot.wait(1)\n"
                "robot.move_joint('Elbow', 30)\n"
                "robot.wait(1)\n"
                "robot.move_tcp(x=250, y=100, z=150)\n"
                "robot.gripper.close()\n"
                "robot.home()\n"
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
        for lang_key, display_name in [("command", "Command"), ("python", "Python")]:
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

        try:
            if self.current_lang == "python" and self._looks_like_task_dsl(code):
                self.run_task_script(code)
            elif self.current_lang == "python":
                self.run_python_code(code)
            else:
                # Standard "command" parsing
                for line in lines:
                    if not self.is_running:
                        break
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    self.execute_line(line)
        finally:
            self.is_running = False
            self.run_btn.setEnabled(True)
            self.mw.log(f"{self.current_lang.capitalize()} Finished.")

    def run_python_code(self, code):
        """Executes Python code with the built-in E-Labs robotics API."""
        robot_api = Robot(self)
        plan_api = PlanAPI(self)
        try:
            # Bind this panel so "from elabs import Robot" attaches to the live simulation.
            with simulation_context(self):
                exec(
                    code,
                    {
                        "Robot": Robot,
                        "robot": robot_api,
                        "plan": plan_api,
                        "print": self.mw.log,
                    },
                )
        except Exception as e:
            self.mw.log(f"Python Error: {e}")

    def _looks_like_task_dsl(self, code):
        """Detect the compact object-operation task syntax."""
        patterns = [
            r"^\s*robot\.operation_pick_and_place\b",
            r"^\s*robot\.operation_weld(?:ing)?\b",
            r"^\s*robot\.operation_paint(?:ing)?\b",
            r"^\s*obj\s*=\s*\d+\s*$",
            r"^\s*Px\s*,\s*Py\s*,\s*Pz\s*:",
            r"^\s*P[12]\s*:",
            r"^\s*robot\.home\s*\(\s*[-+]?\d",
        ]
        return any(re.search(pattern, code, re.IGNORECASE | re.MULTILINE) for pattern in patterns)

    def _parse_task_number(self, text):
        value = float(str(text).strip())
        return int(value) if value.is_integer() else value

    def _parse_task_script(self, code):
        spec = {
            "operation": None,
            "end_effector": None,
            "object_index": None,
            "home_pos": None,
            "start_pos": None,
            "place_pos": None,
            "path_points": 8,
            "paint_color": None,
            "cycles": 1,
            "return_home": False,
        }

        for raw_line in code.splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue

            if re.match(r"^(from\s+elabs\s+import\s+Robot|import\s+Robot)\b", line, re.IGNORECASE):
                continue

            if re.match(r"^robot\.operation_pick_and_place\b", line, re.IGNORECASE):
                spec["operation"] = "pick_and_place"
                continue

            if re.match(r"^robot\.operation_weld(?:ing)?\b", line, re.IGNORECASE):
                spec["operation"] = "welding"
                continue

            if re.match(r"^robot\.operation_paint(?:ing)?\b", line, re.IGNORECASE):
                spec["operation"] = "painting"
                continue

            match = re.match(r"^robot\.end_effector\s*=\s*([A-Za-z_]\w*)\s*$", line, re.IGNORECASE)
            if match:
                spec["end_effector"] = match.group(1).strip().lower()
                continue

            match = re.match(r"^obj\s*=\s*(\d+)\s*$", line, re.IGNORECASE)
            if match:
                spec["object_index"] = int(match.group(1))
                continue

            match = re.match(
                r"^Px\s*,\s*Py\s*,\s*Pz\s*:\s*"
                r"([-+]?\d*\.?\d+)\s*,\s*"
                r"([-+]?\d*\.?\d+)\s*,\s*"
                r"([-+]?\d*\.?\d+)\s*$",
                line,
                re.IGNORECASE,
            )
            if match:
                spec["place_pos"] = tuple(self._parse_task_number(v) for v in match.groups())
                continue

            match = re.match(
                r"^P([12])\s*:\s*"
                r"([-+]?\d*\.?\d+)\s*,\s*"
                r"([-+]?\d*\.?\d+)\s*,\s*"
                r"([-+]?\d*\.?\d+)\s*$",
                line,
                re.IGNORECASE,
            )
            if match:
                point = tuple(self._parse_task_number(v) for v in match.groups()[1:])
                if match.group(1) == "1":
                    spec["start_pos"] = point
                else:
                    spec["place_pos"] = point
                continue

            match = re.match(r"^path_points\s*=\s*(\d+)\s*$", line, re.IGNORECASE)
            if match:
                spec["path_points"] = max(2, min(40, int(match.group(1))))
                continue

            match = re.match(
                r"^paint_color\s*=\s*['\"]?([A-Za-z_]+)['\"]?\s*$",
                line,
                re.IGNORECASE,
            )
            if match:
                spec["paint_color"] = match.group(1).strip().lower()
                continue

            match = re.match(
                r"^robot\.home\s*\(\s*"
                r"([-+]?\d*\.?\d+)\s*,\s*"
                r"([-+]?\d*\.?\d+)\s*,\s*"
                r"([-+]?\d*\.?\d+)"
                r"(?:\s*,\s*([-+]?\d*\.?\d+)\s*)?"
                r"\)\s*$",
                line,
                re.IGNORECASE,
            )
            if match:
                spec["home_pos"] = tuple(self._parse_task_number(v) for v in match.groups()[:3])
                if match.group(4) is not None:
                    spec["cycles"] = max(1, int(float(match.group(4))))
                continue

            match = re.match(
                r"^\(?\s*robot\.home\s*,\s*cycles\s*\)?\s*:\s*\(\s*"
                r"([-+]?\d*\.?\d+)\s*,\s*"
                r"([-+]?\d*\.?\d+)\s*,\s*"
                r"([-+]?\d*\.?\d+)\s*,\s*"
                r"([-+]?\d*\.?\d+)\s*\)\s*$",
                line,
                re.IGNORECASE,
            )
            if match:
                spec["home_pos"] = tuple(self._parse_task_number(v) for v in match.groups()[:3])
                spec["cycles"] = max(1, int(float(match.group(4))))
                continue

            match = re.match(r"^cycles\s*=\s*(\d+)\s*$", line, re.IGNORECASE)
            if match:
                spec["cycles"] = max(1, int(match.group(1)))
                continue

            match = re.match(r"^robot\.home\s*\(\s*\)\s*$", line, re.IGNORECASE)
            if match:
                spec["return_home"] = True
                continue

        return spec

    def _selected_sim_object_by_index(self, index):
        exp_tab = getattr(self.mw, "experiment_tab", None)
        object_tab = getattr(exp_tab, "object_tab", None) if exp_tab is not None else None
        objects_list = getattr(object_tab, "objects_list", None)
        robot = getattr(self.mw, "robot", None)
        if robot is None:
            return None, None

        if objects_list is None:
            sim_names = [
                name for name, link in robot.links.items()
                if getattr(link, "is_sim_obj", False)
            ]
        else:
            sim_names = []
            for i in range(objects_list.count()):
                item = objects_list.item(i)
                if item is None:
                    continue
                name = item.data(QtCore.Qt.UserRole)
                if not isinstance(name, str):
                    name = item.text().strip()
                if name:
                    sim_names.append(name)

        if not sim_names:
            return None, None

        if index is None:
            if objects_list is not None:
                current_item = objects_list.currentItem()
                if current_item is not None:
                    selected_name = current_item.data(QtCore.Qt.UserRole)
                    if not isinstance(selected_name, str):
                        selected_name = current_item.text().strip()
                    if selected_name in robot.links:
                        return selected_name, robot.links[selected_name]
            name = sim_names[0]
            return name, robot.links.get(name)

        try:
            name = sim_names[max(0, int(index) - 1)]
        except Exception:
            return None, None
        return name, robot.links.get(name)

    def run_task_script(self, code):
        """Execute Pick & Place, Welding, or Painting from the Code panel."""
        try:
            spec = self._parse_task_script(code)
            operation = spec["operation"] or "pick_and_place"
            if operation not in ("pick_and_place", "welding", "painting"):
                self.mw.log(f"Task Error: unsupported operation '{spec['operation']}'.")
                return

            tool_aliases = {
                "pick_and_place": {"gripper_tool", "gripper", "grippertool"},
                "welding": {"welding_tool", "weld_tool", "welder", "welding"},
                "painting": {"painting_tool", "paint_tool", "painter", "painting"},
            }
            requested_tool = spec["end_effector"]
            if requested_tool is not None and requested_tool not in tool_aliases[operation]:
                self.mw.log(
                    f"Task Error: end effector '{requested_tool}' does not match the {operation.replace('_', ' ')} operation."
                )
                return

            object_name, link = self._selected_sim_object_by_index(spec["object_index"])
            if link is None or object_name is None:
                self.mw.log("Task Error: Select an imported object first.")
                return

            if object_name not in self.mw.robot.links:
                self.mw.log("Task Error: selected object is not in the robot model.")
                return

            if operation == "pick_and_place" and spec["place_pos"] is None:
                self.mw.log("Task Error: place coordinates are missing.")
                return

            exp_tab = getattr(self.mw, "experiment_tab", None)
            object_tab = getattr(exp_tab, "object_tab", None) if exp_tab is not None else None
            sim_tab = getattr(self.mw, "simulation_tab", None)
            if object_tab is None or sim_tab is None:
                self.mw.log("Task Error: the object task engine is not available.")
                return

            self.mw.log(f"Task DSL: using imported object '{object_name}'.")
            object_tab.current_task_object = object_name
            self.mw.current_task_object = object_name
            sim_tab.current_task_object = object_name

            if hasattr(object_tab, "refresh_object_info"):
                object_tab.refresh_object_info(object_name)

            if hasattr(object_tab, "capture_object_to_p1"):
                object_tab.capture_object_to_p1()

            if hasattr(sim_tab, "objects_list"):
                if hasattr(self.mw, "refresh_sim_objects_list"):
                    self.mw.refresh_sim_objects_list()
                sim_tab.objects_list.setCurrentItem(None)
                matches = sim_tab.objects_list.findItems(object_name, QtCore.Qt.MatchExactly)
                if matches:
                    sim_tab.objects_list.setCurrentItem(matches[0])

            operation_key = "pick_place" if operation == "pick_and_place" else operation
            operation_index = sim_tab.operation_combo.findData(operation_key)
            if operation_index >= 0:
                sim_tab.operation_combo.setCurrentIndex(operation_index)

            # Pick & Place uses the object's bottom centre. Surface operations
            # derive a top-surface START/END path from the selected object.
            sim_tab.capture_object_to_p1()

            if spec["start_pos"] is not None:
                sx, sy, sz = (float(v) for v in spec["start_pos"])
                sim_tab.pick_x.setValue(sx)
                sim_tab.pick_y.setValue(sy)
                sim_tab.pick_z.setValue(sz)

            if spec["place_pos"] is not None and hasattr(sim_tab, "place_x"):
                ax, ay, az = (float(v) for v in spec["place_pos"])
                sim_tab.place_x.setValue(ax)
                sim_tab.place_y.setValue(ay)
                sim_tab.place_z.setValue(az)

            sim_tab.process_points_sb.setValue(int(spec["path_points"]))
            if operation == "painting" and spec["paint_color"]:
                paint_names = {
                    "safety_yellow": "#f9a825",
                    "yellow": "#f9a825",
                    "signal_blue": "#1976d2",
                    "blue": "#1976d2",
                    "machine_red": "#d32f2f",
                    "red": "#d32f2f",
                    "industrial_green": "#388e3c",
                    "green": "#388e3c",
                }
                paint_value = paint_names.get(spec["paint_color"])
                paint_index = sim_tab.paint_color_combo.findData(paint_value) if paint_value else -1
                if paint_index < 0:
                    self.mw.log(f"Task Error: unsupported paint color '{spec['paint_color']}'.")
                    return
                sim_tab.paint_color_combo.setCurrentIndex(paint_index)

            if spec["home_pos"] is not None:
                hx, hy, hz = (float(v) for v in spec["home_pos"])
                self.mw.home_tcp_coords = (hx, hy, hz)
                if hasattr(self.mw, "home_x"):
                    self.mw.home_x.setValue(hx)
                if hasattr(self.mw, "home_y"):
                    self.mw.home_y.setValue(hy)
                if hasattr(self.mw, "home_z"):
                    self.mw.home_z.setValue(hz)

            if spec["cycles"] not in (None, 1):
                self.mw.log("Task DSL note: cycles > 1 is not yet supported by object operations.")

            display_operation = operation.replace("_", " ").title()
            self.mw.log(f"Task DSL: starting {display_operation} from the Code panel.")
            if operation == "pick_and_place":
                sim_tab.run_pick_place_task()
            else:
                sim_tab.run_surface_operation(operation)

            if spec["home_pos"] is not None or spec["return_home"]:
                self.mw.log("Task DSL: home position stored; the robot will return after the operation.")
        except Exception as exc:
            self.mw.log(f"Task Error: {exc}")
            return

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
        sim_tab = getattr(self.mw, "simulation_tab", None)
        if sim_tab is not None and getattr(sim_tab, "is_sim_active", False):
            sim_tab.stop_current_operation()
            self.mw.log("Object operation stopped from the Code panel.")
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
                self.mw.go_home_tcp()
                QtWidgets.QApplication.processEvents()
                
        except Exception as e:
            self.mw.log(f"Execution Error in line '{line}': {e}")





