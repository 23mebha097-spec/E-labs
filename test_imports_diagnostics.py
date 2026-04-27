import sys
try:
    from PyQt5 import QtWidgets, QtCore, QtGui
    print("PyQt5 imported successfully")
except ImportError as e:
    print(f"PyQt5 import failed: {e}")

try:
    import numpy as np
    print("numpy imported successfully")
except ImportError as e:
    print(f"numpy import failed: {e}")

try:
    from ui.main_window import MainWindow
    print("MainWindow imported successfully")
except Exception as e:
    import traceback
    print(f"MainWindow import failed: {e}")
    traceback.print_exc()
