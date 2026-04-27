import sys
from PyQt5 import QtWidgets
from ui.main_window import MainWindow

# --- ToRoTRoN Robot Configuration ---
# The default angle for all joints when the Home command or button is triggered.
HOME_POSITION = 0.0 


import traceback

def exception_handler(exctype, value, tb):
    """Global exception handler for the GUI to prevent silent exits."""
    err_msg = "".join(traceback.format_exception(exctype, value, tb))
    print(f"CRASH DETECTED:\n{err_msg}")
    
    # Show a friendly dialog if app is running
    app = QtWidgets.QApplication.instance()
    if app:
        msg = QtWidgets.QMessageBox()
        msg.setIcon(QtWidgets.QMessageBox.Critical)
        msg.setText("🚀 E-lab Exception")
        msg.setInformativeText("The application encountered an unexpected error during simulation.")
        msg.setDetailedText(err_msg)
        msg.setWindowTitle("System Crash Recovery")
        msg.exec_()
    sys.__excepthook__(exctype, value, tb)

sys.excepthook = exception_handler

def main():
    print("[1/3] Initializing Application...")
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    
    print("[2/3] Loading 3D Engine & UI...")
    try:
        window = MainWindow()
        window.show()
        
        print("[3/3] Application Ready.")
        sys.exit(app.exec_())
    except Exception:
        # Final catch-all for init phase
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
