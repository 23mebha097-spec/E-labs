import os
import signal
import sys
from PyQt5 import QtCore, QtWidgets
from ui.main_window import MainWindow

# --- ToRoTRoN Robot Configuration ---
# The default angle for all joints when the Home command or button is triggered.
HOME_POSITION = 0.0 
DEFAULT_STARTUP_PROJECT = r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\MANIPULATOR\1804.trn"


import traceback

def exception_handler(exctype, value, tb):
    """Global exception handler for real GUI crashes.

    Qt/VTK can surface KeyboardInterrupt/SystemExit through paint/update
    callbacks; those should not trigger the crash-recovery dialog.
    """
    if exctype in (KeyboardInterrupt, SystemExit) or (isinstance(exctype, type) and issubclass(exctype, KeyboardInterrupt)):
        print(f"{exctype.__name__}: application shutdown requested.")
        return

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
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    
    print("[2/3] Loading 3D Engine & UI...")
    try:
        window = MainWindow()
        window.show()

        startup_project = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STARTUP_PROJECT
        if startup_project and os.path.exists(startup_project):
            QtCore.QTimer.singleShot(
                0,
                lambda path=startup_project: window.load_project_from_path(
                    path,
                    show_dialogs=False,
                    auto_finalize=True,
                ),
            )
        
        print("[3/3] Application Ready.")
        # Keep Python signal handling alive while the Qt event loop is running.
        signal_timer = QtCore.QTimer()
        signal_timer.start(250)
        signal_timer.timeout.connect(lambda: None)

        exit_code = app.exec_()
        print(f"Application exited with code {exit_code}.")
        sys.exit(exit_code)
    except Exception:
        # Final catch-all for init phase
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
 