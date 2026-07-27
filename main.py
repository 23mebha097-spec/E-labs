import os

import signal

import sys

import traceback



PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

if PROJECT_ROOT not in sys.path:

    sys.path.insert(0, PROJECT_ROOT)



from PyQt5 import QtCore, QtWidgets



# --- ToRoTRoN Robot Configuration   ---

# The default angle for all joints when the Home command or button is triggered.

HOME_POSITION = 0.0

DEFAULT_STARTUP_PROJECT = r"C:\Users\Bhavin\OneDrive\Desktop\bHaViN\120905\check-1 89884\once again\2207.trm"


def resolve_startup_project(argv):
    """Return the project that should be loaded when the application starts."""
    if "--no-startup" in argv:
        return None

    candidates = [arg for arg in argv if arg != "--no-startup"]
    if candidates:
        requested = candidates[0]
        if os.path.exists(requested):
            return requested
        print(
            f"[startup] Ignoring missing startup project '{requested}'; using default.",
            flush=True,
        )

    return DEFAULT_STARTUP_PROJECT





def exception_handler(exctype, value, tb):

    """Global exception handler for real GUI crashes.



    Qt/VTK can surface KeyboardInterrupt/SystemExit through paint/update

    callbacks; those should not trigger the crash-recovery dialog.

    """

    if exctype in (KeyboardInterrupt, SystemExit) or (

        isinstance(exctype, type) and issubclass(exctype, KeyboardInterrupt)

    ):

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

    print(

        "Loading 3D libraries (PyVista/VTK, SciPy). "

        "First start can take 1-2 minutes on OneDrive; do not press Ctrl+C...",

        flush=True,

    )

    try:

        import pyvista  # noqa: F401

        import scipy  # noqa: F401

        import vtkmodules  # noqa: F401

    except Exception as exc:

        print("Failed to load the 3D/runtime libraries required by E-Lab.", flush=True)

        print(f"Error: {exc}", flush=True)

        print("Run 'pip install -r requirements.txt' in the project environment, then try again.", flush=True)

        raise

    from ui.main_window import MainWindow



    print("[1/3] Initializing Application...")

    app = QtWidgets.QApplication(sys.argv)

    app.setStyle("Fusion")

    signal.signal(signal.SIGINT, lambda *_: app.quit())



    print("[2/3] Loading 3D Engine & UI...")

    try:

        window = MainWindow()

        window.show()

        startup_project = resolve_startup_project(sys.argv[1:])

        if startup_project and os.path.exists(startup_project):
            def load_startup_project(attempt=1):
                ok = window.load_project_from_path(
                    startup_project,
                    show_dialogs=False,
                    auto_finalize=True,
                )
                if ok:
                    print(f"[startup] Loaded project: {startup_project}", flush=True)
                    return

                if attempt == 1:
                    print(
                        f"[startup] Initial load failed for {startup_project}; retrying after the UI settles.",
                        flush=True,
                    )
                    QtCore.QTimer.singleShot(750, lambda: load_startup_project(attempt=2))
                else:
                    print(f"[startup] Failed to load startup project: {startup_project}", flush=True)

            QtCore.QTimer.singleShot(150, load_startup_project)



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


