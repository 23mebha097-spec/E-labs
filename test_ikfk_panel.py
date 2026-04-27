import sys
from PyQt5 import QtWidgets
from ui.panels.ik_fk_panel import IKFKPanel

app = QtWidgets.QApplication(sys.argv)
try:
    mw = QtWidgets.QMainWindow()
    panel = IKFKPanel(mw)
    print("IKFKPanel instantiated successfully")
except Exception as e:
    import traceback
    traceback.print_exc()
