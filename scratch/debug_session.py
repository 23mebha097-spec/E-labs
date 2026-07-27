import sys
from PyQt5 import QtWidgets
from ui.main_window import MainWindow

app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
window = MainWindow()
print('init session0 program=', repr(window.robot_sessions[0].get('program_code', '')))
window._on_new_session_clicked()
print('after new session0=', repr(window.robot_sessions[0].get('program_code', '')))
print('after new session1=', repr(window.robot_sessions[1].get('program_code', '')))
window.experiment_tab.program_tab.code_edit.setPlainText('JOINT Joint_1 45')
window.session_tab_bar.setCurrentIndex(0)
print('current index=', window.current_session_index)
print('session0 stored=', repr(window.robot_sessions[0].get('program_code', '')))
print('session1 stored=', repr(window.robot_sessions[1].get('program_code', '')))
print('editor text=', repr(window.experiment_tab.program_tab.code_edit.toPlainText()))
