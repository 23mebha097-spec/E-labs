import sys
import os
from PyQt5 import QtWidgets, QtCore

# Mock QMessageBox to auto-confirm deletion and other questions
class MockMessageBox:
    @staticmethod
    def question(parent, title, text, buttons, defaultButton):
        print(f"[Mock Dialog] Question: {title} - {text} -> Auto-answering YES")
        return QtWidgets.QMessageBox.Yes

QtWidgets.QMessageBox.question = MockMessageBox.question

from ui.main_window import MainWindow

def test_session_manager():
    print("==================================================")
    print("    STARTING MULTI-ROBOT SESSION MANAGER TESTS    ")
    print("==================================================")

    app = QtWidgets.QApplication(sys.argv)
    
    print("\n--- Test 1: Initialization ---")
    window = MainWindow()
    assert len(window.robot_sessions) == 1, "Should have 1 default session"
    assert window.current_session_index == 0, "Current session index should be 0"
    assert window.session_tab_bar.tabText(0) == "ToRoTrOn", "First tab title should be 'ToRoTrOn'"
    print("[SUCCESS] Initialization validated successfully.")

    print("\n--- Test 2: Creating New Sessions ---")
    window._on_new_session_clicked()
    assert len(window.robot_sessions) == 2, "Should have 2 sessions"
    assert window.current_session_index == 1, "Current session index should be 1"
    assert window.session_tab_bar.tabText(1) == "Robo 2", "Second tab title should be 'Robo 2'"
    print("[SUCCESS] New session creation validated successfully.")

    print("\n--- Test 3: Session State Persistence (Mocks) ---")
    # Simulate writing code in session 1
    window.experiment_tab.program_tab.code_edit.setPlainText("JOINT Joint_1 45")
    
    # Switch back to session 0
    window.session_tab_bar.setCurrentIndex(0)
    assert window.current_session_index == 0, "Current session index should update to 0"
    # Session 0 should have empty code
    assert window.experiment_tab.program_tab.code_edit.toPlainText() == "", "Session 0 code should be empty"
    
    # Switch back to session 1
    window.session_tab_bar.setCurrentIndex(1)
    assert window.current_session_index == 1, "Current session index should update to 1"
    # Session 1 should have our saved code restored
    assert window.experiment_tab.program_tab.code_edit.toPlainText() == "JOINT Joint_1 45", "Session 1 code should be restored"
    print("[SUCCESS] State serialization/deserialization between tabs validated successfully.")

    print("\n--- Test 4: Tab Close and Deletion ---")
    # Close session 1
    window._on_session_tab_close_requested(1)
    assert len(window.robot_sessions) == 1, "Should have 1 session left"
    assert window.current_session_index == 0, "Should fall back to active session 0"
    print("[SUCCESS] Session tab close and deletion validated successfully.")

    print("\n--- Test 5: Safe Fallback on Last Tab Close ---")
    # Close session 0 (the last remaining tab)
    window._on_session_tab_close_requested(0)
    assert len(window.robot_sessions) == 1, "Closing last tab should spawn a new ToRoTrOn tab automatically"
    assert window.current_session_index == 0, "ToRoTrOn tab should be active"
    assert window.session_tab_bar.tabText(0) == "ToRoTrOn", "New tab name should be ToRoTrOn"
    print("[SUCCESS] Last-tab close fallback to ToRoTrOn validated successfully.")

    print("\n==================================================")
    print("      ALL SESSION MANAGER CORE TESTS PASSED!      ")
    print("==================================================")

if __name__ == "__main__":
    try:
        test_session_manager()
        sys.exit(0)
    except AssertionError as e:
        print(f"[FAIL] {e}")
        sys.exit(1)
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
