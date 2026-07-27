from types import MethodType, SimpleNamespace

from PyQt5 import QtWidgets

from ui.main_window import MainWindow


def _session_harness(load_result=True):
    harness = SimpleNamespace()
    harness.robot_sessions = [{"title": "ToRoTrOn", "robot": "existing"}]
    harness.current_session_index = 0
    harness.messages = []
    harness.toasts = []
    harness.restored = []
    harness.loaded_paths = []
    harness._capture_current_robot_session = lambda: None
    harness._default_robot_session = lambda title: {
        "title": title,
        "robot": "empty",
        "project_file_path": None,
    }
    harness._restore_robot_session = lambda index: harness.restored.append(index)
    harness._apply_default_robot_session_ui_state = lambda: None
    harness.log = harness.messages.append
    harness.show_toast = lambda *args: harness.toasts.append(args)

    def load_project(file_path, show_dialogs=True, auto_finalize=True):
        harness.loaded_paths.append(file_path)
        return load_result

    harness.load_project_from_path = load_project
    harness._discard_failed_robot_session = MethodType(
        MainWindow._discard_failed_robot_session,
        harness,
    )
    harness.add_robot_session = MethodType(MainWindow.add_robot_session, harness)
    return harness


def test_add_robo_opens_saved_trn_in_a_new_session(tmp_path):
    project_path = tmp_path / "welding_robot.trn"
    project_path.write_bytes(b"project")
    harness = _session_harness(load_result=True)

    loaded = harness.add_robot_session(str(project_path))

    assert loaded is True
    assert len(harness.robot_sessions) == 2
    assert harness.current_session_index == 1
    assert harness.robot_sessions[1]["title"] == "welding_robot"
    assert harness.robot_sessions[1]["project_file_path"] == str(project_path)
    assert harness.loaded_paths == [str(project_path)]


def test_add_robo_restores_existing_session_when_load_fails(tmp_path):
    project_path = tmp_path / "invalid.trm"
    project_path.write_bytes(b"invalid")
    harness = _session_harness(load_result=False)

    loaded = harness.add_robot_session(str(project_path))

    assert loaded is False
    assert harness.robot_sessions == [{"title": "ToRoTrOn", "robot": "existing"}]
    assert harness.current_session_index == 0
    assert harness.restored[-1] == 0


def test_add_robo_picker_accepts_trn_and_passes_it_to_new_session(monkeypatch, tmp_path):
    project_path = tmp_path / "painting_robot.trn"
    selected_paths = []
    harness = SimpleNamespace(
        _project_dialog_dir=lambda: str(tmp_path),
        add_robot_session=lambda project_file_path=None: selected_paths.append(project_file_path) or True,
    )
    monkeypatch.setattr(
        QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *args, **kwargs: (str(project_path), "Robot Project (*.trm *.trn)"),
    )

    opened = MainWindow.open_saved_robot_session(harness)

    assert opened is True
    assert selected_paths == [str(project_path)]
