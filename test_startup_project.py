import os

import main


def test_default_startup_project_is_used_without_args():
    assert main.resolve_startup_project([]) == main.DEFAULT_STARTUP_PROJECT


def test_missing_argument_falls_back_to_default_startup_project():
    missing_path = os.path.join("missing", "robot.trn")

    assert main.resolve_startup_project([missing_path]) == main.DEFAULT_STARTUP_PROJECT


def test_no_startup_disables_default_project():
    assert main.resolve_startup_project(["--no-startup"]) is None


def test_existing_trm_path_can_be_used_as_startup_project(tmp_path):
    project_path = tmp_path / "robot.trm"
    project_path.write_bytes(b"")

    assert main.resolve_startup_project([str(project_path)]) == str(project_path)
