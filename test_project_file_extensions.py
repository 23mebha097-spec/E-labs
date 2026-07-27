from ui.mixins.project_mixin import (
    DEFAULT_PROJECT_EXTENSION,
    PROJECT_FILE_EXTENSIONS,
    PROJECT_FILE_FILTER,
    ensure_project_extension,
    project_mesh_filename,
    ProjectMixin,
)


def test_project_extensions_include_trm_trn_and_zip():
    assert ".trm" in PROJECT_FILE_EXTENSIONS
    assert ".trn" in PROJECT_FILE_EXTENSIONS
    assert ".zip" in PROJECT_FILE_EXTENSIONS


def test_project_dialog_filter_shows_trm_files():
    assert "*.trm" in PROJECT_FILE_FILTER
    assert "*.trn" in PROJECT_FILE_FILTER


def test_missing_extension_defaults_to_trm():
    assert DEFAULT_PROJECT_EXTENSION == ".trm"
    assert ensure_project_extension("robot") == "robot.trm"


def test_existing_trm_extension_is_preserved():
    assert ensure_project_extension("robot.trm") == "robot.trm"


def test_legacy_nested_mesh_path_is_resolved_by_filename(tmp_path):
    nested_mesh = tmp_path / "old_project" / "geometry" / "Jaw.STL"
    nested_mesh.parent.mkdir(parents=True)
    nested_mesh.write_bytes(b"mesh")

    resolved = ProjectMixin._resolve_project_mesh_path(
        object(),
        str(tmp_path),
        r"meshes\jaw.stl",
    )

    assert resolved == str(nested_mesh)


def test_project_mesh_filename_never_uses_link_name_characters():
    assert project_mesh_filename(0) == "link_0000.stl"
    assert project_mesh_filename(12) == "link_0012.stl"
    assert ":" not in project_mesh_filename(12)


def test_missing_embedded_mesh_recovers_from_saved_source_metadata(tmp_path, monkeypatch):
    import numpy as np
    import trimesh

    source = tmp_path / "assembly.step"
    source.write_bytes(b"step")
    source_mesh = trimesh.creation.box(extents=[1.0, 2.0, 3.0])
    monkeypatch.setattr(trimesh, "load", lambda _path: object())

    class Harness(ProjectMixin):
        def _finalize_loaded_mesh(self, _path, _loaded):
            return [{"name": "Jaw:1", "mesh": source_mesh}]

        def log(self, _message):
            pass

    recovered = Harness()._recover_project_mesh_from_source(
        {
            "import_metadata": {
                "source_path": str(source),
                "source_component_name": "Jaw:1",
                "scale_to_internal": 10.0,
                "up_axis": "preserve",
            }
        },
        {},
    )

    assert recovered is not None
    np.testing.assert_allclose(recovered.extents, [10.0, 20.0, 30.0])
