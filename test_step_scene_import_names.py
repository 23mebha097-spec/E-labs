import numpy as np
import trimesh

from ui.mixins.links_mixin import LinksMixin


class SceneImportNameTests:
    def test_extract_scene_parts_preserves_geometry_names(self):
        scene = trimesh.Scene()
        part_01 = trimesh.creation.box(extents=[1, 1, 1])
        part_02 = trimesh.creation.box(extents=[2, 2, 2])
        scene.add_geometry(part_01, geom_name="part_01", transform=np.eye(4))
        scene.add_geometry(part_02, geom_name="part_02", transform=np.eye(4))

        mixin = LinksMixin.__new__(LinksMixin)
        parts = mixin._extract_scene_parts("sample.step", scene)

        assert [part["name"] for part in parts] == ["part_01", "part_02"]
        assert len(parts) == 2
        assert all(part["mesh"] is not None for part in parts)


if __name__ == "__main__":
    SceneImportNameTests().test_extract_scene_parts_preserves_geometry_names()
