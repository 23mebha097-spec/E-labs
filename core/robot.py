import numpy as np
from itertools import product

try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None

try:
    from scipy.optimize import least_squares
except Exception:
    least_squares = None

try:
    import ikpy.chain
    import ikpy.link
except Exception:
    ikpy = None

try:
    from spatialmath import SE3, SO3
except Exception:
    SE3 = None

from core.kinematics import (
    compute_standard_dh_matrix,
    invert_transform,
    pose_dict_from_transform,
    pose_error,
    transform_from_pose,
)


class Link:
    def __init__(self, name, mesh=None):
        self.name = name
        self.mesh = mesh
        self.color = "lightgray"
        self.is_base = False
        self.pick_pos = [0.0, 0.0, 0.0]
        self.place_pos = [0.0, 0.0, 0.0]

        self.t_offset = np.eye(4)
        self.t_world = np.eye(4)

        self.parent_joint = None
        self.child_joints = []

        # TCP / live point relative to the flange link frame.
        self.custom_tcp_offset = None
        self.custom_tcp_rpy_deg = [0.0, 0.0, 0.0]

        self.import_metadata = {}

        self.mass = 1.0
        self.inertia = {
            "ixx": 0.001,
            "ixy": 0.0,
            "ixz": 0.0,
            "iyy": 0.001,
            "iyz": 0.0,
            "izz": 0.001,
        }
        self.com = [0.0, 0.0, 0.0]
        self.compute_physics_from_mesh()

    def compute_physics_from_mesh(self):
        if self.mesh is None:
            return

        try:
            cm = self.mesh.center_mass
            if cm is not None:
                self.com = cm.tolist()

            inertia = self.mesh.moment_inertia
            if inertia is not None:
                self.inertia = {
                    "ixx": float(inertia[0, 0]),
                    "ixy": float(inertia[0, 1]),
                    "ixz": float(inertia[0, 2]),
                    "iyy": float(inertia[1, 1]),
                    "iyz": float(inertia[1, 2]),
                    "izz": float(inertia[2, 2]),
                }
        except Exception:
            self.com = self.mesh.centroid.tolist()


class Joint:
    def __init__(self, name, parent_link, child_link, joint_type="revolute"):
        self.name = name
        self.parent_link = parent_link
        self.child_link = child_link
        self.joint_type = joint_type
        self.is_gripper = False

        self.origin = np.array([0.0, 0.0, 0.0], dtype=float)
        self.axis = np.array([0.0, 0.0, 1.0], dtype=float)
        self.axis_name = "Z"

        self.min_limit = -180.0
        self.max_limit = 180.0
        self.current_value = 0.0

        parent_link.child_joints.append(self)
        child_link.parent_joint = self

    def get_matrix(self):
        theta = np.radians(self.current_value)
        rotation = self._rotation_matrix(self.axis, theta)
        t_origin = np.eye(4)
        t_origin[:3, 3] = self.origin
        t_origin_inv = np.eye(4)
        t_origin_inv[:3, 3] = -self.origin
        return t_origin @ rotation @ t_origin_inv

    def _rotation_matrix(self, axis, theta):
        axis = np.array(axis, dtype=float)
        axis = axis / (np.linalg.norm(axis) + 1e-9)
        skew = np.array(
            [
                [0.0, -axis[2], axis[1]],
                [axis[2], 0.0, -axis[0]],
                [-axis[1], axis[0], 0.0],
            ],
            dtype=float,
        )
        identity = np.eye(3)
        rot3 = identity + np.sin(theta) * skew + (1.0 - np.cos(theta)) * (skew @ skew)
        ret = np.eye(4)
        ret[:3, :3] = rot3
        return ret


class Robot:
    def __init__(self):
        self.links = {}
        self.joints = {}
        self.base_link = None
        self.joint_relations = {}
        self.workspace_report = None
        self.structure_report = None
        self.ik_experience = {}

    def reset_to_home(self, home_angle=0.0):
        for joint in self.joints.values():
            joint.current_value = home_angle
        self.update_kinematics()

    def add_joint_relation(self, master, slave, ratio=1.0):
        if master not in self.joint_relations:
            self.joint_relations[master] = []
        self.joint_relations[master].append((slave, ratio))

    def add_link(self, name, mesh=None):
        link = Link(name, mesh)
        self.links[name] = link
        return link

    def add_joint(self, name, parent_name, child_name):
        parent = self.links[parent_name]
        child = self.links[child_name]

        if child.parent_joint:
            old_joint = child.parent_joint
            names_to_remove = [joint_name for joint_name, joint in self.joints.items() if joint == old_joint]
            for joint_name in names_to_remove:
                self.remove_joint(joint_name)

        joint = Joint(name, parent, child)
        self.joints[name] = joint
        return joint

    def remove_link(self, name):
        if name not in self.links:
            return

        link = self.links[name]
        to_remove_joints = []
        for joint_name, joint in self.joints.items():
            if joint.parent_link == link:
                joint.child_link.t_offset = joint.child_link.t_world
                to_remove_joints.append(joint_name)
            elif joint.child_link == link:
                to_remove_joints.append(joint_name)

        for joint_name in to_remove_joints:
            joint = self.joints[joint_name]
            if joint in joint.parent_link.child_joints:
                joint.parent_link.child_joints.remove(joint)
            joint.child_link.parent_joint = None
            del self.joints[joint_name]

        del self.links[name]
        if self.base_link == link:
            self.base_link = None

    def remove_joint(self, name):
        if name not in self.joints:
            return

        joint = self.joints[name]
        parent = joint.parent_link
        child = joint.child_link

        if joint in parent.child_joints:
            parent.child_joints.remove(joint)
        child.parent_joint = None

        if name in self.joint_relations:
            del self.joint_relations[name]
        for master, slaves in self.joint_relations.items():
            self.joint_relations[master] = [(slave, ratio) for slave, ratio in slaves if slave != name]

        del self.joints[name]
        self.update_kinematics()

    def update_kinematics(self):
        visited = set()
        roots = [link for link in self.links.values() if link.parent_joint is None]
        if self.base_link and self.base_link in roots:
            roots.remove(self.base_link)
            roots.insert(0, self.base_link)

        for root in roots:
            if root.name in visited:
                continue
            root.t_world = root.t_offset
            visited.add(root.name)
            stack = [root]
            while stack:
                parent = stack.pop()
                for joint in parent.child_joints:
                    child = joint.child_link
                    if child.name in visited:
                        continue
                    child.t_world = parent.t_world @ joint.get_matrix() @ child.t_offset
                    visited.add(child.name)
                    stack.append(child)

    def set_tcp_transform(self, link_name, position=None, rpy_deg=None):
        if link_name not in self.links:
            return False
        link = self.links[link_name]
        if position is not None:
            link.custom_tcp_offset = np.array(position, dtype=float)
        if rpy_deg is not None:
            link.custom_tcp_rpy_deg = list(np.array(rpy_deg, dtype=float))
        return True

    def get_tcp_local_transform(self, tcp_link):
        if tcp_link is None:
            return np.eye(4)
        offset = getattr(tcp_link, "custom_tcp_offset", None)
        if offset is None:
            offset = np.zeros(3, dtype=float)
        rpy_deg = getattr(tcp_link, "custom_tcp_rpy_deg", [0.0, 0.0, 0.0])
        return transform_from_pose(offset, rpy_deg)

    def get_flange_world_pose(self, tcp_link):
        self.update_kinematics()
        if tcp_link is None:
            return np.eye(4)
        return tcp_link.t_world.copy()

    def get_tcp_world_pose(self, tcp_link):
        return self.get_flange_world_pose(tcp_link) @ self.get_tcp_local_transform(tcp_link)

    def get_target_flange_pose(self, target_tcp_pose, tcp_link):
        return np.array(target_tcp_pose, dtype=float) @ invert_transform(self.get_tcp_local_transform(tcp_link))

    def compute_wrist_center(self, target_tcp_pose, tcp_link):
        flange_pose = self.get_target_flange_pose(target_tcp_pose, tcp_link)
        return flange_pose[:3, 3].copy()

    def get_kinematic_chain(self, tcp_link):
        chain = []
        current = tcp_link
        while current is not None and current.parent_joint is not None:
            is_slave = any(
                any(slave_id == current.parent_joint.name for slave_id, _ in slaves)
                for _, slaves in self.joint_relations.items()
            )
            if not is_slave:
                chain.append(current.parent_joint)
            current = current.parent_joint.parent_link
        return list(reversed(chain))

    def get_ikpy_chain(self, tcp_link):
        """Constructs an ikpy.chain.Chain dynamically from the robot's current state."""
        if ikpy is None:
            return None
            
        ik_links = []
        
        # 1. Base Link
        base = self.base_link or list(self.links.values())[0]
        ik_links.append(ikpy.link.Link(
            name=base.name,
            static_transform=base.t_offset
        ))
        
        # 2. Chain Links
        chain = self.get_kinematic_chain(tcp_link)
        for joint in chain:
            # Joint transform
            ik_links.append(ikpy.link.URDFLink(
                name=joint.name,
                origin_translation=joint.origin,
                origin_orientation=[0.0, 0.0, 0.0], # Already handled by DH or origin
                rotation=joint.axis,
                bounds=(np.radians(joint.min_limit), np.radians(joint.max_limit))
            ))
            
            # Child link offset
            ik_links.append(ikpy.link.Link(
                name=f"{joint.child_link.name}_offset",
                static_transform=joint.child_link.t_offset
            ))
            
        # 3. TCP Offset
        tcp_offset = self.get_tcp_local_transform(tcp_link)
        ik_links.append(ikpy.link.Link(
            name="TCP_FRAME",
            static_transform=tcp_offset
        ))
        
        return ikpy.chain.Chain(links=ik_links)

    def _joint_meta_maps(self, joint_meta=None):
        joint_meta = joint_meta or {}
        by_child = {}
        by_joint_id = {}
        for child_name, data in joint_meta.items():
            by_child[child_name] = data
            joint_id = data.get("joint_id", child_name)
            by_joint_id[joint_id] = data
        return by_child, by_joint_id

    def _resolve_joint_meta(self, joint, joint_meta=None):
        by_child, by_joint_id = self._joint_meta_maps(joint_meta)
        return by_joint_id.get(joint.name, by_child.get(joint.child_link.name, {}))

    def _infer_dh_row(self, joint, meta=None, length_scale=1.0):
        meta = meta or {}
        t_offset = joint.child_link.t_offset
        px = t_offset[0, 3] / max(length_scale, 1e-9)
        py = t_offset[1, 3] / max(length_scale, 1e-9)
        pz = t_offset[2, 3] / max(length_scale, 1e-9)

        row = {
            "theta0_deg": float(np.degrees(np.arctan2(py, px))) if abs(px) > 1e-9 or abs(py) > 1e-9 else 0.0,
            "d": float(pz),
            "a": float(np.hypot(px, py)),
            "alpha_deg": float(np.degrees(np.arctan2(t_offset[2, 1], t_offset[2, 2]))),
            "joint_type": str(meta.get("joint_type", joint.joint_type)).lower(),
            "q_value": float(joint.current_value),
            "title": meta.get("custom_name", joint.name),
            "joint_name": joint.name,
            "child_name": joint.child_link.name,
        }

        if "dh_theta" in meta:
            row["theta0_deg"] = float(meta["dh_theta"])
        if "dh_d" in meta:
            row["d"] = float(meta["dh_d"])
        if "dh_a" in meta:
            row["a"] = float(meta["dh_a"])
        if "dh_alpha" in meta:
            row["alpha_deg"] = float(meta["dh_alpha"])
        return row

    def resolve_dh_rows(self, tcp_link, joint_meta=None, length_scale=1.0):
        rows = []
        for joint in self.get_kinematic_chain(tcp_link):
            rows.append(self._infer_dh_row(joint, self._resolve_joint_meta(joint, joint_meta), length_scale))
        return rows

    def forward_kinematics(self, tcp_link, joint_values_deg=None, joint_meta=None, length_scale=1.0):
        rows = self.resolve_dh_rows(tcp_link, joint_meta=joint_meta, length_scale=length_scale)
        if joint_values_deg is not None:
            values = list(joint_values_deg)
            for idx, row in enumerate(rows):
                if idx < len(values):
                    row["q_value"] = float(values[idx])
        chain = self.get_kinematic_chain(tcp_link)
        actual_fk = self._evaluate_chain_fk(chain, tcp_link, joint_values_deg=joint_values_deg)
        transforms = []
        for idx, row in enumerate(rows):
            local = compute_standard_dh_matrix(
                theta_deg=row["theta0_deg"],
                d=row["d"],
                a=row["a"],
                alpha_deg=row["alpha_deg"],
                q_value=row["q_value"],
                joint_type=row["joint_type"],
            )
            cumulative = actual_fk["joint_transforms"][idx]["cumulative"] if idx < len(actual_fk["joint_transforms"]) else local
            transforms.append(
                {
                    "title": row["title"],
                    "joint_name": row["joint_name"],
                    "child_name": row["child_name"],
                    "local": local,
                    "cumulative": cumulative.copy(),
                    "dh": dict(row),
                }
            )

        flange_pose = actual_fk["flange_pose"]
        tcp_local = self.get_tcp_local_transform(tcp_link)
        tcp_pose = actual_fk["tcp_pose"]
        return {
            "dh_rows": rows,
            "joint_transforms": transforms,
            "flange_pose": flange_pose,
            "tcp_pose": tcp_pose,
            "tcp_local": tcp_local,
            "wrist_center": flange_pose[:3, 3].copy(),
            "flange_pose_dict": pose_dict_from_transform(flange_pose),
            "tcp_pose_dict": pose_dict_from_transform(tcp_pose),
        }

    def _apply_joint_vector(self, chain, values_deg):
        values_deg = np.array(values_deg, dtype=float)
        for joint, value in zip(chain, values_deg):
            clamped = np.clip(value, joint.min_limit, joint.max_limit)
            joint.current_value = clamped
            if joint.name in self.joint_relations:
                for slave_id, ratio in self.joint_relations[joint.name]:
                    if slave_id in self.joints:
                        self.joints[slave_id].current_value = np.clip(
                            clamped * ratio,
                            self.joints[slave_id].min_limit,
                            self.joints[slave_id].max_limit,
                        )
        self.update_kinematics()

    def _current_joint_vector(self, chain):
        return np.array([joint.current_value for joint in chain], dtype=float)

    def _joint_bounds(self, chain):
        mins = np.array([joint.min_limit for joint in chain], dtype=float)
        maxs = np.array([joint.max_limit for joint in chain], dtype=float)
        return mins, maxs

    def set_joint_value(self, joint_name, value, propagate_relations=True):
        if joint_name not in self.joints:
            return False

        joint = self.joints[joint_name]
        joint.current_value = float(np.clip(value, joint.min_limit, joint.max_limit))

        if propagate_relations:
            if joint_name in self.joint_relations:
                for slave_id, ratio in self.joint_relations[joint_name]:
                    if slave_id in self.joints:
                        slave_joint = self.joints[slave_id]
                        slave_joint.current_value = float(
                            np.clip(
                                joint.current_value * ratio,
                                slave_joint.min_limit,
                                slave_joint.max_limit,
                            )
                        )
            else:
                for master_id, slaves in self.joint_relations.items():
                    for slave_id, ratio in slaves:
                        if slave_id == joint_name and abs(ratio) > 1e-9 and master_id in self.joints:
                            master_joint = self.joints[master_id]
                            master_joint.current_value = float(
                                np.clip(
                                    joint.current_value / ratio,
                                    master_joint.min_limit,
                                    master_joint.max_limit,
                                )
                            )
                            for sibling_id, sibling_ratio in self.joint_relations.get(master_id, []):
                                if sibling_id in self.joints:
                                    sibling_joint = self.joints[sibling_id]
                                    sibling_joint.current_value = float(
                                        np.clip(
                                            master_joint.current_value * sibling_ratio,
                                            sibling_joint.min_limit,
                                            sibling_joint.max_limit,
                                        )
                                    )
                            break

        self.update_kinematics()
        return True

    def _seed_joint_vectors(self, chain):
        if not chain:
            return []

        mins, maxs = self._joint_bounds(chain)
        current = self._current_joint_vector(chain)
        middle = 0.5 * (mins + maxs)
        zeros = np.clip(np.zeros(len(chain), dtype=float), mins, maxs)
        span = np.maximum(maxs - mins, 0.0)

        candidates = [
            current,
            zeros,
            middle,
            np.clip(current + 0.18 * span, mins, maxs),
            np.clip(current - 0.18 * span, mins, maxs),
            np.clip(middle + 0.32 * span, mins, maxs),
            np.clip(middle - 0.32 * span, mins, maxs),
        ]

        unique = []
        for candidate in candidates:
            if not any(np.allclose(candidate, seen, atol=1e-6) for seen in unique):
                unique.append(candidate.copy())
        return unique

    def _chain_signature(self, chain, tcp_link):
        chain_names = tuple(joint.name for joint in chain)
        tcp_name = tcp_link.name if tcp_link is not None else None
        return chain_names, tcp_name

    def _merge_seed_lists(self, *seed_lists):
        merged = []
        for seed_list in seed_lists:
            for candidate in seed_list or []:
                candidate = np.array(candidate, dtype=float)
                if not any(np.allclose(candidate, seen, atol=1e-6) for seen in merged):
                    merged.append(candidate.copy())
        return merged

    def _workspace_seed_vectors(self, target_tcp_pose, chain, tcp_link, top_k=6):
        report = self.workspace_report
        if not report or not report.get("ok"):
            return []
        if report.get("tcp_link") != getattr(tcp_link, "name", None):
            return []
        configs = report.get("joint_configs")
        effort_scores = report.get("joint_effort_scores")
        points = report.get("points")
        if configs is None or points is None or len(configs) == 0 or len(points) == 0:
            return []
        if len(configs[0]) != len(chain):
            return []

        target_pos = np.array(target_tcp_pose, dtype=float)[:3, 3]
        query_k = min(max(top_k * 2, top_k), len(points))
        if report.get("kdtree") is not None:
            distances, indices = report["kdtree"].query(target_pos, k=query_k)
            distances = np.atleast_1d(distances).tolist()
            indices = np.atleast_1d(indices).tolist()
        else:
            dists = np.linalg.norm(points - target_pos, axis=1)
            indices = np.argsort(dists)[:query_k].tolist()
            distances = [float(dists[idx]) for idx in indices]

        ranked = []
        for dist, idx in zip(distances, indices):
            effort = 0.0 if effort_scores is None else float(effort_scores[idx])
            ranked.append((float(dist), effort, int(idx)))
        ranked.sort(key=lambda item: (item[0], item[1]))

        return [np.array(configs[idx], dtype=float) for _, _, idx in ranked[: min(top_k, len(ranked))]]

    def _experience_seed_vectors(self, target_tcp_pose, chain, tcp_link, top_k=6):
        key = self._chain_signature(chain, tcp_link)
        memory = self.ik_experience.get(key)
        if not memory:
            return []

        positions = memory.get("positions")
        configs = memory.get("configs")
        if positions is None or configs is None or len(positions) == 0:
            return []

        target_pos = np.array(target_tcp_pose, dtype=float)[:3, 3]
        if memory.get("kdtree") is not None:
            distances, indices = memory["kdtree"].query(target_pos, k=min(top_k, len(positions)))
            indices = np.atleast_1d(indices).tolist()
        else:
            dists = np.linalg.norm(positions - target_pos, axis=1)
            indices = np.argsort(dists)[: min(top_k, len(positions))].tolist()

        return [np.array(configs[idx], dtype=float) for idx in indices]

    def _rebuild_memory_index(self, memory):
        positions = memory.get("positions")
        if positions is None or len(positions) == 0 or cKDTree is None:
            memory["kdtree"] = None
            return
        memory["kdtree"] = cKDTree(np.array(positions, dtype=float))

    def remember_ik_solution(self, chain, tcp_link, tcp_pose, joint_vector, max_memory=300):
        key = self._chain_signature(chain, tcp_link)
        memory = self.ik_experience.setdefault(
            key,
            {"positions": [], "configs": [], "poses": [], "kdtree": None},
        )

        pos = np.array(tcp_pose, dtype=float)[:3, 3].copy()
        cfg = np.array(joint_vector, dtype=float).copy()
        poses = memory["poses"]
        positions = memory["positions"]
        configs = memory["configs"]

        if positions:
            dists = np.linalg.norm(np.array(positions, dtype=float) - pos, axis=1)
            best_idx = int(np.argmin(dists))
            if dists[best_idx] < 1e-6:
                configs[best_idx] = cfg
                poses[best_idx] = np.array(tcp_pose, dtype=float).copy()
                self._rebuild_memory_index(memory)
                return

        positions.append(pos)
        configs.append(cfg)
        poses.append(np.array(tcp_pose, dtype=float).copy())
        if len(positions) > max_memory:
            positions.pop(0)
            configs.pop(0)
            poses.pop(0)
        self._rebuild_memory_index(memory)

    def _joint_axis_world(self, joint):
        axis_local = np.array(joint.axis, dtype=float)
        norm = np.linalg.norm(axis_local)
        if norm < 1e-9:
            return np.array([0.0, 0.0, 1.0], dtype=float)
        axis_local = axis_local / norm
        axis_world = joint.parent_link.t_world[:3, :3] @ axis_local
        axis_world_norm = np.linalg.norm(axis_world)
        if axis_world_norm < 1e-9:
            return np.array([0.0, 0.0, 1.0], dtype=float)
        return axis_world / axis_world_norm

    def _joint_origin_world(self, joint):
        return (joint.parent_link.t_world @ np.append(np.array(joint.origin, dtype=float), 1.0))[:3]

    def _subtree_links(self, root_link):
        if root_link is None:
            return []

        result = []
        stack = [root_link]
        visited = set()
        while stack:
            link = stack.pop()
            if link is None or link.name in visited:
                continue
            visited.add(link.name)
            result.append(link)
            for child_joint in link.child_joints:
                if child_joint.child_link is not None:
                    stack.append(child_joint.child_link)
        return result

    def _link_com_world(self, link, length_scale=1.0):
        local_com = np.array(getattr(link, "com", [0.0, 0.0, 0.0]), dtype=float)
        world = (link.t_world @ np.append(local_com, 1.0))[:3]
        return world / max(length_scale, 1e-9)

    def _robot_com_world(self, length_scale=1.0):
        total_mass = 0.0
        weighted = np.zeros(3, dtype=float)
        for link in self.links.values():
            mass = max(float(getattr(link, "mass", 0.0)), 0.0)
            if mass <= 0.0:
                continue
            com_world = self._link_com_world(link, length_scale=length_scale)
            weighted += mass * com_world
            total_mass += mass
        if total_mass <= 1e-12:
            return np.zeros(3, dtype=float), 0.0
        return weighted / total_mass, total_mass

    def compute_static_joint_loads(self, joint_names=None, length_scale=1.0, gravity_cm_s2=None):
        self.update_kinematics()
        gravity = np.array([0.0, 0.0, -9.81] if gravity_cm_s2 is None else gravity_cm_s2, dtype=float)
        selected = []
        if joint_names is None:
            selected = list(self.joints.values())
        else:
            for joint_name in joint_names:
                if joint_name in self.joints:
                    selected.append(self.joints[joint_name])

        loads = []
        for joint in selected:
            axis_world = self._joint_axis_world(joint)
            origin_cm = self._joint_origin_world(joint) / max(length_scale, 1e-9)
            subtree_links = self._subtree_links(joint.child_link)
            subtree_mass = 0.0
            subtree_weighted_com = np.zeros(3, dtype=float)
            torque_vec = np.zeros(3, dtype=float)

            for link in subtree_links:
                mass = max(float(getattr(link, "mass", 0.0)), 0.0)
                if mass <= 0.0:
                    continue
                com_world_cm = self._link_com_world(link, length_scale=length_scale)
                subtree_weighted_com += mass * com_world_cm
                subtree_mass += mass
                force = mass * gravity
                torque_vec += np.cross(com_world_cm - origin_cm, force)

            subtree_com = (
                subtree_weighted_com / subtree_mass if subtree_mass > 1e-12 else origin_cm.copy()
            )
            axis_torque = float(np.dot(torque_vec, axis_world))
            bending_vec = torque_vec - axis_torque * axis_world
            resultant_torque = float(np.linalg.norm(torque_vec))
            lever_arm_cm = float(np.linalg.norm(subtree_com - origin_cm))

            loads.append(
                {
                    "joint_name": joint.name,
                    "joint_type": joint.joint_type,
                    "axis_world": axis_world.copy(),
                    "origin_cm": origin_cm.copy(),
                    "subtree_mass": float(subtree_mass),
                    "subtree_com_cm": subtree_com.copy(),
                    "gravity_torque_vector_ncm": torque_vec.copy(),
                    "axis_torque_ncm": axis_torque,
                    "abs_axis_torque_ncm": float(abs(axis_torque)),
                    "resultant_torque_ncm": resultant_torque,
                    "bending_torque_ncm": float(np.linalg.norm(bending_vec)),
                    "lever_arm_cm": lever_arm_cm,
                }
            )
        return loads

    def _config_effort_score(self, chain, joint_vector, length_scale=1.0, gravity_cm_s2=None):
        old_values = {name: joint.current_value for name, joint in self.joints.items()}
        try:
            self._apply_joint_vector(chain, joint_vector)
            loads = self.compute_static_joint_loads(
                joint_names=[joint.name for joint in chain],
                length_scale=length_scale,
                gravity_cm_s2=gravity_cm_s2,
            )
            return float(sum(item["resultant_torque_ncm"] for item in loads))
        finally:
            for name, value in old_values.items():
                self.joints[name].current_value = value
            self.update_kinematics()

    def compute_structure_dynamics(self, tcp_link=None, workspace_report=None, length_scale=1.0, gravity_cm_s2=None):
        self.update_kinematics()
        gravity = np.array([0.0, 0.0, -9.81] if gravity_cm_s2 is None else gravity_cm_s2, dtype=float)
        total_com_cm, total_mass = self._robot_com_world(length_scale=length_scale)
        base_pos_cm = (
            self.base_link.t_world[:3, 3] / max(length_scale, 1e-9)
            if self.base_link is not None
            else np.zeros(3, dtype=float)
        )
        current_joint_loads = self.compute_static_joint_loads(length_scale=length_scale, gravity_cm_s2=gravity)

        chain = self.get_kinematic_chain(tcp_link) if tcp_link is not None else []
        sampled_joint_loads = {}
        report_workspace = workspace_report or self.workspace_report

        if chain and report_workspace and report_workspace.get("ok") and report_workspace.get("joint_configs") is not None:
            old_values = {name: joint.current_value for name, joint in self.joints.items()}
            try:
                chain_joint_names = [joint.name for joint in chain]
                accum = {
                    joint.name: {
                        "max_abs_axis_torque_ncm": 0.0,
                        "mean_abs_axis_torque_ncm": 0.0,
                        "max_resultant_torque_ncm": 0.0,
                        "mean_resultant_torque_ncm": 0.0,
                        "max_bending_torque_ncm": 0.0,
                        "worst_config": None,
                    }
                    for joint in chain
                }
                sample_count = 0
                for config in report_workspace["joint_configs"]:
                    self._apply_joint_vector(chain, np.array(config, dtype=float))
                    loads = self.compute_static_joint_loads(
                        joint_names=chain_joint_names,
                        length_scale=length_scale,
                        gravity_cm_s2=gravity,
                    )
                    sample_count += 1
                    for load in loads:
                        slot = accum[load["joint_name"]]
                        slot["mean_abs_axis_torque_ncm"] += load["abs_axis_torque_ncm"]
                        slot["mean_resultant_torque_ncm"] += load["resultant_torque_ncm"]
                        if load["abs_axis_torque_ncm"] > slot["max_abs_axis_torque_ncm"]:
                            slot["max_abs_axis_torque_ncm"] = load["abs_axis_torque_ncm"]
                            slot["worst_config"] = np.array(config, dtype=float).copy()
                        slot["max_resultant_torque_ncm"] = max(
                            slot["max_resultant_torque_ncm"],
                            load["resultant_torque_ncm"],
                        )
                        slot["max_bending_torque_ncm"] = max(
                            slot["max_bending_torque_ncm"],
                            load["bending_torque_ncm"],
                        )
                for joint_name, slot in accum.items():
                    if sample_count:
                        slot["mean_abs_axis_torque_ncm"] /= float(sample_count)
                        slot["mean_resultant_torque_ncm"] /= float(sample_count)
                sampled_joint_loads = accum
            finally:
                for name, value in old_values.items():
                    self.joints[name].current_value = value
                self.update_kinematics()

        report = {
            "ok": True,
            "gravity_cm_s2": gravity.copy(),
            "total_mass": float(total_mass),
            "total_com_cm": total_com_cm.copy(),
            "base_position_cm": base_pos_cm.copy(),
            "base_balance_offset_cm": (total_com_cm - base_pos_cm).copy(),
            "current_joint_loads": current_joint_loads,
            "sampled_joint_loads": sampled_joint_loads,
            "tcp_link": None if tcp_link is None else tcp_link.name,
        }
        self.structure_report = report
        return report

    def get_workspace_target_hint(self, target_pos, tcp_link):
        report = self.workspace_report
        if not report or not report.get("ok"):
            return {"ok": False, "reason": "missing_workspace"}
        if report.get("tcp_link") != getattr(tcp_link, "name", None):
            return {"ok": False, "reason": "workspace_tcp_mismatch"}

        points = np.array(report.get("points", []), dtype=float)
        if len(points) == 0:
            return {"ok": False, "reason": "empty_workspace_points"}

        target = np.array(target_pos, dtype=float)
        base_position = np.array(report.get("base_position", np.zeros(3)), dtype=float)
        target_vec = target - base_position
        target_radius = float(np.linalg.norm(target_vec))

        if report.get("kdtree") is not None:
            nearest_distance, nearest_index = report["kdtree"].query(target, k=1)
            nearest_distance = float(nearest_distance)
            nearest_index = int(nearest_index)
        else:
            dists = np.linalg.norm(points - target, axis=1)
            nearest_index = int(np.argmin(dists))
            nearest_distance = float(dists[nearest_index])

        nearest_point = points[nearest_index].copy()
        sample_spacing = float(report.get("sample_spacing", 0.0) or 0.0)
        tolerance = max(sample_spacing * 1.5, report.get("radius_max", 0.0) * 0.03, 1e-6)

        directional_limit = float(report.get("radius_max", 0.0))
        directional = report.get("directional_reach", {})
        if directional.get("ok") and target_radius > 1e-9:
            direction = target_vec / target_radius
            best_alignment = -np.inf
            best_reach = directional_limit
            for sample in directional.get("samples", []):
                sample_dir = np.array(sample.get("direction", [1.0, 0.0, 0.0]), dtype=float)
                alignment = float(np.dot(direction, sample_dir))
                if alignment > best_alignment:
                    best_alignment = alignment
                    best_reach = float(sample.get("reach_radius", directional_limit))
            directional_limit = best_reach

        inside_workspace = (
            nearest_distance <= tolerance
            or target_radius <= (directional_limit + tolerance)
        )

        return {
            "ok": True,
            "nearest_point": nearest_point,
            "nearest_distance": nearest_distance,
            "sample_spacing": sample_spacing,
            "tolerance": tolerance,
            "target_radius": target_radius,
            "directional_limit": directional_limit,
            "inside_workspace": bool(inside_workspace),
        }

    def _compute_pose_jacobian(self, chain, tcp_link, use_tcp=True):
        pose = self.get_tcp_world_pose(tcp_link) if use_tcp else self.get_flange_world_pose(tcp_link)
        point_world = pose[:3, 3]
        jac = np.zeros((6, len(chain)), dtype=float)

        for idx, joint in enumerate(chain):
            axis_world = self._joint_axis_world(joint)
            if (joint.joint_type or "revolute").lower() == "prismatic":
                jac[:3, idx] = axis_world
                jac[3:, idx] = 0.0
            else:
                origin_world = self._joint_origin_world(joint)
                jac[:3, idx] = np.cross(axis_world, point_world - origin_world)
                jac[3:, idx] = axis_world

        return jac

    def _ik_residual(self, chain, tcp_link, target_tcp_pose, joint_vector, orientation_weight=0.35):
        self._apply_joint_vector(chain, joint_vector)
        err6 = pose_error(target_tcp_pose, self.get_tcp_world_pose(tcp_link))
        if orientation_weight <= 1e-9:
            return err6[:3]
        return np.concatenate([err6[:3], orientation_weight * err6[3:]])

    def _least_squares_ik_solve(
        self,
        chain,
        tcp_link,
        target_tcp_pose,
        seed,
        mins,
        maxs,
        orientation_weight=0.35,
        max_nfev=120,
        reference_vector=None,
        joint_change_weight=0.0,
    ):
        if least_squares is None:
            return None

        x0 = np.clip(np.array(seed, dtype=float), mins, maxs)
        if reference_vector is None:
            reference_vector = x0.copy()
        else:
            reference_vector = np.clip(np.array(reference_vector, dtype=float), mins, maxs)
        joint_span = np.maximum(maxs - mins, 1.0)

        def residual(vec):
            base_residual = self._ik_residual(
                chain,
                tcp_link,
                target_tcp_pose,
                np.clip(vec, mins, maxs),
                orientation_weight=orientation_weight,
            )
            if joint_change_weight <= 1e-12:
                return base_residual
            motion_residual = np.sqrt(joint_change_weight) * ((np.clip(vec, mins, maxs) - reference_vector) / joint_span)
            return np.concatenate([base_residual, motion_residual])

        result = least_squares(
            residual,
            x0=x0,
            bounds=(mins, maxs),
            method="trf",
            x_scale="jac",
            loss="linear",
            max_nfev=max_nfev,
            ftol=1e-6,
            xtol=1e-6,
            gtol=1e-6,
        )

        solved = np.clip(np.array(result.x, dtype=float), mins, maxs)
        self._apply_joint_vector(chain, solved)
        err6 = pose_error(target_tcp_pose, self.get_tcp_world_pose(tcp_link))
        motion_score = float(np.linalg.norm((solved - reference_vector) / joint_span))
        score = float(
            np.linalg.norm(err6[:3])
            + (orientation_weight * np.linalg.norm(err6[3:]))
            + (joint_change_weight * motion_score)
        )
        return {
            "vector": solved,
            "err6": err6,
            "score": score,
            "nfev": int(getattr(result, "nfev", 0)),
            "success": bool(getattr(result, "success", False)),
            "motion_score": motion_score,
        }

    def _evaluate_chain_fk(self, chain, tcp_link, joint_values_deg=None):
        old_values = {name: joint.current_value for name, joint in self.joints.items()}
        try:
            if joint_values_deg is not None:
                self._apply_joint_vector(chain, joint_values_deg)
            else:
                self.update_kinematics()

            root_link = chain[0].parent_link if chain else tcp_link
            root_world = root_link.t_world.copy() if root_link is not None else np.eye(4)
            root_inv = invert_transform(root_world)

            transforms = []
            for joint in chain:
                child_world = joint.child_link.t_world.copy()
                transforms.append(
                    {
                        "title": joint.name,
                        "joint_name": joint.name,
                        "child_name": joint.child_link.name,
                        "local": joint.get_matrix().copy(),
                        "cumulative": (root_inv @ child_world),
                    }
                )

            return {
                "joint_transforms": transforms,
                "flange_pose": root_inv @ self.get_flange_world_pose(tcp_link),
                "tcp_pose": root_inv @ self.get_tcp_world_pose(tcp_link),
                "root_pose": root_world,
            }
        finally:
            if joint_values_deg is not None:
                for name, value in old_values.items():
                    self.joints[name].current_value = value
                self.update_kinematics()

    def diagnose_ik_setup(self, tcp_link=None, target_pos=None):
        issues = []
        if not self.links:
            issues.append("No links are loaded into the robot model.")
            return issues
        if self.base_link is None:
            issues.append("Base link is not set, so the kinematic chain has no fixed reference frame.")
        if not self.joints:
            issues.append("No joints are defined, so XYZ targets cannot be converted into joint motion.")
            return issues
        if tcp_link is None:
            issues.append("Tool center point (TCP) link is missing.")
            return issues
        if tcp_link.name not in self.links:
            issues.append(f"TCP link '{tcp_link.name}' is not registered in the robot model.")
            return issues

        chain = self.get_kinematic_chain(tcp_link)
        if not chain:
            issues.append("TCP is not connected to the base through any active joints.")
            return issues

        for joint in chain:
            if np.linalg.norm(joint.axis) < 1e-8:
                issues.append(f"Joint '{joint.name}' has an invalid rotation axis.")
            if joint.max_limit < joint.min_limit:
                issues.append(f"Joint '{joint.name}' has reversed limits (max < min).")
            if abs(joint.max_limit - joint.min_limit) < 1e-8:
                issues.append(f"Joint '{joint.name}' is locked because its min and max limits are identical.")

        tcp_tf = self.get_tcp_local_transform(tcp_link)
        if not np.allclose(tcp_tf[3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-9):
            issues.append("TCP transform is malformed.")

        if target_pos is not None:
            try:
                target = np.array(target_pos, dtype=float)
                root_pos = chain[0].parent_link.t_world[:3, 3]
                hint = self.get_workspace_target_hint(target, tcp_link)
                if hint.get("ok"):
                    if not hint["inside_workspace"]:
                        issues.append(
                            "Target is outside the sampled reachable workspace "
                            f"(distance {hint['target_radius']:.2f}, directional reach {hint['directional_limit']:.2f}, "
                            f"nearest reachable offset {hint['nearest_distance']:.2f})."
                        )
                else:
                    approx_reach = 0.0
                    for joint in chain:
                        approx_reach += np.linalg.norm(joint.child_link.t_offset[:3, 3] - joint.origin)
                    approx_reach += np.linalg.norm(self.get_tcp_local_transform(tcp_link)[:3, 3])
                    dist = np.linalg.norm(target - root_pos)
                    if approx_reach > 1e-6 and dist > approx_reach * 1.35:
                        issues.append(
                            f"Target is likely outside the reachable workspace (distance {dist:.2f}, approximate reach {approx_reach:.2f})."
                        )
            except Exception:
                pass

        return issues

    def inverse_kinematics_pose(
        self,
        target_tcp_pose,
        tcp_link,
        max_iters=300,
        position_tolerance=0.3,
        orientation_tolerance=0.05,
        orientation_weight=0.35,
        joint_change_weight=0.12,
    ):
        if tcp_link is None:
            return False, {"reason": "missing_tcp"}

        target_tcp_pose = np.array(target_tcp_pose, dtype=float)
        wrist_center = self.compute_wrist_center(target_tcp_pose, tcp_link)
        chain = self.get_kinematic_chain(tcp_link)
        if not chain:
            return False, {"reason": "empty_chain"}

        old_values = {name: joint.current_value for name, joint in self.joints.items()}
        mins, maxs = self._joint_bounds(chain)
        reference_vector = self._current_joint_vector(chain)
        joint_span = np.maximum(maxs - mins, 1.0)
        best_vector = reference_vector.copy()
        best_score = np.inf
        weights = np.array([1.0, 1.0, 1.0, orientation_weight, orientation_weight, orientation_weight], dtype=float)
        best_err6 = None
        converged = False

        try:
            seeds = self._merge_seed_lists(
                self._experience_seed_vectors(target_tcp_pose, chain, tcp_link, top_k=8),
                self._workspace_seed_vectors(target_tcp_pose, chain, tcp_link, top_k=8),
                self._seed_joint_vectors(chain),
            )
            iters_per_seed = max(20, int(np.ceil(max_iters / max(len(seeds), 1))))
            optimizer_calls = 0

            for seed in seeds:
                current = np.clip(np.array(seed, dtype=float), mins, maxs)
                lam = 5e-2

                lsq_result = self._least_squares_ik_solve(
                    chain,
                    tcp_link,
                    target_tcp_pose,
                    current,
                    mins,
                    maxs,
                    orientation_weight=orientation_weight,
                    max_nfev=max(60, iters_per_seed * 4),
                    reference_vector=reference_vector,
                    joint_change_weight=joint_change_weight,
                )
                if lsq_result is not None:
                    optimizer_calls += 1
                    current = lsq_result["vector"].copy()
                    pos_err = np.linalg.norm(lsq_result["err6"][:3])
                    rot_err = np.linalg.norm(lsq_result["err6"][3:])
                    score = lsq_result["score"]
                    if score < best_score:
                        best_score = score
                        best_vector = current.copy()
                        best_err6 = lsq_result["err6"].copy()
                    if pos_err <= position_tolerance and (
                        orientation_weight <= 1e-9 or rot_err <= orientation_tolerance
                    ):
                        best_vector = current.copy()
                        best_err6 = lsq_result["err6"].copy()
                        converged = True
                        break

                for _ in range(iters_per_seed):
                    self._apply_joint_vector(chain, current)
                    tcp_pose = self.get_tcp_world_pose(tcp_link)
                    err6 = pose_error(target_tcp_pose, tcp_pose)
                    weighted_err = weights * err6

                    pos_err = np.linalg.norm(err6[:3])
                    rot_err = np.linalg.norm(err6[3:])
                    motion_score = np.linalg.norm((current - reference_vector) / joint_span)
                    score = pos_err + (orientation_weight * rot_err) + (joint_change_weight * motion_score)
                    if score < best_score:
                        best_score = score
                        best_vector = current.copy()
                        best_err6 = err6.copy()

                    if pos_err <= position_tolerance and rot_err <= orientation_tolerance:
                        best_vector = current.copy()
                        best_err6 = err6.copy()
                        converged = True
                        break

                    J = self._compute_pose_jacobian(chain, tcp_link, use_tcp=True)
                    Jw = weights[:, None] * J
                    regularizer = np.zeros(len(chain), dtype=float)
                    if joint_change_weight > 1e-12:
                        regularizer = joint_change_weight / (joint_span ** 2)
                    lhs = Jw.T @ Jw + (lam * lam) * np.eye(len(chain)) + np.diag(regularizer)
                    rhs = Jw.T @ weighted_err + (regularizer * (reference_vector - current))
                    step = np.linalg.solve(lhs, rhs)
                    step = np.clip(step, -6.0, 6.0)

                    trial = np.clip(current + step, mins, maxs)
                    self._apply_joint_vector(chain, trial)
                    trial_err6 = pose_error(target_tcp_pose, self.get_tcp_world_pose(tcp_link))
                    trial_motion_score = np.linalg.norm((trial - reference_vector) / joint_span)
                    trial_score = (
                        np.linalg.norm(trial_err6[:3])
                        + (orientation_weight * np.linalg.norm(trial_err6[3:]))
                        + (joint_change_weight * trial_motion_score)
                    )
                    if trial_score < score:
                        current = trial
                        lam = max(lam * 0.7, 1e-4)
                    else:
                        current = np.clip(current + (0.2 * step), mins, maxs)
                        lam = min(lam * 1.8, 25.0)

                if converged:
                    break

            self._apply_joint_vector(chain, best_vector)
            flange_pose = self.get_flange_world_pose(tcp_link)
            tcp_pose = self.get_tcp_world_pose(tcp_link)
            target_flange_pose = self.get_target_flange_pose(target_tcp_pose, tcp_link)
            final_err = pose_error(target_tcp_pose, tcp_pose)
            success = converged or (
                np.linalg.norm(final_err[:3]) <= position_tolerance
                and np.linalg.norm(final_err[3:]) <= orientation_tolerance
            )
            if success:
                self.remember_ik_solution(chain, tcp_link, tcp_pose, best_vector)
            return success, {
                "wrist_center": wrist_center,
                "flange_target": target_flange_pose,
                "flange_pose": flange_pose,
                "tcp_pose": tcp_pose,
                "joint_values": {joint.name: joint.current_value for joint in chain},
                "position_error": float(np.linalg.norm(final_err[:3])),
                "orientation_error": float(np.linalg.norm(final_err[3:])),
                "fk_validation_pose": tcp_pose.copy(),
                "best_score": float(best_score),
                "motion_score": float(np.linalg.norm((best_vector - reference_vector) / joint_span)),
                "joint_motion_deg": {joint.name: float(best_vector[idx] - reference_vector[idx]) for idx, joint in enumerate(chain)},
                "seed_count": len(seeds),
                "memory_seed_count": len(self._experience_seed_vectors(target_tcp_pose, chain, tcp_link, top_k=8)),
                "workspace_seed_count": len(self._workspace_seed_vectors(target_tcp_pose, chain, tcp_link, top_k=8)),
                "optimizer_calls": optimizer_calls,
                "best_error_vector": None if best_err6 is None else best_err6.copy(),
            }
        except Exception:
            for name, value in old_values.items():
                self.joints[name].current_value = value
            self.update_kinematics()
            raise

    def inverse_kinematics(self, target_pos, tcp_link, max_iters=300, tolerance=0.3, tool_offset=None):
        target_pos = np.array(target_pos, dtype=float)
        target_tcp_pose = self.get_tcp_world_pose(tcp_link)
        if tool_offset is not None:
            tcp_local = self.get_tcp_local_transform(tcp_link)
            tcp_local[:3, 3] = np.array(tool_offset, dtype=float)
            target_tcp_pose = self.get_flange_world_pose(tcp_link) @ tcp_local
        target_tcp_pose[:3, 3] = target_pos
        success, _ = self.inverse_kinematics_pose(
            target_tcp_pose,
            tcp_link,
            max_iters=max_iters,
            position_tolerance=tolerance,
            orientation_tolerance=0.1,
            orientation_weight=0.2,
        )
        return success

    def compute_directional_reach(self, workspace_report, azimuth_steps=24, elevation_steps=13, cone_half_angle_deg=18.0):
        if not workspace_report or not workspace_report.get("ok"):
            return {"ok": False, "reason": "invalid_workspace"}

        points = np.array(workspace_report.get("points", []), dtype=float)
        base_position = np.array(workspace_report.get("base_position", np.zeros(3)), dtype=float)
        if len(points) == 0:
            return {"ok": False, "reason": "no_points"}

        vectors = points - base_position
        radii = np.linalg.norm(vectors, axis=1)
        valid = radii > 1e-9
        vectors = vectors[valid]
        radii = radii[valid]
        if len(vectors) == 0:
            return {"ok": False, "reason": "degenerate_points"}

        unit_vectors = vectors / radii[:, None]
        cone_cos = float(np.cos(np.radians(cone_half_angle_deg)))
        reach_samples = []

        azimuths = np.linspace(-180.0, 180.0, num=max(4, azimuth_steps), endpoint=False)
        elevations = np.linspace(-90.0, 90.0, num=max(3, elevation_steps))

        for elevation_deg in elevations:
            elev_rad = np.radians(elevation_deg)
            ce = np.cos(elev_rad)
            se = np.sin(elev_rad)
            for azimuth_deg in azimuths:
                az_rad = np.radians(azimuth_deg)
                direction = np.array([ce * np.cos(az_rad), ce * np.sin(az_rad), se], dtype=float)
                direction /= max(np.linalg.norm(direction), 1e-12)

                alignment = unit_vectors @ direction
                mask = alignment >= cone_cos
                if np.any(mask):
                    best_idx_local = int(np.argmax(radii[mask]))
                    candidate_indices = np.flatnonzero(mask)
                    best_idx = int(candidate_indices[best_idx_local])
                    reach_radius = float(radii[best_idx])
                else:
                    projected = radii * np.clip(alignment, 0.0, None)
                    best_idx = int(np.argmax(projected))
                    reach_radius = float(projected[best_idx])

                reach_samples.append(
                    {
                        "azimuth_deg": float(azimuth_deg),
                        "elevation_deg": float(elevation_deg),
                        "direction": direction.copy(),
                        "reach_radius": reach_radius,
                        "point": points[np.flatnonzero(valid)[best_idx]].copy(),
                    }
                )

        cardinal_dirs = {
            "+X": np.array([1.0, 0.0, 0.0], dtype=float),
            "-X": np.array([-1.0, 0.0, 0.0], dtype=float),
            "+Y": np.array([0.0, 1.0, 0.0], dtype=float),
            "-Y": np.array([0.0, -1.0, 0.0], dtype=float),
            "+Z": np.array([0.0, 0.0, 1.0], dtype=float),
            "-Z": np.array([0.0, 0.0, -1.0], dtype=float),
        }

        cardinal = {}
        for label, direction in cardinal_dirs.items():
            alignment = unit_vectors @ direction
            mask = alignment >= cone_cos
            if np.any(mask):
                cardinal[label] = float(np.max(radii[mask]))
            else:
                cardinal[label] = float(np.max(radii * np.clip(alignment, 0.0, None)))

        reach_values = np.array([item["reach_radius"] for item in reach_samples], dtype=float)
        best_idx = int(np.argmax(reach_values))
        worst_idx = int(np.argmin(reach_values))

        return {
            "ok": True,
            "cone_half_angle_deg": float(cone_half_angle_deg),
            "azimuth_steps": int(len(azimuths)),
            "elevation_steps": int(len(elevations)),
            "samples": reach_samples,
            "cardinal_reach": cardinal,
            "max_directional_reach": float(reach_values[best_idx]),
            "min_directional_reach": float(reach_values[worst_idx]),
            "best_direction": {
                "azimuth_deg": reach_samples[best_idx]["azimuth_deg"],
                "elevation_deg": reach_samples[best_idx]["elevation_deg"],
            },
            "worst_direction": {
                "azimuth_deg": reach_samples[worst_idx]["azimuth_deg"],
                "elevation_deg": reach_samples[worst_idx]["elevation_deg"],
            },
        }

    def compute_workspace(self, tcp_link, max_samples=1200):
        if tcp_link is None:
            return {"ok": False, "reason": "missing_tcp"}

        chain = self.get_kinematic_chain(tcp_link)
        if not chain:
            return {"ok": False, "reason": "empty_chain"}

        dims = len(chain)
        if dims <= 0:
            return {"ok": False, "reason": "empty_chain"}

        samples_per_joint = max(2, int(round(max_samples ** (1.0 / dims))))
        samples_per_joint = min(samples_per_joint, 7)

        sample_axes = []
        for joint in chain:
            if abs(joint.max_limit - joint.min_limit) < 1e-9:
                sample_axes.append(np.array([joint.min_limit], dtype=float))
            else:
                sample_axes.append(np.linspace(joint.min_limit, joint.max_limit, num=samples_per_joint, dtype=float))

        old_values = {name: joint.current_value for name, joint in self.joints.items()}
        points = []
        configs = []
        effort_scores = []
        visited = 0
        root_pos = chain[0].parent_link.t_world[:3, 3].copy()

        try:
            for config in product(*sample_axes):
                config_arr = np.array(config, dtype=float)
                self._apply_joint_vector(chain, config_arr)
                points.append(self.get_tcp_world_pose(tcp_link)[:3, 3].copy())
                configs.append(config_arr.copy())
                loads = self.compute_static_joint_loads(joint_names=[joint.name for joint in chain])
                effort_scores.append(float(sum(item["resultant_torque_ncm"] for item in loads)))
                visited += 1
        finally:
            for name, value in old_values.items():
                self.joints[name].current_value = value
            self.update_kinematics()

        if not points:
            return {"ok": False, "reason": "no_points"}

        points = np.array(points, dtype=float)
        configs = np.array(configs, dtype=float)
        effort_scores = np.array(effort_scores, dtype=float)
        bounds_min = points.min(axis=0)
        bounds_max = points.max(axis=0)
        center = points.mean(axis=0)
        radii = np.linalg.norm(points - root_pos, axis=1)
        radial_xy = np.linalg.norm(points[:, :2] - root_pos[:2], axis=1)

        report = {
            "ok": True,
            "tcp_link": tcp_link.name,
            "joint_count": dims,
            "sample_axes": [axis.copy() for axis in sample_axes],
            "sample_count": int(visited),
            "points": points,
            "joint_configs": configs,
            "joint_effort_scores": effort_scores,
            "bounds_min": bounds_min,
            "bounds_max": bounds_max,
            "center": center,
            "base_position": root_pos,
            "radius_min": float(radii.min()),
            "radius_max": float(radii.max()),
            "xy_radius_min": float(radial_xy.min()),
            "xy_radius_max": float(radial_xy.max()),
            "z_min": float(bounds_min[2]),
            "z_max": float(bounds_max[2]),
            "kdtree": cKDTree(points) if cKDTree is not None and len(points) else None,
        }
        if len(points) >= 2:
            if report["kdtree"] is not None:
                nn = report["kdtree"].query(points, k=2)[0][:, 1]
                report["sample_spacing"] = float(np.median(nn))
            else:
                diffs = points[None, :, :] - points[:, None, :]
                dist_mat = np.linalg.norm(diffs, axis=2)
                np.fill_diagonal(dist_mat, np.inf)
                report["sample_spacing"] = float(np.median(np.min(dist_mat, axis=1)))
        else:
            report["sample_spacing"] = 0.0
        report["directional_reach"] = self.compute_directional_reach(report)
        self.workspace_report = report
        return report
