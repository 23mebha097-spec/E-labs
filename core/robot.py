import numpy as np

class Link:
    def __init__(self, name, mesh=None):
        self.name = name
        self.mesh = mesh  # trimesh object
        self.color = "lightgray"
        self.is_base = False
        self.pick_pos = [0.0, 0.0, 0.0]
        self.place_pos = [0.0, 0.0, 0.0]
        
        # Local offset matrix from alignment system
        self.t_offset = np.eye(4)
        
        # Current world transform (computed by forward kinematics)
        self.t_world = np.eye(4)
        
        self.parent_joint = None
        self.child_joints = []
        self.custom_tcp_offset = None # Optional [x, y, z] relative to link frame (Live Point)
        
        # Physics Properties (Default placeholders)
        self.mass = 1.0
        self.inertia = {"ixx": 0.001, "ixy": 0.0, "ixz": 0.0, "iyy": 0.001, "iyz": 0.0, "izz": 0.001}
        self.com = [0.0, 0.0, 0.0]
        
        # Auto-calculate from mesh geometry
        self.compute_physics_from_mesh()

    def compute_physics_from_mesh(self):
        """Calculates CoG and Inertia Tensor based on mesh geometry using trimesh."""
        if self.mesh is None:
            return
            
        try:
            # trimesh.center_mass returns the CoG (weighted by volume)
            cm = self.mesh.center_mass
            if cm is not None:
                self.com = cm.tolist()
            
            # trimesh.moment_inertia returns the 3x3 inertia tensor for unit mass
            I = self.mesh.moment_inertia
            if I is not None:
                self.inertia = {
                    "ixx": float(I[0, 0]), "ixy": float(I[0, 1]), "ixz": float(I[0, 2]),
                    "iyy": float(I[1, 1]), "iyz": float(I[1, 2]), "izz": float(I[2, 2])
                }
        except Exception:
            # Fallback to simple centroid if volume-based calculation fails (e.g. non-watertight)
            self.com = self.mesh.centroid.tolist()

class Joint:
    def __init__(self, name, parent_link, child_link, joint_type="revolute"):
        self.name = name
        self.parent_link = parent_link
        self.child_link = child_link
        self.joint_type = joint_type
        self.is_gripper = False
        
        self.origin = np.array([0.0, 0.0, 0.0]) # Relative to parent link frame
        self.axis = np.array([0.0, 0.0, 1.0])   # Unit vector
        self.axis_name = "Z"                    # Explicit axis name (X, Y, or Z)
        
        self.min_limit = -180.0
        self.max_limit = 180.0
        self.current_value = 0.0
        
        # Link children
        parent_link.child_joints.append(self)
        child_link.parent_joint = self

    def get_matrix(self):
        """
        Returns the transform matrix for this joint.
        Math: T = T(origin) * R(axis, theta) * T(-origin)
        This rotates the frame around the defined 'origin' point.
        """
        theta = np.radians(self.current_value)
        
        # 1. Rotation Matrix (R)
        R = self._rotation_matrix(self.axis, theta)
        
        # 2. Translation Matrices for Pivot (T_origin, T_neg_origin)
        T_o = np.eye(4); T_o[:3, 3] = self.origin
        T_no = np.eye(4); T_no[:3, 3] = -self.origin
        
        # T_pivot = T(o) @ R @ T(-o)
        return T_o @ R @ T_no

    def _rotation_matrix(self, axis, theta):
        """Standard Rodrigues' Rotation Formula (Library-Grade Stability)"""
        axis = np.array(axis)
        axis = axis / (np.linalg.norm(axis) + 1e-9)
        
        # Cross-product matrix (Skew-symmetric)
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        
        # R = I + sin(theta)K + (1-cos(theta))K^2
        I = np.eye(3)
        R = I + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
        
        ret = np.eye(4)
        ret[:3, :3] = R
        return ret

class Robot:
    def __init__(self):
        self.links = {}
        self.joints = {}
        self.base_link = None
        self.joint_relations = {} # {master_name: [(slave_name, ratio), ...]}

    def reset_to_home(self, home_angle=0.0):
        """Sets all joints to the specified home angle."""
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
        
        # --- ROBUSTNESS: A child link can have only one parent joint ---
        if child.parent_joint:
            # Find the name of the existing joint and remove it
            old_joint = child.parent_joint
            # Safe deletion from dictionary while iterating
            names_to_remove = [jn for jn, j in self.joints.items() if j == old_joint]
            for jn in names_to_remove:
                self.remove_joint(jn)
                
        joint = Joint(name, parent, child)
        self.joints[name] = joint
        return joint

    def remove_link(self, name):
        if name not in self.links:
            return
        
        link = self.links[name]
        
        # Cleanup joints
        to_remove_joints = []
        for j_name, joint in self.joints.items():
            if joint.parent_link == link:
                # If removing parent, child stays (bake transform is complex, simplest is keep offset)
                joint.child_link.t_offset = joint.child_link.t_world
                to_remove_joints.append(j_name)
            elif joint.child_link == link:
                to_remove_joints.append(j_name)
        
        for j_name in to_remove_joints:
            joint = self.joints[j_name]
            if joint in joint.parent_link.child_joints:
                joint.parent_link.child_joints.remove(joint)
            joint.child_link.parent_joint = None
            del self.joints[j_name]
            
        del self.links[name]
        
        if self.base_link == link:
            self.base_link = None

    def remove_joint(self, name):
        """Safely removes a joint and clears parent/child references"""
        if name not in self.joints:
            return
            
        joint = self.joints[name]
        parent = joint.parent_link
        child = joint.child_link
        
        # Remove from parent's list of children
        if joint in parent.child_joints:
            parent.child_joints.remove(joint)
            
        # Clear child's reference to parent
        child.parent_joint = None
        
        # Cleanup relations
        if name in self.joint_relations:
            del self.joint_relations[name]
            
        # Remove from other's relations as slave
        for master, slaves in self.joint_relations.items():
            self.joint_relations[master] = [(s, r) for s, r in slaves if s != name]
        
        # Remove from robot's global dict
        del self.joints[name]
        
        # Reset child's world transform to its current relative offset
        # (Usually it remains where it was when joint was deleted)
        self.update_kinematics()

    def update_kinematics(self):
        visited = set()
        
        # 1. Identify Roots
        roots = [l for l in self.links.values() if l.parent_joint is None]
        
        # 2. Prioritize Base
        if self.base_link and self.base_link in roots:
            roots.remove(self.base_link)
            roots.insert(0, self.base_link)

        # 3. Propagate
        for root in roots:
            if root.name in visited: continue
            
            root.t_world = root.t_offset
            visited.add(root.name)
            
            stack = [root]
            while stack:
                parent = stack.pop()
                
                for joint in parent.child_joints:
                    child = joint.child_link
                    if child.name in visited: continue
                    
                    # Compute kinematic transform
                    # Child_World = Parent_World * Joint_Transform * Child_Offset
                    # Joint_Transform = T(p) * R * T(-p) (Rotation about pivot in Parent Frame)
                    # Child_Offset = Static position of child relative to Parent Frame
                    
                    joint_matrix = joint.get_matrix()
                    child.t_world = parent.t_world @ joint_matrix @ child.t_offset
                    
                    visited.add(child.name)
                    stack.append(child)

    def get_kinematic_chain(self, tcp_link):
        """Returns the list of joints from the root to the TCP link, excluding slaves."""
        chain = []
        curr = tcp_link
        while curr.parent_joint is not None:
            # Skip slave joints
            is_slave = False
            for master, slaves in self.joint_relations.items():
                if any(s_id == curr.parent_joint.name for s_id, r in slaves):
                    is_slave = True
                    break
            if not is_slave:
                chain.append(curr.parent_joint)
            curr = curr.parent_joint.parent_link
        return list(reversed(chain))

    def inverse_kinematics(self, target_pos, tcp_link, max_iters=300, tolerance=0.3, tool_offset=None):
        """
        Robust multi-pass Cyclic Coordinate Descent (CCD) IK solver.

        Algorithm improvements over basic CCD:
        1. Multi-pass: More iterations for higher accuracy.
        2. Adaptive damping: Larger steps far from target, smaller near it.
        3. Multi-restart: If stuck in a local minimum, perturb joints and retry.
        4. Progressive tolerance: Tries to converge tightly.
        5. Joint-limit enforcement: Clamps all joints throughout.
        """
        target = np.array(target_pos, dtype=float)
        t_off = np.array(tool_offset, dtype=float) if tool_offset is not None else np.zeros(3)

        # --- Build kinematic chain (root->TCP, skip slave joints) ---
        chain = []
        curr = tcp_link
        while curr.parent_joint is not None:
            is_slave = any(
                any(s_id == curr.parent_joint.name for s_id, _ in slaves)
                for _, slaves in self.joint_relations.items()
            )
            if not is_slave:
                chain.append(curr.parent_joint)
            curr = curr.parent_joint.parent_link
        chain.reverse()  # Root -> TCP order

        if not chain:
            return False

        def _get_tcp_world():
            self.update_kinematics()
            return (tcp_link.t_world @ np.append(t_off, 1.0))[:3]

        def _apply_joint(joint, delta_deg):
            """Apply a delta angle to a joint and propagate to slaves."""
            new_val = np.clip(joint.current_value + delta_deg, joint.min_limit, joint.max_limit)
            joint.current_value = new_val
            if joint.name in self.joint_relations:
                for s_id, ratio in self.joint_relations[joint.name]:
                    if s_id in self.joints:
                        self.joints[s_id].current_value = np.clip(
                            new_val * ratio,
                            self.joints[s_id].min_limit,
                            self.joints[s_id].max_limit
                        )
            self.update_kinematics()

        def _ccd_pass():
            """One full CCD sweep (root -> TCP order for best convergence)."""
            for joint in chain:
                parent = joint.parent_link
                pivot_w = (parent.t_world @ np.array([*joint.origin, 1.0]))[:3]
                axis_w  = parent.t_world[:3, :3] @ joint.axis
                axis_w /= (np.linalg.norm(axis_w) + 1e-9)

                tcp_w = _get_tcp_world()
                v_tool   = tcp_w   - pivot_w
                v_target = target  - pivot_w

                # Project onto the rotation plane perpendicular to the joint axis
                vt_proj = v_tool   - np.dot(v_tool,   axis_w) * axis_w
                vg_proj = v_target - np.dot(v_target, axis_w) * axis_w

                nt, ng = np.linalg.norm(vt_proj), np.linalg.norm(vg_proj)
                if nt < 1e-5 or ng < 1e-5:
                    continue

                u1 = vt_proj / nt
                u2 = vg_proj / ng

                cos_a = np.clip(np.dot(u1, u2), -1.0, 1.0)
                sin_a = np.dot(axis_w, np.cross(u1, u2))
                delta_deg = np.degrees(np.arctan2(sin_a, cos_a))

                # Adaptive damping:
                # - Close to target  → small cautious step (max 5°)
                # - Far from target  → larger step (max 30°)
                dist = np.linalg.norm(target - tcp_w)
                if dist < 5.0:
                    max_step = 2.0
                    damp     = 0.3
                elif dist < 20.0:
                    max_step = 10.0
                    damp     = 0.5
                else:
                    max_step = 30.0
                    damp     = 0.8

                delta_deg = np.clip(delta_deg * damp, -max_step, max_step)
                _apply_joint(joint, delta_deg)

        # --- Snapshot initial state for multi-restart ---
        initial_vals = {j.name: j.current_value for j in chain}
        best_vals    = dict(initial_vals)
        best_dist    = np.linalg.norm(target - _get_tcp_world())

        # --- Main solving loop with restarts ---
        NUM_RESTARTS   = 4
        ITERS_PER_RUN  = max_iters // (NUM_RESTARTS + 1)

        for restart in range(NUM_RESTARTS + 1):
            # --- For restart > 0: perturb joint angles to escape local minima ---
            if restart > 0:
                rng = np.random.RandomState(seed=restart * 42)
                for j in chain:
                    span    = j.max_limit - j.min_limit
                    perturb = rng.uniform(-span * 0.25, span * 0.25)
                    j.current_value = np.clip(
                        best_vals[j.name] + perturb, j.min_limit, j.max_limit
                    )
                self.update_kinematics()

            for _ in range(ITERS_PER_RUN):
                _ccd_pass()
                dist = np.linalg.norm(target - _get_tcp_world())

                # Track best solution across all restarts
                if dist < best_dist:
                    best_dist  = dist
                    best_vals  = {j.name: j.current_value for j in chain}

                if dist < tolerance:
                    # Apply best and finish
                    for j in chain:
                        j.current_value = best_vals[j.name]
                    self.update_kinematics()
                    return True

        # --- Apply the globally best configuration found ---
        for j in chain:
            j.current_value = best_vals[j.name]
        # Propagate slave joints for best config
        for j in chain:
            if j.name in self.joint_relations:
                for s_id, ratio in self.joint_relations[j.name]:
                    if s_id in self.joints:
                        self.joints[s_id].current_value = np.clip(
                            j.current_value * ratio,
                            self.joints[s_id].min_limit,
                            self.joints[s_id].max_limit
                        )
        self.update_kinematics()
        return best_dist < tolerance
