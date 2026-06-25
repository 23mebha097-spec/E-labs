import numpy as np
from core.kinematics import transform_from_pose, invert_transform

# Try to import scipy for CubicSpline, fall back to pure NumPy Catmull-Rom
try:
    from scipy.interpolate import CubicSpline
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class WorkspacePlan:
    """
    Represents the physical boundaries, margins, and coordinate frame of the path planning environment.
    All parameters are defined in centimeters.
    """
    def __init__(
        self,
        width=100.0,
        height=100.0,
        grid_size=10.0,
        safe_margin=5.0,
        origin=None,
        inclination_deg=45.0,
        plane_mode="inclined",
        origin_anchor="center",
    ):
        self.width = float(width)
        self.height = float(height)
        self.grid_size = float(grid_size)
        self.safe_margin = float(safe_margin)
        self.inclination_deg = float(inclination_deg)
        self.plane_mode = str(plane_mode or "inclined")
        self.origin_anchor = str(origin_anchor or "center").lower()
        
        # Origin is the 3D translation of the workspace center relative to the robot Base Frame (in cm)
        self.origin = np.array(origin, dtype=float) if origin is not None else np.zeros(3)
        
        # Homogeneous Transformation matrix representing Workspace Frame w.r.t Base Frame
        self.t_workspace_base = np.eye(4, dtype=float)
        self.t_workspace_base[:3, 3] = self.origin
        
        self.t_workspace_base[:3, :3] = self._rotation_for_mode(self.plane_mode)

    def get_local_bounds(self):
        """Return the rectangular workspace limits in local plane coordinates."""
        if self.origin_anchor in ("lower_left", "bottom_left", "positive"):
            return 0.0, self.width, 0.0, self.height

        half_w = self.width / 2.0
        half_h = self.height / 2.0
        return -half_w, half_w, -half_h, half_h

    def _rotation_for_mode(self, plane_mode):
        """Return workspace-frame rotation for the selected drawing plane."""
        mode = str(plane_mode or "inclined").lower()
        if mode in ("horizontal", "xy", "horizontal_xy"):
            return np.eye(3, dtype=float)
        if mode in ("vertical_xz", "xz"):
            return np.array([
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
                [0.0, 1.0, 0.0],
            ], dtype=float)
        if mode in ("vertical_yz", "yz"):
            return np.array([
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ], dtype=float)

        theta = np.radians(self.inclination_deg)
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c]
        ], dtype=float)

    def get_workspace_to_world_transform(self, base_world_transform):
        """
        Calculates the transformation matrix mapping the Workspace Frame to the World Frame.
        base_world_transform: 4x4 homogeneous matrix of the Robot Base Frame in World coordinates.
        """
        base_world = np.array(base_world_transform, dtype=float)
        return base_world @ self.t_workspace_base

    def validate_workspace_bounds(self, point_local):
        """
        Validates if a point (in Workspace Frame local coordinates) lies within the workspace
        boundary minus safety margins.
        point_local: 3D point array [x, y, z] in local Workspace Frame.
        """
        pt = np.array(point_local, dtype=float)
        min_x, max_x, min_y, max_y = self.get_local_bounds()
        return (
            (min_x + self.safe_margin) <= pt[0] <= (max_x - self.safe_margin)
            and (min_y + self.safe_margin) <= pt[1] <= (max_y - self.safe_margin)
        )

    def convert_local_to_world(self, point_local, base_world_transform):
        """
        Converts a 3D point from Workspace Frame local coordinates to World Frame coordinates.
        """
        pt_loc = np.array(point_local, dtype=float)
        pt_4 = np.append(pt_loc, 1.0)
        t_ws_world = self.get_workspace_to_world_transform(base_world_transform)
        return (t_ws_world @ pt_4)[:3]

    def convert_world_to_local(self, point_world, base_world_transform):
        """
        Converts a 3D point from World Frame coordinates to local Workspace Frame coordinates.
        """
        pt_w = np.array(point_world, dtype=float)
        pt_4 = np.append(pt_w, 1.0)
        t_ws_world = self.get_workspace_to_world_transform(base_world_transform)
        t_world_ws = invert_transform(t_ws_world)
        return (t_world_ws @ pt_4)[:3]


class PathTrajectory:
    """
    A time-parameterized representation of a robotic path, containing Cartesian states,
    orientations, and reachability attributes.
    """
    def __init__(self, points, normals=None, timestamps=None, velocities=None, accelerations=None, orientations=None, reachable_flags=None):
        self.points = np.array(points, dtype=float)  # Nx3 array
        n_pts = len(self.points)

        self.normals = np.array(normals, dtype=float) if normals is not None else np.tile([0.0, 0.0, 1.0], (n_pts, 1))
        self.timestamps = np.array(timestamps, dtype=float) if timestamps is not None else np.zeros(n_pts)
        self.velocities = np.array(velocities, dtype=float) if velocities is not None else np.zeros((n_pts, 3))
        self.accelerations = np.array(accelerations, dtype=float) if accelerations is not None else np.zeros((n_pts, 3))
        
        # Orientations are stored as Nx3x3 rotation matrices
        self.orientations = np.array(orientations, dtype=float) if orientations is not None else np.tile(np.eye(3), (n_pts, 1, 1))
        self.reachable_flags = list(reachable_flags) if reachable_flags is not None else [True] * n_pts

    def is_fully_reachable(self):
        """Returns True if every point in the trajectory is solvable by IK."""
        return all(self.reachable_flags)

    def get_total_length(self):
        """Calculates cumulative linear Euclidean length of the path in centimeters."""
        if len(self.points) < 2:
            return 0.0
        diffs = np.diff(self.points, axis=0)
        return float(np.sum(np.linalg.norm(diffs, axis=1)))

    def sample_at_time(self, time):
        """
        Samples the trajectory state at an arbitrary absolute elapsed time (seconds).
        Uses linear interpolation for translation vectors and matrix SVD orthogonalization for rotation.
        Returns: (point, normal, orientation, velocity, acceleration, reachable)
        """
        t = float(time)
        n = len(self.points)
        if n == 0:
            return None
        if n == 1 or t <= self.timestamps[0]:
            return (self.points[0].copy(), self.normals[0].copy(), self.orientations[0].copy(),
                    self.velocities[0].copy(), self.accelerations[0].copy(), self.reachable_flags[0])
        if t >= self.timestamps[-1]:
            return (self.points[-1].copy(), self.normals[-1].copy(), self.orientations[-1].copy(),
                    self.velocities[-1].copy(), self.accelerations[-1].copy(), self.reachable_flags[-1])

        # Locate interval indices
        idx = np.searchsorted(self.timestamps, t) - 1
        t0 = self.timestamps[idx]
        t1 = self.timestamps[idx + 1]
        factor = (t - t0) / (t1 - t0)

        # Interpolate points, normals, velocities, and accelerations
        pt = self.points[idx] + factor * (self.points[idx + 1] - self.points[idx])
        norm = self.normals[idx] + factor * (self.normals[idx + 1] - self.normals[idx])
        norm_len = np.linalg.norm(norm)
        if norm_len > 1e-6:
            norm = norm / norm_len

        vel = self.velocities[idx] + factor * (self.velocities[idx + 1] - self.velocities[idx])
        acc = self.accelerations[idx] + factor * (self.accelerations[idx + 1] - self.accelerations[idx])
        reachable = self.reachable_flags[idx] and self.reachable_flags[idx + 1]

        # Rotate matrix using LERP + SVD projection (highly robust alternative to SLERP for rotation matrices)
        r0 = self.orientations[idx]
        r1 = self.orientations[idx + 1]
        r_interp = r0 + factor * (r1 - r0)
        u, _, vh = np.linalg.svd(r_interp)
        rot = u @ vh
        if np.linalg.det(rot) < 0:
            u[:, -1] *= -1
            rot = u @ vh

        return pt, norm, rot, vel, acc, reachable

    def interpolate_pose(self, t, t0, t1, pose0, pose1):
        """
        Linearly interpolates between two homogeneous 4x4 transform matrices at relative time t.
        """
        factor = (t - t0) / (t1 - t0)
        pos = pose0[:3, 3] + factor * (pose1[:3, 3] - pose0[:3, 3])
        
        r0 = pose0[:3, :3]
        r1 = pose1[:3, :3]
        r_interp = r0 + factor * (r1 - r0)
        u, _, vh = np.linalg.svd(r_interp)
        rot = u @ vh
        if np.linalg.det(rot) < 0:
            u[:, -1] *= -1
            rot = u @ vh
            
        tf = np.eye(4)
        tf[:3, :3] = rot
        tf[:3, 3] = pos
        return tf


class PathPlanner:
    """
    Core engine responsible for generating paths, performing geometric transformations,
    smoothing pathways, and profiling velocity states.
    """
    def __init__(self, workspace=None):
        self.workspace = workspace if workspace is not None else WorkspacePlan()
        self.current_trajectory = None

    def generate_square(self, center_x, center_y, side_length, z_height, num_points=100):
        """Generates coordinate sequence forming a square path on local XY workspace plane."""
        half = side_length / 2.0
        corners = np.array([
            [center_x - half, center_y - half, z_height],
            [center_x + half, center_y - half, z_height],
            [center_x + half, center_y + half, z_height],
            [center_x - half, center_y + half, z_height],
            [center_x - half, center_y - half, z_height]
        ])
        
        distances = [0.0, side_length, 2 * side_length, 3 * side_length, 4 * side_length]
        target_s = np.linspace(0.0, 4 * side_length, num_points)
        
        pts = np.zeros((num_points, 3))
        for i, s in enumerate(target_s):
            for j in range(4):
                if distances[j] <= s <= distances[j+1] + 1e-9:
                    factor = (s - distances[j]) / side_length
                    pts[i] = corners[j] + factor * (corners[j+1] - corners[j])
                    break
        return pts

    def generate_wave(self, start_x, start_y, end_x, end_y, amplitude, periods, z_height, num_points=100):
        """Generates a sinusoidal wave in local workspace XY coordinates."""
        start = np.array([start_x, start_y, z_height], dtype=float)
        end = np.array([end_x, end_y, z_height], dtype=float)
        vec = end - start
        length = np.linalg.norm(vec[:2])
        
        if length < 1e-6:
            # Linear point representation if path collapses
            return np.tile(start, (num_points, 1))
            
        unit_x = vec / length
        unit_y = np.array([-unit_x[1], unit_x[0], 0.0]) # Perpendicular vector
        
        pts = np.zeros((num_points, 3))
        t_vals = np.linspace(0.0, 1.0, num_points)
        for i, t in enumerate(t_vals):
            dist = t * length
            offset = amplitude * np.sin(2.0 * np.pi * periods * t)
            pts[i] = start + dist * unit_x + offset * unit_y
            
        return pts

    def generate_custom_path(self, points_local):
        """Accepts a raw array of 3D local points and converts it to a standard NumPy representation."""
        return np.array(points_local, dtype=float)

    def interpolate_path(self, points, method="cubic", num_points=100):
        """
        Interpolates discrete points using either CubicSplines (via SciPy) or a native Catmull-Rom formulation.
        """
        pts = np.array(points, dtype=float)
        if len(pts) < 3:
            # Fallback to simple linear interpolation
            alphas = np.linspace(0.0, 1.0, num_points)
            return pts[0] + alphas[:, None] * (pts[-1] - pts[0])

        if method == "cubic" and HAS_SCIPY:
            # Parameterize path using cumulative arc-length
            diffs = np.diff(pts, axis=0)
            s_coords = np.zeros(len(pts))
            s_coords[1:] = np.cumsum(np.linalg.norm(diffs, axis=1))
            
            s_query = np.linspace(0.0, s_coords[-1], num_points)
            cs = CubicSpline(s_coords, pts, axis=0)
            return cs(s_query)
        else:
            # Use robust native Catmull-Rom spline interpolation (no external dependencies)
            N = len(pts)
            padded = np.vstack([
                2 * pts[0] - pts[1],
                pts,
                2 * pts[-1] - pts[-2]
            ])
            
            t_out = np.linspace(0, N - 1, num_points)
            out_points = []
            
            for t in t_out:
                i = int(np.floor(t))
                if i >= N - 1:
                    i = N - 2
                u = t - i
                
                p0 = padded[i]
                p1 = padded[i + 1]
                p2 = padded[i + 2]
                p3 = padded[i + 3]
                
                pt = 0.5 * (
                    (2.0 * p1) +
                    (-p0 + p2) * u +
                    (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * (u**2) +
                    (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * (u**3)
                )
                out_points.append(pt)
                
            return np.array(out_points)

    def apply_velocity_profile(self, points, max_vel=20.0, max_accel=10.0):
        """
        Generates a trapezoidal velocity profile over a series of spatial coordinates.
        Computes accurate timestamps, tangent normals, velocities, and acceleration limits.
        """
        pts = np.array(points, dtype=float)
        n_pts = len(pts)
        if n_pts == 0:
            return PathTrajectory(np.zeros((0, 3)))
        if n_pts == 1:
            return PathTrajectory(pts, timestamps=[0.0])

        diffs = np.diff(pts, axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)
        s_coords = np.zeros(n_pts)
        s_coords[1:] = np.cumsum(segment_lengths)
        S = s_coords[-1]

        if S < 1e-9:
            return PathTrajectory(pts, timestamps=np.zeros(n_pts))

        # Check acceleration capacity to reach peak velocity
        d_acc = (max_vel ** 2) / (2.0 * max_accel)
        if S >= 2.0 * d_acc:
            # Trapeze profile
            t_a = max_vel / max_accel
            t_c = (S - 2.0 * d_acc) / max_vel
            t_d = t_a
            T = t_a + t_c + t_d

            s_acc = d_acc
            s_flat = S - d_acc

            t_coords = np.zeros(n_pts)
            v_coords = np.zeros((n_pts, 3))
            a_coords = np.zeros((n_pts, 3))

            for i in range(n_pts):
                s = s_coords[i]
                if i == 0:
                    t_coords[i] = 0.0
                    v_mag = 0.0
                    a_mag = max_accel
                elif i == n_pts - 1:
                    t_coords[i] = T
                    v_mag = 0.0
                    a_mag = -max_accel
                elif s <= s_acc:
                    t_coords[i] = np.sqrt(2.0 * s / max_accel)
                    v_mag = max_accel * t_coords[i]
                    a_mag = max_accel
                elif s <= s_flat:
                    t_coords[i] = t_a + (s - s_acc) / max_vel
                    v_mag = max_vel
                    a_mag = 0.0
                else:
                    s_dec = S - s
                    t_coords[i] = T - np.sqrt(np.clip(2.0 * s_dec / max_accel, 0.0, None))
                    v_mag = max_accel * (T - t_coords[i])
                    a_mag = -max_accel

                # Normal tangent computation
                if i < n_pts - 1:
                    dir_vec = pts[i+1] - pts[i]
                else:
                    dir_vec = pts[i] - pts[i-1]
                dir_norm = np.linalg.norm(dir_vec)
                dir_unit = dir_vec / dir_norm if dir_norm > 1e-9 else np.zeros(3)

                v_coords[i] = dir_unit * v_mag
                a_coords[i] = dir_unit * a_mag
        else:
            # Triangular profile
            v_peak = np.sqrt(S * max_accel)
            t_a = v_peak / max_accel
            t_d = t_a
            T = t_a + t_d

            s_half = S / 2.0

            t_coords = np.zeros(n_pts)
            v_coords = np.zeros((n_pts, 3))
            a_coords = np.zeros((n_pts, 3))

            for i in range(n_pts):
                s = s_coords[i]
                if i == 0:
                    t_coords[i] = 0.0
                    v_mag = 0.0
                    a_mag = max_accel
                elif i == n_pts - 1:
                    t_coords[i] = T
                    v_mag = 0.0
                    a_mag = -max_accel
                elif s <= s_half:
                    t_coords[i] = np.sqrt(2.0 * s / max_accel)
                    v_mag = max_accel * t_coords[i]
                    a_mag = max_accel
                else:
                    s_dec = S - s
                    t_coords[i] = T - np.sqrt(np.clip(2.0 * s_dec / max_accel, 0.0, None))
                    v_mag = max_accel * (T - t_coords[i])
                    a_mag = -max_accel

                # Normal tangent computation
                if i < n_pts - 1:
                    dir_vec = pts[i+1] - pts[i]
                else:
                    dir_vec = pts[i] - pts[i-1]
                dir_norm = np.linalg.norm(dir_vec)
                dir_unit = dir_vec / dir_norm if dir_norm > 1e-9 else np.zeros(3)

                v_coords[i] = dir_unit * v_mag
                a_coords[i] = dir_unit * a_mag

        self.current_trajectory = PathTrajectory(
            points=pts,
            timestamps=t_coords,
            velocities=v_coords,
            accelerations=a_coords
        )
        return self.current_trajectory

    def check_reachability(self, robot, tcp_link, trajectory, position_tolerance=0.5, orientation_tolerance=0.1, orientation_weight=0.35):
        """
        Determines target IK viability for every point in a path trajectory.
        Saves and restores active joint configuration to prevent side-effects.
        Updates the trajectory reachable_flags array.
        """
        if isinstance(tcp_link, str):
            tcp_link_obj = robot.links.get(tcp_link)
        else:
            tcp_link_obj = tcp_link

        if tcp_link_obj is None:
            return [False] * len(trajectory.points)

        reachable_flags = []
        old_values = {name: joint.current_value for name, joint in robot.joints.items()}
        base_world = robot.base_link.t_world if robot.base_link is not None else np.eye(4)
        r_base_world = base_world[:3, :3]

        try:
            for i in range(len(trajectory.points)):
                pt = trajectory.points[i]
                rot = trajectory.orientations[i]

                # 1. Bounds Validation
                if not self.workspace.validate_workspace_bounds(pt):
                    reachable_flags.append(False)
                    continue

                # 2. Frame Transformation (Local -> World)
                pt_world = self.workspace.convert_local_to_world(pt, base_world)
                r_world = r_base_world @ rot

                target_pose = np.eye(4, dtype=float)
                target_pose[:3, :3] = r_world
                target_pose[:3, 3] = pt_world

                # 3. Solver check
                success, _ = robot.inverse_kinematics_pose(
                    target_tcp_pose=target_pose,
                    tcp_link=tcp_link_obj,
                    max_iters=150,
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                    orientation_weight=orientation_weight
                )
                reachable_flags.append(bool(success))
        finally:
            # Restore joints to prevent unsolicited UI movement
            for name, val in old_values.items():
                robot.joints[name].current_value = val
            robot.update_kinematics()

        trajectory.reachable_flags = reachable_flags
        return reachable_flags

    def check_reachability_range(
        self,
        robot,
        tcp_link,
        trajectory,
        start,
        end,
        position_tolerance=0.5,
        orientation_tolerance=0.1,
        orientation_weight=0.35,
    ):
        """Validate IK for trajectory points in [start, end). Updates reachable_flags in place."""
        if isinstance(tcp_link, str):
            tcp_link_obj = robot.links.get(tcp_link)
        else:
            tcp_link_obj = tcp_link

        if tcp_link_obj is None:
            return []

        n_pts = len(trajectory.points)
        if not trajectory.reachable_flags or len(trajectory.reachable_flags) != n_pts:
            trajectory.reachable_flags = [False] * n_pts

        start = max(0, int(start))
        end = min(n_pts, int(end))
        if start >= end:
            return trajectory.reachable_flags[start:end]

        old_values = {name: joint.current_value for name, joint in robot.joints.items()}
        base_world = robot.base_link.t_world if robot.base_link is not None else np.eye(4)
        r_base_world = base_world[:3, :3]

        try:
            for i in range(start, end):
                pt = trajectory.points[i]
                rot = trajectory.orientations[i]

                if not self.workspace.validate_workspace_bounds(pt):
                    trajectory.reachable_flags[i] = False
                    continue

                pt_world = self.workspace.convert_local_to_world(pt, base_world)
                r_world = r_base_world @ rot

                target_pose = np.eye(4, dtype=float)
                target_pose[:3, :3] = r_world
                target_pose[:3, 3] = pt_world

                success, _ = robot.inverse_kinematics_pose(
                    target_tcp_pose=target_pose,
                    tcp_link=tcp_link_obj,
                    max_iters=150,
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                    orientation_weight=orientation_weight,
                )
                trajectory.reachable_flags[i] = bool(success)
        finally:
            for name, val in old_values.items():
                robot.joints[name].current_value = val
            robot.update_kinematics()

        return trajectory.reachable_flags[start:end]
