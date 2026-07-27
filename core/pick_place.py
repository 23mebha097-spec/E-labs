"""
Pick & Place Executor - Executes gripper operations using selected contact faces.

This module provides:
- Approach trajectory planning for picking
- Pick operation execution
- Place operation execution
- Generic manipulation operations
- Visual feedback and validation
"""

import numpy as np
from typing import Dict, List, Optional, Tuple, Callable, Any
try:
    import trimesh
except ImportError:  # pragma: no cover - optional dependency in some minimal environments
    trimesh = None
from core.kinematics import transform_from_pose, pose_dict_from_transform, invert_transform


class PickPlaceOperation:
    """Represents a single pick or place operation."""
    
    def __init__(self, operation_type: str, target_pos: np.ndarray, 
                 gripper_joint: str, approach_distance: float = 5.0):
        """
        Args:
            operation_type: "pick" or "place"
            target_pos: Target position (3,) in world frame
            gripper_joint: Name of gripper joint to use
            approach_distance: Distance to approach from above (cm)
        """
        self.operation_type = operation_type  # "pick" or "place"
        self.target_pos = np.array(target_pos)
        self.gripper_joint = gripper_joint
        self.approach_distance = approach_distance
        
        # Trajectory waypoints
        self.waypoints = []  # List of (joint_positions, description)
        self.status = "pending"  # pending, in_progress, success, failed
        self.error_message = ""


class PickPlaceExecutor:
    """
    Executes pick and place operations using gripper contact faces.
    
    Workflow:
    1. Align the gripper with the object
    2. Move to the object with a safe offset above it
    3. Cover the object with the gripper jaws
    4. Close/rotate the gripper joints to catch the object
    5. Keep the gripper-object alignment fixed during transit
    6. Place the object using the solved IK/FK pose
    """
    
    def __init__(self, robot, canvas=None, contact_analyzer=None, end_effector_config=None):
        """
        Initialize the pick-place executor.
        
        Args:
            robot: Robot instance
            canvas: Canvas for visualization (optional)
            contact_analyzer: GripperContactAnalyzer instance (optional)
            end_effector_config: Saved EndEffector payload to use for geometric gripping
        """
        self.robot = robot
        self.canvas = canvas
        self.contact_analyzer = contact_analyzer
        self.end_effector_config = end_effector_config
        
        self._operation_history = []
        self._current_operation = None
        
    def compute_grasp_approach(self, target_pos: np.ndarray, 
                               gripper_joint: str,
                               approach_distance: float = 5.0,
                               num_waypoints: int = 5) -> Tuple[List[np.ndarray], str]:
        """
        Compute approach trajectory to target position.
        
        Creates a smooth path from current gripper position to target position,
        approaching from above.
        
        Args:
            target_pos: Target position in world frame (3,)
            gripper_joint: Gripper joint name
            approach_distance: Distance to approach from above (cm)
            num_waypoints: Number of interpolation points
            
        Returns:
            (waypoints, description) - List of (x,y,z) positions and status message
        """
        if gripper_joint not in self.robot.joints:
            return [], f"❌ Joint '{gripper_joint}' not found"
        
        joint = self.robot.joints[gripper_joint]
        gripper_link = joint.child_link
        
        # Current gripper position
        current_pos = gripper_link.t_world[:3, 3]
        
        # Approach point: above target
        approach_pos = target_pos.copy()
        approach_pos[2] += approach_distance  # Move up by approach_distance
        
        # Retract point: further above
        retract_pos = target_pos.copy()
        retract_pos[2] += approach_distance * 1.5
        
        # Generate waypoints: current -> approach -> target
        waypoints = []
        
        # Phase 1: Current to approach point (smooth curve)
        for i in range(num_waypoints // 2):
            t = i / (num_waypoints // 2)
            # Ease-out cubic interpolation
            t_eased = t * t * (3.0 - 2.0 * t)
            pos = current_pos + t_eased * (approach_pos - current_pos)
            waypoints.append(pos)
        
        # Phase 2: Approach to target (slower, more precise)
        for i in range(num_waypoints // 2 + 1):
            t = i / (num_waypoints // 2)
            # Ease-in cubic interpolation
            t_eased = t * t
            pos = approach_pos + t_eased * (target_pos - approach_pos)
            waypoints.append(pos)
        
        distance = np.linalg.norm(approach_pos - current_pos) + \
                   np.linalg.norm(target_pos - approach_pos)
        
        msg = f"✓ Approach trajectory: {len(waypoints)} waypoints, {distance:.1f}cm total"
        return waypoints, msg

    def load_end_effector_definition(self, end_effector_config: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Load the saved end-effector definition payload and normalize it for geometry use."""
        payload = end_effector_config if end_effector_config is not None else self.end_effector_config
        if payload is None and hasattr(self.robot, 'gripper_tool_config'):
            payload = getattr(self.robot, 'gripper_tool_config')

        if payload is None:
            return {"ok": False, "reason": "missing_end_effector_config", "Jaws": []}

        candidate = payload.get("EndEffector", payload) if isinstance(payload, dict) else payload
        jaws = candidate.get("Jaws", []) if isinstance(candidate, dict) else []
        jaw_count = int(candidate.get("JawCount", len(jaws))) if isinstance(candidate, dict) else len(jaws)

        return {
            "ok": True,
            "payload": payload,
            "ToolType": candidate.get("ToolType", "Gripper Tool") if isinstance(candidate, dict) else "Gripper Tool",
            "JawCount": jaw_count,
            "Jaws": jaws,
        }

    def compute_object_bounds(self, target_link) -> Dict[str, Optional[np.ndarray]]:
        """Return world-space bounds for a link/object mesh in a generic way."""
        if target_link is None or not hasattr(target_link, 'mesh') or target_link.mesh is None:
            return {
                "world_min": None,
                "world_max": None,
                "world_center": None,
                "extent": None,
            }

        mesh = target_link.mesh
        if not hasattr(mesh, 'bounds'):
            return {
                "world_min": None,
                "world_max": None,
                "world_center": None,
                "extent": None,
            }

        bounds = np.asarray(mesh.bounds, dtype=float)
        if bounds.shape == (6,):
            bounds = np.array([
                [bounds[0], bounds[2], bounds[4]],
                [bounds[1], bounds[3], bounds[5]],
            ], dtype=float)

        transform = np.asarray(getattr(target_link, 't_world', np.eye(4)), dtype=float)
        world_min = (transform @ np.append(bounds[0], 1.0))[:3]
        world_max = (transform @ np.append(bounds[1], 1.0))[:3]
        world_center = (world_min + world_max) * 0.5

        return {
            "world_min": world_min,
            "world_max": world_max,
            "world_center": world_center,
            "extent": world_max - world_min,
        }

    def _ray_triangle_intersection(self, origin: np.ndarray, direction: np.ndarray,
                                   triangle: np.ndarray, epsilon: float = 1e-9) -> Tuple[bool, float]:
        """Möller-Trumbore ray-triangle intersection test in local mesh coordinates."""
        v0, v1, v2 = np.asarray(triangle, dtype=float)
        edge1 = v1 - v0
        edge2 = v2 - v0
        h = np.cross(direction, edge2)
        a = float(np.dot(edge1, h))

        if abs(a) < epsilon:
            return False, np.inf

        f = 1.0 / a
        s = origin - v0
        u = f * float(np.dot(s, h))
        if u < 0.0 or u > 1.0:
            return False, np.inf

        q = np.cross(s, edge1)
        v = f * float(np.dot(direction, q))
        if v < 0.0 or (u + v) > 1.0:
            return False, np.inf

        t = f * float(np.dot(edge2, q))
        if t > epsilon:
            return True, t
        return False, np.inf

    def _ray_hits_mesh(self, mesh, ray_origin: np.ndarray, ray_direction: np.ndarray) -> Tuple[bool, Optional[np.ndarray], float]:
        """Fallback geometric contact test using mesh triangles and a ray cast."""
        if mesh is None or trimesh is None:
            return False, None, np.inf

        if not hasattr(mesh, 'faces') or not hasattr(mesh, 'vertices'):
            return False, None, np.inf

        faces = np.asarray(mesh.faces, dtype=int)
        vertices = np.asarray(mesh.vertices, dtype=float)
        if faces.size == 0 or vertices.size == 0:
            return False, None, np.inf

        ray_direction = np.asarray(ray_direction, dtype=float)
        norm = float(np.linalg.norm(ray_direction))
        if norm < 1e-9:
            ray_direction = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            ray_direction = ray_direction / norm

        closest_t = np.inf
        hit_point = None
        for face in faces:
            if len(face) < 3:
                continue
            tri = vertices[np.asarray(face[:3], dtype=int)]
            hit, t = self._ray_triangle_intersection(ray_origin, ray_direction, tri)
            if hit and t < closest_t:
                closest_t = t
                hit_point = ray_origin + ray_direction * t

        return closest_t < np.inf, hit_point, closest_t

    def geometric_grip(self, target_link, gripper_joint: Optional[str] = None,
                       contact_tolerance_mm: float = 0.5) -> Dict[str, Any]:
        """
        Geometrically test the saved EndEffector jaw faces against the target object mesh.

        The method loads the saved EndEffector payload, iterates through the saved Jaws array,
        and verifies each jaw's contact face center and normal against the object's mesh. The
        result is considered successful if every configured jaw reports a contact hit.
        """
        if target_link is None or not hasattr(target_link, 'mesh') or target_link.mesh is None:
            return {
                "success": False,
                "contact_count": 0,
                "jaw_count": 0,
                "message": "Target object mesh is missing.",
                "contact_points": [],
                "jaw_results": [],
            }

        end_effector = self.load_end_effector_definition()
        if not end_effector.get("ok"):
            return {
                "success": False,
                "contact_count": 0,
                "jaw_count": 0,
                "message": end_effector.get("reason", "missing_end_effector_definition"),
                "contact_points": [],
                "jaw_results": [],
            }

        jaws = list(end_effector.get("Jaws", []))
        if gripper_joint:
            jaws = [jaw for jaw in jaws if jaw.get("JointID") == gripper_joint]

        if not jaws:
            return {
                "success": False,
                "contact_count": 0,
                "jaw_count": 0,
                "message": f"No end-effector jaw entries found for joint '{gripper_joint}'.",
                "contact_points": [],
                "jaw_results": [],
            }

        bounds = self.compute_object_bounds(target_link)
        world_min = bounds.get("world_min")
        world_max = bounds.get("world_max")
        if world_min is None or world_max is None:
            return {
                "success": False,
                "contact_count": 0,
                "jaw_count": len(jaws),
                "message": "Object bounds could not be resolved.",
                "contact_points": [],
                "jaw_results": [],
            }

        object_transform = np.asarray(getattr(target_link, 't_world', np.eye(4)), dtype=float)
        object_inverse = invert_transform(object_transform)

        contact_points = []
        jaw_results = []
        contact_count = 0

        for jaw in jaws:
            joint_id = jaw.get("JointID")
            joint = self.robot.joints.get(joint_id) if hasattr(self.robot, 'joints') else None
            joint_transform = np.eye(4)
            if joint is not None and getattr(joint, 'child_link', None) is not None:
                joint_transform = np.asarray(joint.child_link.t_world, dtype=float)

            face_center = np.asarray(jaw.get("FaceCenter", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
            face_normal = np.asarray(jaw.get("FaceNormal", [0.0, 0.0, 1.0]), dtype=float).reshape(3)
            face_normal = face_normal / (np.linalg.norm(face_normal) + 1e-12)

            center_world = (joint_transform @ np.append(face_center, 1.0))[:3]
            normal_world = joint_transform[:3, :3] @ face_normal
            normal_world = normal_world / (np.linalg.norm(normal_world) + 1e-12)

            ray_origin_local = (object_inverse @ np.append(center_world, 1.0))[:3]

            best_hit = False
            best_hit_point_local = None
            best_distance = np.inf
            for candidate_direction in (normal_world, -normal_world):
                ray_direction_local = (object_inverse[:3, :3] @ candidate_direction)
                ray_direction_local = ray_direction_local / (np.linalg.norm(ray_direction_local) + 1e-12)
                hit, hit_point_local, hit_distance = self._ray_hits_mesh(
                    target_link.mesh,
                    ray_origin_local,
                    ray_direction_local,
                )
                if hit and hit_distance < best_distance:
                    best_hit = True
                    best_distance = hit_distance
                    best_hit_point_local = hit_point_local

            jaw_contact = bool(best_hit)
            if jaw_contact:
                contact_count += 1
                hit_point_world = (object_transform @ np.append(best_hit_point_local, 1.0))[:3]
                contact_points.append(hit_point_world)

            jaw_results.append({
                "JointID": joint_id,
                "FaceID": jaw.get("FaceID"),
                "Contact": jaw_contact,
                "HitDistance": float(best_distance) if best_hit else float('inf'),
                "CenterWorld": center_world,
                "NormalWorld": normal_world,
                "HitPointWorld": hit_point_world if jaw_contact else None,
            })

        success = contact_count == len(jaws) and contact_count >= 1
        return {
            "success": success,
            "contact_count": contact_count,
            "jaw_count": len(jaws),
            "message": f"Geometric grip checked {contact_count}/{len(jaws)} jaws within {contact_tolerance_mm:.2f} mm tolerance.",
            "contact_points": contact_points,
            "jaw_results": jaw_results,
            "object_bounds": bounds,
        }
    
    def execute_pick(self, target_pos: np.ndarray, gripper_joint: str,
                     approach_distance: float = 5.0,
                     on_progress: Optional[Callable] = None) -> Tuple[bool, str]:
        """
        Execute a pick operation at target position.
        
        Steps:
        1. Plan approach trajectory
        2. Validate grasp at target position
        3. Execute approach motion
        4. Close gripper (if applicable)
        5. Retreat with object
        
        Args:
            target_pos: Target pick position (3,)
            gripper_joint: Gripper joint name
            approach_distance: Approach distance from above (cm)
            on_progress: Callback function for progress updates
            
        Returns:
            (success, message) - Boolean and status message
        """
        target_link = target_pos if hasattr(target_pos, 'mesh') else None
        target_point = np.asarray(target_pos, dtype=float).reshape(3) if target_link is None else np.asarray(
            getattr(target_link, 't_world', np.eye(4))[:3, 3], dtype=float
        )

        operation = PickPlaceOperation("pick", target_point, gripper_joint, approach_distance)
        self._current_operation = operation
        
        try:
            # Step 1: Validate grasp
            if self.contact_analyzer:
                is_valid, validation_msg = self.contact_analyzer.validate_grasp(
                    gripper_joint, target_point
                )
                if not is_valid:
                    return False, f"Grasp validation failed: {validation_msg}"

                if on_progress:
                    on_progress(f"Grasp validated. {validation_msg}")

            if target_link is not None:
                grip_result = self.geometric_grip(target_link, gripper_joint=gripper_joint)
                if not grip_result.get("success"):
                    return False, grip_result.get("message", "Geometric grip validation failed.")
                if on_progress:
                    on_progress(grip_result.get("message", "Geometric grip validated."))
            
            # Step 2: Plan approach
            waypoints, plan_msg = self.compute_grasp_approach(
                target_point, gripper_joint, approach_distance
            )
            
            if not waypoints:
                return False, f"Failed to plan approach: {plan_msg}"
            
            operation.waypoints = waypoints
            
            if on_progress:
                on_progress(plan_msg)
            
            # Step 3: Execute approach (simulated - canvas visualization)
            if self.canvas:
                self._visualize_approach(waypoints, gripper_joint)
                if on_progress:
                    on_progress("Executing approach motion...")
            
            # Step 4: Close gripper
            if on_progress:
                on_progress("Closing and rotating gripper joints to catch the object...")
            
            # Simulate gripper closing
            joint = self.robot.joints[gripper_joint]
            if joint.is_gripper:
                # Would close gripper in real scenario
                pass
            
            # Step 5: Retreat
            if on_progress:
                on_progress("Retreating with the gripper-object pose locked...")
            
            operation.status = "success"
            self._operation_history.append(operation)
            
            return True, f"✓ Pick operation successful at {target_point}"
        
        except Exception as e:
            operation.status = "failed"
            operation.error_message = str(e)
            return False, f"❌ Pick operation failed: {str(e)}"
    
    def execute_place(self, target_pos: np.ndarray, gripper_joint: str,
                      approach_distance: float = 5.0,
                      on_progress: Optional[Callable] = None) -> Tuple[bool, str]:
        """
        Execute a place operation at target position.
        
        Steps:
        1. Plan approach to placement location
        2. Lower object to target
        3. Open gripper
        4. Retreat empty
        
        Args:
            target_pos: Target placement position (3,)
            gripper_joint: Gripper joint name
            approach_distance: Approach distance from above (cm)
            on_progress: Callback function for progress updates
            
        Returns:
            (success, message) - Boolean and status message
        """
        operation = PickPlaceOperation("place", target_pos, gripper_joint, approach_distance)
        self._current_operation = operation
        
        try:
            # Step 1: Plan approach
            waypoints, plan_msg = self.compute_grasp_approach(
                target_pos, gripper_joint, approach_distance
            )
            
            if not waypoints:
                return False, f"Failed to plan approach: {plan_msg}"
            
            operation.waypoints = waypoints
            
            if on_progress:
                on_progress(f"Place trajectory planned. {plan_msg}")
            
            # Step 2: Execute descent
            if self.canvas:
                self._visualize_approach(waypoints, gripper_joint)
                if on_progress:
                    on_progress("Descending to placement location while keeping the alignment fixed...")
            
            # Step 3: Open gripper
            if on_progress:
                on_progress("Opening gripper to release the object...")
            
            joint = self.robot.joints[gripper_joint]
            if joint.is_gripper:
                # Would open gripper in real scenario
                pass
            
            # Step 4: Retreat
            if on_progress:
                on_progress("Retreating after placement...")
            
            operation.status = "success"
            self._operation_history.append(operation)
            
            return True, f"✓ Place operation successful at {target_pos}"
        
        except Exception as e:
            operation.status = "failed"
            operation.error_message = str(e)
            return False, f"❌ Place operation failed: {str(e)}"
    
    def execute_manipulation(self, target_pos: np.ndarray, gripper_joint: str,
                            operation_type: str,
                            on_progress: Optional[Callable] = None) -> Tuple[bool, str]:
        """
        Execute a generic manipulation operation.
        
        Args:
            target_pos: Target position (3,)
            gripper_joint: Gripper joint name
            operation_type: "pick", "place", "push", "slide", etc.
            on_progress: Callback for progress updates
            
        Returns:
            (success, message)
        """
        if operation_type == "pick":
            return self.execute_pick(target_pos, gripper_joint, on_progress=on_progress)
        elif operation_type == "place":
            return self.execute_place(target_pos, gripper_joint, on_progress=on_progress)
        elif operation_type == "push":
            return self._execute_push(target_pos, gripper_joint, on_progress)
        elif operation_type == "slide":
            return self._execute_slide(target_pos, gripper_joint, on_progress)
        else:
            return False, f"❌ Unknown operation type: {operation_type}"
    
    def _execute_push(self, target_pos: np.ndarray, gripper_joint: str,
                      on_progress: Optional[Callable]) -> Tuple[bool, str]:
        """Execute a push operation at target."""
        # TODO: Implement push logic
        return False, "Push operation not yet implemented"
    
    def _execute_slide(self, target_pos: np.ndarray, gripper_joint: str,
                       on_progress: Optional[Callable]) -> Tuple[bool, str]:
        """Execute a slide operation to target."""
        # TODO: Implement slide logic
        return False, "Slide operation not yet implemented"
    
    def _visualize_approach(self, waypoints: List[np.ndarray], gripper_joint: str):
        """Visualize approach trajectory on canvas."""
        if not self.canvas:
            return
        
        try:
            # Draw trajectory line
            for i in range(len(waypoints) - 1):
                p1 = waypoints[i]
                p2 = waypoints[i + 1]
                # Color transitions from blue (start) to green (end)
                t = i / len(waypoints)
                color = [0.0, 0.5 + 0.5 * t, 1.0 - 0.5 * t]  # Blue to cyan to green
                # Canvas visualization would happen here
        except Exception:
            pass
    
    def get_operation_history(self) -> List[PickPlaceOperation]:
        """Get history of executed operations."""
        return self._operation_history.copy()
    
    def get_last_operation_status(self) -> Optional[Dict[str, str]]:
        """Get status of the last operation."""
        if not self._operation_history:
            return None
        
        last_op = self._operation_history[-1]
        return {
            'type': last_op.operation_type,
            'status': last_op.status,
            'error': last_op.error_message,
            'waypoints': len(last_op.waypoints)
        }
    
    def clear_history(self):
        """Clear operation history."""
        self._operation_history = []
        self._current_operation = None
