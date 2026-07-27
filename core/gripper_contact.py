"""
Gripper Contact Analysis - Analyzes selected contact faces for grasp operations.

This module provides:
- Contact face retrieval and analysis
- Contact point computation on target objects
- Grasp stability validation
- Collision checking at contact surfaces
"""

import numpy as np
from typing import Dict, List, Optional, Tuple, Set
try:
    from scipy.spatial import ConvexHull
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class ContactFace:
    """Represents a single contact face on a gripper."""
    def __init__(self, face_id: int, vertices: np.ndarray, center: np.ndarray, 
                 normal: np.ndarray, area: float):
        self.face_id = face_id
        self.vertices = vertices  # Shape: (3, 3) for triangle or (4, 3) for quad
        self.center = center      # (3,) center point
        self.normal = normal      # (3,) outward normal
        self.area = area          # Surface area
        self.color = "gray"


class GripperContactAnalyzer:
    """
    Analyzes gripper contact faces and validates grasping operations.
    
    Manages:
    - Selected contact faces per gripper joint
    - Contact point computation
    - Grasp stability analysis
    - Collision validation at contact surfaces
    """
    
    def __init__(self, robot):
        """
        Initialize the contact analyzer.
        
        Args:
            robot: Robot object with links, joints, and kinematics
        """
        self.robot = robot
        self._contact_cache = {}  # Cache of face data
        
    def get_contact_faces(self, joint_name: str, face_role: str = "contact_face") -> Dict[int, ContactFace]:
        """
        Retrieve selected contact faces for a gripper joint.
        
        Args:
            joint_name: Name of the gripper joint
            face_role: Face role type (contact_face, left_jaw, right_jaw, suction_face)
            
        Returns:
            Dictionary mapping face_id to ContactFace objects
        """
        if joint_name not in self.robot.joints:
            return {}
        
        joint = self.robot.joints[joint_name]
        link = joint.child_link
        
        # Get saved face selection from joint configuration
        if not hasattr(joint, 'contact_face_selection'):
            return {}
        
        selection = joint.contact_face_selection
        if link.name not in selection:
            return {}
        
        role_selection = selection[link.name].get(face_role, {})
        contact_faces = {}
        
        # Retrieve face data from cache or mesh
        for face_id in role_selection.keys():
            face_data = role_selection[face_id]
            if face_id not in self._contact_cache:
                self._cache_face_data(link, face_id, face_data)
            
            cached = self._contact_cache.get(face_id)
            if cached:
                contact_faces[face_id] = cached
        
        return contact_faces
    
    def _cache_face_data(self, link, face_id: int, face_data: dict):
        """Cache face geometric data."""
        try:
            vertices = np.array(face_data.get('vertices', []))
            center = np.array(face_data.get('center', [0, 0, 0]))
            normal = np.array(face_data.get('normal', [0, 0, 1]))
            area = float(face_data.get('area', 0.01))
            
            if vertices.shape[0] > 0:
                face = ContactFace(face_id, vertices, center, normal, area)
                self._contact_cache[face_id] = face
        except Exception:
            pass
    
    def compute_contact_points(self, joint_name: str, target_link, 
                               face_role: str = "contact_face") -> List[np.ndarray]:
        """
        Compute contact points between gripper faces and a target object.
        
        Args:
            joint_name: Gripper joint name
            target_link: Target link/object to contact
            face_role: Face role to use for contact
            
        Returns:
            List of contact point positions (3D arrays) in world frame
        """
        contact_faces = self.get_contact_faces(joint_name, face_role)
        if not contact_faces:
            return []
        
        joint = self.robot.joints[joint_name]
        gripper_link = joint.child_link
        
        contact_points = []
        
        # Transform face centers to world frame
        for face_id, face in contact_faces.items():
            # Face center is in gripper link frame, transform to world
            face_center_world = self._transform_point(
                face.center, 
                gripper_link.t_world
            )
            contact_points.append(face_center_world)
        
        return contact_points
    
    def _transform_point(self, point: np.ndarray, transform: np.ndarray) -> np.ndarray:
        """Transform a 3D point by a 4x4 transformation matrix."""
        p_homo = np.append(point, 1.0)
        p_transformed = transform @ p_homo
        return p_transformed[:3]
    
    def validate_grasp(self, joint_name: str, target_position: np.ndarray,
                       target_link=None, collision_checker=None) -> Tuple[bool, str]:
        """
        Validate if selected contact faces can maintain a grasp at target position.
        
        Checks:
        1. Contact faces exist and are selected
        2. Gripper can reach target position
        3. No collision at contact surfaces
        4. Grasp is geometrically stable
        
        Args:
            joint_name: Gripper joint name
            target_position: Target position in world frame (3,)
            target_link: Target link/object (optional)
            collision_checker: Collision checker function (optional)
            
        Returns:
            (is_valid, message) - Boolean and descriptive message
        """
        # Check 1: Contact faces selected
        contact_faces = self.get_contact_faces(joint_name)
        if not contact_faces:
            return False, "❌ No contact faces selected"
        
        num_faces = len(contact_faces)
        
        # Check 2: Multiple contact points for stability
        if num_faces < 2:
            return False, f"⚠️  Need at least 2 contact faces (have {num_faces})"
        
        # Check 3: Check joint reachability
        joint = self.robot.joints[joint_name]
        gripper_link = joint.child_link
        
        # Simple distance check - gripper TCP should be close to target
        current_tcp = gripper_link.t_world[:3, 3]
        distance = np.linalg.norm(current_tcp - target_position)
        
        if distance > 20.0:  # More than 20cm away
            return False, f"⚠️  Gripper too far from target ({distance:.1f}cm)"
        
        # Check 4: Collision at contact (if checker provided)
        if collision_checker:
            contact_points = self.compute_contact_points(joint_name, target_link)
            collision_free = not collision_checker(contact_points)
            if not collision_free:
                return False, "❌ Collision detected at contact faces"
        
        # Check 5: Contact coverage
        total_area = sum(f.area for f in contact_faces.values())
        if total_area < 0.5:  # Less than 0.5 cm² total
            return False, f"⚠️  Contact area too small ({total_area:.2f}cm²)"
        
        return True, f"✓ Valid grasp ({num_faces} faces, {total_area:.2f}cm²)"
    
    def get_grasp_quality_metrics(self, joint_name: str, 
                                   target_position: np.ndarray) -> Dict[str, float]:
        """
        Compute grasp quality metrics for evaluation.
        
        Returns metrics like:
        - num_contacts: Number of contact points
        - contact_area: Total contact area
        - contact_spread: Spatial spread of contact points
        - stability_score: 0-1 quality metric
        
        Args:
            joint_name: Gripper joint name
            target_position: Target grasp position
            
        Returns:
            Dictionary of quality metrics
        """
        contact_faces = self.get_contact_faces(joint_name)
        contact_points = self.compute_contact_points(joint_name, None)
        
        metrics = {
            'num_contacts': len(contact_faces),
            'contact_area': sum(f.area for f in contact_faces.values()),
            'contact_spread': 0.0,
            'stability_score': 0.0,
        }
        
        if len(contact_points) < 2:
            return metrics
        
        # Compute spread of contact points
        points_array = np.array(contact_points)
        centroid = np.mean(points_array, axis=0)
        distances = np.linalg.norm(points_array - centroid, axis=1)
        metrics['contact_spread'] = float(np.max(distances)) if len(distances) > 0 else 0.0
        
        # Simple stability score: more faces and better spread = better
        num_score = min(len(contact_faces) / 4.0, 1.0)  # Max 4 faces
        area_score = min(metrics['contact_area'] / 5.0, 1.0)  # Max 5cm²
        spread_score = min(metrics['contact_spread'] / 10.0, 1.0)  # Max 10cm spread
        
        metrics['stability_score'] = float(
            0.4 * num_score + 0.3 * area_score + 0.3 * spread_score
        )
        
        return metrics
    
    def get_contact_normals(self, joint_name: str, 
                           face_role: str = "contact_face") -> List[np.ndarray]:
        """
        Get outward-facing normals of all contact faces.
        
        Args:
            joint_name: Gripper joint name
            face_role: Face role to use
            
        Returns:
            List of normal vectors (3D arrays) in world frame
        """
        contact_faces = self.get_contact_faces(joint_name, face_role)
        if not contact_faces:
            return []
        
        joint = self.robot.joints[joint_name]
        gripper_link = joint.child_link
        
        normals = []
        rotation = gripper_link.t_world[:3, :3]
        
        for face in contact_faces.values():
            # Transform normal to world frame (rotation only, no translation)
            normal_world = rotation @ face.normal
            normals.append(normal_world)
        
        return normals
    
    def compute_grasp_force_closure(self, joint_name: str, 
                                     friction_coeff: float = 0.5) -> Tuple[bool, float]:
        """
        Check if selected contact faces can achieve force closure grasp.
        
        Force closure means the gripper can apply forces in any direction
        to resist object motion.
        
        Args:
            joint_name: Gripper joint name
            friction_coeff: Coefficient of friction at contact
            
        Returns:
            (has_force_closure, quality_metric) - Boolean and 0-1 quality score
        """
        contact_faces = self.get_contact_faces(joint_name)
        contact_points = self.compute_contact_points(joint_name, None)
        
        if len(contact_points) < 3:
            # Need at least 3 contact points for potential force closure in 3D
            return False, 0.0
        
        try:
            points_array = np.array(contact_points)
            
            # Check if points form a 3D configuration (not coplanar)
            if len(points_array) >= 3:
                centroid = np.mean(points_array, axis=0)
                relative_points = points_array - centroid
                
                # Compute rank of point configuration
                u, s, vt = np.linalg.svd(relative_points)
                rank = np.sum(s > 1e-10)
                
                if rank >= 3:
                    # Good 3D configuration for force closure
                    quality = min(1.0, float(s[0] / (s[2] + 1e-10)) * 0.5)
                    return True, quality
            
            return False, 0.0
        except Exception:
            return False, 0.0
