"""
Contact Face Selector - Manages gripper contact face selection and validation.

This module provides the ContactFaceSelector class which handles:
- Face picking in 3D viewport with visual feedback
- Selection state tracking (selected vs unselected faces)
- Face role organization (Contact, Left Jaw, Right Jaw, etc.)
- Persistence of face selections in joint configuration
- Collision validation using only selected contact faces
"""

import numpy as np
from PyQt5 import QtCore, QtWidgets
from typing import Dict, List, Optional, Set, Tuple


class FaceRole:
    """Enumeration of possible face roles for grippers."""
    CONTACT_FACE = "contact_face"
    LEFT_JAW = "left_jaw"
    RIGHT_JAW = "right_jaw"
    SUCTION_FACE = "suction_face"
    
    ALL_ROLES = [CONTACT_FACE, LEFT_JAW, RIGHT_JAW, SUCTION_FACE]


class ContactFaceSelector(QtCore.QObject):
    """
    Manages face selection for gripper contact surfaces.
    
    Signals:
        - selection_changed(joint_name, selected_faces)
        - mode_changed(is_active)
    """
    
    selection_changed = QtCore.pyqtSignal(str, dict)  # joint_name, selected_faces dict
    mode_changed = QtCore.pyqtSignal(bool)  # is_active
    face_count_changed = QtCore.pyqtSignal(int)  # count
    
    def __init__(self, main_window):
        super().__init__()
        self.mw = main_window
        self.active = False
        self.current_joint_name: Optional[str] = None
        self.current_link_name: Optional[str] = None
        self.current_face_role = FaceRole.CONTACT_FACE
        
        # Selected faces: {link_name: {role: {face_id: face_data}}}
        self._selected_faces: Dict[str, Dict[str, Dict[int, dict]]] = {}
        
        # Face data cache: {link_name: {face_id: {center, normal, area, ...}}}
        self._face_cache: Dict[str, Dict[int, dict]] = {}
        
        # Hovered face for visual feedback
        self._hovered_face: Optional[Tuple[str, int]] = None
        self._hover_color = "yellow"
        self._selected_color = "green"
        self._default_color = "gray"
    
    def start_selection_mode(self, joint_name: str, link_name: str, face_role: str = FaceRole.CONTACT_FACE):
        """Enter face selection mode for a specific gripper joint."""
        if self.active:
            self.cancel_selection()
        
        self.active = True
        self.current_joint_name = joint_name
        self.current_link_name = link_name
        self.current_face_role = face_role
        
        # Initialize selection dict for this link if needed
        if link_name not in self._selected_faces:
            self._selected_faces[link_name] = {}
        if face_role not in self._selected_faces[link_name]:
            self._selected_faces[link_name][face_role] = {}
        
        # Try to load previously saved selection
        self.load_selection_from_joint(joint_name)
        
        self.mode_changed.emit(True)
        self._log(f"Face selection mode active for {joint_name} ({link_name}) - Role: {face_role}")
    
    def end_selection_mode(self):
        """Exit face selection mode."""
        self.active = False
        self.current_joint_name = None
        self.current_link_name = None
        self.mode_changed.emit(False)
        self._log("Face selection mode deactivated")
    
    def on_face_picked(self, link_name: str, world_center: np.ndarray, world_normal: np.ndarray):
        """Handle a face being picked in the 3D viewport."""
        if not self.active or not self.current_link_name or link_name != self.current_link_name:
            return
        
        # Find the closest face in the mesh
        face_id = self._find_closest_face(link_name, world_center, world_normal)
        if face_id is None:
            self._log(f"Could not identify face on {link_name}")
            return
        
        self._toggle_face_selection(link_name, face_id, world_center, world_normal)
    
    def on_face_hovered(self, link_name: str, world_center: np.ndarray, world_normal: np.ndarray):
        """Handle face hover for visual feedback (yellow highlight)."""
        if not self.active or not self.current_link_name or link_name != self.current_link_name:
            return
        
        face_id = self._find_closest_face(link_name, world_center, world_normal)
        if face_id is None:
            self._update_hover(None)
            return
        
        old_hover = self._hovered_face
        self._hovered_face = (link_name, face_id)
        
        if old_hover != self._hovered_face:
            self._update_face_visual(old_hover, revert=True)
            self._update_face_visual(self._hovered_face, color=self._hover_color)
    
    def _find_closest_face(self, link_name: str, world_center: np.ndarray, 
                          world_normal: np.ndarray) -> Optional[int]:
        """Find the mesh face closest to the given world position and normal."""
        try:
            if link_name not in self.mw.robot.links:
                return None
            
            link = self.mw.robot.links[link_name]
            mesh = getattr(link, 'mesh', None)
            if mesh is None:
                return None
            
            # Get face metadata
            facet_centers = np.asarray(getattr(mesh, 'facets_origin', []), dtype=float)
            facet_normals = np.asarray(getattr(mesh, 'facets_normal', []), dtype=float)
            
            if len(facet_centers) == 0 or len(facet_normals) == 0:
                return None
            
            # Transform world center to local coordinates
            t_world_inv = np.linalg.inv(link.t_world)
            local_center = (t_world_inv @ np.append(world_center, 1.0))[:3]
            local_normal = np.linalg.inv(link.t_world[:3, :3]) @ world_normal
            local_normal_norm = np.linalg.norm(local_normal)
            if local_normal_norm > 1e-9:
                local_normal = local_normal / local_normal_norm
            
            # Find closest face by center distance and normal similarity
            best_face_id = None
            best_score = float('inf')
            
            for face_id in range(len(facet_centers)):
                center_delta = np.linalg.norm(facet_centers[face_id] - local_center)
                face_normal = facet_normals[face_id]
                face_normal_norm = np.linalg.norm(face_normal)
                if face_normal_norm > 1e-9:
                    face_normal = face_normal / face_normal_norm
                normal_delta = 1.0 - abs(np.dot(face_normal, local_normal))
                
                # Weighted score: prefer closer faces, then faces with matching normals
                score = center_delta + 0.3 * normal_delta
                
                if score < best_score:
                    best_score = score
                    best_face_id = face_id
            
            # Cache the face data for later
            if best_face_id is not None and link_name not in self._face_cache:
                self._face_cache[link_name] = {}
            if best_face_id is not None:
                self._face_cache[link_name][best_face_id] = {
                    'center_local': facet_centers[best_face_id],
                    'normal_local': facet_normals[best_face_id],
                    'center_world': world_center,
                    'normal_world': world_normal,
                }
            
            return best_face_id
        except Exception as e:
            self._log(f"Error finding closest face: {e}", level="error")
            return None
    
    def _toggle_face_selection(self, link_name: str, face_id: int, 
                              world_center: np.ndarray, world_normal: np.ndarray):
        """Toggle selection state of a face."""
        role = self.current_face_role
        is_selected = face_id in self._selected_faces.get(link_name, {}).get(role, {})
        
        if is_selected:
            # Deselect
            del self._selected_faces[link_name][role][face_id]
            self._update_face_visual((link_name, face_id), revert=True)
            self._log(f"Deselected face {face_id} on {link_name}")
        else:
            # Select
            self._selected_faces[link_name][role][face_id] = {
                'center_local': self._face_cache.get(link_name, {}).get(face_id, {}).get('center_local'),
                'normal_local': self._face_cache.get(link_name, {}).get(face_id, {}).get('normal_local'),
                'center_world': world_center,
                'normal_world': world_normal,
            }
            self._update_face_visual((link_name, face_id), color=self._selected_color)
            self._log(f"Selected face {face_id} on {link_name}")
        
        self._update_selection_counter()
        self.selection_changed.emit(self.current_joint_name, self.get_selected_faces_dict())
    
    def _update_face_visual(self, face_key: Optional[Tuple[str, int]], revert: bool = False, 
                           color: Optional[str] = None):
        """Update visual representation of a face in the 3D viewport."""
        if not hasattr(self.mw, 'canvas') or face_key is None:
            return
        
        link_name, face_id = face_key
        if revert:
            # Restore default color
            self.mw.canvas.reset_face_color(link_name, face_id)
        elif color:
            self.mw.canvas.highlight_face(link_name, face_id, color)
    
    def _update_selection_counter(self):
        """Emit signal with current selection count."""
        total = sum(
            len(face_dict) 
            for link_faces in self._selected_faces.values()
            for face_dict in link_faces.values()
        )
        self.face_count_changed.emit(total)
    
    def _update_hover(self, face_key: Optional[Tuple[str, int]]):
        """Update hovered face."""
        old_hover = self._hovered_face
        self._hovered_face = face_key
        if old_hover and old_hover != face_key:
            self._update_face_visual(old_hover, revert=True)
        if face_key:
            self._update_face_visual(face_key, color=self._hover_color)
    
    def get_selected_faces_dict(self) -> dict:
        """Get all selected faces organized by link and role."""
        return {
            link_name: {
                role: {
                    str(face_id): face_data
                    for face_id, face_data in face_dict.items()
                }
                for role, face_dict in link_faces.items()
            }
            for link_name, link_faces in self._selected_faces.items()
        }
    
    def get_selected_face_count(self) -> int:
        """Get total number of selected faces."""
        return sum(
            len(face_dict) 
            for link_faces in self._selected_faces.values()
            for face_dict in link_faces.values()
        )
    
    def clear_selection(self):
        """Clear all selected faces for current role."""
        if not self.current_link_name:
            return
        
        role = self.current_face_role
        if self.current_link_name in self._selected_faces and role in self._selected_faces[self.current_link_name]:
            for face_id in list(self._selected_faces[self.current_link_name][role].keys()):
                self._update_face_visual((self.current_link_name, face_id), revert=True)
            self._selected_faces[self.current_link_name][role].clear()
            self._update_selection_counter()
            self._log(f"Cleared all selected faces for {role}")
    
    def save_selection_to_joint(self, joint_name: str):
        """Save selected faces to joint configuration."""
        if joint_name not in self.mw.robot.joints:
            self._log(f"Joint {joint_name} not found", level="error")
            return False
        
        joint = self.mw.robot.joints[joint_name]
        selected_faces_dict = self.get_selected_faces_dict()
        
        # Store in joint object
        joint.contact_face_selection = selected_faces_dict
        
        # Also update joint_tab cache if available
        if hasattr(self.mw, 'joint_tab') and joint.child_link:
            link_name = joint.child_link.name
            if link_name in self.mw.joint_tab.joints:
                self.mw.joint_tab.joints[link_name]['contact_face_selection'] = selected_faces_dict
        
        self._log(f"Saved {self.get_selected_face_count()} selected faces to {joint_name}")
        return True
    
    def load_selection_from_joint(self, joint_name: str) -> bool:
        """Load previously saved face selection from joint configuration."""
        if joint_name not in self.mw.robot.joints:
            return False
        
        joint = self.mw.robot.joints[joint_name]
        saved_selection = getattr(joint, 'contact_face_selection', None)
        
        if not isinstance(saved_selection, dict):
            return False
        
        # Restore saved selection
        self._selected_faces = saved_selection.copy()
        self._update_selection_counter()
        self._log(f"Loaded {self.get_selected_face_count()} selected faces from {joint_name}")
        return True
    
    def cancel_selection(self):
        """Cancel current selection without saving."""
        # Clear visuals
        for face_key in self._hovered_face, *self._selected_faces.keys():
            if face_key:
                self._update_face_visual(face_key, revert=True)
        
        self.end_selection_mode()
        self._log("Face selection cancelled (not saved)")
    
    def _log(self, message: str, level: str = "info"):
        """Log message to main window."""
        if hasattr(self.mw, 'log'):
            self.mw.log(f"[ContactFaceSelector] {message}")
        if level == "error" and hasattr(self.mw, 'show_toast'):
            self.mw.show_toast(message, "error")
