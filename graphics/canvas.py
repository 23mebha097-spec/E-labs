import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt5 import QtWidgets, QtCore, QtGui
import numpy as np
import vtkmodules.vtkRenderingCore as vtkRenderingCore
import vtkmodules.vtkCommonCore as vtkCommonCore

class RobotCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        
        # Create the pyvista interactor
        self.plotter = QtInteractor(self)
        self.layout.addWidget(self.plotter.interactor)
        
        # Premium Light Theme Environment
        self.plotter.set_background("white")
        self.plotter.add_axes()
        
        self.grid_units_per_cm = 10.0 
        self.grid_cm_size = 1000.0    # 10m workspace
        
        # Initialize dynamic grid system
        self._init_custom_grids()
        self._init_axis_labels()
        
        # Observe camera changes for grid/label updates
        self.plotter.interactor.AddObserver("InteractionEvent", self._on_camera_change)
        self.plotter.interactor.AddObserver("ModifiedEvent", self._on_camera_change)
        
        # UI Elements
        try:
            self.plotter.remove_scalar_bar()
        except:
            pass
        
        # Initial grid update
        QtCore.QTimer.singleShot(100, self._on_camera_change)
        
        self.actors = {} # Link name -> actor
        self.selected_name = None
        self.is_dragging = False
        self.last_pos = None
        self.fixed_actors = set() # Set of actor names that cannot be picked or moved
       
        # WE DISABLE PyVista's built-in picking to avoid conflicts
        # Instead, we will use dedicated vtk pickers for surgical precision
        self.picker = vtkRenderingCore.vtkPropPicker()
        self.cell_picker = vtkRenderingCore.vtkCellPicker()
        self.cell_picker.SetTolerance(0.0005)
        
        # Override interactor events
        self.plotter.interactor.AddObserver("MouseMoveEvent", self._on_mouse_move)
        self.plotter.interactor.AddObserver("LeftButtonPressEvent", self._on_left_down)
        self.plotter.interactor.AddObserver("LeftButtonReleaseEvent", self._on_left_up)
        self.plotter.interactor.AddObserver("MouseWheelForwardEvent", self._on_wheel_forward)
        self.plotter.interactor.AddObserver("MouseWheelBackwardEvent", self._on_wheel_backward)

        # 3D Orientation Cube (Standard Navigation)
        self.plotter.add_camera_orientation_widget()
        # Optionally style it if needed (PyVista default is high contrast and clear)
        
        # Initial grid update
        QtCore.QTimer.singleShot(100, self._on_camera_change)

        # Keyboard Shortcut: Escape to deselect everything
        self.plotter.add_key_event("Escape", self.deselect_all)

        self.on_face_picked_callback = None
        self.on_drop_callback = None
        self.on_deselect_callback = None
        self.picking_face = False
        self.picking_color = "orange"
        self.enable_drag = True
        self._selection_dim_actors = []
        
        self.interaction_mode = "rotate" # 'rotate'
        self.picking_focus_point = False  # Focus point picking mode
        
        # --- 3D ENGINE HUD (Live Point Location) ---
        self.plotter.add_text(
            "LIVE POINT: X: 0.00, Y: 0.00, Z: 0.00 cm",
            position='upper_left',
            font_size=12,
            color='#1565c0',
            name="live_point_hud",
            shadow=True
        )

    def _dist_point_to_segment(self, p, a, b):
        """Calculates distance from point p to line segment (a, b)"""
        pa = p - a
        ba = b - a
        denom = np.dot(ba, ba)
        if denom < 1e-18: return np.linalg.norm(pa)
        h = np.clip(np.dot(pa, ba) / denom, 0, 1)
        return np.linalg.norm(pa - ba * h)

    def _vtk_mat_to_numpy(self, vtk_mat):
        """Safely converts a vtkMatrix4x4 (or numpy array) to a 4x4 numpy array."""
        if vtk_mat is None:
            return np.eye(4)
        if isinstance(vtk_mat, np.ndarray):
            return vtk_mat
        # It's a vtkMatrix4x4 object
        m = np.eye(4)
        for i in range(4):
            for j in range(4):
                m[i, j] = vtk_mat.GetElement(i, j)
        return m

    def _on_face_pick_click(self, click_pos):
        """Enhanced face picking - detects geometric features and picks specific loops."""
        self.cell_picker.Pick(click_pos[0], click_pos[1], 0, self.plotter.renderer)
        cell_id = self.cell_picker.GetCellId()
        actor = self.cell_picker.GetActor()

        if cell_id != -1 and actor:
            link_name = next((name for name, a in self.actors.items() if a == actor), None)
            
            if link_name:
                # 0. IMPORTANT: Clean the mesh for connectivity! 
                # STL meshes often have 'unwelded' vertices which break feature growth.
                raw_mesh = pv.wrap(actor.GetMapper().GetInput())
                mesh = raw_mesh.clean(tolerance=1e-5) # Merge coincident points
                
                mat = self._vtk_mat_to_numpy(actor.user_matrix)
                inv_mat = np.linalg.inv(mat)
                
                # 1. Get seed face normal
                seed_normal = self._get_face_normal(mesh, cell_id)
                
                # 2. Grow feature region (High tolerance to capture whole faces/cylinders)
                # Now that mesh is cleaned, neighbors will be found correctly.
                feature_cells = self._grow_feature_region(mesh, cell_id, seed_normal, angle_tol=35.0)
                
                # 3. Extract all possible alignment loops (outer boundaries and internal holes)
                loops = self._extract_boundary_edges(mesh, feature_cells)
                
                # 4. Target selection based on click proximity
                world_pick_pt = np.array(self.cell_picker.GetPickPosition())
                local_pick_pt = (inv_mat @ np.append(world_pick_pt, 1))[:3]
                
                if loops:
                    # Find loop closest to click point (surgically targets holes)
                    loop_dists = []
                    for loop in loops:
                        loop_pts = np.array([mesh.GetPoint(edge[0]) for edge in loop])
                        d = np.min(np.linalg.norm(loop_pts - local_pick_pt, axis=1))
                        loop_dists.append(d)
                    
                    best_loop = loops[np.argmin(loop_dists)]
                    center, normal = self._calc_loop_center_normal(mesh, best_loop, seed_normal)
                    
                    # Highlight ONLY the specific edge/loop selected
                    self._highlight_feature_boundary(mesh, best_loop, link_name, mat, color=self.picking_color)
                else:
                    # FALLBACK: Use centroid of the ENTIRE feature cluster (e.g. whole disk)
                    all_cell_pts = []
                    for cid in feature_cells:
                        c = mesh.GetCell(cid)
                        for i in range(c.GetNumberOfPoints()):
                            all_cell_pts.append(mesh.GetPoint(c.GetPointId(i)))
                    
                    center = np.mean(all_cell_pts, axis=0) if all_cell_pts else np.zeros(3)
                    normal = seed_normal
                    self._highlight_feature_surface(mesh, feature_cells, link_name, mat, color=self.picking_color)

                # 5. Transform to World Coordinates
                rot = mat[:3, :3]
                world_normal = rot @ normal
                norm_len = np.linalg.norm(world_normal)
                if norm_len > 1e-9:
                    world_normal /= norm_len
                    
                world_center = (mat @ np.append(center, 1))[:3]
                
                # Visual Feedback: Show a small sphere at the PICKED CENTER
                self.plotter.add_mesh(pv.Sphere(radius=0.3 * self.grid_units_per_cm, center=world_center), 
                                    color="white", name=f"pick_center_marker", pickable=False)
                
                if self.on_face_picked_callback:
                    self.on_face_picked_callback(link_name, world_center, world_normal)
                
                self.picking_face = False
                self.plotter.render()
                return True
        return False

    def _get_face_normal(self, mesh, cell_id):
        """Get normal vector of a face"""
        cell = mesh.GetCell(cell_id)
        points = cell.GetPoints()
        pts = np.array([points.GetPoint(i) for i in range(points.GetNumberOfPoints())])
        
        if len(pts) >= 3:
            v1 = pts[1] - pts[0]
            v2 = pts[2] - pts[0]
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            return normal / norm if norm > 0 else np.array([0,0,1])
        return np.array([0,0,1])

    def _grow_feature_region(self, mesh, seed_id, seed_normal, angle_tol=40.0):
        """Grow region of faces using neighbor-to-neighbor normal comparison.
        This correctly handles both flat faces AND curved surfaces (cylinders).
        For flat faces: all normals are nearly identical, so seed comparison works.
        For cylinders: adjacent faces differ slightly, so we compare neighbor-to-neighbor.
        """
        mesh.BuildLinks()
        
        visited = set()
        feature = []
        # Store (cell_id, parent_normal) so we compare to the NEIGHBOR's normal
        to_visit = [(seed_id, seed_normal)]
        
        cos_tol = np.cos(np.radians(angle_tol))
        
        # Increased limit for high-resolution parts
        while to_visit and len(feature) < 30000:
            current, parent_normal = to_visit.pop(0)
            if current in visited or current < 0 or current >= mesh.GetNumberOfCells():
                continue
                
            visited.add(current)
            current_normal = self._get_face_normal(mesh, current)
            
            # Compare to PARENT normal (neighbor-to-neighbor), not seed
            # This allows smooth growth around curved surfaces
            similarity = abs(np.dot(current_normal, parent_normal))
            if similarity >= cos_tol:
                feature.append(current)
                
                # Find all adjacent cells through shared edges
                cell = mesh.GetCell(current)
                n_edges = cell.GetNumberOfEdges()
                
                for i in range(n_edges):
                    edge = cell.GetEdge(i)
                    p1 = edge.GetPointId(0)
                    p2 = edge.GetPointId(1)
                    
                    id_list = vtkCommonCore.vtkIdList()
                    mesh.GetCellEdgeNeighbors(current, p1, p2, id_list)
                    
                    for j in range(id_list.GetNumberOfIds()):
                        neighbor_id = id_list.GetId(j)
                        if neighbor_id not in visited:
                            # Pass CURRENT normal as parent for next comparison
                            to_visit.append((neighbor_id, current_normal))
                    
        return feature if feature else [seed_id]

    def _extract_boundary_edges(self, mesh, cell_ids):
        """Extract boundary edges and return them as a list of independent continuous loops"""
        edge_count = {}
        for cid in cell_ids:
            cell = mesh.GetCell(cid)
            n_pts = cell.GetNumberOfPoints()
            for i in range(n_pts):
                p1 = cell.GetPointId(i)
                p2 = cell.GetPointId((i + 1) % n_pts)
                edge = tuple(sorted([p1, p2]))
                edge_count[edge] = edge_count.get(edge, 0) + 1
        
        # Boundary edges appear only once in the set of faces
        boundary = [e for e, count in edge_count.items() if count == 1]
        
        if not boundary:
            return []
            
        return self._sort_edges_into_loops(boundary)

    def _sort_edges_into_loops(self, edges):
        """Sort disconnected edges into separate continuous boundary loops"""
        if not edges:
            return []
            
        point_to_edges = {}
        for edge in edges:
            for pt in edge:
                if pt not in point_to_edges:
                    point_to_edges[pt] = []
                point_to_edges[pt].append(edge)
        
        all_loops = []
        remaining = set(edges)
        
        while remaining:
            loop = []
            current_edge = remaining.pop()
            loop.append(current_edge)
            
            # Trace from the current end point
            current_end = current_edge[1]
            
            while True:
                next_edge = None
                for candidate in point_to_edges.get(current_end, []):
                    if candidate in remaining:
                        next_edge = candidate
                        break
                
                if next_edge is None:
                    break
                    
                # Orient edge correctly to maintain flow
                if next_edge[0] == current_end:
                    loop.append(next_edge)
                    current_end = next_edge[1]
                else:
                    loop.append((next_edge[1], next_edge[0]))
                    current_end = next_edge[0]
                    
                remaining.discard(next_edge)
            all_loops.append(loop)
            
        return all_loops

    def _calc_loop_center_normal(self, mesh, loop, seed_normal):
        """Calculates robust center (length-weighted) and normal for a loop."""
        edge_pts = []
        for edge in loop:
            p1 = np.array(mesh.GetPoint(edge[0]))
            p2 = np.array(mesh.GetPoint(edge[1]))
            edge_pts.append((p1, p2))
            
        if not edge_pts:
            return np.zeros(3), seed_normal
            
        # 1. Length-Weighted Centroid
        # This is critical for non-uniform meshes (e.g. circles with unequal edge lengths)
        total_len = 0
        sum_pos = np.zeros(3)
        for p1, p2 in edge_pts:
            length = np.linalg.norm(p2 - p1)
            if length < 1e-9: continue
            mid = (p1 + p2) / 2.0
            sum_pos += mid * length
            total_len += length
            
        if total_len > 1e-9:
            center = sum_pos / total_len
        else:
            center = edge_pts[0][0]
            
        if len(edge_pts) < 3:
            return center, seed_normal
            
        # 2. Newell's Method for robust normal using all segments
        n = np.zeros(3)
        for p1, p2 in edge_pts:
            n[0] += (p1[1] - p2[1]) * (p1[2] + p2[2])
            n[1] += (p1[2] - p2[2]) * (p1[0] + p2[0])
            n[2] += (p1[0] - p2[0]) * (p1[1] + p2[1])
            
        norm_len = np.linalg.norm(n)
        if norm_len > 1e-9:
            loop_normal = n / norm_len
            if np.dot(loop_normal, seed_normal) < 0:
                loop_normal = -loop_normal
            return center, loop_normal
        
        return center, seed_normal

    def _highlight_feature_surface(self, mesh, cell_ids, link_name, matrix, color="blue", opacity=0.35):
        """Creates a semi-transparent surface highlight covering the entire face area."""
        try:
            surface_mesh = mesh.extract_cells(cell_ids)
            prefix = getattr(self, 'highlight_prefix', 'pick')
            surface_name = f"{prefix}_surface_{link_name}"
            
            # matrix must be numpy 4x4
            np_matrix = self._vtk_mat_to_numpy(matrix)
            
            self.plotter.add_mesh(surface_mesh, color=color, opacity=opacity,
                                name=surface_name, user_matrix=np_matrix, pickable=False,
                                lighting=False) # Flat color stands out better
        except Exception:
            pass

    def _highlight_feature_boundary(self, mesh, boundary_edges, link_name, matrix, color="blue"):
        """Create thicker visual highlight for feature boundary."""
        if not boundary_edges:
            return
            
        # Create line segments for boundary
        points = []
        lines = []
        point_map = {}
        
        for edge in boundary_edges:
            # Get or create points
            for pt_id in [edge[0], edge[1]]:
                if pt_id not in point_map:
                    point_map[pt_id] = len(points)
                    points.append(mesh.GetPoint(pt_id))
            
            # Add line: [num_points, idx1, idx2]
            lines.extend([2, point_map[edge[0]], point_map[edge[1]]])
        
        if points and lines:
            boundary_mesh = pv.PolyData(points)
            boundary_mesh.lines = np.array(lines)
            
            prefix = getattr(self, 'highlight_prefix', 'pick')
            highlight_name = f"{prefix}_highlight_{link_name}"
            
            np_matrix = self._vtk_mat_to_numpy(matrix)
            
            self.plotter.add_mesh(boundary_mesh, color=color, line_width=5,
                                name=highlight_name, user_matrix=np_matrix, pickable=False)

    def clear_highlights(self):
        """Removes all temporary selection markers (faces, arrows) from the scene."""
        current_actors = list(self.plotter.renderer.actors.keys())
        for actor_name in current_actors:
            if "pick_highlight_" in actor_name or "pick_arrow_" in actor_name:
                self.plotter.remove_actor(actor_name)
        self.plotter.render()

    def resizeEvent(self, event):
        super().resizeEvent(event)

    def mw_log(self, msg):
        # Helper to log back to main window if possible
        if hasattr(self.parent(), 'log'):
            self.parent().log(msg)
        elif hasattr(self.window(), 'log'):
            self.window().log(msg)

    def deselect_all(self):
        """Standard CAD behavior: Escape or blank click clears everything."""
        self.selected_name = None
        self.picking_face = False
        self.picking_focus_point = False
        self.clear_highlights()
        self._update_selection_visuals() # Clear dimension lines
        self.clear_focus_point()
        if self.on_deselect_callback:
            self.on_deselect_callback()
        
        # Reset visual highlights (Edge Colors)
        for actor in self.actors.values():
            actor.GetProperty().SetEdgeColor([0.5, 0.5, 0.5])
        self.plotter.render()
            
        self.mw_log("Selection cleared.")
        self.setCursor(QtCore.Qt.ArrowCursor)

    def start_face_picking(self, callback, color="orange"):
        """Activates specialized face picking mode."""
        self.on_face_picked_callback = callback
        self.picking_face = True
        self.picking_color = color
        self.mw_log(f"Face Picking Active: Click a face on the 3D model...")

    # ... [Existing events remain unchanged] ...

    def focus_on_bounds(self, bounds):
        """Resets camera to fit the specified bounds."""
        self.plotter.reset_camera(bounds=bounds)
        self.plotter.render()

    def view_isometric(self):
        """
        Standard CAD Isometric view: Frames all objects in the scene while 
        ignoring the background grid. If objects are far from the center, 
        it 'locates' them and brings them into focus.
        """
        # 1. Collect all model actors (exclude grids/ghosts)
        model_actors = [v for k, v in self.actors.items() if k in self.actors]
        
        if not model_actors:
            # If no model exists, just show the base grid at a sensible zoom
            self.plotter.view_isometric()
            self.plotter.reset_camera()
        else:
            # 2. Calculate the total bounding box of all components
            xmin, xmax, ymin, ymax, zmin, zmax = 1e9, -1e9, 1e9, -1e9, 1e9, -1e9
            for actor in model_actors:
                b = actor.GetBounds()
                xmin, xmax = min(xmin, b[0]), max(xmax, b[1])
                ymin, ymax = min(ymin, b[2]), max(ymax, b[3])
                zmin, zmax = min(zmin, b[4]), max(zmax, b[5])
            
            # 3. Apply standard Isometric camera direction (1,1,1)
            self.plotter.camera_position = 'iso'
            
            # 4. Snap and Frame the objects
            self.plotter.reset_camera(bounds=(xmin, xmax, ymin, ymax, zmin, zmax))
            
        self.plotter.render()
        self._on_camera_change() 

    def focus_on_actor(self, name):
        """Snaps to isometric view and frames ONLY the specified actor."""
        if name not in self.actors:
            return
            
        actor = self.actors[name]
        bounds = actor.GetBounds()
        
        # Set Isometric view
        self.plotter.camera_position = 'iso'
        # Frame only this specific actor
        self.plotter.reset_camera(bounds=bounds)
        self.plotter.render()
        self._on_camera_change()

    def view_top(self):
        """Snaps camera to top view."""
        self.plotter.view_xy()
        self.plotter.render()
        self._on_camera_change() # Update grids

    def start_object_picking(self, callback, label="Object"):
        """Activates silent object picking - returns name without highlighting"""
        self.on_object_picked_callback = callback
        self.picking_object = True
        self.picking_label = label
        self.mw_log(f"Click a 3D object to select it as {label}...")
        self.setCursor(QtCore.Qt.PointingHandCursor)

    def start_object_picking_double(self, callback, label="Object"):
        """Activates double-click object picking - returns name without highlighting"""
        self.on_object_picked_callback = callback
        self.picking_object_double = True
        self.picking_label = label
        self.mw_log(f"Double-click a 3D object to select it as {label}...")
        self.setCursor(QtCore.Qt.PointingHandCursor)
        
        # Add double-click observer if not already added
        if not hasattr(self, '_double_click_observer_added'):
            self.plotter.interactor.AddObserver("LeftButtonDoubleClickEvent", self._on_double_click)
            self._double_click_observer_added = True

    def _on_double_click(self, obj, event):
        """Handle double-click event for object selection"""
        if not hasattr(self, 'picking_object_double') or not self.picking_object_double:
            return
        
        self.mw_log("Double-click detected!")  # Debug
        
        click_pos = self.plotter.interactor.GetEventPosition()
        self.picker.Pick(click_pos[0], click_pos[1], 0, self.plotter.renderer)
        actor = self.picker.GetActor()
        
        if actor:
            # Find the name of the clicked actor
            for name, a in self.actors.items():
                if a == actor:
                    if self.on_object_picked_callback:
                        self.on_object_picked_callback(name)
                    self.mw_log(f"{self.picking_label} selected: {name}")
                    break
        else:
            self.mw_log("No object under cursor")
        
        # Reset state
        self.picking_object_double = False
        self.on_object_picked_callback = None
        self.setCursor(QtCore.Qt.ArrowCursor)

    def cancel_object_picking(self):
        """Cancel active object picking mode without callback"""
        self.picking_object = False
        self.picking_object_double = False
        self.on_object_picked_callback = None
        self.setCursor(QtCore.Qt.ArrowCursor)
        self.mw_log("Selection cancelled.")

    def start_point_picking(self, callback):
        """Activates point picking mode for Joint Origin selection."""
        self.on_point_picked_callback = callback
        self.picking_point = True
        self.mw_log("Pick a point in 3D space for the Joint Pivot...")
        self.setCursor(QtCore.Qt.CrossCursor)

    def start_focus_point_picking(self):
        """Activates focus point picking mode - click any surface to place a marker and zoom in."""
        self.picking_focus_point = True
        self.mw_log("Click anywhere on the 3D scene to set a focus point...")
        self.setCursor(QtCore.Qt.CrossCursor)

    def focus_on_point(self, point):
        """Place a small sphere marker at the 3D point and focus the camera on it."""
        point = np.array(point)
        
        # Remove previous focus marker if exists
        self.clear_focus_point()
        
        # Calculate adaptive sphere radius based on scene scale
        radius = 0.005  # default tiny
        if self.actors:
            xmin, xmax, ymin, ymax, zmin, zmax = 1e9, -1e9, 1e9, -1e9, 1e9, -1e9
            for actor in self.actors.values():
                b = actor.GetBounds()
                xmin, xmax = min(xmin, b[0]), max(xmax, b[1])
                ymin, ymax = min(ymin, b[2]), max(ymax, b[3])
                zmin, zmax = min(zmin, b[4]), max(zmax, b[5])
            scene_diag = np.sqrt((xmax-xmin)**2 + (ymax-ymin)**2 + (zmax-zmin)**2)
            radius = scene_diag * 0.02  # 2% of scene diagonal
        
        # Create a sphere at the focus point
        sphere = pv.Sphere(radius=radius, center=point)
        self.plotter.add_mesh(sphere, color="#1976d2", opacity=0.85,
                             name="focus_point_sphere", pickable=False)
        
        # Focus camera: set focal point and zoom in
        cam = self.plotter.camera
        view_up = np.array(cam.GetViewUp())
        cam_pos = np.array(cam.position)
        cam_dir = cam_pos - point
        cam_dist = np.linalg.norm(cam_dir)
        cam_dir_norm = cam_dir / (cam_dist + 1e-9)
        
        zoom_region = radius * 60
        new_dist = max(zoom_region, cam_dist * 0.15)
        
        cam.focal_point = point
        cam.position = point + cam_dir_norm * new_dist
        cam.up = view_up
        
        self.plotter.render()
        self._on_camera_change()
        self.mw_log(f"Focus point set at ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")

    def clear_focus_point(self):
        """Remove the focus point marker from the scene."""
        try:
            self.plotter.remove_actor("focus_point_sphere")
        except:
            pass

    def _on_left_down(self, obj, event):
        click_pos = self.plotter.interactor.GetEventPosition()
        
        # --- DOUBLE-CLICK DETECTION (CUSTOM) ---
        if hasattr(self, 'picking_object_double') and self.picking_object_double:
            import time
            current_time = time.time()
            
            # Pick the actor
            self.picker.Pick(click_pos[0], click_pos[1], 0, self.plotter.renderer)
            actor = self.picker.GetActor()
            
            # Check if this is a double-click (within 300ms of last click on same actor)
            if hasattr(self, '_last_click_time') and hasattr(self, '_last_click_actor'):
                time_diff = current_time - self._last_click_time
                if time_diff < 0.3 and actor == self._last_click_actor and actor is not None:
                    # DOUBLE-CLICK DETECTED!
                    self.mw_log("Double-click detected!")
                    
                    # Find the name of the clicked actor
                    for name, a in self.actors.items():
                        if a == actor:
                            if self.on_object_picked_callback:
                                self.on_object_picked_callback(name)
                            self.mw_log(f"{self.picking_label} selected: {name}")
                            break
                    
                    # Reset state
                    self.picking_object_double = False
                    self.on_object_picked_callback = None
                    self.setCursor(QtCore.Qt.ArrowCursor)
                    self._last_click_time = None
                    self._last_click_actor = None
                    return
            
            # Store this click for double-click detection
            self._last_click_time = current_time
            self._last_click_actor = actor
            return
        
        # --- OBJECT PICKING MODE (JOINT PARENT/CHILD) ---
        if hasattr(self, 'picking_object') and self.picking_object:
            self.picker.Pick(click_pos[0], click_pos[1], 0, self.plotter.renderer)
            actor = self.picker.GetActor()
            
            if actor:
                # Find the name of the clicked actor
                for name, a in self.actors.items():
                    if a == actor:
                        if self.on_object_picked_callback:
                            self.on_object_picked_callback(name)
                        self.mw_log(f"{self.picking_label} selected: {name}")
                        break
            
            # Reset state
            self.picking_object = False
            self.on_object_picked_callback = None
            self.setCursor(QtCore.Qt.ArrowCursor)
            return
        
        # --- POINT PICKING MODE (JOINT ORIGIN) ---
        if hasattr(self, 'picking_point') and self.picking_point:
            self.picker.Pick(click_pos[0], click_pos[1], 0, self.plotter.renderer)
            picked_pos = self.picker.GetPickPosition()
            
            # Use picked position (even if on grid/empty space, though usually on object)
            if self.on_point_picked_callback:
                self.on_point_picked_callback(picked_pos)
            
            # Reset state
            self.picking_point = False
            self.on_point_picked_callback = None
            self.setCursor(QtCore.Qt.ArrowCursor)
            self.mw_log(f"Point picked: {np.round(picked_pos, 2)}")
            return # Block other interactions

        # --- FOCUS POINT PICKING MODE ---
        if self.picking_focus_point:
            self.cell_picker.Pick(click_pos[0], click_pos[1], 0, self.plotter.renderer)
            picked_actor = self.cell_picker.GetActor()
            
            if picked_actor:
                picked_pos = self.cell_picker.GetPickPosition()
                self.focus_on_point(picked_pos)
            else:
                self.mw_log("No surface under cursor - click on a model or grid.")
            
            self.picking_focus_point = False
            self.setCursor(QtCore.Qt.ArrowCursor)
            return

        # CASE 0: FACE PICKING IN PROGRESS
        if self.picking_face:
            if self._on_face_pick_click(click_pos):
                return # Successfully picked face, block everything else
            else:
                self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown()
                return
        
        # Check if we hit an actor
        self.picker.Pick(click_pos[0], click_pos[1], 0, self.plotter.renderer)
        actor = self.picker.GetActor()

        # CASE 1: We are already dragging something
        if self.is_dragging:
            # A left click while dragging confirms placement (drops it)
            if self.selected_name and self.on_drop_callback:
                mat = self.actors[self.selected_name].user_matrix
                self.on_drop_callback(self.selected_name, mat)

            self.is_dragging = False
            self.selected_name = None
            for a in self.actors.values():
                a.GetProperty().SetEdgeColor([0.5, 0.5, 0.5]) # Reset highlights
            
            # Allow camera interaction immediately after drop
            self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown()
            return

        # CASE 2: We clicked on an object (START DRAGGING)
        if actor:
            # CHECK: Interaction Mode (Disable drag in Align/Joint tabs)
            if not self.enable_drag:
                # Still allow selection for UI sync, but don't start dragging
                for name, a in self.actors.items():
                    if a == actor:
                        self.select_actor(name)
                        break
                self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown()
                return
            
            # Identify the clicked link
            clicked_name = None
            for name, a in self.actors.items():
                if a == actor:
                    clicked_name = name
                    break
            
            # --- SIMULATION MODE DRAG CONSTRAINT ---
            # In simulation mode, we only allow moving "simulation objects"
            # (objects imported while in simulation mode).
            if hasattr(self.window(), 'sim_toggle_btn') and self.window().sim_toggle_btn.isChecked():
                if clicked_name is None:
                    self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown()
                    return
                    
                link = None
                if clicked_name in self.window().robot.links:
                    link = self.window().robot.links[clicked_name]
                
                if not link or not getattr(link, 'is_sim_obj', False):
                    self.mw_log(f"⚠ Locked: '{clicked_name}' belongs to the Robot. Only imported Simulation Objects can be moved in this mode.")
                    self.select_actor(clicked_name)
                    self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown()
                    return
            
            # --- ENGINEERING CONSTRAINT: LOCKED/ALIGNED COMPONENTS ---
            # If a link has a parent joint (i.e., it is aligned/attached to something),
            # it should NOT be moveable by the free-drag tool. It is "Constrained".
            # Also blocks components that have been through alignment (exist in alignment_cache).
            if hasattr(self.window(), 'robot') and clicked_name:
                robot = self.window().robot
                if clicked_name in robot.links:
                    link = robot.links[clicked_name]
                    
                    # Check 1: Jointed (has a parent joint)
                    if link.parent_joint:
                        self.mw_log(f"\u26a0 Locked: '{clicked_name}' is jointed. Remove the joint first to move freely.")
                        self.select_actor(clicked_name) # Select it visually
                        self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown() # Allow camera rotate
                        return
                    
                    # Check 2: Aligned (exists in alignment_cache as child)
                    if hasattr(self.window(), 'alignment_cache'):
                        for (parent, child), pt in self.window().alignment_cache.items():
                            if child == clicked_name:
                                self.mw_log(f"\u26a0 Locked: '{clicked_name}' is aligned to '{parent}'. Undo alignment first to move freely.")
                                self.select_actor(clicked_name)
                                self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown()
                                return
            
            # --- ALIGNMENT MODE LOCK ---
            # If a component is currently being used for alignment (face picked), 
            # it should NOT be moved, otherwise the world-coords of the face become invalid.
            if hasattr(self.window(), 'align_tab') and clicked_name:
                at = self.window().align_tab
                staged = []
                if hasattr(at, 'parent_pick_data') and at.parent_pick_data: 
                    staged.append(at.parent_pick_data['name'])
                if hasattr(at, 'child_pick_data') and at.child_pick_data: 
                    staged.append(at.child_pick_data['name'])
                
                if clicked_name in staged:
                    self.mw_log(f"\u26a0 Locked: '{clicked_name}' has an active alignment selection. Reset or Save to move.")
                    self.select_actor(clicked_name)
                    self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown()
                    return

            # CHECK: Is this the Base Link? (Bases are fixed/non-pickable for dragging)
            if clicked_name in self.fixed_actors:
                self.mw_log(f"⚠️ Locked: '{clicked_name}' is the Base and its position is frozen.")
                # Just let the camera rotate, don't select or drag
                self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown()
                return
            
            # If we clicked the base, IGNORE it (don't start dragging)
            # We assume the user has access to MainWindow.robot.base_link
            # But here we can just check if it's the first imported object or marked as base
            # For robustness, we will let MainWindow handle the 'Base' logic 
            # and just check a 'fixed' property on actors if we want, 
            # but for now, we'll implement the 'not selectable' logic in MainWindow.
            
            self.is_dragging = True
            self.last_pos = click_pos
            
            # Find and select the clicked link
            found = False
            for name, a in self.actors.items():
                if a == actor:
                    self.selected_name = name
                    a.GetProperty().SetEdgeColor([1, 1, 0]) # Yellow focus
                    found = True
                else:
                    a.GetProperty().SetEdgeColor([0.5, 0.5, 0.5]) # Gray out others
            
            if found:
                return # Stop event here (don't rotate camera)

        # CASE 3: We clicked on empty space or we are starting a camera interaction
        # AUTO-PIVOT: If we have a hover point, set it as the center of rotation now.
        if hasattr(self, '_current_hover_pt') and self._current_hover_pt is not None:
            P = self._current_hover_pt
            old_focal = np.array(self.plotter.camera.focal_point)
            offset = P - old_focal
            # Update position and focal point to pan to the new pivot silently
            self.plotter.camera.position = np.array(self.plotter.camera.position) + offset
            self.plotter.camera.focal_point = P

        # Just rotate the camera, do NOT select anything
        self.plotter.interactor.GetInteractorStyle().OnLeftButtonDown()

    def _on_left_up(self, obj, event):
        # Pass through to default interactor to finish camera rotation
        self.plotter.interactor.GetInteractorStyle().OnLeftButtonUp()
        
    def _on_right_down(self, obj, event):
        # Right click to CANCEL / DROP selection
        if self.is_dragging:
            # SYNC: Save final position before dropping
            if self.selected_name and self.on_drop_callback:
                mat = self.actors[self.selected_name].user_matrix
                self.on_drop_callback(self.selected_name, mat)

            self.is_dragging = False
            self.selected_name = None
            # Reset colors
            for a in self.actors.values():
                a.GetProperty().SetEdgeColor([0.5, 0.5, 0.5])
            return # Consume event
            
        # Otherwise, let default right-click behavior happen (usually zoom)
        self.plotter.interactor.GetInteractorStyle().OnRightButtonDown()

    def _on_mouse_move(self, obj, event):
        if self.is_dragging and self.selected_name:
            curr_pos = self.plotter.interactor.GetEventPosition()
            if self.last_pos:
                # Calculate mouse delta in pixels
                dx = curr_pos[0] - self.last_pos[0]
                dy = curr_pos[1] - self.last_pos[1]
                
                actor = self.actors[self.selected_name]
                camera = self.plotter.camera
                
                # Get exact camera and window properties
                window_size = self.plotter.render_window.GetSize()
                view_angle = np.radians(camera.GetViewAngle())
                
                # Distance from camera to object
                dist = np.linalg.norm(np.array(camera.GetPosition()) - np.array(actor.GetCenter()))
                
                # Pixel-to-World scaling: 
                # At distance 'dist', the screen height in world units is 2 * dist * tan(FOV/2)
                world_height = 2.0 * dist * np.tan(view_angle / 2.0)
                scale = world_height / window_size[1] # Scale based on vertical pixels
                
                # View-plane basis vectors
                view_up = np.array(camera.GetViewUp())
                view_up /= (np.linalg.norm(view_up) + 1e-6)
                view_dir = np.array(camera.GetDirectionOfProjection())
                side = np.cross(view_dir, view_up)
                side /= (np.linalg.norm(side) + 1e-6)
                
                # Translate the part
                move_vector = (side * dx + view_up * dy) * scale
                
                mat = actor.user_matrix
                mat[0, 3] += move_vector[0]
                mat[1, 3] += move_vector[1]
                mat[2, 3] += move_vector[2]
                actor.user_matrix = mat
                
                self.last_pos = curr_pos
                self.plotter.render()
            return 
            
        # --- DYNAMIC CAMERA TRACKING (POTENTIAL PIVOT) ---
        # We track the point under the cursor so we can zoom/rotate around it live.
        if not self.is_dragging and not self.picking_face and not getattr(self, 'picking_point', False):
            curr_pos = self.plotter.interactor.GetEventPosition()
            self.cell_picker.Pick(curr_pos[0], curr_pos[1], 0, self.plotter.renderer)
            picked_actor = self.cell_picker.GetActor()
            
            if picked_actor and (picked_actor in self.actors.values() or picked_actor in self.grids.values()):
                # We store the point but we DON'T move the camera yet 
                # to avoid jumpy "sliding" views while simply hovering.
                self._current_hover_pt = np.array(self.cell_picker.GetPickPosition())
            else:
                self._current_hover_pt = None
                
        self.plotter.interactor.GetInteractorStyle().OnMouseMove()

    def _on_wheel_forward(self, obj, event):
        self._zoom_at_cursor(1.2)
        try:
            obj.SetAbortFlag(1) # Block default zoom
        except:
            pass

    def _on_wheel_backward(self, obj, event):
        self._zoom_at_cursor(1.0/1.2)
        try:
            obj.SetAbortFlag(1) # Block default zoom
        except:
            pass

    def _zoom_at_cursor(self, amount):
        """
        Dolly zoom towards the current mouse position. 
        Calculates new camera position and focal point such that the 
        point under the cursor remains at the same pixel location.
        """
        curr_pos = self.plotter.interactor.GetEventPosition()
        self.cell_picker.Pick(curr_pos[0], curr_pos[1], 0, self.plotter.renderer)
        picked_actor = self.cell_picker.GetActor()
        
        # Determine the pivot point for the zoom
        # Priority: Exact actor under cursor > Exact grid under cursor > Last known hover pt > Focal point
        if picked_actor and (picked_actor in self.actors.values() or picked_actor in self.grids.values()):
            P = np.array(self.cell_picker.GetPickPosition())
        elif hasattr(self, '_current_hover_pt') and self._current_hover_pt is not None:
            P = self._current_hover_pt
        else:
            P = np.array(self.plotter.camera.focal_point)

        C = np.array(self.plotter.camera.position)
        F = np.array(self.plotter.camera.focal_point)
        
        # Zoom Factor (Scale around P)
        # s > 1 means zooming in (C and F move towards P)
        scale = 1.0 / amount
        
        new_C = P + (C - P) * scale
        new_F = P + (F - P) * scale
        
        # Apply transformation
        self.plotter.camera.position = new_C
        self.plotter.camera.focal_point = new_F
        self._update_axis_labels()
        self.plotter.render()
    def update_link_mesh(self, link_name, mesh, transform, color="silver"):
        """Adds or updates a link mesh in the scene."""
        if link_name in self.actors:
            self.plotter.remove_actor(self.actors[link_name])
        
        # Convert trimesh to pyvista if needed
        import trimesh
        if isinstance(mesh, trimesh.Trimesh):
            # Clean mesh (merge duplicate points) - CRITICAL for geometric feature detection
            poly = pv.wrap(mesh).clean()
        else:
            # Wrap and clean in case it's a generic VTK mesh or needs welding
            poly = pv.wrap(mesh).clean()
            
        # Use provided color, hide edges for cleaner look
        actor = self.plotter.add_mesh(poly, color=color, show_edges=False, name=link_name)
        # Apply transform
        actor.user_matrix = transform
        self.actors[link_name] = actor
        self.plotter.render()

    def set_actor_color(self, name, hex_color):
        """Changes the color of an existing actor."""
        if name in self.actors:
            self.actors[name].GetProperty().SetColor(QtGui.QColor(hex_color).getRgbF()[:3])
            self.plotter.render()

    def select_actor(self, name):
        """Programmatically select and highlight an actor by name."""
        if name not in self.actors:
            return
        
        self.selected_name = name
        # Highlight
        for n, actor in self.actors.items():
            if n == name:
                actor.GetProperty().SetEdgeColor([1, 1, 0]) # Yellow
            else:
                actor.GetProperty().SetEdgeColor([0.5, 0.5, 0.5]) # Gray
        self.plotter.render()
        self.plotter.render()

    def remove_actor(self, name):
        """Removes an actor from the scene by name."""
        if name in self.actors:
            if self.selected_name == name:
                self.deselect_all()
            self.plotter.remove_actor(self.actors[name])
            del self.actors[name]
            self.plotter.render()

    def _update_selection_visuals(self):
        """Draws dimension lines and labels around the selected object."""
        # 1. Clean up old actors
        if not hasattr(self, '_selection_dim_actors'):
             self._selection_dim_actors = []
             
        for actor in self._selection_dim_actors:
            self.plotter.renderer.RemoveActor(actor)
        self._selection_dim_actors = []

        if not self.selected_name or self.selected_name not in self.actors:
            return

        actor = self.actors[self.selected_name]
        try:
            # actor.GetBounds() returns (xmin, xmax, ymin, ymax, zmin, zmax)
            b = actor.GetBounds()
            
            # Use small offset to prevent Z-fighting with the mesh
            pad = (b[1]-b[0]) * 0.05
            if pad == 0: pad = 0.5
            
            ratio = self.grid_units_per_cm
            
            # Draw 3 representative dimension lines: X, Y, Z
            # X dimension line (bottom edge)
            self._create_dim_line(
                (b[0], b[2]-pad, b[4]), (b[1], b[2]-pad, b[4]), 
                f"X: {(b[1]-b[0])/ratio:.1f} cm", "#1976d2"
            )
            
            # Y dimension line
            self._create_dim_line(
                (b[1]+pad, b[2], b[4]), (b[1]+pad, b[3], b[4]), 
                f"Y: {(b[3]-b[2])/ratio:.1f} cm", "#388E3C"
            )
            
            # Z dimension line
            self._create_dim_line(
                (b[0]-pad, b[2]-pad, b[4]), (b[0]-pad, b[2]-pad, b[5]), 
                f"Z: {(b[5]-b[4])/ratio:.1f} cm", "#D32F2F"
            )

        except Exception as e:
            print(f"Error drawing dimensions: {e}")

    def _create_dim_line(self, start, end, label, color):
        """Helper to create a 3D line with a centered billboard label."""
        import vtkmodules.vtkRenderingCore as vtkRC
        
        # 1. The Line
        try:
            line_mesh = pv.Line(start, end)
            actor = self.plotter.add_mesh(
                line_mesh, color=color, line_width=2, 
                pickable=False, lighting=False, name=f"_dim_line_{label}"
            )
            self._selection_dim_actors.append(actor)
            
            # 2. The Label (Billboard)
            mid = [(start[i] + end[i])/2.0 for i in range(3)]
            txt_actor = vtkRC.vtkBillboardTextActor3D()
            txt_actor.SetInput(label)
            txt_actor.SetPosition(mid[0], mid[1], mid[2])
            txt_actor.GetTextProperty().SetFontSize(12)
            txt_actor.GetTextProperty().SetColor(pv.Color(color))
            txt_actor.GetTextProperty().SetBold(True)
            txt_actor.GetTextProperty().SetFontFamilyToArial()
            txt_actor.GetTextProperty().SetJustificationToCentered()
            txt_actor.SetPickable(False)
            
            self.plotter.renderer.AddActor(txt_actor)
            self._selection_dim_actors.append(txt_actor)
        except Exception:
            pass

    def update_hud_coords(self, x, y, z):
        """Updates the Live Point HUD text on the 3D screen."""
        text = f"LIVE POINT: X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f} cm"
        self.plotter.add_text(
            text, 
            position='upper_left', 
            font_size=12, 
            color='#1565c0', 
            name="live_point_hud",
            shadow=True
        )
        self.plotter.render()

    def update_transforms(self, robot):
        """Updates all actor transforms based on robot's current kinematics state."""
        for name, link in robot.links.items():
            if name in self.actors:
                self.actors[name].user_matrix = link.t_world
        self.plotter.render()


    def _init_custom_grids(self):
        """Creates the 3 principal plane grids for the '3D Graph' system."""
        # Clean up existing grids if any
        if hasattr(self, 'grids'):
            for actor in self.grids.values():
                self.plotter.remove_actor(actor)
        
        self.grids = {}
        # Dynamic size: grid_cm_size * units_per_cm
        size = self.grid_cm_size * self.grid_units_per_cm
        # Performance Cap: Avoid GPU freeze on massive scales
        res = min(int(self.grid_cm_size), 500) 
        
        # 1. XY Grid (Bottom/Top) - Blueish tint
        xy_mesh = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), i_size=size, j_size=size, i_resolution=res, j_resolution=res)
        self.grids['xy'] = self.plotter.add_mesh(xy_mesh, color="#e3f2fd", opacity=0.3, 
                                               show_edges=True, edge_color="#1565c0", line_width=1,
                                               name="grid_xy", pickable=True, lighting=False)
        
        # 2. XZ Grid (Front/Back) - Greenish tint
        xz_mesh = pv.Plane(center=(0, 0, 0), direction=(0, 1, 0), i_size=size, j_size=size, i_resolution=res, j_resolution=res)
        self.grids['xz'] = self.plotter.add_mesh(xz_mesh, color="#e8f5e9", opacity=0.3, 
                                               show_edges=True, edge_color="#2e7d32", line_width=1,
                                               name="grid_xz", pickable=True, lighting=False)
        
        # 3. YZ Grid (Left/Right) - Reddish tint
        yz_mesh = pv.Plane(center=(0, 0, 0), direction=(1, 0, 0), i_size=size, j_size=size, i_resolution=res, j_resolution=res)
        self.grids['yz'] = self.plotter.add_mesh(yz_mesh, color="#ffebee", opacity=0.3, 
                                               show_edges=True, edge_color="#c62828", line_width=1,
                                               name="grid_yz", pickable=True, lighting=False)
        
        # Initially hide all except XY
        for name, actor in self.grids.items():
            actor.SetVisibility(bool(name == 'xy'))

    def _init_axis_labels(self):
        """Creates distance labels along the center axis lines (through origin)."""
        import vtkmodules.vtkRenderingCore as vtkRC
        import vtkmodules.vtkRenderingFreeType as vtkFT  # noqa - needed for text rendering
        
        self._axis_labels = []  # List of {'actor', 'val', 'grid', 'axis'}
        self._center_axis_actors = {}  # Grid key -> list of line actors
        
        # Dynamic half-size and offset
        half = (self.grid_cm_size / 2.0) * self.grid_units_per_cm
        offset = self.grid_units_per_cm * 0.5  # Label offset

        # Calculate dynamic ticks relative to workspace size
        if self.grid_cm_size > 10000:
            cm_interval = self.grid_cm_size / 20.0  # Target ~20 labels per axis for massive scales
        elif self.grid_cm_size > 100:
            cm_interval = 10.0  # Show label every 10cm for standard workspaces
        else:
            cm_interval = 1.0   # 1cm for precision views
            
        unit_interval = cm_interval * self.grid_units_per_cm
        ticks = [i for i in np.arange(-half, half + 1.0, unit_interval) if abs(i) > 1e-6]
        
        # Dark color for labels (readable on light grid)
        label_color = (0.25, 0.25, 0.25)  # Dark gray
        
        # ── CENTER AXIS LINES (bold dark lines through origin) ──
        axis_line_defs = {
            # grid -> list of (start_point, end_point)
            'xy': [
                ((-half, 0, 0), (half, 0, 0)),  # X-axis line
                ((0, -half, 0), (0, half, 0)),   # Y-axis line
            ],
            'xz': [
                ((-half, 0, 0), (half, 0, 0)),  # X-axis line
                ((0, 0, -half), (0, 0, half)),   # Z-axis line
            ],
            'yz': [
                ((0, -half, 0), (0, half, 0)),  # Y-axis line
                ((0, 0, -half), (0, 0, half)),   # Z-axis line
            ],
        }
        
        for grid_key, lines in axis_line_defs.items():
            actors = []
            for start, end in lines:
                line_mesh = pv.Line(start, end, resolution=1)
                actor = self.plotter.add_mesh(
                    line_mesh, color="#333333", line_width=2,
                    name=f"_centerline_{grid_key}_{start}", pickable=False, lighting=False
                )
                actors.append(actor)
            self._center_axis_actors[grid_key] = actors
        
        # Initially show only XY center lines
        for gk, actors in self._center_axis_actors.items():
            for a in actors:
                a.SetVisibility(bool(gk == 'xy'))
        
        # ── DISTANCE LABELS along center axes ──
        # Labels placed ON the axis lines with a small perpendicular offset
        # XY grid: X-labels along y=0 line (offset in -Y), Y-labels along x=0 line (offset in -X)
        # XZ grid: X-labels along z=0 line (offset in -Z), Z-labels along x=0 line (offset in -X)
        # YZ grid: Y-labels along z=0 line (offset in -Z), Z-labels along y=0 line (offset in -Y)
        label_defs = [
            # (grid, axis, position_fn) — offset perpendicular to the axis so text doesn't overlap the line
            ('xy', 'x', lambda t: (t, -offset, 0)),       # Numbers along X-axis, nudged below
            ('xy', 'y', lambda t: (-offset, t, 0)),        # Numbers along Y-axis, nudged left
            ('xz', 'x', lambda t: (t, 0, -offset)),       # Numbers along X-axis
            ('xz', 'z', lambda t: (-offset, 0, t)),        # Numbers along Z-axis
            ('yz', 'y', lambda t: (0, t, -offset)),        # Numbers along Y-axis
            ('yz', 'z', lambda t: (0, -offset, t)),        # Numbers along Z-axis
        ]
        
        for grid_key, axis_name, pos_fn in label_defs:
            for tick_val in ticks:
                pos = pos_fn(tick_val)
                # Calculate the CM value for this internal unit coordinate
                cm_val = tick_val / self.grid_units_per_cm
                txt = f"{cm_val:.0f} cm"
                
                # Billboard text actor — always faces camera
                txt_actor = vtkRC.vtkBillboardTextActor3D()
                txt_actor.SetInput(txt)
                txt_actor.SetPosition(pos[0], pos[1], pos[2])
                txt_actor.GetTextProperty().SetFontSize(14)
                txt_actor.GetTextProperty().SetColor(label_color)
                txt_actor.GetTextProperty().SetJustificationToCentered()
                txt_actor.GetTextProperty().SetBold(True)
                txt_actor.GetTextProperty().SetFontFamilyToArial()
                txt_actor.SetPickable(False)
                
                self.plotter.renderer.AddActor(txt_actor)
                
                self._axis_labels.append({
                    'actor': txt_actor,
                    'val': tick_val,
                    'grid': grid_key,
                    'axis': axis_name,
                })
        
        # Initially show only XY labels
        for lbl in self._axis_labels:
            lbl['actor'].SetVisibility(bool(lbl['grid'] == 'xy'))

    def update_grid_scale(self, units_per_cm):
        """Updates the graph labeling scale to match robot units (e.g., 10.0 for mm)."""
        self.grid_units_per_cm = float(units_per_cm)
        self._refresh_grid_visuals()

    def ensure_grid_fits_bounds(self, bounds):
        """Checks if the component bounds exceed the current grid and expands it if necessary."""
        # bounds = (xmin, xmax, ymin, ymax, zmin, zmax)
        raw_max = max(abs(bounds[0]), abs(bounds[1]), abs(bounds[2]), abs(bounds[3]), abs(bounds[4]), abs(bounds[5]))
        cm_needed = (raw_max / self.grid_units_per_cm) * 2.2 # Add padding
        
        if cm_needed > self.grid_cm_size:
            self.grid_cm_size = cm_needed
            self._refresh_grid_visuals()

    def _refresh_grid_visuals(self):
        """Internal helper to redraw grids and labels when size or ratio changes."""
        # Cleanup old labels and center lines
        if hasattr(self, '_axis_labels'):
            for lbl in self._axis_labels:
                self.plotter.renderer.RemoveActor(lbl['actor'])
        if hasattr(self, '_center_axis_actors'):
            for actors in self._center_axis_actors.values():
                for a in actors:
                    self.plotter.remove_actor(a)

        # Re-initialize everything with new scale/size
        self._init_custom_grids() 
        self._init_axis_labels()  
        self.plotter.render()

    def _update_axis_labels(self):
        """Update axis label & center line visibility based on camera view and zoom."""
        if not hasattr(self, '_axis_labels'):
            return
        
        # Camera distance in CM for zoom logic
        cam_pos = np.array(self.plotter.camera.position)
        focal = np.array(self.plotter.camera.focal_point)
        cam_dist = np.linalg.norm(cam_pos - focal)
        cam_dist_cm = cam_dist / self.grid_units_per_cm
        
        # Dynamic label granularity based on zoom (Requested: 10cm close, 50cm far)
        if cam_dist_cm < 150:     # Within 1.5 meters: Show every 10cm
            target_step_cm = 10
        elif cam_dist_cm < 500:   # Within 5 meters: Show every 50cm
            target_step_cm = 50
        elif cam_dist_cm < 1500:  # Within 15 meters: Show every 100cm (1m)
            target_step_cm = 100
        else:                     # Massive view: Show every 500cm (5m)
            target_step_cm = 500
        
        # Which grids are visible (snapped view detection)
        direction = focal - cam_pos
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            return
        direction /= norm
        abs_dir = np.abs(direction)
        tol = 0.95
        
        show_xy = abs_dir[2] > tol
        show_xz = abs_dir[1] > tol
        show_yz = abs_dir[0] > tol
        
        grid_vis = {
            'xy': True,  # XY grid is always visible (default ground plane)
            'xz': show_xz,
            'yz': show_yz,
        }
        
        # Update center axis lines visibility
        if hasattr(self, '_center_axis_actors'):
            for gk, actors in self._center_axis_actors.items():
                vis = bool(grid_vis.get(gk, False))
                for a in actors:
                    a.SetVisibility(vis)
        
        # Update label visibility
        for lbl in self._axis_labels:
            # Calculate actual CM value for this label
            cm_val = round(abs(lbl['val'] / self.grid_units_per_cm))
            
            gv = bool(grid_vis.get(lbl['grid'], False))
            # Visibility condition: matches target step and grid is visible
            tv = (cm_val % target_step_cm) == 0
            lbl['actor'].SetVisibility(bool(gv and tv))

    def _on_camera_change(self, *args):
        """Dynamically toggles grid visibility based on camera orientation."""
        if not hasattr(self, 'grids'):
            return
        # Get normalized direction vector from camera to focal point
        pos = np.array(self.plotter.camera.position)
        focal = np.array(self.plotter.camera.focal_point)
        direction = focal - pos
        direction /= np.linalg.norm(direction)
        
        # Absolute components to detect alignment with axes
        abs_dir = np.abs(direction)
        
        # Threshold for 'snapped' or 'near-snapped' view
        tol = 0.95
        
        # Determine which grid to show
        show_xy = abs_dir[2] > tol # Looking mostly along Z (Top/Bottom)
        show_xz = abs_dir[1] > tol # Looking mostly along Y (Front/Back)
        show_yz = abs_dir[0] > tol # Looking mostly along X (Left/Right)
        
        # Special case: If we are in free-rotation (isometric-ish), default to XY or hide?
        # User said "if i am seeng topview show me side plans grid only ... applied for all sides"
        # This implies when NOT in side view, maybe we show nothing or just a base grid.
        # Let's show XY as a default ground plane if not snapped to a side.
        
        is_snapped = show_xy or show_xz or show_yz
        
        if not is_snapped:
            # Optionally show a faint XY grid in 3D view
            self.grids['xy'].SetVisibility(True)
            self.grids['xy'].GetProperty().SetOpacity(0.15)
            self.grids['xz'].SetVisibility(False)
            self.grids['yz'].SetVisibility(False)
        else:
            self.grids['xy'].SetVisibility(bool(show_xy))
            self.grids['xy'].GetProperty().SetOpacity(0.5 if show_xy else 0.3)
            
            self.grids['xz'].SetVisibility(bool(show_xz))
            self.grids['xz'].GetProperty().SetOpacity(0.5 if show_xz else 0.3)
            
            self.grids['yz'].SetVisibility(bool(show_yz))
            self.grids['yz'].GetProperty().SetOpacity(0.5 if show_yz else 0.3)
        
        # Update axis labels based on zoom and grid visibility
        self._update_axis_labels()
    def _init_ghost_system(self):
        """Initialize ghost trail tracking (called lazily on first use)."""
        if not hasattr(self, '_ghost_data'):
            self._ghost_data = {}  # name -> {'actor': actor, 'time': start_time, 'link': link_name}
            self._ghost_counter = 0
            self._fade_timer = QtCore.QTimer(self)
            self._fade_timer.timeout.connect(self._process_ghost_fading)
            self._fade_timer.start(500) # Update every 500ms

    def add_joint_ghost(self, link_name, mesh, transform, color="#888888", opacity=0.1):
        """
        Adds one semi-transparent ghost snapshot of a link at its current
        transform. Resets the 10-second auto-clear timer on every call.

        Parameters:
            mesh      : trimesh or pyvista mesh of the link
            transform : 4x4 world transform numpy array
            color     : hex or named color — ideally the link's own color
            opacity   : transparency level (0=invisible, 1=solid)
        """
        self._init_ghost_system()

        try:
            import pyvista as _pv
            import trimesh as _trimesh
            import time as _time

            # Convert if trimesh
            if isinstance(mesh, _trimesh.Trimesh):
                poly = _pv.wrap(mesh)
            else:
                poly = mesh

            # Cap ghost count at 5000 for maximum persistence
            if len(self._ghost_data) >= 5000:
                oldest_key = next(iter(self._ghost_data))
                try:
                    self.plotter.remove_actor(self._ghost_data[oldest_key]['actor'])
                except Exception:
                    pass
                del self._ghost_data[oldest_key]

            # Add to scene
            ghost_name = f"_ghost_{self._ghost_counter}"
            self._ghost_counter += 1

            actor = self.plotter.add_mesh(
                poly,
                color=color,
                opacity=opacity,
                show_edges=True,
                edge_color="black",
                line_width=1,
                name=ghost_name,
                pickable=False,
                user_matrix=transform,
                lighting=False,
            )
            
            # REFRESH RULE: When moving this link, refresh all its existing shadows
            # so the trail only starts its countdown after the LAST move.
            now = _time.time()
            for g_name, g_data in self._ghost_data.items():
                if g_data.get('link') == link_name:
                    g_data['start_time'] = now

            self._ghost_data[ghost_name] = {
                'actor': actor,
                'start_time': now,
                'link': link_name,
                'init_opacity': opacity
            }

        except Exception:
            pass

    def _process_ghost_fading(self):
        """Shadows stay while simulation is running, then whisper (fade) over 10s."""
        import time as _time
        now = _time.time()
        
        # Check if simulation is running to prevent any expiration
        is_running = False
        try:
            if hasattr(self.window(), 'experiment_tab') and hasattr(self.window().experiment_tab, 'program_tab'):
                is_running = self.window().experiment_tab.program_tab.is_running
        except:
            pass

        to_remove = []
        for name, data in self._ghost_data.items():
            if is_running:
                # Refresh constantly so simulation period doesn't count towards 10s
                data['start_time'] = now
                continue

            age = now - data['start_time']
            if age >= 13.0: # Total 13s post-sim (10s solid + 3s whisper)
                to_remove.append(name)
            elif age >= 10.0:
                # Whisper effect: starts after 10s, fades over 3s
                fade = 1.0 - ((age - 10.0) / 3.0)
                new_opacity = data.get('init_opacity', 0.1) * max(0, fade)
                data['actor'].GetProperty().SetOpacity(new_opacity)
            else:
                # Stay fully visible for the first 10s after sim
                data['actor'].GetProperty().SetOpacity(data.get('init_opacity', 0.1))
        
        if to_remove:
            for name in to_remove:
                try:
                    self.plotter.remove_actor(self._ghost_data[name]['actor'])
                except Exception:
                    pass
                del self._ghost_data[name]
                
        if self._ghost_data or to_remove:
            try:
                self.plotter.render()
            except:
                pass

    def clear_joint_ghosts(self):
        """Removes all ghost shadow actors from the scene."""
        if not hasattr(self, '_ghost_data'): return
        for name in list(self._ghost_data.keys()):
            try:
                self.plotter.remove_actor(self._ghost_data[name]['actor'])
            except:
                pass
        self._ghost_data.clear()
        self.plotter.render()

    def clear_rotation_discs(self):
        """Removes rotation disc overlays from the scene."""
        pass
