
    def show_workspace_planner(self, workspace_plan):
        """
        Visualizes the 2D workspace boundary, safe margin rectangle, light grid,
        center origin marker, and coordinate axis labels.
        """
        self.clear_workspace_planner()
        self.current_workspace_plan = workspace_plan

        # 1. Determine base world transform
        try:
            base_world_transform = self.window().robot.base_link.t_world
        except Exception:
            base_world_transform = np.eye(4)

        t_ws_world = workspace_plan.get_workspace_to_world_transform(base_world_transform)

        # Scale translation to internal units
        t_ws_world_scaled = t_ws_world.copy()
        t_ws_world_scaled[:3, 3] *= self.grid_units_per_cm

        # 2. Dimensions in internal units
        w = workspace_plan.width * self.grid_units_per_cm
        h = workspace_plan.height * self.grid_units_per_cm
        half_w = w / 2.0
        half_h = h / 2.0

        # 3. Draw Workspace Outer Boundary (Green)
        pts_local = np.array([
            [-half_w, -half_h, 0.0],
            [half_w, -half_h, 0.0],
            [half_w, half_h, 0.0],
            [-half_w, half_h, 0.0],
            [-half_w, -half_h, 0.0]
        ])
        boundary_line = pv.MultipleLines(points=pts_local)
        self._ws_boundary_actor = self.plotter.add_mesh(
            boundary_line,
            color="#2e7d32",  # Dark Green
            line_width=3,
            name="ws_boundary",
            pickable=False,
            lighting=False
        )
        self._ws_boundary_actor.user_matrix = t_ws_world_scaled

        # 4. Draw Workspace Safe Margin Boundary (Light Green)
        margin = workspace_plan.safe_margin * self.grid_units_per_cm
        if margin > 0:
            safe_pts_local = np.array([
                [-(half_w - margin), -(half_h - margin), 0.0],
                [(half_w - margin), -(half_h - margin), 0.0],
                [(half_w - margin), (half_h - margin), 0.0],
                [-(half_w - margin), (half_h - margin), 0.0],
                [-(half_w - margin), -(half_h - margin), 0.0]
            ])
            safe_boundary_line = pv.MultipleLines(points=safe_pts_local)
            self._ws_safe_boundary_actor = self.plotter.add_mesh(
                safe_boundary_line,
                color="#81c784",  # Light Green
                line_width=2,
                name="ws_safe_boundary",
                pickable=False,
                lighting=False
            )
            self._ws_safe_boundary_actor.user_matrix = t_ws_world_scaled

        # 5. Draw Light Grid
        res_x = int(round(workspace_plan.width / workspace_plan.grid_size))
        res_y = int(round(workspace_plan.height / workspace_plan.grid_size))
        res_x = max(1, res_x)
        res_y = max(1, res_y)
        grid_mesh = pv.Plane(
            center=(0.0, 0.0, 0.0),
            direction=(0.0, 0.0, 1.0),
            i_size=w,
            j_size=h,
            i_resolution=res_x,
            j_resolution=res_y
        )
        self._ws_grid_actor = self.plotter.add_mesh(
            grid_mesh,
            color="#e3f2fd",
            opacity=0.3,
            show_edges=True,
            edge_color="#bdbdbd",
            line_width=1,
            name="ws_grid",
            pickable=False,
            lighting=False
        )
        self._ws_grid_actor.user_matrix = t_ws_world_scaled

        # 6. Draw Center Origin Marker
        origin_mesh = pv.Sphere(radius=1.2 * self.grid_units_per_cm, center=(0.0, 0.0, 0.0))
        self._ws_origin_actor = self.plotter.add_mesh(
            origin_mesh,
            color="#1976d2",  # Accent Blue
            name="ws_origin_marker",
            pickable=False,
            lighting=True
        )
        self._ws_origin_actor.user_matrix = t_ws_world_scaled

        # 7. Draw Coordinate labels using Billboard Text Actors
        import vtkmodules.vtkRenderingCore as vtkRC
        self._workspace_label_actors = []

        x_ticks_cm = np.linspace(-workspace_plan.width / 2.0, workspace_plan.width / 2.0, num=5)
        y_ticks_cm = np.linspace(-workspace_plan.height / 2.0, workspace_plan.height / 2.0, num=5)

        for x_cm in x_ticks_cm:
            lbl_pos_local = np.array([x_cm, -workspace_plan.height / 2.0 - 4.0, 0.0])
            lbl_pos_world = (t_ws_world @ np.append(lbl_pos_local, 1))[:3] * self.grid_units_per_cm
            
            txt_actor = vtkRC.vtkBillboardTextActor3D()
            txt_actor.SetInput(f"{int(round(x_cm))} cm")
            txt_actor.SetPosition(lbl_pos_world[0], lbl_pos_world[1], lbl_pos_world[2])
            txt_actor.GetTextProperty().SetFontSize(10)
            txt_actor.GetTextProperty().SetColor(pv.Color("#424242"))
            txt_actor.GetTextProperty().SetBold(True)
            txt_actor.GetTextProperty().SetFontFamilyToArial()
            txt_actor.GetTextProperty().SetJustificationToCentered()
            txt_actor.SetPickable(False)
            self.plotter.renderer.AddActor(txt_actor)
            self._workspace_label_actors.append(txt_actor)

        for y_cm in y_ticks_cm:
            lbl_pos_local = np.array([-workspace_plan.width / 2.0 - 4.0, y_cm, 0.0])
            lbl_pos_world = (t_ws_world @ np.append(lbl_pos_local, 1))[:3] * self.grid_units_per_cm
            
            txt_actor = vtkRC.vtkBillboardTextActor3D()
            txt_actor.SetInput(f"{int(round(y_cm))} cm")
            txt_actor.SetPosition(lbl_pos_world[0], lbl_pos_world[1], lbl_pos_world[2])
            txt_actor.GetTextProperty().SetFontSize(10)
            txt_actor.GetTextProperty().SetColor(pv.Color("#424242"))
            txt_actor.GetTextProperty().SetBold(True)
            txt_actor.GetTextProperty().SetFontFamilyToArial()
            txt_actor.GetTextProperty().SetJustificationToCentered()
            txt_actor.SetPickable(False)
            self.plotter.renderer.AddActor(txt_actor)
            self._workspace_label_actors.append(txt_actor)

        self.plotter.render()

    def clear_workspace_planner(self):
        """Remove all workspace-related actors and labels."""
        self.current_workspace_plan = None
        actors_to_remove = ["ws_boundary", "ws_safe_boundary", "ws_grid", "ws_origin_marker"]
        for name in actors_to_remove:
            try:
                self.plotter.remove_actor(name)
            except Exception:
                pass
        
        if hasattr(self, "_workspace_label_actors"):
            for actor in self._workspace_label_actors:
                try:
                    self.plotter.renderer.RemoveActor(actor)
                except Exception:
                    pass
            self._workspace_label_actors = []
            
        self.plotter.render()

    def draw_trajectory_preview(self, trajectory):
        """
        Draws the 2D/3D trajectory path preview:
        - Blue preview line
        - Green reachable points
        - Red unreachable points
        - Highlighting path start and end points
        """
        self.clear_trajectory_preview()

        if self.current_workspace_plan is None:
            return

        try:
            base_world_transform = self.window().robot.base_link.t_world
        except Exception:
            base_world_transform = np.eye(4)

        t_ws_world = self.current_workspace_plan.get_workspace_to_world_transform(base_world_transform)

        pts_local = np.array(trajectory.points, dtype=float)
        pts_hom = np.hstack([pts_local, np.ones((len(pts_local), 1))])
        pts_world = (pts_hom @ t_ws_world.T)[:, :3]
        pts_world_scaled = pts_world * self.grid_units_per_cm

        if len(pts_world_scaled) > 1:
            line_mesh = pv.MultipleLines(points=pts_world_scaled)
            self._traj_line_actor = self.plotter.add_mesh(
                line_mesh,
                color="#1e88e5",  # Royal Blue
                line_width=3,
                name="traj_line",
                pickable=False,
                lighting=False
            )

        reachable_flags = trajectory.reachable_flags
        if not reachable_flags:
            reachable_flags = [True] * len(pts_world_scaled)

        reach_indices = [i for i, r in enumerate(reachable_flags) if r]
        unreach_indices = [i for i, r in enumerate(reachable_flags) if not r]

        if reach_indices:
            reach_pts = pts_world_scaled[reach_indices]
            reach_poly = pv.PolyData(reach_pts)
            reach_spheres = reach_poly.glyph(geom=pv.Sphere(radius=0.55 * self.grid_units_per_cm))
            self._traj_reach_actor = self.plotter.add_mesh(
                reach_spheres,
                color="#4caf50",  # Green
                name="traj_reach_pts",
                pickable=False,
                lighting=True
            )

        if unreach_indices:
            unreach_pts = pts_world_scaled[unreach_indices]
            unreach_poly = pv.PolyData(unreach_pts)
            unreach_spheres = unreach_poly.glyph(geom=pv.Sphere(radius=0.55 * self.grid_units_per_cm))
            self._traj_unreach_actor = self.plotter.add_mesh(
                unreach_spheres,
                color="#f44336",  # Red
                name="traj_unreach_pts",
                pickable=False,
                lighting=True
            )

        if len(pts_world_scaled) > 0:
            start_pt = pts_world_scaled[0]
            start_mesh = pv.Sphere(radius=1.0 * self.grid_units_per_cm, center=start_pt)
            self._traj_start_actor = self.plotter.add_mesh(
                start_mesh,
                color="#00b0ff",  # Bright Cyan
                name="traj_start_pt",
                pickable=False,
                lighting=True
            )

            end_pt = pts_world_scaled[-1]
            end_mesh = pv.Sphere(radius=1.0 * self.grid_units_per_cm, center=end_pt)
            self._traj_end_actor = self.plotter.add_mesh(
                end_mesh,
                color="#ff9100",  # Bright Orange
                name="traj_end_pt",
                pickable=False,
                lighting=True
            )

        self.plotter.render()

    def clear_trajectory_preview(self):
        """Remove all trajectory-related actors from the scene."""
        actors_to_remove = ["traj_line", "traj_reach_pts", "traj_unreach_pts", "traj_start_pt", "traj_end_pt"]
        for name in actors_to_remove:
            try:
                self.plotter.remove_actor(name)
            except Exception:
                pass
        self.plotter.render()
