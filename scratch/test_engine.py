import pyvista as pv
import numpy as np

def test_engine():
    print("Initializing PyVista Test Engine...")
    
    # Create a simple plotter
    plotter = pv.Plotter()
    plotter.set_background("white")
    
    # Add a floor grid
    grid = pv.Plane(i_size=10, j_size=10)
    plotter.add_mesh(grid, color="#e0e0e0", show_edges=True)
    
    # Create a "robot link" (box)
    link1 = pv.Box(bounds=[-0.5, 0.5, -0.5, 0.5, 0, 2])
    actor1 = plotter.add_mesh(link1, color="#1976d2", show_edges=True)
    
    # Create a second "robot link" (cylinder)
    link2 = pv.Cylinder(center=(0, 0, 2.5), direction=(0, 0, 1), radius=0.3, height=1.0)
    actor2 = plotter.add_mesh(link2, color="#2e7d32", show_edges=True)
    
    # Add axes
    plotter.add_axes()
    
    print("Opening 3D Window... (Close it to continue)")
    plotter.show()

if __name__ == "__main__":
    test_engine()
