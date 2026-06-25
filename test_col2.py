import numpy as np
from core.robot import Robot, Link
import pyvista as pv

r = Robot()
link1 = Link("link1")
link1.mesh = pv.Box(bounds=(-5, 5, -5, 5, -5, 5))
link1.t_world = np.eye(4)
r.links["link1"] = link1

link2 = Link("link2")
link2.mesh = pv.Box(bounds=(0, 10, 0, 10, 0, 10))
link2.t_world = np.eye(4)
r.links["link2"] = link2

print("Collision:", r.has_self_collision())
