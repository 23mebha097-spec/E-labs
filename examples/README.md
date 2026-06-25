Examples for running simple robot demos with the repo.

Run examples from the repository root. Example commands:

 - Run the simple FK/IK demo:

```bash
python -m examples.simple_robot_demo
```

 - Run the square-drawing reachability check:

```bash
python -m examples.draw_square_demo
```

Notes:
- These demos use the project's internal `core` modules and assume the current working directory is the repository root so Python can import `core` as a top-level package.
- Units are in centimeters for transforms and workspace coordinates.
