import time
from enum import Enum
import numpy as np

class ExecutionState(Enum):
    IDLE = 1
    READY = 2
    RUNNING = 3
    PAUSED = 4
    STOPPED = 5
    COMPLETED = 6
    ERROR = 7

class PathExecutor:
    """
    Simulation engine responsible for trajectory playback, execution state management,
    timing synchronization, and safe sampling.
    """
    def __init__(self):
        self.state = ExecutionState.IDLE
        self.trajectory = None
        
        self.elapsed_time = 0.0
        self.speed_factor = 1.0
        self.direction = 1.0 # 1.0 = forward, -1.0 = reverse
        self.total_duration = 0.0
        
        # Timing internals
        self._last_tick_time = None
        
    def load_trajectory(self, trajectory):
        """Loads a PathTrajectory and resets execution state."""
        self.trajectory = trajectory
        if trajectory and len(trajectory.points) > 0:
            self.total_duration = trajectory.timestamps[-1]
            self.state = ExecutionState.READY
            self.elapsed_time = 0.0
        else:
            self.state = ExecutionState.IDLE
            self.total_duration = 0.0

    def play(self, direction=1.0):
        """Starts or resumes trajectory execution."""
        self.direction = direction
        if self.state in [ExecutionState.READY, ExecutionState.PAUSED, ExecutionState.STOPPED, ExecutionState.COMPLETED]:
            if self.state == ExecutionState.COMPLETED and direction >= 0:
                self.elapsed_time = 0.0 # Auto-restart from beginning
            elif self.state == ExecutionState.READY and direction < 0:
                self.elapsed_time = self.total_duration # Auto-restart from end
            self.state = ExecutionState.RUNNING
            self._last_tick_time = time.time()

    def pause(self):
        """Pauses trajectory execution."""
        if self.state == ExecutionState.RUNNING:
            self.state = ExecutionState.PAUSED

    def stop(self):
        """Stops and rewinds execution to the beginning."""
        if self.state in [ExecutionState.RUNNING, ExecutionState.PAUSED, ExecutionState.COMPLETED]:
            self.state = ExecutionState.STOPPED
            self.elapsed_time = 0.0

    def reset(self):
        """Resets execution to READY state."""
        if self.trajectory:
            self.state = ExecutionState.READY
            self.elapsed_time = 0.0
            self._last_tick_time = None

    def seek(self, t):
        """Seeks execution timer to specific timestamp."""
        if not self.trajectory:
            return None
        self.elapsed_time = max(0.0, min(t, self.total_duration))
        if self.state in [ExecutionState.COMPLETED, ExecutionState.ERROR, ExecutionState.STOPPED]:
            self.state = ExecutionState.PAUSED
        return self.trajectory.sample_at_time(self.elapsed_time)

    def set_speed(self, speed_percent):
        """Sets playback speed multiplier (e.g. 100 = 1.0x)."""
        self.speed_factor = max(0.01, speed_percent / 100.0)

    def tick(self, dt=None):
        """
        Advances the execution clock deterministically or real-time.
        Returns: (point, normal, rotation_matrix, velocity, acceleration, reachable)
        or None if execution is not running.
        """
        if self.state != ExecutionState.RUNNING or not self.trajectory:
            return None

        now = time.time()
        
        if dt is None:
            if self._last_tick_time is None:
                self._last_tick_time = now
            dt = now - self._last_tick_time
            
        self._last_tick_time = now
        
        # Advance time considering speed multiplier and direction
        self.elapsed_time += dt * self.speed_factor * self.direction
        
        # Check completion
        if self.direction >= 0.0:
            if self.elapsed_time >= self.total_duration:
                self.elapsed_time = self.total_duration
                self.state = ExecutionState.COMPLETED
                return self.trajectory.sample_at_time(self.total_duration)
        else:
            if self.elapsed_time <= 0.0:
                self.elapsed_time = 0.0
                self.state = ExecutionState.READY
                return self.trajectory.sample_at_time(0.0)
            
        # Sample trajectory
        sample = self.trajectory.sample_at_time(self.elapsed_time)
        if sample:
            _, _, _, _, _, reachable = sample
            if not reachable:
                self.state = ExecutionState.ERROR
        
        return sample
        
    def get_progress(self):
        """Returns completion percentage 0.0 to 100.0"""
        if self.total_duration <= 0.0:
            return 0.0
        return min(100.0, (self.elapsed_time / self.total_duration) * 100.0)
