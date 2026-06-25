"""Runtime binding between user scripts and the active E-Labs simulation."""

from contextlib import contextmanager

_active_simulation = None


def bind_simulation(simulation):
    """Bind the active MainWindow or ProgramPanel for subsequently created API objects."""
    global _active_simulation
    _active_simulation = simulation
    return simulation


def get_simulation():
    """Return the currently bound simulation object, or None outside the GUI runner."""
    return _active_simulation


def clear_simulation():
    """Clear the current simulation binding."""
    global _active_simulation
    _active_simulation = None


@contextmanager
def simulation_context(simulation):
    """Temporarily bind a simulation while executing user code."""
    global _active_simulation
    previous = _active_simulation
    _active_simulation = simulation
    try:
        yield simulation
    finally:
        _active_simulation = previous
