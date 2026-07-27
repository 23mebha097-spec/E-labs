import unittest

from graphics.canvas import RobotCanvas


class DummyRenderer:
    def __init__(self):
        self.actors = {
            "pick_highlight_a": object(),
            "pick_arrow_b": object(),
            "other_actor": object(),
        }
        self.removed = []

    def RemoveActor(self, actor):
        self.removed.append(actor)


class DummyPlotter:
    def __init__(self):
        self.renderer = DummyRenderer()
        self.render_calls = 0

    def remove_actor(self, actor):
        if isinstance(actor, str):
            raise AssertionError("clear_highlights should not rely on remove_actor by name")
        self.renderer.RemoveActor(actor)

    def render(self):
        self.render_calls += 1


class DummyRendererSequence:
    def __init__(self):
        self.actors = {
            "pick_highlight_a": object(),
            "pick_arrow_b": object(),
            "other_actor": object(),
        }
        self._actors = [self.actors["pick_highlight_a"], self.actors["pick_arrow_b"]]
        self.removed = []

    def RemoveActor(self, actor):
        self.removed.append(actor)


class DummyPlotterSequence(DummyPlotter):
    def __init__(self):
        self.renderer = DummyRendererSequence()
        self.render_calls = 0


class CanvasClearHighlightsTests(unittest.TestCase):
    def test_clear_highlights_removes_face_highlight_actors(self):
        canvas = RobotCanvas.__new__(RobotCanvas)
        canvas.plotter = DummyPlotter()

        actors_to_remove = [canvas.plotter.renderer.actors["pick_highlight_a"], canvas.plotter.renderer.actors["pick_arrow_b"]]

        canvas.clear_highlights()

        self.assertEqual(canvas.plotter.render_calls, 1)
        self.assertEqual(len(canvas.plotter.renderer.removed), 2)
        self.assertTrue(all(actor in canvas.plotter.renderer.removed for actor in actors_to_remove))

    def test_clear_highlights_handles_sequence_backed_renderer_actors(self):
        canvas = RobotCanvas.__new__(RobotCanvas)
        canvas.plotter = DummyPlotterSequence()

        canvas.clear_highlights()

        self.assertEqual(canvas.plotter.render_calls, 1)
        self.assertEqual(len(canvas.plotter.renderer.removed), 2)
        self.assertEqual(canvas.plotter.renderer._actors, [])


if __name__ == "__main__":
    unittest.main()
