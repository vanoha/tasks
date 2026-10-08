"""STUDENT FILE. Implement the search state machine. No ROS calls in this module.
Use raster() and drive_to(); never read simulator internals or web /state.
Return Outcome('absent') ONLY after covering the entire requested area.
Return Outcome('found', x, y) after measuring the rectangle centre.
"""
from course_lab.world import Area, Sample, Decision, Outcome
from course_lab.navigation import drive_to, raster
from course_lab.probe import RectangleProbe

def spiral(a, step=0.45, margin=0.02):
    """
    Another way to reach the gift via more attractive way - spiral
    """
    x0, y0 = a.x0 + margin, a.y0 + margin
    x1, y1 = a.x1 - margin, a.y1 - margin
    pts = []
    while x0 <= x1 and y0 <= y1:
        pts += [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0 + step)]
        x0 += step; y0 += step; x1 -= step; y1 -= step
    return pts

class Mission:
    def __init__(self, area: Area):
        self.area = area
        # self.waypoints = spiral(area)
        self.waypoints = raster(area)
        self.index = 0
        self.probe = None
        self.distance = 0.0
        self.last = None

    def step(self, sample: Sample) -> Decision:
        dist = lambda a, b: ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5
        if self.last is not None:
            self.distance += dist(sample, self.last)
        self.last = sample

        if self.probe is None and sample.green and self.area.contains(sample.x, sample.y):
            self.probe = RectangleProbe(self.area, sample)

        if self.probe is not None:
            d = self.probe.step(sample)
            return Decision(d.v, d.w, d.outcome, True, self.distance)

        if self.index >= len(self.waypoints):
            return Decision(outcome=Outcome('absent'), distance=self.distance)

        v, w, reached = drive_to(sample, self.waypoints[self.index])
        if reached:
            self.index += 1
        return Decision(v, w, distance=self.distance)
