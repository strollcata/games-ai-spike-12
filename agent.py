from math import sqrt
from point2d import Point2D
from graphics import egi


class Agent(object):
    def __init__(self, world):
        self.world = world
        self.source_box = next((box for box in self.world.boxes if box.idx == self.world.path.source_idx), None)
        self.end_box = next((box for box in self.world.boxes if box.idx == self.world.path.target_idx), None)
        self.my_x = self.source_box._vc.x
        self.my_y = self.source_box._vc.y
        self.start_x = self.my_x
        self.start_y = self.my_y
        self.at_goal = False
        self.path_step = 1

    def draw(self):
        egi.blue_pen()
        egi.circle(Point2D(self.my_x, self.my_y), 5)

    def path_to_goal(self):
        target_box = next((box for box in self.world.boxes if box.idx == self.world.path.path[self.path_step]), None)
        self.my_x += (target_box._vc.x - self.start_x) / 10
        self.my_y += (target_box._vc.y - self.start_y) / 10
        if ((round(self.my_x, 2) == round(target_box._vc.x, 2)) and (round(self.my_y, 2) == round(target_box._vc.y, 2))):
            self.path_step += 1
            self.start_x = self.my_x
            self.start_y = self.my_y
        if ((round(self.my_x, 2) == round(self.end_box._vc.x, 2)) and (round(self.my_y, 2) == round(self.end_box._vc.y, 2))):
            self.at_goal = True
