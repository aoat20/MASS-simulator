import numpy as np


class World():
    def __init__(self,
                 t_step):
        self.t_step = t_step
        self.t_elapsed = 0

    def next_step(self):
        self.t_elapsed += self.t_step
