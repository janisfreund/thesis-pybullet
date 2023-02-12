import random

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), '../thesis-ompl/ompl/py-bindings'))
    # sys.path.insert(0, join(dirname(abspath(__file__)), '../whole-body-motion-planning/src/ompl/py-bindings'))
    # print(sys.path)
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og

import camera_state_sampler

class Obj:
    def __init__(self):
        self.rng_ = ou.RNG()
        self.rng_.setLocalSeed(1)
        random.seed(1)

    def display(self):
        print(self.rng_.uniformReal(0, 100))
        print(self.rng_.uniformReal(0, 100))
        print(self.rng_.uniformReal(0, 50))
        print(self.rng_.uniformReal(0, 100))

    def display_random(self):
        print(random.uniform(0, 100))
        print(random.uniform(0, 100))
        print(random.uniform(0, 50))
        print(random.uniform(0, 100))

    def reset(self):
        self.rng_.setLocalSeed(1)
        random.seed(1)


if __name__ == '__main__':
    o = Obj()
    o.display_random()
    o.display_random()
    o.reset()
    o.display_random()
