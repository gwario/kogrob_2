from __future__ import absolute_import
import random
import math
import bisect
import numpy as np
from draw import Maze

# 0 - empty square
# 1 - occupied square
# 2 - occupied square with a beacon at each corner, detectable by the robot

maze_data = ((1, 1, 0, 0, 2, 0, 0, 0, 0, 1),
             (1, 2, 0, 0, 1, 1, 0, 0, 0, 0),
             (0, 1, 1, 0, 0, 0, 0, 1, 0, 1),
             (0, 0, 0, 0, 1, 0, 0, 1, 1, 2),
             (1, 1, 0, 1, 1, 2, 0, 0, 1, 0),
             (1, 1, 1, 0, 1, 1, 1, 0, 2, 0),
             (2, 0, 0, 0, 0, 0, 0, 0, 0, 0),
             (1, 2, 0, 1, 1, 1, 1, 0, 0, 0),
             (0, 0, 0, 0, 1, 0, 0, 0, 1, 0),
             (0, 0, 1, 0, 0, 2, 1, 1, 1, 0))

PARTICLE_COUNT = 2000    # Total number of particles

ROBOT_HAS_COMPASS = True
# ------------------------------------------------------------------------
# Some utility functions


def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]


def add_little_noise(*coords):
    return add_noise(0.02, *coords)


def add_some_noise(*coords):
    return add_noise(0.1, *coords)

# This is just a gaussian kernel I pulled out of my hat, to transform
# values near to robbie's measurement => 1, further away => 0
sigma2 = 0.9 ** 2


def w_gauss(a, b):
    error = a - b
    g = math.e ** -(error ** 2 / (2 * sigma2))
    return g
# ------------------------------------------------------------------------


def compute_mean_point(particles):
    """
    Compute the mean for all particles that have a reasonably good weight.
    This is not part of the particle filter algorithm but rather an
    addition to show the "best belief" for current position.
    """

    m_x, m_y, m_count = 0, 0, 0
    for p in particles:
        m_count += p.w
        m_x += p.x * p.w
        m_y += p.y * p.w

    if m_count == 0:
        return -1, -1, False

    m_x /= m_count
    m_y /= m_count

    # Now compute how good that mean is -- check how many particles
    # actually are in the immediate vicinity
    m_count = 0
    for p in particles:
        if world.distance(p.x, p.y, m_x, m_y) < 1:
            m_count += 1

    return m_x, m_y, m_count > PARTICLE_COUNT * 0.95
# ------------------------------------------------------------------------


class Particle(object):
    def __init__(self, x, y, heading=None, w=1, noisy=False):
        if heading is None:
            heading = random.uniform(0, 360)
        if noisy:
            x, y, heading = add_some_noise(x, y, heading)

        self.x = x
        self.y = y
        self.h = heading
        self.w = w

    def __repr__(self):
        return "(%f, %f, w=%f)" % (self.x, self.y, self.w)

    @property
    def xy(self):
        return self.x, self.y

    @property
    def xyh(self):
        return self.x, self.y, self.h

    @classmethod
    def create_random(cls, count, maze):
        return [cls(*maze.random_free_place()) for _ in range(0, count)]

    def read_sensor(self, maze):
        """
        Find distance to nearest beacon.
        """
        return maze.distance_to_nearest_beacon(*self.xy)

    def advance_by(self, speed, checker=None, noisy=False):
        h = self.h
        if noisy:
            speed, h = add_little_noise(speed, h)
            h += random.uniform(-3, 3)  # needs more noise to disperse better
        r = math.radians(h)
        dx = math.sin(r) * speed
        dy = math.cos(r) * speed
        if checker is None or checker(self, dx, dy):
            self.move_by(dx, dy)
            return True
        return False

    def move_by(self, x, y):
        self.x += x
        self.y += y
# ------------------------------------------------------------------------


class Robot(Particle):
    speed = 0.2

    def __init__(self, maze):
        super(Robot, self).__init__(*maze.random_free_place(), heading=90)
        self.chose_random_direction()
        self.step_count = 0

    def chose_random_direction(self):
        heading = random.uniform(0, 360)
        self.h = heading

    def read_sensor(self, maze):
        """
        Poor robot, it's sensors are noisy and pretty strange,
        it only can measure the distance to the nearest beacon(!)
        and is not very accurate at that too!
        """
        return add_little_noise(super(Robot, self).read_sensor(maze))[0]

    def move(self, maze):
        """
        Move the robot. Note that the movement is stochastic too.
        """
        while True:
            self.step_count += 1
            if self.advance_by(self.speed, noisy=True, checker=lambda r, dx, dy: maze.is_free(r.x+dx, r.y+dy)):
                break
            # Bumped into something or too long in same direction,
            # chose random new direction
            self.chose_random_direction()

# ------------------------------------------------------------------------


def rws(particles, pointers):
    keep = []
    weights = [p.w for p in particles]
    sums = np.cumsum(weights)
    for p in pointers:
        i = 0
        while sums[i] < p:
            i += 1

        keep.append(particles[i])

    return keep


def sus(particles):
    weight_sum = sum([p.w for p in particles])
    n = len(particles)
    p = weight_sum/n
    start = random.uniform(0, p)
    pointers = [start + i*p for i in range(n)]
    return rws(particles, pointers)


world = Maze(maze_data)
world.draw()

# initial distribution assigns each particle an equal probability
particles = Particle.create_random(PARTICLE_COUNT, world)
nao = Robot(world)

while True:
    # Read Robot's sensor
    r_d = nao.read_sensor(world)

    weight_sum = 0
    for particle in particles:

        d = particle.read_sensor(world)

        dev = max(abs(d - r_d), 0.00001)
        w = 1 / dev
        particle.w = w

        #particle.w = w_gauss(d, r_d)
        weight_sum += particle.w

    for particle in particles:

        particle.w /= weight_sum


    # ---------- Show current state ----------
    world.show_particles(particles)
    # world.show_mean(m_x, m_y, m_confident)
    world.show_robot(nao)

    # ---------- Shuffle particles ----------
    new_particles = sus(particles)

    for p in new_particles:
        x, y = p.x, p.y
        p.x, p.y = add_some_noise(p.x, p.y)

    particles = new_particles

    # ---------- Move things ----------
    old_heading = nao.h
    nao.move(world)
    d_h = nao.h - old_heading

    # Move particles according to my belief of movement
    for p in particles:
        p.h += d_h  # in case robot changed heading, swirl particle heading too
        p.advance_by(nao.speed)
