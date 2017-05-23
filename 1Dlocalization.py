# Example of 1D Robot localization using the histogramm method
from __future__ import division
import numpy as np
import math as m

DIMENSIONS = 2

if DIMENSIONS == 2:
    world = [['green', 'green', 'green', 'red', 'green'],
             ['green', 'red',   'red',   'red', 'green'],  #          ^ -> -> |
             ['red',   'red',   'green', 'red', 'red'],    # red -> | |       v -> -> start/red
             ['green', 'red',   'green', 'red', 'green'],  #        | |
             ['green', 'red',   'green', 'red', 'green'],  #        v |
             ['green', 'green', 'green', 'red', 'green']]
elif DIMENSIONS == 1:
        world = ['green', 'red', 'red', 'green', 'green']

pHit = 0.6      # 1.0
pMiss = 0.2     # 0.0
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1


def sense(p, z):

    if DIMENSIONS == 1:
        q = []

        # calculate p after sense
        for i in range(len(p)):
            hit = (z == world[i])
            q.append(p[i] * (hit * pHit + (1 - hit) * pMiss))

        # normalize
        s = sum(q)
        for i in range(len(q)):
            q[i] = q[i] / s

        return q

    elif DIMENSIONS == 2:
        q = []

        # calculate p after sense
        for y in range(len(p)):
            q.append([])
            for x in range(len(p[y])):
                hit = (z == world[y][x])
                q[y].append(p[y][x] * (pHit if hit else pMiss))

        # normalize
        s = sum(map(sum, q))
        for y in range(len(q)):
            for x in range(len(q[y])):
                q[y][x] = q[y][x] / s

        return q


def move(p, u):

    if DIMENSIONS == 1:
        q = []
        for i in range(len(p)):
            pExactPos = p[(i - u) % len(p)]
            pOvershootPos = p[(i - u - 1) % len(p)]
            pUndershootPos = p[(i - u + 1) % len(p)]

            s = pExact * pExactPos + pOvershoot * pOvershootPos + pUndershoot * pUndershootPos
            q.append(s)
        return q

    elif DIMENSIONS == 2:
        q = []
        for y in range(len(p)):
            q.append([])
            for x in range(len(p[y])):
                # To make the movement direction intuitive, [0,1] results in a move up and [0,-1] down.
                xNew = (x - u[0]) % len(p[y])
                yNew = (y + u[1]) % len(p)
                pExactPos = p[yNew][xNew]

                # Prevent diagonal movements due to over-/undershoot. The over/undershoot is defined as one step
                # further/less further in direction of movement.
                if u[0] != 0:
                    xNewOvershoot = (x - u[0] + int(m.copysign(1, u[0]))) % len(p[y])
                    xNewUndershoot = (x - u[0] - int(m.copysign(1, u[0]))) % len(p[y])
                else:
                    xNewOvershoot = x
                    xNewUndershoot = x

                if u[1] != 0:
                    yNewOvershoot = (y + u[1] + int(m.copysign(1, u[1]))) % len(p)
                    yNewUndershoot = (y + u[1] - int(m.copysign(1, u[1]))) % len(p)
                else:
                    yNewOvershoot = y
                    yNewUndershoot = y

                pOvershootPos = p[yNewOvershoot][xNewOvershoot]
                pUndershootPos = p[yNewUndershoot][xNewUndershoot]

                s = pExact * pExactPos + pOvershoot * pOvershootPos + pUndershoot * pUndershootPos
                q[y].append(s)
        return q


def print_aligned(myArray):
    mx = max((len(str(ele)) for sub in myArray for ele in sub))
    for row in myArray:
        print(" ".join(["{:<{mx}}".format(ele,mx=mx) for ele in row]))


def print_loc(world, sense, move, p):
    if sense is not None:
        print("Sense:")
        print("sensed: "+sense)
    if move is not None:
        print("Move:")
        print("moved: {}".format(move))

    if DIMENSIONS == 1:
        world_row = ["world:"]
        world_row.extend(world)
        p_row = ["p:"]
        p_row.extend(p)

        print_aligned([world_row, p_row])

    elif DIMENSIONS == 2:
        print("| world |")
        print("| p     |")
        world_with_p = []
        for i in range(len(world)):
            world_with_p.append(world[i])
            world_with_p.append(p[i])
            world_with_p.append([""] * len(world[i]))

        print_aligned(world_with_p)


def get_uniform_distributed_positions(world):
    postitionCount = len(world)*len(world[0])
    pPos = 1 / postitionCount
    return np.full((len(world), len(world[0])), pPos)


if __name__ == "__main__":
    if DIMENSIONS == 2:
        p = get_uniform_distributed_positions(world)
    elif DIMENSIONS == 1:
        p = [0.2, 0.2, 0.2, 0.2, 0.2]

    if DIMENSIONS == 1:
        measurements = ['red'] * 2
    else:
        measurements = ['red'] * 11

    if DIMENSIONS == 1:
        motions = [1, 1]
    elif DIMENSIONS == 2:
        motions = [[1, 0], [0, -1], [0, -1], [0, 1], [0, 1], [0, 1], [1, 0], [1, 0], [0, -1], [1, 0], [1, 0]]

    print_loc(world, None, None, np.around(p, 3))
    print("---------------------")

    for k in range(len(measurements)):
        p = sense(p, measurements[k])
        print_loc(world, measurements[k], None, np.around(p, 3))

        p = move(p, motions[k])
        print_loc(world, None, motions[k], np.around(p, 3))
        print("---------------------")

    #print("p", np.around(p, 4))
