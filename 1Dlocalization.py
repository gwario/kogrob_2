# Example of 1D Robot localization using the histogramm method
from __future__ import division
import numpy as np
import pprint as pp


#world = [['green', 'red', 'red', 'green', 'green'],
#         ['green', 'red', 'red', 'green', 'green'],
#         ['green', 'red', 'red', 'red', 'green'],
#         ['red', 'green', 'red', 'green', 'green'],
#         ['green', 'red', 'red', 'green', 'red'],
#         ['green', 'green', 'red', 'green', 'green'],
#         ['red', 'red', 'green', 'green', 'green']]
world = ['green', 'red', 'red', 'green', 'green']

pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1


def sense(p, z):
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


def move(p, u):
    q = []
    for i in range(len(p)):
        s = pExact * p[(i - u) % len(p)]
        s = s + pOvershoot * p[(i - u - 1) % len(p)]
        s = s + pUndershoot * p[(i - u + 1) % len(p)]
        q.append(s)
    return q


def print_aligned(myArray):
    mx = max((len(str(ele)) for sub in myArray for ele in sub))
    for row in myArray:
        print(" ".join(["{:<{mx}}".format(ele,mx=mx) for ele in row]))


def print_loc(world, p):
    world_row = ["world:"]
    world_row.extend(world)
    p_row = ["p:"]
    p_row.extend(p)

    print_aligned([world_row, p_row])


def print_mov(move, sense):
    print_aligned([["sense:", sense], ["move:", move]])


if __name__ == "__main__":
    #p = [[0.2, 0.2, 0.2, 0.2, 0.2],
    #     [0.2, 0.2, 0.2, 0.2, 0.2],
    #     [0.2, 0.2, 0.2, 0.2, 0.2],
    #     [0.2, 0.2, 0.2, 0.2, 0.2],
    #     [0.2, 0.2, 0.2, 0.2, 0.2],
    #     [0.2, 0.2, 0.2, 0.2, 0.2]]
    p = [0.2, 0.2, 0.2, 0.2, 0.2]

    measurements = ['red', 'green']
    motions = [1, 1]

    print_loc(world, np.around(p, 4))
    print("---------------------")

    for k in range(len(measurements)):
        p = sense(p, measurements[k])
        p = move(p, motions[k])

        print_mov(motions[k], measurements[k])
        print_loc(world, np.around(p, 4))
        print("---------------------")

    #print("p", np.around(p, 4))
