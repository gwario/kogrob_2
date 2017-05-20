# Example of 1D Robot localization using the histogramm method
from __future__ import division
import numpy as np

DIMENSIONS = 2

if DIMENSIONS == 2:
    world = [['green', 'red', 'red', 'green', 'green'],
             ['green', 'red', 'red', 'green', 'green'],
             ['green', 'red', 'red', 'red', 'green'],
             ['red', 'green', 'red', 'green', 'green'],
             ['green', 'red', 'red', 'green', 'red'],
             ['green', 'green', 'red', 'green', 'green'],
             ['red', 'red', 'green', 'green', 'green']]
elif DIMENSIONS == 1:
        world = ['green', 'red', 'red', 'green', 'green']

pHit = 0.6
pMiss = 0.2
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
                xNew = (x + u[0]) % len(p[y])
                yNew = (y - u[1]) % len(p)
                pExactPos = p[yNew][xNew]

                # Prevent diagonal movements due to over-/undershoot 
                if u[0] != 0:
                    xNewOvershoot = (x + u[0] + 1) % len(p[y])
                    xNewUndershoot = (x + u[0] - 1) % len(p[y])
                else:
                    xNewOvershoot = x
                    xNewUndershoot = x

                if u[1] != 0:
                    yNewOvershoot = (y + u[1] + 1) % len(p)
                    yNewUndershoot = (y + u[1] - 1) % len(p[y])
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


def print_loc(world, p):
    if DIMENSIONS == 1:
        world_row = ["world:"]
        world_row.extend(world)
        p_row = ["p:"]
        p_row.extend(p)

        print_aligned([world_row, p_row])

    elif DIMENSIONS == 2:
        print("world:")
        print_aligned(world)
        print("p:")
        print_aligned(p)


def print_mov(move, sense):
    print_aligned([["sense:", sense], ["move:", move]])


def get_uniform_distributed_positions(world):
    postitionCount = len(world)*len(world[0])
    pPos = 1 / postitionCount
    return np.full((len(world), len(world[0])), pPos)


if __name__ == "__main__":
    if DIMENSIONS == 2:
        p = get_uniform_distributed_positions(world)
    elif DIMENSIONS == 1:
        p = [0.2, 0.2, 0.2, 0.2, 0.2]

    measurements = ['red', 'red', 'red', 'red', 'red']

    if DIMENSIONS == 1:
        motions = [1, 1]
    elif DIMENSIONS == 2:
        motions = [[0, -1], [0, -1], [0, -1], [0, -1], [0, -1]]#only one step at a time possible

    print_loc(world, np.around(p, 3))
    print("---------------------")

    for k in range(len(measurements)):
        p = sense(p, measurements[k])
        print("Sense:")
        print_mov(motions[k], measurements[k])
        print_loc(world, np.around(p, 3))

        p = move(p, motions[k])
        print("Move:")
        print_mov(motions[k], measurements[k])
        print_loc(world, np.around(p, 3))
        print("---------------------")

    #print("p", np.around(p, 4))
