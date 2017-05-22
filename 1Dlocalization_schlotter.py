# Example of 1D Robot localization using the histogramm method
from __future__ import division

world = ['green', 'red', 'green', 'green', 'green']
pHit = 1
pMiss = 0.0
pExact = 0.8
pOvershoot = 0.05
pUndershoot = 0.15

# Das Beispiel verdeutlicht warum i - u RICHTIG ist.

def sense(p, z):
    q = []
    for i in range(len(p)):
        hit = (z == world[i])
        q.append(p[i] * (hit * pHit + (1 - hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q


def move(p, u):
    q = []
    for i in range(len(p)):
        print(p)
        print(q)
        s = pExact * p[(i - u) % len(p)]
        s = s + pOvershoot * p[(i - u - 1) % len(p)]
        s = s + pUndershoot * p[(i - u + 1) % len(p)]
        q.append(s)
    return q


if __name__ == "__main__":
    p = [0.2, 0.2, 0.2, 0.2, 0.2]
    measurements = ['red']
    motions = [-1]

    for k in range(len(measurements)):
        p = sense(p, measurements[k])
        p = move(p, motions[k])

    print(p)
