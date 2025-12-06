from ast import literal_eval

import numpy as np
from tnm import tnm, tnm_ba, householder, sub_reproj
import matplotlib.pyplot as plt
import plot

DATA_DIR = '../data/'

model = np.loadtxt(f'{DATA_DIR}/model.txt')
print(f'3D points (#_of_reference_points x 3): {model.shape}')

input1 = np.loadtxt(f'{DATA_DIR}/input1.txt')
input2 = np.loadtxt(f'{DATA_DIR}/input2.txt')
input3 = np.loadtxt(f'{DATA_DIR}/input3.txt')
input4 = np.loadtxt(f'{DATA_DIR}/input4.txt')
input5 = np.loadtxt(f'{DATA_DIR}/input5.txt')
#input = np.array([input1, input2, input3])
input = np.array([input1, input2, input3, input4, input5])
print(f'2D points (#_of_mirrors x #_of_reference_points x 2)): {input.shape}')

def comma_handler(val):
    # handle problem of `,` at .txt file
    res = str(val).replace(",", "")
    res = literal_eval(res)
    return res

K = np.loadtxt(f'{DATA_DIR}/camera.txt', converters=comma_handler)
print(f'K = {K}')

# linear solution
R0, T0, n0, d0, rep0 = tnm(model, input, K)
print(f'Reprojection error (linear) = {rep0:.3} px')

# non-linear refinement
R, T, n, d, rep = tnm_ba(model, input, K, R0, T0, n0, d0)
print(f'Reprojection error (BA) = {rep:.3} px')

print(R, T, n, d, rep)

# let's say 25% of observations are masked out, by assigning -1
if False:
    TH = 25
    rng = np.random.default_rng(0)
    for x in input:
        sign = rng.integers(0, 100, len(x))
        x[sign < TH, :] = -1
    #    print(x)

    print(f'{TH}% of observations are randomly masked')

    # linear solution
    R0, T0, n0, d0, rep0 = tnm(model, input, K)
    print(f'Reprojection error (linear) = {rep0:.3} px')

    # non-linear refinement
    R, T, n, d, rep = tnm_ba(model, input, K, R0, T0, n0, d0)
    print(f'Reprojection error (BA) = {rep:.3} px')
"""
R = np.array([[0.9552,   -0.0316,   -0.2942],
              [0.0262,    0.9994,   -0.0221],
              [0.2947,    0.0134,    0.9555]])
T = np.array([71.6716,   84.3404,  120.2696]).reshape((3, 1))
n1 = np.array([    0.2679,     0.0307,    -0.9629 ]).reshape((3, 1))
n2 = np.array([    0.4356,    0.0844,   -0.8962 ]).reshape((3, 1))
n3 = np.array([   -0.0443,   -0.0112,   -0.9990 ]).reshape((3, 1))
d1 =  386.2302
d2 =  355.0478
d3 =  404.7066
n = np.array([n1, n2, n3])
d = np.array([d1, d2, d3])
"""
e = sub_reproj(model, input, R, T, n, d, K)
print(e)

#sys.exit(0)

# plot
fig = plt.figure(figsize=(16,9))
ax = fig.add_subplot(1, 1, 1, projection='3d')

# chessboard
Cp = R @ model.T + T.reshape((3,1))
Cp = Cp.T
plt.plot(Cp[:,0], Cp[:,1], Cp[:,2], color='blue', marker='o')
# camera
plot.plotCamera(ax, np.eye(3), np.zeros(3).reshape((3,1)), color='red', scale=100)
for ni, di in zip(n, d):
    # mirrored points
    h = householder(ni, di)
    x = h[:3, :3] @ Cp.T + h[:3, 3].reshape((3,1))
    x = x.T
    p = plt.plot(x[:,0], x[:,1], x[:,2], marker='x')
    c = p[-1].get_color()

    # mirror
    m = (x + Cp) / 2
    plt.plot(m[:,0], m[:,1], m[:,2], marker='.', color=c)

plot.axisEqual3D(ax)
plt.show()

