# import numpy as np
# import matplotlib.pylab as plt
# import padasip as pa
# from astropy.modeling import models, fitting
#
# # creation of data
# N = 500
# x = np.random.normal(0, 1, (N, 4)) # input matrix
# v = np.random.normal(0, 0.1, N) # noise
# d = 2*x[:,0] + 0.1*x[:,1] - 4*x[:,2] + 0.5*x[:,3] + v # target
#
# # identification
# f = pa.filters.FilterLMS(n=4, mu=0.1, w="random")
# y, e, w = f.run(d, x)
# print(y, e, w)
# print("W ", w)
# # show results
# plt.figure(figsize=(15,9))
# plt.subplot(211);plt.title("Adaptation");plt.xlabel("samples - k")
# plt.plot(d,"b", label="d - target")
# plt.plot(y,"g", label="y - output");plt.legend()
# plt.subplot(212);plt.title("Filter error");plt.xlabel("samples - k")
# plt.plot(10*np.log10(e**2),"r", label="e - error [dB]");plt.legend()
# plt.tight_layout()
# # plt.plot(d)
# plt.show()




import numpy as np
import matplotlib.pyplot as plt
import random

# weight_vector = np.zeros(3)
# w = np.array([0.1, 0.1, 0.1]) #25 weights

error_grp = []
iter_grp = []
# initialise the training patterns
training = [[np.array([-0.5, 1.2, -0.1]).transpose(), 0.2], [np.array([0.7, -0.5, -0.2]).transpose(), -0.8],
            [np.array([0.3, 1.2, 2.3]).transpose(), 0.8],
            [np.array([1.2, 0.8, 1.0]).transpose(), 0.4], [np.array([-0.5, 1.2, -0.1]).transpose(), -0.2],
            [np.array([1.0, -0.3, 0.5]).transpose(), -0.1]]


def lms(w):
    i = 0
    alpha = [0.1, 0.01, 0.001]
    Emax = 1000000
    maxIter = 100
    E = 0
    while ((i < maxIter) and (E < Emax)):
        E = 0
        for pair in training:
            y = np.dot(w.transpose(), pair[0])
            w = w + np.dot((alpha[2] * (pair[1] - y)), pair[0])
            E = E + np.power((pair[1] - y), 2)
        # Put the error in the array after going thru the whole training pattern
        error_grp.append(E)
        iter_grp.append(i)
        i = i + 1

    print('Final error: ' + str(E))
    print('final weight: ' + str(w))

w = []
for x in range(25):
    for y in range(3):
        val = round(random.random(), 1)
        print(val)
        w.append(val)
    weight_vector = np.array(w)
    print('Weight vector: ' + str(weight_vector))
    lms(weight_vector)
    w = []

# Draw the graph
plt.plot(iter_grp, error_grp)
plt.ylabel('Error')
plt.xlabel('No. of iterations')
plt.show()
# plt.savefig('0.01.png')