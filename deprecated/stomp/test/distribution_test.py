#!/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt

#t = np.arange(0.0, 5.0, 0.001)
#s = np.exp(-t*t)
#plt.plot(t,s)
#plt.show()

d = 10

A = np.random.random([d,d])

print "A = ", A

R = np.dot(A.T,A)

R = np.eye(d)

print "R = ", R

R_inv = np.linalg.inv(R)

print "R_inv= ", R_inv

R_inv_chol = np.linalg.cholesky(R_inv)

print "R_inv_chol = ", R_inv_chol

test = np.dot(R_inv_chol.T, np.dot(R, R_inv_chol))
print "test = ", test

N = 100000
costs = np.zeros((N))
costs2 = np.zeros((N))
costs3 = np.zeros((N))
values = np.zeros((N))

for i in range(0,N):
    eps = np.random.randn(d)
    #print "eps = ", eps
    n_eps = np.dot(R_inv_chol, eps)

    #print "n_eps = ", n_eps
    old_length = np.dot(n_eps.T, np.dot(R, n_eps))
    #nn_eps = np.dot(R_inv, eps)
    #print "nn_eps = ", nn_eps
    #cost2 = np.dot(nn_eps.T, np.dot(R, nn_eps))
    #print cost, cost2

    # sample a random length and apply it
    new_length = np.random.randn()
    n_eps = n_eps / math.sqrt(old_length)
    n_eps = n_eps * new_length;

    cost = np.dot(n_eps.T, np.dot(R, n_eps))
    costs[i] = cost
    costs2[i] = math.exp(-cost)
    costs3[i] = math.sqrt(cost)
    values[i] = eps[0]
    
    #if (costs[i] > 0.02):
    #    costs[i] = 0.02

    #costs[i] = cost
    #costs2[i] = cost2

plt.figure()
plt.hist(costs, 1000)
plt.title('cost')

plt.figure()
plt.hist(costs2, 1000)
plt.title('exp(-cost)')

plt.figure()
plt.hist(costs3, 1000)
plt.title('sqrt(cost)')

plt.figure()
plt.hist(values, 1000)
plt.title('eps[0]')

plt.show()