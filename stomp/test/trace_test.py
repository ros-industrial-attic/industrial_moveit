#!/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt

d = 10

A = np.random.random([d,d])

print "A = ", A

R = np.dot(A.T,A)

print "R = ", R

R_inv = np.linalg.inv(R)

#print "R_inv= ", R_inv

R_p = R.copy()
print "before: ", R_p

for i in xrange(0, d):
    R_p[i,i] = 0.0 * R_p[i,i]

print "after: ", R_p
sigma_p = np.linalg.inv(R_p)

print "sigma before: ", R_inv
print "sigma after: ", sigma_p

product = R.dot(sigma_p)

print "product = ", product
trace = product.trace()
print "trace = ", trace
