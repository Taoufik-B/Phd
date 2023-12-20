[C2M1L1](https://www.coursera.org/learn/state-estimation-localization-self-driving-cars/ungradedLab/oUqWi/lesson-1-practice-notebook-least-squares/lab?path=%2Fnotebooks%2Fmodule%25201%2FC2M1L1.ipynb#)

  

# Define the H matrix - what does it contain?

# what we know V = RI => R = V / I => r1, r2, r3 ... = v1/i1 + n1, v2/i2 + n2

# y = r1 ...

# x = v1/i1 ...

# e1 = y1 -x1

# e = y - Hx

# stacked voltage measurement:

# H = ...

R_hat = V / I

  

print(R_hat)

  

H = np.array([[1, 1, 1, 1, 1]]).T

  

print(H)

  

# Now estimate the resistance parameter.

# inv(H.T@H)@H.T@y

# R = ...

  

R = inv(H.T@H)@H.T@R_hat

print(R)

print('The slope parameter of the best-fit line (i.e., the resistance) is:')

# print(R[0, 0])