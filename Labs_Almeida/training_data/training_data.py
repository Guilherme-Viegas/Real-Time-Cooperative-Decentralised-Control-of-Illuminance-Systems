import numpy as np
from numpy.linalg import inv, eig
import matplotlib.pyplot as plt
import codecs

# plot information on the console
DEBUG = True

# Question 2.1.2
def computeBeta(x: np.array, y: np.array) -> np.array:
    
    beta = inv(x.T@x)@x.T@y # compute beta

    if DEBUG: print(f"log10(R) = {beta[1]}m + {beta[0]}\n\nm -> {beta[1]}")

    return beta


# load variables from files and check their dimensions
def load_variables(name_file_x: str,) -> (np.array,np.array):

    # load files
    f = codecs.open(name_file_x, "r", "utf-16")

    temp = f.read() # read file
    temp = temp.rsplit('\r\n')  # slip each line
    temp.pop()  # remove last line

    vo_ = []
    R2 = []

    for line in temp:
        s = line.rsplit('\t')
        vo_.append(s[0])
        R2.append(s[1])

    vo_ = np.array(vo_, dtype=np.float64)
    R2 = np.array(R2, dtype=np.float64)


    N = max(R2.shape)    # number of polynomials
    vo = np.ones(shape=(N, 2), dtype=np.float)

    vo[:,1] = np.log(vo_)
    R2 = np.log(R2)

    return (vo,R2)


# Question 2.1.3
x1,y1 = load_variables('training_data.txt')

# Question 2.1.3.a)
beta = computeBeta(x1,y1)
fit = x1@beta
