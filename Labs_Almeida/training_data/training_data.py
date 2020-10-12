import numpy as np
from numpy.linalg import inv, eig
import matplotlib.pyplot as plt
import codecs

TRAINING = False


def computeBeta(x: np.array, y: np.array) -> np.array:
    return inv(x.T@x)@x.T@y # compute beta

# load variables from files and check their dimensions
def load_variables(name_file_x: str,) -> (np.array,np.array):

    # load files
    f = codecs.open(name_file_x, "r", "utf-16")

    temp = f.read() # read file
    temp = temp.rsplit('\r\n')  # slip each line
    temp.pop() # remove last line

    lux_ = []
    R2 = []

    for line in temp:
        s = line.rsplit('\t')
        lux_.append(s[0])
        R2.append(s[1])

    lux_ = np.array(lux_, dtype=np.float64)
    R2 = np.array(R2, dtype=np.float64)
    R2 = np.log10(R2)

    N = max(R2.shape)    # number of polynomials
    lux = np.ones(shape=(N, 2), dtype=np.float)
    lux[:,1] = np.log10(lux_)

    return (lux,R2)



if TRAINING:

    x,y = load_variables('training_data3.txt')
    beta = computeBeta(x,y)
    fit = x@beta
    print(f"Data size: {max(y.shape)}")
    print(f"Model Fit: log10(R) = {beta[1]}m + {beta[0]}\n")


else:

    x,y = load_variables('results2.txt')
    plt.scatter(x[:,1],y, lw=1)
    plt.show()