import numpy as np
from numpy.linalg import inv, eig
import matplotlib.pyplot as plt
import codecs
from sklearn.linear_model import LinearRegression

MULTIPLE = False


# load variables from files and check their dimensions
def load_variables(name_file_x: str) -> (np.array,np.array):

    # load files
    f = codecs.open(name_file_x, "r", "utf-8")

    temp = f.read() # read file
    temp = temp.rsplit('\r\n')  # slip each line
    #temp.pop() # remove last line

    lux = [[],]
    R2 = [[],]
    i = 0
    for line in temp:
        
        if line == 'STEP':
            i+=1
            lux.append([])
            R2.append([])
            continue

        s = line.rsplit('\t')
        lux[i].append(s[0])
        R2[i].append(s[1])

    lux = np.array(lux, dtype=np.float64)
    R2 = np.array(R2, dtype=np.float64)

    return (lux,R2)


def mean_vect(x,y):
    a = []
    for i in range(256):
        b = y[x==i]
        a.append(b.mean())

    return a

def computeSSE(y: np.array, fit: np.array) -> np.array:


    y = y.reshape(-1, 1)
    fit = fit.reshape(-1, 1)

    e = y - fit # compute error vector
    sse = e.T@e # compute SSE

    return sse

filename='calibrate_steps.txt'

best = []
if MULTIPLE:
    xx,yy = load_variables(filename)
    for i in range(len(yy)):
        x=xx[i]
        y=yy[i]
        a = np.array(mean_vect(x,y)).reshape(-1, 1)

        declive = (a[-1]-a[0])/256
        Y = np.linspace(a[0],a[-1],256).reshape(-1, 1)

        plt.figure()
        plt.plot(a,lw=2)
        plt.plot(Y, ls='--', color='red')
        plt.draw()
        best.append(computeSSE(a,Y))
        print(f'SSE: {best[-1]}')
        breakpoint

else:

    x,y = load_variables(filename)
    a = np.array(mean_vect(x,y)).reshape(-1, 1)
    plt.figure()
    plt.plot(a,lw=2)
    plt.plot((0,255),(a[0],a[-1]), ls='--', color='red', lw=0.5)
    plt.draw()

    X = np.linspace(0,255,256).reshape(-1, 1)
    Y = np.linspace(a[0],a[-1],256).reshape(-1, 1)
    best.append(computeSSE(a,Y))

    print(f'SSE: {best[-1]}')
    
    beta = inv(X.T@X)@(X.T@a)

    print(f'G: {beta[0][0]} [Lux/PWM]')


plt.xlabel('PWM')
plt.ylabel('Lux')
plt.show()


breakpoint

