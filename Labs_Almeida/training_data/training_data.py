import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import codecs


MULTIPLE = False
IMAGE = True
CALC_TAU_TEORICO = False

# load variables from files and check their dimensions
def load_variables(name_file_x: str) -> (np.array,np.array):

    # load files
    f = codecs.open(name_file_x, "r", "utf-16")

    temp = f.read() # read file
    temp = temp.rsplit('\r\n')  # slip each line
    #temp.pop() # remove last line

    lux = [[],]
    PWM = [[],]
    i = 0
    for line in temp:
        
        if line == 'STEP':
            i+=1
            lux.append([])
            PWM.append([])
            continue

        s = line.rsplit('\t')
        lux[i].append(s[0])
        PWM[i].append(s[1])

    lux = np.array(lux, dtype=np.float64)
    PWM = np.array(PWM, dtype=np.float64)

    return (lux,PWM)

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


best = []
if MULTIPLE:
    xx,yy = load_variables('multiples3.txt')
    for i in range(len(yy)):
        x=xx[i]
        y=yy[i]
        lux = np.array(mean_vect(x,y)).reshape(-1, 1)

        declive = (lux[-1]-lux[0])/255
        Y = np.linspace(lux[0],lux[-1],256).reshape(-1, 1)

        plt.figure()
        plt.plot(lux,lw=2)
        plt.plot(Y, ls='--', color='red')
        plt.draw()
        best.append(computeSSE(lux,Y))
        print(f'SSE: {best[-1]}')
        breakpoint

else:

    x,y = load_variables('calibrated3.txt')
    lux = np.array(mean_vect(x,y)).reshape(-1, 1)
    plt.figure()
    plt.plot(lux,lw=2)
    plt.plot((0,255),(lux[0],lux[-1]), ls='--', color='red', lw=0.5)
    plt.draw()

    X = np.linspace(0,255,256).reshape(-1, 1)
    Y = np.linspace(lux[0],lux[-1],256).reshape(-1, 1)
    best.append(computeSSE(lux,Y))

    print(f'SSE: {best[-1][0][0]}')
    
    beta = inv(X.T@X)@(X.T@lux)

    print(f'G: {beta[0][0]} [Lux/PWM]')


plt.xlabel('PWM')
plt.ylabel('Lux')
if IMAGE: plt.show()

if CALC_TAU_TEORICO:
    m = -0.672;
    b = np.log10(5E4)
    function = m*np.log10(lux)+b

    R2_x = pow(10, function)
    R1 = 1E4
    C1 = 1E-6
    Req_x = (R1*R2_x)/(R1+R2_x)
    tau_teorico_x = C1*Req_x

    plt.clf()
    plt.plot(tau_teorico_x)
    plt.show()

breakpoint

