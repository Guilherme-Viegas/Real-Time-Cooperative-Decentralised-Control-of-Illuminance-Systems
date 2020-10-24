import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import codecs
from scipy.optimize import curve_fit

MULTIPLE = False
CALIBRATED = False
IMAGE = False
CALC_TAU_TEORICO = False
STEP_RESPONSE = True


# load variables from files and check their dimensions
def load_variables(name_file_x: str) -> (np.array,np.array):

    # load files
    f = codecs.open(name_file_x, "r", "utf-16")

    temp = f.read() # read file
    temp = temp.rsplit('\r\n')  # slip each line
    #temp.pop() # remove last line

    lux = []
    PWM = []
    lux_aux = []
    PWM_aux = []
    i = 0
    for line in temp:
        
        if line == 'STEP':
            lux.append(np.array(lux_aux, dtype=np.float64)); lux_aux = []
            PWM.append(np.array(PWM_aux, dtype=np.float64)); PWM_aux = []
            i+=1
            continue

        s = line.rsplit('\t')
        lux_aux.append(s[0])
        PWM_aux.append(s[1])


    return (lux,PWM) if MULTIPLE else (np.array(lux_aux, dtype=np.float64),np.array(PWM_aux, dtype=np.float64))

def mean_vect(x,y):
    a = []
    for i in range(256):
        b = y[x==i]
        a.append(b.mean())

    return a

def computeSSE(y: np.array, fit: np.array) -> (np.array):

    y = y.reshape(-1, 1)
    fit = fit.reshape(-1, 1)

    e = y - fit # compute error vector
    sse = e.T@e # compute SSE

    return sse

def load_response(name_file_x: str) -> (np.array,np.array):
    
    # load files
    f = codecs.open(name_file_x, "r", "utf-16")

    temp = f.read() # read file
    temp = temp.rsplit('\r\n')  # slip each line
    #temp.pop() # remove last line
    time_offset = int(temp.pop(0))

    lux = []
    time = []
    lux_aux = []
    time_aux = []
    i = 0
    for line in temp:

        s = line.rsplit('\t')
        
        if s[0] == 'PWM':

            if time_aux != []:  # there are no first values
                
                
                size = int(len(lux_aux)*10)  # repetition last element
                lux_aux += [lux_aux[-1]]*size
                time_aux += [str(float(time_aux[-1])+1+i) for i in range(size)]

                time.append(np.array(time_aux, dtype=np.float64)); time_aux=[] # *time_offset, time in milliseconds
                lux.append(np.array(lux_aux, dtype=np.float64)); lux_aux=[]

            i = s[1]
            continue

        time_aux.append(s[0])
        lux_aux.append(s[1])
        
    # duplicate last element
    size = int(len(lux_aux)*10)  # repetition last element
    lux_aux += [lux_aux[-1]]*size
    time_aux += [str(float(time_aux[-1])+1+i) for i in range(size)]
    # append last pwm
    time.append(np.array(time_aux, dtype=np.float64))
    lux.append(np.array(lux_aux, dtype=np.float64))



    return (time,lux)

# estimate function
def exp(x, a, b, c):
    return a * np.exp(b * x) + c

# estimate function
def exp_get_x(y, a, b, c):
    return np.log((y-c)/a)/b

best = []
if MULTIPLE:
    xx,yy = load_variables('multiples3.txt')
    for i in range(len(yy)):    # for each trial
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
        print(f'SSE: {best[-1]}')   # new (last) trial
        breakpoint

if CALIBRATED:

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

if IMAGE:
    plt.xlabel('PWM')
    plt.ylabel('Lux')
    plt.show()

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

if STEP_RESPONSE:

    steps = ('positive', 'negative');
    for type in steps:
    
        time, lux = load_response(f"{type}_step_reponse.txt")
        beta = []
        tau = []
        dead_time = []
        for i in range(len(time)):

            coef_beta, _ = curve_fit(exp, time[i], lux[i], bounds=([-np.inf, -np.inf, 0],[0, 0, np.inf]))
            beta.append(coef_beta)
            # print(f'real value: {lux[i][-1]} - estimation: {coef_beta[2]} - {coef_beta[0]} - {coef_beta[1]}')

            # get tau values when system response is 63% of the final value
            tau.append(exp_get_x(0.63*coef_beta[2], *coef_beta))
            # get dead time for system response
            dead_time.append(exp_get_x(0, *coef_beta))
        
        pwm_array = np.linspace(1,256, 255)

        # get parameters 
        coef_tau, _ = curve_fit(exp, pwm_array, tau, bounds=([0, -np.inf, 0],[np.inf, 0, np.inf]))
        print('tau_{}: y = {} e^( {} x) + {}'.format(type, *coef_tau))
        coef_dead_time, _ = curve_fit(exp, pwm_array, dead_time, bounds=([0, -np.inf, 0],[np.inf, 0, np.inf]))
        print('dead_time_{}: y = {} e^( {} x) + {}'.format(type, *dead_time))

        plt.figure(10 + steps.index(type))
        plt.title(f"Tau according to PWM during {type} step")
        plt.plot(tau, marker='+', ls='')
        plt.plot(pwm_array, exp(pwm_array, *coef_tau), 'r')
        plt.xlabel('PWM')
        plt.ylabel('Tau [ms]')
        plt.draw()

        plt.figure(12 + steps.index(type))
        plt.title(f"Dead Time according to PWM during {type} step")
        plt.plot(dead_time, marker='+', ls='')
        plt.plot(pwm_array, exp(pwm_array, *coef_dead_time), 'r')
        plt.xlabel('PWM')
        plt.ylabel('Dead Time [ms]')
        plt.draw()

    plt.show()

    breakpoint