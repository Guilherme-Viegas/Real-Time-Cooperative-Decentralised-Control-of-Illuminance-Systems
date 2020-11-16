import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import codecs
from scipy.optimize import curve_fit

MULTIPLE = True
CALIBRATED = True
CALC_TAU_TEORICO = False
IMAGE = True
STEP_RESPONSE = True


# load variables from files and check their dimensions
def load_variables(name_file_x: str, multiple: bool) -> (np.array,np.array):

    # load files
    f = codecs.open(name_file_x, "r", "utf-16")

    temp = f.read() # read file
    temp = temp.replace('\r','').split('\n')  # slip each line
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


    return (lux,PWM) if multiple else (np.array(lux_aux, dtype=np.float64),np.array(PWM_aux, dtype=np.float64))

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
    temp = temp.replace('\r','').split('\n')  # slip each line
    #temp.pop() # remove last line
    time_offset = int(temp.pop(0))

    lux = []
    time = []
    lux_aux = []
    time_aux = []
    lux_tau = []
    for line in temp:

        s = line.rsplit('\t')
        
        if s[0] == 'PWM':

            lux_tau.append(float(s[2]))

            if time_aux != []:  # there are no first values
                
                time.append(np.array(time_aux, dtype=np.float64)); time_aux=[] # *time_offset, time in milliseconds
                lux.append(np.array(lux_aux, dtype=np.float64)); lux_aux=[]

            continue

        time_aux.append(s[0])
        lux_aux.append(s[1])
        
    # append last pwm
    time.append(np.array(time_aux, dtype=np.float64))
    lux.append(np.array(lux_aux, dtype=np.float64))



    return (time,lux, lux_tau)

# estimate function
def exp(x, a, b, c):
    return a * np.exp(b * x) + c

best = []
if MULTIPLE:
    suf = '' # sufixe
    xx,yy = load_variables(f'text_files/multiples{suf}.txt', True)
    func = lambda x: -0.71 - 0.001*x if suf == '' else -0.6 - 0.01*x
    for i in range(len(yy)):    # for each trial
        x=xx[i]
        y=yy[i]
        lux = np.array(mean_vect(x,y)).reshape(-1, 1)

        declive = (lux[-1]-lux[0])/255
        Y = np.linspace(lux[0],lux[-1],256).reshape(-1, 1)

        best.append(computeSSE(lux,Y))
        print(f'SSE: {best[-1]}')   # new (last) trial

        plt.figure()
        plt.title(f'm = {func(i) :.3f}   -->   SSE: {best[-1][0][0] :.3f}')
        plt.plot(lux,lw=2)
        plt.plot(Y, ls='--', color='red')
        plt.draw()
        plt.xlabel('PWM')
        plt.ylabel('Lux')
        

if CALIBRATED:

    x,y = load_variables('text_files/calibrated.txt', False)
    lux = np.array(mean_vect(x,y)).reshape(-1, 1)

    X = np.linspace(0,255,256).reshape(-1, 1)
    Y = np.linspace(lux[0],lux[-1],256).reshape(-1, 1)
    best.append(computeSSE(lux,Y))

    print(f'SSE: {best[-1][0][0]}')
    
    beta = inv(X.T@X)@(X.T@lux)

    print(f'G: {beta[0][0]} [Lux/PWM]')
    
    plt.figure()
    plt.title("System gain")
    plt.plot(lux,lw=2)
    plt.plot((0,255),(lux[0],lux[-1]), ls='--', color='red', lw=0.5)
    plt.legend([f'Gaind: {beta[0][0]: .4f} [Lux/PWM]', 'Linear approx'])
    plt.xlabel('PWM')
    plt.ylabel('Lux')
    plt.draw()

    if CALC_TAU_TEORICO:

        m = -0.66;
        b = np.log10(5E4)
        function = m*np.log10(lux[lux!=0])+b

        R2_x = pow(10, function)
        R1 = 1E4
        C1 = 1E-6
        Req_x = (R1*R2_x)/(R1+R2_x)
        tau_teorico_x = C1*Req_x

        plt.figure()
        plt.title(f"Theorical Tau = Req*C")
        plt.plot(tau_teorico_x)
        plt.xlabel('PWM')
        plt.ylabel('Lux')
        plt.draw()

if STEP_RESPONSE:

    steps = ('positive', 'negative') #('positive', 'negative')
    
    dead_time = []  # computational delay, which should be constant for each arduino
    tau_main = {}

    for type in steps:
    
        time, lux, lux_tau = load_response(f"text_files/{type}_step_response.txt")

        beta = []
        tau = []
        pwm_array = []
        
        for i in range(len(lux)):
           
            # approx to a linear function  between two points, representing the gradient

            # at 63% of system response 
            if (lux[i][-1] != lux[i][-2]):  # the ldr can´t detect light changes
                declive_tau  = ( lux[i][-1] - lux[i][-2] ) / ( time[i][-1] - time[i][-2] ) 
                ordenada_origem_tau = lux[i][-1] - declive_tau * time[i][-1]
                tau.append( (lux_tau[i] - ordenada_origem_tau) / declive_tau)

            # measure the dead time

            if(len(lux[i]) > 4): # nees to have 2 points in the middle of 0 and 63%

                for j in range(len(lux[i])-2):  # the last element (63%) can not be in the equation

                    if lux[i][j] != lux[i][0] and lux[i][j] != lux[i][j+1]: # the ldr noticed changes during this gap
                        break

                else:           # there is not data enough to compute the dead time
                    continue 

                gradient = lux[i][j+1] - lux[i][j]
                if( gradient < 0 and type == 'positive' ) or (gradient > 0 and type == 'negative'):
                    # ensure that regardless the noise, the function is continuous
                    continue
                
                #computing dead time, by linear approach
                ordenada_origem_dead_time = lux[i][j] - gradient * time[i][j]
                dead_time.append( (lux[i][0] - ordenada_origem_dead_time) / gradient)

            breakpoint
        
        tau = np.array(tau)

        tau_main[type] = tau

    # compute dead time average 
    dead_time_value = sum(dead_time)/len(dead_time)
    print(f"Arduino's dead time is {dead_time_value} ms")

    for type in steps: 

        tau = tau_main[type] - dead_time_value
        pwm_array = np.linspace(0, len(tau)-1, len(tau)) + (1 if type == 'positive' else 0) # remember that when step is positive we read 1:255 and when is negative the range is 254:0


        # get parameters 
        coef_tau, _ = curve_fit(exp, pwm_array, tau, bounds=([0, -np.inf, 0],[np.inf, 0, np.inf]))
        print('tau_{}: y = {:.6f} e^( {:.6f} lux) + {:.6f}'.format(type, *coef_tau))


        plt.figure()
        plt.title(f"Tau according to PWM during {type} step")
        plt.plot(pwm_array, tau, marker='+', ls='-')
        plt.plot(pwm_array, exp(pwm_array, *coef_tau), 'r')
        plt.xlabel('PWM')
        plt.ylabel('Tau [ms]')
        plt.draw()

    breakpoint

if IMAGE: plt.show()