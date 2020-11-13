import codecs
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

UP = True

def exp(x, a, b, c):
    return a * np.exp(b * x) + c

def load_variables(name_file_x: str) -> (np.array,np.array):
    f = codecs.open(name_file_x, "r", "utf-8")

    temp = f.read() # read file
    temp = temp.rsplit('\r\n')
    
    analog = [[],]
    time_arr = [[],]
    pwm = []
    i = 0;
    counter = 0;
    flag = 0;
    for line in temp:
        s = line.rsplit(" ")
        if s[0]=='PWM:':
            pwm.append(float(s[1]))
            if counter > 0:
                analog.append([])
                time_arr.append([])
                i+=1
            counter+=1
            continue
        elif s[0]=='-1':
            
            continue
        
        analog[i].append(s[0])
        time_arr[i].append(s[1])
            
        
    
    #analog = np.array(analog, dtype=np.float64)
    #time_arr = np.array(time_arr, dtype=np.float64)

    return (analog,time_arr,pwm)

def get63(max_array):
    return max_array*0.63

def get_stable_vals(values):
    sum = 0
    i = 0
    while i < 6:  #To get the sum of the last 5 numbers of x
        sum += float(values[len(values)-1-i])
        i+=1
    return sum/6

def computeTau(analog, time_arr):
    val_63 = get63(get_stable_vals(analog))
    i=1
    while  i  < len(analog):
        if (float(analog[i-1]) <= val_63) and (float(analog[i]) >= val_63):
            analog_amp = float(analog[i])-float(analog[i-1])
            time_arr_amp = float(time_arr[i])-float(time_arr[i-1])
            aux = val_63 - float(analog[i-1])
            return ((aux*time_arr_amp)/analog_amp) + float(time_arr[i-1])
        i+=1
    return -1

def computeTauDown(analog, time_arr):
    stable = get_stable_vals(analog)
    maximize = float(analog[0])
    interval = maximize-stable
    val_37 = interval*0.37
    i = 1
    while i < len(analog):
        if (float(analog[i-1]) >= val_37+stable) and (float(analog[i]) <= val_37+stable):
            analog_amp = float(analog[i])-float(analog[i-1])
            time_arr_amp = float(time_arr[i])-float(time_arr[i-1])
            slope = time_arr_amp/analog_amp
            b = float(time_arr[i])-slope*float(analog[i])
            point = val_37+stable
            return slope*point+b
        i+=1
    return 1
    
def createArrayTau(analog, time_arr):
    i = 0
    tau = []
    while i<len(analog):
        tau.append(computeTau(analog[i], time_arr[i]))
        i+=1
    
    return tau

def createArrayTauDown(analog, time_arr):
    i = 1
    tau = []
    while i<len(analog):
        tau.append(computeTauDown(analog[i], time_arr[i]))
        i+=1
    
    return tau

if UP:
    analog,time_arr,pwm = load_variables("new_b.txt")
    tau = createArrayTau(analog, time_arr)
    tau = tau[1:]
    pwm = pwm[1:]
    pwm = np.array(pwm)
    tau = np.array(tau)
    coef_tau, _ = curve_fit(exp, pwm, tau, bounds=([0, -np.inf, 0],[np.inf, 0, np.inf]))
    print('tau_{}: y = {} e^( {} x) + {}'.format(type, *coef_tau))
    
    
    plt.figure()
    plt.plot(pwm,tau, '+')
    plt.plot(pwm, exp(pwm, *coef_tau), 'r')
    plt.xlabel('PWM')
    plt.ylabel('Tau(micro s)')
    plt.show()

