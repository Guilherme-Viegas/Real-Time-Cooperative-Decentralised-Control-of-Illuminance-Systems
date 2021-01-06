import numpy as np
from numpy.linalg import inv, eig
import matplotlib.pyplot as plt
import codecs
from sklearn.linear_model import LinearRegression
import math
from scipy.optimize import curve_fit

filename = 'tau_down.txt'

#Change filename according to data used
#If data is for step up, uncomment step up code down below, if for step down, uncomment step down code, and comment step up
#If we want delay function, it's in the final... I believe it's a constant function if we descard the 1s or 2s steps

#Aproximmate to a function of type y = a * e^kx which is y = a * b^x
def func(x, a, b, c):
    return a * np.exp(b * x) + c

def load_variables(name_file_x: str) -> (np.array, np.array, np.array):
    # load files
    f = codecs.open(name_file_x, "r", "utf-8")

    temp = f.read() #read file
    temp = temp.rsplit('\r\n') # split each line

    vo = [[],]
    times = [[],]
    pwm = []

    i = 0

    for line in temp:
        if line == 'STEP':
            i+=1
            vo.append([])
            times.append([])
            continue

        s = line.rsplit(' ')

        if(s[0] == 'PWM:'):
            pwm.append(s[1])
        else:
            vo[i].append(s[0])
            times[i].append(s[1])

    #vo = np.array(vo, dtype=np.intc) #int normal
    #times = np.array(times, dtype=np.int_) #long
    pwm = np.array(pwm, dtype=np.intc)

    return (vo, times, pwm)

def get_stable_vals(values, index):
    #return sum(float(values[index][-5:])) / 5
    sum = 0
    for i in range(1, 6):  #To get the sum of the last 5 numbers of x
        sum += float(values[index][-i])
    print("\n")
    return sum/5
    
#Returns the 63% of the constant value of vo
def get_63(max_val):
    return max_val * 0.63

#If PWM=255 means analog=710, and stable value of PWM=200 is 689, then the 63% is 710-0.63*(710-689)
#max_val is 710, and ref_val is 689
def get_63_down(max_val, ref_val):
    return max_val - 0.63 * (max_val - ref_val)

def compute_tau_up(x, y, index, val_at_63):
    for vo in range(1, len(x[index])):
        if((float(x[index][vo-1]) <= val_at_63) and (float(x[index][vo]) >= val_at_63)):
            #The we found the interval of vo(and time) of the 63% tau
            #Now we do some 3 simple rules to find tau
            vo_amplitude = float(x[index][vo]) - float(x[index][vo-1])
            time_amplitude = float(y[index][vo]) - float(y[index][vo-1])
            tmp = val_at_63 - float(x[index][vo-1])
            return ((tmp * time_amplitude)/vo_amplitude) + float(y[index][vo-1])
            break
    return -1

def compute_tau_down(x, y, index, val_at_63):
    for vo in range(1, len(x[index])):
        if((float(x[index][vo-1]) >= val_at_63) and (float(x[index][vo]) <= val_at_63)):
            #The we found the interval of vo(and time) of the 63% tau
            #Now we do some 3 simple rules to find tau
            vo_amplitude = float(x[index][vo]) - float(x[index][vo-1])
            time_amplitude = float(y[index][vo]) - float(y[index][vo-1])
            tmp = val_at_63 - float(x[index][vo-1])
            return ((tmp * time_amplitude)/vo_amplitude) + float(y[index][vo-1])
            break
    return -1


def get_lag(x, y):
    lag = []
    is_any_zero = 0
    for i in range(len(x)):
        for j in range(len(x[i])-1):
            if(float(x[i][j]) == 0 and float(x[i][j+1]) != 0):
                lag.append(float(y[i][j]))
                is_any_zero = 1
        if(is_any_zero == 0):
            lag.append(0)
        else:
            is_any_zero = 0
    return lag

#*******  Start of code ********
G = 0.05353
x,y,z = load_variables(filename)


#********* For Stepping Up **********
'''
lag = get_lag(x, y)

tau_vect = []
for step in range(1, len(x)):
    tau_vect.append(compute_tau_up(x, y, step, get_63(get_stable_vals(x, step)))) #returns tau in micros

final_tau_vect = []
for i in range(len(tau_vect)):
    final_tau_vect.append(tau_vect[i] - lag[i])


X = np.linspace(10, 255, 50)
X = X*G

plt.figure()
plt.plot(X, final_tau_vect, '+')


plt.xlabel('LUX')
plt.ylabel('Tau')


popt, pcov = curve_fit(func, X, final_tau_vect, bounds=([15000, -5, 0], [np.inf, 0, np.inf]))
plt.plot(X, func(X, *popt), 'g--')
print("y = ", popt[0], "*e^(", popt[1], "x) + ", popt[2])
plt.show()

'''
#************

#********* For Stepping Down ********


#lag = get_lag(x, y)

tau_vect = []
for step in range( len(x)):
    tau_vect.append(compute_tau_down(x, y, step, get_63_down(float(x[step][0]),get_stable_vals(x, step)))) #returns tau in micros


#Im not discarding the lag time, maybe do it later

X = np.linspace(255, 5, 51)
X = X*G

plt.figure()
plt.plot(X, tau_vect, lw=2)


plt.xlabel('PWM')
plt.ylabel('Tau')

print(tau_vect)

popt, pcov = curve_fit(func, X, tau_vect, bounds=([15000, -5, 0], [np.inf, 0, np.inf]))
plt.plot(X, func(X, *popt), 'g--')
print("y = ", popt[0], "*e^(", popt[1], "x) + ", popt[2])



plt.show()


#********* For the Dealy Function ************
'''
lag = get_lag(x, y)
print(lag)

X = np.linspace(5, 255, 51)

plt.figure()
plt.plot(X, lag, '+')


plt.xlabel('PWM')
plt.ylabel('Lag')

plt.show()

#If delay function is not dependent on input, then is a constant, so we average lag

print(sum(lag) / len(lag))  #We got for our values 369.09 micro seconds

'''