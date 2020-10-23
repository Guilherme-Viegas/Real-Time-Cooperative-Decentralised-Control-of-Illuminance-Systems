import numpy as np
from numpy.linalg import inv, eig
import matplotlib.pyplot as plt
import codecs
from sklearn.linear_model import LinearRegression

filename = 'tau.txt'

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
    for i in range(6):  #To get the sum of the last 5 numbers of x
        sum += float(values[index][-i])
    return sum/5
    
#Returns the 63% of the constant value of vo
def get_63(max_val):
    return max_val * 0.63

def compute_tau(x, y, index, val_at_63):
    for vo in range(1, len(x[index])):
        #print(float(x[index][vo-1]), " <= ", val_at_63, " <= ", float(x[index][vo]))
        if((float(x[index][vo-1]) <= val_at_63) and (float(x[index][vo]) >= val_at_63)):
            #The we found the interval of vo(and time) of the 63% tau
            #Now we do some 3 simple rules to find tau
            vo_amplitude = float(x[index][vo]) - float(x[index][vo-1])
            time_amplitude = float(y[index][vo]) - float(y[index][vo-1])
            tmp = val_at_63 - float(x[index][vo-1])
            return ((tmp * time_amplitude)/vo_amplitude) + float(y[index][vo-1])
            break
    return -1



x,y,z = load_variables(filename)

tau_vect = []
for step in range(len(x)):
    tau_vect.append(compute_tau(x, y, step, get_63(get_stable_vals(x, step)))) #returns tau in micros

print(tau_vect)

plt.figure()
plt.plot(tau_vect, lw=2)

X = np.linspace(5, 255, 5)
Y = np.linspace(tau_vect[0], tau_vect[-1], 1)

plt.xlabel('PWM / 5')
plt.ylabel('Tau')
plt.show()

#print(x[6])
#print("\n\n")
#print(y[6])
#print("\n\n")
#print(compute_tau(x, y, 6, get_63(get_stable_vals(x, 6)))) #returns tau in micros
#print(int(x[6][3]))
#print("\n\n")
#print(y)

