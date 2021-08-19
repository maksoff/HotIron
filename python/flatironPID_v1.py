import matplotlib
import matplotlib.pyplot as plt
import numpy as np

import copy

class IRON():
    """
T0 - medium temperature
L - Thermo lambda
"""
    def __init__(self, T0=25, L=0.8, C=500, lc=1, tlc=1, ttlc=1, dt = 1):
        self.T0 = T0
        self.T = T0 # measured temperature
        self.Ti = T0 #internal temperature
        self.Tout = T0 #air temperature
        self.L = L
        self.C = C
        self.lc = lc
        self.dt = dt
        self.tlc=tlc
        self.ttlc = ttlc
        self.time = [0]
        self.history = [self.T0]
        self.historyTi = [self.T0]
        self.historydTi = [self.T0]
        self.historydT = [0]
        self.power = [0]
        self.dT = [0]
    def step(self, P):
        #dTi = (P - (self.Ti - self.T0)*self.L)/self.C * self.dt
        dTi = (P - (self.Ti - self.Tout)*self.L)/self.C * self.dt
        self.Tout += (self.Ti - self.Tout)*self.tlc - (self.Tout-self.T0)*self.ttlc
        self.Ti += dTi
        if self.lc == 0:
            dT = dTi
        else:
            dT = (self.Ti - self.T) * self.lc * self.dt
        self.T += dT
        self.time.append(self.time[-1] + self.dt)
        self.history.append(self.T)
        self.historyTi.append(self.Ti)
        self.historydT.append(dT + np.random.randint(-4, +4)/8)
        self.historydTi.append(dTi)
        self.power.append(P)
        return self.T
    def step_time(self, P, time):
        for i in range(int(time/self.dt)):
            self.step(P)

class PID():
    integral = 0
    last_value = -1
    def __init__(self, P = 1, I = 0, D = 0, limit_int = 10000,
                 limit_int_bot=0, limit_bot=0, limit_top=100,
                 boost=False, rnd=True):
        self.P = int(P*32768)
        self.I = int(I*32768)
        self.D = int(D*32768)
        self.time = [0]
        self.historyERR = [0]
        self.historyPID = [(0, 0, 0)]
        self.historyOUT = [0]
        self.limit_bot = limit_bot
        self.limit_top = limit_top
        self.limit_int = int(limit_int*4)
        self.limit_int_bot = limit_int_bot
        self.limit_d = -1e7
        self.boost=boost
        self.rnd = rnd
        #self.gen = np.random.RandomState()
    def pid(self, value, setpoint, dt=1):
        class dFIL:
            size = 4
            arr = [0]*size
            def __init__(self, val):
                self.arr=[val]*self.size
            def fil(self, val): #emulate c function
                temp = self.arr[0]
                for i in range(1, self.size):
                    self.arr[i-1] = self.arr[i]
                self.arr[self.size-1] = val
                return int((val - temp)/self.size)
            
        value *= 4
        if self.rnd:
            value += np.random.randint(-4, +4)
        setpoint *= 4
        if self.last_value < 0: # we are here first time
            self.last_value = value
            self.last_sp = setpoint
            self.time_to_boost = 0
            self.fil = dFIL(value)

        #value += self.gen.randn()/10
        error = -(value - setpoint)
        dataP = error * self.P
        if (error > 0):
            self.integral += error
            if (error < 3*4):
                self.integral +=50
        else:
            self.integral += error/4
            if (error > -3*4):
                self.integral -= 25
        if self.integral > self.limit_int:
            self.integral = self.limit_int
        if self.integral < self.limit_int_bot:
            self.integral = self.limit_int_bot
        dataI = self.integral * self.I
        #dataD = -(value - self.last_value) * self.D
        dataD = -self.fil.fil(value)*self.D

        if dataD > 0:
            dataD = 0
        if dataD < self.limit_d:
            dataD = self.limit_d
        out_val = dataP + dataI + dataD

        self.time.append(self.time[-1] + dt)
        self.historyERR.append(value-setpoint)
        self.historyPID.append((dataP, dataI, dataD))

        out_val /= 4*32768
        if out_val > self.limit_top:
            out_val = self.limit_top
        if out_val < self.limit_bot:
            out_val = self.limit_bot
    
        self.last_value = value
        
        if self.boost and self.last_sp < setpoint:
            self.time_to_boost = int((setpoint-value)*2/3.7/4)
            print(self.time_to_boost)
        self.last_sp = setpoint
        
        if self.time_to_boost > 0:
            self.time_to_boost -=1
            out_val = self.limit_top/2
        
        out_val = int(out_val)
        self.historyOUT.append(out_val)
        return out_val
    
class _PID():
    integral = 0
    last_value = 0
    def __init__(self, P = 1, I = 0, D = 0, limit_int = 10000,limit_int_bot=0, limit_bot=0, limit_top=100):
        self.P = P
        self.I = I
        self.D = D
        self.time = [0]
        self.historyERR = [0]
        self.historyPID = [(0, 0, 0)]
        self.historyOUT = [0]
        self.limit_bot = limit_bot
        self.limit_top = limit_top
        self.limit_int = limit_int
        self.limit_int_bot = limit_int_bot
        #self.gen = np.random.RandomState()
    def pid(self, value, setpoint, dt=1):
        #value += self.gen.randn()/10
        error = -(value - setpoint)
        dataP = error * self.P
        self.integral += error
        if self.integral > self.limit_int:
            self.integral = self.limit_int
        if self.integral < self.limit_int_bot:
            self.integral = self.limit_int_bot
        dataI = self.integral * self.I
        dataD = (value - self.last_value) * self.D

        if dataD > self.limit_int:
            dataD = self.limit_int
        if dataD < -self.limit_int:
            dataD = -self.limit_int
        self.last_value = value
        out_val = dataP + dataI - dataD

        self.time.append(self.time[-1] + dt)
        self.historyERR.append(value-setpoint)
        self.historyPID.append((dataP, dataI, dataD))

        if out_val > self.limit_top:
            out_val = self.limit_top
        if out_val < self.limit_bot:
            out_val = self.limit_bot
        out_val = int(out_val)
        self.historyOUT.append(out_val)
        return out_val


def datestr2num (x):
    x = x.decode().split(':')
    return int(x[0])*3600+int(x[1])*60+float(x[2])

def compare_with_real():

    #meas = np.loadtxt(r"D:\20210808085626_temp_set_100proc.txt", delimiter=' ', converters = {0: datestr2num})
    #meas = np.loadtxt(r"D:\20210808095219_temp_50proc.txt", delimiter=' ', converters = {0: datestr2num})
    meas = np.loadtxt(r"D:\20210808103327_temp_30proc.txt", delimiter=' ', converters = {0: datestr2num})
    meas -= np.array([meas[0][0], 0, 0])

    T0 = meas[0][1]
    delay = 0
    while meas[delay][2] == 0:
        delay+=1
    delay = meas[delay][0]

    tlimit = 0

    for i in range(1, len(meas)):
        if meas[i-1][2] > 0 and 0 == meas[i][2]:
            tlimit = meas[i-1][1]

    print("T0: {}, delay: {}, Tlimit: {}".format(T0, delay, tlimit))

    C=500 #500
    L=1.3 #0.85
    lc=0.05 #0.05
    tlc=0.3
    ttlc=0.9
    P=1850*.3 #1600

    tec = IRON(T0 = T0, C=C, L=L, lc = lc, tlc=tlc, ttlc=ttlc)
    tec.step_time(0, delay)
    while tec.T < tlimit:
        tec.step(P)
    while tec.time[-1] < max(meas[:, 0]):
        tec.step(0)

    fig, axs = plt.subplots(1, 1, sharex=True)
    fig.set_size_inches(12, 7)
    fig.set_dpi(100)
    fig.subplots_adjust(left=0.07, right=0.95, bottom=0.07, top=0.95, hspace=0.03)
    axs.plot(tec.time, tec.history, '--', color='blue')
    axs.plot(tec.time, tec.historyTi, ':', color='orange')
    #axs[1].plot(tec.time, tec.power)
    #axs[2].plot(tec.time, tec.historydT)
    axs.plot(meas[:, 0], meas[:, 1], color='red')
    axs.grid(b=True, axis='both', which='major', linestyle='-')
    #axs[1].grid(b=True, axis='both', which='major', linestyle='-')
    #axs[2].grid(b=True, axis='both', which='major', linestyle='-')
    ax2 = axs.twinx()
    ax2.plot(meas[:,0], meas[:, 2] / max(meas[:,2])*max(tec.power), color='red', linestyle='--', lw=0.5)
    ax2.plot(tec.time, tec.power, color='blue', linestyle='--', lw=0.5)
    axs.axhline(222, ls=':')
    plt.show()

def do_pid(pidnr=0):
    C=500 #500
    L=1.3 #1.3
    lc=0.05 #0.05
    tlc=0.3 #0.3
    ttlc=0.9 #0.9
    P=1850 #1600
    T0 = 28

    
    pid = PID(P=1, I=0.00153, D=15, limit_top=100, limit_int_bot=0, rnd=True)
    if pidnr==1:
        #pid = PID(P=.5, I=0.001, D=0, limit_top=100, limit_int_bot=0, limit_int=20000)
        pid = PID(P=1, I=0.0015, D=10, limit_top=100, limit_int_bot=0, limit_int=20000)
    if pidnr==3:
        pid = PID(P=1, I=0.0017, D=15, limit_top=100, limit_int_bot=0, boost=True)
        #pid = PID(P=3, I=0.003, D=100, limit_top=100, limit_int_bot=0, limit_int=20000)
    #pid = PID(P=1, I=0.000, D=0, limit_top=100) 

    temperature = T0
    tec = IRON(T0 = temperature, C=C, L=L, lc = lc, tlc=tlc, ttlc=ttlc)

    if pidnr!=2:
        temp = [T0]*60+[150]*400+[160]*400+[222]*600+[200]*600+[330]*600+[0]*600
        temp = [T0]*60+[270]*600+[150]*600+[222]*600+[0]*600
        #temp = [T0]*60+[100]*600+[190]*600+[200]*600+[0]*600
        #temp = [T0]*60+[100]*600+[110]*600+[190]*600+[200]*600+[0]*600
        for t in temp:
            power = int(pid.pid(temperature, t))/100
            temperature = tec.step(P*power)
    else:
        temp = []
        t = temperature
        print("holding {} grad".format(t))
        for i in range(50): 
            power = int(pid.pid(temperature, t))/100
            temperature = tec.step(P*power)
            temp.append(t)
            
        t = 150
        print("going to {} grad".format(t))
        while temperature < t - 5:
            power = int(pid.pid(temperature, t))/100
            temperature = tec.step(P*power)
            temp.append(t)

        ann_preheat = len(temp), t

        print("holding {} grad".format(t))
        for i in range(100): # hold 100 sec
            power = int(pid.pid(temperature, t))/100
            temperature = tec.step(P*power)
            temp.append(t)
            
        
        t = 220
        print("going to {} grad".format(t))
        while not t - 2 <temperature < t + 2:
            power = int(pid.pid(temperature, t))/100
            temperature = tec.step(P*power)
            temp.append(t)
            
        ann_reflow = len(temp), t
        
        print("holding {} grad".format(t))    
        for i in range(30): # hold 30 sec
            power = int(pid.pid(temperature, t))/100
            temperature = tec.step(P*power)
            temp.append(t)
            
        ann_cooldown = len(temp), t

        t = 0
        print("going to {} grad".format(t))
        for i in range(1800): # cooldown
            power = int(pid.pid(temperature, t))/100
            temperature = tec.step(P*power)
            temp.append(t)
            if temperature < 100:
                break
        
            
        
    fig, axs = plt.subplots(3, 1, sharex=True)
    fig.set_size_inches(12, 7)
    fig.set_dpi(100)
    fig.subplots_adjust(left=0.07, right=0.95, bottom=0.07, top=0.95, hspace=0.03)
    axs[0].plot(tec.time, tec.history, color='blue', label='Measured T')
    axs[0].plot(tec.time, tec.historyTi, ':', color='orange', label='Internal T')
    axs[0].plot(tec.time, [0]+temp, ':', color='red', label='Target T')
    if pidnr == 2:
        axs[0].annotate('preheat', ann_preheat, xytext = (ann_preheat[0]-20,ann_preheat[1]-50), arrowprops=dict(arrowstyle="->"))
        axs[0].annotate('reflow', ann_reflow, xytext = (ann_reflow[0]-30,ann_reflow[1]-50), arrowprops=dict(arrowstyle="->"))
        axs[0].annotate('cooldown', ann_cooldown, xytext = (ann_cooldown[0]+10,ann_cooldown[1]-50), arrowprops=dict(arrowstyle="->"))
    axs[1].plot(tec.time, pid.historyOUT, lw='2', alpha=0.7, label='Power')
    axs[1].plot(tec.time, [t[0]/32768/4 if t[0] > 0 else 0 for t in pid.historyPID], 'k:', label='P')
    axs[1].plot(tec.time, [t[1]/32768/4 for t in pid.historyPID], 'b:', label='I')
    axs[1].plot(tec.time, [t[2]/32768/4 for t in pid.historyPID], 'r:', label='D')
    axs[1].legend(loc=1)
    if 1:
        axs[2].plot(tec.time, tec.historydT)
        axs[2].set_ylabel('dT/dt')
    else:
        axs[2].plot(tec.time, [t[1] for t in pid.historyPID], 'b', label='I')
        axs[2].set_ylabel('Integral')
        ax3 = axs[2].twinx()
        ax3.plot(tec.time, [t[2] for t in pid.historyPID], 'r', label='D')
        ax4 = axs[2].twinx()
        ax4.plot(tec.time, [t[0] if t[0]>-1000000 else -1000000 for t in pid.historyPID], 'k', label='P')


    axs[0].grid(b=True, axis='both', which='major', linestyle='-')
    axs[1].grid(b=True, axis='both', which='major', linestyle='-')
    axs[2].grid(b=True, axis='both', which='major', linestyle='-')

    axs[0].legend(loc=1)
    
    axs[0].set_ylabel('Temperature')
    axs[1].set_ylabel('Power')
    #axs[2].plot(tec.time, pid.historyPID)
    #ax2 = axs.twinx()
    #ax2.plot(meas[:,0], meas[:, 2] / max(meas[:,2])*max(tec.power), color='red', linestyle='--', lw=0.5)
    #ax2.plot(tec.time, tec.power, color='blue', linestyle='--', lw=0.5)
    #axs[0].axhline(222, ls=':')
    axs[2].set_xlabel('Time (s)')
    

if __name__ == "__main__":
    #compare_with_real()
    do_pid()
    do_pid(2)
    do_pid(5)
    plt.show()
