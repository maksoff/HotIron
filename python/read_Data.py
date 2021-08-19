import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import glob


files = glob.glob('*')
for i, f in enumerate(files):
    print("{}: f {}".format(i, f))
n = int(input('Nr: '))
file = files[n]

fd=open(file,"r")
d=fd.read()
fd.close()
m=d.split("\n")
s="\n".join(m[:-5])
fd=open(file,"w+")
for i in range(len(s)):
    fd.write(s[i])
fd.close()


data = np.loadtxt(file, skiprows=3, usecols=(1, 3, 5, 7, 9, 11, 13),
                  converters={1: lambda x: int(x[:-1]),
                              3: lambda x: float(x[:-1]),
                              5: lambda x: int(x[:-1]),
                              7: lambda x: int(x[:-1]),
                              9: lambda x: int(x[:-1]),
                              11: lambda x: int(x[:-1]),
                              13: lambda x: int(x),})

for i in range(1, len(data)):
    if data[i][0] < data[i-1][0]:
        data[i][0] = data[i-1][0]+1

fig, axs = plt.subplots(3, 1, sharex=True)
fig.set_size_inches(12, 7)
fig.set_dpi(100)
fig.subplots_adjust(left=0.07, right=0.95, bottom=0.07, top=0.95, hspace=0.03)
axs[0].plot(data[:,0], data[:,1], color='blue', label='Measured T')
axs[0].plot(data[:,0], data[:,2], ':', color='red', label='Target T')

axs[1].plot(data[:,0], data[:,3], lw='2', alpha=0.7, label='Power')
axs[1].plot(data[:,0], [d if d > -10 else -10 for d in data[:,4]], 'k:', label='P')
axs[1].plot(data[:,0], data[:,5], 'b:', label='I')
axs[1].plot(data[:,0], data[:,6], 'r:', label='D')
axs[1].legend(loc=1)

dT = [0]
for i in range(1, len(data)):
    dT.append(data[i][1]-data[i-1][1])

axs[2].plot(data[:,0], dT)
axs[2].set_ylabel('dT/dt')
axs[0].grid(b=True, axis='both', which='major', linestyle='-')
axs[1].grid(b=True, axis='both', which='major', linestyle='-')
axs[2].grid(b=True, axis='both', which='major', linestyle='-')

axs[0].legend(loc=1)

axs[0].set_ylabel('Temperature')
axs[1].set_ylabel('Power')
axs[2].set_xlabel('Time (s)')
plt.show()

