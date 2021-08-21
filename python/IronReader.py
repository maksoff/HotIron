import numpy as np
import serial
import serial.tools.list_ports

import re
import os

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
import tkinter as tk
import tkinter.ttk as ttk
import sys

from idlelib.ToolTip import ToolTip as tt

from datetime import datetime

## search port
ports = serial.tools.list_ports.comports()
print('*** Found ports ***')
for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))

print('')
print('*** Auto-detecting port ***')
port = None
for p, _, _ in sorted(ports):
    print('Checking {}'.format(p))
    ser = serial.Serial(p, 115200, timeout=2)
    s = ser.readline()
    if (s):
        #first line read, but it can be only partially read, so ignore it
        s = ser.readline() # reading second one
        if (s.startswith(b'Tick:')):
            port = p
            break
    ser.close()

if (port):
    print('> Port {} looks great!'.format(port))
else:
    print('Nothing found')
    # TODO quit
## help functions

data = None
start_tick = 0
dt_string = ''
dt_file = ''

def parse_data(line):
    return [float(x) for x in line.replace(';', '').replace(':', '').split(' ') if not x[0].isalpha()]

def clear_data():
    global data, dt_string, dt_file
    data = None
    dt_string = datetime.now().strftime("%Y.%m.%d %H:%M:%S")
    dt_file = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

def update_data():
    global data, start_tick
    s = ser.readline().decode()[:-2]
    print(s)
    if data is None:
        clear_data() #update time
        data = np.array([parse_data(s)])
        start_tick = data[0,0]
        data[0,0] = 0
    else:
        data = np.append(data, [parse_data(s)], axis=0)
        data[-1,0] -= start_tick
    return data



class Application(tk.Frame):
    update = True
    def __init__(self, master=None):
        tk.Frame.__init__(self,master)
        self.createWidgets()
        

    def createWidgets(self):
        self.fig, (self.a0, self.a1) = plt.subplots(2, 1, sharex=True, gridspec_kw={'height_ratios': [2, 1]})
        self.fig.set_size_inches(12, 7)
        self.fig.set_dpi(100)
        self.fig.subplots_adjust(left=0.07, right=0.95, bottom=0.07, top=0.95, hspace=0.03)

        self.canvas=FigureCanvasTkAgg(self.fig,master=root)
        self.canvas.get_tk_widget().grid(row=1,column=0, columnspan=5)
        self.canvas.draw()
        
        toolbarFrame = tk.Frame(master=root)
        toolbarFrame.grid(row=0,column=0, sticky='SW', columnspan=3)
        toolbar = NavigationToolbar2Tk(self.canvas, toolbarFrame)


        self.figtitle = tk.Entry(master=root)
        self.figtitle.grid(row=0, column=3, sticky='WE')
        tt(self.figtitle, 'Graph title' )

        butframe = tk.Frame(root)
        butframe.grid(row=0, column =3, columnspan=2, sticky='E')

        self.plotbutton=tk.Button(butframe, text="pause", fg='red', command=self.en_update)
        self.plotbutton.pack(side=tk.RIGHT)
        tt(self.plotbutton, 'Pause the graph update\n(data will be received\nin the background)')

        
        self.clearbutton=tk.Button(butframe, text="clear", bg='red', command=self.clear)
        self.clearbutton.pack(side=tk.RIGHT)
        tt(self.clearbutton, 'Received data will be deleted')
        
        self.savebutton=tk.Button(butframe, text="save", command=self.save)
        self.savebutton.pack(side=tk.RIGHT)
        tt(self.savebutton, 'Save the plot')

        ser.flushInput()
        ser.readline() # clear data and wait end of string

        self.after(100, self.plot)

    def en_update(self):
        self.update = not self.update
        if self.update:
            self.plotbutton.config(text = 'pause', fg='red')
        else:
            self.plotbutton.config(text = 'pause', fg = 'blue')

    def clear(self):
        if not self.update:
            self.en_update()
        clear_data()

    def save(self):
        filename = dt_file+'_' + re.sub(r'[\\/*?:"<>| ]','_',self.figtitle.get()) + '.png'
        plt.savefig(filename)
        os.startfile(filename)

    def plot(self):
        data = update_data()
        if (self.update):
            self.a0.clear()
            self.a0.plot(data[:,0], data[:,1], color='blue', label='Measured T')
            self.a0.plot(data[:,0], data[:,2], ':', color='red', label='Target T')
            self.a0.set_ylim(0, 300)
            self.a0.legend(loc=1)
            self.a0.grid(b=True, axis='both', which='major')
            self.a0.grid(b=True, axis='both', which='minor', linewidth=0.3, linestyle=':')
            self.a0.set_ylabel('Temperature')
            self.a0.set_title(dt_string + ' ' + self.figtitle.get())

            self.a0.yaxis.set_major_locator(MultipleLocator(50))
            self.a0.yaxis.set_minor_locator(MultipleLocator(10))

            self.a1.clear()
            self.a1.plot(data[:,0], data[:,3], lw='2', label='Power')
            self.a1.plot(data[:,0], data[:,4], 'k--', label='P', alpha=0.7)
            self.a1.plot(data[:,0], data[:,5], 'b--', label='I', alpha=0.7)
            self.a1.plot(data[:,0], data[:,6], 'r--', label='D', alpha=0.7)
            self.a1.set_ylim(-30, 105)
            self.a1.grid(b=True, axis='both', which='major', linestyle='-')
            self.a1.set_ylabel('Power')
            self.a1.legend(loc=1)
                        
            self.a1.set_xlabel('Time')
            
            self.canvas.draw()
        self.after(700, self.plot)
        #here set axes

def on_closing():
    root.destroy()
    root.quit()            

root=tk.Tk()
root.title('Hot Iron temperature measurement ({})'.format(port))
root.protocol("WM_DELETE_WINDOW", on_closing)
app=Application(master=root)
root.resizable(False, False)
app.mainloop()
print('window is closed')
