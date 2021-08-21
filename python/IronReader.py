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
        fig=plt.figure(figsize=(10,6))
        ax=fig.add_axes([0.05,0.05,0.9,0.9])
        canvas=FigureCanvasTkAgg(fig,master=root)
        canvas.get_tk_widget().grid(row=1,column=0, columnspan=5)
        canvas.draw()
        
        toolbarFrame = tk.Frame(master=root)
        toolbarFrame.grid(row=0,column=0, sticky='SW', columnspan=3)
        toolbar = NavigationToolbar2Tk(canvas, toolbarFrame)


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

        self.after(100, self.plot, canvas, ax)

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

    def plot(self,canvas,ax):
        data = update_data()
        if (self.update):
            ax.clear()
            ax.plot(data[:,0], data[:,1])
            ax.set_ylim(0, 300)
            ax.grid(b=True, axis='both', which='major')
            ax.set_title(dt_string + ' ' + self.figtitle.get())
            canvas.draw()
        self.after(1000, self.plot, canvas, ax)
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
