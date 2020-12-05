import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
from matplotlib.figure import Figure
import tkinter as Tk
from pid_controller import PID
class PID_simulator:
    #PID simulator with tkinter & matplotlib. Code might have been taken here and there from stackoverflow & github
    def __init__(self,pid = None):
        self.output = 0
        self.pid = PID(1.2,1,0.001)
        if pid is not None:
            self.pid = pid
        self.x = []
        self.y = []
        self.start = time.time()
        self.line = None
        self.feedback = 0
        root = Tk.Tk()
        root.title("Brought to you by Your Mom and co.")
        fig = plt.Figure()

        #bunch of labels & their position on the grid. play around to figure it out
        self.error_label = Tk.Label(root,text= "Your Mom")
        self.error_label.grid(column = 5, row = 0)
        p_label = Tk.Label(root,text= "P Constant").grid(column = 0, row = 0)
        i_label = Tk.Label(root,text= "I Constant").grid(column = 1, row = 0)
        d_label = Tk.Label(root,text= "D Constant").grid(column = 2, row = 0)
        pid_label = Tk.Label(root,text= "PID Setpoint").grid(column = 3, row = 0)
        #we only care about the text in the box. All other elements work on their own with Tkinter
        self.p_constant = Tk.Text(root,height = 2, width = 10, bg = "light yellow")
        self.p_constant.grid(column = 0, row = 1)
        self.i_constant = Tk.Text(root,height = 2, width = 10, bg = "light yellow")
        self.i_constant.grid(column = 1, row = 1)
        self.d_constant = Tk.Text(root,height = 2, width = 10, bg = "light yellow")
        self.d_constant.grid(column = 2, row = 1)
        self.sp = Tk.Text(root,height = 2, width = 10, bg = "light yellow")
        self.sp.grid(column = 3, row = 1)
    
        changePID = Tk.Button(root,text = "Change PID value", command = self.change_PID).grid(column = 1, row = 2)
    
        canvas = FigureCanvasTkAgg(fig,master = root)
        canvas.get_tk_widget().grid(column = 4, row = 3)
        #create a plot and put it into matplot's figure. 111 divides into 1 row & 1 column of plot
        #refers to the online doc. But yeah, we only need 1 plot.
        ax = fig.add_subplot(111)
        #set x and y axis. This can be put into variable in future sprint.
        ax.set_ylim([-180,180])
        ax.set_xlim([0,30])
        #matplot automatically draw the line from a list of x and y values. line need to be redraw again and again
        self.line, = ax.plot(self.x, self.y)

        #set an animation with delay of 500 mili.
        ani = FuncAnimation(fig,
                        self.func_animate,interval=20, blit=False)
        Tk.mainloop()
    #callback function for the button. Triggered when button pressed. 
    #Every element is changed when button pressed. Bunch of try except in case of dumb/blank input
    def change_PID(self):
        pconst = self.p_constant.get("1.0",'end-1c')
        try:
            p_c = float(pconst)
            self.pid.setKp(p_c)
        except:
            pass
        iconst = self.i_constant.get("1.0",'end-1c')
        try:
            i_c = float(iconst)
            self.pid.setKi = i_c
        except:
            pass
        dconst = self.d_constant.get("1.0",'end-1c')
        try:
            d_c = float(dconst)
            pid.setKd = d_c
        except:
            pass
        setpoint = self.sp.get("1.0",'end-1c')
        try:
            s_p = float(setpoint)
            self.pid.SetPoint = s_p
        except:
            pass
    #callback function for the FuncAnimation. I have no idea what i does (interval maybe)
    def func_animate(self,i):
        if(len(self.y)<60):
            self.x.append((time.time()-self.start)*25)
            self.output = self.pid.update(self.feedback) 
            #uncomment this line when start getting feedback from actual env
            #self.feedback += self.output
            self.y.append(self.feedback)
        else:
            self.x.clear()
            self.y.clear()
            self.start = time.time()
            self.x.append(0)
            self.y.append(self.feedback)
        self.error_label.config(text = '%.3g'%(self.feedback-self.pid.SetPoint))
        self.line.set_data(self.x, self.y)
        return self.line,
    #update feedback here
    def get_output(self):
        return self.output
    def update_feedback(self,feedback):
        self.feedback = feedback
a = PID_simulator()