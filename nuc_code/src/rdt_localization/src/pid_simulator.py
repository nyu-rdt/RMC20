import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
#This library has not been installed
#Once it is installed, it should work since it was tested successfully
from mttkinter import mtTkinter as Tk
from pid_controller import PID
class PID_simulator:
    #PID simulator with tkinter & matplotlib. Code might have been taken here and there from stackoverflow & github
    def __init__(self,pid = None):
        self.closed = False
        self.pid = PID(1.2,1,0.001)
        if pid is not None:
            self.pid = pid
        #how many inputs are in the graph at a time
        self.x_interval = 30
        self.output = 0
        self.x = []
        self.y = []
        self.start = 0
        self.line = None
        self.root = Tk.Tk()
        self.root.title("Brought to you by Your Mom and co.")
        fig = plt.Figure()

        #bunch of labels & their position on the grid. play around to figure it out
        self.error_label = Tk.Label(self.root,text= "Your Mom")
        self.error_label.grid(column = 5, row = 0)
        p_label = Tk.Label(self.root,text= "P Constant").grid(column = 0, row = 0)
        i_label = Tk.Label(self.root,text= "I Constant").grid(column = 1, row = 0)
        d_label = Tk.Label(self.root,text= "D Constant").grid(column = 2, row = 0)
        pid_label = Tk.Label(self.root,text= "PID Setpoint").grid(column = 3, row = 0)
        #we only care about the text in the box. All other elements work on their own with Tkinter
        self.p_constant = Tk.Text(self.root,height = 2, width = 10, bg = "light yellow")
        self.p_constant.grid(column = 0, row = 1)
        self.i_constant = Tk.Text(self.root,height = 2, width = 10, bg = "light yellow")
        self.i_constant.grid(column = 1, row = 1)
        self.d_constant = Tk.Text(self.root,height = 2, width = 10, bg = "light yellow")
        self.d_constant.grid(column = 2, row = 1)
        self.sp = Tk.Text(self.root,height = 2, width = 10, bg = "light yellow")
        self.sp.grid(column = 3, row = 1)
    
        changePID = Tk.Button(self.root,text = "Change PID value", command = self.change_PID).grid(column = 1, row = 2)
    
        self.canvas = FigureCanvasTkAgg(fig,master = self.root)
        self.canvas.get_tk_widget().grid(column = 4, row = 3)
        #create a plot and put it into matplot's figure. 111 divides into 1 row & 1 column of plot
        #refers to the online doc. But yeah, we only need 1 plot.
        ax = fig.add_subplot(111)
        #set x and y axis. This can be put into variable in future sprint.
        ax.set_ylim([-180,180])
        ax.set_xlim([0,self.x_interval])
        #matplot automatically draw the line from a list of x and y values. line need to be redraw again and again
        self.line, = ax.plot(self.x, self.y)
        def on_closing():
            self.closed = True
            self.root.destroy()
        self.root.protocol("WM_DELETE_WINDOW", on_closing)
        #threading bad =(
        #set an animation with delay of 500 mili.
        #ani = FuncAnimation(fig,
         #               self.func_animate,interval=0, blit=False)
        #Tk.mainloop()
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
    def func_animate(self,feedback):
        if(len(self.y)<self.x_interval):
            self.x.append(self.start)
            self.start += 1
            self.output = self.pid.update(feedback)
            #uncomment this line when start getting feedback from actual env
            #self.feedback += self.output
            self.y.append(feedback)
        else:
            self.x = []
            self.y = []
            self.start = 0
            self.x.append(0)
            self.y.append(feedback)
        if not self.closed:
            #self.error_label.config(text = '%.3g'%(feedback-self.pid.SetPoint))
            self.line.set_data(self.x, self.y)
            self.canvas.draw()
            self.root.update()
    #update feedback here
    def get_output(self):
        return self.output
    def update_feedback(self,feedback):
        self.func_animate(feedback)
        return self.output