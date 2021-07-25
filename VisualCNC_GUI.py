import tkinter as tk


LARGE_FONT= ("Verdana", 12)

#% Import dependencies
import time
import os
os.chdir("C:/Users/aleks/Desktop/DTU/Speciale - Visual CNC/Python_Code")
from Library import *
os.chdir("C:/Users/aleks/Desktop/DTU/Speciale - Visual CNC/Tests")

def runMain():
    os.chdir("C:/Users/aleks/Desktop/DTU/Speciale - Visual CNC/Python_Code")
    with open("Main Script.py","r") as rnf:
        exec(rnf.read())
    os.chdir("C:/Users/aleks/Desktop/DTU/Speciale - Visual CNC/Tests")


class SeaofBTCapp(tk.Tk):

    def __init__(self, *args, **kwargs):
        
        tk.Tk.__init__(self, *args, **kwargs)
        container = tk.Frame(self)

        container.pack(side="top", fill="both", expand = True)

        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}

        for F in (StartPage, MCQ):

            frame = F(container, self)

            self.frames[F] = frame

            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(StartPage)

    def show_frame(self, cont):

        frame = self.frames[cont]
        frame.tkraise()

        
class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self,parent)
        label = tk.Label(self, text="Welcome to VisualCNC", font=LARGE_FONT)
        label.pack(pady=5,padx=5)

        label2 = tk.Label(self, text="Before use, prepare CNC bed by mounting the \n material with a drawn contour safely onto the CNC bed. \n Mount camera before beginning the acquisition process. \n Remove camera before streaming G-code")        
        label2.pack(pady=5, padx=5)

        button = tk.Button(self, text="Acquire and stitch",
                            command=lambda: acquireAndStitch(stepsize_x, stepsize_y, gridsize_x, gridsize_y, 3, 3, acquire=True))
        button.pack(pady=5, padx=5)

        
        button2 = tk.Button(self, text="Image file to Gcode",
                            command=lambda: runMain())
        button2.pack(pady=5, padx=5)
        
        button3 = tk.Button(self, text="Stream Gcode to CNC machine",
                            command=lambda: gcodeStream())
        button3.pack(pady=5, padx=5)

        button3 = tk.Button(self, text="Exit",
                            command=lambda: controller.destroy())
        button3.pack(pady=5, padx=5)
        

class MCQ(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="MCQ test!!!", font=LARGE_FONT)
        label.pack(pady=10,padx=10)
        label.after(1000, controller.show_frame(StartPage))

    def correspondingBehavior(self,choice):
        print(choice)
    


app = SeaofBTCapp()
app.mainloop()