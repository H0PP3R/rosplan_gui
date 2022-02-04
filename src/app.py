from tkinter import Tk, Button, Checkbutton, ttk
from components.collapsiblePane import CollapsiblePane as cp
from components.toggledFrame import ToggledFrame
from styling import *

class App():
  def __init__(self):
    root = Tk()
    root.geometry(f'{ROOT_SIZE[0]}x{ROOT_SIZE[1]}')

    self.toggleFrame(root)
    # self.collapsiblePane(root)

    root.mainloop()
  
  def toggleFrame(self, parent):
    t = ToggledFrame(parent, text='Rotate', relief="raised", borderwidth=1)
    t.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")

    ttk.Label(t.sub_frame, text='Rotation [deg]:').pack(side="left", fill="x", expand=1)
    ttk.Entry(t.sub_frame).pack(side="left")

    t2 = ToggledFrame(parent, text='Resize', relief="raised", borderwidth=1)
    t2.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")

    for i in range(10):
        ttk.Label(t2.sub_frame, text='Test' + str(i)).pack()

    t3 = ToggledFrame(parent, text='Fooo', relief="raised", borderwidth=1)
    t3.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")

    for i in range(10):
        ttk.Label(t3.sub_frame, text='Bar' + str(i)).pack()
  
  def collapsiblePane(self, parent):
    # Creating Object of Collapsible Pane Container
    # If we do not pass these strings in
    # parameter the the default strings will appear
    # on button that were, expand >>, collapse <<
    cpane = cp(parent, 'Expanded', 'Collapsed')
    cpane.grid(row = 0, column = 0)
    
    # Button and checkbutton, these will
    # appear in collapsible pane container
    b1 = Button(cpane.frame, text ="GFG").grid(
                row = 1, column = 2, pady = 10)
    
    cb1 = Checkbutton(cpane.frame, text ="GFG").grid(
                      row = 2, column = 3, pady = 10)
    cpane2 = cp(parent, 'Expanded2', 'Collapsed')
    cpane2.grid(row = 0, column = 0)
    b2 = Button(cpane2.frame, text ="GFG").grid(
                row = 1, column = 2, pady = 10)
    
    cb2 = Checkbutton(cpane2.frame, text ="GFG").grid(
                      row = 2, column = 3, pady = 10)