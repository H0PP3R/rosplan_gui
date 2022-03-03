from tkinter import Tk, Button, Frame
from styling import *
from viewPane import ViewPane
from editPane import EditPane

class App(Tk):
  def __init__(self):
    Tk.__init__(self)
    self.geometry(f"{ROOT_SIZE[0]}x{ROOT_SIZE[1]}")

    container = Frame(self)
    container.pack(side="top", fill="both", expand=True)
    container.grid_rowconfigure(0, weight=1)
    container.grid_columnconfigure(0, weight=1)

    # self.viewPane = ViewPane(self.root)
    self.frames = {}
    for F in (EditPane, ViewPane):
      pageName = F.__name__
      frame = F(parent=container, controller=self)
      self.frames[pageName] = frame
      frame.grid(row=0, column=0, sticky="nsew")
    
    self.mainloop()
  
  def show_frame(self, page_name):
    '''Show a frame for the given page name'''
    frame = self.frames[page_name]
    frame.tkraise()