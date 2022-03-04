from tkinter import Tk, Button, Frame, Grid
from styling import *
from viewPane import ViewPane
from editPane import EditPane
from components.frameWithScrollBar import FrameWithScrollBar

class App(Tk):
  def __init__(self):
    Tk.__init__(self)
    self.geometry(f"{ROOT_SIZE[0]}x{ROOT_SIZE[1]}")

    container = FrameWithScrollBar(self)
    container.pack(side="top", fill="both", expand=True)

    self.frames = {}
    for F in (EditPane, ViewPane):
      pageName = F.__name__
      frame = F(parent=container.frame, controller=self)
      self.frames[pageName] = frame
      frame.grid(row=0, column=0, sticky="wesn")
      Grid.columnconfigure(container.frame,0,weight=1)
    
    self.mainloop()
  
  def show_frame(self, page_name):
    '''Show a frame for the given page name'''
    frame = self.frames[page_name]
    frame.tkraise()