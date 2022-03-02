from tkinter import Tk, Button
from styling import *
from viewPane import ViewPane
from editPane import EditPane

class App():
  def __init__(self):
    self.mainPane = None
    self.root = Tk()
    self.root.geometry(f"{ROOT_SIZE[0]}x{ROOT_SIZE[1]}")

    self.viewPane = ViewPane(self.root)
    self.mainPane = self.viewPane
    self.paneID = 0
    button = Button(self.root, text ="switch", command=self.toggleView)
    button.pack(side='left')

    self.root.mainloop()
  
  def toggleView(self):
    if self.paneID == 0: 
      del self.viewPane
      self.editPane = EditPane(self.root)
      self.mainPane = self.editPane
      self.paneID = 1
    elif self.paneID == 1:
      del self.editPane
      self.viewPane = ViewPane(self.root)
      self.mainPane = self.viewPane
      self.paneID = 0  