# Component taken from:
# https://stackoverflow.com/a/13169685
import tkinter as tk
from tkinter import ttk

class CollapsibleFrame(tk.Frame):
  '''
  Widget that creates a collapsible frame with a name.
  Initially collapsed.
  Extension of the Frame widget.
  '''
  def __init__(self, parent, text="", *args, **options):
    '''
    Constructor, sets the default state of the frame to be collapsed
    @param self: the class itself
    @param parent: the frame in which this widget will be in
    @param text: collapsible frame header text
    '''
    tk.Frame.__init__(self, parent, *args, **options)

    self.show = tk.IntVar()
    self.show.set(0)
    self.text = text
    self._populateFrame()
  
  def _populateFrame(self):
    '''
    Procedure that creates and populates the frame with
    labelling text and gives it a subframe
    @param self: the class itself
    '''
    titleFrame = ttk.Frame(self)
    titleFrame.pack(fill="x", expand=1)

    ttk.Label(titleFrame, text=self.text).pack(side="left", fill="x", expand=1)

    self.toggleButton = ttk.Checkbutton(titleFrame, width=2, text='+', command=self._toggle,
                                        variable=self.show, style='Toolbutton')
    self.toggleButton.pack(side="left")

    self.subFrame = tk.Frame(self, relief="sunken", borderwidth=1)

  def _toggle(self):
    '''
    Procedure to toggle the state of the collapsed frame 
    The frame will collapse it it is open and close if expanded
    @param self: the class itself
    '''
    if bool(self.show.get()):
      self.subFrame.pack(fill="x", expand=1)
      self.toggleButton.configure(text='-')
    else:
      self.subFrame.forget()
      self.toggleButton.configure(text='+')

  def getSubFrame(self):
    '''
    Function to return the subframe of the collapsible frame
    @param self: the class itself
    @return frame representing the subframe
    '''
    return self.subFrame
