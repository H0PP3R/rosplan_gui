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
    tk.Frame.__init__(self, parent, *args, **options)

    self.show = tk.IntVar()
    self.show.set(0)

    self.titleFrame = ttk.Frame(self)
    self.titleFrame.pack(fill="x", expand=1)

    ttk.Label(self.titleFrame, text=text).pack(side="left", fill="x", expand=1)

    self.toggleButton = ttk.Checkbutton(self.titleFrame, width=2, text='+', command=self.toggle,
                                        variable=self.show, style='Toolbutton')
    self.toggleButton.pack(side="left")

    self.subFrame = tk.Frame(self, relief="sunken", borderwidth=1)

  def toggle(self):
    if bool(self.show.get()):
      self.subFrame.pack(fill="x", expand=1)
      self.toggleButton.configure(text='-')
    else:
      self.sub_frame.forget()
      self.toggleButton.configure(text='+')

  def getSubFrame(self):
    return self.subFrame
