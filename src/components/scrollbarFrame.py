# Component taken from:
# https://stackoverflow.com/a/57745179
import tkinter as tk

class ScrollbarFrame(tk.Frame):
  '''
  Widget that creates a empty frame with a scrollbar
  that adjusts contents horizontally as the window is adjusted
  Extension of the Frame widget.
  '''
  def __init__(self, parent, *args, **kwargs):
    self.count = 0
    super().__init__(parent, *args, **kwargs)
    self.canvas = tk.Canvas(self)
    self.frame = tk.Frame(self.canvas)
    self.scrollbar = tk.Scrollbar(self, orient='vertical',
                            command=self.canvas.yview)
    self.canvas.configure(yscrollcommand=self.scrollbar.set)
    self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    self.scrollbar.pack(side=tk.RIGHT, fill=tk.BOTH)
    self.frame.pack(fill=tk.BOTH, expand=True)
    self.frameID = self.canvas.create_window(
                              self.canvas.winfo_width(), 0,
                              anchor='nw',
                              window=self.frame)
    self.frame.bind('<Configure>', self.onFrameConfigure)
    self.canvas.bind('<Configure>', self.onCanvasConfigure)

  def onFrameConfigure(self, event):
    '''
    Procedure that configures the scroll region to the whole frame
    @param self: the class itself
    @param event: event the triggered this method
    '''
    self.canvas.configure(scrollregion=self.frame.bbox('all'))

  def onCanvasConfigure(self, event):
    '''
    Procedure that configures the canvas to span the width
    of the frame within it
    @param self: the class itself
    @param event: event the triggered this method
    '''
    self.canvas.itemconfigure(self.frameID, width=self.canvas.winfo_width())

  def getFrame(self):
    '''
    Function to return the frame within self
    @param self: the class itself
    @return frame widget
    '''
    return self.frame
