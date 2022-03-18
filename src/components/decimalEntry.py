from tkinter import StringVar, Entry, Tk

class DecimalEntry(Entry):
  '''
  Widget that creates a decimal only entry
  Extension of the Entry widget.
  '''
  def __init__(self, master=None, **kwargs):
    self.var = StringVar()
    Entry.__init__(self, master, textvariable=self.var, **kwargs)
    self.old_value = ''
    self.var.trace('w', self.check)
    self.get, self.set = self.var.get, self.var.set

  def check(self, *args):
    if self.get() == '':
      self.old_value = self.get()
    else:
      try: 
        float(self.get())
        self.old_value = self.get()
      except ValueError:
        self.set(self.old_value)