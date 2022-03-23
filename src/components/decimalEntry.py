from tkinter import StringVar, Entry

class DecimalEntry(Entry):
  '''
  Widget that creates a decimal only entry
  Extension of the Entry widget.
  '''
  def __init__(self, parent=None, **kwargs):
    '''
    Constructor, traces the user input into entry
    @param self: the class itself
    @param parent: the frame in which this widget will be in
    '''
    self.var = StringVar()
    Entry.__init__(self, parent, textvariable=self.var, **kwargs)
    self.oldValue = ''
    self.var.trace('w', self.check)
    self.get, self.set = self.var.get, self.var.set

  def check(self, *args):
    '''
    Procedure that validates the input and only allows number entries
    @param self: the class itself
    '''
    if self.get() == '':
      self.oldValue = self.get()
    else:
      try:
        float(self.get())
        self.oldValue = self.get()
      except ValueError:
        self.set(self.oldValue)