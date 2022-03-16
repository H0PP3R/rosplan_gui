from tkinter import StringVar, Entry, Tk

class DecimalEntry(Entry):
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

#demo:
if __name__ == '__main__':
  window = Tk()
  From_entry=DecimalEntry(window, width=25)
  From_entry.grid(column=1,row=2,padx=5)
  window.mainloop()