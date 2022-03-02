from tkinter import *
from  tkinter import ttk
from styling import TABLE_STYLING
class TablePane(ttk.Frame):
  def __init__(self, parent, data, mode='read'):
    self.data = data
    self.frame = ttk.Frame(parent)
    self.frame.pack(fill="x")

    if mode=='read':
      self._populateViewTable()
    elif mode=='write':
      # for edit version -----
      # for i in range(len(data)):
      #   for j in range(len(data[0])):
      #     self.e = Entry(self.frame, width=20)
      #     self.e.grid(row=i, column=j)
      #     self.e.insert(END, data[i][j])
      pass
    else:
      pass

  def _populateViewTable(self):
    self.headings = self.data[0]
    self.table = ttk.Treeview(self.frame, columns=self.headings, show='headings')
    self.table.pack(fill="x")
    self.table['columns'] = tuple(self.headings)

    for i in range(len(self.headings)):
      self.table.column(self.headings[i], anchor=TABLE_STYLING["ANCHOR"], width=80)
    for i in range(len(self.headings)):
      self.table.heading(self.headings[i], text=str(self.headings[i]), anchor=TABLE_STYLING["ANCHOR"])
    if len(self.data) > 1:
      for i in range(1, len(self.data)):
          self.table.insert(parent='',index='end',iid=i, text='',
            values=(tuple(self.data[i])))