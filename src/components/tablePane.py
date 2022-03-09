from tkinter import END, Frame, ttk, Entry
from styling import TABLE_STYLING
class TablePane(Frame):
  def __init__(self, parent, data, height=5):
    Frame.__init__(self, parent)
    self.data = data
    self.height = height
    self.pack(fill="x")
    self._populateTable()

  def _populateTable(self):
    self.headings = self.data[0]
    self.table = ttk.Treeview(self, columns=self.headings, show='headings', height=self.height)
    self.table.pack(fill="x")
    self.table['columns'] = tuple(self.headings)
    
    # Format column
    for i in range(len(self.headings)):
      self.table.column(self.headings[i], anchor=TABLE_STYLING["ANCHOR"], width=80)
    # Create headings
    for i in range(len(self.headings)):
      self.table.heading(self.headings[i], text=str(self.headings[i]), anchor=TABLE_STYLING["ANCHOR"])
    # Add data
    if len(self.data) > 1:
      for i in range(1, len(self.data)):
          self.table.insert(parent='',index='end',iid=i, text='',
            values=(tuple(self.data[i])))