from tkinter import END, Frame, ttk, Entry
from styling import TABLE_STYLING
class TablePane(Frame):
  def __init__(self, parent, data, height=5, callback=None, name='TablePane'):
    Frame.__init__(self, parent)
    self.name=name
    self.data = data
    self.height = height
    self.callback = callback
    self.pack(fill="x")
    self.numRecords = 0
    self.selectedRowValues = []
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
          self.numRecords += 1
    self.table.bind('<ButtonRelease-1>', self.selectItem)
  
  def selectItem(self, a):   # added self and a (event)
    self.selectedRowValues = self.table.item(self.table.selection())['values']# gets all the values of the selected row
    if self.numRecords < 1:
      print("Table empty")
    if self.callback != None: 
      self.callback(self.name)

  def deselect(self):
    self.table.selection_remove(self.table.selection()[0])
  