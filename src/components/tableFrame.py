from tkinter import Frame, ttk

class TableFrame(Frame):
  '''
  Widget that creates a table.
  Extension of the Frame widget.
  '''
  def __init__(self, parent, data, height=5, callback=None, name='TablePane'):
    '''
    Constructor, sets the parameters as class variables
    @param self: the class itself
    @param parent: the frame in which this widget will be in
    @param data: formatted table data
    @param height: default height of records shown in frame
    @param callback: function to call when a record is selected
    @param name: string name of the frame
    '''
    Frame.__init__(self, parent)
    self.name=name
    self.data = data
    self.height = height
    self.callback = callback
    self.numRecords = 0
    self.selectedRowValues = []
    self._populateTable()
    self.table.bind('<ButtonRelease-1>', self._selectItem)
    self.pack(fill="x")

  def _populateTable(self):
    '''
    Procedure to populate the table with the data passed to object
    @param self: the class itself
    '''
    self.headings = self.data[0]
    self.table = ttk.Treeview(self, columns=self.headings, show='headings', height=self.height)
    self.table.pack(fill="x")
    self.table['columns'] = tuple(self.headings)

    # Format column
    for i in range(len(self.headings)):
      self.table.column(self.headings[i], anchor="center", width=80)
    # Create headings
    for i in range(len(self.headings)):
      self.table.heading(self.headings[i], text=str(self.headings[i]), anchor="center")
    # Add data
    if len(self.data) > 1:
      for i in range(1, len(self.data)):
          self.table.insert(parent='',index='end',iid=i, text='',
                            values=(tuple(self.data[i])))
          self.numRecords += 1

  def _selectItem(self, a):
    '''
    Procedure to called when table is clicked and gets the selected values
    @param self: the class itself
    @param a: event of mouse click
    '''
    # gets all the values of the selected row
    self.selectedRowValues = self.table.item(self.table.selection())['values']
    # when table is empty or when the heading of the table is selected
    if self.numRecords < 1 or len(self.selectedRowValues) < 1:
      pass
    else:
      if self.callback != None:
        self.callback(self.name)

  def deselect(self):
    '''
    Procedure to deselect the first selected record on the table
    @param self: the class itself
    '''
    self.table.selection_remove(self.table.selection()[0])

  def getSelectedRowValues(self):
    return self.selectedRowValues

  def getName(self):
    return self.name
