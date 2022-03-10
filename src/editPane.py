from tkinter import Entry, Label, Frame, Button, StringVar
from styling import *
from components.tablePane import TablePane
from tkinter.messagebox import askyesno

class EditPane(Frame):
  def __init__(self, parent, controller, tableData, predicateData, headerText):
    Frame.__init__(self, parent)
    self.controller = controller
    self.tableData = tableData
    self.predicateData = predicateData
    self.headerText = headerText
    self.predNames = list(self.predicateData.keys())
    self.predParameters = list(self.predicateData.values())
    self.selectedTables = dict(zip(self.predNames, [0]*len(self.predNames)))
    self.prvSelected = None
    self.editValues = []
    self.request = None
    self._populatePane(self.controller)
    print(self.predicateData)
  
  def _populatePane(self, controller):
    self.mainPane = Frame(self)
    self.mainPane.pack(fill='x')

    Label(self.mainPane, text='Edit the most recent predicate value',
            anchor='w').pack(fill='x')
    self._createTablePanes(self.mainPane)

    Label(self.mainPane, text='').pack(fill='x')
    self.editValuesPane = self._createEditEntries(self.mainPane)

    self._createButtons(controller)
  
  def _confirm(self):
    ask = askyesno(title='confirmation', message='Confirm your changes')
    if ask:
      self.KB.update(self.request)
  
  def _createButtons(self, controller):
    buttonFrame = Frame(self.mainPane)
    buttonFrame.pack()
    top = Frame(buttonFrame)
    bottom = Frame(buttonFrame)
    top.pack(side='top')
    bottom.pack(side='bottom', fill='both', expand=True)
    submitButton = Button(buttonFrame, text="Apply", command=self._confirm)
    submitButton.pack(in_=top, side='left')
    switchPaneButton = Button(buttonFrame, text ="View", 
                              command=lambda: controller.showFrame("ViewPane"))
    switchPaneButton.pack(in_=top, side='left')

  def _createTablePanes(self, parent):
    self.listofTP = {}
    predNames = self.predNames
    predParameters = self.predParameters
    headings = self.headerText
    for i in range(len(self.predNames)):
      Label(parent, text=headings[i], 
            borderwidth=2, relief="raised", anchor='w', 
            ).pack(fill='x')
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
      frame = Frame(parent)
      frame.pack(fill='x')
      TablePane(frame, crntTableData, height=1, callback=self.callback, name=predNames[i])
      self.listofTP[predNames[i]] = frame

  def _parseTableData(self, data, attrName, attrVals):
    tableHeadings = ["timestep"]+list(attrVals.keys())+["True/False"]
    crntTableData = [tableHeadings]
    if attrName in list(data.keys()):
      crntTableData = crntTableData+[data[attrName][-1]]
    return crntTableData

  def updateTPane(self):
    predNames = self.predNames
    predParameters = self.predParameters
    for i in range(len(self.predNames)):
      crntFrame = self.listofTP[predNames[i]]
      for widgets in crntFrame.winfo_children():
        widgets.destroy()
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
      TablePane(crntFrame, crntTableData, height=1)

  def callback(self, attrName):
    if self.prvSelected!=attrName and attrName in self.tableData.keys():
      if list(self.selectedTables.values()).count(1) > 0:
        self.listofTP[self.prvSelected].winfo_children()[0].deselect()
        self.selectedTables[self.prvSelected] = 0

      self.selectedTables[attrName] = 1
      self.prvSelected = attrName
      print(self.selectedTables)
      self.editValues = self._parseEditVals(self.listofTP[attrName].winfo_children()[0])
      if len(self.editValues) > 0:
        self._updateEditEntries(self.editValuesPane, self.editValues)

  def _parseEditVals(self, crntTP):
    result = []
    if crntTP.numRecords > 0:
      editVals = crntTP.selectedRowValues
      editHeadings = ['timestep']+ list(self.predicateData[crntTP.name].keys())+["True/False"]
      result = [editHeadings, editVals]
    return result
  
  def _createEditEntries(self, parent):
    editValuesPane = Frame(parent)
    editValuesPane.pack(fill='x', side='top')
    l = Label(editValuesPane, text='Select predicate data to update')
    l.pack(fill='x')
    return editValuesPane

  def _updateEditEntries(self, parent, editValues):
    for widgets in parent.winfo_children():
      widgets.destroy()
    for i in range(len(editValues[0])):
      label = Label(parent, text=editValues[0][i], borderwidth=1, relief="raised")
      label.grid(row=0, column=i, sticky="nsew")
      entry = Entry(parent, textvariable=StringVar(), relief="sunken", borderwidth=1)
      entry.grid(row=1, column=i, sticky="nsew")
      # Insert current values
      entry.insert(0, editValues[1][i])
      parent.grid_columnconfigure(i, weight=1)
    parent.grid_rowconfigure(0, weight=1)
    parent.grid_rowconfigure(1, weight=1)