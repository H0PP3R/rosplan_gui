from tkinter import Entry, Label, Frame, Button, StringVar, ttk
from styling import *
from components.tablePane import TablePane
from components.decimalEntry import DecimalEntry
from tkinter.messagebox import askyesno
from diagnostic_msgs.msg import KeyValue
import copy

class EditPane(Frame):
  def __init__(self, parent, controller, data, update):
    Frame.__init__(self, parent)
    self.controller = controller
    self.tableData = data['tableData']
    self.predicateData = data['predicateData']
    self.knowedgeTypes = data['knowledgeTypes']
    self.headerText = data['headerText']
    self.update = update
    self.predNames = list(self.predicateData.keys())
    self.predParameters = list(self.predicateData.values())
    self.selectedTables = dict(zip(self.predNames, [0]*len(self.predNames)))
    self.prvSelected = None
    self.editValues = []
    self.request = None
    self._populatePane(self.controller)
    # print(self.predicateData)
  
  def _populatePane(self, controller):
    self.mainPane = Frame(self)
    self.mainPane.pack(fill='x')

    editLabel = Label(self.mainPane, text='Edit the most recent predicate value',
            anchor='w')
    editLabel.pack(fill='x')
    self._createTablePanes(self.mainPane)

    emptySpace = Label(self.mainPane, text='')
    emptySpace.pack(fill='x')
    self.editValuesPane = self._createEditEntries(self.mainPane)

    self._createButtons(controller)
  
  def _confirm(self):
    ask = askyesno(title='confirmation', message='Confirm your changes')
    if ask:
      self._updateRecord()
  
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
      tpLabel = Label(parent, text=headings[i], 
                borderwidth=2, relief="raised", anchor='w', 
                )
      tpLabel.pack(fill='x')
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
      TablePane(crntFrame, crntTableData, height=1, callback=self.callback, name=predNames[i])

  def callback(self, attrName):
    if attrName in self.tableData.keys():
      if self.prvSelected == attrName:
        self.listofTP[self.prvSelected].winfo_children()[0].deselect()
        self.selectedTables[self.prvSelected] = 0
      else:
        if list(self.selectedTables.values()).count(1) > 0:
          self.listofTP[self.prvSelected].winfo_children()[0].deselect()
          self.selectedTables[self.prvSelected] = 0

        self.selectedTables[attrName] = 1
        self.prvSelected = attrName
        self.editValues = self._parseEditVals(self.listofTP[attrName].winfo_children()[0])
        if len(self.editValues) > 0:
          self._updateEditEntries(self.editValuesPane, self.editValues)

  def _parseEditVals(self, crntTP):
    result = []
    if crntTP.numRecords > 0:
      editVals = crntTP.selectedRowValues
      del editVals[0]
      editHeadings = list(self.predicateData[crntTP.name].keys())+["True/False"]
      result = [editHeadings, editVals]
    return result
  
  def _createEditEntries(self, parent):
    editValuesPane = Frame(parent)
    editValuesPane.pack(fill='x', side='top')
    l = Label(editValuesPane, text='Select predicate data to update')
    l.pack(fill='x')
    return editValuesPane

  def _updateEditEntries(self, parent, editValues):
    self.entriesList = []
    for widgets in parent.winfo_children():
      widgets.destroy()
    for i in range(len(editValues[0])):
      label = Label(parent, text=editValues[0][i], borderwidth=1, relief="raised")
      label.grid(row=0, column=i, sticky="nsew")
      if editValues[0][i] == 'function_value':
        entry = DecimalEntry(parent)
        entry.insert(0, editValues[1][i])
      elif editValues[0][i] == 'True/False':
        vals = ['True','False']
        idx = vals.index(editValues[1][i])
        entry = ttk.Combobox(parent, values=vals)
        entry.current(idx)
      else:
        entry = Entry(parent, textvariable=StringVar(), relief="sunken", borderwidth=1)
        entry.insert(0, editValues[1][i])
      entry.grid(row=1, column=i, sticky="nsew")
      # Insert current values
      self.entriesList.append(entry)
      parent.grid_columnconfigure(i, weight=1)
    parent.grid_rowconfigure(0, weight=1)
    parent.grid_rowconfigure(1, weight=1)
  
  def _updateRecord(self):
    selectedAttrName = list(self.selectedTables.keys())[list(self.selectedTables.values()).index(1)]
    newVals = {'attribute_name': selectedAttrName}
    copyOfEditVals = copy.deepcopy(self.editValues)
    # format the true/false values to 'is_negative'
    tf = self.entriesList[-1].get()
    if tf=='True': tf = True
    else: tf = False
    newVals['is_negative'] = not tf
    copyOfEditVals[0].remove('True/False')
    copyOfEditVals[1].pop(-1)
    # convert other data to 'values'
    values = []
    for i in range(len(copyOfEditVals[0])): 
      if 'function_value' == copyOfEditVals[0][i]:
        newVals['function_value'] = self.entriesList[i].get()
      else:
        pair = KeyValue()
        pair.key = copyOfEditVals[0][i]
        pair.value = self.entriesList[i].get()
        values.append(pair)
    newVals['values'] = values
    self.update(newVals)