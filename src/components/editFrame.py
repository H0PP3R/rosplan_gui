import copy
from tkinter import Entry, Label, Frame, Button, StringVar, ttk
from tkinter.messagebox import askyesno
from diagnostic_msgs.msg import KeyValue

from .tableFrame import TableFrame as tf
from .decimalEntry import DecimalEntry

class EditFrame(Frame):
  def __init__(self, parent, controller, data, KB):
    Frame.__init__(self, parent)
    self.controller = controller
    self.tableData = data['tableData']
    self.predicateData = data['predicateData']
    self.headerText = data['headerText']
    self.KB = KB
    self.predNames = list(self.predicateData.keys())
    self.predParameters = list(self.predicateData.values())
    self.selectedTables = dict(zip(self.predNames, [0]*len(self.predNames)))
    self.prvSelected = None
    self.editValues = []
    self.selectedAttrName = ''
    self._populatePane(self.controller)
  
  def _populatePane(self, controller):
    self.mainPane = Frame(self)
    self.mainPane.pack(fill='x')

    editLabel = Label(self.mainPane, 
                      text='Edit the most recent predicate value', anchor='w')
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
                              command=lambda: controller.showFrame("ViewFrame"))
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
      tf(frame, crntTableData, height=1, callback=self.callback, name=predNames[i])
      self.listofTP[predNames[i]] = frame

  def _parseTableData(self, data, attrName, attrVals):
    tableHeadings = ["timestep"]+list(attrVals.keys())
    if 'function_value' not in attrVals.keys():
      tableHeadings += ["True/False"]
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
      tf(crntFrame, crntTableData, height=1, callback=self.callback, name=predNames[i])

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
      self.selectedAttrName = list(self.selectedTables.keys())[list(self.selectedTables.values()).index(1)]

  def _parseEditVals(self, crntTP):
    result = []
    if crntTP.numRecords > 0:
      editVals = crntTP.selectedRowValues
      del editVals[0]
      predicateHeaders = list(self.predicateData[crntTP.name].keys())
      editHeadings = list(self.predicateData[crntTP.name].keys())
      if 'function_value' not in predicateHeaders:
        editHeadings += ["True/False"]
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
    crntVals = self._getSelectedState(self.selectedAttrName)
    newVals = self._getEditedState(self.selectedAttrName)
    facts = {
      'crnt': crntVals,
      'new': newVals
    }
    self.KB.update(facts)

  def _deleteRecord(self):
    crntVals = self._getSelectedState(self.selectedAttrName)
    facts = {'crnt': crntVals}
    self.KB.delete(facts)
  
  def _getSelectedState(self, selectedAttrName):
    crntVals = {'attribute_name': selectedAttrName}
    copyOfEditVals = copy.deepcopy(self.editValues)
    # format the true/false values to 'is_negative'
    if 'function_value' not in copyOfEditVals[0]:
      oldTf = self._parseIsNegative(copyOfEditVals[1][-1])
      crntVals['is_negative'] = oldTf
      copyOfEditVals[0].remove('True/False')
      copyOfEditVals[1].pop(-1)
    # convert other data to 'values'
    crntValues = []
    for i in range(len(copyOfEditVals[0])): 
      if copyOfEditVals[0][i] == 'function_value':
        crntVals['function_value'] = copyOfEditVals[1][-1]
        copyOfEditVals[0].remove('function_value')
        copyOfEditVals[1].pop(-1)
      else:
        oldPair = KeyValue()
        oldPair.key = copyOfEditVals[0][i]
        oldPair.value = copyOfEditVals[1][i]
        crntValues.append(oldPair)
    crntVals['values'] = crntValues
    return crntVals

  def _getEditedState(self, selectedAttrName):
    newVals = {'attribute_name': selectedAttrName}
    copyOfEditVals = copy.deepcopy(self.editValues)
    # format the true/false values to 'is_negative'
    if 'function_value' not in copyOfEditVals[0]:
      newTf = self._parseIsNegative(self.entriesList[-1].get())
      newVals['is_negative'] = newTf
      copyOfEditVals[0].remove('True/False')
      copyOfEditVals[1].pop(-1)
    # convert other data to 'values'
    newValues = []
    for i in range(len(copyOfEditVals[0])): 
      if copyOfEditVals[0][i] == 'function_value':
        newVals['function_value'] = self.entriesList[-1].get()
        copyOfEditVals[0].remove('function_value')
        copyOfEditVals[1].pop(-1)
      else:
        newPair = KeyValue()
        newPair.key = copyOfEditVals[0][i]
        newPair.value = self.entriesList[i].get()
        newValues.append(newPair)
    newVals['values'] = newValues
    return newVals

  def _parseIsNegative(self, tf):
    if tf=='True': tf = True
    else: tf = False
    return not tf