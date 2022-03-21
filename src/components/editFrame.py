import copy
from tkinter import Entry, Label, Frame, Button, StringVar, ttk
from tkinter.messagebox import askyesno
from diagnostic_msgs.msg import KeyValue

from .tableFrame import TableFrame as tFrame
from .decimalEntry import DecimalEntry

class EditFrame(Frame):
  '''
  Widget that edits the latest knowledge base state
  Auto-refreshes with knowledge base changes
  Extension of the Frame widget.
  '''
  def __init__(self, parent, controller, data, KB):
    '''
    Constructor, sets the parameters as class variables
    @param self: the class itself
    @param parent: the frame in which this widget will be in
    @param controller: controls the order of the frames
    @param data: formatted predicate and proposition data
    @param knowledgeBase: instance of KnowledgeBaseNode that interacts with the Knowledge Base
    '''
    Frame.__init__(self, parent)
    self.listofTP = {}
    self.entriesList = []
    self.controller = controller
    self.tableData = data['tableData']
    self.predicateData = data['predicateData']
    self.headerText = data['headerText']
    self.knowledgeBase = KB
    self.predNames = list(self.predicateData.keys())
    self.predParameters = list(self.predicateData.values())
    self.selectedTables = dict(zip(self.predNames, [0]*len(self.predNames)))
    self.prvSelected = None
    self.editValues = []
    self.selectedAttrName = ''
    self._populateFrame()

  def _populateFrame(self):
    '''
    Procedure to create and populate the EditFrame with widgets
    @param self: the class itself
    '''
    self.mainFrame = Frame(self)
    self.mainFrame.pack(fill='x')

    editLabel = Label(self.mainFrame,
                      text='Edit the most recent predicate value', anchor='w')
    editLabel.pack(fill='x')
    self._createTableFrames(self.mainFrame)

    emptySpace = Label(self.mainFrame, text='')
    emptySpace.pack(fill='x')
    self.editValuesFrame = self._createEditEntries(self.mainFrame)

    self._createButtons(self.mainFrame)

  def _confirm(self):
    '''
    Procedure to create a pop up for user to double confirm their changes
    before updating
    @param self: the class itself
    '''
    ask = askyesno(title='confirmation', message='Confirm your changes')
    if ask:
      self._updateRecord()

  def _createButtons(self, parent):
    '''
    Procedure to create the buttons used in the EditFrame
    @param self: the class itself
    @param parent: the frame in which this widget will be in
    '''
    buttonFrame = Frame(parent)
    buttonFrame.pack()
    # Two frames used to center the two buttons horizontally
    top = Frame(buttonFrame)
    bottom = Frame(buttonFrame)
    top.pack(side='top')
    bottom.pack(side='bottom', fill='both', expand=True)
    submitButton = Button(buttonFrame, text='Apply', command=self._confirm)
    submitButton.pack(in_=top, side='left')
    switchFrameButton = Button(buttonFrame, text ='View',
                              command=lambda: self.controller.showFrame('ViewFrame'))
    switchFrameButton.pack(in_=top, side='left')

  def _createTableFrames(self, parent):
    '''
    Procedure that creates the TableFrames with Labels
    @param self: the class itself
    @param parent: the frame in which this widget will be in
    '''
    predNames = self.predNames
    predParameters = self.predParameters
    headings = self.headerText
    for i in range(len(self.predNames)):
      tpLabel = Label(parent, text=headings[i],
                borderwidth=2, relief='raised', anchor='w')
      tpLabel.pack(fill='x')
      crntTableData = self._parseTableData(
        self.tableData, predNames[i], predParameters[i]
      )
      frame = Frame(parent)
      frame.pack(fill='x')
      tFrame(frame, crntTableData, height=1, callback=self.callback, name=predNames[i])
      self.listofTP[predNames[i]] = frame

  def _parseTableData(self, data, attrName, attrVals):
    '''
    Function that prepares tableData and returns the prepared data
    @param self: the class itself
    @param data: tableData
    @param attrName: string name of attribute
    @param attrVals: dictionary of attribute values
    @return prepared attribute table data
    '''
    tableHeadings = ['timestep']+list(attrVals.keys())
    if 'function_value' not in attrVals.keys():
      tableHeadings += ['True/False']
    crntTableData = [tableHeadings]
    if attrName in list(data.keys()):
      crntTableData = crntTableData+[data[attrName][-1]]
    return crntTableData

  def updateTFrame(self):
    '''
    Procedure to update the TableFrames to display new tableData
    @param self: the class itself
    '''
    predNames = self.predNames
    predParameters = self.predParameters
    # destroys the TableFrames within the frame
    # and recreate the tableFrames with new tableData
    for i in range(len(self.predNames)):
      crntFrame = self.listofTP[predNames[i]]
      for widgets in crntFrame.winfo_children():
        widgets.destroy()
      crntTableData = self._parseTableData(
        self.tableData, predNames[i], predParameters[i]
      )
      tFrame(crntFrame, crntTableData, height=1,
            callback=self.callback, name=predNames[i])

  def callback(self, attrName):
    '''
    Procedure to update the most recently selected table record
    Then deselect the previously selected table record
    @param self: the class itself
    @param attrName: string name of attribute
    '''
    if attrName in self.tableData.keys():
      if self.prvSelected == attrName:
        self.listofTP[self.prvSelected].winfo_children()[0].deselect()
        self.selectedTables[self.prvSelected] = 0
      else:
        if list(self.selectedTables.values()).count(1) > 0:
          x = self.listofTP[self.prvSelected].winfo_children()
          self.listofTP[self.prvSelected].winfo_children()[0].deselect()
          self.selectedTables[self.prvSelected] = 0

        self.selectedTables[attrName] = 1
        self.prvSelected = attrName
        self.editValues = self._parseEditVals(
          self.listofTP[attrName].winfo_children()[0]
        )
        if len(self.editValues) > 0:
          self._updateEditEntries(self.editValuesFrame, self.editValues)
        self.selectedAttrName = list(self.selectedTables.keys())[
          list(self.selectedTables.values()).index(1)
        ]

  def _parseEditVals(self, crntTF):
    '''
    Function that updates the frames
    @param self: the class itself
    @param crntTF: selected TableFrame
    @return selected table records
    '''
    result = []
    if crntTF.numRecords > 0:
      editVals = crntTF.getSelectedRowValues()
      del editVals[0] # delete timestep value
      tableName = crntTF.getName()
      predicateHeaders = list(self.predicateData[tableName].keys())
      editHeadings = list(self.predicateData[tableName].keys())
      if 'function_value' not in predicateHeaders:
        editHeadings += ['True/False']
      result = [editHeadings, editVals]
    return result

  def _createEditEntries(self, parent):
    '''
    Function that creates the frame that displays selected values
    to be edited and returns it.
    @param self: the class itself
    @param parent: the frame in which this widget will be in
    @return frame of the editable values
    '''
    editValuesFrame = Frame(parent)
    editValuesFrame.pack(fill='x', side='top')
    label = Label(editValuesFrame, text='Select predicate data to update')
    label.pack(fill='x')
    return editValuesFrame

  def _updateEditEntries(self, parent, editValues):
    '''
    Procedure that updates the frame that contains the selected editable
    values.
    @param self: the class itself
    @param parent: the frame in which this widget will be in
    @param editValues: values to update in the frame
    '''
    self.entriesList = []
    for widgets in parent.winfo_children():
      widgets.destroy()
    for i, editValue in enumerate(editValues[0]):
      label = Label(parent, text=editValue, borderwidth=1, relief='raised')
      label.grid(row=0, column=i, sticky='nsew')
      if editValue == 'function_value':
        entry = DecimalEntry(parent)
        entry.insert(0, editValues[1][i])
      elif editValue == 'True/False':
        vals = ['True','False']
        idx = vals.index(editValues[1][i])
        entry = ttk.Combobox(parent, values=vals)
        entry.current(idx)
      else:
        entry = Entry(parent, textvariable=StringVar(),
                      relief='sunken', borderwidth=1)
        entry.insert(0, editValues[1][i])
      entry.grid(row=1, column=i, sticky='nsew')
      # Insert current values
      self.entriesList.append(entry)
      parent.grid_columnconfigure(i, weight=1)
    parent.grid_rowconfigure(0, weight=1)
    parent.grid_rowconfigure(1, weight=1)

  def _updateRecord(self):
    '''
    Procedure that sends the records to update to the KnowledgeBaseNode
    procedure that updates the KnowledgeBase state
    @param self: the class itself
    '''
    crntVals = self._prepareRecords(self.selectedAttrName, type='select')
    newVals = self._prepareRecords(self.selectedAttrName, type='edit')
    facts = {
      'crnt': crntVals,
      'new': newVals
    }
    if facts['new']['is_negative']:
      self.knowledgeBase.delete(facts)
    else:
      self.knowledgeBase.update(facts)
    self.selectedTables[self.selectedAttrName] = 0

  def _prepareRecords(self, selectedAttrName, type='select'):
    '''
    Function that parses and returns the prepared
    table record values
    @param self: the class itself
    @param selectedAttrName: string name of the selected attribute
    @param type: indicates where to get state data from, the selected
                  table record or the edited table record
    @return dictionary of prepared table record values
    '''
    vals = {'attribute_name': selectedAttrName}
    copyOfEditVals = copy.deepcopy(self.editValues)
    headings = copyOfEditVals[0]
    editedVals = copyOfEditVals[1]
    # format the true/false values to 'is_negative'
    if 'function_value' not in headings:
      if type == 'select':
        newTf = self._parseIsNegative(editedVals[-1])
      else:
        newTf = self._parseIsNegative(self.entriesList[-1].get())
      vals['is_negative'] = newTf
      headings.remove('True/False')
      editedVals.pop(-1)
    # convert other data to 'values'
    values = []
    for i, heading in enumerate(headings):
      if heading == 'function_value':
        if type == 'select':
          vals['function_value'] = editedVals[-1]
        else:
          vals['function_value'] = self.entriesList[-1].get()
        headings.remove('function_value')
        editedVals.pop(-1)
      else:
        pair = KeyValue()
        pair.key = heading
        if type == 'select':
          pair.value = editedVals[i]
        else:
          pair.value = self.entriesList[i].get()
        values.append(pair)
    vals['values'] = values
    return vals

  def _parseIsNegative(self, trueFalse):
    '''
    Function to parse string True and False, prepare
    it for is_negative and returns it
    @param self: the class itself
    @param trueFalse: string of either 'True' or 'False'
    @return boolean that is the opposite of the parameter trueFalse
    '''
    if trueFalse == 'True':
      trueFalse = True
    else:
      trueFalse = False
    return not trueFalse
