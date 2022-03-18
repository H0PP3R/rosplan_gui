from tkinter import Tk, Grid

from components.viewFrame import ViewFrame
from components.editFrame import EditFrame
from components.scrollbarFrame import ScrollbarFrame
from scripts.knowledgeBaseNode import KnowledgeBaseNode

APP_SIZE = (640,640)

class App(Tk):
  '''
  Main class of the rosplan_gui
  Extension of the top level widget of Tk.
  '''
  def __init__(self):
    '''
    Constructor, initialises variables to hold important data
    from the KnowledgeBase.
    Initialises the KnowledgeBaseNode.
    @param self: the class itself
    '''
    Tk.__init__(self)
    self.tableData = None
    self.predicateData = None
    self.headerText = None
    self.knowledgeBase = KnowledgeBaseNode(self._callback)
    self._setInitialData()
    self.headerText = self._createHeaderText()
    self.geometry(f'{APP_SIZE[0]}x{APP_SIZE[1]}')

    self._populateFrame()
    self.mainloop()

  def _populateFrame(self):
    '''
    Procedure to create and populate the App's main frame with widgets
    @param self: the class itself
    '''
    container = ScrollbarFrame(self)
    container.pack(side='top', fill='both', expand=True)

    data = {
      'tableData': self.tableData,
      'predicateData': self.predicateData,
      'headerText': self.headerText,
    }
    self.frames = {}
    self.viewFrame = ViewFrame(parent=container.getFrame(), controller=self, data=data)
    self.editFrame = EditFrame(parent=container.getFrame(), controller=self, data=data, 
                                KB=self.knowledgeBase)
    # Make frames stacked on top of on another
    self.frames['ViewFrame'] = self.viewFrame
    self.frames['EditFrame'] = self.editFrame
    for frame in list(self.frames.values()):
      frame.grid(row=0, column=0, sticky='wesn')
    Grid.columnconfigure(container.getFrame(),0,weight=1)
    # Puts ViewFrame on top of the other frames
    self.showFrame('ViewFrame')

  def showFrame(self, frameName):
    '''
    Procedure to raise the frame for the given frame name
    @param self: the class itself
    @param frameName: name of the frame to be shown
    '''
    frame = self.frames[frameName]
    frame.tkraise()

  def _callback(self, data):
    '''
    Procedure to react whenever a status update is published.
    Currently calls the procedure to update frames
    @param self: the class itself
    @param data: StatusUpdate object
    '''
    self._updateFrame()

  def _setInitialData(self):
    '''
    Procedure that sets the data from the KB.
    @param self: the class itself
    '''
    predicateData = self.knowledgeBase.getPredicates()
    numPredicateData = self.knowledgeBase.getNumPredicates()
    for i in numPredicateData:
      numPredicateData[i]['function_value'] = 'function_value'
    predicateData.update(numPredicateData)
    tableData = self.knowledgeBase.getPropositions()
    numericPropData = self.knowledgeBase.getNumPropositions()
    tableData.update(numericPropData)
    # predicateData used mostly to populate labels for tableData
    self.predicateData = predicateData
    # tableData used to populate table frames
    self.tableData = tableData

  def _updateTableData(self):
    '''
    Procedure that identifies new data added or changed in the KB
    and updates the tableData accordingly
    @param self: the class itself
    '''
    tableData = self.knowledgeBase.getPropositions()
    numericPropData = self.knowledgeBase.getNumPropositions()
    tableData.update(numericPropData)
    predicateNames = list(tableData.keys())
    crntPredicateNames = list(self.tableData.keys())
    for predName in predicateNames:
      # add new predicates
      if predName not in crntPredicateNames:
        self.tableData[predName] = tableData[predName]
      # add new propositions
      for prop in tableData[predName]:
        if prop not in self.tableData[predName]:
          self.tableData[predName].append(prop)
    # add new data for removed KB states
    for predName in crntPredicateNames:
      if predName not in predicateNames:
        entry = self.tableData[predName][-1]
        tmp = []
        for item in entry: tmp.append(item)
        tmp[-1] = 'False'
        if tmp not in self.tableData[predName]:
          self.tableData[predName].append(tmp)

  def _createHeaderText(self):
    '''
    Function that parses and creates tableData labels from predicateData
    @param self: the class itself
    @return the labels for each predicateData
    '''
    predNames = list(self.predicateData.keys())
    predParameters = list(self.predicateData.values())
    headerTexts = []
    for i, predName in enumerate(predNames):
      crntPredicateParams = predParameters[i]
      keys = list(crntPredicateParams.keys())
      for j in range(len(crntPredicateParams)):
        predName += f' ?{keys[j]} - {crntPredicateParams[keys[j]]}'
      headerTexts.append(f'({predName})')
    return headerTexts

  def _updateFrame(self):
    '''
    Procedure that updates the frames
    @param self: the class itself
    '''
    self._updateTableData()
    self.frames['ViewFrame'].updateCFrame()
    self.frames['EditFrame'].updateTFrame()
