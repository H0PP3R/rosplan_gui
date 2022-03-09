from tkinter import Tk, Button, Frame, Grid
from styling import *
from viewPane import ViewPane
from editPane import EditPane
from components.frameWithScrollBar import FrameWithScrollBar
from scripts.KB_CRUD import KnowledgeBaseNode

class App(Tk):
  def __init__(self):
    Tk.__init__(self)
    self.tableData = None
    self.predicateData = None
    self.headerText = None
    self.KB = KnowledgeBaseNode(self._callback)
    self._setInitialData()
    self.headerText = self._createHeaderText()
    self.geometry(f"{ROOT_SIZE[0]}x{ROOT_SIZE[1]}")

    container = FrameWithScrollBar(self)
    container.pack(side="top", fill="both", expand=True)

    # self.frames = {}
    # for F in (EditPane, ViewPane):
    #   pageName = F.__name__
    #   frame = F(parent=container.frame, controller=self,
    #             tableData=self.tableData, predicateData=self.predicateData)
    #   self.frames[pageName] = frame
    #   frame.grid(row=0, column=0, sticky="wesn")
    #   Grid.columnconfigure(container.frame,0,weight=1)

    self.editPane = EditPane(parent=container.frame, controller=self,
                              tableData=self.tableData, predicateData=self.predicateData,
                              headerText=self.headerText)
    self.editPane.grid(row=0, column=0, sticky="wesn")
    # self.viewPane = ViewPane(parent=container.frame, controller=self, 
    #                           tableData=self.tableData, predicateData=self.predicateData,
    #                           headerText=self.headerText)
    # self.viewPane.grid(row=0, column=0, sticky="wesn")
    Grid.columnconfigure(container.frame,0,weight=1)
    
    self.mainloop()
  
  def showFrame(self, page_name):
    '''Show a frame for the given page name'''
    frame = self.frames[page_name]
    frame.tkraise()

  def _callback(self, data):
    self._updatePane()
  
  def _setInitialData(self):
    predicateData = self.KB.getPredicates()
    numPredicateData = self.KB.getNumPredicates()
    numPredicateData = self._parseNumericPropTableData(numPredicateData)
    predicateData.update(numPredicateData)
    tableData = self.KB.getPropositions()
    numericPropData = self.KB.getNumPropositions()
    tableData.update(numericPropData)
    self.predicateData = predicateData
    self.tableData = tableData
      
  def _updateTableData(self):
    tableData = self.KB.getPropositions()
    numericPropData = self.KB.getNumPropositions()
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
  
  def _parseNumericPropTableData(self, propositions):
    for i in propositions:
      propositions[i]['funcVal'] = 'functional_value'
    return propositions

  def _createHeaderText(self):
    predNames = list(self.predicateData.keys())
    predParameters = list(self.predicateData.values())
    headerTexts = []
    for i in range(len(predNames)):
      tmp = f"{predNames[i]}"
      crntPredicateParams = predParameters[i]
      keys = list(crntPredicateParams.keys())
      for j in range(len(crntPredicateParams)):
        tmp += f" ?{keys[j]} - {crntPredicateParams[keys[j]]}"
      headerTexts.append(f'({tmp})')
    return headerTexts

  def _updatePane(self):
    self._updateTableData()
    # self.frames['ViewPane']._updateCPPane()
    # self.frames['EditPane']._updateTPane()
    # self.viewPane.updateCPane()
    self.editPane.updateTPane()