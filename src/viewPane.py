from tkinter import ttk
from components.collapsiblePane import ToggledFrame as cp
from styling import *
from scripts.KB_CRUD import KnowledgeBaseNode
from components.tablePane import TablePane

class ViewPane():
  def __init__(self, parent):
    self.parent = parent
    self.tableData = None
    self.predicateData = None
    self.KB = KnowledgeBaseNode(self._callback)
    self._setInitialData()
    self._createViewPane(parent)
  
  def _createViewPane(self, parent):
    print("createViewPane")
    self.viewPane = ttk.Frame(parent)
    vScroll = ttk.Scrollbar(self.viewPane)
    vScroll.pack(side="right", fill = "y")

    # predicatesPane = self._createPredicatePanes(viewPane, self.predicateData, self.tableData)
    # predicatesPane.pack(fill='x')
    self._updateViewPane(self.viewPane)

    self.viewPane.pack(fill="x")
    # vScroll.configure(command=predicatesPane.yview)
  
  def _updateViewPane(self, parent):
    predicatesPane = self._createPredicatePanes(parent, self.predicateData, self.tableData)
    predicatesPane.pack(fill='x')
  
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
      
  def _updatePane(self):
    print("updatePane")
    self._updateTableData()
    for widgets in self.viewPane.winfo_children():
      widgets.destroy()
    self._updateViewPane(self.viewPane)
    # self.tableData = tableData
  
  def _updateTableData(self):
    tableData = self.KB.getPropositions()
    numericPropData = self.KB.getNumPropositions()
    tableData.update(numericPropData)
    print(f'new data pulled from KB:\n{tableData}')
    # print(f'prev table_data:{self.tableData}\nnew data pulled from KB:{tableData}')
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
    # print(f'updated tableData: {self.tableData}')
  
  def _createPredicatePanes(self, parent, predicateData, tableData):
    predNames = list(predicateData.keys())
    predParameters = list(predicateData.values())
    predicatesPane = ttk.Frame(parent)

    predicateHeaders = self._createPredicateHeaders(predNames, predParameters)
    for i in range(len(predicateData)):
      cp = self._createCollapsiblePane(predicatesPane, predicateHeaders[i])
      # Parse current predicate propositional data and show as a table
      crntTableData = self._parseTableData(tableData, predNames[i], predParameters[i])
      TablePane(cp.sub_frame, crntTableData)
    return predicatesPane

  def _createPredicateHeaders(self, predNames, predParameters):
    cpHeaders = []
    for i in range(len(predNames)):
      tmp = f"{predNames[i]}"
      crntPredicateParams = predParameters[i]
      keys = list(crntPredicateParams.keys())
      for j in range(len(crntPredicateParams)):
        tmp += f" ?{keys[j]} - {crntPredicateParams[keys[j]]}"
      cpHeaders.append(f'({tmp})')
    return cpHeaders

  def _createCollapsiblePane(self, parent, headerText):
    predicateCP = cp(parent, text=headerText, relief="raised", borderwidth=1)
    predicateCP.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")
    return predicateCP

  def _parseTableData(self, data, attrName, attrVals):
    tableHeadings = ["timestep"]+list(attrVals.keys())+["True/False"]
    crntTableData = [tableHeadings]
    if attrName not in list(data.keys()):
      print(f'{attrName} is not in propositions')
    else:
      crntTableData = crntTableData+data[attrName]
    return crntTableData

  def _parseNumericPropTableData(self, propositions):
    for i in propositions:
      propositions[i]['funcVal'] = 'functional_value'
    return propositions

  def _callback(self, data):
    print("status update to KB made")
    self._updatePane()