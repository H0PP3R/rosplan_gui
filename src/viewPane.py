from tkinter import Frame, ttk, Button
from components.collapsiblePane import ToggledFrame as cp
from styling import *
from scripts.KB_CRUD import KnowledgeBaseNode
from components.tablePane import TablePane

class ViewPane(Frame):
  def __init__(self, parent, controller):
    Frame.__init__(self, parent)
    self.controller = controller
    self.tableData = None
    self.predicateData = None
    self.KB = KnowledgeBaseNode(self._callback)
    self._setInitialData()
    self._createViewPane(self.controller)
  
  def _createViewPane(self, controller):
    print("createViewPane")
    vScroll = ttk.Scrollbar(self)
    vScroll.pack(side="right", fill = "y")

    predicatesPane = self._createPredicatePanes(self)
    predicatesPane.pack(fill='x')

    button = Button(self, text ="Edit/Delete", 
                    command=lambda: controller.show_frame("EditPane"))
    button.pack()

    # self.grid(column=0, row=0)
    # vScroll.configure(command=predicatesPane.yview)
  
  def _updateCPPane(self):
    predNames = list(self.predicateData.keys())
    predParameters = list(self.predicateData.values())
    for i in range(len(self.predicateData)):
      crntFrame = self.listofCP[i].sub_frame
      for widgets in crntFrame.winfo_children():
        widgets.destroy()
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
      TablePane(crntFrame, crntTableData)
  
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
    self._updateTableData()
    self._updateCPPane()
  
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
  
  def _createPredicatePanes(self, parent):
    self.listofCP = []
    predNames = list(self.predicateData.keys())
    predParameters = list(self.predicateData.values())
    predicatesPane = ttk.Frame(parent)

    predicateHeaders = self._createPredicateHeaders(predNames, predParameters)
    for i in range(len(self.predicateData)):
      cp = self._createCollapsiblePane(predicatesPane, predicateHeaders[i])
      self.listofCP.append(cp)
      # Parse current predicate propositional data and show as a table
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
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
    self._updatePane()