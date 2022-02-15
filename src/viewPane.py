from tkinter import ttk
from components.collapsiblePane import ToggledFrame as cp
from styling import *
from scripts.KB_CRUD import KnowledgeBaseNode
from components.tablePane import TablePane

class ViewPane():
  def __init__(self, parent):
    self.KB = KnowledgeBaseNode()
    self._createViewPane(parent)
  
  def _createViewPane(self, parent):
    viewPane = ttk.Frame(parent)
    vScroll = ttk.Scrollbar(viewPane)
    vScroll.pack(side="right", fill = "y")

    # creating and populating predicate panes
    predicateData = self.KB.getPredicates()
    numPredicateData = self.KB.getNumPredicates()
    predicateData.update(numPredicateData)
    tableData = self.KB.getPropositions()
    predicatesPane = self._createPredicatePanes(viewPane, predicateData, tableData)
    predicatesPane.pack(fill='x')

    viewPane.pack(fill="x")
    # vScroll.configure(command=predicatesPane.yview)
  
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
    tableHeadings = ["timestep"]+list(attrVals.keys())
    crntTableData = [tableHeadings]
    if attrName not in list(data.keys()):
      print(f'{attrName} is not in propositions')
    else:
      crntTableData = crntTableData+data[attrName]
    return crntTableData
