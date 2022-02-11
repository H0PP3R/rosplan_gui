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
    predicatesPane = self._createPredicatePanes(viewPane, predicateData)
    predicatesPane.pack(fill='x')

    viewPane.pack(fill="x")
    # vScroll.configure(command=predicatesPane.yview)
  
  def _createPredicatePanes(self, parent, data):
    predParameters = list(data.values())
    predicatesPane = ttk.Frame(parent)

    predicateHeaders = self._createPredicateHeaders(data)
    for i in range(len(data)):
      cp = self._createCollapsiblePane(predicatesPane, predicateHeaders[i])
      TablePane(cp.sub_frame, self._parseTableData(predParameters[i]))
    return predicatesPane

  def _createPredicateHeaders(self, data):
    predNames = list(data.keys())
    predParameters = list(data.values())
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

  def _parseTableData(self, data):
    # data initial struct:
    # {key: value, key: value}
    # return struct:
    # [[header1, header2, ..., header_n],[data1, data2, ..., data_n],...]
    result = []
    tableHeadings = list(data.keys())
    result.append(["timestep"]+tableHeadings)
    # for i in range(len(keys)):
    #   result.append([keys[i], data[keys[i]]])
    return result
    