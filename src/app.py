from tkinter import Tk, ttk
from components.collapsiblePane import ToggledFrame as cp
from styling import *
from scripts.KB_CRUD import KnowledgeBaseNode
from components.tablePane import TablePane

class App():
  def __init__(self):
    root = Tk()
    root.geometry(f'{ROOT_SIZE[0]}x{ROOT_SIZE[1]}')
    self.KB = KnowledgeBaseNode()

    self._createViewPane(root)

    root.mainloop()
  
  def _createViewPane(self, parent):
    viewPane = ttk.Frame(parent)
    vScroll = ttk.Scrollbar(viewPane)
    vScroll.pack(side='right', fill = 'y')

    # creating and populating predicate panes
    predicateData = self.KB.getPredicates()
    predicatesPane = self._createPredicatePanes(viewPane, predicateData)
    predicatesPane.pack(fill='x')

    viewPane.pack(fill="x")
    vScroll.configure(command=predicatesPane.yview)
  
  def _createPredicatePanes(self, parent, data):
    predNames = list(data.keys())
    predParameters = list(data.values())
    predicatesPane = ttk.Frame(parent)
    for i in range(len(data)):
      cp = self._createCollapsiblePane(predicatesPane, predNames[i])
      crntPredicate = predParameters[i]
      TablePane(cp.sub_frame, self._parseTableData(crntPredicate))
    return predicatesPane

  def _createCollapsiblePane(self, parent, headerText):
    predicateCP = cp(parent, text=headerText, relief="raised", borderwidth=1)
    predicateCP.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")
    return predicateCP

  def _parseTableData(self, data):
    # data initial struct:
    # {key: value, key: value}
    # return struct:
    # [[header1, header2, ..., header_n],[data1, data2, ..., data_n],...]
    result = [['key', 'value']]
    keys = list(data.keys())
    for i in range(len(keys)):
      result.append([keys[i], data[keys[i]]])
    return result
    