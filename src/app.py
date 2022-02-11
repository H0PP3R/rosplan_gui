from tkinter import Tk, ttk
from components.collapsiblePane import ToggledFrame as cp
from styling import *
from scripts.KB_CRUD import KnowledgeBaseNode

class App():
  def __init__(self):
    root = Tk()
    root.geometry(f'{ROOT_SIZE[0]}x{ROOT_SIZE[1]}')
    self.KB = KnowledgeBaseNode()

    self._readPane(root)

    root.mainloop()
  
  def _readPane(self, parent):
    readPane = ttk.Frame(parent)
    predicates = self.KB.getPredicates()
    self._variablePane(readPane, predicates)
    readPane.pack(fill="x")
  
  def _variablePane(self, parent, data):
    predNames = list(data.keys())
    predParameters = list(data.values())
    # print(f'len(data):{len(data)}\npredNames:{predNames}\npredParams:{predParameters}')
    for i in range(len(data)):
      predicateCP = cp(parent, text=predNames[i], relief="raised", borderwidth=1)
      predicateCP.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")
      crntPredicate = predParameters[i]
      
      for j in range(len(crntPredicate)):
        # print(f'crntPredicate:{crntPredicate}')
        x = list(crntPredicate.keys())
        txt = str(x[j] + crntPredicate[x[j]])
        ttk.Label(predicateCP.sub_frame, text=txt).pack()
