from tkinter import Frame, Button

from .collapsibleFrame import CollapsibleFrame as cf
from .tableFrame import TableFrame as tf

class ViewFrame(Frame):
  def __init__(self, parent, controller, data):
    Frame.__init__(self, parent)
    self.controller = controller
    self.tableData = data['tableData']
    self.predicateData = data['predicateData']
    self.headerText = data['headerText']
    self.predNames = list(self.predicateData.keys())
    self.predParameters = list(self.predicateData.values())
    self._populatePane(self.controller)
  
  def _populatePane(self, controller):
    predicatesPane = self._createPredicatePanes(self)
    predicatesPane.pack(fill='x')

    button = Button(self, text ="Edit/Delete", 
                    command=lambda: controller.showFrame("EditFrame"))
    button.pack()
  
  def updateCPane(self):
    predNames = self.predNames
    predParameters = self.predParameters
    for i in range(len(self.predNames)):
      crntFrame = self.listofCP[i].sub_frame
      for widgets in crntFrame.winfo_children():
        widgets.destroy()
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
      tf(crntFrame, crntTableData, height=len(crntTableData))

  def _createPredicatePanes(self, parent):
    self.listofCP = []
    predNames = self.predNames
    predParameters = self.predParameters
    predicatesPane = Frame(parent)

    predicateHeaders = self.headerText
    for i in range(len(self.predNames)):
      cf = self._createCollapsiblePane(predicatesPane, predicateHeaders[i])
      self.listofCP.append(cf)
      # Parse current predicate propositional data and show as a table
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
      tf(cf.sub_frame, crntTableData, height=len(crntTableData))
    return predicatesPane

  def _createCollapsiblePane(self, parent, headerText):
    predicateCP = cf(parent, text=headerText, relief="raised", borderwidth=1)
    predicateCP.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")
    return predicateCP

  def _parseTableData(self, data, attrName, attrVals):
    tableHeadings = ["timestep"]+list(attrVals.keys())
    if 'function_value' not in attrVals.keys():
      tableHeadings += ["True/False"]
    crntTableData = [tableHeadings]
    if attrName in list(data.keys()):
      crntTableData = crntTableData+data[attrName]
    return crntTableData