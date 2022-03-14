from tkinter import Frame, Button
from components.collapsiblePane import ToggledFrame as cp
from styling import *
from components.tablePane import TablePane

class ViewPane(Frame):
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
                    command=lambda: controller.showFrame("EditPane"))
    button.pack()
  
  def updateCPane(self):
    predNames = self.predNames
    predParameters = self.predParameters
    for i in range(len(self.predNames)):
      crntFrame = self.listofCP[i].sub_frame
      for widgets in crntFrame.winfo_children():
        widgets.destroy()
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
      TablePane(crntFrame, crntTableData, height=len(crntTableData))

  def _createPredicatePanes(self, parent):
    self.listofCP = []
    predNames = self.predNames
    predParameters = self.predParameters
    predicatesPane = Frame(parent)

    predicateHeaders = self.headerText
    for i in range(len(self.predNames)):
      cp = self._createCollapsiblePane(predicatesPane, predicateHeaders[i])
      self.listofCP.append(cp)
      # Parse current predicate propositional data and show as a table
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
      TablePane(cp.sub_frame, crntTableData, height=len(crntTableData))
    return predicatesPane

  def _createCollapsiblePane(self, parent, headerText):
    predicateCP = cp(parent, text=headerText, relief="raised", borderwidth=1)
    predicateCP.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")
    return predicateCP

  def _parseTableData(self, data, attrName, attrVals):
    tableHeadings = ["timestep"]+list(attrVals.keys())+["True/False"]
    crntTableData = [tableHeadings]
    if attrName in list(data.keys()):
      crntTableData = crntTableData+data[attrName]
    return crntTableData