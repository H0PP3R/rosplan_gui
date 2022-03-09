from tkinter import Label, Frame, Button
from styling import *
from components.tablePane import TablePane
from tkinter.messagebox import askyesno

class EditPane(Frame):
  def __init__(self, parent, controller, tableData, predicateData, headerText):
    Frame.__init__(self, parent)
    self.controller = controller
    self.tableData = tableData
    self.predicateData = predicateData
    self.headerText = headerText
    self.predNames = list(self.predicateData.keys())
    self.predParameters = list(self.predicateData.values())
    self.request = None
    self._populatePane(self.controller)
  
  def _populatePane(self, controller):
    self.mainPane = Frame(self)
    self.mainPane.pack(fill='x')
    Label(self.mainPane, text='Edit the most recent predicate value',
            anchor='w').pack(fill='x')
    self._createTablePanes()
    submitButton = Button(self.mainPane, text="Apply", command=self._confirm).pack()
    switchPaneButton = Button(self.mainPane, text ="View", 
                              command=lambda: controller.showFrame("ViewPane"))
    switchPaneButton.pack()
  
  def _confirm(self):
    ask = askyesno(title='confirmation', message='Confirm your changes')
    if ask:
      self.KB.update(self.request)
  
  def _createTablePanes(self):
    self.listofTP = {}
    predNames = self.predNames
    predParameters = self.predParameters
    
    headings = self.headerText
    for i in range(len(self.predNames)):
      Label(self.mainPane, text=headings[i], 
            borderwidth=2, relief="raised", anchor='w', 
            ).pack(fill='x')
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
      frame = Frame(self.mainPane)
      frame.pack(fill='x')
      TablePane(frame, crntTableData, height=1)
      self.listofTP[predNames[i]] = frame

  def _parseTableData(self, data, attrName, attrVals):
    tableHeadings = ["timestep"]+list(attrVals.keys())+["True/False"]
    crntTableData = [tableHeadings]
    if attrName in list(data.keys()):
      crntTableData = crntTableData+[data[attrName][-1]]
    return crntTableData

  def updateTPane(self):
    predNames = self.predNames
    predParameters = self.predParameters
    for i in range(len(self.predNames)):
      crntFrame = self.listofTP[predNames[i]]
      for widgets in crntFrame.winfo_children():
        widgets.destroy()
      crntTableData = self._parseTableData(self.tableData, predNames[i], predParameters[i])
      TablePane(crntFrame, crntTableData, height=1)