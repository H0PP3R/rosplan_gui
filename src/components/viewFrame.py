from tkinter import Frame, Button, Label

from .collapsibleFrame import CollapsibleFrame as cFrame
from .tableFrame import TableFrame as tFrame

class ViewFrame(Frame):
  '''
  Widget that displays the knowledge base state.
  Auto-refreshes with knowledge base changes
  Extension of the Frame widget.
  '''
  def __init__(self, parent, controller, data):
    '''
    Constructor, sets the parameters as class variables
    @param self: the class itself
    @param parent: the frame in which this widget will be in
    @param controller: controls the order of the frames
    @param data: formatted predicate and proposition data
    '''
    Frame.__init__(self, parent)
    self.listofCP = []
    self.controller = controller
    self.tableData = data['tableData']
    self.predicateData = data['predicateData']
    self.headerText = data['headerText']
    self.predNames = list(self.predicateData.keys())
    self.predParameters = list(self.predicateData.values())
    self._populateFrame()

  def _populateFrame(self):
    '''
    Procedure to create and populate the ViewFrame with widgets
    @param self: the class itself
    '''
    predicatesFrame = self._createPredicateFrames(self)
    predicatesFrame.pack(fill='x')

    button = Button(self, text ='Edit/Delete',
                    command=lambda: self.controller.showFrame('EditFrame'))
    button.pack()

  def updateCFrame(self):
    '''
    Procedure to update the CollapsibleFrames to display new tableData
    @param self: the class itself
    '''
    predNames = self.predNames
    predParameters = self.predParameters
    # destroys the tableFrames within the collapsible frames' subframe
    # and recreate the tableFrames with new tableData
    for i in range(len(self.predNames)):
      crntFrame = self.listofCP[i].getSubFrame()
      for widgets in crntFrame.winfo_children():
        widgets.destroy()
      crntTableData = self._parseTableData(
        self.tableData, predNames[i], predParameters[i]
      )
      tFrame(crntFrame, crntTableData, height=len(crntTableData))

  def _createPredicateFrames(self, parent):
    '''
    Function to create the frame for the predicate frames and returns it
    @param self: the class itself
    @param parent: the frame in which the predicate frames will be in
    @return a frame widget holding all the predicate frames
    '''
    predNames = self.predNames
    predParameters = self.predParameters
    predicatesFrame = Frame(parent)

    predicateHeaders = self.headerText
    firstFluent = True
    for i in range(len(self.predNames)):
      if 'function_value' in self.predicateData[self.predNames[i]] and firstFluent:
        l = Label(predicatesFrame, text='Numeric Fluents')
        l.pack(fill='x')
        firstFluent = False
      cFrame = self._createCollapsibleFrame(predicatesFrame, predicateHeaders[i])
      self.listofCP.append(cFrame)
      # Parse current predicate propositional data and show as a table
      crntTableData = self._parseTableData(
        self.tableData, predNames[i], predParameters[i]
      )
      tFrame(cFrame.getSubFrame(), crntTableData, height=len(crntTableData))
    return predicatesFrame

  def _createCollapsibleFrame(self, parent, headerText):
    '''
    Function that creates the individual predicate frame and returns it
    @param self: the class itself
    @param parent: the frame in which the CollapsibleFrames will be in
    @param headerText: labels for the tableData
    @return CollapsibleFrame with specific headerText label
    '''
    predicateCF = cFrame(parent, text=headerText, relief='raised', borderwidth=1)
    predicateCF.pack(fill='x', expand=1, pady=2, padx=2, anchor='n')
    return predicateCF

  def _parseTableData(self, data, attrName, attrVals):
    '''
    Function that prepares tableData and returns the prepared data
    @param self: the class itself
    @param data: tableData
    @param attrName: string name of attribute
    @param attrVals: dictionary of attribute values
    @return prepared attribute table data
    '''
    tableHeadings = ['timestep']+list(attrVals.keys())
    if 'function_value' not in attrVals.keys():
      tableHeadings += ['True/False']
    crntTableData = [tableHeadings]
    if attrName in list(data.keys()):
      crntTableData = crntTableData+data[attrName]
    return crntTableData
