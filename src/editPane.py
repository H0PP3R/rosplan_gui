from tkinter import ttk, Frame, Button
from components.collapsiblePane import ToggledFrame as cp
from styling import *
from scripts.KB_CRUD import KnowledgeBaseNode
from components.tablePane import TablePane

class EditPane(Frame):
  def __init__(self, parent, controller):
    Frame.__init__(self, parent)
    self.controller = controller
    self.tableData = None
    self.predicateData = None
    # self.KB = KnowledgeBaseNode(self._callback)
    # self._setInitialData()
    self._createEditPane(self.controller)
  
  def _createEditPane(self, controller):
    print("createEditPane")
    button = Button(self, text ="View", 
                    command=lambda: controller.show_frame("ViewPane"))
    button.pack()
    # self.grid(column=0, row=0)