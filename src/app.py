from tkinter import Tk, ttk
from components.collapsiblePane import ToggledFrame as cp
from styling import *
from scripts.KB_CRUD import KnowledgeBaseNode
from components.tablePane import TablePane
from viewPane import ViewPane

class App():
  def __init__(self):
    root = Tk()
    root.geometry(f"{ROOT_SIZE[0]}x{ROOT_SIZE[1]}")

    self.viewPane = ViewPane(root)

    root.mainloop()