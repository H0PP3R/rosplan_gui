from tkinter import *
from  tkinter import ttk

# Not fully functional, have to clean data and design
class tablePane(ttk.Frame):
  def __init__(self, parent, data):
    for i in range(len(data)):
      for j in range(len(data[0])):
        self.e = Entry(parent, width=20)
        self.e.grid(row=i, column=j)
        self.e.insert(END, data[i][j])