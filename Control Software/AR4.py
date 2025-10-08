############################################################################
## Version AR4 4.0 #########################################################
############################################################################
""" AR4 - robot control software
    Copyright (c) 2023, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
        * you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
		* Selling robots, robot parts, or any versions of robots or software based on this 
		  work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com
"""
##########################################################################
### VERSION DOC ##########################################################
##########################################################################
''' 
**VERSION 1.0 INITIAL RELEASE
  VERSION 1.1 3/5/22 bug fix, position register function 
  VERSION 1.2 4/21/22 added timeout to ser com
  VERSION 1.3 6/17/22 removed timeout ser com - modified cal file
  VERSION 2.0 10/1/22 added spline lookahead
  VERSION 2.2 11/6/22 added opencv integrated vision tab
  VERSION 3.0 2/3/23 move open loop bypass to teensy / add J8 & J9
  VERSION 3.1 5/1/23 gcode initial development
  VERSION 3.2 6/3/23 remove RoboDK kinematics
  VERSION 3.3 6/4/23 update geometric kinematics
  VERSION 4.0 11/5/23 .txt .ar4 extension, gcode tab, kinematics tab. Initial MK2 release.

'''
##########################################################################
##########################################################################



from multiprocessing.resource_sharer import stop
from os import execv
from tkinter import *
from tkinter.ttk import *
from tkinter import ttk
from tkinter import simpledialog
from ttkthemes import ThemedStyle
from tkinter import messagebox
from PIL import Image, ImageTk
from matplotlib import pyplot as plt
from pygrabber.dshow_graph import FilterGraph
from tkinter import filedialog as fd

import pickle
import serial
import time
import threading
import math
import tkinter.messagebox
import webbrowser
import numpy as np
import datetime
import cv2
import pathlib
import os
from numpy import mean
from os import path


DIR = pathlib.Path(__file__).parent.resolve()

cropping = False


root = Tk()
root.wm_title("AR4 Software Ver 4.0")
root.iconbitmap(r'AR.ico')
root.resizable(width=False, height=False)
root.geometry('1536x792+0+0')
root.runTrue = 0
root.GCrunTrue = 0

def on_closing():
    if messagebox.askokcancel("Close Program", "Do you want to quit?"):
      try:
        command = "CL"
        ser.write(command.encode())
      except:
        print ("foo")
      ser.close()
      root.destroy()

#root.wm_protocol("WM_DELETE_WINDOW", on_closing)


 

global JogStepsStat
JogStepsStat = IntVar()
global J1OpenLoopStat
J1OpenLoopStat = IntVar()
global J2OpenLoopStat
J2OpenLoopStat = IntVar()
global J3OpenLoopStat
J3OpenLoopStat = IntVar()
global J4OpenLoopStat
J4OpenLoopStat = IntVar()
global J5OpenLoopStat
J5OpenLoopStat = IntVar()
global J6OpenLoopStat
J6OpenLoopStat = IntVar()
global DisableWristRot
DisableWristRot = IntVar()
global xboxUse
global curTheme
global J1CalStat
J1CalStat = IntVar()
global J2CalStat
J2CalStat = IntVar()
global J3CalStat
J3CalStat = IntVar()
global J4CalStat
J4CalStat = IntVar()
global J5CalStat
J5CalStat = IntVar()
global J6CalStat
J6CalStat = IntVar()

global J1CalStat2
J1CalStat2 = IntVar()
global J2CalStat2
J2CalStat2 = IntVar()
global J3CalStat2
J3CalStat2 = IntVar()
global J4CalStat2
J4CalStat2 = IntVar()
global J5CalStat2
J5CalStat2 = IntVar()
global J6CalStat2
J6CalStat2 = IntVar()

global IncJogStat
IncJogStat = IntVar()

global fullRot
fullRot = IntVar()

global pick180
pick180 = IntVar()

global pickClosest
pickClosest = IntVar()

global autoBG
autoBG = IntVar()




global SplineTrue;
SplineTrue = False;

global gcodeSpeed;
gcodeSpeed = "10";

global inchTrue;
inchTrue = False;

global moveInProc
moveInProc = 0


#define axis limits in degrees
J1PosLim = 170;
J1NegLim = 170;
J2PosLim = 90;
J2NegLim = 42;
J3PosLim = 52;
J3NegLim = 89;
J4PosLim = 165;
J4NegLim = 165;
J5PosLim = 105;
J5NegLim = 105;
J6PosLim = 155;
J6NegLim = 155;
J7PosLim = 500;
J7NegLim = 0;
J8PosLim = 500;
J8NegLim = 0;
J9PosLim = 500;
J9NegLim = 0;



############################################################################
### DEFINE TABS ############################################################
############################################################################

nb = tkinter.ttk.Notebook(root, width=1536, height=792)
nb.place(x=0, y=0)

tab1 = tkinter.ttk.Frame(nb)
nb.add(tab1, text=' Main Controls ')

tab2 = tkinter.ttk.Frame(nb)
nb.add(tab2, text='  Config Settings  ')

tab3 = tkinter.ttk.Frame(nb)
nb.add(tab3, text='   Kinematics    ')

tab4 = tkinter.ttk.Frame(nb)
nb.add(tab4, text=' Inputs Outputs ')

tab5 = tkinter.ttk.Frame(nb)
nb.add(tab5, text='   Registers    ')

tab6 = tkinter.ttk.Frame(nb)
nb.add(tab6, text='   Vision    ')

tab7 = tkinter.ttk.Frame(nb)
nb.add(tab7, text='    G-Code     ')

tab8 = tkinter.ttk.Frame(nb)
nb.add(tab8, text='      Log      ')

tab9 = tkinter.ttk.Frame(nb)
nb.add(tab9, text='   Info    ')




cam_on = False
cap = None

###############################################################################################################################################################
### STARTUP DEFS ################################################################################################################# COMMUNICATION DEFS ###
###############################################################################################################################################################


def startup():
  #global moveInProc
  moveInProc = 0
  updateParams()
  time.sleep(.1)
  calExtAxis()
  time.sleep(.1)
  sendPos()
  time.sleep(.1)
  requestPos()



###############################################################################################################################################################
### COMMUNICATION DEFS ################################################################################################################# COMMUNICATION DEFS ###
###############################################################################################################################################################

def setCom(): 
  try:
    global ser    
    port = "COM" + comPortEntryField.get()  
    baud = 9600    
    ser = serial.Serial(port,baud)
    almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
    Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
    tab8.ElogView.insert(END, Curtime+" - COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
    time.sleep(.1)
    ser.flushInput()
    startup()
  except:
    almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER", style="Alarm.TLabel")
    almStatusLab2.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER", style="Alarm.TLabel")
    Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
    tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))


def setCom2(): 
  try:
    global ser2    
    port = "COM" + com2PortEntryField.get()  
    baud = 115200    
    ser2 = serial.Serial(port,baud)
    almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
    Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
    tab8.ElogView.insert(END, Curtime+" - COMMUNICATIONS STARTED WITH ARDUINO IO BOARD")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
  except:
    #almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD", style="Alarm.TLabel")
    #almStatusLab2.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD", style="Alarm.TLabel")
    Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
    tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
    
def darkTheme():
  global curTheme
  curTheme = 0
  style = ThemedStyle(root)
  style.set_theme("black")
  style = ttk.Style()
  style.configure("Alarm.TLabel", foreground="IndianRed1", font = ('Arial','10','bold'))
  style.configure("Warn.TLabel", foreground="orange", font = ('Arial','10','bold'))
  style.configure("OK.TLabel", foreground="light green", font = ('Arial','10','bold'))
  style.configure("Jointlim.TLabel", foreground="light blue", font = ('Arial','8'))
  style.configure('AlarmBut.TButton', foreground ='IndianRed1')
  style.configure('Frame1.TFrame', background='white')

def lightTheme():
  global curTheme
  curTheme = 1
  style = ThemedStyle(root)
  style.set_theme("keramik")
  style = ttk.Style()
  style.configure("Alarm.TLabel", foreground="red", font = ('Arial','10','bold'))
  style.configure("Warn.TLabel", foreground="dark orange", font = ('Arial','10','bold'))
  style.configure("OK.TLabel", foreground="green", font = ('Arial','10','bold'))
  style.configure("Jointlim.TLabel", foreground="dark blue", font = ('Arial','8'))
  style.configure('AlarmBut.TButton', foreground ='red')
  style.configure('Frame1.TFrame', background='black')


###############################################################################################################################################################  
### EXECUTION DEFS ######################################################################################################################### EXECUTION DEFS ###  
############################################################################################################################################################### 

def runProg():
  def threadProg():
    global rowinproc
    global stopQueue
    global splineActive
    stopQueue = "0"
    splineActive = "0"
    try:
      curRow = tab1.progView.curselection()[0]
      if (curRow == 0):
        curRow=1
    except:
      curRow=1
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(curRow)
    tab1.runTrue = 1
    while tab1.runTrue == 1:
      if (tab1.runTrue == 0):
        almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
        almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
      else:
        almStatusLab.config(text="PROGRAM RUNNING",  style="OK.TLabel")
        almStatusLab2.config(text="PROGRAM RUNNING",  style="OK.TLabel") 
      rowinproc = 1
      executeRow()
      while rowinproc == 1:
        time.sleep(.1)	  
      selRow = tab1.progView.curselection()[0]
      last = tab1.progView.index('end')
      #for row in range (0,selRow):
        #tab1.progView.itemconfig(row, {'fg': 'dodger blue'})
      #tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
      #for row in range (selRow+1,last):
        #tab1.progView.itemconfig(row, {'fg': 'black'})
      tab1.progView.selection_clear(0, END)
      selRow += 1
      tab1.progView.select_set(selRow)
      curRow += 1
      time.sleep(.1)
      try:
        selRow = tab1.progView.curselection()[0]
        curRowEntryField.delete(0, 'end')
        curRowEntryField.insert(0,selRow)
      except:
        curRowEntryField.delete(0, 'end')
        curRowEntryField.insert(0,"---") 
        tab1.runTrue = 0
        almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
        almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel") 
  t = threading.Thread(target=threadProg)
  t.start()
  
def stepFwd():
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel") 
    executeRow() 
    selRow = tab1.progView.curselection()[0]
    last = tab1.progView.index('end')
    for row in range (0,selRow):
      tab1.progView.itemconfig(row, {'fg': 'dodger blue'})
    tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
    for row in range (selRow+1,last):
      tab1.progView.itemconfig(row, {'fg': 'black'})
    tab1.progView.selection_clear(0, END)
    selRow += 1
    tab1.progView.select_set(selRow)
    try:
      selRow = tab1.progView.curselection()[0]
      curRowEntryField.delete(0, 'end')
      curRowEntryField.insert(0,selRow)
    except:
      curRowEntryField.delete(0, 'end')
      curRowEntryField.insert(0,"---")
 
def stepRev():
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel") 
    executeRow()  
    selRow = tab1.progView.curselection()[0]
    last = tab1.progView.index('end')
    for row in range (0,selRow):
      tab1.progView.itemconfig(row, {'fg': 'black'})
    tab1.progView.itemconfig(selRow, {'fg': 'red'})
    for row in range (selRow+1,last):
      tab1.progView.itemconfig(row, {'fg': 'tomato2'})
    tab1.progView.selection_clear(0, END)
    selRow -= 1
    tab1.progView.select_set(selRow)
    try:
      selRow = tab1.progView.curselection()[0]
      curRowEntryField.delete(0, 'end')
      curRowEntryField.insert(0,selRow)
    except:
      curRowEntryField.delete(0, 'end')
      curRowEntryField.insert(0,"---")  
    
def stopProg():
  global cmdType
  global splineActive
  global stopQueue
  lastProg = ""
  tab1.runTrue = 0
  almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
  almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")  
  
  
def executeRow():
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global calStat
  global rowinproc
  global LineDist
  global Xv
  global Yv
  global Zv
  global commandCalc
  global moveInProc
  global splineActive
  global stopQueue
  selRow = tab1.progView.curselection()[0]
  tab1.progView.see(selRow+2)
  data = list(map(int, tab1.progView.curselection()))
  command=tab1.progView.get(data[0]).decode().strip()
  cmdType=command[:6]
  
  ##Call Program##
  if (cmdType == "Call P"):
    if (moveInProc == 1):
      moveInProc == 2
    tab1.lastRow = tab1.progView.curselection()[0]
    tab1.lastProg = ProgEntryField.get()
    programIndex = command.find("Program -")
    progNum = str(command[programIndex+10:])
    ProgEntryField.delete(0, 'end')
    ProgEntryField.insert(0,progNum)
    callProg(progNum)
    time.sleep(.4) 
    index = 0
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(index) 
  ##Return Program##
  if (cmdType == "Return"):
    if (moveInProc == 1):
      moveInProc == 2
    lastRow = tab1.lastRow
    lastProg = tab1.lastProg
    ProgEntryField.delete(0, 'end')
    ProgEntryField.insert(0,lastProg)
    callProg(lastProg)
    time.sleep(.4) 
    index = 0
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(lastRow)  
  ##Test Limit Switches
  if (cmdType == "Test L"):
    if (moveInProc == 1):
      moveInProc == 2
    command = "TL\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.05)
    response = str(ser.readline().strip(),'utf-8')
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)
  ##Set Encoders 1000
  if (cmdType == "Set En"):
    if (moveInProc == 1):
      moveInProc == 2
    command = "SE\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.05)
    ser.read() 
  ##Read Encoders
  if (cmdType == "Read E"):
    if (moveInProc == 1):
      moveInProc == 2
    command = "RE\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.05)
    response = str(ser.readline().strip(),'utf-8')
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)   
  ##Servo Command##
  if (cmdType == "Servo "):
    if (moveInProc == 1):
      moveInProc == 2
    servoIndex = command.find("number ")
    posIndex = command.find("position: ")
    servoNum = str(command[servoIndex+7:posIndex-4])
    servoPos = str(command[posIndex+10:])
    command = "SV"+servoNum+"P"+servoPos+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser2.write(command.encode())
    ser2.flushInput()
    time.sleep(.1)
    ser2.read() 

  ##If Input On Jump to Tab IO Board##
  if (cmdType == "If On "):
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("Input-")
    tabIndex = command.find("Tab-")
    inputNum = str(command[inputIndex+6:tabIndex-9])
    tabNum = str(command[tabIndex+4:])
    command = "JFX"+inputNum+"T"+tabNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)   
    ser2.write(command.encode())
    ser2.flushInput()
    time.sleep(.1)
    response = str(ser2.readline().strip(),'utf-8')
    if (response == "T"):
      tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
      index = tab1.progView.get(0, "end").index(tabNum)
      index = index-1
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index)
  ##If Input Off Jump to Tab IO Board##
  if (cmdType == "If Off"):
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("Input-")
    tabIndex = command.find("Tab-")
    inputNum = str(command[inputIndex+6:tabIndex-9])
    tabNum = str(command[tabIndex+4:])
    command = "JFX"+inputNum+"T"+tabNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)   
    ser2.write(command.encode())
    ser2.flushInput()
    time.sleep(.1)
    response = str(ser2.readline().strip(),'utf-8')
    if (response == "F"):
      tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
      index = tab1.progView.get(0, "end").index(tabNum)
      index = index-1
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index)


  ##If Input On Jump to Tab Teensy##
  if (cmdType == "TifOn "):
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("Input-")
    tabIndex = command.find("Tab-")
    inputNum = str(command[inputIndex+6:tabIndex-9])
    tabNum = str(command[tabIndex+4:])
    command = "JFX"+inputNum+"T"+tabNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)   
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response == "T"):
      tabNum = ("Tab Number " + str(command[tabIndex+4:])).encode('utf-8')
      index = tab1.progView.get(0, "end").index(tabNum)
      index = index-1
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index)
  ##If Input Off Jump to Tab Teensy##
  if (cmdType == "TifOff"):
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("Input-")
    tabIndex = command.find("Tab-")
    inputNum = str(command[inputIndex+6:tabIndex-9])
    tabNum = str(command[tabIndex+4:])
    command = "JFX"+inputNum+"T"+tabNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)   
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response == "F"):
      tabNum = ("Tab Number " + str(command[tabIndex+4:])).encode('utf-8')
      index = tab1.progView.get(0, "end").index(tabNum)
      index = index-1
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index)

  ##Jump to Row##
  if (cmdType == "Jump T"):
    if (moveInProc == 1):
      moveInProc == 2
    tabIndex = command.find("Tab-")
    tabNum = ("Tab Number " + str(command[tabIndex+4:]) + "\r\n").encode('utf-8')
    index = tab1.progView.get(0, "end").index(tabNum)
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(index)  


  ##Set Output ON Command IO Board##
  if (cmdType == "Out On"):
    if (moveInProc == 1):
      moveInProc == 2
    outputIndex = command.find("Out On = ")
    outputNum = str(command[outputIndex+9:])
    command = "ONX"+outputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser2.write(command.encode())
    ser2.flushInput()
    time.sleep(.1)
    ser2.read() 
  ##Set Output OFF Command IO Board##
  if (cmdType == "Out Of"):
    if (moveInProc == 1):
      moveInProc == 2
    outputIndex = command.find("Out Off = ")
    outputNum = str(command[outputIndex+10:])
    command = "OFX"+outputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser2.write(command.encode())
    ser2.flushInput()
    time.sleep(.1)
    ser2.read() 

  ##Set Output ON Command Teensy##
  if (cmdType == "ToutOn"):
    if (moveInProc == 1):
      moveInProc == 2
    outputIndex = command.find("outOn = ")
    outputNum = str(command[outputIndex+8:])
    command = "ONX"+outputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.read() 
  ##Set Output OFF Command Teensy##
  if (cmdType == "ToutOf"):
    if (moveInProc == 1):
      moveInProc == 2
    outputIndex = command.find("outOff = ")
    outputNum = str(command[outputIndex+9:])
    command = "OFX"+outputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.read() 

  ##Wait Input ON Command IO Board##
  if (cmdType == "Wait I"):
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("Wait Input On = ")
    inputNum = str(command[inputIndex+16:])
    command = "WIN"+inputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser2.write(command.encode())
    ser2.flushInput()
    time.sleep(.1)
    ser2.read() 
  ##Wait Input OFF Command IO Board##
  if (cmdType == "Wait O"):
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("Wait Off Input = ")
    inputNum = str(command[inputIndex+17:])
    command = "WON"+inputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser2.write(command.encode())
    ser2.flushInput()
    time.sleep(.1)
    ser2.read() 

  ##Wait Input ON Command Teensy##
  if (cmdType == "TwaitI"):
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("TwaitInput On = ")
    inputNum = str(command[inputIndex+16:])
    command = "WIN"+inputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.read() 
  ##Wait Input OFF Command Teensy##
  if (cmdType == "TwaitO"):
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("TwaitOff Input = ")
    inputNum = str(command[inputIndex+16:])
    command = "WON"+inputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.read()   


  ##Wait Time Command##
  if (cmdType == "Wait T"):
    if (moveInProc == 1):
      moveInProc == 2
    timeIndex = command.find("Wait Time = ")
    timeSeconds = str(command[timeIndex+12:])
    command = "WTS"+timeSeconds+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.read() 

  ##Set Register##  
  if (cmdType == "Regist"):
    if (moveInProc == 1):
      moveInProc == 2
    regNumIndex = command.find("Register ")
    regEqIndex = command.find(" = ")
    regNumVal = str(command[regNumIndex+9:regEqIndex])
    regEntry = "R"+regNumVal+"EntryField"
    testOper = str(command[regEqIndex+3:regEqIndex+5])
    if (testOper == "++"):
      regCEqVal = str(command[regEqIndex+5:])
      curRegVal = eval(regEntry).get()
      regEqVal = str(int(regCEqVal)+int(curRegVal))      
    elif (testOper == "--"):
      regCEqVal = str(command[regEqIndex+5:])
      curRegVal = eval(regEntry).get()
      regEqVal = str(int(curRegVal)-int(regCEqVal))
    else:
      regEqVal = str(command[regEqIndex+3:])    
    eval(regEntry).delete(0, 'end')
    eval(regEntry).insert(0,regEqVal)
  ##Set Position Register##  
  if (cmdType == "Positi"):
    if (moveInProc == 1):
      moveInProc == 2
    regNumIndex = command.find("Position Register ")
    regElIndex = command.find("Element")
    regEqIndex = command.find(" = ")
    regNumVal = str(command[regNumIndex+18:regElIndex-1])
    regNumEl = str(command[regElIndex+8:regEqIndex])
    regEntry = "SP_"+regNumVal+"_E"+regNumEl+"_EntryField"
    testOper = str(command[regEqIndex+3:regEqIndex+5])
    if (testOper == "++"):
      regCEqVal = str(command[regEqIndex+4:])
      curRegVal = eval(regEntry).get()
      regEqVal = str(float(regCEqVal)+float(curRegVal))      
    elif (testOper == "--"):
      regCEqVal = str(command[regEqIndex+5:])
      curRegVal = eval(regEntry).get()
      regEqVal = str(float(curRegVal)-float(regCEqVal))
    else:
      regEqVal = str(command[regEqIndex+3:])    
    eval(regEntry).delete(0, 'end')
    eval(regEntry).insert(0,regEqVal)
  ##If Register Jump to Row##
  if (cmdType == "If Reg"):
    if (moveInProc == 1):
      moveInProc == 2
    regIndex = command.find("If Register ")
    regEqIndex = command.find(" = ")
    regJmpIndex = command.find(" Jump to Tab ")    
    regNum = str(command[regIndex+12:regEqIndex])
    regEq = str(command[regEqIndex+3:regJmpIndex])
    tabNum = str(command[regJmpIndex+13:])
    regEntry = "R"+regNum+"EntryField"
    curRegVal = eval(regEntry).get()
    if (curRegVal == regEq):
      tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
      index = tab1.progView.get(0, "end").index(tabNum)
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index)  

  ##Calibrate Command##   
  if (cmdType == "Calibr"):
    if (moveInProc == 1):
      moveInProc == 2
    calRobotAll()
    if (calStat == 0):
      stopProg()

  ##Set tool##  
  if (cmdType == "Tool S"): 
    if (moveInProc == 1):
      moveInProc == 2
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel") 
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    xVal = command[xIndex+3:yIndex]
    yVal = command[yIndex+3:zIndex]
    zVal = command[zIndex+3:rzIndex]
    rzVal = command[rzIndex+4:ryIndex]
    ryVal = command[ryIndex+4:rxIndex]
    rxVal = command[rxIndex+4:]
    TFxEntryField.delete(0,'end')
    TFyEntryField.delete(0,'end')
    TFzEntryField.delete(0,'end')
    TFrzEntryField.delete(0,'end')
    TFryEntryField.delete(0,'end')
    TFrxEntryField.delete(0,'end')
    TFxEntryField.insert(0,str(xVal))
    TFyEntryField.insert(0,str(yVal))
    TFzEntryField.insert(0,str(zVal))
    TFrzEntryField.insert(0,str(rzVal))
    TFryEntryField.insert(0,str(ryVal))
    TFrxEntryField.insert(0,str(rxVal))
    command = "TF"+"A"+xVal+"B"+yVal+"C"+zVal+"D"+rzVal+"E"+ryVal+"F"+rxVal+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.read()
     
  
  ##Move J Command##  
  if (cmdType == "Move J"): 
    if (moveInProc == 0):
      moveInProc == 1
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")	
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    xVal = command[xIndex+3:yIndex]
    yVal = command[yIndex+3:zIndex]
    zVal = command[zIndex+3:rzIndex]
    rzVal = command[rzIndex+4:ryIndex]
    ryVal = command[ryIndex+4:rxIndex]
    rxVal = command[rxIndex+4:J7Index]
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    #ser.read()
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response) 


 ##Offs J Command##  
  if (cmdType == "OFF J "): 
    if (moveInProc == 0):
      moveInProc == 1
    SPnewInex = command.find("[ PR: ")  
    SPendInex = command.find(" ] [")
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")	
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    SP = str(command[SPnewInex+6:SPendInex])
    cx = eval("SP_"+SP+"_E1_EntryField").get()
    cy = eval("SP_"+SP+"_E2_EntryField").get()
    cz = eval("SP_"+SP+"_E3_EntryField").get()
    crz = eval("SP_"+SP+"_E4_EntryField").get()
    cry = eval("SP_"+SP+"_E5_EntryField").get()
    crx = eval("SP_"+SP+"_E6_EntryField").get()
    xVal = str(float(cx) + float(command[xIndex+3:yIndex]))
    yVal = str(float(cy) + float(command[yIndex+3:zIndex]))
    zVal = str(float(cz) + float(command[zIndex+3:rzIndex]))
    rzVal = str(float(crz) + float(command[rzIndex+4:ryIndex]))
    ryVal = str(float(cry) + float(command[ryIndex+4:rxIndex]))
    rxVal = str(float(crx) + float(command[rxIndex+4:J7Index]))
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response)  

  ##Move Vis Command##  
  if (cmdType == "Move V"): 
    if (moveInProc == 0):
      moveInProc == 1
    SPnewInex = command.find("[ PR: ")  
    SPendInex = command.find(" ] [")
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")	
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    SP = str(command[SPnewInex+6:SPendInex])
    cx = eval("SP_"+SP+"_E1_EntryField").get()
    cy = eval("SP_"+SP+"_E2_EntryField").get()
    cz = eval("SP_"+SP+"_E3_EntryField").get()
    crz = eval("SP_"+SP+"_E4_EntryField").get()
    cry = eval("SP_"+SP+"_E5_EntryField").get()
    crx = eval("SP_"+SP+"_E6_EntryField").get()
    xVal = str(float(cx) + float(VisRetXrobEntryField.get()))
    yVal = str(float(cy) + float(VisRetYrobEntryField.get()))
    zVal = str(float(cz) + float(command[zIndex+3:rzIndex]))
    rzVal = str(float(crz) + float(command[rzIndex+4:ryIndex]))
    ryVal = str(float(cry) + float(command[ryIndex+4:rxIndex]))
    rxVal = str(float(crx) + float(command[rxIndex+4:J7Index]))
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    visRot = VisRetAngleEntryField.get()
    LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
    command = "MV"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Vr"+visRot+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response)      

  ##Move PR Command##  
  if (cmdType == "Move P"): 
    if (moveInProc == 0):
      moveInProc == 1
    SPnewInex = command.find("[ PR: ")  
    SPendInex = command.find(" ] [")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")		
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    SP = str(command[SPnewInex+6:SPendInex])
    cx = eval("SP_"+SP+"_E1_EntryField").get()
    cy = eval("SP_"+SP+"_E2_EntryField").get()
    cz = eval("SP_"+SP+"_E3_EntryField").get()
    crz = eval("SP_"+SP+"_E4_EntryField").get()
    cry = eval("SP_"+SP+"_E5_EntryField").get()
    crx = eval("SP_"+SP+"_E6_EntryField").get()
    xVal = str(float(cx))
    yVal = str(float(cy))
    zVal = str(float(cz))
    rzVal = str(float(crz))
    ryVal = str(float(cry))
    rxVal = str(float(crx))
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response)  

  ##OFFS PR Command##  
  if (cmdType == "OFF PR"): 
    if (moveInProc == 0):
      moveInProc == 1
    SPnewInex = command.find("[ PR: ")  
    SPendInex = command.find(" ] offs")
    SP2newInex = command.find("[ *PR: ")  
    SP2endInex = command.find(" ]  [")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    SP = str(command[SPnewInex+6:SPendInex])
    SP2 = str(command[SP2newInex+7:SP2endInex])
    xVal = str(float(eval("SP_"+SP+"_E1_EntryField").get()) + float(eval("SP_"+SP2+"_E1_EntryField").get()))
    yVal = str(float(eval("SP_"+SP+"_E2_EntryField").get()) + float(eval("SP_"+SP2+"_E2_EntryField").get()))
    zVal = str(float(eval("SP_"+SP+"_E3_EntryField").get()) + float(eval("SP_"+SP2+"_E3_EntryField").get()))
    rzVal = str(float(eval("SP_"+SP+"_E4_EntryField").get()) + float(eval("SP_"+SP2+"_E4_EntryField").get()))
    ryVal = str(float(eval("SP_"+SP+"_E5_EntryField").get()) + float(eval("SP_"+SP2+"_E5_EntryField").get()))
    rxVal = str(float(eval("SP_"+SP+"_E6_EntryField").get()) + float(eval("SP_"+SP2+"_E6_EntryField").get()))	
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response) 

  ##Move L Command##  
  if (cmdType == "Move L"): 
    if (moveInProc == 0):
      moveInProc == 1
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    RoundingIndex = command.find(" Rnd ")
    WristConfIndex = command.find(" $")
    xVal = command[xIndex+3:yIndex]
    yVal = command[yIndex+3:zIndex]
    zVal = command[zIndex+3:rzIndex]
    rzVal = command[rzIndex+4:ryIndex]
    if (np.sign(float(rzVal)) != np.sign(float(RzcurPos))):
      rzVal=str(float(rzVal)*-1)
    ryVal = command[ryIndex+4:rxIndex]
    rxVal = command[rxIndex+4:J7Index]
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:RoundingIndex]
    Rounding = command[RoundingIndex+5:WristConfIndex]
    WC = command[WristConfIndex+3:]
    LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
    DisWrist = str(DisableWristRot.get())
    command = "ML"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+WC+"Lm"+LoopMode+"Q"+DisWrist+"\n"
    
    #tab8.ElogView.insert(END, command)
    #value=tab8.ElogView.get(0,END)
    #pickle.dump(value,open("ErrorLog","wb"))
    start = time.time()
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    end = time.time()
    #manEntryField.delete(0, 'end')
    #manEntryField.insert(0,end-start)  

    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response)

  ##Move R Command##  
  if (cmdType == "Move R"):
    if (moveInProc == 0):
      moveInProc == 1 
    J1Index = command.find(" J1 ")
    J2Index = command.find(" J2 ")
    J3Index = command.find(" J3 ")
    J4Index = command.find(" J4 ")
    J5Index = command.find(" J5 ")
    J6Index = command.find(" J6 ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    J1Val = command[J1Index+4:J2Index]
    J2Val = command[J2Index+4:J3Index]
    J3Val = command[J3Index+4:J4Index]
    J4Val = command[J4Index+4:J5Index]
    J5Val = command[J5Index+4:J6Index]
    J6Val = command[J6Index+4:J7Index]
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
    command = "RJ"+"A"+J1Val+"B"+J2Val+"C"+J3Val+"D"+J4Val+"E"+J5Val+"F"+J6Val+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response) 

      
  ##Move A Command##  
  if (cmdType == "Move A"):
    if (moveInProc == 0):
      moveInProc == 1
    subCmd=command[:10]
    if (subCmd == "Move A End"):
      almStatusLab.config(text="Move A must start with a Mid followed by End", style="Alarm.TLabel")
      almStatusLab2.config(text="Move A must start with a Mid followed by End", style="Alarm.TLabel")
    else:
      xIndex = command.find(" X ")
      yIndex = command.find(" Y ")
      zIndex = command.find(" Z ")
      rzIndex = command.find(" Rz ")
      ryIndex = command.find(" Ry ")
      rxIndex = command.find(" Rx ")
      trIndex = command.find(" Tr ")	
      SpeedIndex = command.find(" S")
      ACCspdIndex = command.find(" Ac ")
      DECspdIndex = command.find(" Dc ")
      ACCrampIndex = command.find(" Rm ")
      WristConfIndex = command.find(" $")
      xVal = command[xIndex+3:yIndex]
      yVal = command[yIndex+3:zIndex]
      zVal = command[zIndex+3:rzIndex]
      rzVal = command[rzIndex+4:ryIndex]
      ryVal = command[ryIndex+4:rxIndex]
      rxVal = command[rxIndex+4:trIndex]
      trVal = command[trIndex+4:SpeedIndex]
      speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
      Speed = command[SpeedIndex+4:ACCspdIndex]
      ACCspd = command[ACCspdIndex+4:DECspdIndex]
      DECspd = command[DECspdIndex+4:ACCrampIndex]
      ACCramp = command[ACCrampIndex+4:WristConfIndex]
      WC = command[WristConfIndex+3:]
      TCX = 0
      TCY = 0 
      TCZ = 0
      TCRx = 0
      TCRy = 0
      TCRz = 0
      ##read next row for End position	
      curRow = tab1.progView.curselection()[0]
      selRow = tab1.progView.curselection()[0]
      last = tab1.progView.index('end')
      for row in range (0,selRow):
        tab1.progView.itemconfig(row, {'fg': 'dodger blue'})
      tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
      for row in range (selRow+1,last):
        tab1.progView.itemconfig(row, {'fg': 'black'})
      tab1.progView.selection_clear(0, END)
      selRow += 1
      tab1.progView.select_set(selRow)
      curRow += 1
      selRow = tab1.progView.curselection()[0]
      tab1.progView.see(selRow+2)
      data = list(map(int, tab1.progView.curselection()))
      command=tab1.progView.get(data[0]).decode()
      xIndex = command.find(" X ")
      yIndex = command.find(" Y ")
      zIndex = command.find(" Z ")
      rzIndex = command.find(" Rz ")
      ryIndex = command.find(" Ry ")
      rxIndex = command.find(" Rx ")
      trIndex = command.find(" Tr ")	
      SpeedIndex = command.find(" S")
      ACCspdIndex = command.find(" Ac ")
      DECspdIndex = command.find(" Dc ")
      ACCrampIndex = command.find(" Rm ")
      WristConfIndex = command.find(" $")
      Xend = command[xIndex+3:yIndex]
      Yend = command[yIndex+3:zIndex]
      Zend = command[zIndex+3:rzIndex]
      rzVal = command[rzIndex+4:ryIndex]
      ryVal = command[ryIndex+4:rxIndex]
      rxVal = command[rxIndex+4:trIndex]
      trVal = command[trIndex+4:SpeedIndex]
      speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
      Speed = command[SpeedIndex+4:ACCspdIndex]
      ACCspd = command[ACCspdIndex+4:DECspdIndex]
      DECspd = command[DECspdIndex+4:ACCrampIndex]
      ACCramp = command[ACCrampIndex+4:WristConfIndex]
      WC = command[WristConfIndex+3:]
      TCX = 0
      TCY = 0 
      TCZ = 0
      TCRx = 0
      TCRy = 0
      TCRz = 0
      #move arc command
      LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
      command = "MA"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Ex"+Xend+"Ey"+Yend+"Ez"+Zend+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      ser.write(command.encode())
      ser.flushInput()
      time.sleep(.1)
      response = str(ser.readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response) 

  ##Move C Command##  
  if (cmdType == "Move C"):
    if (moveInProc == 0):
      moveInProc == 1
    subCmd=command[:10]
    if (subCmd == "Move C Sta" or subCmd == "Move C Pla"):
      almStatusLab.config(text="Move C must start with a Center followed by Start & Plane", style="Alarm.TLabel")
      almStatusLab2.config(text="Move C must start with a Center followed by Start & Plane", style="Alarm.TLabel")
    else:
      xIndex = command.find(" X ")
      yIndex = command.find(" Y ")
      zIndex = command.find(" Z ")
      rzIndex = command.find(" Rz ")
      ryIndex = command.find(" Ry ")
      rxIndex = command.find(" Rx ")
      trIndex = command.find(" Tr ")	
      SpeedIndex = command.find(" S")
      ACCspdIndex = command.find(" Ac ")
      DECspdIndex = command.find(" Dc ")
      ACCrampIndex = command.find(" Rm ")
      WristConfIndex = command.find(" $")
      xVal = command[xIndex+3:yIndex]
      yVal = command[yIndex+3:zIndex]
      zVal = command[zIndex+3:rzIndex]
      rzVal = command[rzIndex+4:ryIndex]
      ryVal = command[ryIndex+4:rxIndex]
      rxVal = command[rxIndex+4:trIndex]
      trVal = command[trIndex+4:SpeedIndex]
      speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
      Speed = command[SpeedIndex+4:ACCspdIndex]
      ACCspd = command[ACCspdIndex+4:DECspdIndex]
      DECspd = command[DECspdIndex+4:ACCrampIndex]
      ACCramp = command[ACCrampIndex+4:WristConfIndex]
      WC = command[WristConfIndex+3:]
      TCX = 0
      TCY = 0 
      TCZ = 0
      TCRx = 0
      TCRy = 0
      TCRz = 0
      ##read next row for Mid position	
      curRow = tab1.progView.curselection()[0]
      selRow = tab1.progView.curselection()[0]
      last = tab1.progView.index('end')
      for row in range (0,selRow):
        tab1.progView.itemconfig(row, {'fg': 'dodger blue'})
      tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
      for row in range (selRow+1,last):
        tab1.progView.itemconfig(row, {'fg': 'black'})
      tab1.progView.selection_clear(0, END)
      selRow += 1
      tab1.progView.select_set(selRow)
      curRow += 1
      selRow = tab1.progView.curselection()[0]
      tab1.progView.see(selRow+2)
      data = list(map(int, tab1.progView.curselection()))
      command=tab1.progView.get(data[0]).decode()
      xIndex = command.find(" X ")
      yIndex = command.find(" Y ")
      zIndex = command.find(" Z ")
      Xmid = command[xIndex+3:yIndex]
      Ymid = command[yIndex+3:zIndex]
      Zmid = command[zIndex+3:rzIndex]
      ##read next row for End position	
      curRow = tab1.progView.curselection()[0]
      selRow = tab1.progView.curselection()[0]
      last = tab1.progView.index('end')
      for row in range (0,selRow):
        tab1.progView.itemconfig(row, {'fg': 'dodger blue'})
      tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
      for row in range (selRow+1,last):
        tab1.progView.itemconfig(row, {'fg': 'black'})
      tab1.progView.selection_clear(0, END)
      selRow += 1
      tab1.progView.select_set(selRow)
      curRow += 1
      selRow = tab1.progView.curselection()[0]
      tab1.progView.see(selRow+2)
      data = list(map(int, tab1.progView.curselection()))
      command=tab1.progView.get(data[0]).decode()
      xIndex = command.find(" X ")
      yIndex = command.find(" Y ")
      zIndex = command.find(" Z ")
      Xend = command[xIndex+3:yIndex]
      Yend = command[yIndex+3:zIndex]
      Zend = command[zIndex+3:rzIndex]
      #move j to the beginning (second or mid point is start of circle)
      LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
      command = "MJ"+"X"+Xmid+"Y"+Ymid+"Z"+Zmid+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
      ser.write(command.encode())
      ser.flushInput()
      time.sleep(.1)
      response = str(ser.readline().strip(),'utf-8')
      #move circle command
      start = time.time()
      LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
      command = "MC"+"Cx"+xVal+"Cy"+yVal+"Cz"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Bx"+Xmid+"By"+Ymid+"Bz"+Zmid+"Px"+Xend+"Py"+Yend+"Pz"+Zend+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      ser.write(command.encode())
      ser.flushInput()
      time.sleep(.1)
      response = str(ser.readline().strip(),'utf-8')
      end = time.time()
      #manEntryField.delete(0, 'end')
      #manEntryField.insert(0,end-start) 
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response) 

  ##Start Spline
  if (cmdType == "Start "):
    splineActive = "1"
    if (moveInProc == 1):
      moveInProc == 2
    command = "SL\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.read() 

  ##End Spline
  if (cmdType == "End Sp"):
    splineActive = "0"
    if(stopQueue == "1"):
      stopQueue = "0"
      stop()
    if (moveInProc == 1):
      moveInProc == 2
    command = "SS\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response) 

  ##Camera On
  if(cmdType == "Cam On"):
    if (moveInProc == 1):
      moveInProc == 2
    start_vid()

  ##Camera Off
  if(cmdType == "Cam Of"):
    if (moveInProc == 1):
      moveInProc == 2
    stop_vid()  

  ##Vision Find
  if(cmdType == "Vis Fi"):
    #if (moveInProc == 1):
      #moveInProc == 2
    templateIndex = command.find("Vis Find - ")
    bgColorIndex = command.find(" - BGcolor ")
    scoreIndex = command.find(" Score ")
    passIndex = command.find(" Pass ")
    failIndex = command.find(" Fail ")
    template = command[templateIndex+11:bgColorIndex]
    checkBG = command[bgColorIndex+11:scoreIndex]
    if(checkBG == "(Auto)"):
      background = "Auto"
    else:  
      background = eval(command[bgColorIndex+11:scoreIndex])
    min_score = float(command[scoreIndex+7:passIndex])*.01
    take_pic()
    status = visFind(template,min_score,background)
    if (status == "pass"):
      tabNum = ("Tab Number " + str(command[passIndex+6:failIndex]) + "\r\n").encode('utf-8')
      index = tab1.progView.get(0, "end").index(tabNum)
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index)  
    elif (status == "fail"): 
      tabNum = ("Tab Number " + str(command[failIndex+6:]) + "\r\n").encode('utf-8')
      index = tab1.progView.get(0, "end").index(tabNum)
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index) 

    
  rowinproc = 0
  




  
##############################################################################################################################################################
### BUTTON JOGGING DEFS ############################################################################################################## BUTTON JOGGING DEFS ###
##############################################################################################################################################################  
 

def xbox():
  def threadxbox():
    from inputs import get_gamepad
    global xboxUse
    jogMode = 1
    if xboxUse == 0:
      xboxUse = 1
      mainMode = 1
      jogMode = 1
      grip = 0
      almStatusLab.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
      almStatusLab2.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
      xbcStatusLab.config(text='Xbox ON', )
      ChgDis(2)
    else:
      xboxUse = 0
      almStatusLab.config(text='XBOX CONTROLLER OFF', style="Warn.TLabel")
      almStatusLab2.config(text='XBOX CONTROLLER OFF', style="Warn.TLabel")
      xbcStatusLab.config(text='Xbox OFF', )
    while xboxUse == 1:
      try:
      #if (TRUE):
        events = get_gamepad()
        for event in events:
          ##DISTANCE
          if (event.code == 'ABS_RZ' and event.state >= 100):
            ChgDis(0)
          elif (event.code == 'ABS_Z' and event.state >= 100): 
            ChgDis(1)
          ##SPEED
          elif (event.code == 'BTN_TR' and event.state == 1): 
            ChgSpd(0)
          elif (event.code == 'BTN_TL' and event.state == 1): 
            ChgSpd(1)
          ##JOINT MODE
          elif (event.code == 'BTN_WEST' and event.state == 1): 
            if mainMode != 1:
              mainMode = 1
              jogMode = 1
              almStatusLab.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
            else:                
              jogMode +=1        
            if jogMode == 2:
              almStatusLab.config(text='JOGGING JOINTS 3 & 4', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING JOINTS 3 & 4', style="Warn.TLabel")
            elif jogMode == 3:
              almStatusLab.config(text='JOGGING JOINTS 5 & 6', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING JOINTS 5 & 6', style="Warn.TLabel")
            elif jogMode == 4:
              jogMode = 1
              almStatusLab.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
          ##JOINT JOG
          elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 1): 
            J1jogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 1): 
            J1jogPos(float(incrementEntryField.get()))
          elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 1): 
            J2jogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 1): 
            J2jogPos(float(incrementEntryField.get()))           
          elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 2): 
            J3jogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 2): 
            J3jogPos(float(incrementEntryField.get()))
          elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 2): 
            J4jogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 2): 
            J4jogPos(float(incrementEntryField.get()))           
          elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 3): 
            J5jogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 3): 
            J5jogPos(float(incrementEntryField.get()))
          elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 3): 
           J6jogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 3): 
            J6jogPos(float(incrementEntryField.get()))                      
         ##CARTESIAN DIR MODE
          elif (event.code == 'BTN_SOUTH' and event.state == 1): 
            if mainMode != 2:
              mainMode = 2
              jogMode = 1
              almStatusLab.config(text='JOGGING X & Y AXIS', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING X & Y AXIS', style="Warn.TLabel")
            else:                
              jogMode +=1        
            if jogMode == 2:
              almStatusLab.config(text='JOGGING Z AXIS', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING Z AXIS', style="Warn.TLabel")
            elif jogMode == 3:
              jogMode = 1
              almStatusLab.config(text='JOGGING X & Y AXIS', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING X & Y AXIS', style="Warn.TLabel")
          ##CARTESIAN DIR JOG
          elif (mainMode == 2 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 1): 
            XjogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 2 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 1): 
            XjogPos(float(incrementEntryField.get()))
          elif (mainMode == 2 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 1): 
            YjogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 2 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 1): 
            YjogPos(float(incrementEntryField.get()))           
          elif (mainMode == 2 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 2): 
            ZjogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 2 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 2): 
            ZjogPos(float(incrementEntryField.get()))                          
         ##CARTESIAN ORIENTATION MODE
          elif (event.code == 'BTN_EAST' and event.state == 1): 
            if mainMode != 3:
              mainMode = 3
              jogMode = 1
              almStatusLab.config(text='JOGGING Rx & Ry AXIS', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING Rx & Ry AXIS', style="Warn.TLabel")
            else:                
              jogMode +=1        
            if jogMode == 2:
              almStatusLab.config(text='JOGGING Rz AXIS', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING Rz AXIS', style="Warn.TLabel")
            elif jogMode == 3:
              jogMode = 1
              almStatusLab.config(text='JOGGING Rx & Ry AXIS', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING Rx & Ry AXIS', style="Warn.TLabel")
          ##CARTESIAN ORIENTATION JOG
          elif (mainMode == 3 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 1): 
            RxjogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 3 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 1): 
            RxjogPos(float(incrementEntryField.get()))
          elif (mainMode == 3 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 1): 
            RyjogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 3 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 1): 
            RyjogPos(float(incrementEntryField.get()))           
          elif (mainMode == 3 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 2): 
            RzjogNeg(float(incrementEntryField.get()))    
          elif (mainMode == 3 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 2): 
            RzjogPos(float(incrementEntryField.get()))
          ##J7 MODE
          elif (event.code == 'BTN_START' and event.state == 1): 
            mainMode = 4
            almStatusLab.config(text='JOGGING TRACK', style="Warn.TLabel")
            almStatusLab2.config(text='JOGGING TRACK', style="Warn.TLabel")
          ##TRACK JOG
          elif (mainMode == 4 and event.code == 'ABS_HAT0X' and event.state == 1): 
            J7jogPos(float(incrementEntryField.get()))    
          elif (mainMode == 4 and event.code == 'ABS_HAT0X' and event.state == -1): 
            J7jogNeg(float(incrementEntryField.get()))                   
          ##TEACH POS          
          elif (event.code == 'BTN_NORTH' and event.state == 1): 
            teachInsertBelSelected()
          ##GRIPPER         
          elif (event.code == 'BTN_SELECT' and event.state == 1): 
            if grip == 0:
              grip = 1
              outputNum = DO1offEntryField.get()
              command = "OFX"+outputNum+"\n"
              ser2.write(command.encode())
              ser2.flushInput()
              time.sleep(.1)
              ser2.read() 
            else:
              grip = 0
              outputNum = DO1onEntryField.get()
              command = "ONX"+outputNum+"\n"
              ser2.write(command.encode())
              ser2.flushInput()
              time.sleep(.1)
              ser2.read()     
              time.sleep(.1)
          else:
            pass   
      except:
      #else:
        almStatusLab.config(text='XBOX CONTROLLER NOT RESPONDING', style="Alarm.TLabel")
        almStatusLab2.config(text='XBOX CONTROLLER NOT RESPONDING', style="Alarm.TLabel")        
  t = threading.Thread(target=threadxbox)
  t.start()



  
def ChgDis(val):
  curSpd = int(incrementEntryField.get())
  if curSpd >=100 and val == 0:
    curSpd = 100 
  elif curSpd < 5 and val == 0:  
    curSpd += 1
  elif val == 0:
    curSpd += 5   
  if curSpd <=1 and val == 1:
    curSpd = 1 
  elif curSpd <= 5 and val == 1:  
    curSpd -= 1
  elif val == 1:
    curSpd -= 5
  elif val == 2:
    curSpd = 5  
  incrementEntryField.delete(0, 'end')
  incrementEntryField.insert(0,str(curSpd))

  time.sleep(.3)  


def ChgSpd(val):
  curSpd = int(speedEntryField.get())
  if curSpd >=100 and val == 0:
    curSpd = 100 
  elif curSpd < 5 and val == 0:  
    curSpd += 1
  elif val == 0:
    curSpd += 5   
  if curSpd <=1 and val == 1:
    curSpd = 1 
  elif curSpd <= 5 and val == 1:  
    curSpd -= 1
  elif val == 1:
    curSpd -= 5
  elif val == 2:
    curSpd = 5  
  speedEntryField.delete(0, 'end')    
  speedEntryField.insert(0,str(curSpd))  
 
def J1jogNeg(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+str(float(J1AngCur)-value)+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 

def J1jogPos(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+str(float(J1AngCur)+value)+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 

def J2jogNeg(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+str(float(J2AngCur)-value)+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J2jogPos(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+str(float(J2AngCur)+value)+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J3jogNeg(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+str(float(J3AngCur)-value)+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J3jogPos(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+str(float(J3AngCur)+value)+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J4jogNeg(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+str(float(J4AngCur)-value)+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J4jogPos(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+str(float(J4AngCur)+value)+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  

def J5jogNeg(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+str(float(J5AngCur)-value)+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J5jogPos(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+str(float(J5AngCur)+value)+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  

def J6jogNeg(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+str(float(J6AngCur)-value)+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J6jogPos(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+str(float(J6AngCur)+value)+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 




def J7jogNeg(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(float(J7PosCur)-value)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J7jogPos(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(float(J7PosCur)+value)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 



def J8jogNeg(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(float(J8PosCur)-value)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)



def J8jogPos(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(float(J8PosCur)+value)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  


def J9jogNeg(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(float(J9PosCur)-value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)



def J9jogPos(value):
  global xboxUse
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7PosCur
  global J8PosCur
  global J9PosCur
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(float(J9PosCur)+value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)        




def LiveJointJog(value):
  global xboxUse
  almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
  almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  checkSpeedVals()
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "LJ"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  ser.read()






def LiveCarJog(value):
  global xboxUse
  almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
  almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  checkSpeedVals()
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "LC"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  ser.read()


  

def LiveToolJog(value):
  global xboxUse
  almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
  almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  checkSpeedVals()
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "LT"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  ser.read()  


def StopJog(self):
  command = "S\n"
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 0):
    ser.write(command.encode()) 
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)    
    else:
      displayPosition(response)


def J7jogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(float(J7PosCur)-value)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J7jogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(float(J7PosCur)+value)+"J8"+str(J8PosCur)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 


def J8jogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(float(J8PosCur)-value)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J8jogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(float(J8PosCur)+value)+"J9"+str(J9PosCur)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 



def J9jogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(float(J9PosCur)-value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J9jogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to percent
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"J7"+str(J7PosCur)+"J8"+str(J8PosCur)+"J9"+str(float(J9PosCur)+value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)     
    


def XjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = str(float(XcurPos) - value)
  yVal = YcurPos
  zVal = ZcurPos
  rzVal = RzcurPos
  ryVal = RycurPos
  rxVal = RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)  

def YjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = str(float(YcurPos) - value)
  zVal = ZcurPos
  rzVal = RzcurPos
  ryVal = RycurPos
  rxVal = RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response) 

def ZjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = YcurPos
  zVal = str(float(ZcurPos) - value)
  rzVal = RzcurPos
  ryVal = RycurPos
  rxVal = RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)  

def RxjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = YcurPos
  zVal = ZcurPos
  rzVal = RzcurPos
  ryVal = RycurPos
  rxVal =  str(float(RxcurPos) - value)
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)  

def RyjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = YcurPos
  zVal = ZcurPos
  rzVal = RzcurPos
  ryVal = str(float(RycurPos) - value)
  rxVal =  RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)  

def RzjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = YcurPos
  zVal = ZcurPos
  rzVal =  str(float(RzcurPos) - value)
  ryVal = RycurPos
  rxVal = RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)

def XjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = str(float(XcurPos) + value)
  yVal = YcurPos
  zVal = ZcurPos
  rzVal = RzcurPos
  ryVal = RycurPos
  rxVal = RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)  

def YjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = str(float(YcurPos) + value)
  zVal = ZcurPos
  rzVal = RzcurPos
  ryVal = RycurPos
  rxVal = RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response) 


def ZjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = YcurPos
  zVal = str(float(ZcurPos) + value)
  rzVal = RzcurPos
  ryVal = RycurPos
  rxVal = RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)  

def RxjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = YcurPos
  zVal = ZcurPos
  rzVal = RzcurPos
  ryVal = RycurPos
  rxVal =  str(float(RxcurPos) + value)
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)  

def RyjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = YcurPos
  zVal = ZcurPos
  rzVal = RzcurPos
  ryVal = str(float(RycurPos) + value)
  rxVal =  RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)

def RzjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  xVal = XcurPos
  yVal = YcurPos
  zVal = ZcurPos
  rzVal =  str(float(RzcurPos) + value)
  ryVal = RycurPos
  rxVal = RxcurPos
  j7Val = str(J7PosCur)
  j8Val = str(J8PosCur)
  j9Val = str(J9PosCur)
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)

   
  
def TXjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTX1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  

def TYjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTY1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  

def TZjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTZ1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  


def TRxjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTW1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  

def TRyjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTP1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 

def TRzjogNeg(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTR1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def TXjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTX0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  

def TYjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTY0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  

def TZjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTZ0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  

def TRxjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTW0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  

def TRyjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTP0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 

def TRzjogPos(value):
  global xboxUse
  checkSpeedVals()
  if xboxUse != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "JTR0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)


  
  
##############################################################################################################################################################  
### TEACH DEFS ################################################################################################################################ TEACH DEFS ###
##############################################################################################################################################################  

def teachInsertBelSelected():
  global XcurPos
  global YcurPos
  global ZcurPos
  global RxcurPos
  global RycurPos
  global RzcurPos
  global WC
  global J7PosCur
  checkSpeedVals()
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  Speed = speedEntryField.get()
  speedtype = speedOption.get()
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  if(speedtype == "Percent"):
    speedPrefix = "Sp"    
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  Rounding = roundEntryField.get()
  movetype = options.get()
  if(movetype == "OFF J"):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ]"
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  if(movetype == "Move Vis"):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ]"
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close() 
  elif(movetype == "Move PR"):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ]"
    newPos = movetype + " [*]"+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC          
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "OFF PR "):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ] offs [ *PR: "+str(int(SavePosEntryField.get())+1)+" ] "
    newPos = movetype + " [*]"+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move J"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move L"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" Rnd "+Rounding+" $ "+WC 
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move R"):
    newPos = movetype + " [*] J1 "+J1AngCur+" J2 "+J2AngCur+" J3 "+J3AngCur+" J4 "+J4AngCur+" J5 "+J5AngCur+" J6 "+J6AngCur+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC            
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move A Mid"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC             
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()	
  elif(movetype == "Move A End"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC             
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()	
  elif(movetype == "Move C Center"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" J7 "+str(J7PosCur)+" J8 "+str(J8PosCur)+" J9 "+str(J9PosCur)+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move C Start"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos                 
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()	
  elif(movetype == "Move C Plane"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Start Spline" or movetype == "End Spline"):
    newPos = movetype              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Teach PR"):
    PR = str(SavePosEntryField.get())
    SPE6 = "Position Register "+PR+" Element 6 = "+RxcurPos         
    tab1.progView.insert(selRow, bytes(SPE6 + '\n', 'utf-8')) 
    SPE5 = "Position Register "+PR+" Element 5 = "+RycurPos            
    tab1.progView.insert(selRow, bytes(SPE5 + '\n', 'utf-8')) 
    SPE4 = "Position Register "+PR+" Element 4 = "+RzcurPos           
    tab1.progView.insert(selRow, bytes(SPE4 + '\n', 'utf-8')) 	
    SPE3 = "Position Register "+PR+" Element 3 = "+ZcurPos       
    tab1.progView.insert(selRow, bytes(SPE3 + '\n', 'utf-8')) 	
    SPE2 = "Position Register "+PR+" Element 2 = "+YcurPos            
    tab1.progView.insert(selRow, bytes(SPE2 + '\n', 'utf-8')) 
    SPE1 = "Position Register "+PR+" Element 1 = "+XcurPos         
    tab1.progView.insert(selRow, bytes(SPE1 + '\n', 'utf-8'))    	
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()

def teachReplaceSelected():
  try:
    deleteitem()
    selRow = tab1.progView.curselection()[0]
    tab1.progView.select_set(selRow-1)
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  teachInsertBelSelected()



 

############################################################################################################################################################## 
### PROGRAM FUNCTION DEFS ########################################################################################################## PROGRAM FUNCTION DEFS ###
############################################################################################################################################################## 
  
def deleteitem():
  selRow = tab1.progView.curselection()[0]
  selection = tab1.progView.curselection()  
  tab1.progView.delete(selection[0])
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()  
  
def manInsItem():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow) 
  tab1.progView.insert(selRow, bytes(manEntryField.get() + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow) 
  selRow = tab1.progView.curselection()[0]
  curRowEntryField.delete(0, 'end')
  curRowEntryField.insert(0,selRow)
  tab1.progView.itemconfig(selRow, {'fg': 'darkgreen'})
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()
  
def manReplItem():
  #selRow = curRowEntryField.get()
  selRow = tab1.progView.curselection()[0]
  tab1.progView.delete(selRow) 
  tab1.progView.insert(selRow, bytes(manEntryField.get() + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  tab1.progView.itemconfig(selRow, {'fg': 'darkgreen'})  
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()
  
def waitTime():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  seconds = waitTimeEntryField.get()
  newTime = "Wait Time = "+seconds               
  tab1.progView.insert(selRow, bytes(newTime + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()


def waitInputOn():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  input = waitInputEntryField.get()
  newInput = "Wait Input On = "+input              
  tab1.progView.insert(selRow, bytes(newInput + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def waitInputOff():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  input = waitInputOffEntryField.get()
  newInput = "Wait Off Input = "+input              
  tab1.progView.insert(selRow, bytes(newInput + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def setOutputOn():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  output = outputOnEntryField.get()
  newOutput = "Out On = "+output              
  tab1.progView.insert(selRow, bytes(newOutput + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def setOutputOff():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  output = outputOffEntryField.get()
  newOutput = "Out Off = "+output              
  tab1.progView.insert(selRow, bytes(newOutput + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def tabNumber():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  tabNum = tabNumEntryField.get()
  tabins = "Tab Number "+tabNum              
  tab1.progView.insert(selRow, bytes(tabins + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def jumpTab():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  tabNum = jumpTabEntryField.get()
  tabjmp = "Jump Tab-"+tabNum              
  tab1.progView.insert(selRow, bytes(tabjmp + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()
 
def cameraOn():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  value = "Cam On"
  tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def cameraOff():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  value = "Cam Off"
  tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()
  
def IfOnjumpTab():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  inpNum = IfOnjumpInputTabEntryField.get()
  tabNum = IfOnjumpNumberTabEntryField.get()
  tabjmp = "If On Jump - Input-"+inpNum+" Jump to Tab-"+tabNum             
  tab1.progView.insert(selRow, bytes(tabjmp + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def IfOffjumpTab():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  inpNum = IfOffjumpInputTabEntryField.get()
  tabNum = IfOffjumpNumberTabEntryField.get()
  tabjmp = "If Off Jump - Input-"+inpNum+" Jump to Tab-"+tabNum             
  tab1.progView.insert(selRow, bytes(tabjmp + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()
  tabNumEntryField.delete(0, 'end')

def Servo():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  servoNum = servoNumEntryField.get()
  servoPos = servoPosEntryField.get()
  servoins = "Servo number "+servoNum+" to position: "+servoPos              
  tab1.progView.insert(selRow, bytes(servoins + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def loadProg():
  folder = os.path.dirname(os.path.realpath(__file__))
  filetypes = (('robot program', '*.ar4'),("all files", "*.*"))
  filename = fd.askopenfilename(title='Open files',initialdir=folder,filetypes=filetypes)
  name = os.path.basename(filename)
  ProgEntryField.delete(0, 'end')
  ProgEntryField.insert(0,name)
  tab1.progView.delete(0,END)
  Prog = open(filename,"rb")
  time.sleep(.1)
  for item in Prog:
    tab1.progView.insert(END,item)
  tab1.progView.pack()
  scrollbar.config(command=tab1.progView.yview)
  savePosData()

def callProg(name):  
  ProgEntryField.delete(0, 'end')
  ProgEntryField.insert(0,name)
  tab1.progView.delete(0,END)
  Prog = open(name,"rb")
  time.sleep(.1)
  for item in Prog:
    tab1.progView.insert(END,item)
  tab1.progView.pack()
  scrollbar.config(command=tab1.progView.yview)

def CreateProg():
  user_input = simpledialog.askstring(title="New Program", prompt="New Program Name:")
  file_path = user_input + ".ar4"
  with open(file_path,'w', encoding='utf-8') as f:
    f.write("##BEGINNING OF PROGRAM##")
    f.write('\n')
  f.close()
  ProgEntryField.delete(0, 'end')
  ProgEntryField.insert(0,file_path)
  tab1.progView.delete(0,END)
  Prog = open(file_path,"rb")
  time.sleep(.1)
  for item in Prog:
    tab1.progView.insert(END,item)
  tab1.progView.pack()
  scrollbar.config(command=tab1.progView.yview)
  savePosData() 



def insertCallProg():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  newProg = changeProgEntryField.get()
  changeProg = "Call Program - "+newProg
  if  str(changeProg[-4:]) != ".ar4":
    changeProg = changeProg + ".ar4"             
  tab1.progView.insert(selRow, bytes(changeProg + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def insertReturn():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  value = "Return"           
  tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()


def openText():
  file_path = path.relpath(ProgEntryField.get())
  os.startfile(file_path) 

def reloadProg():
  file_path = path.relpath(ProgEntryField.get())
  ProgEntryField.delete(0, 'end')
  ProgEntryField.insert(0,file_path)
  tab1.progView.delete(0,END)
  Prog = open(file_path,"rb")
  time.sleep(.1)
  for item in Prog:
    tab1.progView.insert(END,item)
  tab1.progView.pack()
  scrollbar.config(command=tab1.progView.yview)
  savePosData()      


def insertvisFind():
  global ZcurPos
  global RxcurPos
  global RycurPos
  global RzcurPos
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  template = selectedTemplate.get()
  if (template == ""):
    template = "None_Selected.jpg"
  autoBGVal = int(autoBG.get())  
  if (autoBGVal == 1):
    BGcolor = "(Auto)"
  else:
    BGcolor = VisBacColorEntryField.get()
  score = VisScoreEntryField.get()
  passTab = visPassEntryField.get()
  failTab = visFailEntryField.get()
  value = "Vis Find - "+template+" - BGcolor "+BGcolor+" Score "+score+" Pass "+passTab+" Fail "+failTab
  tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()


def IfRegjumpTab():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  regNum = regNumJmpEntryField.get()
  regEqNum = regEqJmpEntryField.get()
  tabNum = regTabJmpEntryField.get()
  tabjmp = "If Register "+regNum+" = "+regEqNum+" Jump to Tab "+ tabNum            
  tab1.progView.insert(selRow, bytes(tabjmp + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def insertRegister():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  regNum = regNumEntryField.get()
  regCmd = regEqEntryField.get()
  regIns = "Register "+regNum+" = "+regCmd             
  tab1.progView.insert(selRow, bytes(regIns + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()
  
def storPos():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  regNum = storPosNumEntryField.get()
  regElmnt = storPosElEntryField.get()
  regCmd = storPosValEntryField.get()
  regIns = "Position Register "+regNum+" Element "+regElmnt+" = "+regCmd             
  tab1.progView.insert(selRow, bytes(regIns + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()
  
def insCalibrate():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  insCal = "Calibrate Robot"          
  tab1.progView.insert(selRow, bytes(insCal + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
  	  f.write(str(item.strip(), encoding='utf-8'))
  	  f.write('\n')
    f.close()

def progViewselect(e):
  selRow = tab1.progView.curselection()[0]
  curRowEntryField.delete(0, 'end')
  curRowEntryField.insert(0,selRow)
 
def getSel():
  selRow = tab1.progView.curselection()[0]
  tab1.progView.see(selRow+2)
  data = list(map(int, tab1.progView.curselection()))
  command=tab1.progView.get(data[0]).decode()
  manEntryField.delete(0, 'end')
  manEntryField.insert(0, command)  
  
def Servo0on():
  savePosData() 
  servoPos = servo0onEntryField.get()
  command = "SV0P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read()


def Servo0off():
  savePosData() 
  servoPos = servo0offEntryField.get()
  command = "SV0P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read()


def Servo1on():
  savePosData() 
  servoPos = servo1onEntryField.get()
  command = "SV1P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 


def Servo1off():
  savePosData() 
  servoPos = servo1offEntryField.get()
  command = "SV1P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read()
 

def Servo2on():
  savePosData() 
  servoPos = servo2onEntryField.get()
  command = "SV2P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 


def Servo2off():
  savePosData() 
  servoPos = servo2offEntryField.get()
  command = "SV2P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read()

def Servo3on():
  savePosData() 
  servoPos = servo3onEntryField.get()
  command = "SV3P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 

def Servo3off():
  savePosData() 
  servoPos = servo3offEntryField.get()
  command = "SV3P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read()

def DO1on():
  outputNum = DO1onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 


def DO1off():
  outputNum = DO1offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 
 

def DO2on():
  outputNum = DO2onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read()
 

def DO2off():
  outputNum = DO2offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 


def DO3on():
  outputNum = DO3onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 


def DO3off():
  outputNum = DO3offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 
 

def DO4on():
  outputNum = DO4onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read()
 

def DO4off():
  outputNum = DO4offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 


def DO5on():
  outputNum = DO5onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 


def DO5off():
  outputNum = DO5offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 
 

def DO6on():
  outputNum = DO6onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read()
 

def DO6off():
  outputNum = DO6offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.1)
  ser2.read() 
  
def TestString():
  message = testSendEntryField.get()
  command = "TM"+message+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(0)
  echo = ser.readline()
  testRecEntryField.delete(0, 'end')
  testRecEntryField.insert(0,echo)  

def ClearTestString():
  testRecEntryField.delete(0, 'end')
  
def CalcLinDist(X2,Y2,Z2):
  global XcurPos
  global YcurPos
  global ZcurPos
  global LineDist
  X1 = XcurPos
  Y1 = YcurPos
  Z1 = ZcurPos
  LineDist = (((X2-X1)**2)+((Y2-Y1)**2)+((Z2-Z1)**2))**.5
  return (LineDist)

def CalcLinVect(X2,Y2,Z2):
  global XcurPos
  global YcurPos
  global ZcurPos
  global Xv
  global Yv
  global Zv
  X1 = XcurPos
  Y1 = YcurPos
  Z1 = ZcurPos
  Xv = X2-X1
  Yv = Y2-Y1
  Zv = Z2-Z1
  return (Xv,Yv,Zv)  

def CalcLinWayPt(CX,CY,CZ,curWayPt,):
  global XcurPos
  global YcurPos
  global ZcurPos

 


	
	
##############################################################################################################################################################	
### CALIBRATION & SAVE DEFS ###################################################################################################### CALIBRATION & SAVE DEFS ###
##############################################################################################################################################################	

def calRobotAll():
  ##### STAGE 1 ########
  command = "LL"+"A"+str(J1CalStatVal)+"B"+str(J2CalStatVal)+"C"+str(J3CalStatVal)+"D"+str(J4CalStatVal)+"E"+str(J5CalStatVal)+"F"+str(J6CalStatVal)+"G0H0I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "Auto Calibration Stage 1 Successful"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "Auto Calibration Stage 1 Failed - See Log" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb")) 
  ##### STAGE 2 ########
  CalStatVal2 = int(J1CalStatVal2)+int(J2CalStatVal2)+int(J3CalStatVal2)+int(J4CalStatVal2)+int(J5CalStatVal2)+int(J6CalStatVal2)
  if(CalStatVal2>0):
    command = "LL"+"A"+str(J1CalStatVal2)+"B"+str(J2CalStatVal2)+"C"+str(J3CalStatVal2)+"D"+str(J4CalStatVal2)+"E"+str(J5CalStatVal2)+"F"+str(J6CalStatVal2)+"G0H0I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n" 
    ser.write(command.encode())
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.flushInput()
    response = str(ser.readline().strip(),'utf-8')
    cmdRecEntryField.delete(0, 'end')
    cmdRecEntryField.insert(0,response)
    if (response[:1] == 'A'):
      displayPosition(response)  
      message = "Auto Calibration Stage 2 Successful"
      almStatusLab.config(text=message, style="OK.TLabel")
      almStatusLab2.config(text=message, style="OK.TLabel") 
    else:
      message = "Auto Calibration Stage 2 Failed - See Log" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      ErrorHandler(response)
    Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 


def calRobotJ1():
  command = "LLA1B0C0D0E0F0G0H0I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J1 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J1 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ2():
  command = "LLA0B1C0D0E0F0G0H0I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J2 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J2 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ3():
  command = "LLA0B0C1D0E0F0G0H0I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J3 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J3 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ4():
  command = "LLA0B0C0D1E0F0G0H0I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n" 
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J4 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel" ) 
  else:
    message = "J4 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ5():
  command = "LLA0B0C0D0E1F0G0H0I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J5 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J5 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ6():
  command = "LLA0B0C0D0E0F1G0H0I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J6 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J6 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))   

def calRobotJ7():
  command = "LLA0B0C0D0E0F0G1H0I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J7 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J7 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb")) 

def calRobotJ8():
  command = "LLA0B0C0D0E0F0G0H1I0"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J8 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J8 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))    

def calRobotJ9():
  command = "LLA0B0C0D0E0F0G0H0I1"+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J9 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J9 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))             
	


def calRobotMid():
  print ("foo")
  #add mid command

def correctPos():
  command = "CP\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  displayPosition(response) 

def requestPos():
  command = "RP\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  displayPosition(response) 

def updateParams():
  TFx  = TFxEntryField.get()
  TFy  = TFyEntryField.get()
  TFz  = TFzEntryField.get()
  TFrz = TFrzEntryField.get()
  TFry = TFryEntryField.get()
  TFrx = TFrxEntryField.get()
  J1motDir = J1MotDirEntryField.get()
  J2motDir = J2MotDirEntryField.get()
  J3motDir = J3MotDirEntryField.get()
  J4motDir = J4MotDirEntryField.get()
  J5motDir = J5MotDirEntryField.get()
  J6motDir = J6MotDirEntryField.get()
  J7motDir = J7MotDirEntryField.get()
  J8motDir = J8MotDirEntryField.get()
  J9motDir = J9MotDirEntryField.get()
  J1calDir = J1CalDirEntryField.get()
  J2calDir = J2CalDirEntryField.get()
  J3calDir = J3CalDirEntryField.get()
  J4calDir = J4CalDirEntryField.get()
  J5calDir = J5CalDirEntryField.get()
  J6calDir = J6CalDirEntryField.get()
  J7calDir = J7CalDirEntryField.get()
  J8calDir = J8CalDirEntryField.get()
  J9calDir = J9CalDirEntryField.get()
  J1PosLim = J1PosLimEntryField.get()
  J1NegLim = J1NegLimEntryField.get()
  J2PosLim = J2PosLimEntryField.get()
  J2NegLim = J2NegLimEntryField.get()
  J3PosLim = J3PosLimEntryField.get()
  J3NegLim = J3NegLimEntryField.get()
  J4PosLim = J4PosLimEntryField.get()
  J4NegLim = J4NegLimEntryField.get()
  J5PosLim = J5PosLimEntryField.get()
  J5NegLim = J5NegLimEntryField.get()
  J6PosLim = J6PosLimEntryField.get()
  J6NegLim = J6NegLimEntryField.get()
  J1StepDeg = J1StepDegEntryField.get()
  J2StepDeg = J2StepDegEntryField.get()
  J3StepDeg = J3StepDegEntryField.get()
  J4StepDeg = J4StepDegEntryField.get()
  J5StepDeg = J5StepDegEntryField.get()
  J6StepDeg = J6StepDegEntryField.get()
  J1EncMult = str(float(J1EncCPREntryField.get())/float(J1DriveMSEntryField.get()))
  J2EncMult = str(float(J2EncCPREntryField.get())/float(J2DriveMSEntryField.get()))
  J3EncMult = str(float(J3EncCPREntryField.get())/float(J3DriveMSEntryField.get()))
  J4EncMult = str(float(J4EncCPREntryField.get())/float(J4DriveMSEntryField.get()))
  J5EncMult = str(float(J5EncCPREntryField.get())/float(J5DriveMSEntryField.get()))
  J6EncMult = str(float(J6EncCPREntryField.get())/float(J6DriveMSEntryField.get()))
  J1ΘDHpar = J1ΘEntryField.get()
  J2ΘDHpar = J2ΘEntryField.get()
  J3ΘDHpar = J3ΘEntryField.get()
  J4ΘDHpar = J4ΘEntryField.get()
  J5ΘDHpar = J5ΘEntryField.get()
  J6ΘDHpar = J6ΘEntryField.get()
  J1αDHpar = J1αEntryField.get()
  J2αDHpar = J2αEntryField.get()
  J3αDHpar = J3αEntryField.get()
  J4αDHpar = J4αEntryField.get()
  J5αDHpar = J5αEntryField.get()
  J6αDHpar = J6αEntryField.get()
  J1dDHpar = J1dEntryField.get()
  J2dDHpar = J2dEntryField.get()
  J3dDHpar = J3dEntryField.get()
  J4dDHpar = J4dEntryField.get()
  J5dDHpar = J5dEntryField.get()
  J6dDHpar = J6dEntryField.get()
  J1aDHpar = J1aEntryField.get()
  J2aDHpar = J2aEntryField.get()
  J3aDHpar = J3aEntryField.get()
  J4aDHpar = J4aEntryField.get()
  J5aDHpar = J5aEntryField.get()
  J6aDHpar = J6aEntryField.get()

  J1negLimLab.config(text="-"+J1NegLim, style="Jointlim.TLabel")
  J1posLimLab.config(text=J1PosLim, style="Jointlim.TLabel")
  J1jogslide.config(from_=float("-"+J1NegLim), to=float(J1PosLim),  length=180, orient=HORIZONTAL,  command=J1sliderUpdate)
  J2negLimLab.config(text="-"+J2NegLim, style="Jointlim.TLabel")
  J2posLimLab.config(text=J2PosLim, style="Jointlim.TLabel")
  J2jogslide.config(from_=float("-"+J2NegLim), to=float(J2PosLim),  length=180, orient=HORIZONTAL,  command=J2sliderUpdate)
  J3negLimLab.config(text="-"+J3NegLim, style="Jointlim.TLabel")
  J3posLimLab.config(text=J3PosLim, style="Jointlim.TLabel")
  J3jogslide.config(from_=float("-"+J3NegLim), to=float(J3PosLim),  length=180, orient=HORIZONTAL,  command=J3sliderUpdate)
  J4negLimLab.config(text="-"+J4NegLim, style="Jointlim.TLabel")
  J4posLimLab.config(text=J4PosLim, style="Jointlim.TLabel")
  J4jogslide.config(from_=float("-"+J4NegLim), to=float(J4PosLim),  length=180, orient=HORIZONTAL,  command=J4sliderUpdate)
  J5negLimLab.config(text="-"+J5NegLim, style="Jointlim.TLabel")
  J5posLimLab.config(text=J5PosLim, style="Jointlim.TLabel")
  J5jogslide.config(from_=float("-"+J5NegLim), to=float(J5PosLim),  length=180, orient=HORIZONTAL,  command=J5sliderUpdate)
  J6negLimLab.config(text="-"+J6NegLim, style="Jointlim.TLabel")
  J6posLimLab.config(text=J6PosLim, style="Jointlim.TLabel")
  J6jogslide.config(from_=float("-"+J6NegLim), to=float(J6PosLim),  length=180, orient=HORIZONTAL,  command=J6sliderUpdate)

  command = "UP"+"A"+TFx+"B"+TFy+"C"+TFz+"D"+TFrz+"E"+TFry+"F"+TFrx+\
  "G"+J1motDir+"H"+J2motDir+"I"+J3motDir+"J"+J4motDir+"K"+J5motDir+"L"+J6motDir+"M"+J7motDir+"N"+J8motDir+"O"+J9motDir+\
  "P"+J1calDir+"Q"+J2calDir+"R"+J3calDir+"S"+J4calDir+"T"+J5calDir+"U"+J6calDir+"V"+J7calDir+"W"+J8calDir+"X"+J9calDir+\
  "Y"+J1PosLim+"Z"+J1NegLim+"a"+J2PosLim+"b"+J2NegLim+"c"+J3PosLim+"d"+J3NegLim+"e"+J4PosLim+"f"+J4NegLim+"g"+J5PosLim+"h"+J5NegLim+"i"+J6PosLim+"j"+J6NegLim+\
  "k"+J1StepDeg+"l"+J2StepDeg+"m"+J3StepDeg+"n"+J4StepDeg+"o"+J5StepDeg+"p"+J6StepDeg+\
  "q"+J1EncMult+"r"+J2EncMult+"s"+J3EncMult+"t"+J4EncMult+"u"+J5EncMult+"v"+J6EncMult+\
  "w"+J1ΘDHpar+"x"+J2ΘDHpar+"y"+J3ΘDHpar+"z"+J4ΘDHpar+"!"+J5ΘDHpar+"@"+J6ΘDHpar+\
  "#"+J1αDHpar+"$"+J2αDHpar+"%"+J3αDHpar+"^"+J4αDHpar+"&"+J5αDHpar+"*"+J6αDHpar+\
  "("+J1dDHpar+")"+J2dDHpar+"+"+J3dDHpar+"="+J4dDHpar+","+J5dDHpar+"_"+J6dDHpar+\
  "<"+J1aDHpar+">"+J2aDHpar+"?"+J3aDHpar+"{"+J4aDHpar+"}"+J5aDHpar+"~"+J6aDHpar+\
  "\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = ser.read()

def calExtAxis():
  J7NegLim = 0
  J8NegLim = 0
  J9NegLim = 0

  J7PosLim = float(axis7lengthEntryField.get())
  J8PosLim = float(axis8lengthEntryField.get())
  J9PosLim = float(axis9lengthEntryField.get())

  J7negLimLab.config(text=str(-J7NegLim), style="Jointlim.TLabel")
  J8negLimLab.config(text=str(-J8NegLim), style="Jointlim.TLabel")
  J9negLimLab.config(text=str(-J9NegLim), style="Jointlim.TLabel")

  J7posLimLab.config(text=str(J7PosLim), style="Jointlim.TLabel")
  J8posLimLab.config(text=str(J8PosLim), style="Jointlim.TLabel")
  J9posLimLab.config(text=str(J9PosLim), style="Jointlim.TLabel")

  J7jogslide.config(from_=-J7NegLim, to=J7PosLim,  length=125, orient=HORIZONTAL,  command=J7sliderUpdate)
  J8jogslide.config(from_=-J8NegLim, to=J8PosLim,  length=125, orient=HORIZONTAL,  command=J8sliderUpdate)
  J9jogslide.config(from_=-J9NegLim, to=J9PosLim,  length=125, orient=HORIZONTAL,  command=J9sliderUpdate)

  command = "CE"+"A"+str(J7PosLim)+"B"+str(J7rotation)+"C"+str(J7steps)+"D"+str(J8PosLim)+"E"+str(J8rotation)+"F"+str(J8steps)+"G"+str(J9PosLim)+"H"+str(J9rotation)+"I"+str(J9steps)+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = ser.read()

def zeroAxis7():
  command = "Z7"+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  almStatusLab.config(text="J7 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J7 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J7 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  
  response = str(ser.readline().strip(),'utf-8')
  displayPosition(response) 

def zeroAxis8():
  command = "Z8"+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  almStatusLab.config(text="J8 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J8 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J8 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  
  response = str(ser.readline().strip(),'utf-8')
  displayPosition(response) 

def zeroAxis9():
  command = "Z9"+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  almStatusLab.config(text="J9 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J9 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J9 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  
  response = str(ser.readline().strip(),'utf-8')
  displayPosition(response)   


def sendPos():
  command = "SP"+"A"+str(J1AngCur)+"B"+str(J2AngCur)+"C"+str(J3AngCur)+"D"+str(J4AngCur)+"E"+str(J5AngCur)+"F"+str(J6AngCur)+"G"+str(J7PosCur)+"H"+str(J8PosCur)+"I"+str(J9PosCur)+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = ser.read()

def CalZeroPos():
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  command = "SPA0B0C0D0E90F0\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = ser.read()
  requestPos()
  almStatusLab.config(text="Calibration Forced to Home", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Home", style="Warn.TLabel")
  message = "Calibration Forced to Home - this is for commissioning and testing - be careful!"
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  

def CalRestPos():
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  command = "SPA0B0C-89D0E0F0\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = ser.read()
  requestPos()
  almStatusLab.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")
  message = "Calibration Forced to Vertical - this is for commissioning and testing - be careful!"
  tab8.ElogView.insert(END, Curtime+" - "+message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  


def ResetDrives():
  ResetDriveBut = Button(tab1,  text="Reset Drives",   command = ResetDrives)
  ResetDriveBut.place(x=307, y=42)
  command = "RD"+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.1)
  response = ser.read()
  almStatusLab.config(text="DRIVES RESET - PLEASE CHECK CALIBRATION", style="Warn.TLabel")
  almStatusLab2.config(text="DRIVES RESET - PLEASE CHECK CALIBRATION", style="Warn.TLabel")
  requestPos()


def displayPosition(response):
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J7StepCur
  global XcurPos
  global YcurPos
  global ZcurPos
  global RxcurPos
  global RycurPos
  global RzcurPos 
  global J7PosCur
  global J8PosCur
  global J9PosCur
  global WC 

  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  J1AngIndex = response.find('A');
  J2AngIndex = response.find('B');
  J3AngIndex = response.find('C');
  J4AngIndex = response.find('D');
  J5AngIndex = response.find('E');
  J6AngIndex = response.find('F');
  XposIndex = response.find('G');
  YposIndex = response.find('H');
  ZposIndex = response.find('I');
  RzposIndex = response.find('J');
  RyposIndex = response.find('K');
  RxposIndex = response.find('L');
  SpeedVioIndex = response.find('M');
  DebugIndex = response.find('N');
  FlagIndex = response.find('O');
  J7PosIndex = response.find('P');
  J8PosIndex = response.find('Q');
  J9PosIndex = response.find('R');
  J1AngCur = response[J1AngIndex+1:J2AngIndex].strip();
  J2AngCur = response[J2AngIndex+1:J3AngIndex].strip();
  J3AngCur = response[J3AngIndex+1:J4AngIndex].strip();
  J4AngCur = response[J4AngIndex+1:J5AngIndex].strip();
  J5AngCur = response[J5AngIndex+1:J6AngIndex].strip();
  J6AngCur = response[J6AngIndex+1:XposIndex].strip();

  if (float(J5AngCur) > 0):
    WC = "F"
  else:
    WC = "N"
  XcurPos = response[XposIndex+1:YposIndex].strip();
  YcurPos = response[YposIndex+1:ZposIndex].strip();
  ZcurPos = response[ZposIndex+1:RzposIndex].strip();
  RzcurPos = response[RzposIndex+1:RyposIndex].strip();
  RycurPos = response[RyposIndex+1:RxposIndex].strip();
  RxcurPos = response[RxposIndex+1:SpeedVioIndex].strip();
  SpeedVioation = response[SpeedVioIndex+1:DebugIndex].strip();
  Debug = response[DebugIndex+1:FlagIndex].strip();
  Flag = response[FlagIndex+1:J7PosIndex].strip();
  J7PosCur = float(response[J7PosIndex+1:J8PosIndex].strip());
  J8PosCur = float(response[J8PosIndex+1:J9PosIndex].strip());
  J9PosCur = float(response[J9PosIndex+1:].strip());
  
  J1curAngEntryField.delete(0, 'end')
  J1curAngEntryField.insert(0,J1AngCur)
  J2curAngEntryField.delete(0, 'end')
  J2curAngEntryField.insert(0,J2AngCur)
  J3curAngEntryField.delete(0, 'end')
  J3curAngEntryField.insert(0,J3AngCur)
  J4curAngEntryField.delete(0, 'end')
  J4curAngEntryField.insert(0,J4AngCur)
  J5curAngEntryField.delete(0, 'end')
  J5curAngEntryField.insert(0,J5AngCur)
  J6curAngEntryField.delete(0, 'end')
  J6curAngEntryField.insert(0,J6AngCur)
  XcurEntryField.delete(0, 'end')
  XcurEntryField.insert(0,XcurPos)
  YcurEntryField.delete(0, 'end')
  YcurEntryField.insert(0,YcurPos)
  ZcurEntryField.delete(0, 'end')
  ZcurEntryField.insert(0,ZcurPos)
  RzcurEntryField.delete(0, 'end')
  RzcurEntryField.insert(0,RzcurPos)
  RycurEntryField.delete(0, 'end')
  RycurEntryField.insert(0,RycurPos)
  RxcurEntryField.delete(0, 'end')
  RxcurEntryField.insert(0,RxcurPos)
  J7curAngEntryField.delete(0, 'end')
  J7curAngEntryField.insert(0,J7PosCur)
  J8curAngEntryField.delete(0, 'end')
  J8curAngEntryField.insert(0,J8PosCur)
  J9curAngEntryField.delete(0, 'end')
  J9curAngEntryField.insert(0,J9PosCur)
  J1jogslide.set(J1AngCur)
  J2jogslide.set(J2AngCur)
  J3jogslide.set(J3AngCur)
  J4jogslide.set(J4AngCur)
  J5jogslide.set(J5AngCur)
  J6jogslide.set(J6AngCur)
  J7jogslide.set(J7PosCur)
  J8jogslide.set(J8PosCur)
  J9jogslide.set(J9PosCur)
  manEntryField.delete(0, 'end')
  manEntryField.insert(0,Debug)

  #tab8.ElogView.insert(END, Debug)
  #value=tab8.ElogView.get(0,END)
  #pickle.dump(value,open("ErrorLog","wb"))

  savePosData()
  if (Flag != ""):
      ErrorHandler(Flag) 
  if (SpeedVioation=='1'):
      Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
      message = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))          
      almStatusLab.config(text=message, style="Warn.TLabel")
      almStatusLab2.config(text=message, style="Warn.TLabel")


def ClearKinTabFields():
  J1MotDirEntryField.delete(0, 'end')
  J2MotDirEntryField.delete(0, 'end')
  J3MotDirEntryField.delete(0, 'end')
  J4MotDirEntryField.delete(0, 'end')
  J5MotDirEntryField.delete(0, 'end')
  J6MotDirEntryField.delete(0, 'end')
  J7MotDirEntryField.delete(0, 'end')
  J8MotDirEntryField.delete(0, 'end')
  J9MotDirEntryField.delete(0, 'end')
  J1CalDirEntryField.delete(0, 'end')
  J2CalDirEntryField.delete(0, 'end')
  J3CalDirEntryField.delete(0, 'end')
  J4CalDirEntryField.delete(0, 'end')
  J5CalDirEntryField.delete(0, 'end')
  J6CalDirEntryField.delete(0, 'end')
  J7CalDirEntryField.delete(0, 'end')
  J8CalDirEntryField.delete(0, 'end')
  J9CalDirEntryField.delete(0, 'end')
  J1PosLimEntryField.delete(0, 'end')
  J1NegLimEntryField.delete(0, 'end')
  J2PosLimEntryField.delete(0, 'end')
  J2NegLimEntryField.delete(0, 'end')
  J3PosLimEntryField.delete(0, 'end')
  J3NegLimEntryField.delete(0, 'end')
  J4PosLimEntryField.delete(0, 'end')
  J4NegLimEntryField.delete(0, 'end')
  J5PosLimEntryField.delete(0, 'end')
  J5NegLimEntryField.delete(0, 'end')
  J6PosLimEntryField.delete(0, 'end')
  J6NegLimEntryField.delete(0, 'end')  
  J1StepDegEntryField.delete(0, 'end')
  J2StepDegEntryField.delete(0, 'end') 
  J3StepDegEntryField.delete(0, 'end') 
  J4StepDegEntryField.delete(0, 'end') 
  J5StepDegEntryField.delete(0, 'end') 
  J6StepDegEntryField.delete(0, 'end')
  J1DriveMSEntryField.delete(0, 'end')
  J2DriveMSEntryField.delete(0, 'end')  
  J3DriveMSEntryField.delete(0, 'end')  
  J4DriveMSEntryField.delete(0, 'end')  
  J5DriveMSEntryField.delete(0, 'end')  
  J6DriveMSEntryField.delete(0, 'end')
  J1EncCPREntryField.delete(0, 'end')
  J2EncCPREntryField.delete(0, 'end')
  J3EncCPREntryField.delete(0, 'end')
  J4EncCPREntryField.delete(0, 'end')
  J5EncCPREntryField.delete(0, 'end')
  J6EncCPREntryField.delete(0, 'end')
  J1ΘEntryField.delete(0, 'end')
  J2ΘEntryField.delete(0, 'end')
  J3ΘEntryField.delete(0, 'end')
  J4ΘEntryField.delete(0, 'end')
  J5ΘEntryField.delete(0, 'end')
  J6ΘEntryField.delete(0, 'end')
  J1αEntryField.delete(0, 'end')
  J2αEntryField.delete(0, 'end')
  J3αEntryField.delete(0, 'end')
  J4αEntryField.delete(0, 'end')
  J5αEntryField.delete(0, 'end')
  J6αEntryField.delete(0, 'end')
  J1dEntryField.delete(0, 'end')
  J2dEntryField.delete(0, 'end')
  J3dEntryField.delete(0, 'end')
  J4dEntryField.delete(0, 'end')
  J5dEntryField.delete(0, 'end')
  J6dEntryField.delete(0, 'end')
  J1aEntryField.delete(0, 'end')
  J2aEntryField.delete(0, 'end')
  J3aEntryField.delete(0, 'end')
  J4aEntryField.delete(0, 'end')
  J5aEntryField.delete(0, 'end')
  J6aEntryField.delete(0, 'end')


def LoadAR4Mk2default():
  ClearKinTabFields()
  J1MotDirEntryField.insert(0,str(0))
  J2MotDirEntryField.insert(0,str(1))
  J3MotDirEntryField.insert(0,str(1))
  J4MotDirEntryField.insert(0,str(1))
  J5MotDirEntryField.insert(0,str(1))
  J6MotDirEntryField.insert(0,str(1))
  J7MotDirEntryField.insert(0,str(1))
  J8MotDirEntryField.insert(0,str(1))
  J9MotDirEntryField.insert(0,str(1))
  J1CalDirEntryField.insert(0,str(1))
  J2CalDirEntryField.insert(0,str(0))
  J3CalDirEntryField.insert(0,str(1))
  J4CalDirEntryField.insert(0,str(0))
  J5CalDirEntryField.insert(0,str(0))
  J6CalDirEntryField.insert(0,str(1))
  J7CalDirEntryField.insert(0,str(0))
  J8CalDirEntryField.insert(0,str(0))
  J9CalDirEntryField.insert(0,str(0))
  J1PosLimEntryField.insert(0,str(170))
  J1NegLimEntryField.insert(0,str(170))
  J2PosLimEntryField.insert(0,str(90))
  J2NegLimEntryField.insert(0,str(42))
  J3PosLimEntryField.insert(0,str(52))
  J3NegLimEntryField.insert(0,str(89))
  J4PosLimEntryField.insert(0,str(165))
  J4NegLimEntryField.insert(0,str(165))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(155))
  J6NegLimEntryField.insert(0,str(155))  
  J1StepDegEntryField.insert(0,str(44.4444))
  J2StepDegEntryField.insert(0,str(55.5555)) 
  J3StepDegEntryField.insert(0,str(55.5555)) 
  J4StepDegEntryField.insert(0,str(49.7777)) 
  J5StepDegEntryField.insert(0,str(21.8602)) 
  J6StepDegEntryField.insert(0,str(22.2222))
  J1DriveMSEntryField.insert(0,str(400))
  J2DriveMSEntryField.insert(0,str(400))  
  J3DriveMSEntryField.insert(0,str(400))  
  J4DriveMSEntryField.insert(0,str(400))  
  J5DriveMSEntryField.insert(0,str(800))  
  J6DriveMSEntryField.insert(0,str(400))
  J1EncCPREntryField.insert(0,str(4000))
  J2EncCPREntryField.insert(0,str(4000))
  J3EncCPREntryField.insert(0,str(4000))
  J4EncCPREntryField.insert(0,str(4000))
  J5EncCPREntryField.insert(0,str(4000))
  J6EncCPREntryField.insert(0,str(4000))
  J1ΘEntryField.insert(0,str(0))
  J2ΘEntryField.insert(0,str(-90))
  J3ΘEntryField.insert(0,str(180))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(0))
  J1αEntryField.insert(0,str(-90))
  J2αEntryField.insert(0,str(0))
  J3αEntryField.insert(0,str(90))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(0))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(36.25))
  J1aEntryField.insert(0,str(64.2))
  J2aEntryField.insert(0,str(305))
  J3aEntryField.insert(0,str(0))
  J4aEntryField.insert(0,str(0))
  J5aEntryField.insert(0,str(0))
  J6aEntryField.insert(0,str(0)) 


def LoadAR4default():
  ClearKinTabFields()
  J1MotDirEntryField.insert(0,str(1))
  J2MotDirEntryField.insert(0,str(0))
  J3MotDirEntryField.insert(0,str(0))
  J4MotDirEntryField.insert(0,str(1))
  J5MotDirEntryField.insert(0,str(0))
  J6MotDirEntryField.insert(0,str(0))
  J7MotDirEntryField.insert(0,str(1))
  J8MotDirEntryField.insert(0,str(1))
  J9MotDirEntryField.insert(0,str(1))
  J1CalDirEntryField.insert(0,str(1))
  J2CalDirEntryField.insert(0,str(0))
  J3CalDirEntryField.insert(0,str(1))
  J4CalDirEntryField.insert(0,str(0))
  J5CalDirEntryField.insert(0,str(0))
  J6CalDirEntryField.insert(0,str(1))
  J7CalDirEntryField.insert(0,str(0))
  J8CalDirEntryField.insert(0,str(0))
  J9CalDirEntryField.insert(0,str(0))
  J1PosLimEntryField.insert(0,str(170))
  J1NegLimEntryField.insert(0,str(170))
  J2PosLimEntryField.insert(0,str(90))
  J2NegLimEntryField.insert(0,str(42))
  J3PosLimEntryField.insert(0,str(52))
  J3NegLimEntryField.insert(0,str(89))
  J4PosLimEntryField.insert(0,str(165))
  J4NegLimEntryField.insert(0,str(165))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(155))
  J6NegLimEntryField.insert(0,str(155))  
  J1StepDegEntryField.insert(0,str(44.4444))
  J2StepDegEntryField.insert(0,str(55.5555)) 
  J3StepDegEntryField.insert(0,str(55.5555)) 
  J4StepDegEntryField.insert(0,str(42.7266)) 
  J5StepDegEntryField.insert(0,str(21.8602)) 
  J6StepDegEntryField.insert(0,str(22.2222))
  J1DriveMSEntryField.insert(0,str(400))
  J2DriveMSEntryField.insert(0,str(400))  
  J3DriveMSEntryField.insert(0,str(400))  
  J4DriveMSEntryField.insert(0,str(400))  
  J5DriveMSEntryField.insert(0,str(800))  
  J6DriveMSEntryField.insert(0,str(400))
  J1EncCPREntryField.insert(0,str(4000))
  J2EncCPREntryField.insert(0,str(4000))
  J3EncCPREntryField.insert(0,str(4000))
  J4EncCPREntryField.insert(0,str(4000))
  J5EncCPREntryField.insert(0,str(4000))
  J6EncCPREntryField.insert(0,str(4000))
  J1ΘEntryField.insert(0,str(0))
  J2ΘEntryField.insert(0,str(-90))
  J3ΘEntryField.insert(0,str(180))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(0))
  J1αEntryField.insert(0,str(-90))
  J2αEntryField.insert(0,str(0))
  J3αEntryField.insert(0,str(90))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(0))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(36.25))
  J1aEntryField.insert(0,str(64.2))
  J2aEntryField.insert(0,str(305))
  J3aEntryField.insert(0,str(0))
  J4aEntryField.insert(0,str(0))
  J5aEntryField.insert(0,str(0))
  J6aEntryField.insert(0,str(0)) 

def LoadAR3default():
  ClearKinTabFields()
  J1MotDirEntryField.insert(0,str(1))
  J2MotDirEntryField.insert(0,str(0))
  J3MotDirEntryField.insert(0,str(0))
  J4MotDirEntryField.insert(0,str(1))
  J5MotDirEntryField.insert(0,str(0))
  J6MotDirEntryField.insert(0,str(0))
  J7MotDirEntryField.insert(0,str(1))
  J8MotDirEntryField.insert(0,str(1))
  J9MotDirEntryField.insert(0,str(1))
  J1CalDirEntryField.insert(0,str(1))
  J2CalDirEntryField.insert(0,str(0))
  J3CalDirEntryField.insert(0,str(1))
  J4CalDirEntryField.insert(0,str(0))
  J5CalDirEntryField.insert(0,str(0))
  J6CalDirEntryField.insert(0,str(1))
  J7CalDirEntryField.insert(0,str(0))
  J8CalDirEntryField.insert(0,str(0))
  J9CalDirEntryField.insert(0,str(0))
  J1PosLimEntryField.insert(0,str(170))
  J1NegLimEntryField.insert(0,str(170))
  J2PosLimEntryField.insert(0,str(90))
  J2NegLimEntryField.insert(0,str(42))
  J3PosLimEntryField.insert(0,str(52))
  J3NegLimEntryField.insert(0,str(89))
  J4PosLimEntryField.insert(0,str(165))
  J4NegLimEntryField.insert(0,str(165))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(155))
  J6NegLimEntryField.insert(0,str(155))  
  J1StepDegEntryField.insert(0,str(44.4444))
  J2StepDegEntryField.insert(0,str(55.5555)) 
  J3StepDegEntryField.insert(0,str(55.5555)) 
  J4StepDegEntryField.insert(0,str(42.7266)) 
  J5StepDegEntryField.insert(0,str(21.8602)) 
  J6StepDegEntryField.insert(0,str(22.2222))
  J1DriveMSEntryField.insert(0,str(400))
  J2DriveMSEntryField.insert(0,str(400))  
  J3DriveMSEntryField.insert(0,str(400))  
  J4DriveMSEntryField.insert(0,str(400))  
  J5DriveMSEntryField.insert(0,str(800))  
  J6DriveMSEntryField.insert(0,str(400))
  J1EncCPREntryField.insert(0,str(2048))
  J2EncCPREntryField.insert(0,str(2048))
  J3EncCPREntryField.insert(0,str(2048))
  J4EncCPREntryField.insert(0,str(2048))
  J5EncCPREntryField.insert(0,str(2048))
  J6EncCPREntryField.insert(0,str(2048))
  J1ΘEntryField.insert(0,str(0))
  J2ΘEntryField.insert(0,str(-90))
  J3ΘEntryField.insert(0,str(180))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(0))
  J1αEntryField.insert(0,str(-90))
  J2αEntryField.insert(0,str(0))
  J3αEntryField.insert(0,str(90))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(0))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(36.25))
  J1aEntryField.insert(0,str(64.2))
  J2aEntryField.insert(0,str(305))
  J3aEntryField.insert(0,str(0))
  J4aEntryField.insert(0,str(0))
  J5aEntryField.insert(0,str(0))
  J6aEntryField.insert(0,str(0))   
  

def SaveAndApplyCalibration():
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global XcurPos
  global YcurPos
  global ZcurPos
  global RxcurPos
  global RycurPos
  global RzcurPos
  global J7PosCur
  global J8PosCur
  global J9PosCur
  global VisFileLoc
  global VisProg
  global VisOrigXpix
  global VisOrigXmm
  global VisOrigYpix
  global VisOrigYmm
  global VisEndXpix
  global VisEndXmm
  global VisEndYpix
  global VisEndYmm
  global J1calOff
  global J2calOff 
  global J3calOff 
  global J4calOff 
  global J5calOff 
  global J6calOff 
  global J7calOff 
  global J8calOff 
  global J9calOff 
  global J1OpenLoopVal
  global J2OpenLoopVal
  global J3OpenLoopVal
  global J4OpenLoopVal
  global J5OpenLoopVal
  global J6OpenLoopVal
  global DisableWristRotVal 
  global J1CalStatVal
  global J2CalStatVal
  global J3CalStatVal
  global J4CalStatVal
  global J5CalStatVal
  global J6CalStatVal 
  global J1CalStatVal2
  global J2CalStatVal2
  global J3CalStatVal2
  global J4CalStatVal2
  global J5CalStatVal2
  global J6CalStatVal2
  global J7PosLim
  global J7rotation
  global J7steps
  global J8length
  global J8rotation
  global J8steps
  global J9length
  global J9rotation
  global J9steps
  global IncJogStat
  J7PosCur = J7curAngEntryField.get()
  J8PosCur = J8curAngEntryField.get()
  J9PosCur = J9curAngEntryField.get()
  #VisFileLoc = VisFileLocEntryField.get()
  VisProg = visoptions.get()
  #VisOrigXpix = float(VisPicOxPEntryField.get())
  #VisOrigXmm  = float(VisPicOxMEntryField.get())
  #VisOrigYpix = float(VisPicOyPEntryField.get())
  #VisOrigYmm  = float(VisPicOyMEntryField.get())
  #VisEndXpix  = float(VisPicXPEntryField.get())
  #VisEndXmm   = float(VisPicXMEntryField.get())
  #VisEndYpix  = float(VisPicYPEntryField.get())
  #VisEndYmm   = float(VisPicYMEntryField.get())
  J1calOff    = float(J1calOffEntryField.get())
  J2calOff    = float(J2calOffEntryField.get())
  J3calOff    = float(J3calOffEntryField.get())
  J4calOff    = float(J4calOffEntryField.get())
  J5calOff    = float(J5calOffEntryField.get())
  J6calOff    = float(J6calOffEntryField.get())
  J7calOff    = float(J7calOffEntryField.get())
  J8calOff    = float(J8calOffEntryField.get())
  J9calOff    = float(J9calOffEntryField.get())
  J1OpenLoopVal = int(J1OpenLoopStat.get())
  J2OpenLoopVal = int(J2OpenLoopStat.get())
  J3OpenLoopVal = int(J3OpenLoopStat.get())
  J4OpenLoopVal = int(J4OpenLoopStat.get())
  J5OpenLoopVal = int(J5OpenLoopStat.get())
  J6OpenLoopVal = int(J6OpenLoopStat.get())
  DisableWristRotVal = int(DisableWristRot.get())
  J1CalStatVal = int(J1CalStat.get())
  J2CalStatVal = int(J2CalStat.get())
  J3CalStatVal = int(J3CalStat.get())
  J4CalStatVal = int(J4CalStat.get())
  J5CalStatVal = int(J5CalStat.get())
  J6CalStatVal = int(J6CalStat.get())
  J1CalStatVal2 = int(J1CalStat2.get())
  J2CalStatVal2 = int(J2CalStat2.get())
  J3CalStatVal2 = int(J3CalStat2.get())
  J4CalStatVal2 = int(J4CalStat2.get())
  J5CalStatVal2 = int(J5CalStat2.get())
  J6CalStatVal2 = int(J6CalStat2.get())
  J7PosLim     = float(axis7lengthEntryField.get())
  J7rotation   = float(axis7rotEntryField.get())
  J7steps      = float(axis7stepsEntryField.get())
  J8length     = float(axis8lengthEntryField.get())
  J8rotation   = float(axis8rotEntryField.get())
  J8steps      = float(axis8stepsEntryField.get())
  J9length     = float(axis9lengthEntryField.get())
  J9rotation   = float(axis9rotEntryField.get())
  J9steps      = float(axis9stepsEntryField.get())
  try:
    updateParams()
    time.sleep(.1)
    calExtAxis()
  except:
    print("no serial connection with Teensy board")  
  savePosData()

def savePosData():
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global XcurPos
  global YcurPos
  global ZcurPos
  global RxcurPos
  global RycurPos
  global RzcurPos
  global curTheme
  global J7PosLim
  global J7rotation
  global J7steps
  global J8length
  global J8rotation
  global J8steps
  global J9length
  global J9rotation
  global J9steps
  global mX1
  global mY1
  global mX2
  global mY2
  calibration.delete(0, END)
  calibration.insert(END, J1AngCur)
  calibration.insert(END, J2AngCur)
  calibration.insert(END, J3AngCur)
  calibration.insert(END, J4AngCur)
  calibration.insert(END, J5AngCur)
  calibration.insert(END, J6AngCur)
  calibration.insert(END, XcurPos)
  calibration.insert(END, YcurPos)
  calibration.insert(END, ZcurPos)
  calibration.insert(END, RzcurPos)
  calibration.insert(END, RycurPos)
  calibration.insert(END, RxcurPos)
  calibration.insert(END, comPortEntryField.get())  
  calibration.insert(END, ProgEntryField.get())
  calibration.insert(END, servo0onEntryField.get())
  calibration.insert(END, servo0offEntryField.get())
  calibration.insert(END, servo1onEntryField.get())
  calibration.insert(END, servo1offEntryField.get())
  calibration.insert(END, DO1onEntryField.get())
  calibration.insert(END, DO1offEntryField.get())
  calibration.insert(END, DO2onEntryField.get())
  calibration.insert(END, DO2offEntryField.get())
  calibration.insert(END, TFxEntryField.get())
  calibration.insert(END, TFyEntryField.get())
  calibration.insert(END, TFzEntryField.get())
  calibration.insert(END, TFrxEntryField.get())
  calibration.insert(END, TFryEntryField.get())
  calibration.insert(END, TFrzEntryField.get())
  calibration.insert(END, J7curAngEntryField.get())
  calibration.insert(END, J8curAngEntryField.get())
  calibration.insert(END, J9curAngEntryField.get())
  calibration.insert(END, "VisFileLocEntryField")
  calibration.insert(END, visoptions.get())
  calibration.insert(END, "VisPicOxPEntryField")
  calibration.insert(END, "VisPicOxMEntryField")
  calibration.insert(END, "VisPicOyPEntryField")
  calibration.insert(END, "VisPicOyMEntryField")
  calibration.insert(END, "VisPicXPEntryField")
  calibration.insert(END, "VisPicXMEntryField")
  calibration.insert(END, "VisPicYPEntryField")
  calibration.insert(END, "VisPicYMEntryField")
  calibration.insert(END, J1calOffEntryField.get())
  calibration.insert(END, J2calOffEntryField.get())
  calibration.insert(END, J3calOffEntryField.get())
  calibration.insert(END, J4calOffEntryField.get())
  calibration.insert(END, J5calOffEntryField.get())
  calibration.insert(END, J6calOffEntryField.get())
  calibration.insert(END, J1OpenLoopVal)
  calibration.insert(END, J2OpenLoopVal)
  calibration.insert(END, J3OpenLoopVal)
  calibration.insert(END, J4OpenLoopVal)
  calibration.insert(END, J5OpenLoopVal)
  calibration.insert(END, J6OpenLoopVal)
  calibration.insert(END, com2PortEntryField.get())  
  calibration.insert(END, curTheme)
  calibration.insert(END, J1CalStatVal)
  calibration.insert(END, J2CalStatVal)
  calibration.insert(END, J3CalStatVal)
  calibration.insert(END, J4CalStatVal)
  calibration.insert(END, J5CalStatVal)
  calibration.insert(END, J6CalStatVal)
  calibration.insert(END, J7PosLim)
  calibration.insert(END, J7rotation)
  calibration.insert(END, J7steps)
  calibration.insert(END, J7StepCur) #is this used?
  calibration.insert(END, J1CalStatVal2)
  calibration.insert(END, J2CalStatVal2)
  calibration.insert(END, J3CalStatVal2)
  calibration.insert(END, J4CalStatVal2)
  calibration.insert(END, J5CalStatVal2)
  calibration.insert(END, J6CalStatVal2)
  calibration.insert(END, VisBrightSlide.get())
  calibration.insert(END, VisContrastSlide.get())
  calibration.insert(END, VisBacColorEntryField.get())  
  calibration.insert(END, VisScoreEntryField.get())
  calibration.insert(END, VisX1PixEntryField.get())
  calibration.insert(END, VisY1PixEntryField.get())
  calibration.insert(END, VisX2PixEntryField.get())
  calibration.insert(END, VisY2PixEntryField.get())
  calibration.insert(END, VisX1RobEntryField.get())
  calibration.insert(END, VisY1RobEntryField.get())
  calibration.insert(END, VisX2RobEntryField.get())
  calibration.insert(END, VisY2RobEntryField.get())
  calibration.insert(END, VisZoomSlide.get())
  calibration.insert(END, pick180.get())
  calibration.insert(END, pickClosest.get())
  calibration.insert(END, visoptions.get())
  calibration.insert(END, fullRot.get()) 
  calibration.insert(END, autoBG.get())
  calibration.insert(END, mX1)
  calibration.insert(END, mY1)
  calibration.insert(END, mX2)
  calibration.insert(END, mY2)
  calibration.insert(END, J8length)
  calibration.insert(END, J8rotation)
  calibration.insert(END, J8steps)
  calibration.insert(END, J9length)
  calibration.insert(END, J9rotation)
  calibration.insert(END, J9steps)
  calibration.insert(END, J7calOffEntryField.get())
  calibration.insert(END, J8calOffEntryField.get())
  calibration.insert(END, J9calOffEntryField.get())
  calibration.insert(END, GC_ST_E1_EntryField.get())
  calibration.insert(END, GC_ST_E2_EntryField.get()) 
  calibration.insert(END, GC_ST_E3_EntryField.get()) 
  calibration.insert(END, GC_ST_E4_EntryField.get()) 
  calibration.insert(END, GC_ST_E5_EntryField.get()) 
  calibration.insert(END, GC_ST_E6_EntryField.get())
  calibration.insert(END, GC_SToff_E1_EntryField.get())
  calibration.insert(END, GC_SToff_E2_EntryField.get()) 
  calibration.insert(END, GC_SToff_E3_EntryField.get()) 
  calibration.insert(END, GC_SToff_E4_EntryField.get()) 
  calibration.insert(END, GC_SToff_E5_EntryField.get()) 
  calibration.insert(END, GC_SToff_E6_EntryField.get())
  calibration.insert(END, DisableWristRotVal)
  calibration.insert(END, J1MotDirEntryField.get())
  calibration.insert(END, J2MotDirEntryField.get()) 
  calibration.insert(END, J3MotDirEntryField.get()) 
  calibration.insert(END, J4MotDirEntryField.get()) 
  calibration.insert(END, J5MotDirEntryField.get()) 
  calibration.insert(END, J6MotDirEntryField.get()) 
  calibration.insert(END, J7MotDirEntryField.get()) 
  calibration.insert(END, J8MotDirEntryField.get()) 
  calibration.insert(END, J9MotDirEntryField.get())
  calibration.insert(END, J1CalDirEntryField.get())
  calibration.insert(END, J2CalDirEntryField.get()) 
  calibration.insert(END, J3CalDirEntryField.get()) 
  calibration.insert(END, J4CalDirEntryField.get()) 
  calibration.insert(END, J5CalDirEntryField.get()) 
  calibration.insert(END, J6CalDirEntryField.get()) 
  calibration.insert(END, J7CalDirEntryField.get()) 
  calibration.insert(END, J8CalDirEntryField.get()) 
  calibration.insert(END, J9CalDirEntryField.get())             
  calibration.insert(END, J1PosLimEntryField.get())
  calibration.insert(END, J1NegLimEntryField.get())
  calibration.insert(END, J2PosLimEntryField.get())
  calibration.insert(END, J2NegLimEntryField.get())
  calibration.insert(END, J3PosLimEntryField.get())
  calibration.insert(END, J3NegLimEntryField.get())
  calibration.insert(END, J4PosLimEntryField.get())
  calibration.insert(END, J4NegLimEntryField.get())
  calibration.insert(END, J5PosLimEntryField.get())
  calibration.insert(END, J5NegLimEntryField.get())
  calibration.insert(END, J6PosLimEntryField.get())
  calibration.insert(END, J6NegLimEntryField.get())  
  calibration.insert(END, J1StepDegEntryField.get())
  calibration.insert(END, J2StepDegEntryField.get())
  calibration.insert(END, J3StepDegEntryField.get())
  calibration.insert(END, J4StepDegEntryField.get())
  calibration.insert(END, J5StepDegEntryField.get())
  calibration.insert(END, J6StepDegEntryField.get())
  calibration.insert(END, J1DriveMSEntryField.get())
  calibration.insert(END, J2DriveMSEntryField.get())
  calibration.insert(END, J3DriveMSEntryField.get())
  calibration.insert(END, J4DriveMSEntryField.get())
  calibration.insert(END, J5DriveMSEntryField.get())
  calibration.insert(END, J6DriveMSEntryField.get())
  calibration.insert(END, J1EncCPREntryField.get())
  calibration.insert(END, J2EncCPREntryField.get())
  calibration.insert(END, J3EncCPREntryField.get())
  calibration.insert(END, J4EncCPREntryField.get())
  calibration.insert(END, J5EncCPREntryField.get())
  calibration.insert(END, J6EncCPREntryField.get())
  calibration.insert(END, J1ΘEntryField.get())
  calibration.insert(END, J2ΘEntryField.get())
  calibration.insert(END, J3ΘEntryField.get())
  calibration.insert(END, J4ΘEntryField.get())
  calibration.insert(END, J5ΘEntryField.get())
  calibration.insert(END, J6ΘEntryField.get())
  calibration.insert(END, J1αEntryField.get())
  calibration.insert(END, J2αEntryField.get())
  calibration.insert(END, J3αEntryField.get())
  calibration.insert(END, J4αEntryField.get())
  calibration.insert(END, J5αEntryField.get())
  calibration.insert(END, J6αEntryField.get())
  calibration.insert(END, J1dEntryField.get())
  calibration.insert(END, J2dEntryField.get())
  calibration.insert(END, J3dEntryField.get())
  calibration.insert(END, J4dEntryField.get())
  calibration.insert(END, J5dEntryField.get())
  calibration.insert(END, J6dEntryField.get())
  calibration.insert(END, J1aEntryField.get())
  calibration.insert(END, J2aEntryField.get())
  calibration.insert(END, J3aEntryField.get())
  calibration.insert(END, J4aEntryField.get())
  calibration.insert(END, J5aEntryField.get())
  calibration.insert(END, J6aEntryField.get())
  

  ###########
  value=calibration.get(0,END)
  pickle.dump(value,open("ARbot.cal","wb"))

def checkSpeedVals():
  speedtype = speedOption.get()
  Speed = float(speedEntryField.get())
  if(speedtype == "mm per Sec"):
    if(Speed <= .01):
      speedEntryField.delete(0, 'end')
      speedEntryField.insert(0,"5")
  if(speedtype == "Seconds"):
    if(Speed <= .001):
      speedEntryField.delete(0, 'end')
      speedEntryField.insert(0,"1")
  if(speedtype == "Percent"):
    if(Speed <= .01 or Speed > 100):
      speedEntryField.delete(0, 'end')
      speedEntryField.insert(0,"10")
  ACCspd = float(ACCspeedField.get())
  if(ACCspd <= .01 or ACCspd > 100):
    ACCspeedField.delete(0, 'end')
    ACCspeedField.insert(0,"10")
  DECspd = float(DECspeedField.get())
  if(DECspd <= .01 or DECspd >=100):
    DECspeedField.delete(0, 'end')
    DECspeedField.insert(0,"10")
  ACCramp = float(ACCrampField.get())
  if(ACCramp <= .01 or ACCramp > 100):
    ACCrampField.delete(0, 'end')
    ACCrampField.insert(0,"50")



def ErrorHandler(response):
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  ##AXIS LIMIT ERROR
  if (response[1:2] == 'L'):
    if (response[2:3] == '1'):
      message = "J1 Axis Limit"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[3:4] == '1'):
      message = "J2 Axis Limit"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[4:5] == '1'):
      message = "J3 Axis Limit"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[5:6] == '1'):
      message = "J4 Axis Limit"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[6:7] == '1'):
      message = "J5 Axis Limit"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[7:8] == '1'):
      message = "J6 Axis Limit"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[8:9] == '1'):
      message = "J7 Axis Limit"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[9:10] == '1'):
      message = "J8 Axis Limit"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[10:11] == '1'):
      message = "J9 Axis Limit"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))         
    cmdRecEntryField.delete(0, 'end')
    cmdRecEntryField.insert(0,response)            
    message = "Axis Limit Error - See Log"
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    #stopProg()
  ##COLLISION ERROR   
  elif (response[1:2] == 'C'):
    if (response[2:3] == '1'):
      message = "J1 Collision or Motor Error"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
      correctPos()
      stopProg()
      message = "Collision or Motor Error - See Log"
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")   
    if (response[3:4] == '1'):
      message = "J2 Collision or Motor Error"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
      correctPos()
      stopProg()
      message = "Collision or Motor Error - See Log"
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
    if (response[4:5] == '1'):
      message = "J3 Collision or Motor Error"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
      correctPos()
      stopProg()
      message = "Collision or Motor Error - See Log"
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
    if (response[5:6] == '1'):
      message = "J4 Collision or Motor Error"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
      correctPos()
      stopProg()
      message = "Collision or Motor Error - See Log"
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
    if (response[6:7] == '1'):
      message = "J5 Collision or Motor Error"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
      correctPos()
      stopProg()
      message = "Collision or Motor Error - See Log"
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
    if (response[7:8] == '1'):
      message = "J6 Collision or Motor Error"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))  
      correctPos()
      stopProg()        
      message = "Collision or Motor Error - See Log"
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")

  ##REACH ERROR   
  elif (response[1:2] == 'R'):
    stopProg()
    message = "Position Out of Reach"
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")

  ##SPLINE ERROR   
  elif (response[1:2] == 'S'):  
    stopProg()
    message = "Spline Can Only Have Move L Types"
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")

  ##CALIBRATION ERROR 
  elif (response[1:2] == 'A'):  
    if (response[2:3] == '1'):
      message = "J1 CALIBRATION ERROR"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[2:3] == '2'):
      message = "J2 CALIBRATION ERROR"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb")) 
    if (response[2:3] == '3'):
      message = "J3 CALIBRATION ERROR"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb")) 
    if (response[2:3] == '4'):
      message = "J4 CALIBRATION ERROR"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb")) 
    if (response[2:3] == '5'):
      message = "J5 CALIBRATION ERROR"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb")) 
    if (response[2:3] == '6'):
      message = "J6 CALIBRATION ERROR"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb")) 
    if (response[2:3] == '7'):
      message = "J7 CALIBRATION ERROR"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[2:3] == '8'):
      message = "J8 CALIBRATION ERROR"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb")) 
    if (response[2:3] == '9'):
      message = "J9 CALIBRATION ERROR"
      tab8.ElogView.insert(END, Curtime+" - "+message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))                    
     
  else:
    stopProg() 
    message = "Unknown Error"
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
      
	
	

###VISION DEFS###################################################################
#################################################################################	
 
def testvis():  
  visprog = visoptions.get()
  if(visprog[:]== "Openvision"):
    openvision()
  if(visprog[:]== "Roborealm 1.7.5"):
    roborealm175()
  if(visprog[:]== "x,y,r"):
    xyr()	
	
	

def openvision():
  global Xpos
  global Ypos
  global VisEndYmm
  visfail = 1
  while (visfail == 1):
    value = 0
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
    while (value == 0): 
      try:
        f = open(VisFileLoc,"r")
        value = f.readlines()[-1]#.decode()
      except:
        value = 0  
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
    x = int(value[110:122])
    y = int(value[130:142])
    viscalc(x,y)
    if (Ypos > VisEndYmm):
      visfail = 1
      time.sleep(.1)
    else:
      visfail = 0
  open(VisFileLoc,"w").close()	  
  VisXfindEntryField.delete(0, 'end')
  VisXfindEntryField.insert(0,Xpos) 
  VisYfindEntryField.delete(0, 'end')
  VisYfindEntryField.insert(0,Ypos) 
  VisRZfindEntryField.delete(0, 'end')
  VisRZfindEntryField.insert(0,0)
  ##
  VisXpixfindEntryField.delete(0, 'end')
  VisXpixfindEntryField.insert(0,x) 
  VisYpixfindEntryField.delete(0, 'end')
  VisYpixfindEntryField.insert(0,y) 
  ##
  SP_1_E1_EntryField.delete(0, 'end')
  SP_1_E1_EntryField.insert(0,Xpos) 
  SP_1_E2_EntryField.delete(0, 'end')
  SP_1_E2_EntryField.insert(0,Ypos) 
 


  
def roborealm175():
  global Xpos
  global Ypos
  global VisEndYmm
  visfail = 1
  while (visfail == 1):
    value = 0
    almStatusLab.config(text="WAITING FOR CAMERA", style="Alarm.TLabel")
    almStatusLab2.config(text="WAITING FOR CAMERA", style="Alarm.TLabel")
    while (value == 0): 
      try:
        f = open(VisFileLoc,"r")
        value = f.readlines()[-1]#.decode()
      except:
        value = 0 
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
    Index = value.index(',')
    x = float(value[:Index])
    y = float(value[Index+1:])
    viscalc(x,y)
    if (float(Ypos) > float(VisEndYmm)):
      visfail = 1
      time.sleep(.1)
    else:
      visfail = 0
  open(VisFileLoc,"w").close() 
  VisXfindEntryField.delete(0, 'end')
  VisXfindEntryField.insert(0,Xpos) 
  VisYfindEntryField.delete(0, 'end')
  VisYfindEntryField.insert(0,Ypos) 
  VisRZfindEntryField.delete(0, 'end')
  VisRZfindEntryField.insert(0,0)
  ##
  VisXpixfindEntryField.delete(0, 'end')
  VisXpixfindEntryField.insert(0,x) 
  VisYpixfindEntryField.delete(0, 'end')
  VisYpixfindEntryField.insert(0,y) 
  ##
  SP_1_E1_EntryField.delete(0, 'end')
  SP_1_E1_EntryField.insert(0,Xpos) 
  SP_1_E2_EntryField.delete(0, 'end')
  SP_1_E2_EntryField.insert(0,Ypos) 
 


def xyr():
  global Xpos
  global Ypos
  global VisEndYmm
  visfail = 1
  while (visfail == 1):
    value = 0
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
    while (value == 0): 
      try:
        f = open(VisFileLoc,"r")
        value = f.readlines()[-1]#.decode()
      except:
        value = 0 
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
    Index = value.index(',')
    x = float(value[:Index])
    value2 = value[Index+1:]
    Index2 = value2.index(',')
    y = float(value2[:Index2])
    r = float(value2[Index2+1:])
    viscalc(x,y)
    if (Ypos > float(VisEndYmm)):
      visfail = 1
      time.sleep(.1)
    else:
      visfail = 0
  open(VisFileLoc,"w").close() 
  VisXfindEntryField.delete(0, 'end')
  VisXfindEntryField.insert(0,Xpos) 
  VisYfindEntryField.delete(0, 'end')
  VisYfindEntryField.insert(0,Ypos) 
  VisRZfindEntryField.delete(0, 'end')
  VisRZfindEntryField.insert(0,r)
  ##
  VisXpixfindEntryField.delete(0, 'end')
  VisXpixfindEntryField.insert(0,x) 
  VisYpixfindEntryField.delete(0, 'end')
  VisYpixfindEntryField.insert(0,y) 
  ##
  SP_1_E1_EntryField.delete(0, 'end')
  SP_1_E1_EntryField.insert(0,str(Xpos)) 
  SP_1_E2_EntryField.delete(0, 'end')
  SP_1_E2_EntryField.insert(0,str(Ypos)) 
  SP_1_E3_EntryField.delete(0, 'end')
  SP_1_E3_EntryField.insert(0,r)      

    
  

def viscalc():
  global xMMpos
  global yMMpos
  #origin x1 y1
  VisOrigXpix = float(VisX1PixEntryField.get())
  VisOrigXmm = float(VisX1RobEntryField.get()) 
  VisOrigYpix = float(VisY1PixEntryField.get()) 
  VisOrigYmm = float(VisY1RobEntryField.get()) 
  # x2 y2
  VisEndXpix = float(VisX2PixEntryField.get())
  VisEndXmm = float(VisX2RobEntryField.get()) 
  VisEndYpix = float(VisY2PixEntryField.get()) 
  VisEndYmm = float(VisY2RobEntryField.get())

  x = float(VisRetXpixEntryField.get()) 
  y = float(VisRetYpixEntryField.get()) 

  XPrange = float(VisEndXpix) - float(VisOrigXpix)
  XPratio = (x-float(VisOrigXpix)) / XPrange
  XMrange = float(VisEndXmm) - float(VisOrigXmm)
  XMpos = float(XMrange) * float(XPratio)
  xMMpos = float(VisOrigXmm) + XMpos
  ##
  YPrange = float(VisEndYpix) - float(VisOrigYpix)
  YPratio = (y-float(VisOrigYpix)) / YPrange
  YMrange = float(VisEndYmm) - float(VisOrigYmm)
  YMpos = float(YMrange) * float(YPratio)
  yMMpos = float(VisOrigYmm) + YMpos
  return (xMMpos,yMMpos)





# Define function to show frame
def show_frame():

    if cam_on:

        ret, frame = cap.read()    

        if ret:
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)    
            img = Image.fromarray(cv2image).resize((480,320))
            imgtk = ImageTk.PhotoImage(image=img)        
            live_lbl.imgtk = imgtk    
            live_lbl.configure(image=imgtk)    
        
        live_lbl.after(10, show_frame)

def start_vid():
    global cam_on, cap
    global cap
    stop_vid()
    cam_on = True
    curVisStingSel = visoptions.get()
    l = len(camList)
    for i in range(l):
      if (visoptions.get() == camList[i]):
        selectedCam = i
    cap = cv2.VideoCapture(selectedCam) 
    show_frame()

def stop_vid():
    global cam_on
    cam_on = False
    
    if cap:
        cap.release()

#vismenu.size

def take_pic():
  global selectedCam
  global cap
  global BGavg
  global mX1
  global mY1
  global mX2
  global mY2

  if(cam_on == True):
    ret, frame = cap.read()
  else:
    curVisStingSel = visoptions.get()
    l = len(camList)
    for i in range(l):
      if (visoptions.get() == camList[i]):
        selectedCam = i
        #print(selectedCam) 
    cap = cv2.VideoCapture(selectedCam) 
    ret, frame = cap.read()

  brightness = int(VisBrightSlide.get())
  contrast = int(VisContrastSlide.get())
  zoom = int(VisZoomSlide.get())

  frame = np.int16(frame)
  frame = frame * (contrast/127+1) - contrast + brightness
  frame = np.clip(frame, 0, 255)
  frame = np.uint8(frame) 
  cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
  

  #get the webcam size
  height, width = cv2image.shape

  #prepare the crop
  centerX,centerY=int(height/2),int(width/2)
  radiusX,radiusY= int(zoom*height/100),int(zoom*width/100)

  minX,maxX=centerX-radiusX,centerX+radiusX
  minY,maxY=centerY-radiusY,centerY+radiusY

  cropped = cv2image[minX:maxX, minY:maxY]
  cv2image = cv2.resize(cropped, (width, height))

  autoBGVal = int(autoBG.get())
  if(autoBGVal==1):
    BG1 = cv2image[int(VisX1PixEntryField.get())][int(VisY1PixEntryField.get())]
    BG2 = cv2image[int(VisX1PixEntryField.get())][int(VisY2PixEntryField.get())]
    BG3 = cv2image[int(VisX2PixEntryField.get())][int(VisY2PixEntryField.get())]
    avg = int(mean([BG1,BG2,BG3]))
    BGavg = (avg,avg,avg) 
    background = avg
    VisBacColorEntryField.configure(state='enabled')  
    VisBacColorEntryField.delete(0, 'end')
    VisBacColorEntryField.insert(0,str(BGavg))
    VisBacColorEntryField.configure(state='disabled')  
  else:
    temp = VisBacColorEntryField.get()  
    startIndex = temp.find("(")
    endIndex = temp.find(",")
    background = int(temp[startIndex+1:endIndex])
    #background = eval(VisBacColorEntryField.get())

  h = cv2image.shape[0]
  w = cv2image.shape[1]
  # loop over the image
  for y in range(0, h):
    for x in range(0, w):
      # change the pixel
      cv2image[y, x] = background if x >= mX2 or x <= mX1 or y <= mY1 or y >= mY2 else cv2image[y, x]  

  img = Image.fromarray(cv2image).resize((640,480))

  

  imgtk = ImageTk.PhotoImage(image=img) 
  vid_lbl.imgtk = imgtk    
  vid_lbl.configure(image=imgtk) 
  filename = 'curImage.jpg'
  cv2.imwrite(filename, cv2image)


def mask_pic():
  global selectedCam
  global cap
  global BGavg
  global mX1
  global mY1
  global mX2
  global mY2

  if(cam_on == True):
    ret, frame = cap.read()
  else:
    curVisStingSel = visoptions.get()
    l = len(camList)
    for i in range(l):
      if (visoptions.get() == camList[i]):
        selectedCam = i
        #print(selectedCam) 
    cap = cv2.VideoCapture(selectedCam) 
    ret, frame = cap.read()
  brightness = int(VisBrightSlide.get())
  contrast = int(VisContrastSlide.get())
  zoom = int(VisZoomSlide.get())
  frame = np.int16(frame)
  frame = frame * (contrast/127+1) - contrast + brightness
  frame = np.clip(frame, 0, 255)
  frame = np.uint8(frame) 
  cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
  #get the webcam size
  height, width = cv2image.shape
  #prepare the crop
  centerX,centerY=int(height/2),int(width/2)
  radiusX,radiusY= int(zoom*height/100),int(zoom*width/100)
  minX,maxX=centerX-radiusX,centerX+radiusX
  minY,maxY=centerY-radiusY,centerY+radiusY
  cropped = cv2image[minX:maxX, minY:maxY]
  cv2image = cv2.resize(cropped, (width, height))
  #img = Image.fromarray(cv2image).resize((640,480))
  #imgtk = ImageTk.PhotoImage(image=img) 
  #vid_lbl.imgtk = imgtk    
  #vid_lbl.configure(image=imgtk) 
  filename = 'curImage.jpg'
  cv2.imwrite(filename, cv2image)

  



def mask_crop(event, x, y, flags, param):
    global x_start, y_start, x_end, y_end, cropping
    global oriImage
    global box_points
    global button_down
    global mX1
    global mY1
    global mX2
    global mY2


    cropDone = False
    

    if (button_down == False) and (event == cv2.EVENT_LBUTTONDOWN):
        x_start, y_start, x_end, y_end = x, y, x, y
        cropping = True
        button_down = True
        box_points = [(x, y)]
        
    # Mouse is Moving
    elif (button_down == True) and (event == cv2.EVENT_MOUSEMOVE):
        if cropping == True:
            image_copy = oriImage.copy()
            x_end, y_end = x, y
            point = (x, y)
            cv2.rectangle(image_copy, box_points[0], point, (0, 255, 0), 2)
            cv2.imshow("image", image_copy)

    # if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        button_down = False
        box_points.append((x, y))
        cv2.rectangle(oriImage, box_points[0], box_points[1], (0, 255, 0), 2)
        cv2.imshow("image", oriImage)
        # record the ending (x, y) coordinates
        x_end, y_end = x, y
        cropping = False # cropping is finished

        mX1 = x_start+3
        mY1 = y_start+3
        mX2 = x_end-3
        mY2 = y_end-3

        autoBGVal = int(autoBG.get())
        if(autoBGVal==1):
          BG1 = oriImage[int(VisX1PixEntryField.get())][int(VisY1PixEntryField.get())]
          BG2 = oriImage[int(VisX1PixEntryField.get())][int(VisY2PixEntryField.get())]
          BG3 = oriImage[int(VisX2PixEntryField.get())][int(VisY2PixEntryField.get())]
          avg = int(mean([BG1,BG2,BG3]))
          BGavg = (avg,avg,avg) 
          background = avg
          VisBacColorEntryField.configure(state='enabled')  
          VisBacColorEntryField.delete(0, 'end')
          VisBacColorEntryField.insert(0,str(BGavg))
          VisBacColorEntryField.configure(state='disabled')   
        else:  
          background = eval(VisBacColorEntryField.get())

        h = oriImage.shape[0]
        w = oriImage.shape[1]
        # loop over the image
        for y in range(0, h):
            for x in range(0, w):
                # change the pixel
                oriImage[y, x] = background if x >= mX2 or x <= mX1 or y <= mY1 or y >= mY2 else oriImage[y, x]

        img = Image.fromarray(oriImage)
        imgtk = ImageTk.PhotoImage(image=img) 
        vid_lbl.imgtk = imgtk    
        vid_lbl.configure(image=imgtk) 
        filename = 'curImage.jpg'
        cv2.imwrite(filename, oriImage)
        cv2.destroyAllWindows()



def selectMask():
  global oriImage
  global button_down
  button_down = False
  x_start, y_start, x_end, y_end = 0, 0, 0, 0
  mask_pic()

  image = cv2.imread('curImage.jpg')
  oriImage = image.copy()
  
  cv2.namedWindow("image")
  cv2.setMouseCallback("image", mask_crop)
  cv2.imshow("image", image)



def mouse_crop(event, x, y, flags, param):
    global x_start, y_start, x_end, y_end, cropping
    global oriImage
    global box_points
    global button_down

    cropDone = False
    

    if (button_down == False) and (event == cv2.EVENT_LBUTTONDOWN):
        x_start, y_start, x_end, y_end = x, y, x, y
        cropping = True
        button_down = True
        box_points = [(x, y)]
        
    # Mouse is Moving
    elif (button_down == True) and (event == cv2.EVENT_MOUSEMOVE):
        if cropping == True:
            image_copy = oriImage.copy()
            x_end, y_end = x, y
            point = (x, y)
            cv2.rectangle(image_copy, box_points[0], point, (0, 255, 0), 2)
            cv2.imshow("image", image_copy)

    # if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        button_down = False
        box_points.append((x, y))
        cv2.rectangle(oriImage, box_points[0], box_points[1], (0, 255, 0), 2)
        cv2.imshow("image", oriImage)
        # record the ending (x, y) coordinates
        x_end, y_end = x, y
        cropping = False # cropping is finished

        refPoint = [(x_start+3, y_start+3), (x_end-3, y_end-3)]

        if len(refPoint) == 2: #when two points were found
            roi = oriImage[refPoint[0][1]:refPoint[1][1], refPoint[0][0]:refPoint[1][0]]
            
            cv2.imshow("Cropped", roi)
            USER_INP = simpledialog.askstring(title="Teach Vision Object",
                                  prompt="Save Object As:")
            templateName = USER_INP+".jpg"                      
            cv2.imwrite(templateName, roi)
            cv2.destroyAllWindows()
            updateVisOp()  



def selectTemplate():
  global oriImage
  global button_down
  button_down = False
  x_start, y_start, x_end, y_end = 0, 0, 0, 0
  image = cv2.imread('curImage.jpg')
  oriImage = image.copy()
  
  cv2.namedWindow("image")
  cv2.setMouseCallback("image", mouse_crop)
  cv2.imshow("image", image)




def snapFind():
  global selectedTemplate
  global BGavg
  take_pic()
  template = selectedTemplate.get()
  min_score = float(VisScoreEntryField.get())*.01
  autoBGVal = int(autoBG.get())
  if(autoBGVal==1):
    background = BGavg
    VisBacColorEntryField.configure(state='enabled')  
    VisBacColorEntryField.delete(0, 'end')
    VisBacColorEntryField.insert(0,str(BGavg))
    VisBacColorEntryField.configure(state='disabled')  
  else:  
    background = eval(VisBacColorEntryField.get())
  visFind(template,min_score,background)




def rotate_image(img,angle,background):
    image_center = tuple(np.array(img.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, -angle, 1.0)
    result = cv2.warpAffine(img, rot_mat, img.shape[1::-1],borderMode=cv2.BORDER_CONSTANT, borderValue=background, flags=cv2.INTER_LINEAR)
    return result

def visFind(template,min_score,background):
    global xMMpos
    global yMMpos
    global autoBG

    if(background == "Auto"):
      background = BGavg
      VisBacColorEntryField.configure(state='enabled')  
      VisBacColorEntryField.delete(0, 'end')
      VisBacColorEntryField.insert(0,str(BGavg))
      VisBacColorEntryField.configure(state='disabled')  
      

    green = (0,255,0)
    red = (255,0,0)
    blue = (0,0,255)
    dkgreen = (0,128,0)
    status = "fail"
    highscore = 0
    img1 = cv2.imread('curImage.jpg')  # target Image
    img2 = cv2.imread(template)  # target Image
    
    #method = cv2.TM_CCOEFF_NORMED
    #method = cv2.TM_CCORR_NORMED

    img = img1.copy()

    fullRotVal = int(fullRot.get())

    for i in range (1):
      if(i==0):
        method = cv2.TM_CCOEFF_NORMED
      else:
        #method = cv2.TM_CCOEFF_NORMED
        method = cv2.TM_CCORR_NORMED  

      #USE 1/3 - EACH SIDE SEARCH
      if (fullRotVal == 0): 
        ## fist pass 1/3rds
        curangle = 0
        highangle = 0
        highscore = 0
        highmax_loc = 0
        for x in range(3):
          template = img2
          template = rotate_image(img2,curangle,background)
          w, h = template.shape[1::-1]
          res = cv2.matchTemplate(img,template,method)
          min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
          if(max_val>highscore):
            highscore=max_val
            highangle=curangle
            highmax_loc=max_loc
            highw,highh = w,h
          curangle += 120
        
        #check each side and narrow in
        while True:
          curangle=curangle/2
          if(curangle<.9):
            break
          nextangle1 = highangle+curangle
          nextangle2 = highangle-curangle
          template = img2
          template = rotate_image(img2,nextangle1,background)
          w, h = template.shape[1::-1]
          res = cv2.matchTemplate(img,template,method)
          min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
          if(max_val>highscore):
            highscore=max_val
            highangle=nextangle1
            highmax_loc=max_loc
            highw,highh = w,h
          template = img2
          template = rotate_image(img2,nextangle2,background)
          w, h = template.shape[1::-1]
          res = cv2.matchTemplate(img,template,method)
          min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
          if(max_val>highscore):
            highscore=max_val
            highangle=nextangle2
            highmax_loc=max_loc
            highw,highh = w,h     
    
      #USE FULL 360 SEARCh
      else:
        for i in range (720):
          template = rotate_image(img2,i,background)
          w, h = template.shape[1::-1]

          img = img1.copy()
          # Apply template Matching
          res = cv2.matchTemplate(img,template,method)
          min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
          highscore=max_val
          highangle=i
          highmax_loc=max_loc
          highw,highh = w,h
          if highscore >= min_score:
            break
      if(i==1):
        highscore = highscore*.5    
      if highscore >= min_score:
        break         

    if highscore >= min_score:
      status = "pass"
      #normalize angle to increment of +180 to -180
      if(highangle>180):
        highangle = -360 + highangle
      #pick closest 180   
      pick180Val = int(pick180.get())  
      if (pick180Val == 1):
        if (highangle>90):
          highangle = -180 + highangle
        elif (highangle<-90):
          highangle = 180 + highangle
      #try closest
      pickClosestVal = int(pickClosest.get())
      if (pickClosestVal == highangle and highangle>J6PosLim):
        highangle=J6PosLim
      elif (pickClosestVal == 0 and highangle>J6PosLim):    
        status = "fail"
      if (pickClosestVal == 1 and highangle<(J6NegLim*-1)):
        highangle=J6NegLim*-1
      elif (pickClosestVal == 0 and highangle<(J6NegLim*-1)):  
        status = "fail"

      top_left = highmax_loc
      bottom_right = (top_left[0] + highw, top_left[1] + highh)
      #find center
      center = (top_left[0] + highw/2, top_left[1] + highh/2)
      xPos = int(center[1])
      yPos = int(center[0])

      imgxPos = int(center[0])
      imgyPos = int(center[1])

      #find line 1 end
      line1x = int(imgxPos + 60*math.cos(math.radians(highangle-90)))
      line1y = int(imgyPos + 60*math.sin(math.radians(highangle-90)))
      cv2.line(img, (imgxPos,imgyPos), (line1x,line1y), green, 3) 

      #find line 2 end
      line2x = int(imgxPos + 60*math.cos(math.radians(highangle+90)))
      line2y = int(imgyPos + 60*math.sin(math.radians(highangle+90)))
      cv2.line(img, (imgxPos,imgyPos), (line2x,line2y), green, 3)  

      #find line 3 end
      line3x = int(imgxPos + 30*math.cos(math.radians(highangle)))
      line3y = int(imgyPos + 30*math.sin(math.radians(highangle)))
      cv2.line(img, (imgxPos,imgyPos), (line3x,line3y), green, 3)

      #find line 4 end
      line4x = int(imgxPos + 30*math.cos(math.radians(highangle+180)))
      line4y = int(imgyPos + 30*math.sin(math.radians(highangle+180)))
      cv2.line(img, (imgxPos,imgyPos), (line4x,line4y), green, 3) 

      #find tip start
      lineTx = int(imgxPos + 56*math.cos(math.radians(highangle-90)))
      lineTy = int(imgyPos + 56*math.sin(math.radians(highangle-90)))
      cv2.line(img, (lineTx,lineTy), (line1x,line1y), dkgreen, 2) 



      cv2.circle(img, (imgxPos,imgyPos), 20, green, 1)
      #cv2.rectangle(img,top_left, bottom_right, green, 2)
      cv2.imwrite('temp.jpg', img)
      img = Image.fromarray(img).resize((640,480))
      imgtk = ImageTk.PhotoImage(image=img)        
      vid_lbl.imgtk = imgtk    
      vid_lbl.configure(image=imgtk)
      VisRetScoreEntryField.delete(0, 'end')
      VisRetScoreEntryField.insert(0,str(round((highscore*100),2))) 
      VisRetAngleEntryField.delete(0, 'end')
      VisRetAngleEntryField.insert(0,str(highangle)) 
      VisRetXpixEntryField.delete(0, 'end')
      VisRetXpixEntryField.insert(0,str(xPos))
      VisRetYpixEntryField.delete(0, 'end')
      VisRetYpixEntryField.insert(0,str(yPos))           
      viscalc()
      VisRetXrobEntryField .delete(0, 'end')
      VisRetXrobEntryField .insert(0,str(round(xMMpos,2)))  
      VisRetYrobEntryField .delete(0, 'end')
      VisRetYrobEntryField .insert(0,str(round(yMMpos,2)))  

      


          #break
        #if (score > highscore):
          #highscore=score


    if status == "fail":
      cv2.rectangle(img,(5,5), (635,475), red, 5)
      cv2.imwrite('temp.jpg', img)
      img = Image.fromarray(img).resize((640,480))
      imgtk = ImageTk.PhotoImage(image=img)        
      vid_lbl.imgtk = imgtk    
      vid_lbl.configure(image=imgtk)
      VisRetScoreEntryField.delete(0, 'end')
      VisRetScoreEntryField.insert(0,str(round((highscore*100),2)))
      VisRetAngleEntryField.delete(0, 'end')
      VisRetAngleEntryField.insert(0,"NA")
      VisRetXpixEntryField.delete(0, 'end')
      VisRetXpixEntryField.insert(0,"NA")
      VisRetYpixEntryField.delete(0, 'end')
      VisRetYpixEntryField.insert(0,"NA") 

    return (status)    
    





# initial vis attempt using sift with flann pattern match
#def visFind(template):
#  take_pic()
#  MIN_MATCH_COUNT = 10
#  img1 = cv2.imread(template)  # query Image
#  img2 = cv2.imread('curImage.jpg')  # target Image
#  # Initiate SIFT detector
#  sift = cv2.SIFT_create()
#  # find the keypoints and descriptors with SIFT
#  kp1, des1 = sift.detectAndCompute(img1,None)
#  kp2, des2 = sift.detectAndCompute(img2,None)
#  FLANN_INDEX_KDTREE = 1
#  index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
#  search_params = dict(checks = 50)
#  flann = cv2.FlannBasedMatcher(index_params, search_params)
#  matches = flann.knnMatch(des1,des2,k=2)
#  # store all the good matches as per Lowe's ratio test.
#  good = []
#  for m,n in matches:
#      if m.distance < 1.1*n.distance:
#          good.append(m)

#  if len(good)>MIN_MATCH_COUNT:
#      src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
#      dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
#      M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
#      matchesMask = mask.ravel().tolist()
#      h,w,c = img1.shape
#      pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
#      dst = cv2.perspectiveTransform(pts,M)
#      #img2 = cv.polylines(img2,[np.int32(dst)],True,255,3, cv.LINE_AA)
#
#      pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
#      dst = cv2.perspectiveTransform(pts,M)
#
#      crosspts = np.float32([ [w/2,0],[w/2,h-1],[0,h/2],[w-1,h/2] ]).reshape(-1,1,2)
#      crossCoord = cv2.perspectiveTransform(crosspts,M)
#
#      cenPt = np.float32([w/2,h/2]).reshape(-1,1,2)
#      cenCoord = cv2.perspectiveTransform(cenPt,M)
#
#      cenResult = cenCoord[0].reshape(1,-1).flatten().tolist()
#      theta = - math.atan2(M[0,1], M[0,0]) * 180 / math.pi
#
#      xPos = cenResult[0]
#      yPos = cenResult[1]
#
#      cross1Result = crossCoord[0].reshape(2,-1).flatten().tolist()
#      cross2Result = crossCoord[1].reshape(2,-1).flatten().tolist()
#      cross3Result = crossCoord[2].reshape(2,-1).flatten().tolist()
#      cross4Result = crossCoord[3].reshape(2,-1).flatten().tolist()
#
#      x1Pos = int(cross1Result[0])
#      y1Pos = int(cross1Result[1])
#      x2Pos = int(cross2Result[0])
#      y2Pos = int(cross2Result[1])
#      x3Pos = int(cross3Result[0])
#      y3Pos = int(cross3Result[1])
#      x4Pos = int(cross4Result[0])
#      y4Pos = int(cross4Result[1])
#
#
#      print(xPos)
#      print(yPos)
#      print(theta)
#
#
#      #draw bounding box
#      #img2 = cv2.polylines(img2, [np.int32(dst)], True, (0,255,0),3, cv2.LINE_AA)
#
#      #draw circle
#      img2 = cv2.circle(img2, (int(xPos),int(yPos)), radius=30, color=(0, 255, 0), thickness=3)
#
#      #draw line 1
#      cv2.line(img2, (x1Pos,y1Pos), (x2Pos,y2Pos), (0,255,0), 3) 
#      #draw line 2
#      cv2.line(img2, (x3Pos,y3Pos), (x4Pos,y4Pos), (0,255,0), 3)
#
#      #save image
#      cv2.imwrite('curImage.jpg', img2)
#      img = Image.fromarray(img2)
#      imgtk = ImageTk.PhotoImage(image=img)        
#      vid_lbl.imgtk = imgtk    
#      vid_lbl.configure(image=imgtk) 
#
#
#
#
#  else:
#      print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
#      matchesMask = None 





def updateVisOp():
  global selectedTemplate
  selectedTemplate = StringVar()
  folder = os.path.dirname(os.path.realpath(__file__))
  filelist = [fname for fname in os.listdir(folder) if fname.endswith('.jpg')]
  Visoptmenu = ttk.Combobox(tab6, textvariable=selectedTemplate, values=filelist, state='readonly')
  Visoptmenu.place(x=390, y=52)
  Visoptmenu.bind("<<ComboboxSelected>>", VisOpUpdate)

def VisOpUpdate(foo):
  global selectedTemplate
  file = selectedTemplate.get()
  print(file)
  img = cv2.imread(file, cv2.IMREAD_COLOR)
  img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  


  TARGET_PIXEL_AREA = 22500

  ratio = float(img.shape[1]) / float(img.shape[0])
  new_h = int(math.sqrt(TARGET_PIXEL_AREA / ratio) + 0.5)
  new_w = int((new_h * ratio) + 0.5)

  img = cv2.resize(img, (new_w,new_h))



  img = Image.fromarray(img)
  imgtk = ImageTk.PhotoImage(image=img)        
  template_lbl.imgtk = imgtk    
  template_lbl.configure(image=imgtk) 


def zeroBrCn():
  global mX1
  global mY1
  global mX2
  global mY2
  mX1 = 0
  mY1 = 0
  mX2 = 640
  mY2 = 480
  VisBrightSlide.set(0)
  VisContrastSlide.set(0)
  #VisZoomSlide.set(50)
  take_pic()

def VisUpdateBriCon(foo):
  take_pic()  

  
  
       
def motion(event):
    y = event.x
    x = event.y

    if (x<=240 and y<=320):
      VisX1PixEntryField.delete(0, 'end')
      VisX1PixEntryField.insert(0,x)
      VisY1PixEntryField.delete(0, 'end')
      VisY1PixEntryField.insert(0,y)
    elif (x>240):
      VisX2PixEntryField.delete(0, 'end')
      VisX2PixEntryField.insert(0,x)
    elif (y>320):   
      VisY2PixEntryField.delete(0, 'end')
      VisY2PixEntryField.insert(0,y)

    #print(str(x) +","+str(y))
    

def checkAutoBG():
  autoBGVal = int(autoBG.get())
  if(autoBGVal==1):
    VisBacColorEntryField.configure(state='disabled')
  else:
    VisBacColorEntryField.configure(state='enabled')  



### GCODE DEFS ###################################################################
##################################################################################




def gcodeFrame():
  gcodeframe=Frame(tab7)
  gcodeframe.place(x=300,y=10)
  #progframe.pack(side=RIGHT, fill=Y)
  scrollbar = Scrollbar(gcodeframe) 
  scrollbar.pack(side=RIGHT, fill=Y)
  tab7.gcodeView = Listbox(gcodeframe,width=105,height=46, yscrollcommand=scrollbar.set)
  tab7.gcodeView.bind('<<ListboxSelect>>', gcodeViewselect)
  time.sleep(.1)
  tab7.gcodeView.pack()
  scrollbar.config(command=tab7.gcodeView.yview)



def gcodeViewselect(e):
  gcodeRow = tab7.gcodeView.curselection()[0]
  GcodCurRowEntryField.delete(0, 'end')
  GcodCurRowEntryField.insert(0,gcodeRow)  


def loadGcodeProg():
  filetypes = (('gcode files', '*.gcode *.nc *.ngc *.cnc *.tap'),('text files', '*.txt'))
  filename = fd.askopenfilename(title='Open files',initialdir='/',filetypes=filetypes)
  GcodeProgEntryField.delete(0, 'end')
  GcodeProgEntryField.insert(0,filename)
  gcodeProg = open(GcodeProgEntryField.get(),"rb")
  tab7.gcodeView.delete(0,END)
  previtem = ""
  for item in gcodeProg:
    try:
      commentIndex=item.find(b";")
      item = item[:commentIndex]
    except:
      pass
    item=item + b" " 
    if(item != previtem ):
      tab7.gcodeView.insert(END,item)
    previtem = item 
  tab7.gcodeView.pack()
  gcodescrollbar.config(command=tab7.gcodeView.yview)

def SetGcodeStartPos():
  GC_ST_E1_EntryField.delete(0, 'end')
  GC_ST_E1_EntryField.insert(0,str(XcurPos))
  GC_ST_E2_EntryField.delete(0, 'end')
  GC_ST_E2_EntryField.insert(0,str(YcurPos))  
  GC_ST_E3_EntryField.delete(0, 'end')
  GC_ST_E3_EntryField.insert(0,str(ZcurPos))  
  GC_ST_E4_EntryField.delete(0, 'end')
  GC_ST_E4_EntryField.insert(0,str(RzcurPos))  
  GC_ST_E5_EntryField.delete(0, 'end')
  GC_ST_E5_EntryField.insert(0,str(RycurPos))  
  GC_ST_E6_EntryField.delete(0, 'end')
  GC_ST_E6_EntryField.insert(0,str(RxcurPos))  

def MoveGcodeStartPos():
  xVal = str(float(GC_ST_E1_EntryField.get())+float(GC_SToff_E1_EntryField.get()))
  yVal = str(float(GC_ST_E2_EntryField.get())+float(GC_SToff_E2_EntryField.get()))
  zVal = str(float(GC_ST_E3_EntryField.get())+float(GC_SToff_E3_EntryField.get()))
  rzVal = str(float(GC_ST_E4_EntryField.get())+float(GC_SToff_E4_EntryField.get()))
  ryVal = str(float(GC_ST_E5_EntryField.get())+float(GC_SToff_E5_EntryField.get()))
  rxVal = str(float(GC_ST_E6_EntryField.get())+float(GC_SToff_E6_EntryField.get()))
  J7Val = str(J7PosCur)
  J8Val = str(J8PosCur)
  J9Val = str(J9PosCur)
  speedPrefix = "Sm"
  Speed = "25"
  ACCspd = "10"
  DECspd = "10"
  ACCramp = "100"
  WC = "F"
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response) 
  



def GCstepFwd():
    GCalmStatusLab.config(text="GCODE READY",  style="OK.TLabel")
    GCexecuteRow() 
    GCselRow = tab7.gcodeView.curselection()[0]
    last = tab7.gcodeView.index('end')
    for row in range (0,GCselRow):
      tab7.gcodeView.itemconfig(row, {'fg': 'dodger blue'})
    tab7.gcodeView.itemconfig(GCselRow, {'fg': 'blue2'})
    for row in range (GCselRow+1,last):
      tab7.gcodeView.itemconfig(row, {'fg': 'black'})
    tab7.gcodeView.selection_clear(0, END)
    GCselRow += 1
    tab7.gcodeView.select_set(GCselRow)
    try:
      GCselRow = tab7.gcodeView.curselection()[0]
      GcodCurRowEntryField.delete(0, 'end')
      GcodCurRowEntryField.insert(0,GCselRow)
    except:
      GcodCurRowEntryField.delete(0, 'end')
      GcodCurRowEntryField.insert(0,"---")  


def GCrunProg():
  last = tab7.gcodeView.index('end')
  for row in range (0,last):
    tab7.gcodeView.itemconfig(row, {'fg': 'black'})
  def GCthreadProg():
    global GCrowinproc
    global GCstopQueue
    global splineActive
    global prevxVal
    global prevyVal
    global prevzVal
    prevxVal = 0
    prevyVal = 0
    prevzVal = 0
    GCstopQueue = "0"
    splineActive = "0"
    try:
      GCselRow = tab7.gcodeView.curselection()[0]
      if (GCselRow == 0):
        GCselRow=1
    except:
      GCselRow=1
      tab7.gcodeView.selection_clear(0, END)
      tab7.gcodeView.select_set(GCselRow)
    tab7.GCrunTrue = 1
    while tab7.GCrunTrue == 1:
      if (tab7.GCrunTrue == 0):
        GCalmStatusLab.config(text="GCODE PROGRAM STOPPED",  style="Alarm.TLabel")
      else:
        GCalmStatusLab.config(text="GCODE PROGRAM RUNNING",  style="OK.TLabel")
      GCrowinproc = 1
      GCexecuteRow()
      while GCrowinproc == 1:
        time.sleep(.1)	  
      GCselRow = tab7.gcodeView.curselection()[0]
      #last = tab7.gcodeView.index('end')
      #for row in range (0,GCselRow):
      #  tab7.gcodeView.itemconfig(row, {'fg': 'dodger blue'})
      tab7.gcodeView.itemconfig(GCselRow, {'fg': 'blue2'})
      #for row in range (GCselRow+1,last):
      #  tab7.gcodeView.itemconfig(row, {'fg': 'black'})
      tab7.gcodeView.selection_clear(0, END)
      GCselRow += 1
      tab7.gcodeView.select_set(GCselRow)
      #gcodeRow += 1
      #GcodCurRowEntryField.delete(0, 'end')
      #GcodCurRowEntryField.insert(0,GCselRow)
      #time.sleep(.1)
      try:
        GCselRow = tab7.gcodeView.curselection()[0]
        GcodCurRowEntryField.delete(0, 'end')
        GcodCurRowEntryField.insert(0,GCselRow)
      except:
        GcodCurRowEntryField.delete(0, 'end')
        GcodCurRowEntryField.insert(0,"---") 
        tab7.GCrunTrue = 0
        GCalmStatusLab.config(text="GCODE PROGRAM STOPPED",  style="Alarm.TLabel")
  GCt = threading.Thread(target=GCthreadProg)
  GCt.start()    



def GCstopProg():
    global cmdType
    global splineActive
    global GCstopQueue
    lastProg = ""
    tab7.GCrunTrue = 0
    GCalmStatusLab.config(text="GCODE PROGRAM STOPPED",  style="Alarm.TLabel")
    if(splineActive==1):
      splineActive = "0"
      if(stopQueue == "1"):
        stopQueue = "0"
        stop()
      if (moveInProc == 1):
        moveInProc == 2
      command = "SS\n" 
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      ser.write(command.encode())
      ser.flushInput()
      response = str(ser.readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)         

def GCexecuteRow():
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global calStat
  global GCrowinproc
  global LineDist
  global Xv
  global Yv
  global Zv
  global commandCalc
  global moveInProc
  global splineActive
  global stopQueue
  global gcodeSpeed
  global inchTrue
  global prevxVal
  global prevyVal
  global prevzVal
  GCstartTime = time.time()
  GCselRow = tab7.gcodeView.curselection()[0]
  tab7.gcodeView.see(GCselRow+2)
  data = list(map(int, tab7.gcodeView.curselection()))
  command=tab7.gcodeView.get(data[0]).decode()
  cmdType=command[:1]
  subCmd=command[1:command.find(" ")].rstrip()


  ## F ##
  if (cmdType == "F"):
    gcodeSpeed=command[command.find("F")+1:]


  ## G ##
  if (cmdType == "G"):

    #IMPERIAL
    if (subCmd == "20"):
      inchTrue = True; 
    
    #METRIC
    if (subCmd == "21"):
      inchTrue = False;
    
    #ABSOLUTE / INCREMENTAL - HOME (absolute is forced and moves to start position offset)
    if (subCmd == "90" or subCmd == "91" or subCmd == "28"):
      
      xVal = str(float(GC_ST_E1_EntryField.get())+float(GC_SToff_E1_EntryField.get()))
      yVal = str(float(GC_ST_E2_EntryField.get())+float(GC_SToff_E2_EntryField.get()))
      zVal = str(float(GC_ST_E3_EntryField.get())+float(GC_SToff_E3_EntryField.get()))
      rzVal = str(float(GC_ST_E4_EntryField.get())+float(GC_SToff_E4_EntryField.get()))
      ryVal = str(float(GC_ST_E5_EntryField.get())+float(GC_SToff_E5_EntryField.get()))
      rxVal = str(float(GC_ST_E6_EntryField.get())+float(GC_SToff_E6_EntryField.get()))
      J7Val = str(J7PosCur)
      J8Val = str(J8PosCur)
      J9Val = str(J9PosCur)
      speedPrefix = "Sm"
      Speed = "25"
      ACCspd = "10"
      DECspd = "10"
      ACCramp = "100"
      WC = "F"
      LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
      command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      ser.write(command.encode())
      ser.flushInput()
      time.sleep(.1)
      response = str(ser.readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response) 


      # START SPLINE ##
      splineActive = "1"
      if (moveInProc == 1):
        moveInProc == 2
      command = "SL\n" 
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      ser.write(command.encode())
      ser.flushInput()
      time.sleep(.1)
      ser.read()    


    #LINEAR MOVE
    if (subCmd == "0" or subCmd == "1"):

      if("X" in command):
        xtemp=command[command.find("X")+1:]     
        xVal=xtemp[:xtemp.find(" ")]
        xVal=str(round(float(xVal),3))
      else:
        xVal=""  
      if("Y" in command):
        ytemp=command[command.find("Y")+1:]     
        yVal=ytemp[:ytemp.find(" ")]
        yVal=str(round(float(yVal),3))
      else:
        yVal=""
      if("Z" in command):
        ztemp=command[command.find("Z")+1:]     
        zVal=ztemp[:ztemp.find(" ")]
        zVal=str(round(float(zVal),3))
      else:
        zVal=""
      if("A" in command):
        atemp=command[command.find("A")+1:]     
        aVal=atemp[:atemp.find(" ")]
        aVal=str(round(float(aVal),3))
      else:
        aVal=""
      if("B" in command):
        btemp=command[command.find("B")+1:]     
        bVal=btemp[:btemp.find(" ")]
        bVal=str(round(float(bVal),3))
      else:
        bVal=""
      if("C" in command):
        ctemp=command[command.find("C")+1:]     
        cVal=ctemp[:ctemp.find(" ")]
        cVal=str(round(float(cVal),3))
      else:
        cVal=""
      if("E" in command):
        etemp=command[command.find("E")+1:]     
        eVal=etemp[:etemp.find(" ")]
        eVal=str(round(float(eVal),3))
      else:
        eVal=""
      if("F" in command):
        ftemp=command[command.find("F")+1:]     
        fVal=ftemp[:ftemp.find(" ")]
        fVal=str(round(float(fVal),3))
      else:
        fVal=""
       


      if(xVal != ""):
        if(inchTrue == True):
          xVal=str(float(xVal)*25.4)
        xVal = str(round((float(GC_ST_E1_EntryField.get())+float(xVal)),3))
      else:
        try:
          if(prevxVal != 0):
            xVal = prevxVal
          else:  
            xVal = str(XcurPos)
        except:
          xVal = str(XcurPos)   


      if(yVal != ""):
        if(inchTrue == True):
          yVal=str(float(yVal)*25.4)
        yVal = str(round((float(GC_ST_E2_EntryField.get())+float(yVal)),3))
      else:
        try:
          if(prevyVal != 0):
            yVal = prevyVal
          else: 
            yVal = str(YcurPos)
        except:
          yVal = str(YcurPos)  
        
      if(zVal != ""):
        if(inchTrue == True):
          zVal=str(float(zVal)*25.4)
        zVal = str(round((float(GC_ST_E3_EntryField.get())+float(zVal)),3))
      else:
        try:
          if(prevzVal != 0):
            zVal = prevzVal
          else: 
            zVal = str(ZcurPos)
        except:
          zVal = str(ZcurPos)          

      if(aVal != ""):
        rzVal = str(float(GC_ST_E4_EntryField.get())+float(aVal))
        if (np.sign(float(rzVal)) != np.sign(float(RzcurPos))):
          rzVal=str(round((float(rzVal)*-1),3))
      else:
        rzVal = str(RzcurPos)
      
      if(bVal != ""):
        ryVal = str(round((float(GC_ST_E5_EntryField.get())+float(bVal))),3)
      else:
        ryVal = str(RycurPos)

      if(cVal != ""):
        rxVal = str(round((float(GC_ST_E6_EntryField.get())+float(cVal)),3))
      else:
        rxVal = str(RxcurPos)

      if(eVal != ""):
        J7Val = eVal
      else:
        J7Val = str(J7PosCur)
      
      J8Val = str(J8PosCur)
      J9Val = str(J9PosCur)
      
      if(fVal != ""):
        if(inchTrue == True):
          gcodeSpeed = str(round((float(fVal)/25.4),2))
        else:
          gcodeSpeed = str(round((float(fVal)/60),2))  
      speedPrefix = "Sm"
      Speed = gcodeSpeed



      if (subCmd == "0"):
        Speed = speedEntryField.get()

      #FORCE ROTATIONS TO BASE VALUE FOR NOW
      rzVal = GC_ST_E4_EntryField.get()
      ryVal = GC_ST_E5_EntryField.get()
      rxVal = GC_ST_E6_EntryField.get()

      #ACCspd = ACCspeedField.get()
      #DECspd = DECspeedField.get()
      #ACCramp = ACCrampField.get()

      ACCspd = ".1"
      DECspd = ".1"
      ACCramp = "100"


      Rounding = "0"
      WC = "F"
      #LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
      LoopMode ="111111"
      DisWrist = str(DisableWristRot.get())
      command = "MG"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+WC+"Lm"+LoopMode+"Q"+DisWrist+"\n"
      prevxVal = xVal
      prevyVal = yVal
      prevzVal = zVal
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)

      #tab8.ElogView.insert(END, command)
      #value=tab8.ElogView.get(0,END)
      #pickle.dump(value,open("ErrorLog","wb"))

      ser.write(command.encode())
      ser.flushInput()
      time.sleep(.05)
      #ser.read()
      response = str(ser.readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)

  GCrowinproc = 0

  

      


        



  
####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
#####TAB 1



  

###LABELS#################################################################
##########################################################################

CartjogFrame = Frame(tab1, width=1536, height=792,)
CartjogFrame.place(x=330, y=0)

curRowLab = Label(tab1, text = "Current Row:")
curRowLab.place(x=98, y=120)


almStatusLab = Label(tab1, text = "SYSTEM READY - NO ACTIVE ALARMS", style="OK.TLabel")
almStatusLab.place(x=25, y=12)

xbcStatusLab = Label(tab1, text = "Xbox OFF")
xbcStatusLab.place(x=1270, y=80)

runStatusLab = Label(tab1, text = "PROGRAM STOPPED")
runStatusLab.place(x=20, y=150)





ProgLab = Label(tab1, text = "Program:")
ProgLab.place(x=10, y=45)

jogIncrementLab = Label(tab1, text = "Increment Value:")
#jogIncrementLab.place(x=370, y=45)

speedLab = Label(tab1, text = "Speed")
speedLab.place(x=300, y=83)

ACCLab = Label(tab1, text = "Acceleration               %")
ACCLab.place(x=300, y=103)

DECLab = Label(tab1, text = "Deceleration               %")
DECLab.place(x=300, y=123)

DECLab = Label(tab1, text = "Ramp                           %")
DECLab.place(x=300, y=143)

RoundLab = Label(tab1, text = "Rounding               mm")
RoundLab.place(x=525, y=82)




XLab = Label(CartjogFrame, font=("Arial", 18), text = " X")
XLab.place(x=660, y=162)

YLab = Label(CartjogFrame, font=("Arial",18), text = " Y")
YLab.place(x=750, y=162)

ZLab = Label(CartjogFrame, font=("Arial", 18), text = " Z")
ZLab.place(x=840, y=162)

yLab = Label(CartjogFrame, font=("Arial", 18), text = "Rz")
yLab.place(x=930, y=162)

pLab = Label(CartjogFrame, font=("Arial", 18), text = "Ry")
pLab.place(x=1020, y=162)

rLab = Label(CartjogFrame, font=("Arial", 18), text = "Rx")
rLab.place(x=1110, y=162)



TXLab = Label(CartjogFrame, font=("Arial", 18), text = "Tx")
TXLab.place(x=660, y=265)

TYLab = Label(CartjogFrame, font=("Arial",18), text = "Ty")
TYLab.place(x=750, y=265)

TZLab = Label(CartjogFrame, font=("Arial", 18), text = "Tz")
TZLab.place(x=840, y=265)

TyLab = Label(CartjogFrame, font=("Arial", 18), text = "Trz")
TyLab.place(x=930, y=265)

TpLab = Label(CartjogFrame, font=("Arial", 18), text = "Try")
TpLab.place(x=1020, y=265)

J7Lab = Label(CartjogFrame, font=("Arial", 18), text = "Trx")
J7Lab.place(x=1110, y=265)
















### JOINT CONTROL ################################################################
##########################################################################
##J1
J1jogFrame = Frame(tab1, width=340, height=40,)
J1jogFrame.place(x=810, y=10)
J1Lab = Label(J1jogFrame, font=("Arial", 18), text = "J1")
J1Lab.place(x=5, y=5)
J1curAngEntryField = Entry(J1jogFrame,width=5)
J1curAngEntryField.place(x=35, y=9)
def SelJ1jogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J1jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(10)  
J1jogNegBut = Button(J1jogFrame,  text="-", width=3)
J1jogNegBut.bind("<ButtonPress>", SelJ1jogNeg)
J1jogNegBut.bind("<ButtonRelease>", StopJog)
J1jogNegBut.place(x=77, y=7, width=30, height=25)
def SelJ1jogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J1jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(11)  
J1jogPosBut = Button(J1jogFrame, text="+",  width=3)
J1jogPosBut.bind("<ButtonPress>", SelJ1jogPos)
J1jogPosBut.bind("<ButtonRelease>", StopJog)
J1jogPosBut.place(x=300, y=7, width=30, height=25)
J1negLimLab = Label(J1jogFrame, font=("Arial", 8), text = str(-J1NegLim), style="Jointlim.TLabel")
J1negLimLab.place(x=115, y=25)
J1posLimLab = Label(J1jogFrame, font=("Arial", 8), text = str(J1PosLim), style="Jointlim.TLabel")
J1posLimLab.place(x=270, y=25)
J1slidelabel = Label(J1jogFrame)
J1slidelabel.place(x=190, y=25)
def J1sliderUpdate(foo):
  J1slidelabel.config(text=round(float(J1jogslide.get()),2))   
def J1sliderExecute(foo): 
  J1delta = float(J1jogslide.get()) - float(J1curAngEntryField.get())
  if (J1delta < 0):
    J1jogNeg(abs(J1delta))
  else:
    J1jogPos(abs(J1delta))       
J1jogslide = Scale(J1jogFrame, from_=-J1NegLim, to=J1PosLim,  length=180, orient=HORIZONTAL,  command=J1sliderUpdate)
J1jogslide.bind("<ButtonRelease-1>", J1sliderExecute)
J1jogslide.place(x=115, y=7)

##J2
J2jogFrame = Frame(tab1, width=340, height=40,)
J2jogFrame.place(x=810, y=55)
J2Lab = Label(J2jogFrame, font=("Arial", 18), text = "J2")
J2Lab.place(x=5, y=5)
J2curAngEntryField = Entry(J2jogFrame,width=5)
J2curAngEntryField.place(x=35, y=9)
def SelJ2jogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J2jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(20)  
J2jogNegBut = Button(J2jogFrame,  text="-", width=3)
J2jogNegBut.bind("<ButtonPress>", SelJ2jogNeg)
J2jogNegBut.bind("<ButtonRelease>", StopJog)
J2jogNegBut.place(x=77, y=7, width=30, height=25)
def SelJ2jogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J2jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(21)  
J2jogPosBut = Button(J2jogFrame, text="+",  width=3)
J2jogPosBut.bind("<ButtonPress>", SelJ2jogPos)
J2jogPosBut.bind("<ButtonRelease>", StopJog)
J2jogPosBut.place(x=300, y=7, width=30, height=25)
J2negLimLab = Label(J2jogFrame, font=("Arial", 8), text = str(-J2NegLim), style="Jointlim.TLabel")
J2negLimLab.place(x=115, y=25)
J2posLimLab = Label(J2jogFrame, font=("Arial", 8), text = str(J2PosLim), style="Jointlim.TLabel")
J2posLimLab.place(x=270, y=25)
J2slidelabel = Label(J2jogFrame)
J2slidelabel.place(x=190, y=25)
def J2sliderUpdate(foo):
  J2slidelabel.config(text=round(float(J2jogslide.get()),2))   
def J2sliderExecute(foo): 
  J2delta = float(J2jogslide.get()) - float(J2curAngEntryField.get())
  if (J2delta < 0):
    J2jogNeg(abs(J2delta))
  else:
    J2jogPos(abs(J2delta))       
J2jogslide = Scale(J2jogFrame, from_=-J2NegLim, to=J2PosLim,  length=180, orient=HORIZONTAL,  command=J2sliderUpdate)
J2jogslide.bind("<ButtonRelease-1>", J2sliderExecute)
J2jogslide.place(x=115, y=7)

##J3
J3jogFrame = Frame(tab1, width=340, height=40,)
J3jogFrame.place(x=810, y=100)
J3Lab = Label(J3jogFrame, font=("Arial", 18), text = "J3")
J3Lab.place(x=5, y=5)
J3curAngEntryField = Entry(J3jogFrame,width=5)
J3curAngEntryField.place(x=35, y=9)
def SelJ3jogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J3jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(30)  
J3jogNegBut = Button(J3jogFrame,  text="-", width=3)
J3jogNegBut.bind("<ButtonPress>", SelJ3jogNeg)
J3jogNegBut.bind("<ButtonRelease>", StopJog)
J3jogNegBut.place(x=77, y=7, width=30, height=25)
def SelJ3jogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J3jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(31)  
J3jogPosBut = Button(J3jogFrame, text="+",  width=3)
J3jogPosBut.bind("<ButtonPress>", SelJ3jogPos)
J3jogPosBut.bind("<ButtonRelease>", StopJog)
J3jogPosBut.place(x=300, y=7, width=30, height=25)
J3negLimLab = Label(J3jogFrame, font=("Arial", 8), text = str(-J3NegLim), style="Jointlim.TLabel")
J3negLimLab.place(x=115, y=25)
J3posLimLab = Label(J3jogFrame, font=("Arial", 8), text = str(J3PosLim), style="Jointlim.TLabel")
J3posLimLab.place(x=270, y=25)
J3slidelabel = Label(J3jogFrame)
J3slidelabel.place(x=190, y=25)
def J3sliderUpdate(foo):
  J3slidelabel.config(text=round(float(J3jogslide.get()),2))   
def J3sliderExecute(foo): 
  J3delta = float(J3jogslide.get()) - float(J3curAngEntryField.get())
  if (J3delta < 0):
    J3jogNeg(abs(J3delta))
  else:
    J3jogPos(abs(J3delta))       
J3jogslide = Scale(J3jogFrame, from_=-J3NegLim, to=J3PosLim,  length=180, orient=HORIZONTAL,  command=J3sliderUpdate)
J3jogslide.bind("<ButtonRelease-1>", J3sliderExecute)
J3jogslide.place(x=115, y=7)

##J4
J4jogFrame = Frame(tab1, width=340, height=40,)
J4jogFrame.place(x=1160, y=10)
J4Lab = Label(J4jogFrame, font=("Arial", 18), text = "J4")
J4Lab.place(x=5, y=5)
J4curAngEntryField = Entry(J4jogFrame,width=5)
J4curAngEntryField.place(x=35, y=9)
def SelJ4jogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J4jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(40)  
J4jogNegBut = Button(J4jogFrame,  text="-", width=3)
J4jogNegBut.bind("<ButtonPress>", SelJ4jogNeg)
J4jogNegBut.bind("<ButtonRelease>", StopJog)
J4jogNegBut.place(x=77, y=7, width=30, height=25)
def SelJ4jogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J4jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(41)  
J4jogPosBut = Button(J4jogFrame, text="+",  width=3)
J4jogPosBut.bind("<ButtonPress>", SelJ4jogPos)
J4jogPosBut.bind("<ButtonRelease>", StopJog)
J4jogPosBut.place(x=300, y=7, width=30, height=25)
J4negLimLab = Label(J4jogFrame, font=("Arial", 8), text = str(-J4NegLim), style="Jointlim.TLabel")
J4negLimLab.place(x=115, y=25)
J4posLimLab = Label(J4jogFrame, font=("Arial", 8), text = str(J4PosLim), style="Jointlim.TLabel")
J4posLimLab.place(x=270, y=25)
J4slidelabel = Label(J4jogFrame)
J4slidelabel.place(x=190, y=25)
def J4sliderUpdate(foo):
  J4slidelabel.config(text=round(float(J4jogslide.get()),2))   
def J4sliderExecute(foo): 
  J4delta = float(J4jogslide.get()) - float(J4curAngEntryField.get())
  if (J4delta < 0):
    J4jogNeg(abs(J4delta))
  else:
    J4jogPos(abs(J4delta))       
J4jogslide = Scale(J4jogFrame, from_=-J4NegLim, to=J4PosLim,  length=180, orient=HORIZONTAL,  command=J4sliderUpdate)
J4jogslide.bind("<ButtonRelease-1>", J4sliderExecute)
J4jogslide.place(x=115, y=7)

##J5
J5jogFrame = Frame(tab1, width=340, height=40,)
J5jogFrame.place(x=1160, y=55)
J5Lab = Label(J5jogFrame, font=("Arial", 18), text = "J5")
J5Lab.place(x=5, y=5)
J5curAngEntryField = Entry(J5jogFrame,width=5)
J5curAngEntryField.place(x=35, y=9)
def SelJ5jogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J5jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(50)  
J5jogNegBut = Button(J5jogFrame,  text="-", width=3)
J5jogNegBut.bind("<ButtonPress>", SelJ5jogNeg)
J5jogNegBut.bind("<ButtonRelease>", StopJog)
J5jogNegBut.place(x=77, y=7, width=30, height=25)
def SelJ5jogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J5jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(51)  
J5jogPosBut = Button(J5jogFrame, text="+",  width=3)
J5jogPosBut.bind("<ButtonPress>", SelJ5jogPos)
J5jogPosBut.bind("<ButtonRelease>", StopJog)
J5jogPosBut.place(x=300, y=7, width=30, height=25)
J5negLimLab = Label(J5jogFrame, font=("Arial", 8), text = str(-J5NegLim), style="Jointlim.TLabel")
J5negLimLab.place(x=115, y=25)
J5posLimLab = Label(J5jogFrame, font=("Arial", 8), text = str(J5PosLim), style="Jointlim.TLabel")
J5posLimLab.place(x=270, y=25)
J5slidelabel = Label(J5jogFrame)
J5slidelabel.place(x=190, y=25)
def J5sliderUpdate(foo):
  J5slidelabel.config(text=round(float(J5jogslide.get()),2))   
def J5sliderExecute(foo): 
  J5delta = float(J5jogslide.get()) - float(J5curAngEntryField.get())
  if (J5delta < 0):
    J5jogNeg(abs(J5delta))
  else:
    J5jogPos(abs(J5delta))       
J5jogslide = Scale(J5jogFrame, from_=-J5NegLim, to=J5PosLim,  length=180, orient=HORIZONTAL,  command=J5sliderUpdate)
J5jogslide.bind("<ButtonRelease-1>", J5sliderExecute)
J5jogslide.place(x=115, y=7)

##J6
J6jogFrame = Frame(tab1, width=340, height=40,)
J6jogFrame.place(x=1160, y=100)
J6Lab = Label(J6jogFrame, font=("Arial", 18), text = "J6")
J6Lab.place(x=5, y=5)
J6curAngEntryField = Entry(J6jogFrame,width=5)
J6curAngEntryField.place(x=35, y=9)
def SelJ6jogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J6jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(60)  
J6jogNegBut = Button(J6jogFrame,  text="-", width=3)
J6jogNegBut.bind("<ButtonPress>", SelJ6jogNeg)
J6jogNegBut.bind("<ButtonRelease>", StopJog)
J6jogNegBut.place(x=77, y=7, width=30, height=25)
def SelJ6jogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J6jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(61)  
J6jogPosBut = Button(J6jogFrame, text="+",  width=3)
J6jogPosBut.bind("<ButtonPress>", SelJ6jogPos)
J6jogPosBut.bind("<ButtonRelease>", StopJog)
J6jogPosBut.place(x=300, y=7, width=30, height=25)
J6negLimLab = Label(J6jogFrame, font=("Arial", 8), text = str(-J6NegLim), style="Jointlim.TLabel")
J6negLimLab.place(x=115, y=25)
J6posLimLab = Label(J6jogFrame, font=("Arial", 8), text = str(J6PosLim), style="Jointlim.TLabel")
J6posLimLab.place(x=270, y=25)
J6slidelabel = Label(J6jogFrame)
J6slidelabel.place(x=190, y=25)
def J6sliderUpdate(foo):
  J6slidelabel.config(text=round(float(J6jogslide.get()),2))   
def J6sliderExecute(foo): 
  J6delta = float(J6jogslide.get()) - float(J6curAngEntryField.get())
  if (J6delta < 0):
    J6jogNeg(abs(J6delta))
  else:
    J6jogPos(abs(J6delta))       
J6jogslide = Scale(J6jogFrame, from_=-J6NegLim, to=J6PosLim,  length=180, orient=HORIZONTAL,  command=J6sliderUpdate)
J6jogslide.bind("<ButtonRelease-1>", J6sliderExecute)
J6jogslide.place(x=115, y=7)





J7jogFrame = Frame(tab1, width=145, height=100)
J7jogFrame['relief'] = 'raised'
J7jogFrame.place(x=1340, y=350)
J7Lab = Label(J7jogFrame, font=("Arial", 14), text = "7th Axis")
J7Lab.place(x=15, y=5)
J7curAngEntryField = Entry(J7jogFrame,width=5)
J7curAngEntryField.place(x=95, y=9)
def SelJ7jogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J7jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(70) 
J7jogNegBut = Button(J7jogFrame,  text="-", width=3)
J7jogNegBut.bind("<ButtonPress>", SelJ7jogNeg)
J7jogNegBut.bind("<ButtonRelease>", StopJog)
J7jogNegBut.place(x=10, y=65, width=30, height=25)
def SelJ7jogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J7jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(71)  
J7jogPosBut = Button(J7jogFrame, text="+",  width=3)
J7jogPosBut.bind("<ButtonPress>", SelJ7jogPos)
J7jogPosBut.bind("<ButtonRelease>", StopJog)
J7jogPosBut.place(x=105, y=65, width=30, height=25)
J7negLimLab = Label(J7jogFrame, font=("Arial", 8), text = str(-J7NegLim), style="Jointlim.TLabel")
J7negLimLab.place(x=10, y=30)
J7posLimLab = Label(J7jogFrame, font=("Arial", 8), text = str(J7PosLim), style="Jointlim.TLabel")
J7posLimLab.place(x=110, y=30)
J7slideLimLab = Label(J7jogFrame)
J7slideLimLab.place(x=60, y=70)
def J7sliderUpdate(foo):
  J7slideLimLab.config(text=round(float(J7jogslide.get()),2))   
def J7sliderExecute(foo): 
  J7delta = float(J7jogslide.get()) - float(J7curAngEntryField.get())
  if (J7delta < 0):
    J7jogNeg(abs(J7delta))
  else:
    J7jogPos(abs(J7delta))       
J7jogslide = Scale(J7jogFrame, from_=-J7NegLim, to=J7PosLim,  length=125, orient=HORIZONTAL,  command=J7sliderUpdate)
J7jogslide.bind("<ButtonRelease-1>", J7sliderExecute)
J7jogslide.place(x=10, y=43)


J8jogFrame = Frame(tab1, width=145, height=100)
J8jogFrame['relief'] = 'raised'
J8jogFrame.place(x=1340, y=460)
J8Lab = Label(J8jogFrame, font=("Arial", 14), text = "8th Axis")
J8Lab.place(x=15, y=5)
J8curAngEntryField = Entry(J8jogFrame,width=5)
J8curAngEntryField.place(x=95, y=9)
def SelJ8jogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J8jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(80) 
J8jogNegBut = Button(J8jogFrame,  text="-", width=3)
J8jogNegBut.bind("<ButtonPress>", SelJ8jogNeg)
J8jogNegBut.bind("<ButtonRelease>", StopJog)
J8jogNegBut.place(x=10, y=65, width=30, height=25)
def SelJ8jogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J8jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(81)  
J8jogPosBut = Button(J8jogFrame, text="+",  width=3)
J8jogPosBut.bind("<ButtonPress>", SelJ8jogPos)
J8jogPosBut.bind("<ButtonRelease>", StopJog)
J8jogPosBut.place(x=105, y=65, width=30, height=25)
J8negLimLab = Label(J8jogFrame, font=("Arial", 8), text = str(-J8NegLim), style="Jointlim.TLabel")
J8negLimLab.place(x=10, y=30)
J8posLimLab = Label(J8jogFrame, font=("Arial", 8), text = str(J8PosLim), style="Jointlim.TLabel")
J8posLimLab.place(x=110, y=30)
J8slideLimLab = Label(J8jogFrame)
J8slideLimLab.place(x=60, y=70)
def J8sliderUpdate(foo):
  J8slideLimLab.config(text=round(float(J8jogslide.get()),2))   
def J8sliderExecute(foo): 
  J8delta = float(J8jogslide.get()) - float(J8curAngEntryField.get())
  if (J8delta < 0):
    J8jogNeg(abs(J8delta))
  else:
    J8jogPos(abs(J8delta))       
J8jogslide = Scale(J8jogFrame, from_=-J8NegLim, to=J8PosLim,  length=125, orient=HORIZONTAL,  command=J8sliderUpdate)
J8jogslide.bind("<ButtonRelease-1>", J8sliderExecute)
J8jogslide.place(x=10, y=43)


J9jogFrame = Frame(tab1, width=145, height=100)
J9jogFrame['relief'] = 'raised'
J9jogFrame.place(x=1340, y=570)
J9Lab = Label(J9jogFrame, font=("Arial", 14), text = "9th Axis")
J9Lab.place(x=15, y=5)
J9curAngEntryField = Entry(J9jogFrame,width=5)
J9curAngEntryField.place(x=95, y=9)
def SelJ9jogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J9jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(90) 
J9jogNegBut = Button(J9jogFrame,  text="-", width=3)
J9jogNegBut.bind("<ButtonPress>", SelJ9jogNeg)
J9jogNegBut.bind("<ButtonRelease>", StopJog)
J9jogNegBut.place(x=10, y=65, width=30, height=25)
def SelJ9jogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    J9jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(91)  
J9jogPosBut = Button(J9jogFrame, text="+",  width=3)
J9jogPosBut.bind("<ButtonPress>", SelJ9jogPos)
J9jogPosBut.bind("<ButtonRelease>", StopJog)
J9jogPosBut.place(x=105, y=65, width=30, height=25)
J9negLimLab = Label(J9jogFrame, font=("Arial", 8), text = str(-J9NegLim), style="Jointlim.TLabel")
J9negLimLab.place(x=10, y=30)
J9posLimLab = Label(J9jogFrame, font=("Arial", 8), text = str(J9PosLim), style="Jointlim.TLabel")
J9posLimLab.place(x=110, y=30)
J9slideLimLab = Label(J9jogFrame)
J9slideLimLab.place(x=60, y=70)
def J9sliderUpdate(foo):
  J9slideLimLab.config(text=round(float(J9jogslide.get()),2))   
def J9sliderExecute(foo): 
  J9delta = float(J9jogslide.get()) - float(J9curAngEntryField.get())
  if (J9delta < 0):
    J9jogNeg(abs(J9delta))
  else:
    J9jogPos(abs(J9delta))       
J9jogslide = Scale(J9jogFrame, from_=-J9NegLim, to=J9PosLim,  length=125, orient=HORIZONTAL,  command=J9sliderUpdate)
J9jogslide.bind("<ButtonRelease-1>", J9sliderExecute)
J9jogslide.place(x=10, y=43)


####ENTRY FIELDS##########################################################
##########################################################################


progframe=Frame(tab1)
progframe.place(x=7,y=174)
scrollbar = Scrollbar(progframe) 
scrollbar.pack(side=RIGHT, fill=Y)
tab1.progView = Listbox(progframe,exportselection=0,width=105,height=31, yscrollcommand=scrollbar.set)
tab1.progView.bind('<<ListboxSelect>>', progViewselect)
tab1.progView.pack()
scrollbar.config(command=tab1.progView.yview)



incrementEntryField = Entry(tab1,width=4)
incrementEntryField.place(x=380, y=45)

curRowEntryField = Entry(tab1,width=4)
curRowEntryField.place(x=174, y=120)

manEntryField = Entry(tab1,width=105)
manEntryField.place(x=10, y=700)

ProgEntryField = Entry(tab1,width=20)
ProgEntryField.place(x=70, y=45)



speedEntryField = Entry(tab1,width=4)
speedEntryField.place(x=380, y=80)

ACCspeedField = Entry(tab1,width=4)
ACCspeedField.place(x=380, y=100)

DECspeedField = Entry(tab1,width=4)
DECspeedField.place(x=380, y=120)

ACCrampField = Entry(tab1,width=4)
ACCrampField.place(x=380, y=140)

roundEntryField = Entry(tab1,width=4)
roundEntryField.place(x=590, y=80)














  ### X ###

XcurEntryField = Entry(CartjogFrame,width=5)
XcurEntryField.place(x=660, y=195)


   ### Y ###

YcurEntryField = Entry(CartjogFrame,width=5)
YcurEntryField.place(x=750, y=195)


   ### Z ###

ZcurEntryField = Entry(CartjogFrame,width=5)
ZcurEntryField.place(x=840, y=195)


   ### Rz ###

RzcurEntryField = Entry(CartjogFrame,width=5)
RzcurEntryField.place(x=930, y=195)


   ### Ry ###

RycurEntryField = Entry(CartjogFrame,width=5)
RycurEntryField.place(x=1020, y=195)


   ### Rx ###

RxcurEntryField = Entry(CartjogFrame,width=5)
RxcurEntryField.place(x=1110, y=195)



###BUTTONS################################################################
##########################################################################


def posRegFieldVisible(self):
  curCmdtype = options.get()
  if (curCmdtype=="Move PR" or curCmdtype=="OFF PR " or curCmdtype=="Teach PR"):
    SavePosEntryField.place(x=780, y=183)
  else:
    SavePosEntryField.place_forget()

getSelBut = Button(tab1,  text="Get Selected", width=12,  command = getSel)
getSelBut.place(x=10, y=725)

manInsBut = Button(tab1, text="Insert", width=12,  command = manInsItem)
manInsBut.place(x=105, y=725)

manRepBut = Button(tab1,  text="Replace", width=12,  command = manReplItem)
manRepBut.place(x=200, y=725)

openTextBut = Button(tab1,  text="Open Text", width=12,  command = openText)
openTextBut.place(x=295, y=725)

reloadProgBut = Button(tab1,  text="Reload", width=12,  command = reloadProg)
reloadProgBut.place(x=390, y=725)


speedOption=StringVar(tab1)
speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
speedMenu.place(x=412, y=76)



#single buttons

options=StringVar(tab1)
menu=OptionMenu(tab1, options, "Move J", "Move J", "OFF J", "Move L", "Move R", "Move A Mid", "Move A End", "Move C Center", "Move C Start", "Move C Plane", "Start Spline", "End Spline", "Move PR", "OFF PR ", "Teach PR", "Move Vis", command=posRegFieldVisible)
menu.grid(row=2,column=2)
menu.config(width=18)
menu.place(x=700, y=180)

SavePosEntryField = Entry(tab1,width=5)
#SavePosEntryField.place(x=800, y=183)


teachInsBut = Button(tab1,  text="Teach New Position", width=22, command = teachInsertBelSelected)
teachInsBut.place(x=700, y=220)

teachReplaceBut = Button(tab1, text="Modify Position", width=22,  command = teachReplaceSelected)
teachReplaceBut.place(x=700, y=260)

deleteBut = Button(tab1,  text="Delete", width=22,   command = deleteitem)
deleteBut.place(x=700, y=300)

CalibrateBut = Button(tab1,  text="Auto Calibrate CMD", width=22,   command = insCalibrate)
CalibrateBut.place(x=700, y=340)

camOnBut = Button(tab1,  text="Camera On",  width=22, command = cameraOn)
camOnBut.place(x=700, y=380)

camOffBut = Button(tab1,  text="Camera Off",  width=22, command = cameraOff)
camOffBut.place(x=700, y=420)



#buttons with 1 entry

waitTimeBut = Button(tab1, text="Wait Time (seconds)",  width=22,  command = waitTime)
waitTimeBut.place(x=700, y=460)

waitInputOnBut = Button(tab1, text="Wait Input ON",  width=22,  command = waitInputOn)
waitInputOnBut.place(x=700, y=500)

waitInputOffBut = Button(tab1,  text="Wait Input OFF",  width=22,  command = waitInputOff)
waitInputOffBut.place(x=700, y=540)

setOutputOnBut = Button(tab1,  text="Set Output On",  width=22,  command = setOutputOn)
setOutputOnBut.place(x=700, y=580)

setOutputOffBut = Button(tab1,  text="Set Output OFF",  width=22,   command = setOutputOff)
setOutputOffBut.place(x=700, y=620)

tabNumBut = Button(tab1,  text="Create Tab",  width=22, command = tabNumber)
tabNumBut.place(x=700, y=660)

jumpTabBut = Button(tab1,  text="Jump to Tab",  width=22, command = jumpTab)
jumpTabBut.place(x=700, y=700)




waitTimeEntryField = Entry(tab1,width=5)
waitTimeEntryField.place(x=855, y=465)

waitInputEntryField = Entry(tab1,width=5)
waitInputEntryField.place(x=855, y=505)

waitInputOffEntryField = Entry(tab1,width=5)
waitInputOffEntryField.place(x=855, y=545)

outputOnEntryField = Entry(tab1,width=5)
outputOnEntryField.place(x=855, y=585)

outputOffEntryField = Entry(tab1,width=5)
outputOffEntryField.place(x=855, y=625)

tabNumEntryField = Entry(tab1,width=5)
tabNumEntryField.place(x=855, y=665)

jumpTabEntryField = Entry(tab1,width=5)
jumpTabEntryField.place(x=855, y=705)


#buttons with multiple entry

IfOnjumpTabBut = Button(tab1,  text="If On Jump",  width=22,   command = IfOnjumpTab)
IfOnjumpTabBut.place(x=950, y=360)

IfOffjumpTabBut = Button(tab1,  text="If Off Jump",  width=22,   command = IfOffjumpTab)
IfOffjumpTabBut.place(x=950, y=400)

servoBut = Button(tab1,  text="Servo",  width=22,   command = Servo)
servoBut.place(x=950, y=440)

RegNumBut = Button(tab1,  text="Register",  width=22,   command = insertRegister)
RegNumBut.place(x=950, y=480)

RegJmpBut = Button(tab1,  text="If Register Jump",  width=22,   command = IfRegjumpTab)
RegJmpBut.place(x=950, y=520)

StorPosBut = Button(tab1,  text="Position Register",  width=22,   command = storPos)
StorPosBut.place(x=950, y=560)

callBut = Button(tab1,  text="Call Program",  width=22,   command = insertCallProg)
callBut.place(x=950, y=600)

returnBut = Button(tab1,  text="Return",  width=22,   command = insertReturn)
returnBut.place(x=950, y=640)

visFindBut = Button(tab1,  text="Vision Find",  width=22,   command = insertvisFind)
visFindBut.place(x=950, y=680)

##
IfOnjumpInputTabEntryField = Entry(tab1,width=5)
IfOnjumpInputTabEntryField.place(x=1107, y=363)

IfOnjumpNumberTabEntryField = Entry(tab1,width=5)
IfOnjumpNumberTabEntryField.place(x=1147, y=363)

IfOffjumpInputTabEntryField = Entry(tab1,width=5)
IfOffjumpInputTabEntryField.place(x=1107, y=403)

IfOffjumpNumberTabEntryField = Entry(tab1,width=5)
IfOffjumpNumberTabEntryField.place(x=1147, y=403)

servoNumEntryField = Entry(tab1,width=5)
servoNumEntryField.place(x=1107, y=443)

servoPosEntryField = Entry(tab1,width=5)
servoPosEntryField.place(x=1147, y=443)

regNumEntryField = Entry(tab1,width=5)
regNumEntryField.place(x=1107, y=483)

regEqEntryField = Entry(tab1,width=5)
regEqEntryField.place(x=1147, y=483)

regNumJmpEntryField = Entry(tab1,width=5)
regNumJmpEntryField.place(x=1107, y=523)

regEqJmpEntryField = Entry(tab1,width=5)
regEqJmpEntryField.place(x=1147, y=523)

regTabJmpEntryField = Entry(tab1,width=5)
regTabJmpEntryField.place(x=1187, y=523)

storPosNumEntryField = Entry(tab1,width=5)
storPosNumEntryField.place(x=1107, y=563)

storPosElEntryField = Entry(tab1,width=5)
storPosElEntryField.place(x=1147, y=563)

storPosValEntryField = Entry(tab1,width=5)
storPosValEntryField.place(x=1187, y=563)

changeProgEntryField = Entry(tab1,width=22)
changeProgEntryField.place(x=1107, y=603)

visPassEntryField = Entry(tab1,width=5)
visPassEntryField.place(x=1107, y=683)

visFailEntryField = Entry(tab1,width=5)
visFailEntryField.place(x=1147, y=683)

manEntLab = Label(tab1, font=("Arial", 6), text = "Manual Program Entry")
manEntLab.place(x=10, y=685)

ifOnLab = Label(tab1,font=("Arial", 6), text = " Input            Tab")
ifOnLab.place(x=1107, y=350)

ifOffLab = Label(tab1,font=("Arial", 6), text = " Input            Tab")
ifOffLab.place(x=1107, y=390) 

regEqLab = Label(tab1,font=("Arial", 6), text = "Register       (++/--)")
regEqLab.place(x=1107, y=469)

ifregTabJmpLab = Label(tab1,font=("Arial", 6), text = "Register        Num         Tab")
ifregTabJmpLab.place(x=1107, y=509)

servoLab = Label(tab1,font=("Arial", 6), text = "Number      Position")
servoLab.place(x=1107, y=430)

storPosEqLab = Label(tab1,font=("Arial", 6), text = " Pos Reg      Element       (++/--)")
storPosEqLab.place(x=1107, y=549)

visPassLab = Label(tab1,font=("Arial", 6), text = "Pass Tab     Fail Tab")
visPassLab.place(x=1107, y=670)

ProgBut = Button(tab1,  text="Load", width=10,  command = loadProg)
ProgBut.place(x=202, y=42)

CreateBut = Button(tab1,  text="New Prog", width=10,  command = CreateProg)
CreateBut.place(x=285, y=42)


runProgBut = Button(tab1,   command = runProg)
playPhoto=PhotoImage(file="play-icon.gif")
runProgBut.config(image=playPhoto)
runProgBut.place(x=20, y=80)

xboxBut = Button(tab1,  command = xbox)
xboxPhoto=PhotoImage(file="xbox.gif")
xboxBut.config(image=xboxPhoto)
xboxBut.place(x=700, y=80)

stopProgBut = Button(tab1,   command = stopProg)
stopPhoto=PhotoImage(file="stop-icon.gif")
stopProgBut.config(image=stopPhoto)
stopProgBut.place(x=220, y=80)

revBut = Button(tab1,  text="REV ",  command = stepRev)
revBut.place(x=105, y=80)

fwdBut = Button(tab1,  text="FWD", command = stepFwd)
fwdBut.place(x=160, y=80)

IncJogCbut = Checkbutton(tab1, text="Incremental Jog",variable = IncJogStat)
IncJogCbut.place(x=412, y=46)


def SelXjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    XjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(10)  

XjogNegBut = Button(CartjogFrame, text="-",  width=3)
XjogNegBut.bind("<ButtonPress>", SelXjogNeg)
XjogNegBut.bind("<ButtonRelease>", StopJog)
XjogNegBut.place(x=642, y=225, width=30, height=25)


def SelXjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    XjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(11) 

XjogPosBut = Button(CartjogFrame, text="+",  width=3)
XjogPosBut.bind("<ButtonPress>", SelXjogPos)
XjogPosBut.bind("<ButtonRelease>", StopJog)
XjogPosBut.place(x=680, y=225, width=30, height=25)

def SelYjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    YjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(20)

YjogNegBut = Button(CartjogFrame, text="-",  width=3)
YjogNegBut.bind("<ButtonPress>", SelYjogNeg)
YjogNegBut.bind("<ButtonRelease>", StopJog)
YjogNegBut.place(x=732, y=225, width=30, height=25)

def SelYjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    YjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(21)

YjogPosBut = Button(CartjogFrame, text="+",  width=3)
YjogPosBut.bind("<ButtonPress>", SelYjogPos)
YjogPosBut.bind("<ButtonRelease>", StopJog)
YjogPosBut.place(x=770, y=225, width=30, height=25)

def SelZjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    ZjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(30)

ZjogNegBut = Button(CartjogFrame, text="-",  width=3)
ZjogNegBut.bind("<ButtonPress>", SelZjogNeg)
ZjogNegBut.bind("<ButtonRelease>", StopJog)
ZjogNegBut.place(x=822, y=225, width=30, height=25)

def SelZjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    ZjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(31)

ZjogPosBut = Button(CartjogFrame, text="+",  width=3)
ZjogPosBut.bind("<ButtonPress>", SelZjogPos)
ZjogPosBut.bind("<ButtonRelease>", StopJog)
ZjogPosBut.place(x=860, y=225, width=30, height=25)

def SelRzjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    RzjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(40)

RzjogNegBut = Button(CartjogFrame, text="-",  width=3)
RzjogNegBut.bind("<ButtonPress>", SelRzjogNeg)
RzjogNegBut.bind("<ButtonRelease>", StopJog)
RzjogNegBut.place(x=912, y=225, width=30, height=25)

def SelRzjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    RzjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(41)

RzjogPosBut = Button(CartjogFrame, text="+",  width=3)
RzjogPosBut.bind("<ButtonPress>", SelRzjogPos)
RzjogPosBut.bind("<ButtonRelease>", StopJog)
RzjogPosBut.place(x=950, y=225, width=30, height=25)

def SelRyjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    RyjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(50)

RyjogNegBut = Button(CartjogFrame, text="-",  width=3)
RyjogNegBut.bind("<ButtonPress>", SelRyjogNeg)
RyjogNegBut.bind("<ButtonRelease>", StopJog)
RyjogNegBut.place(x=1002, y=225, width=30, height=25)

def SelRyjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    RyjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(51)

RyjogPosBut = Button(CartjogFrame, text="+",  width=3)
RyjogPosBut.bind("<ButtonPress>", SelRyjogPos)
RyjogPosBut.bind("<ButtonRelease>", StopJog)
RyjogPosBut.place(x=1040, y=225, width=30, height=25)

def SelRxjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    RxjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(60)

RxjogNegBut = Button(CartjogFrame, text="-",  width=3)
RxjogNegBut.bind("<ButtonPress>", SelRxjogNeg)
RxjogNegBut.bind("<ButtonRelease>", StopJog)
RxjogNegBut.place(x=1092, y=225, width=30, height=25)

def SelRxjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    RxjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(61)

RxjogPosBut = Button(CartjogFrame, text="+",  width=3)
RxjogPosBut.bind("<ButtonPress>", SelRxjogPos)
RxjogPosBut.bind("<ButtonRelease>", StopJog)
RxjogPosBut.place(x=1130, y=225, width=30, height=25)


def SelTxjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TXjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(10)

TXjogNegBut = Button(CartjogFrame, text="-",  width=3)
TXjogNegBut.bind("<ButtonPress>", SelTxjogNeg)
TXjogNegBut.bind("<ButtonRelease>", StopJog)
TXjogNegBut.place(x=642, y=300, width=30, height=25)

def SelTxjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TXjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(11)

TXjogPosBut = Button(CartjogFrame, text="+",  width=3)
TXjogPosBut.bind("<ButtonPress>", SelTxjogPos)
TXjogPosBut.bind("<ButtonRelease>", StopJog)
TXjogPosBut.place(x=680, y=300, width=30, height=25)

def SelTyjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TYjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(20)

TYjogNegBut = Button(CartjogFrame, text="-",  width=3)
TYjogNegBut.bind("<ButtonPress>", SelTyjogNeg)
TYjogNegBut.bind("<ButtonRelease>", StopJog)
TYjogNegBut.place(x=732, y=300, width=30, height=25)

def SelTyjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TYjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(21)

TYjogPosBut = Button(CartjogFrame, text="+",  width=3)
TYjogPosBut.bind("<ButtonPress>", SelTyjogPos)
TYjogPosBut.bind("<ButtonRelease>", StopJog)
TYjogPosBut.place(x=770, y=300, width=30, height=25)

def SelTzjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TZjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(30)

TZjogNegBut = Button(CartjogFrame, text="-",  width=3)
TZjogNegBut.bind("<ButtonPress>", SelTzjogNeg)
TZjogNegBut.bind("<ButtonRelease>", StopJog)
TZjogNegBut.place(x=822, y=300, width=30, height=25)

def SelTzjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TZjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(31)

TZjogPosBut = Button(CartjogFrame, text="+",  width=3)
TZjogPosBut.bind("<ButtonPress>", SelTzjogPos)
TZjogPosBut.bind("<ButtonRelease>", StopJog)
TZjogPosBut.place(x=860, y=300, width=30, height=25)

def SelTRzjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TRzjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(40)

TRzjogNegBut = Button(CartjogFrame, text="-",  width=3)
TRzjogNegBut.bind("<ButtonPress>", SelTRzjogNeg)
TRzjogNegBut.bind("<ButtonRelease>", StopJog)
TRzjogNegBut.place(x=912, y=300, width=30, height=25)

def SelTRzjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TRzjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(41)

TRzjogPosBut = Button(CartjogFrame, text="+",  width=3)
TRzjogPosBut.bind("<ButtonPress>", SelTRzjogPos)
TRzjogPosBut.bind("<ButtonRelease>", StopJog)
TRzjogPosBut.place(x=950, y=300, width=30, height=25)

def SelTRyjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TRyjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(50)

TRyjogNegBut = Button(CartjogFrame, text="-",  width=3)
TRyjogNegBut.bind("<ButtonPress>", SelTRyjogNeg)
TRyjogNegBut.bind("<ButtonRelease>", StopJog)
TRyjogNegBut.place(x=1002, y=300, width=30, height=25)

def SelTRyjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TRyjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(51)

TRyjogPosBut = Button(CartjogFrame, text="+",  width=3)
TRyjogPosBut.bind("<ButtonPress>", SelTRyjogPos)
TRyjogPosBut.bind("<ButtonRelease>", StopJog)
TRyjogPosBut.place(x=1040, y=300, width=30, height=25)

def SelTRxjogNeg(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TRxjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(60)

TRxjogNegBut = Button(CartjogFrame, text="-",  width=3)
TRxjogNegBut.bind("<ButtonPress>", SelTRxjogNeg)
TRxjogNegBut.bind("<ButtonRelease>", StopJog)
TRxjogNegBut.place(x=1092, y=300, width=30, height=25)

def SelTRxjogPos(self):
  IncJogStatVal = int(IncJogStat.get())
  if (IncJogStatVal == 1):
    TRxjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(61)

TRxjogPosBut = Button(CartjogFrame, text="+",  width=3)
TRxjogPosBut.bind("<ButtonPress>", SelTRxjogPos)
TRxjogPosBut.bind("<ButtonRelease>", StopJog)
TRxjogPosBut.place(x=1130, y=300, width=30, height=25)




####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 2




### 2 LABELS#################################################################
#############################################################################

ComPortLab = Label(tab2, text = "TEENSY COM PORT:")
ComPortLab.place(x=66, y=90)

ComPortLab = Label(tab2, text = "IO BOARD COM PORT:")
ComPortLab.place(x=60, y=160)

almStatusLab2 = Label(tab2, text = "SYSTEM READY - NO ACTIVE ALARMS", style="OK.TLabel")
almStatusLab2.place(x=25, y=20)




comLab = Label(tab2, text = "Communication")
comLab.place(x=72, y=60)

jointCalLab = Label(tab2, text = "Robot Calibration")
jointCalLab.place(x=290, y=60)

axis7Lab = Label(tab2, text = "7th Axis Calibration")
axis7Lab.place(x=665, y=300)

axis7lengthLab = Label(tab2, text = "7th Axis Length:")
axis7lengthLab.place(x=651, y=340)

axis7rotLab = Label(tab2, text = "MM per Rotation:")
axis7rotLab.place(x=645, y=370)

axis7stepsLab = Label(tab2, text = "Drive Steps:")
axis7stepsLab.place(x=675, y=400)

axis7pinsetLab = Label(tab2,font=("Arial", 8), text = "StepPin = 12 / DirPin = 13 / CalPin = 36")
axis7pinsetLab.place(x=627, y=510)

axis8pinsetLab = Label(tab2,font=("Arial", 8), text = "StepPin = 32 / DirPin = 33 / CalPin = 37")
axis8pinsetLab.place(x=827, y=510)

axis9pinsetLab = Label(tab2,font=("Arial", 8), text = "StepPin = 34 / DirPin = 35 / CalPin = 38")
axis9pinsetLab.place(x=1027, y=510)





axis8Lab = Label(tab2, text = "8th Axis Calibration")
axis8Lab.place(x=865, y=300)

axis8lengthLab = Label(tab2, text = "8th Axis Length:")
axis8lengthLab.place(x=851, y=340)

axis8rotLab = Label(tab2, text = "MM per Rotation:")
axis8rotLab.place(x=845, y=370)

axis8stepsLab = Label(tab2, text = "Drive Steps:")
axis8stepsLab.place(x=875, y=400)


axis9Lab = Label(tab2, text = "9th Axis Calibration")
axis9Lab.place(x=1065, y=300)

axis9lengthLab = Label(tab2, text = "9th Axis Length:")
axis9lengthLab.place(x=1051, y=340)

axis9rotLab = Label(tab2, text = "MM per Rotation:")
axis9rotLab.place(x=1045, y=370)

axis9stepsLab = Label(tab2, text = "Drive Steps:")
axis9stepsLab.place(x=1075, y=400)





CalibrationOffsetsLab = Label(tab2, text = "Calibration Offsets")
CalibrationOffsetsLab.place(x=485, y=60)

J1calLab = Label(tab2, text = "J1 Offset")
J1calLab.place(x=480, y=90)

J2calLab = Label(tab2, text = "J2 Offset")
J2calLab.place(x=480, y=120)

J3calLab = Label(tab2, text = "J3 Offset")
J3calLab.place(x=480, y=150)

J4calLab = Label(tab2, text = "J4 Offset")
J4calLab.place(x=480, y=180)

J5calLab = Label(tab2, text = "J5 Offset")
J5calLab.place(x=480, y=210)

J6calLab = Label(tab2, text = "J6 Offset")
J6calLab.place(x=480, y=240)

J7calLab = Label(tab2, text = "J7 Offset")
J7calLab.place(x=480, y=280)

J8calLab = Label(tab2, text = "J8 Offset")
J8calLab.place(x=480, y=310)

J9calLab = Label(tab2, text = "J9 Offset")
J9calLab.place(x=480, y=340)





CalibrationOffsetsLab = Label(tab2, text = "Encoder Control")
CalibrationOffsetsLab.place(x=715, y=60)

cmdSentLab = Label(tab2, text = "Last Command Sent to Controller")
cmdSentLab.place(x=10, y=565)

cmdRecLab = Label(tab2, text = "Last Response From Controller")
cmdRecLab.place(x=10, y=625)

ThemeLab = Label(tab2, text = "Theme")
ThemeLab.place(x=925, y=60)


### 2 BUTTONS################################################################
#############################################################################

comPortBut = Button(tab2,  text="  Set Com Teensy  ",   command = setCom)
comPortBut.place(x=85, y=110)

comPortBut2 = Button(tab2,  text="Set Com IO Board",   command = setCom2)
comPortBut2.place(x=85, y=180)


lightBut = Button(tab2,  text="  Light  ",  command = lightTheme)
lightBut.place(x=890, y=90)

darkBut = Button(tab2,  text="  Dark   ",  command = darkTheme)
darkBut.place(x=950, y=90)


autoCalBut = Button(tab2, text="  Auto Calibrate  ",   command = calRobotAll)
autoCalBut.place(x=285, y=90)

J1calCbut = Checkbutton(tab2, text="J1",variable = J1CalStat)
J1calCbut.place(x=285, y=125)

J2calCbut = Checkbutton(tab2, text="J2",variable = J2CalStat)
J2calCbut.place(x=320, y=125)

J3calCbut = Checkbutton(tab2, text="J3",variable = J3CalStat)
J3calCbut.place(x=355, y=125)

J4calCbut = Checkbutton(tab2, text="J4",variable = J4CalStat)
J4calCbut.place(x=285, y=145)

J5calCbut = Checkbutton(tab2, text="J5",variable = J5CalStat)
J5calCbut.place(x=320, y=145)

J6calCbut = Checkbutton(tab2, text="J6",variable = J6CalStat)
J6calCbut.place(x=355, y=145)


J1calCbut2 = Checkbutton(tab2, text="J1",variable = J1CalStat2)
J1calCbut2.place(x=285, y=180)

J2calCbut2 = Checkbutton(tab2, text="J2",variable = J2CalStat2)
J2calCbut2.place(x=320, y=180)

J3calCbut2 = Checkbutton(tab2, text="J3",variable = J3CalStat2)
J3calCbut2.place(x=355, y=180)

J4calCbut2 = Checkbutton(tab2, text="J4",variable = J4CalStat2)
J4calCbut2.place(x=285, y=200)

J5calCbut2 = Checkbutton(tab2, text="J5",variable = J5CalStat2)
J5calCbut2.place(x=320, y=200)

J6calCbut2 = Checkbutton(tab2, text="J6",variable = J6CalStat2)
J6calCbut2.place(x=355, y=200)





J7zerobut = Button(tab2, text="Set Axis 7 Calibration to Zero",  width=28, command = zeroAxis7)
J7zerobut.place(x=627, y=440)

J8zerobut = Button(tab2, text="Set Axis 8 Calibration to Zero",  width=28, command = zeroAxis8)
J8zerobut.place(x=827, y=440)

J9zerobut = Button(tab2, text="Set Axis 9 Calibration to Zero",  width=28, command = zeroAxis9)
J9zerobut.place(x=1027, y=440)

J7calbut = Button(tab2, text="Autocalibrate Axis 7",  width=28, command = calRobotJ7)
J7calbut.place(x=627, y=475)

J8calbut = Button(tab2, text="Autocalibrate Axis 8",  width=28, command = calRobotJ8)
J8calbut.place(x=827, y=475)

J9calbut = Button(tab2, text="Autocalibrate Axis 9",  width=28, command = calRobotJ9)
J9calbut.place(x=1027, y=475)




CalJ1But = Button(tab2,   text="Calibrate J1 Only",   command = calRobotJ1)
CalJ1But.place(x=285, y=240)

CalJ2But = Button(tab2,   text="Calibrate J2 Only",   command = calRobotJ2)
CalJ2But.place(x=285, y=270)

CalJ3But = Button(tab2,   text="Calibrate J3 Only",   command = calRobotJ3)
CalJ3But.place(x=285, y=300)

CalJ4But = Button(tab2,   text="Calibrate J4 Only",   command = calRobotJ4)
CalJ4But.place(x=285, y=330)

CalJ5But = Button(tab2,   text="Calibrate J5 Only",   command = calRobotJ5)
CalJ5But.place(x=285, y=360)

CalJ6But = Button(tab2,   text="Calibrate J6 Only",   command = calRobotJ6)
CalJ6But.place(x=285, y=390)

CalZeroBut = Button(tab2,   text="Force CaL to Home",  width=20,   command = CalZeroPos)
CalZeroBut.place(x=270, y=425)

CalRestBut = Button(tab2,   text="Force Cal to Rest",  width=20,   command = CalRestPos)
CalRestBut.place(x=270, y=460)

J1OpenLoopCbut = Checkbutton(tab2, text="J1 Open Loop (disable encoder)",variable = J1OpenLoopStat)
J1OpenLoopCbut.place(x=665, y=90)

J2OpenLoopCbut = Checkbutton(tab2, text="J2 Open Loop (disable encoder)",variable = J2OpenLoopStat)
J2OpenLoopCbut.place(x=665, y=110)

J3OpenLoopCbut = Checkbutton(tab2, text="J3 Open Loop (disable encoder)",variable = J3OpenLoopStat)
J3OpenLoopCbut.place(x=665, y=130)

J4OpenLoopCbut = Checkbutton(tab2, text="J4 Open Loop (disable encoder)",variable = J4OpenLoopStat)
J4OpenLoopCbut.place(x=665, y=150)

J5OpenLoopCbut = Checkbutton(tab2, text="J5 Open Loop (disable encoder)",variable = J5OpenLoopStat)
J5OpenLoopCbut.place(x=665, y=170)

J6OpenLoopCbut = Checkbutton(tab2, text="J6 Open Loop (disable encoder)",variable = J6OpenLoopStat)
J6OpenLoopCbut.place(x=665, y=190)

saveCalBut = Button(tab2,  text="    SAVE    ",  width=26, command = SaveAndApplyCalibration)
saveCalBut.place(x=1150, y=630)

#### 2 ENTRY FIELDS##########################################################
#############################################################################


comPortEntryField = Entry(tab2,width=4)
comPortEntryField.place(x=50, y=114)

com2PortEntryField = Entry(tab2,width=4)
com2PortEntryField.place(x=50, y=184)

cmdSentEntryField = Entry(tab2,width=95)
cmdSentEntryField.place(x=10, y=585)

cmdRecEntryField = Entry(tab2,width=95)
cmdRecEntryField.place(x=10, y=645)


J1calOffEntryField = Entry(tab2,width=8)
J1calOffEntryField.place(x=540, y=90)

J2calOffEntryField = Entry(tab2,width=8)
J2calOffEntryField.place(x=540, y=120)

J3calOffEntryField = Entry(tab2,width=8)
J3calOffEntryField.place(x=540, y=150)

J4calOffEntryField = Entry(tab2,width=8)
J4calOffEntryField.place(x=540, y=180)

J5calOffEntryField = Entry(tab2,width=8)
J5calOffEntryField.place(x=540, y=210)

J6calOffEntryField = Entry(tab2,width=8)
J6calOffEntryField.place(x=540, y=240)

J7calOffEntryField = Entry(tab2,width=8)
J7calOffEntryField.place(x=540, y=280)

J8calOffEntryField = Entry(tab2,width=8)
J8calOffEntryField.place(x=540, y=310)

J9calOffEntryField = Entry(tab2,width=8)
J9calOffEntryField.place(x=540, y=340)



axis7lengthEntryField = Entry(tab2,width=6)
axis7lengthEntryField.place(x=750, y=340)

axis7rotEntryField = Entry(tab2,width=6)
axis7rotEntryField.place(x=750, y=370)

axis7stepsEntryField = Entry(tab2,width=6)
axis7stepsEntryField.place(x=750, y=400)

axis8lengthEntryField = Entry(tab2,width=6)
axis8lengthEntryField.place(x=950, y=340)

axis8rotEntryField = Entry(tab2,width=6)
axis8rotEntryField.place(x=950, y=370)

axis8stepsEntryField = Entry(tab2,width=6)
axis8stepsEntryField.place(x=950, y=400)

axis9lengthEntryField = Entry(tab2,width=6)
axis9lengthEntryField.place(x=1150, y=340)

axis9rotEntryField = Entry(tab2,width=6)
axis9rotEntryField.place(x=1150, y=370)

axis9stepsEntryField = Entry(tab2,width=6)
axis9stepsEntryField.place(x=1150, y=400)




####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 3

#### TOOL FRAME ####
ToolFrameLab = Label(tab3, text = "Tool Frame Offset")
ToolFrameLab.place(x=970, y=60)

UFxLab = Label(tab3, font=("Arial", 11), text = "X")
UFxLab.place(x=920, y=90)

UFyLab = Label(tab3, font=("Arial", 11), text = "Y")
UFyLab.place(x=960, y=90)

UFzLab = Label(tab3, font=("Arial", 11), text = "Z")
UFzLab.place(x=1000, y=90)

UFRxLab = Label(tab3, font=("Arial", 11), text = "Rz")
UFRxLab.place(x=1040, y=90)

UFRyLab = Label(tab3, font=("Arial", 11), text = "Ry")
UFRyLab.place(x=1080, y=90)

UFRzLab = Label(tab3, font=("Arial", 11), text = "Rx")
UFRzLab.place(x=1120, y=90)

TFxEntryField = Entry(tab3,width=5)
TFxEntryField.place(x=910, y=115)
TFyEntryField = Entry(tab3,width=5)
TFyEntryField.place(x=950, y=115)
TFzEntryField = Entry(tab3,width=5)
TFzEntryField.place(x=990, y=115)
TFrzEntryField = Entry(tab3,width=5)
TFrzEntryField.place(x=1030, y=115)
TFryEntryField = Entry(tab3,width=5)
TFryEntryField.place(x=1070, y=115)
TFrxEntryField = Entry(tab3,width=5)
TFrxEntryField.place(x=1110, y=115)

DisableWristCbut = Checkbutton(tab3, text="Disable Wrist Rotation - Linear Moves",variable = DisableWristRot)
DisableWristCbut.place(x=910, y=150)


####  MOTOR DIRECTIONS ####

J1MotDirLab = Label(tab3, font=("Arial", 8), text = "J1 Motor Direction")
J1MotDirLab.place(x=10, y=20)
J2MotDirLab = Label(tab3, font=("Arial", 8), text = "J2 Motor Direction")
J2MotDirLab.place(x=10, y=45)
J3MotDirLab = Label(tab3, font=("Arial", 8), text = "J3 Motor Direction")
J3MotDirLab.place(x=10, y=70)
J4MotDirLab = Label(tab3, font=("Arial", 8), text = "J4 Motor Direction")
J4MotDirLab.place(x=10, y=95)
J5MotDirLab = Label(tab3, font=("Arial", 8), text = "J5 Motor Direction")
J5MotDirLab.place(x=10, y=120)
J6MotDirLab = Label(tab3, font=("Arial", 8), text = "J6 Motor Direction")
J6MotDirLab.place(x=10, y=145)
J7MotDirLab = Label(tab3, font=("Arial", 8), text = "J7 Motor Direction")
J7MotDirLab.place(x=10, y=170)
J8MotDirLab = Label(tab3, font=("Arial", 8), text = "J8 Motor Direction")
J8MotDirLab.place(x=10, y=195)
J9MotDirLab = Label(tab3, font=("Arial", 8), text = "J9 Motor Direction")
J9MotDirLab.place(x=10, y=220)

J1MotDirEntryField = Entry(tab3,width=8)
J1MotDirEntryField.place(x=110, y=20)
J2MotDirEntryField = Entry(tab3,width=8)
J2MotDirEntryField.place(x=110, y=45)
J3MotDirEntryField = Entry(tab3,width=8)
J3MotDirEntryField.place(x=110, y=70)
J4MotDirEntryField = Entry(tab3,width=8)
J4MotDirEntryField.place(x=110, y=95)
J5MotDirEntryField = Entry(tab3,width=8)
J5MotDirEntryField.place(x=110, y=120)
J6MotDirEntryField = Entry(tab3,width=8)
J6MotDirEntryField.place(x=110, y=145)
J7MotDirEntryField = Entry(tab3,width=8)
J7MotDirEntryField.place(x=110, y=170)
J8MotDirEntryField = Entry(tab3,width=8)
J8MotDirEntryField.place(x=110, y=195)
J9MotDirEntryField = Entry(tab3,width=8)
J9MotDirEntryField.place(x=110, y=220)


####  CALIBRATION DIRECTIONS ####

J1CalDirLab = Label(tab3, font=("Arial", 8), text = "J1 Calibration Dir.")
J1CalDirLab.place(x=10, y=280)
J2CalDirLab = Label(tab3, font=("Arial", 8), text = "J2 Calibration Dir.")
J2CalDirLab.place(x=10, y=305)
J3CalDirLab = Label(tab3, font=("Arial", 8), text = "J3 Calibration Dir.")
J3CalDirLab.place(x=10, y=330)
J4CalDirLab = Label(tab3, font=("Arial", 8), text = "J4 Calibration Dir.")
J4CalDirLab.place(x=10, y=355)
J5CalDirLab = Label(tab3, font=("Arial", 8), text = "J5 Calibration Dir.")
J5CalDirLab.place(x=10, y=380)
J6CalDirLab = Label(tab3, font=("Arial", 8), text = "J6 Calibration Dir.")
J6CalDirLab.place(x=10, y=405)
J7CalDirLab = Label(tab3, font=("Arial", 8), text = "J7 Calibration Dir.")
J7CalDirLab.place(x=10, y=430)
J8CalDirLab = Label(tab3, font=("Arial", 8), text = "J8 Calibration Dir.")
J8CalDirLab.place(x=10, y=455)
J9CalDirLab = Label(tab3, font=("Arial", 8), text = "J9 Calibration Dir.")
J9CalDirLab.place(x=10, y=480)

J1CalDirEntryField = Entry(tab3,width=8)
J1CalDirEntryField.place(x=110, y=280)
J2CalDirEntryField = Entry(tab3,width=8)
J2CalDirEntryField.place(x=110, y=305)
J3CalDirEntryField = Entry(tab3,width=8)
J3CalDirEntryField.place(x=110, y=330)
J4CalDirEntryField = Entry(tab3,width=8)
J4CalDirEntryField.place(x=110, y=355)
J5CalDirEntryField = Entry(tab3,width=8)
J5CalDirEntryField.place(x=110, y=380)
J6CalDirEntryField = Entry(tab3,width=8)
J6CalDirEntryField.place(x=110, y=405)
J7CalDirEntryField = Entry(tab3,width=8)
J7CalDirEntryField.place(x=110, y=430)
J8CalDirEntryField = Entry(tab3,width=8)
J8CalDirEntryField.place(x=110, y=455)
J9CalDirEntryField = Entry(tab3,width=8)
J9CalDirEntryField.place(x=110, y=480)

### axis limits
J1PosLimLab = Label(tab3, font=("Arial", 8), text = "J1 Pos Limit")
J1PosLimLab.place(x=200, y=20)
J1NegLimLab = Label(tab3, font=("Arial", 8), text = "J1 Neg Limit")
J1NegLimLab.place(x=200, y=45)
J2PosLimLab = Label(tab3, font=("Arial", 8), text = "J2 Pos Limit")
J2PosLimLab.place(x=200, y=70)
J2NegLimLab = Label(tab3, font=("Arial", 8), text = "J2 Neg Limit")
J2NegLimLab.place(x=200, y=95)
J3PosLimLab = Label(tab3, font=("Arial", 8), text = "J3 Pos Limit")
J3PosLimLab.place(x=200, y=120)
J3NegLimLab = Label(tab3, font=("Arial", 8), text = "J3 Neg Limit")
J3NegLimLab.place(x=200, y=145)
J4PosLimLab = Label(tab3, font=("Arial", 8), text = "J4 Pos Limit")
J4PosLimLab.place(x=200, y=170)
J4NegLimLab = Label(tab3, font=("Arial", 8), text = "J4 Neg Limit")
J4NegLimLab.place(x=200, y=195)
J5PosLimLab = Label(tab3, font=("Arial", 8), text = "J5 Pos Limit")
J5PosLimLab.place(x=200, y=220)
J5NegLimLab = Label(tab3, font=("Arial", 8), text = "J5 Neg Limit")
J5NegLimLab.place(x=200, y=245)
J6PosLimLab = Label(tab3, font=("Arial", 8), text = "J6 Pos Limit")
J6PosLimLab.place(x=200, y=270)
J6NegLimLab = Label(tab3, font=("Arial", 8), text = "J6 Neg Limit")
J6NegLimLab.place(x=200, y=295)

J1PosLimEntryField = Entry(tab3,width=8)
J1PosLimEntryField.place(x=280, y=20)
J1NegLimEntryField = Entry(tab3,width=8)
J1NegLimEntryField.place(x=280, y=45)
J2PosLimEntryField = Entry(tab3,width=8)
J2PosLimEntryField.place(x=280, y=70)
J2NegLimEntryField = Entry(tab3,width=8)
J2NegLimEntryField.place(x=280, y=95)
J3PosLimEntryField = Entry(tab3,width=8)
J3PosLimEntryField.place(x=280, y=120)
J3NegLimEntryField = Entry(tab3,width=8)
J3NegLimEntryField.place(x=280, y=145)
J4PosLimEntryField = Entry(tab3,width=8)
J4PosLimEntryField.place(x=280, y=170)
J4NegLimEntryField = Entry(tab3,width=8)
J4NegLimEntryField.place(x=280, y=195)
J5PosLimEntryField = Entry(tab3,width=8)
J5PosLimEntryField.place(x=280, y=220)
J5NegLimEntryField = Entry(tab3,width=8)
J5NegLimEntryField.place(x=280, y=245)
J6PosLimEntryField = Entry(tab3,width=8)
J6PosLimEntryField.place(x=280, y=270)
J6NegLimEntryField = Entry(tab3,width=8)
J6NegLimEntryField.place(x=280, y=295)


### steps per degress
J1StepDegLab = Label(tab3, font=("Arial", 8), text = "J1 Step/Deg")
J1StepDegLab.place(x=200, y=345)
J2StepDegLab = Label(tab3, font=("Arial", 8), text = "J2 Step/Deg")
J2StepDegLab.place(x=200, y=370)
J3StepDegLab = Label(tab3, font=("Arial", 8), text = "J3 Step/Deg")
J3StepDegLab.place(x=200, y=395)
J4StepDegLab = Label(tab3, font=("Arial", 8), text = "J4 Step/Deg")
J4StepDegLab.place(x=200, y=420)
J5StepDegLab = Label(tab3, font=("Arial", 8), text = "J5 Step/Deg")
J5StepDegLab.place(x=200, y=445)
J6StepDegLab = Label(tab3, font=("Arial", 8), text = "J6 Step/Deg")
J6StepDegLab.place(x=200, y=470)

J1StepDegEntryField = Entry(tab3,width=8)
J1StepDegEntryField.place(x=280, y=345)
J2StepDegEntryField = Entry(tab3,width=8)
J2StepDegEntryField.place(x=280, y=370)
J3StepDegEntryField = Entry(tab3,width=8)
J3StepDegEntryField.place(x=280, y=395)
J4StepDegEntryField = Entry(tab3,width=8)
J4StepDegEntryField.place(x=280, y=420)
J5StepDegEntryField = Entry(tab3,width=8)
J5StepDegEntryField.place(x=280, y=445)
J6StepDegEntryField = Entry(tab3,width=8)
J6StepDegEntryField.place(x=280, y=470)


### DRIVER STEPS
J1DriveMSLab = Label(tab3, font=("Arial", 8), text = "J1 Drive Microstep")
J1DriveMSLab.place(x=390, y=20)
J2DriveMSLab = Label(tab3, font=("Arial", 8), text = "J2 Drive Microstep")
J2DriveMSLab.place(x=390, y=45)
J3DriveMSLab = Label(tab3, font=("Arial", 8), text = "J3 Drive Microstep")
J3DriveMSLab.place(x=390, y=70)
J4DriveMSLab = Label(tab3, font=("Arial", 8), text = "J4 Drive Microstep")
J4DriveMSLab.place(x=390, y=95)
J5DriveMSLab = Label(tab3, font=("Arial", 8), text = "J5 Drive Microstep")
J5DriveMSLab.place(x=390, y=120)
J6DriveMSLab = Label(tab3, font=("Arial", 8), text = "J6 Drive Microstep")
J6DriveMSLab.place(x=390, y=145)

J1DriveMSEntryField = Entry(tab3,width=8)
J1DriveMSEntryField.place(x=500, y=20)
J2DriveMSEntryField = Entry(tab3,width=8)
J2DriveMSEntryField.place(x=500, y=45)
J3DriveMSEntryField = Entry(tab3,width=8)
J3DriveMSEntryField.place(x=500, y=70)
J4DriveMSEntryField = Entry(tab3,width=8)
J4DriveMSEntryField.place(x=500, y=95)
J5DriveMSEntryField = Entry(tab3,width=8)
J5DriveMSEntryField.place(x=500, y=120)
J6DriveMSEntryField = Entry(tab3,width=8)
J6DriveMSEntryField.place(x=500, y=145)


###ENCODER CPR
J1EncCPRLab = Label(tab3, font=("Arial", 8), text = "J1 Encoder CPR")
J1EncCPRLab.place(x=390, y=195)
J2EncCPRLab = Label(tab3, font=("Arial", 8), text = "J2 Encoder CPR")
J2EncCPRLab.place(x=390, y=220)
J3EncCPRLab = Label(tab3, font=("Arial", 8), text = "J3 Encoder CPR")
J3EncCPRLab.place(x=390, y=245)
J4EncCPRLab = Label(tab3, font=("Arial", 8), text = "J4 Encoder CPR")
J4EncCPRLab.place(x=390, y=270)
J5EncCPRLab = Label(tab3, font=("Arial", 8), text = "J5 Encoder CPR")
J5EncCPRLab.place(x=390, y=295)
J6EncCPRLab = Label(tab3, font=("Arial", 8), text = "J6 Encoder CPR")
J6EncCPRLab.place(x=390, y=320)

J1EncCPREntryField = Entry(tab3,width=8)
J1EncCPREntryField.place(x=500, y=195)
J2EncCPREntryField = Entry(tab3,width=8)
J2EncCPREntryField.place(x=500, y=220)
J3EncCPREntryField = Entry(tab3,width=8)
J3EncCPREntryField.place(x=500, y=245)
J4EncCPREntryField = Entry(tab3,width=8)
J4EncCPREntryField.place(x=500, y=270)
J5EncCPREntryField = Entry(tab3,width=8)
J5EncCPREntryField.place(x=500, y=295)
J6EncCPREntryField = Entry(tab3,width=8)
J6EncCPREntryField.place(x=500, y=320)


### DH PARAMS

### DRIVER STEPS
J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J1")
J1DHparamLab.place(x=600, y=45)
J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J2")
J1DHparamLab.place(x=600, y=70)
J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J3")
J1DHparamLab.place(x=600, y=95)
J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J4")
J1DHparamLab.place(x=600, y=120)
J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J5")
J1DHparamLab.place(x=600, y=145)
J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J6")
J1DHparamLab.place(x=600, y=170)

ΘDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-Θ")
ΘDHparamLab.place(x=645, y=20)
αDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-α")
αDHparamLab.place(x=700, y=20)
dDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-d")
dDHparamLab.place(x=755, y=20)
aDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-a")
aDHparamLab.place(x=810, y=20)


J1ΘEntryField = Entry(tab3,width=8)
J1ΘEntryField.place(x=630, y=45)
J2ΘEntryField = Entry(tab3,width=8)
J2ΘEntryField.place(x=630, y=70)
J3ΘEntryField = Entry(tab3,width=8)
J3ΘEntryField.place(x=630, y=95)
J4ΘEntryField = Entry(tab3,width=8)
J4ΘEntryField.place(x=630, y=120)
J5ΘEntryField = Entry(tab3,width=8)
J5ΘEntryField.place(x=630, y=145)
J6ΘEntryField = Entry(tab3,width=8)
J6ΘEntryField.place(x=630, y=170)

J1αEntryField = Entry(tab3,width=8)
J1αEntryField.place(x=685, y=45)
J2αEntryField = Entry(tab3,width=8)
J2αEntryField.place(x=685, y=70)
J3αEntryField = Entry(tab3,width=8)
J3αEntryField.place(x=685, y=95)
J4αEntryField = Entry(tab3,width=8)
J4αEntryField.place(x=685, y=120)
J5αEntryField = Entry(tab3,width=8)
J5αEntryField.place(x=685, y=145)
J6αEntryField = Entry(tab3,width=8)
J6αEntryField.place(x=685, y=170)

J1dEntryField = Entry(tab3,width=8)
J1dEntryField.place(x=740, y=45)
J2dEntryField = Entry(tab3,width=8)
J2dEntryField.place(x=740, y=70)
J3dEntryField = Entry(tab3,width=8)
J3dEntryField.place(x=740, y=95)
J4dEntryField = Entry(tab3,width=8)
J4dEntryField.place(x=740, y=120)
J5dEntryField = Entry(tab3,width=8)
J5dEntryField.place(x=740, y=145)
J6dEntryField = Entry(tab3,width=8)
J6dEntryField.place(x=740, y=170)

J1aEntryField = Entry(tab3,width=8)
J1aEntryField.place(x=795, y=45)
J2aEntryField = Entry(tab3,width=8)
J2aEntryField.place(x=795, y=70)
J3aEntryField = Entry(tab3,width=8)
J3aEntryField.place(x=795, y=95)
J4aEntryField = Entry(tab3,width=8)
J4aEntryField.place(x=795, y=120)
J5aEntryField = Entry(tab3,width=8)
J5aEntryField.place(x=795, y=145)
J6aEntryField = Entry(tab3,width=8)
J6aEntryField.place(x=795, y=170)


### LOAD DEFAULT ###

loadAR3But = Button(tab3,  text="Load AR3 Defaults",  width=26, command = LoadAR3default)
loadAR3But.place(x=1150, y=510)

loadAR4But = Button(tab3,  text="Load AR4 Defaults",  width=26, command = LoadAR4default)
loadAR4But.place(x=1150, y=550)

loadAR4Mk2But = Button(tab3,  text="Load AR4-MK2 Defaults",  width=26, command = LoadAR4Mk2default)
loadAR4Mk2But.place(x=1150, y=590)


#### SAVE ####

saveCalBut = Button(tab3,  text="SAVE",  width=26, command = SaveAndApplyCalibration)
saveCalBut.place(x=1150, y=630)



####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 4



### 4 LABELS#################################################################
#############################################################################

servo0onequalsLab = Label(tab4, text = "=")
servo0onequalsLab.place(x=70, y=12)

servo0offequalsLab = Label(tab4, text = "=")
servo0offequalsLab.place(x=70, y=52)

servo1onequalsLab = Label(tab4, text = "=")
servo1onequalsLab.place(x=70, y=92)

servo1offequalsLab = Label(tab4, text = "=")
servo1offequalsLab.place(x=70, y=132)

servo2onequalsLab = Label(tab4, text = "=")
servo2onequalsLab.place(x=70, y=172)

servo2offequalsLab = Label(tab4, text = "=")
servo2offequalsLab.place(x=70, y=212)

servo3onequalsLab = Label(tab4, text = "=")
servo3onequalsLab.place(x=70, y=252)

servo3offequalsLab = Label(tab4, text = "=")
servo3offequalsLab.place(x=70, y=292)



Do1onequalsLab = Label(tab4, text = "=")
Do1onequalsLab.place(x=210, y=12)

Do1offequalsLab = Label(tab4, text = "=")
Do1offequalsLab.place(x=210, y=52)

Do2onequalsLab = Label(tab4, text = "=")
Do2onequalsLab.place(x=210, y=92)

Do2offequalsLab = Label(tab4, text = "=")
Do2offequalsLab.place(x=210, y=132)

Do3onequalsLab = Label(tab4, text = "=")
Do3onequalsLab.place(x=210, y=172)

Do3offequalsLab = Label(tab4, text = "=")
Do3offequalsLab.place(x=210, y=212)

Do4onequalsLab = Label(tab4, text = "=")
Do4onequalsLab.place(x=210, y=252)

Do4offequalsLab = Label(tab4, text = "=")
Do4offequalsLab.place(x=210, y=292)

Do5onequalsLab = Label(tab4, text = "=")
Do5onequalsLab.place(x=210, y=332)

Do5offequalsLab = Label(tab4, text = "=")
Do5offequalsLab.place(x=210, y=372)

Do6onequalsLab = Label(tab4, text = "=")
Do6onequalsLab.place(x=210, y=412)

Do6offequalsLab = Label(tab4, text = "=")
Do6offequalsLab.place(x=210, y=452)


inoutavailLab = Label(tab4, text = "NOTE: the following are available when using the default Nano board for IO:   Inputs = 2-7  /  Outputs = 8-13  /  Servos = A0-A7")
inoutavailLab.place(x=10, y=640)

inoutavailLab = Label(tab4, text = "If using IO on Teensy board:  Inputs = 32-36  /  Outputs = 37-41 - if using IO on Teensy you must manually change the command from 'Out On =' to 'ToutOn ='")
inoutavailLab.place(x=10, y=655)


### 4 BUTTONS################################################################
#############################################################################

servo0onBut = Button(tab4,  text="Servo 0",  command = Servo0on)
servo0onBut.place(x=10, y=10)

servo0offBut = Button(tab4,  text="Servo 0",  command = Servo0off)
servo0offBut.place(x=10, y=50)

servo1onBut = Button(tab4,  text="Servo 1",  command = Servo1on)
servo1onBut.place(x=10, y=90)

servo1offBut = Button(tab4,  text="Servo 1",  command = Servo1off)
servo1offBut.place(x=10, y=130)

servo2onBut = Button(tab4,  text="Servo 2",  command = Servo2on)
servo2onBut.place(x=10, y=170)

servo2offBut = Button(tab4,  text="Servo 2",  command = Servo2off)
servo2offBut.place(x=10, y=210)

servo3onBut = Button(tab4,  text="Servo 3",  command = Servo3on)
servo3onBut.place(x=10, y=250)

servo3offBut = Button(tab4,  text="Servo 3",  command = Servo3off)
servo3offBut.place(x=10, y=290)





DO1onBut = Button(tab4,  text="DO on",  command = DO1on)
DO1onBut.place(x=150, y=10)

DO1offBut = Button(tab4,  text="DO off",  command = DO1off)
DO1offBut.place(x=150, y=50)

DO2onBut = Button(tab4,  text="DO on",  command = DO2on)
DO2onBut.place(x=150, y=90)

DO2offBut = Button(tab4,  text="DO off",  command = DO2off)
DO2offBut.place(x=150, y=130)

DO3onBut = Button(tab4,  text="DO on",  command = DO3on)
DO3onBut.place(x=150, y=170)

DO3offBut = Button(tab4,  text="DO off",  command = DO3off)
DO3offBut.place(x=150, y=210)

DO4onBut = Button(tab4,  text="DO on",  command = DO4on)
DO4onBut.place(x=150, y=250)

DO4offBut = Button(tab4,  text="DO off",  command = DO4off)
DO4offBut.place(x=150, y=290)

DO5onBut = Button(tab4,  text="DO on",  command = DO5on)
DO5onBut.place(x=150, y=330)

DO5offBut = Button(tab4,  text="DO off",  command = DO5off)
DO5offBut.place(x=150, y=370)

DO6onBut = Button(tab4,  text="DO on",  command = DO6on)
DO6onBut.place(x=150, y=410)

DO6offBut = Button(tab4,  text="DO off",  command = DO6off)
DO6offBut.place(x=150, y=450)



#### 4 ENTRY FIELDS##########################################################
#############################################################################


servo0onEntryField = Entry(tab4,width=5)
servo0onEntryField.place(x=90, y=15)

servo0offEntryField = Entry(tab4,width=5)
servo0offEntryField.place(x=90, y=55)

servo1onEntryField = Entry(tab4,width=5)
servo1onEntryField.place(x=90, y=95)

servo1offEntryField = Entry(tab4,width=5)
servo1offEntryField.place(x=90, y=135)

servo2onEntryField = Entry(tab4,width=5)
servo2onEntryField.place(x=90, y=175)

servo2offEntryField = Entry(tab4,width=5)
servo2offEntryField.place(x=90, y=215)


servo3onEntryField = Entry(tab4,width=5)
servo3onEntryField.place(x=90, y=255)

servo3offEntryField = Entry(tab4,width=5)
servo3offEntryField.place(x=90, y=295)





DO1onEntryField = Entry(tab4,width=5)
DO1onEntryField.place(x=230, y=15)

DO1offEntryField = Entry(tab4,width=5)
DO1offEntryField.place(x=230, y=55)

DO2onEntryField = Entry(tab4,width=5)
DO2onEntryField.place(x=230, y=95)

DO2offEntryField = Entry(tab4,width=5)
DO2offEntryField.place(x=230, y=135)

DO3onEntryField = Entry(tab4,width=5)
DO3onEntryField.place(x=230, y=175)

DO3offEntryField = Entry(tab4,width=5)
DO3offEntryField.place(x=230, y=215)

DO4onEntryField = Entry(tab4,width=5)
DO4onEntryField.place(x=230, y=255)

DO4offEntryField = Entry(tab4,width=5)
DO4offEntryField.place(x=230, y=295)

DO5onEntryField = Entry(tab4,width=5)
DO5onEntryField.place(x=230, y=335)

DO5offEntryField = Entry(tab4,width=5)
DO5offEntryField.place(x=230, y=375)

DO6onEntryField = Entry(tab4,width=5)
DO6onEntryField.place(x=230, y=415)

DO6offEntryField = Entry(tab4,width=5)
DO6offEntryField.place(x=230, y=455)



####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 5




### 5 LABELS#################################################################
#############################################################################

R1Lab = Label(tab5, text = "R1")
R1Lab.place(x=70, y=30)

R2Lab = Label(tab5, text = "R2")
R2Lab.place(x=70, y=60)

R3Lab = Label(tab5, text = "R3")
R3Lab.place(x=70, y=90)

R4Lab = Label(tab5, text = "R4")
R4Lab.place(x=70, y=120)

R5Lab = Label(tab5, text = "R5")
R5Lab.place(x=70, y=150)

R6Lab = Label(tab5, text = "R6")
R6Lab.place(x=70, y=180)

R7Lab = Label(tab5, text = "R7")
R7Lab.place(x=70, y=210)

R8Lab = Label(tab5, text = "R8")
R8Lab.place(x=70, y=240)

R9Lab = Label(tab5, text = "R9")
R9Lab.place(x=70, y=270)

R10Lab = Label(tab5, text = "R10")
R10Lab.place(x=70, y=300)

R11Lab = Label(tab5, text = "R11")
R11Lab.place(x=70, y=330)

R12Lab = Label(tab5, text = "R12")
R12Lab.place(x=70, y=360)

R13Lab = Label(tab5, text = "R14")
R13Lab.place(x=70, y=390)

R14Lab = Label(tab5, text = "R14")
R14Lab.place(x=70, y=420)

R15Lab = Label(tab5, text = "R15")
R15Lab.place(x=70, y=450)

R16Lab = Label(tab5, text = "R16")
R16Lab.place(x=70, y=480)


SP1Lab = Label(tab5, text = "PR1")
SP1Lab.place(x=640, y=30)

SP2Lab = Label(tab5, text = "PR2")
SP2Lab.place(x=640, y=60)

SP3Lab = Label(tab5, text = "PR3")
SP3Lab.place(x=640, y=90)

SP4Lab = Label(tab5, text = "PR4")
SP4Lab.place(x=640, y=120)

SP5Lab = Label(tab5, text = "PR5")
SP5Lab.place(x=640, y=150)

SP6Lab = Label(tab5, text = "PR6")
SP6Lab.place(x=640, y=180)

SP7Lab = Label(tab5, text = "PR7")
SP7Lab.place(x=640, y=210)

SP8Lab = Label(tab5, text = "PR8")
SP8Lab.place(x=640, y=240)

SP9Lab = Label(tab5, text = "PR9")
SP9Lab.place(x=640, y=270)

SP10Lab = Label(tab5, text = "PR10")
SP10Lab.place(x=640, y=300)

SP11Lab = Label(tab5, text = "PR11")
SP11Lab.place(x=640, y=330)

SP12Lab = Label(tab5, text = "PR12")
SP12Lab.place(x=640, y=360)

SP13Lab = Label(tab5, text = "PR14")
SP13Lab.place(x=640, y=390)

SP14Lab = Label(tab5, text = "PR14")
SP14Lab.place(x=640, y=420)

SP15Lab = Label(tab5, text = "PR15")
SP15Lab.place(x=640, y=450)

SP16Lab = Label(tab5, text = "PR16")
SP16Lab.place(x=640, y=480)


SP_E1_Lab = Label(tab5, text = "X")
SP_E1_Lab.place(x=410, y=10)

SP_E2_Lab = Label(tab5, text = "Y")
SP_E2_Lab.place(x=450, y=10)

SP_E3_Lab = Label(tab5, text = "Z")
SP_E3_Lab.place(x=490, y=10)

SP_E4_Lab = Label(tab5, text = "Rz")
SP_E4_Lab.place(x=530, y=10)

SP_E5_Lab = Label(tab5, text = "Ry")
SP_E5_Lab.place(x=570, y=10)

SP_E6_Lab = Label(tab5, text = "Rx")
SP_E6_Lab.place(x=610, y=10)



### 5 BUTTONS################################################################
#############################################################################




#### 5 ENTRY FIELDS##########################################################
#############################################################################

R1EntryField = Entry(tab5,width=5)
R1EntryField.place(x=30, y=30)

R2EntryField = Entry(tab5,width=5)
R2EntryField.place(x=30, y=60)

R3EntryField = Entry(tab5,width=5)
R3EntryField.place(x=30, y=90)

R4EntryField = Entry(tab5,width=5)
R4EntryField.place(x=30, y=120)

R5EntryField = Entry(tab5,width=5)
R5EntryField.place(x=30, y=150)

R6EntryField = Entry(tab5,width=5)
R6EntryField.place(x=30, y=180)

R7EntryField = Entry(tab5,width=5)
R7EntryField.place(x=30, y=210)

R8EntryField = Entry(tab5,width=5)
R8EntryField.place(x=30, y=240)

R9EntryField = Entry(tab5,width=5)
R9EntryField.place(x=30, y=270)

R10EntryField = Entry(tab5,width=5)
R10EntryField.place(x=30, y=300)

R11EntryField = Entry(tab5,width=5)
R11EntryField.place(x=30, y=330)

R12EntryField = Entry(tab5,width=5)
R12EntryField.place(x=30, y=360)

R13EntryField = Entry(tab5,width=5)
R13EntryField.place(x=30, y=390)

R14EntryField = Entry(tab5,width=5)
R14EntryField.place(x=30, y=420)

R15EntryField = Entry(tab5,width=5)
R15EntryField.place(x=30, y=450)

R16EntryField = Entry(tab5,width=5)
R16EntryField.place(x=30, y=480)




SP_1_E1_EntryField = Entry(tab5,width=5)
SP_1_E1_EntryField.place(x=400, y=30)

SP_2_E1_EntryField = Entry(tab5,width=5)
SP_2_E1_EntryField.place(x=400, y=60)

SP_3_E1_EntryField = Entry(tab5,width=5)
SP_3_E1_EntryField.place(x=400, y=90)

SP_4_E1_EntryField = Entry(tab5,width=5)
SP_4_E1_EntryField.place(x=400, y=120)

SP_5_E1_EntryField = Entry(tab5,width=5)
SP_5_E1_EntryField.place(x=400, y=150)

SP_6_E1_EntryField = Entry(tab5,width=5)
SP_6_E1_EntryField.place(x=400, y=180)

SP_7_E1_EntryField = Entry(tab5,width=5)
SP_7_E1_EntryField.place(x=400, y=210)

SP_8_E1_EntryField = Entry(tab5,width=5)
SP_8_E1_EntryField.place(x=400, y=240)

SP_9_E1_EntryField = Entry(tab5,width=5)
SP_9_E1_EntryField.place(x=400, y=270)

SP_10_E1_EntryField = Entry(tab5,width=5)
SP_10_E1_EntryField.place(x=400, y=300)

SP_11_E1_EntryField = Entry(tab5,width=5)
SP_11_E1_EntryField.place(x=400, y=330)

SP_12_E1_EntryField = Entry(tab5,width=5)
SP_12_E1_EntryField.place(x=400, y=360)

SP_13_E1_EntryField = Entry(tab5,width=5)
SP_13_E1_EntryField.place(x=400, y=390)

SP_14_E1_EntryField = Entry(tab5,width=5)
SP_14_E1_EntryField.place(x=400, y=420)

SP_15_E1_EntryField = Entry(tab5,width=5)
SP_15_E1_EntryField.place(x=400, y=450)

SP_16_E1_EntryField = Entry(tab5,width=5)
SP_16_E1_EntryField.place(x=400, y=480)





SP_1_E2_EntryField = Entry(tab5,width=5)
SP_1_E2_EntryField.place(x=440, y=30)

SP_2_E2_EntryField = Entry(tab5,width=5)
SP_2_E2_EntryField.place(x=440, y=60)

SP_3_E2_EntryField = Entry(tab5,width=5)
SP_3_E2_EntryField.place(x=440, y=90)

SP_4_E2_EntryField = Entry(tab5,width=5)
SP_4_E2_EntryField.place(x=440, y=120)

SP_5_E2_EntryField = Entry(tab5,width=5)
SP_5_E2_EntryField.place(x=440, y=150)

SP_6_E2_EntryField = Entry(tab5,width=5)
SP_6_E2_EntryField.place(x=440, y=180)

SP_7_E2_EntryField = Entry(tab5,width=5)
SP_7_E2_EntryField.place(x=440, y=210)

SP_8_E2_EntryField = Entry(tab5,width=5)
SP_8_E2_EntryField.place(x=440, y=240)

SP_9_E2_EntryField = Entry(tab5,width=5)
SP_9_E2_EntryField.place(x=440, y=270)

SP_10_E2_EntryField = Entry(tab5,width=5)
SP_10_E2_EntryField.place(x=440, y=300)

SP_11_E2_EntryField = Entry(tab5,width=5)
SP_11_E2_EntryField.place(x=440, y=330)

SP_12_E2_EntryField = Entry(tab5,width=5)
SP_12_E2_EntryField.place(x=440, y=360)

SP_13_E2_EntryField = Entry(tab5,width=5)
SP_13_E2_EntryField.place(x=440, y=390)

SP_14_E2_EntryField = Entry(tab5,width=5)
SP_14_E2_EntryField.place(x=440, y=420)

SP_15_E2_EntryField = Entry(tab5,width=5)
SP_15_E2_EntryField.place(x=440, y=450)

SP_16_E2_EntryField = Entry(tab5,width=5)
SP_16_E2_EntryField.place(x=440, y=480)




SP_1_E3_EntryField = Entry(tab5,width=5)
SP_1_E3_EntryField.place(x=480, y=30)

SP_2_E3_EntryField = Entry(tab5,width=5)
SP_2_E3_EntryField.place(x=480, y=60)

SP_3_E3_EntryField = Entry(tab5,width=5)
SP_3_E3_EntryField.place(x=480, y=90)

SP_4_E3_EntryField = Entry(tab5,width=5)
SP_4_E3_EntryField.place(x=480, y=120)

SP_5_E3_EntryField = Entry(tab5,width=5)
SP_5_E3_EntryField.place(x=480, y=150)

SP_6_E3_EntryField = Entry(tab5,width=5)
SP_6_E3_EntryField.place(x=480, y=180)

SP_7_E3_EntryField = Entry(tab5,width=5)
SP_7_E3_EntryField.place(x=480, y=210)

SP_8_E3_EntryField = Entry(tab5,width=5)
SP_8_E3_EntryField.place(x=480, y=240)

SP_9_E3_EntryField = Entry(tab5,width=5)
SP_9_E3_EntryField.place(x=480, y=270)

SP_10_E3_EntryField = Entry(tab5,width=5)
SP_10_E3_EntryField.place(x=480, y=300)

SP_11_E3_EntryField = Entry(tab5,width=5)
SP_11_E3_EntryField.place(x=480, y=330)

SP_12_E3_EntryField = Entry(tab5,width=5)
SP_12_E3_EntryField.place(x=480, y=360)

SP_13_E3_EntryField = Entry(tab5,width=5)
SP_13_E3_EntryField.place(x=480, y=390)

SP_14_E3_EntryField = Entry(tab5,width=5)
SP_14_E3_EntryField.place(x=480, y=420)

SP_15_E3_EntryField = Entry(tab5,width=5)
SP_15_E3_EntryField.place(x=480, y=450)

SP_16_E3_EntryField = Entry(tab5,width=5)
SP_16_E3_EntryField.place(x=480, y=480)




SP_1_E4_EntryField = Entry(tab5,width=5)
SP_1_E4_EntryField.place(x=520, y=30)

SP_2_E4_EntryField = Entry(tab5,width=5)
SP_2_E4_EntryField.place(x=520, y=60)

SP_3_E4_EntryField = Entry(tab5,width=5)
SP_3_E4_EntryField.place(x=520, y=90)

SP_4_E4_EntryField = Entry(tab5,width=5)
SP_4_E4_EntryField.place(x=520, y=120)

SP_5_E4_EntryField = Entry(tab5,width=5)
SP_5_E4_EntryField.place(x=520, y=150)

SP_6_E4_EntryField = Entry(tab5,width=5)
SP_6_E4_EntryField.place(x=520, y=180)

SP_7_E4_EntryField = Entry(tab5,width=5)
SP_7_E4_EntryField.place(x=520, y=210)

SP_8_E4_EntryField = Entry(tab5,width=5)
SP_8_E4_EntryField.place(x=520, y=240)

SP_9_E4_EntryField = Entry(tab5,width=5)
SP_9_E4_EntryField.place(x=520, y=270)

SP_10_E4_EntryField = Entry(tab5,width=5)
SP_10_E4_EntryField.place(x=520, y=300)

SP_11_E4_EntryField = Entry(tab5,width=5)
SP_11_E4_EntryField.place(x=520, y=330)

SP_12_E4_EntryField = Entry(tab5,width=5)
SP_12_E4_EntryField.place(x=520, y=360)

SP_13_E4_EntryField = Entry(tab5,width=5)
SP_13_E4_EntryField.place(x=520, y=390)

SP_14_E4_EntryField = Entry(tab5,width=5)
SP_14_E4_EntryField.place(x=520, y=420)

SP_15_E4_EntryField = Entry(tab5,width=5)
SP_15_E4_EntryField.place(x=520, y=450)

SP_16_E4_EntryField = Entry(tab5,width=5)
SP_16_E4_EntryField.place(x=520, y=480)

SP_1_E5_EntryField = Entry(tab5,width=5)
SP_1_E5_EntryField.place(x=560, y=30)

SP_2_E5_EntryField = Entry(tab5,width=5)
SP_2_E5_EntryField.place(x=560, y=60)

SP_3_E5_EntryField = Entry(tab5,width=5)
SP_3_E5_EntryField.place(x=560, y=90)

SP_4_E5_EntryField = Entry(tab5,width=5)
SP_4_E5_EntryField.place(x=560, y=120)

SP_5_E5_EntryField = Entry(tab5,width=5)
SP_5_E5_EntryField.place(x=560, y=150)

SP_6_E5_EntryField = Entry(tab5,width=5)
SP_6_E5_EntryField.place(x=560, y=180)

SP_7_E5_EntryField = Entry(tab5,width=5)
SP_7_E5_EntryField.place(x=560, y=210)

SP_8_E5_EntryField = Entry(tab5,width=5)
SP_8_E5_EntryField.place(x=560, y=240)

SP_9_E5_EntryField = Entry(tab5,width=5)
SP_9_E5_EntryField.place(x=560, y=270)

SP_10_E5_EntryField = Entry(tab5,width=5)
SP_10_E5_EntryField.place(x=560, y=300)

SP_11_E5_EntryField = Entry(tab5,width=5)
SP_11_E5_EntryField.place(x=560, y=330)

SP_12_E5_EntryField = Entry(tab5,width=5)
SP_12_E5_EntryField.place(x=560, y=360)

SP_13_E5_EntryField = Entry(tab5,width=5)
SP_13_E5_EntryField.place(x=560, y=390)

SP_14_E5_EntryField = Entry(tab5,width=5)
SP_14_E5_EntryField.place(x=560, y=420)

SP_15_E5_EntryField = Entry(tab5,width=5)
SP_15_E5_EntryField.place(x=560, y=450)

SP_16_E5_EntryField = Entry(tab5,width=5)
SP_16_E5_EntryField.place(x=560, y=480)




SP_1_E6_EntryField = Entry(tab5,width=5)
SP_1_E6_EntryField.place(x=600, y=30)

SP_2_E6_EntryField = Entry(tab5,width=5)
SP_2_E6_EntryField.place(x=600, y=60)

SP_3_E6_EntryField = Entry(tab5,width=5)
SP_3_E6_EntryField.place(x=600, y=90)

SP_4_E6_EntryField = Entry(tab5,width=5)
SP_4_E6_EntryField.place(x=600, y=120)

SP_5_E6_EntryField = Entry(tab5,width=5)
SP_5_E6_EntryField.place(x=600, y=150)

SP_6_E6_EntryField = Entry(tab5,width=5)
SP_6_E6_EntryField.place(x=600, y=180)

SP_7_E6_EntryField = Entry(tab5,width=5)
SP_7_E6_EntryField.place(x=600, y=210)

SP_8_E6_EntryField = Entry(tab5,width=5)
SP_8_E6_EntryField.place(x=600, y=240)

SP_9_E6_EntryField = Entry(tab5,width=5)
SP_9_E6_EntryField.place(x=600, y=270)

SP_10_E6_EntryField = Entry(tab5,width=5)
SP_10_E6_EntryField.place(x=600, y=300)

SP_11_E6_EntryField = Entry(tab5,width=5)
SP_11_E6_EntryField.place(x=600, y=330)

SP_12_E6_EntryField = Entry(tab5,width=5)
SP_12_E6_EntryField.place(x=600, y=360)

SP_13_E6_EntryField = Entry(tab5,width=5)
SP_13_E6_EntryField.place(x=600, y=390)

SP_14_E6_EntryField = Entry(tab5,width=5)
SP_14_E6_EntryField.place(x=600, y=420)

SP_15_E6_EntryField = Entry(tab5,width=5)
SP_15_E6_EntryField.place(x=600, y=450)

SP_16_E6_EntryField = Entry(tab5,width=5)
SP_16_E6_EntryField.place(x=600, y=480)











####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 6




### 6 LABELS#################################################################
#############################################################################

VisBackdropImg = ImageTk.PhotoImage(Image.open('VisBackdrop.png'))
VisBackdromLbl = Label(tab6, image = VisBackdropImg)
VisBackdromLbl.place(x=15, y=215)



#cap= cv2.VideoCapture(0)
video_frame = Frame(tab6,width=640,height=480)
video_frame.place(x=50, y=250)


vid_lbl = Label(video_frame)
vid_lbl.place(x=0, y=0)

vid_lbl.bind('<Button-1>', motion)


LiveLab = Label(tab6, text = "LIVE VIDEO FEED")
LiveLab.place(x=750, y=390)
liveCanvas = Canvas(tab6, width=490, height=330)
liveCanvas.place(x=750, y=410)
live_frame = Frame(tab6,width=480,height=320)
live_frame.place(x=757, y=417)
live_lbl = Label(live_frame)
live_lbl.place(x=0, y=0)



template_frame = Frame(tab6,width=150,height=150)
template_frame.place(x=575, y=50)

template_lbl = Label(template_frame)
template_lbl.place(x=0, y=0)

FoundValuesLab = Label(tab6, text = "FOUND VALUES")
FoundValuesLab.place(x=750, y=30)

CalValuesLab = Label(tab6, text = "CALIBRATION VALUES")
CalValuesLab.place(x=900, y=30)





### 6 BUTTONS################################################################
#############################################################################

graph = FilterGraph()
try:
  camList = graph.get_input_devices()
except:
  camList = ["Select a Camera"]
visoptions=StringVar(tab6)
visoptions.set("Select a Camera")
vismenu=OptionMenu(tab6, visoptions, camList[0], *camList)
vismenu.config(width=20)
vismenu.place(x=10, y=10)


 



StartCamBut = Button(tab6,  text="Start Camera",  width=15, command = start_vid)
StartCamBut.place(x=200, y=10)

StopCamBut = Button(tab6,  text="Stop Camera",  width=15, command = stop_vid)
StopCamBut.place(x=315, y=10)

CapImgBut = Button(tab6,  text="Snap Image",  width=15, command = take_pic)
CapImgBut.place(x=10, y=50)

TeachImgBut = Button(tab6,  text="Teach Object",  width=15, command = selectTemplate)
TeachImgBut.place(x=140, y=50)

FindVisBut = Button(tab6,  text="Snap & Find",  width=15, command = snapFind)
FindVisBut.place(x=270, y=50)


ZeroBrCnBut = Button(tab6, text="Zero",  width=5, command = zeroBrCn)
ZeroBrCnBut.place(x=10, y=110)

maskBut = Button(tab6, text="Mask",  width=5, command = selectMask)
maskBut.place(x=10, y=150)







VisZoomSlide = Scale(tab6, from_=50, to=1,  length=250, orient=HORIZONTAL)
VisZoomSlide.bind("<ButtonRelease-1>", VisUpdateBriCon)
VisZoomSlide.place(x=75, y=95)
VisZoomSlide.set(50)

VisZoomLab = Label(tab6, text = "Zoom")
VisZoomLab.place(x=75, y=115)

VisBrightSlide = Scale(tab6, from_=-127, to=127,  length=250, orient=HORIZONTAL)
VisBrightSlide.bind("<ButtonRelease-1>", VisUpdateBriCon)
VisBrightSlide.place(x=75, y=130)

VisBrightLab = Label(tab6, text = "Brightness")
VisBrightLab.place(x=75, y=150)

VisContrastSlide = Scale(tab6, from_=-127, to=127,  length=250, orient=HORIZONTAL)
VisContrastSlide.bind("<ButtonRelease-1>", VisUpdateBriCon)
VisContrastSlide.place(x=75, y=165)

VisContrastLab = Label(tab6, text = "Contrast")
VisContrastLab.place(x=75, y=185)


fullRotCbut = Checkbutton(tab6, text="Full Rotation Search",variable = fullRot)
fullRotCbut.place(x=900, y=255)

pick180Cbut = Checkbutton(tab6, text="Pick Closest 180°",variable = pick180)
pick180Cbut.place(x=900, y=275)

pickClosestCbut = Checkbutton(tab6, text="Try Closest When Out of Range",variable = pickClosest)
pickClosestCbut.place(x=900, y=295)





saveCalBut = Button(tab6,  text="SAVE VISION DATA",  width=26, command = SaveAndApplyCalibration)
saveCalBut.place(x=915, y=340)





#### 6 ENTRY FIELDS##########################################################
#############################################################################



VisBacColorEntryField = Entry(tab6,width=15)
VisBacColorEntryField.place(x=390, y=100)
VisBacColorLab = Label(tab6, text = "Background Color")
VisBacColorLab.place(x=390, y=120)

bgAutoCbut = Checkbutton(tab6, command=checkAutoBG, text="Auto",variable = autoBG)
bgAutoCbut.place(x=490, y=101)

VisScoreEntryField = Entry(tab6,width=15)
VisScoreEntryField.place(x=390, y=150)
VisScoreLab = Label(tab6, text = "Score Threshold")
VisScoreLab.place(x=390, y=170)




VisRetScoreEntryField = Entry(tab6,width=15)
VisRetScoreEntryField.place(x=750, y=55)
VisRetScoreLab = Label(tab6, text = "Scored Value")
VisRetScoreLab.place(x=750, y=75)

VisRetAngleEntryField = Entry(tab6,width=15)
VisRetAngleEntryField.place(x=750, y=105)
VisRetAngleLab = Label(tab6, text = "Found Angle")
VisRetAngleLab.place(x=750, y=125)

VisRetXpixEntryField = Entry(tab6,width=15)
VisRetXpixEntryField.place(x=750, y=155)
VisRetXpixLab = Label(tab6, text = "Pixel X Position")
VisRetXpixLab.place(x=750, y=175)

VisRetYpixEntryField = Entry(tab6,width=15)
VisRetYpixEntryField.place(x=750, y=205)
VisRetYpixLab = Label(tab6, text = "Pixel Y Position")
VisRetYpixLab.place(x=750, y=225)

VisRetXrobEntryField = Entry(tab6,width=15)
VisRetXrobEntryField.place(x=750, y=255)
VisRetXrobLab = Label(tab6, text = "Robot X Position")
VisRetXrobLab.place(x=750, y=275)

VisRetYrobEntryField = Entry(tab6,width=15)
VisRetYrobEntryField.place(x=750, y=305)
VisRetYrobLab = Label(tab6, text = "Robot Y Position")
VisRetYrobLab.place(x=750, y=325)







VisX1PixEntryField = Entry(tab6,width=15)
VisX1PixEntryField.place(x=900, y=55)
VisX1PixLab = Label(tab6, text = "X1 Pixel Pos")
VisX1PixLab.place(x=900, y=75)

VisY1PixEntryField = Entry(tab6,width=15)
VisY1PixEntryField.place(x=900, y=105)
VisY1PixLab = Label(tab6, text = "Y1 Pixel Pos")
VisY1PixLab.place(x=900, y=125)

VisX2PixEntryField = Entry(tab6,width=15)
VisX2PixEntryField.place(x=900, y=155)
VisX2PixLab = Label(tab6, text = "X2 Pixel Pos")
VisX2PixLab.place(x=900, y=175)

VisY2PixEntryField = Entry(tab6,width=15)
VisY2PixEntryField.place(x=900, y=205)
VisY2PixLab = Label(tab6, text = "Y2 Pixel Pos")
VisY2PixLab.place(x=900, y=225)


VisX1RobEntryField = Entry(tab6,width=15)
VisX1RobEntryField.place(x=1010, y=55)
VisX1RobLab = Label(tab6, text = "X1 Robot Pos")
VisX1RobLab.place(x=1010, y=75)

VisY1RobEntryField = Entry(tab6,width=15)
VisY1RobEntryField.place(x=1010, y=105)
VisY1RobLab = Label(tab6, text = "Y1 Robot Pos")
VisY1RobLab.place(x=1010, y=125)

VisX2RobEntryField = Entry(tab6,width=15)
VisX2RobEntryField.place(x=1010, y=155)
VisX2RobLab = Label(tab6, text = "X2 Robot Pos")
VisX2RobLab.place(x=1010, y=175)

VisY2RobEntryField = Entry(tab6,width=15)
VisY2RobEntryField.place(x=1010, y=205)
VisY2RobLab = Label(tab6, text = "Y2 Robot Pos")
VisY2RobLab.place(x=1010, y=225)





####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 7


GCalmStatusLab = Label(tab7, text = "GCODE IDLE", style="OK.TLabel")
GCalmStatusLab.place(x=400, y=20)


gcodeframe=Frame(tab7)
gcodeframe.place(x=400,y=53)
gcodescrollbar = Scrollbar(gcodeframe) 
gcodescrollbar.pack(side=RIGHT, fill=Y)
tab7.gcodeView = Listbox(gcodeframe,exportselection=0,width=105,height=43, yscrollcommand=gcodescrollbar.set)
tab7.gcodeView.bind('<<ListboxSelect>>', gcodeViewselect)
tab7.gcodeView.pack()
gcodescrollbar.config(command=tab7.gcodeView.yview)


LoadGcodeBut = Button(tab7,  text="Load Program", width=25, command = loadGcodeProg)
LoadGcodeBut.place(x=20, y=20)

GcodeStartPosBut = Button(tab7,  text="Set Start Position", width=25, command = SetGcodeStartPos)
GcodeStartPosBut.place(x=20, y=100)

GcodeMoveStartPosBut = Button(tab7,  text="Move to Start Offset", width=25, command = MoveGcodeStartPos)
GcodeMoveStartPosBut.place(x=20, y=240)

runGcodeBut = Button(tab7,   command = GCrunProg)
playGPhoto=PhotoImage(file="play-icon.gif")
runGcodeBut.config(image=playGPhoto)
runGcodeBut.place(x=20, y=290)

stopGcodeBut = Button(tab7,   command = GCstopProg)
stopGPhoto=PhotoImage(file="stop-icon.gif")
stopGcodeBut.config(image=stopGPhoto)
stopGcodeBut.place(x=100, y=290)

revGcodeBut = Button(tab7,  text="REV ",  command = stepRev)
revGcodeBut.place(x=180, y=290)

fwdGcodeBut = Button(tab7,  text="FWD", command = GCstepFwd)
fwdGcodeBut.place(x=230, y=290)

saveGCBut = Button(tab7,  text="SAVE DATA",  width=26, command = SaveAndApplyCalibration)
saveGCBut.place(x=20, y=500)





GcodeProgEntryField = Entry(tab7,width=60)
GcodeProgEntryField.place(x=20, y=55)

GcodCurRowEntryField = Entry(tab7,width=10)
GcodCurRowEntryField.place(x=1175, y=20)

GC_ST_E1_EntryField = Entry(tab7,width=8)
GC_ST_E1_EntryField.place(x=20, y=140)

GC_ST_E2_EntryField = Entry(tab7,width=8)
GC_ST_E2_EntryField.place(x=75, y=140)

GC_ST_E3_EntryField = Entry(tab7,width=8)
GC_ST_E3_EntryField.place(x=130, y=140)

GC_ST_E4_EntryField = Entry(tab7,width=8)
GC_ST_E4_EntryField.place(x=185, y=140)

GC_ST_E5_EntryField = Entry(tab7,width=8)
GC_ST_E5_EntryField.place(x=240, y=140)

GC_ST_E6_EntryField = Entry(tab7,width=8)
GC_ST_E6_EntryField.place(x=295, y=140)


GC_SToff_E1_EntryField = Entry(tab7,width=8)
GC_SToff_E1_EntryField.place(x=20, y=205)

GC_SToff_E2_EntryField = Entry(tab7,width=8)
GC_SToff_E2_EntryField.place(x=75, y=205)

GC_SToff_E3_EntryField = Entry(tab7,width=8)
GC_SToff_E3_EntryField.place(x=130, y=205)

GC_SToff_E4_EntryField = Entry(tab7,width=8)
GC_SToff_E4_EntryField.place(x=185, y=205)

GC_SToff_E5_EntryField = Entry(tab7,width=8)
GC_SToff_E5_EntryField.place(x=240, y=205)

GC_SToff_E6_EntryField = Entry(tab7,width=8)
GC_SToff_E6_EntryField.place(x=295, y=205)





gcodeCurRowLab = Label(tab7, text = "Current Row: ")
gcodeCurRowLab.place(x=1100, y=21)

gcodStartPosOFfLab = Label(tab7, text = "Start Position Offset")
gcodStartPosOFfLab.place(x=20, y=180)







####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 8

Elogframe=Frame(tab8)
Elogframe.place(x=40,y=15)
scrollbar = Scrollbar(Elogframe) 
scrollbar.pack(side=RIGHT, fill=Y)
tab8.ElogView = Listbox(Elogframe,width=150,height=40, yscrollcommand=scrollbar.set)
try:
  Elog = pickle.load(open("ErrorLog","rb"))
except:
  Elog = ['##BEGINNING OF LOG##']
  pickle.dump(Elog,open("ErrorLog","wb"))
time.sleep(.1)
for item in Elog:
  tab8.ElogView.insert(END,item) 
tab8.ElogView.pack()
scrollbar.config(command=tab8.ElogView.yview)

def clearLog():
 tab8.ElogView.delete(1,END)
 value=tab8.ElogView.get(0,END)
 pickle.dump(value,open("ErrorLog","wb"))

clearLogBut = Button(tab8,  text="Clear Log",  width=26, command = clearLog)
clearLogBut.place(x=1000, y=630)




####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 9

link = Label(tab9, font='12', text="https://www.anninrobotics.com/tutorials",  cursor="hand2")
link.bind("<Button-1>", lambda event: webbrowser.open(link.cget("text")))
link.place(x=10, y=9)

def callback():
    webbrowser.open_new(r"https://www.paypal.me/ChrisAnnin")

donateBut = Button(tab9,  command = callback)
donatePhoto=PhotoImage(file="pp.gif")
donateBut.config(image=donatePhoto)
donateBut.place(x=1250, y=2)


scroll = Scrollbar(tab9)
scroll.pack(side=RIGHT, fill=Y)
configfile = Text(tab9, wrap=WORD, width=166, height=40, yscrollcommand=scroll.set)
filename='information.txt'
with open(filename, 'r', encoding='utf-8-sig') as file:
  configfile.insert(INSERT, file.read())
configfile.pack(side="left")
scroll.config(command=configfile.yview)
configfile.place(x=10, y=40)






##############################################################################################################################################################
### OPEN CAL FILE AND LOAD LIST ##############################################################################################################################
##############################################################################################################################################################



calibration = Listbox(tab2,height=60)

try:
  Cal = pickle.load(open("ARbot.cal","rb"))
except:
  Cal = "0"
  pickle.dump(Cal,open("ARbot.cal","wb"))
for item in Cal:
  calibration.insert(END,item)
global mX1
global mY1
global mX2
global mY2 
J1AngCur   =calibration.get("0")
J2AngCur   =calibration.get("1")
J3AngCur   =calibration.get("2")
J4AngCur   =calibration.get("3")
J5AngCur   =calibration.get("4")
J6AngCur   =calibration.get("5")
XcurPos     =calibration.get("6")
YcurPos     =calibration.get("7")
ZcurPos     =calibration.get("8")
RxcurPos    =calibration.get("9")
RycurPos    =calibration.get("10")
RzcurPos    =calibration.get("11")
comPort     =calibration.get("12")
Prog        =calibration.get("13")
Servo0on    =calibration.get("14")
Servo0off   =calibration.get("15")
Servo1on    =calibration.get("16")
Servo1off   =calibration.get("17")
DO1on       =calibration.get("18")
DO1off      =calibration.get("19")
DO2on       =calibration.get("20")
DO2off      =calibration.get("21")
TFx         =calibration.get("22")
TFy         =calibration.get("23")
TFz         =calibration.get("24")
TFrx        =calibration.get("25")
TFry        =calibration.get("26")
TFrz        =calibration.get("27")
J7PosCur    =calibration.get("28")
J8PosCur    =calibration.get("29")
J9PosCur    =calibration.get("30")
VisFileLoc  =calibration.get("31")
VisProg     =calibration.get("32")
VisOrigXpix =calibration.get("33")
VisOrigXmm  =calibration.get("34")
VisOrigYpix =calibration.get("35")
VisOrigYmm  =calibration.get("36")
VisEndXpix  =calibration.get("37")
VisEndXmm   =calibration.get("38")
VisEndYpix  =calibration.get("39")
VisEndYmm   =calibration.get("40")
J1calOff    =calibration.get("41")
J2calOff    =calibration.get("42")
J3calOff    =calibration.get("43")
J4calOff    =calibration.get("44")
J5calOff    =calibration.get("45")
J6calOff    =calibration.get("46")
J1OpenLoopVal=calibration.get("47")
J2OpenLoopVal=calibration.get("48")
J3OpenLoopVal=calibration.get("49")
J4OpenLoopVal=calibration.get("50")
J5OpenLoopVal=calibration.get("51")
J6OpenLoopVal=calibration.get("52")
com2Port     =calibration.get("53")
curTheme     =calibration.get("54")
J1CalStatVal= calibration.get("55")
J2CalStatVal= calibration.get("56")
J3CalStatVal= calibration.get("57")
J4CalStatVal= calibration.get("58")
J5CalStatVal= calibration.get("59")
J6CalStatVal= calibration.get("60")
J7PosLim= calibration.get("61")
J7rotation  = calibration.get("62")
J7steps     = calibration.get("63")
J7StepCur   = calibration.get("64") #is this used
J1CalStatVal2= calibration.get("65")
J2CalStatVal2= calibration.get("66")
J3CalStatVal2= calibration.get("67")
J4CalStatVal2= calibration.get("68")
J5CalStatVal2= calibration.get("69")
J6CalStatVal2= calibration.get("70")
VisBrightVal= calibration.get("71")
VisContVal  = calibration.get("72")
VisBacColor = calibration.get("73")
VisScore    = calibration.get("74")
VisX1Val    = calibration.get("75")
VisY1Val    = calibration.get("76")
VisX2Val    = calibration.get("77")
VisY2Val    = calibration.get("78")
VisRobX1Val = calibration.get("79")
VisRobY1Val = calibration.get("80")
VisRobX2Val = calibration.get("81")
VisRobY2Val = calibration.get("82")
zoom        = calibration.get("83")
pick180Val  = calibration.get("84")
pickClosestVal=calibration.get("85")
curCam      = calibration.get("86")
fullRotVal  = calibration.get("87")
autoBGVal   = calibration.get("88")
mX1val      = calibration.get("89")
mY1val      = calibration.get("90")
mX2val      = calibration.get("91")
mY2val      = calibration.get("92")
J8length    = calibration.get("93")
J8rotation  = calibration.get("94")
J8steps     = calibration.get("95")
J9length    = calibration.get("96")
J9rotation  = calibration.get("97")
J9steps     = calibration.get("98")
J7calOff    = calibration.get("99")
J8calOff    = calibration.get("100")
J9calOff    = calibration.get("101")
GC_ST_E1    = calibration.get("102")
GC_ST_E2    = calibration.get("103")
GC_ST_E3    = calibration.get("104")
GC_ST_E4    = calibration.get("105")
GC_ST_E5    = calibration.get("106")
GC_ST_E6    = calibration.get("107")
GC_SToff_E1 = calibration.get("108")
GC_SToff_E2 = calibration.get("109")
GC_SToff_E3 = calibration.get("110")
GC_SToff_E4 = calibration.get("111")
GC_SToff_E5 = calibration.get("112")
GC_SToff_E6 = calibration.get("113")
DisableWristRotVal = calibration.get("114")
J1MotDir    = calibration.get("115")
J2MotDir    = calibration.get("116")
J3MotDir    = calibration.get("117")
J4MotDir    = calibration.get("118")
J5MotDir    = calibration.get("119")
J6MotDir    = calibration.get("120")
J7MotDir    = calibration.get("121")
J8MotDir    = calibration.get("122")
J9MotDir    = calibration.get("123")
J1CalDir    = calibration.get("124")
J2CalDir    = calibration.get("125")
J3CalDir    = calibration.get("126")
J4CalDir    = calibration.get("127")
J5CalDir    = calibration.get("128")
J6CalDir    = calibration.get("129")
J7CalDir    = calibration.get("130")
J8CalDir    = calibration.get("131")
J9CalDir    = calibration.get("132")
J1PosLim    = calibration.get("133")
J1NegLim    = calibration.get("134")
J2PosLim    = calibration.get("135")
J2NegLim    = calibration.get("136")
J3PosLim    = calibration.get("137")
J3NegLim    = calibration.get("138")
J4PosLim    = calibration.get("139")
J4NegLim    = calibration.get("140")
J5PosLim    = calibration.get("141")
J5NegLim    = calibration.get("142")
J6PosLim    = calibration.get("143")
J6NegLim    = calibration.get("144")
J1StepDeg   = calibration.get("145")
J2StepDeg   = calibration.get("146")
J3StepDeg   = calibration.get("147")
J4StepDeg   = calibration.get("148")
J5StepDeg   = calibration.get("149")
J6StepDeg   = calibration.get("150")
J1DriveMS   = calibration.get("151")
J2DriveMS   = calibration.get("152")
J3DriveMS   = calibration.get("153")
J4DriveMS   = calibration.get("154")
J5DriveMS   = calibration.get("155")
J6DriveMS   = calibration.get("156")
J1EncCPR    = calibration.get("157")
J2EncCPR    = calibration.get("158")
J3EncCPR    = calibration.get("159")
J4EncCPR    = calibration.get("160")
J5EncCPR    = calibration.get("161")
J6EncCPR    = calibration.get("162")
J1ΘDHpar    = calibration.get("163")
J2ΘDHpar    = calibration.get("164")
J3ΘDHpar    = calibration.get("165")
J4ΘDHpar    = calibration.get("166")
J5ΘDHpar    = calibration.get("167")
J6ΘDHpar    = calibration.get("168")
J1αDHpar    = calibration.get("169")
J2αDHpar    = calibration.get("170")
J3αDHpar    = calibration.get("171")
J4αDHpar    = calibration.get("172")
J5αDHpar    = calibration.get("173")
J6αDHpar    = calibration.get("174")
J1dDHpar    = calibration.get("175")
J2dDHpar    = calibration.get("176")
J3dDHpar    = calibration.get("177")
J4dDHpar    = calibration.get("178")
J5dDHpar    = calibration.get("179")
J6dDHpar    = calibration.get("180")
J1aDHpar    = calibration.get("181")
J2aDHpar    = calibration.get("182")
J3aDHpar    = calibration.get("183")
J4aDHpar    = calibration.get("184")
J5aDHpar    = calibration.get("185")
J6aDHpar    = calibration.get("186")



####  

comPortEntryField.insert(0,str(comPort))
com2PortEntryField.insert(0,str(com2Port))
incrementEntryField.insert(0,"10")
speedEntryField.insert(0,"25")
ACCspeedField.insert(0,"20")
DECspeedField.insert(0,"20")
ACCrampField.insert(0,"100")
roundEntryField.insert(0,"0")
#ProgEntryField.insert(0,(Prog))
SavePosEntryField.insert(0,"1")
R1EntryField.insert(0,"0")
R2EntryField.insert(0,"0")
R3EntryField.insert(0,"0")
R4EntryField.insert(0,"0")
R5EntryField.insert(0,"0")
R6EntryField.insert(0,"0")
R7EntryField.insert(0,"0")
R8EntryField.insert(0,"0")
R9EntryField.insert(0,"0")
R10EntryField.insert(0,"0")
R11EntryField.insert(0,"0")
R12EntryField.insert(0,"0")
R13EntryField.insert(0,"0")
R14EntryField.insert(0,"0")
R15EntryField.insert(0,"0")
R16EntryField.insert(0,"0")
SP_1_E1_EntryField.insert(0,"0")
SP_2_E1_EntryField.insert(0,"0")
SP_3_E1_EntryField.insert(0,"0")
SP_4_E1_EntryField.insert(0,"0")
SP_5_E1_EntryField.insert(0,"0")
SP_6_E1_EntryField.insert(0,"0")
SP_7_E1_EntryField.insert(0,"0")
SP_8_E1_EntryField.insert(0,"0")
SP_9_E1_EntryField.insert(0,"0")
SP_10_E1_EntryField.insert(0,"0")
SP_11_E1_EntryField.insert(0,"0")
SP_12_E1_EntryField.insert(0,"0")
SP_13_E1_EntryField.insert(0,"0")
SP_14_E1_EntryField.insert(0,"0")
SP_15_E1_EntryField.insert(0,"0")
SP_16_E1_EntryField.insert(0,"0")
SP_1_E2_EntryField.insert(0,"0")
SP_2_E2_EntryField.insert(0,"0")
SP_3_E2_EntryField.insert(0,"0")
SP_4_E2_EntryField.insert(0,"0")
SP_5_E2_EntryField.insert(0,"0")
SP_6_E2_EntryField.insert(0,"0")
SP_7_E2_EntryField.insert(0,"0")
SP_8_E2_EntryField.insert(0,"0")
SP_9_E2_EntryField.insert(0,"0")
SP_10_E2_EntryField.insert(0,"0")
SP_11_E2_EntryField.insert(0,"0")
SP_12_E2_EntryField.insert(0,"0")
SP_13_E2_EntryField.insert(0,"0")
SP_14_E2_EntryField.insert(0,"0")
SP_15_E2_EntryField.insert(0,"0")
SP_16_E2_EntryField.insert(0,"0")
SP_1_E3_EntryField.insert(0,"0")
SP_2_E3_EntryField.insert(0,"0")
SP_3_E3_EntryField.insert(0,"0")
SP_4_E3_EntryField.insert(0,"0")
SP_5_E3_EntryField.insert(0,"0")
SP_6_E3_EntryField.insert(0,"0")
SP_7_E3_EntryField.insert(0,"0")
SP_8_E3_EntryField.insert(0,"0")
SP_9_E3_EntryField.insert(0,"0")
SP_10_E3_EntryField.insert(0,"0")
SP_11_E3_EntryField.insert(0,"0")
SP_12_E3_EntryField.insert(0,"0")
SP_13_E3_EntryField.insert(0,"0")
SP_14_E3_EntryField.insert(0,"0")
SP_15_E3_EntryField.insert(0,"0")
SP_16_E3_EntryField.insert(0,"0")
SP_1_E4_EntryField.insert(0,"0")
SP_2_E4_EntryField.insert(0,"0")
SP_3_E4_EntryField.insert(0,"0")
SP_4_E4_EntryField.insert(0,"0")
SP_5_E4_EntryField.insert(0,"0")
SP_6_E4_EntryField.insert(0,"0")
SP_7_E4_EntryField.insert(0,"0")
SP_8_E4_EntryField.insert(0,"0")
SP_9_E4_EntryField.insert(0,"0")
SP_10_E4_EntryField.insert(0,"0")
SP_11_E4_EntryField.insert(0,"0")
SP_12_E4_EntryField.insert(0,"0")
SP_13_E4_EntryField.insert(0,"0")
SP_14_E4_EntryField.insert(0,"0")
SP_15_E4_EntryField.insert(0,"0")
SP_16_E4_EntryField.insert(0,"0")
SP_1_E5_EntryField.insert(0,"0")
SP_2_E5_EntryField.insert(0,"0")
SP_3_E5_EntryField.insert(0,"0")
SP_4_E5_EntryField.insert(0,"0")
SP_5_E5_EntryField.insert(0,"0")
SP_6_E5_EntryField.insert(0,"0")
SP_7_E5_EntryField.insert(0,"0")
SP_8_E5_EntryField.insert(0,"0")
SP_9_E5_EntryField.insert(0,"0")
SP_10_E5_EntryField.insert(0,"0")
SP_11_E5_EntryField.insert(0,"0")
SP_12_E5_EntryField.insert(0,"0")
SP_13_E5_EntryField.insert(0,"0")
SP_14_E5_EntryField.insert(0,"0")
SP_15_E5_EntryField.insert(0,"0")
SP_16_E5_EntryField.insert(0,"0")
SP_1_E6_EntryField.insert(0,"0")
SP_2_E6_EntryField.insert(0,"0")
SP_3_E6_EntryField.insert(0,"0")
SP_4_E6_EntryField.insert(0,"0")
SP_5_E6_EntryField.insert(0,"0")
SP_6_E6_EntryField.insert(0,"0")
SP_7_E6_EntryField.insert(0,"0")
SP_8_E6_EntryField.insert(0,"0")
SP_9_E6_EntryField.insert(0,"0")
SP_10_E6_EntryField.insert(0,"0")
SP_11_E6_EntryField.insert(0,"0")
SP_12_E6_EntryField.insert(0,"0")
SP_13_E6_EntryField.insert(0,"0")
SP_14_E6_EntryField.insert(0,"0")
SP_15_E6_EntryField.insert(0,"0")
SP_16_E6_EntryField.insert(0,"0")
servo0onEntryField.insert(0,str(Servo0on))
servo0offEntryField.insert(0,str(Servo0off))
servo1onEntryField.insert(0,str(Servo1on))
servo1offEntryField.insert(0,str(Servo1off))
DO1onEntryField.insert(0,str(DO1on))
DO1offEntryField.insert(0,str(DO1off))
DO2onEntryField.insert(0,str(DO2on))
DO2offEntryField.insert(0,str(DO2off))
TFxEntryField.insert(0,str(TFx))
TFyEntryField.insert(0,str(TFy))
TFzEntryField.insert(0,str(TFz))
TFrxEntryField.insert(0,str(TFrx))
TFryEntryField.insert(0,str(TFry))
TFrzEntryField.insert(0,str(TFrz))
J7curAngEntryField.insert(0,str(J7PosCur))
J8curAngEntryField.insert(0,str(J8PosCur))
J9curAngEntryField.insert(0,str(J9PosCur))
J1calOffEntryField.insert(0,str(J1calOff))
J2calOffEntryField.insert(0,str(J2calOff))
J3calOffEntryField.insert(0,str(J3calOff))
J4calOffEntryField.insert(0,str(J4calOff))
J5calOffEntryField.insert(0,str(J5calOff))
J6calOffEntryField.insert(0,str(J6calOff))
J7calOffEntryField.insert(0,str(J7calOff))
J8calOffEntryField.insert(0,str(J8calOff))
J9calOffEntryField.insert(0,str(J9calOff))
if (J1OpenLoopVal == 1):
  J1OpenLoopStat.set(True)
if (J2OpenLoopVal == 1):
  J2OpenLoopStat.set(True)
if (J3OpenLoopVal == 1):
  J3OpenLoopStat.set(True)
if (J4OpenLoopVal == 1):
  J4OpenLoopStat.set(True)
if (J5OpenLoopVal == 1):
  J5OpenLoopStat.set(True)
if (J6OpenLoopVal == 1):
  J6OpenLoopStat.set(True)
if (DisableWristRotVal == 1):
  DisableWristRot.set(True)  
if (curTheme == 1): 
  lightTheme()
else:
  darkTheme()  
if (J1CalStatVal == 1):
  J1CalStat.set(True)
if (J2CalStatVal == 1):
  J2CalStat.set(True)
if (J3CalStatVal == 1):
  J3CalStat.set(True)
if (J4CalStatVal == 1):
  J4CalStat.set(True)
if (J5CalStatVal == 1):
  J5CalStat.set(True)
if (J6CalStatVal == 1):
  J6CalStat.set(True)  
if (J1CalStatVal2 == 1):
  J1CalStat2.set(True)
if (J2CalStatVal2 == 1):
  J2CalStat2.set(True)
if (J3CalStatVal2 == 1):
  J3CalStat2.set(True)
if (J4CalStatVal2 == 1):
  J4CalStat2.set(True)
if (J5CalStatVal2 == 1):
  J5CalStat2.set(True)
if (J6CalStatVal2 == 1):
  J6CalStat2.set(True)    
axis7lengthEntryField.insert(0,str(J7PosLim))
axis7rotEntryField.insert(0,str(J7rotation))
axis7stepsEntryField.insert(0,str(J7steps))
VisBrightSlide.set(VisBrightVal)
VisContrastSlide.set(VisContVal)
VisBacColorEntryField.insert(0,str(VisBacColor))
VisScoreEntryField.insert(0,str(VisScore))
VisX1PixEntryField.insert(0,str(VisX1Val))
VisY1PixEntryField.insert(0,str(VisY1Val))
VisX2PixEntryField.insert(0,str(VisX2Val))
VisY2PixEntryField.insert(0,str(VisY2Val))
VisX1RobEntryField.insert(0,str(VisRobX1Val))
VisY1RobEntryField.insert(0,str(VisRobY1Val))
VisX2RobEntryField.insert(0,str(VisRobX2Val))
VisY2RobEntryField.insert(0,str(VisRobY2Val))
VisZoomSlide.set(zoom)
if (pickClosestVal == 1):
  pickClosest.set(True)
if (pick180Val == 1):
  pick180.set(True)  
visoptions.set(curCam)
if (fullRotVal == 1):
  fullRot.set(True)
if (autoBGVal == 1):
  autoBG.set(True)  
mX1 = mX1val
mY1 = mY1val
mX2 = mX2val
mY2 = mY2val
axis8lengthEntryField.insert(0,str(J8length))
axis8rotEntryField.insert(0,str(J8rotation))
axis8stepsEntryField.insert(0,str(J8steps))
axis9lengthEntryField.insert(0,str(J9length))
axis9rotEntryField.insert(0,str(J9rotation))
axis9stepsEntryField.insert(0,str(J9steps))
GC_ST_E1_EntryField.insert(0,str(GC_ST_E1))
GC_ST_E2_EntryField.insert(0,str(GC_ST_E2))
GC_ST_E3_EntryField.insert(0,str(GC_ST_E3))
GC_ST_E4_EntryField.insert(0,str(GC_ST_E4))
GC_ST_E5_EntryField.insert(0,str(GC_ST_E5))
GC_ST_E6_EntryField.insert(0,str(GC_ST_E6))
GC_SToff_E1_EntryField.insert(0,str(GC_SToff_E1))
GC_SToff_E2_EntryField.insert(0,str(GC_SToff_E2))
GC_SToff_E3_EntryField.insert(0,str(GC_SToff_E3))
GC_SToff_E4_EntryField.insert(0,str(GC_SToff_E4))
GC_SToff_E5_EntryField.insert(0,str(GC_SToff_E5))
GC_SToff_E6_EntryField.insert(0,str(GC_SToff_E6))
J1MotDirEntryField.insert(0,str(J1MotDir))
J2MotDirEntryField.insert(0,str(J2MotDir))
J3MotDirEntryField.insert(0,str(J3MotDir))
J4MotDirEntryField.insert(0,str(J4MotDir))
J5MotDirEntryField.insert(0,str(J5MotDir))
J6MotDirEntryField.insert(0,str(J6MotDir))
J7MotDirEntryField.insert(0,str(J7MotDir))
J8MotDirEntryField.insert(0,str(J8MotDir))
J9MotDirEntryField.insert(0,str(J9MotDir))
J1CalDirEntryField.insert(0,str(J1CalDir))
J2CalDirEntryField.insert(0,str(J2CalDir))
J3CalDirEntryField.insert(0,str(J3CalDir))
J4CalDirEntryField.insert(0,str(J4CalDir))
J5CalDirEntryField.insert(0,str(J5CalDir))
J6CalDirEntryField.insert(0,str(J6CalDir))
J7CalDirEntryField.insert(0,str(J7CalDir))
J8CalDirEntryField.insert(0,str(J8CalDir))
J9CalDirEntryField.insert(0,str(J9CalDir))
J1PosLimEntryField.insert(0,str(J1PosLim))
J1NegLimEntryField.insert(0,str(J1NegLim))
J2PosLimEntryField.insert(0,str(J2PosLim))
J2NegLimEntryField.insert(0,str(J2NegLim))
J3PosLimEntryField.insert(0,str(J3PosLim))
J3NegLimEntryField.insert(0,str(J3NegLim))
J4PosLimEntryField.insert(0,str(J4PosLim))
J4NegLimEntryField.insert(0,str(J4NegLim))
J5PosLimEntryField.insert(0,str(J5PosLim))
J5NegLimEntryField.insert(0,str(J5NegLim))
J6PosLimEntryField.insert(0,str(J6PosLim))
J6NegLimEntryField.insert(0,str(J6NegLim))  
J1StepDegEntryField.insert(0,str(J1StepDeg))
J2StepDegEntryField.insert(0,str(J2StepDeg)) 
J3StepDegEntryField.insert(0,str(J3StepDeg)) 
J4StepDegEntryField.insert(0,str(J4StepDeg)) 
J5StepDegEntryField.insert(0,str(J5StepDeg)) 
J6StepDegEntryField.insert(0,str(J6StepDeg))
J1DriveMSEntryField.insert(0,str(J1DriveMS))
J2DriveMSEntryField.insert(0,str(J2DriveMS))  
J3DriveMSEntryField.insert(0,str(J3DriveMS))  
J4DriveMSEntryField.insert(0,str(J4DriveMS))  
J5DriveMSEntryField.insert(0,str(J5DriveMS))  
J6DriveMSEntryField.insert(0,str(J6DriveMS))
J1EncCPREntryField.insert(0,str(J1EncCPR))
J2EncCPREntryField.insert(0,str(J2EncCPR))
J3EncCPREntryField.insert(0,str(J3EncCPR))
J4EncCPREntryField.insert(0,str(J4EncCPR))
J5EncCPREntryField.insert(0,str(J5EncCPR))
J6EncCPREntryField.insert(0,str(J6EncCPR))
J1ΘEntryField.insert(0,str(J1ΘDHpar))
J2ΘEntryField.insert(0,str(J2ΘDHpar))
J3ΘEntryField.insert(0,str(J3ΘDHpar))
J4ΘEntryField.insert(0,str(J4ΘDHpar))
J5ΘEntryField.insert(0,str(J5ΘDHpar))
J6ΘEntryField.insert(0,str(J6ΘDHpar))
J1αEntryField.insert(0,str(J1αDHpar))
J2αEntryField.insert(0,str(J2αDHpar))
J3αEntryField.insert(0,str(J3αDHpar))
J4αEntryField.insert(0,str(J4αDHpar))
J5αEntryField.insert(0,str(J5αDHpar))
J6αEntryField.insert(0,str(J6αDHpar))
J1dEntryField.insert(0,str(J1dDHpar))
J2dEntryField.insert(0,str(J2dDHpar))
J3dEntryField.insert(0,str(J3dDHpar))
J4dEntryField.insert(0,str(J4dDHpar))
J5dEntryField.insert(0,str(J5dDHpar))
J6dEntryField.insert(0,str(J6dDHpar))
J1aEntryField.insert(0,str(J1aDHpar))
J2aEntryField.insert(0,str(J2aDHpar))
J3aEntryField.insert(0,str(J3aDHpar))
J4aEntryField.insert(0,str(J4aDHpar))
J5aEntryField.insert(0,str(J5aDHpar))
J6aEntryField.insert(0,str(J6aDHpar))     

setCom()
time.sleep(.1)
setCom2()
time.sleep(.1)
updateVisOp()
time.sleep(.1)
checkAutoBG()


msg = "ANNIN ROBOTICS SOFTWARE AND DESIGNS ARE FREE:\n\
\n\
*for personal use.\n\
*for educational use.\n\
*for building your own robot(s).\n\
*for automating your own business.\n\
\n\
IT IS NOT OK TO RESELL THIS SOFTWARE OR ROBOTS\n\
FOR A PROFIT - IT MUST REMAIN FREE.\n\
\n\
IT IS NOT OK TO SELL ANNIN ROBOTICS ROBOTS,\n\
ROBOT PARTS, OR ANY OTHER VERSION \n\
OF ROBOT OR SOFTWARE BASED ON\n\
ANNIN ROBOTICS DESIGNS FOR PROFIT.\n\
\n\
ANY AR ROBOTS OR PARTS FOR SALE ON ALIEXPRESS\n\
OR ANY OTHER PLATFORM NOT PURCHASED FROM ANNIN ROBOTICS\n\
ARE COUNTERFEIT & ILLEGAL\n\
\n\
AR3 and AR4 are registered trademarks of Annin Robotics\n\
Copyright © 2022 by Annin Robotics. All Rights Reserved"


tkinter.messagebox.showwarning("AR4 License / Copyright notice", msg)
xboxUse = 0



tab1.mainloop()



#manEntryField.delete(0, 'end')
#manEntryField.insert(0,value)