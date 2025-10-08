############################################################################
## Version AR4 2.0 #########################################################
############################################################################
""" AR4 - robot control software
    Copyright (c) 2021, Chris Annin
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

'''
##########################################################################
##########################################################################



from multiprocessing.resource_sharer import stop
from os import execv
from tkinter import *
from tkinter.ttk import *
from tkinter import ttk
from ttkthemes import ThemedStyle
from tkinter import messagebox

import pickle
import serial
import time
import threading
import queue
import math
import tkinter.messagebox
import webbrowser
import numpy as np
import datetime


root = Tk()
root.wm_title("AR4 Software Ver 2.0")
root.iconbitmap(r'AR.ico')
root.resizable(width=False, height=False)
root.geometry('1536x792+0+0')
root.runTrue = 0

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

global SplineTrue;
SplineTrue = False;

#define axis limits in degrees
J1axisLimPos = 170;
J1axisLimNeg = 170;
J2axisLimPos = 90;
J2axisLimNeg = 42;
J3axisLimPos = 52;
J3axisLimNeg = 89;
J4axisLimPos = 165;
J4axisLimNeg = 165;
J5axisLimPos = 105;
J5axisLimNeg = 105;
J6axisLimPos = 155;
J6axisLimNeg = 155;
TRaxisLimPos = 340;
TRaxisLimNeg = 0;

#define total axis travel
J1axisLim = J1axisLimPos + J1axisLimNeg;
J2axisLim = J2axisLimPos + J2axisLimNeg;
J3axisLim = J3axisLimPos + J3axisLimNeg;
J4axisLim = J4axisLimPos + J4axisLimNeg;
J5axisLim = J5axisLimPos + J5axisLimNeg;
J6axisLim = J6axisLimPos + J6axisLimNeg;


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
nb.add(tab3, text=' Inputs Outputs ')

tab4 = tkinter.ttk.Frame(nb)
nb.add(tab4, text='   Registers    ')

tab5 = tkinter.ttk.Frame(nb)
nb.add(tab5, text='   Vision    ')

tab6 = tkinter.ttk.Frame(nb)
nb.add(tab6, text='      Log      ')

tab7 = tkinter.ttk.Frame(nb)
nb.add(tab7, text='   Info    ')

tab10 = tkinter.ttk.Frame(nb)
#nb.add(tab10, text='   Testing    ')


###############################################################################################################################################################
### STARTUP DEFS ################################################################################################################# COMMUNICATION DEFS ###
###############################################################################################################################################################


def startup():
  global moveInProc
  moveInProc = 0
  toolFrame()
  calAxis7()
  sendPos()
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
    tab6.ElogView.insert(END, Curtime+" - COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER")
    value=tab6.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
    time.sleep(1)
    ser.flushInput()
    startup()
  except:
    almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER", style="Alarm.TLabel")
    almStatusLab2.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER", style="Alarm.TLabel")
    Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
    tab6.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER")
    value=tab6.ElogView.get(0,END)
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
    tab6.ElogView.insert(END, Curtime+" - COMMUNICATIONS STARTED WITH ARDUINO IO BOARD")
    value=tab6.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
  except:
    #almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD", style="Alarm.TLabel")
    #almStatusLab2.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD", style="Alarm.TLabel")
    Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
    tab6.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD")
    value=tab6.ElogView.get(0,END)
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
        time.sleep(.01)	  
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
      time.sleep(.01)
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
    time.sleep(.2)
    command = "RP\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.05)
    response = str(ser.readline().strip(),'utf-8')
    displayPosition(response) 
    if (selRow >= tab1.progView.size()): 
        curRowEntryField.delete(0, 'end')
        curRowEntryField.insert(0,"---") 
        tab1.runTrue = 0
        time.sleep(.01)
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
    time.sleep(.2)
    command = "RP\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.05)
    response = str(ser.readline().strip(),'utf-8')
    displayPosition(response) 
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
  startTime = time.time()
  selRow = tab1.progView.curselection()[0]
  tab1.progView.see(selRow+2)
  data = list(map(int, tab1.progView.curselection()))
  command=tab1.progView.get(data[0])
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
    loadProg()
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
    loadProg()
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
    time.sleep(.2)
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
    time.sleep(.2)
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
    time.sleep(.2)
    response = str(ser2.readline().strip(),'utf-8')
    if (response == "T"):
      index = tab1.progView.get(0, "end").index("Tab Number " + tabNum)
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
    time.sleep(.2)
    response = str(ser2.readline().strip(),'utf-8')
    if (response == "F"):
      index = tab1.progView.get(0, "end").index("Tab Number " + tabNum)
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
    time.sleep(.2)
    response = str(ser.readline().strip(),'utf-8')
    if (response == "T"):
      index = tab1.progView.get(0, "end").index("Tab Number " + tabNum)
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
    time.sleep(.2)
    response = str(ser.readline().strip(),'utf-8')
    if (response == "F"):
      index = tab1.progView.get(0, "end").index("Tab Number " + tabNum)
      index = index-1
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index)

  ##Jump to Row##
  if (cmdType == "Jump T"):
    if (moveInProc == 1):
      moveInProc == 2
    tabIndex = command.find("Tab-")
    tabNum = str(command[tabIndex+4:])
    index = tab1.progView.get(0, "end").index("Tab Number " + tabNum)
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
    time.sleep(.2)
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
    time.sleep(.2)
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
    time.sleep(.2)
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
    time.sleep(.2)
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
    time.sleep(.2)
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
    time.sleep(.2)
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
    time.sleep(.2)
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
    time.sleep(.2)
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
    time.sleep(.2)
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
  ## Get Vision ##
  if (cmdType == "Get Vi"):
    if (moveInProc == 1):
      moveInProc == 2
    testvis()	
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
      index = tab1.progView.get(0, "end").index("Tab Number " + tabNum)
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
    time.sleep(.2)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.2)
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
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.2)
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
    trIndex = command.find(" Tr ")	
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
    rxVal = str(float(crx) + float(command[rxIndex+4:trIndex]))
    trVal = command[trIndex+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.2)
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
    trIndex = command.find(" Tr ")	
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
    trVal = command[trIndex+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.2)
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
    trIndex = command.find(" Tr ")	
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
    trVal = command[trIndex+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.2)
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
    trIndex = command.find(" Tr ")	
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
    rxVal = command[rxIndex+4:trIndex]
    trVal = command[trIndex+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:RoundingIndex]
    Rounding = command[RoundingIndex+5:WristConfIndex]
    WC = command[WristConfIndex+3:]
    command = "ML"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+WC+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.2)
    response = str(ser.readline().strip(),'utf-8')
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
    trIndex = command.find(" Tr ")	
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
    J6Val = command[J6Index+4:trIndex]
    trVal = command[trIndex+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    WC = command[WristConfIndex+3:]
    command = "RJ"+"A"+J1Val+"B"+J2Val+"C"+J3Val+"D"+J4Val+"E"+J5Val+"F"+J6Val+"G"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.2)
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
      command=tab1.progView.get(data[0])
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
      command = "MA"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Ex"+Xend+"Ey"+Yend+"Ez"+Zend+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      ser.write(command.encode())
      ser.flushInput()
      time.sleep(.2)
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
      command=tab1.progView.get(data[0])
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
      command=tab1.progView.get(data[0])
      xIndex = command.find(" X ")
      yIndex = command.find(" Y ")
      zIndex = command.find(" Z ")
      Xend = command[xIndex+3:yIndex]
      Yend = command[yIndex+3:zIndex]
      Zend = command[zIndex+3:rzIndex]
      #move j to the beginning (second or mid point is start of circle)
      command = "MJ"+"X"+Xmid+"Y"+Ymid+"Z"+Zmid+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
      ser.write(command.encode())
      ser.flushInput()
      time.sleep(.2)
      response = str(ser.readline().strip(),'utf-8')
      #move circle command
      command = "MC"+"Cx"+xVal+"Cy"+yVal+"Cz"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Bx"+Xmid+"By"+Ymid+"Bz"+Zmid+"Px"+Xend+"Py"+Yend+"Pz"+Zend+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
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
    time.sleep(.2)
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
    time.sleep(.2)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response) 

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
          ##TRACK MODE
          elif (event.code == 'BTN_START' and event.state == 1): 
            mainMode = 4
            almStatusLab.config(text='JOGGING TRACK', style="Warn.TLabel")
            almStatusLab2.config(text='JOGGING TRACK', style="Warn.TLabel")
          ##TRACK JOG
          elif (mainMode == 4 and event.code == 'ABS_HAT0X' and event.state == 1): 
            TrackjogPos(float(incrementEntryField.get()))    
          elif (mainMode == 4 and event.code == 'ABS_HAT0X' and event.state == -1): 
            TrackjogNeg(float(incrementEntryField.get()))                   
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
              time.sleep(.2)
              ser2.read() 
            else:
              grip = 0
              outputNum = DO1onEntryField.get()
              command = "ONX"+outputNum+"\n"
              ser2.write(command.encode())
              ser2.flushInput()
              time.sleep(.2)
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
  command = "RJ"+"A"+str(float(J1AngCur)-value)+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+str(float(J1AngCur)+value)+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+str(float(J2AngCur)-value)+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+str(float(J2AngCur)+value)+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+str(float(J3AngCur)-value)+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+str(float(J3AngCur)+value)+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+str(float(J4AngCur)-value)+"E"+J5AngCur+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+str(float(J4AngCur)+value)+"E"+J5AngCur+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+str(float(J5AngCur)-value)+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+str(float(J5AngCur)+value)+"F"+J6AngCur+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+str(float(J6AngCur)-value)+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+str(float(J6AngCur)+value)+"G"+TrackcurPos+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 

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
  trVal = TRcurAngEntryField.get()
  command = "LC"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.1)
  ser.read()

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
  trVal = TRcurAngEntryField.get()
  command = "LJ"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
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
  trVal = TRcurAngEntryField.get()
  command = "LT"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
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
    time.sleep(.2)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)    
    else:
      displayPosition(response)


def TrackjogNeg(value):
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"G"+str(float(TrackcurPos)-value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def TrackjogPos(value):
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
  command = "RJ"+"A"+J1AngCur+"B"+J2AngCur+"C"+J3AngCur+"D"+J4AngCur+"E"+J5AngCur+"F"+J6AngCur+"G"+str(float(TrackcurPos)+value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 

def TRstilljogNeg(value):
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
  trVal = str(float(TrackcurPos) - value)
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    displayPosition(response)     

def TRstilljogPos(value):
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
  trVal = str(float(TrackcurPos) + value)
  command = "ML"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  trVal = TRcurAngEntryField.get()
  command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTX1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTY1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTZ1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTW1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTP1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTR1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTX0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTY0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTZ0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTW0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTP0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  command = "JTR0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  ser.flushInput()
  time.sleep(.2)
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
  global TrackcurPos
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
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" Tr "+TrackcurPos+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC              
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))
  elif(movetype == "Move PR"):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ]"
    newPos = movetype + " [*]"+" Tr "+TrackcurPos+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC            
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))
  elif(movetype == "OFF PR "):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ] offs [ *PR: "+str(int(SavePosEntryField.get())+1)+" ] "
    newPos = movetype + " [*]"+" Tr "+TrackcurPos+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))
  elif(movetype == "Move J"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" Tr "+TrackcurPos+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC              
    tab1.progView.insert(selRow, newPos)
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))
  elif(movetype == "Move L"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" Tr "+TrackcurPos+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" Rnd "+Rounding+" $ "+WC             
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))
  elif(movetype == "Move R"):
    newPos = movetype + " [*] J1 "+J1AngCur+" J2 "+J2AngCur+" J3 "+J3AngCur+" J4 "+J4AngCur+" J5 "+J5AngCur+" J6 "+J6AngCur+" Tr "+TrackcurPos+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC              
    tab1.progView.insert(selRow, newPos)
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))
  elif(movetype == "Move A Mid"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" Tr "+TrackcurPos+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC             
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))	
  elif(movetype == "Move A End"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" Tr "+TrackcurPos+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC             
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))	
  elif(movetype == "Move C Center"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos+" Rz "+RzcurPos+" Ry "+RycurPos+" Rx "+RxcurPos+" Tr "+TrackcurPos+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+WC              
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))
  elif(movetype == "Move C Start"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos                 
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))	
  elif(movetype == "Move C Plane"):
    newPos = movetype + " [*] X "+XcurPos+" Y "+YcurPos+" Z "+ZcurPos
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))
  elif(movetype == "Start Spline" or movetype == "End Spline"):
    newPos = movetype              
    tab1.progView.insert(selRow, newPos) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))
  elif(movetype == "Teach PR"):
    PR = str(SavePosEntryField.get())
    SPE6 = "Position Register "+PR+" Element 6 = "+RxcurPos         
    tab1.progView.insert(selRow, SPE6)	
    SPE5 = "Position Register "+PR+" Element 5 = "+RycurPos            
    tab1.progView.insert(selRow, SPE5)
    SPE4 = "Position Register "+PR+" Element 4 = "+RzcurPos           
    tab1.progView.insert(selRow, SPE4)	
    SPE3 = "Position Register "+PR+" Element 3 = "+ZcurPos       
    tab1.progView.insert(selRow, SPE3)	
    SPE2 = "Position Register "+PR+" Element 2 = "+YcurPos            
    tab1.progView.insert(selRow, SPE2)	
    SPE1 = "Position Register "+PR+" Element 1 = "+XcurPos         
    tab1.progView.insert(selRow, SPE1)   	
    value=tab1.progView.get(0,END)
    pickle.dump(value,open(ProgEntryField.get(),"wb"))

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
  tab1.progView.select_set(selRow)  
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))  
  
def manInsItem():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow) 
  tab1.progView.insert(selRow, manEntryField.get())
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow) 
  selRow = tab1.progView.curselection()[0]
  curRowEntryField.delete(0, 'end')
  curRowEntryField.insert(0,selRow)
  tab1.progView.itemconfig(selRow, {'fg': 'darkgreen'})
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  
def manReplItem():
  #selRow = curRowEntryField.get()
  selRow = tab1.progView.curselection()[0]
  tab1.progView.delete(selRow) 
  tab1.progView.insert(selRow, manEntryField.get())
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  tab1.progView.itemconfig(selRow, {'fg': 'darkgreen'})  
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  
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
  tab1.progView.insert(selRow, newTime)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow) 
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))


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
  tab1.progView.insert(selRow, newInput)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow) 
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))

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
  tab1.progView.insert(selRow, newInput)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow) 
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))

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
  tab1.progView.insert(selRow, newOutput)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow) 
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))

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
  tab1.progView.insert(selRow, newOutput)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow) 
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))

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
  tab1.progView.insert(selRow, tabins) 
  value=tab1.progView.get(0,END)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  tabNumEntryField.delete(0, 'end')

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
  tab1.progView.insert(selRow, tabjmp) 
  value=tab1.progView.get(0,END)
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  tabNumEntryField.delete(0, 'end')
 
def getvision():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  value = "Get Vision"
  tab1.progView.insert(selRow, value) 
  value=tab1.progView.get(0,END)
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  
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
  tab1.progView.insert(selRow, tabjmp)   
  value=tab1.progView.get(0,END)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  tabNumEntryField.delete(0, 'end')

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
  tab1.progView.insert(selRow, tabjmp) 
  value=tab1.progView.get(0,END)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
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
  tab1.progView.insert(selRow, servoins)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow) 
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))

def loadProg():
  progframe=Frame(tab1)
  progframe.place(x=7,y=174)
  #progframe.pack(side=RIGHT, fill=Y)
  scrollbar = Scrollbar(progframe) 
  scrollbar.pack(side=RIGHT, fill=Y)
  tab1.progView = Listbox(progframe,width=105,height=31, yscrollcommand=scrollbar.set)
  tab1.progView.bind('<<ListboxSelect>>', progViewselect)
  try:
    Prog = pickle.load(open(ProgEntryField.get(),"rb"))
  except:
    try:
      Prog = ['##BEGINNING OF PROGRAM##','Tab Number 1']
      pickle.dump(Prog,open(ProgEntryField.get(),"wb"))    
    except:
      Prog = ['##BEGINNING OF PROGRAM##','Tab Number 1']
      pickle.dump(Prog,open("new","wb"))
      ProgEntryField.insert(0,"new")
  time.sleep(.2)
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
  tab1.progView.insert(selRow, changeProg)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow)  
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))

def insertReturn():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  value = "Return"           
  tab1.progView.insert(selRow, value)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow)  
  value=tab1.progView.get(0,END)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))

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
  tab1.progView.insert(selRow, tabjmp)   
  value=tab1.progView.get(0,END)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  tabNumEntryField.delete(0, 'end')

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
  tab1.progView.insert(selRow, regIns)   
  value=tab1.progView.get(0,END)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  tabNumEntryField.delete(0, 'end')
  
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
  tab1.progView.insert(selRow, regIns)   
  value=tab1.progView.get(0,END)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  tabNumEntryField.delete(0, 'end')
  
def insCalibrate():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  insCal = "Calibrate Robot"          
  tab1.progView.insert(selRow, insCal)   
  value=tab1.progView.get(0,END)
  tab1.progView.selection_clear(0, END) 
  tab1.progView.select_set(selRow)
  pickle.dump(value,open(ProgEntryField.get(),"wb"))
  tabNumEntryField.delete(0, 'end')

def progViewselect(e):
  selRow = tab1.progView.curselection()[0]
  curRowEntryField.delete(0, 'end')
  curRowEntryField.insert(0,selRow)
 
def getSel():
  selRow = tab1.progView.curselection()[0]
  tab1.progView.see(selRow+2)
  data = list(map(int, tab1.progView.curselection()))
  command=tab1.progView.get(data[0])
  manEntryField.delete(0, 'end')
  manEntryField.insert(0, command)  
  
def Servo0on():
  savePosData() 
  servoPos = servo0onEntryField.get()
  command = "SV0P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read()


def Servo0off():
  savePosData() 
  servoPos = servo0offEntryField.get()
  command = "SV0P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read()


def Servo1on():
  savePosData() 
  servoPos = servo1onEntryField.get()
  command = "SV1P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 


def Servo1off():
  savePosData() 
  servoPos = servo1offEntryField.get()
  command = "SV1P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read()
 

def Servo2on():
  savePosData() 
  servoPos = servo2onEntryField.get()
  command = "SV2P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 


def Servo2off():
  savePosData() 
  servoPos = servo2offEntryField.get()
  command = "SV2P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read()

def Servo3on():
  savePosData() 
  servoPos = servo3onEntryField.get()
  command = "SV3P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 

def Servo3off():
  savePosData() 
  servoPos = servo3offEntryField.get()
  command = "SV3P"+servoPos+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read()

def DO1on():
  outputNum = DO1onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 


def DO1off():
  outputNum = DO1offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 
 

def DO2on():
  outputNum = DO2onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read()
 

def DO2off():
  outputNum = DO2offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 


def DO3on():
  outputNum = DO3onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 


def DO3off():
  outputNum = DO3offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 
 

def DO4on():
  outputNum = DO4onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read()
 

def DO4off():
  outputNum = DO4offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 


def DO5on():
  outputNum = DO5onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 


def DO5off():
  outputNum = DO5offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read() 
 

def DO6on():
  outputNum = DO6onEntryField.get()
  command = "ONX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
  ser2.read()
 

def DO6off():
  outputNum = DO6offEntryField.get()
  command = "OFX"+outputNum+"\n"
  ser2.write(command.encode())
  ser2.flushInput()
  time.sleep(.2)
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
  command = "LL"+"A"+str(J1CalStatVal)+"B"+str(J2CalStatVal)+"C"+str(J3CalStatVal)+"D"+str(J4CalStatVal)+"E"+str(J5CalStatVal)+"F"+str(J6CalStatVal)+"G"+str(J1calOff)+"H"+str(J2calOff)+"I"+str(J3calOff)+"J"+str(J4calOff)+"K"+str(J5calOff)+"L"+str(J6calOff)+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "Auto Calibration Stage 1 Successful"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "Auto Calibration Stage 1 Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb")) 
  ##### STAGE 2 ########
  command = "LL"+"A"+str(J1CalStatVal2)+"B"+str(J2CalStatVal2)+"C"+str(J3CalStatVal2)+"D"+str(J4CalStatVal2)+"E"+str(J5CalStatVal2)+"F"+str(J6CalStatVal2)+"G"+str(J1calOff)+"H"+str(J2calOff)+"I"+str(J3calOff)+"J"+str(J4calOff)+"K"+str(J5calOff)+"L"+str(J6calOff)+"\n"
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "Auto Calibration Stage 2 Successful"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "Auto Calibration Stage 2 Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb")) 


def calRobotJ1():
  command = "LLA1B0C0D0E0F0G"+str(J1calOff)+"H"+str(J2calOff)+"I"+str(J3calOff)+"J"+str(J4calOff)+"K"+str(J5calOff)+"L"+str(J6calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J1 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J1 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ2():
  command = "LLA0B1C0D0E0F0G"+str(J1calOff)+"H"+str(J2calOff)+"I"+str(J3calOff)+"J"+str(J4calOff)+"K"+str(J5calOff)+"L"+str(J6calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J2 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J2 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ3():
  command = "LLA0B0C1D0E0F0G"+str(J1calOff)+"H"+str(J2calOff)+"I"+str(J3calOff)+"J"+str(J4calOff)+"K"+str(J5calOff)+"L"+str(J6calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J3 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J3 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ4():
  command = "LLA0B0C0D1E0F0G"+str(J1calOff)+"H"+str(J4calOff)+"I"+str(J3calOff)+"J"+str(J4calOff)+"K"+str(J5calOff)+"L"+str(J6calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J4 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel" ) 
  else:
    message = "J4 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ5():
  command = "LLA0B0C0D0E1F0G"+str(J1calOff)+"H"+str(J4calOff)+"I"+str(J3calOff)+"J"+str(J4calOff)+"K"+str(J5calOff)+"L"+str(J6calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J5 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J5 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ6():
  command = "LLA0B0C0D0E0F1G"+str(J1calOff)+"H"+str(J4calOff)+"I"+str(J3calOff)+"J"+str(J4calOff)+"K"+str(J5calOff)+"L"+str(J6calOff)+"\n"  
  ser.write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.flushInput()
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J6 Calibrated Successfully"
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J6 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     
	


def calRobotMid():
  print ("foo")
  #add mid command

def correctPos():
  command = "CP\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  response = str(ser.readline().strip(),'utf-8')
  displayPosition(response) 

def requestPos():
  command = "RP\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  response = str(ser.readline().strip(),'utf-8')
  displayPosition(response) 

def toolFrame():
  TFx  = TFxEntryField.get()
  TFy  = TFyEntryField.get()
  TFz  = TFzEntryField.get()
  TFrz = TFrzEntryField.get()
  TFry = TFryEntryField.get()
  TFrx = TFrxEntryField.get()
  command = "TF"+"A"+TFx+"B"+TFy+"C"+TFz+"D"+TFrz+"E"+TFry+"F"+TFrx+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  response = ser.read()

def calAxis7():
  TRaxisLimNeg = 0
  TRaxisLimPos = TRlength
  TRnegLimLab.config(text=str(-TRaxisLimNeg), style="Jointlim.TLabel")
  TRposLimLab.config(text=str(TRaxisLimPos), style="Jointlim.TLabel")
  TRjogslide.config(from_=-TRaxisLimNeg, to=TRlength,  length=125, orient=HORIZONTAL,  command=TRsliderUpdate)
  command = "CT"+"A"+str(TRlength)+"B"+str(TRrotation)+"C"+str(TRsteps)+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  response = ser.read()

def zeroAxis7():
  TRaxisLimNeg = 0
  TRaxisLimPos = TRlength
  command = "ZT"+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  response = str(ser.readline().strip(),'utf-8')
  displayPosition(response) 


def sendPos():
  command = "SP"+"A"+str(J1AngCur)+"B"+str(J2AngCur)+"C"+str(J3AngCur)+"D"+str(J4AngCur)+"E"+str(J5AngCur)+"F"+str(J6AngCur)+"G"+str(TRStepCur)+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  response = ser.read()

def CalZeroPos():
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  command = "SPA0B0C0D0E0F0\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  response = ser.read()
  requestPos()
  almStatusLab.config(text="Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Zero", style="Warn.TLabel")
  message = "Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  

def CalRestPos():
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  command = "SPA0B0C-89D0E0F0\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  response = ser.read()
  requestPos()
  almStatusLab.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")
  message = "Calibration Forced to Vertical - this is for commissioning and testing - be careful!"
  tab6.ElogView.insert(END, Curtime+" - "+message)
  value=tab6.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  


def ResetDrives():
  ResetDriveBut = Button(tab1,  text="Reset Drives",   command = ResetDrives)
  ResetDriveBut.place(x=307, y=42)
  command = "RD"+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
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
  global TRStepCur
  global XcurPos
  global YcurPos
  global ZcurPos
  global RxcurPos
  global RycurPos
  global RzcurPos 
  global TrackcurPos
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
  TRStepIndex = response.find('P');
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
  Flag = response[FlagIndex+1:TRStepIndex].strip();
  TRStepCur = float(response[TRStepIndex+1:].strip());
  TrackcurPos = str(round(TRStepCur/(TRsteps/TRrotation),2));
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
  TRcurAngEntryField.delete(0, 'end')
  TRcurAngEntryField.insert(0,TrackcurPos)
  J1jogslide.set(J1AngCur)
  J2jogslide.set(J2AngCur)
  J3jogslide.set(J3AngCur)
  J4jogslide.set(J4AngCur)
  J5jogslide.set(J5AngCur)
  J6jogslide.set(J6AngCur)
  TRjogslide.set(TrackcurPos)
  manEntryField.delete(0, 'end')
  manEntryField.insert(0,Debug)
  savePosData()
  if (Flag != ""):
      ErrorHandler(Flag) 
  if (SpeedVioation=='1'):
      Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
      message = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"
      tab6.ElogView.insert(END, Curtime+" - "+message)
      value=tab6.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))          
      almStatusLab.config(text=message, style="Warn.TLabel")
      almStatusLab2.config(text=message, style="Warn.TLabel")
  

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
  global TrackcurPos
  global TrackLength
  global TrackStepLim
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
  global J1OpenLoopVal
  global J2OpenLoopVal
  global J3OpenLoopVal
  global J4OpenLoopVal
  global J5OpenLoopVal
  global J6OpenLoopVal 
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
  global TRlength
  global TRrotation
  global TRsteps
  global IncJogStat
  TrackcurPos = TRcurAngEntryField.get()
  TrackLength = float(TrackLengthEntryField.get())
  TrackStepLim = float(TrackStepLimEntryField.get())
  VisFileLoc = VisFileLocEntryField.get()
  VisProg = visoptions.get()
  VisOrigXpix = float(VisPicOxPEntryField.get())
  VisOrigXmm  = float(VisPicOxMEntryField.get())
  VisOrigYpix = float(VisPicOyPEntryField.get())
  VisOrigYmm  = float(VisPicOyMEntryField.get())
  VisEndXpix  = float(VisPicXPEntryField.get())
  VisEndXmm   = float(VisPicXMEntryField.get())
  VisEndYpix  = float(VisPicYPEntryField.get())
  VisEndYmm   = float(VisPicYMEntryField.get())
  J1calOff    = float(J1calOffEntryField.get())
  J2calOff    = float(J2calOffEntryField.get())
  J3calOff    = float(J3calOffEntryField.get())
  J4calOff    = float(J4calOffEntryField.get())
  J5calOff    = float(J5calOffEntryField.get())
  J6calOff    = float(J6calOffEntryField.get())
  J1OpenLoopVal = int(J1OpenLoopStat.get())
  J2OpenLoopVal = int(J2OpenLoopStat.get())
  J3OpenLoopVal = int(J3OpenLoopStat.get())
  J4OpenLoopVal = int(J4OpenLoopStat.get())
  J5OpenLoopVal = int(J5OpenLoopStat.get())
  J6OpenLoopVal = int(J6OpenLoopStat.get())
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
  TRlength     = float(axis7lengthEntryField.get())
  TRrotation   = float(axis7rotEntryField.get())
  TRsteps      = float(axis7stepsEntryField.get())
  toolFrame()
  time.sleep(.5)
  calAxis7()
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
  global TRlength
  global TRrotation
  global TRsteps
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
  calibration.insert(END, TRcurAngEntryField.get())
  calibration.insert(END, TrackLengthEntryField.get())
  calibration.insert(END, TrackStepLimEntryField.get())
  calibration.insert(END, VisFileLocEntryField.get())
  calibration.insert(END, visoptions.get())
  calibration.insert(END, VisPicOxPEntryField.get())
  calibration.insert(END, VisPicOxMEntryField.get())
  calibration.insert(END, VisPicOyPEntryField.get())
  calibration.insert(END, VisPicOyMEntryField.get())
  calibration.insert(END, VisPicXPEntryField.get())
  calibration.insert(END, VisPicXMEntryField.get())
  calibration.insert(END, VisPicYPEntryField.get())
  calibration.insert(END, VisPicYMEntryField.get())
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
  calibration.insert(END, TRlength)
  calibration.insert(END, TRrotation)
  calibration.insert(END, TRsteps)
  calibration.insert(END, TRStepCur)
  calibration.insert(END, J1CalStatVal2)
  calibration.insert(END, J2CalStatVal2)
  calibration.insert(END, J3CalStatVal2)
  calibration.insert(END, J4CalStatVal2)
  calibration.insert(END, J5CalStatVal2)
  calibration.insert(END, J6CalStatVal2)
  
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
  ##AXIS LIMIT ERROR
  if (response[1:2] == 'L'):
    if (response[2:] == '1000000'):
      message = "J1 Axis Limit"
      tab6.ElogView.insert(END, Curtime+" - "+message)
      value=tab6.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[2:] == '0100000'):
      message = "J2 Axis Limit"
      tab6.ElogView.insert(END, Curtime+" - "+message)
      value=tab6.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[2:] == '0010000'):
      message = "J3 Axis Limit"
      tab6.ElogView.insert(END, Curtime+" - "+message)
      value=tab6.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[2:] == '0001000'):
      message = "J4 Axis Limit"
      tab6.ElogView.insert(END, Curtime+" - "+message)
      value=tab6.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[2:] == '0000100'):
      message = "J5 Axis Limit"
      tab6.ElogView.insert(END, Curtime+" - "+message)
      value=tab6.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[2:] == '0000010'):
      message = "J6 Axis Limit"
      tab6.ElogView.insert(END, Curtime+" - "+message)
      value=tab6.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    if (response[2:] == '0000001'):
      message = "Track Axis Limit"
      tab6.ElogView.insert(END, Curtime+" - "+message)
      value=tab6.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))  
    cmdRecEntryField.delete(0, 'end')
    cmdRecEntryField.insert(0,response)            
    message = "Axis Limit Error - See Log"
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    stopProg()
  ##COLLISION ERROR   
  elif (response[1:2] == 'C'):
    if (J1OpenLoopStat.get() == 0):
      if (response[2:3] == '1'):
        message = "J1 Collision or Motor Error"
        tab6.ElogView.insert(END, Curtime+" - "+message)
        value=tab6.ElogView.get(0,END)
        pickle.dump(value,open("ErrorLog","wb"))
        correctPos()
        stopProg()
        message = "Collision or Motor Error - See Log"
        almStatusLab.config(text=message, style="Alarm.TLabel")
        almStatusLab2.config(text=message, style="Alarm.TLabel")
    if (J2OpenLoopStat.get() == 0):    
      if (response[3:4] == '1'):
        message = "J2 Collision or Motor Error"
        tab6.ElogView.insert(END, Curtime+" - "+message)
        value=tab6.ElogView.get(0,END)
        pickle.dump(value,open("ErrorLog","wb"))
        correctPos()
        stopProg()
        message = "Collision or Motor Error - See Log"
        almStatusLab.config(text=message, style="Alarm.TLabel")
        almStatusLab2.config(text=message, style="Alarm.TLabel")
    if (J3OpenLoopStat.get() == 0):
      if (response[4:5] == '1'):
        message = "J3 Collision or Motor Error"
        tab6.ElogView.insert(END, Curtime+" - "+message)
        value=tab6.ElogView.get(0,END)
        pickle.dump(value,open("ErrorLog","wb"))
        correctPos()
        stopProg()
        message = "Collision or Motor Error - See Log"
        almStatusLab.config(text=message, style="Alarm.TLabel")
        almStatusLab2.config(text=message, style="Alarm.TLabel")
    if (J4OpenLoopStat.get() == 0):
      if (response[5:6] == '1'):
        message = "J4 Collision or Motor Error"
        tab6.ElogView.insert(END, Curtime+" - "+message)
        value=tab6.ElogView.get(0,END)
        pickle.dump(value,open("ErrorLog","wb"))
        correctPos()
        stopProg()
        message = "Collision or Motor Error - See Log"
        almStatusLab.config(text=message, style="Alarm.TLabel")
        almStatusLab2.config(text=message, style="Alarm.TLabel")
    if (J5OpenLoopStat.get() == 0):
      if (response[6:7] == '1'):
        message = "J5 Collision or Motor Error"
        tab6.ElogView.insert(END, Curtime+" - "+message)
        value=tab6.ElogView.get(0,END)
        pickle.dump(value,open("ErrorLog","wb"))
        correctPos()
        stopProg()
        message = "Collision or Motor Error - See Log"
        almStatusLab.config(text=message, style="Alarm.TLabel")
        almStatusLab2.config(text=message, style="Alarm.TLabel")
    if (J6OpenLoopStat.get() == 0):
      if (response[7:8] == '1'):
        message = "J6 Collision or Motor Error"
        tab6.ElogView.insert(END, Curtime+" - "+message)
        value=tab6.ElogView.get(0,END)
        pickle.dump(value,open("ErrorLog","wb"))  
        correctPos()
        stopProg()        
        message = "Collision or Motor Error - See Log"
        almStatusLab.config(text=message, style="Alarm.TLabel")
        almStatusLab2.config(text=message, style="Alarm.TLabel")
    #ResetDriveBut = Button(tab1,  text="Reset Drives",   command = ResetDrives, style="AlarmBut.TButton")
    #ResetDriveBut.place(x=307, y=42) 
    ##REACH ERROR   
  elif (response[1:2] == 'R'):
    stopProg()
    message = "Position Out of Reach"
    tab6.ElogView.insert(END, Curtime+" - "+message)
    value=tab6.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
     
  elif (response[1:2] == 'S'):  
    stopProg()
    message = "Spline Can Only Have Move L Types"
    tab6.ElogView.insert(END, Curtime+" - "+message)
    value=tab6.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
     
  else:
    stopProg() 
    message = "Unknown Error"
    tab6.ElogView.insert(END, Curtime+" - "+message)
    value=tab6.ElogView.get(0,END)
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

    
  

def viscalc(x,y):
  global VisOrigXpix
  global VisOrigXmm
  global VisOrigYpix
  global VisOrigYmm
  global VisEndXpix
  global VisEndXmm
  global VisEndYpix
  global VisEndYmm
  global Xpos
  global Ypos
  XPrange = float(VisEndXpix) - float(VisOrigXpix)
  XPratio = (x-float(VisOrigXpix)) / XPrange
  XMrange = float(VisEndXmm) - float(VisOrigXmm)
  XMpos = float(XMrange) * float(XPratio)
  Xpos = float(VisOrigXmm) + XMpos
  ##
  YPrange = float(VisEndYpix) - float(VisOrigYpix)
  YPratio = (y-float(VisOrigYpix)) / YPrange
  YMrange = float(VisEndYmm) - float(VisOrigYmm)
  YMpos = float(YMrange) * float(YPratio)
  Ypos = float(VisOrigYmm) + YMpos
  return (Xpos,Ypos)


def posRegFieldVisible(self):
  curCmdtype = options.get()
  if (curCmdtype=="Move PR" or curCmdtype=="OFF PR " or curCmdtype=="Teach PR"):
    SavePosEntryField.place(x=800, y=183)
  else:
    SavePosEntryField.place_forget()




 
 
  
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

TrLab = Label(CartjogFrame, font=("Arial", 18), text = "Trx")
TrLab.place(x=1110, y=265)
















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
J1negLimLab = Label(J1jogFrame, font=("Arial", 8), text = str(-J1axisLimNeg), style="Jointlim.TLabel")
J1negLimLab.place(x=115, y=25)
J1posLimLab = Label(J1jogFrame, font=("Arial", 8), text = str(J1axisLimPos), style="Jointlim.TLabel")
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
J1jogslide = Scale(J1jogFrame, from_=-J1axisLimNeg, to=J1axisLimPos,  length=180, orient=HORIZONTAL,  command=J1sliderUpdate)
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
J2negLimLab = Label(J2jogFrame, font=("Arial", 8), text = str(-J2axisLimNeg), style="Jointlim.TLabel")
J2negLimLab.place(x=115, y=25)
J2posLimLab = Label(J2jogFrame, font=("Arial", 8), text = str(J2axisLimPos), style="Jointlim.TLabel")
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
J2jogslide = Scale(J2jogFrame, from_=-J2axisLimNeg, to=J2axisLimPos,  length=180, orient=HORIZONTAL,  command=J2sliderUpdate)
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
J3negLimLab = Label(J3jogFrame, font=("Arial", 8), text = str(-J3axisLimNeg), style="Jointlim.TLabel")
J3negLimLab.place(x=115, y=25)
J3posLimLab = Label(J3jogFrame, font=("Arial", 8), text = str(J3axisLimPos), style="Jointlim.TLabel")
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
J3jogslide = Scale(J3jogFrame, from_=-J3axisLimNeg, to=J3axisLimPos,  length=180, orient=HORIZONTAL,  command=J3sliderUpdate)
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
J4negLimLab = Label(J4jogFrame, font=("Arial", 8), text = str(-J4axisLimNeg), style="Jointlim.TLabel")
J4negLimLab.place(x=115, y=25)
J4posLimLab = Label(J4jogFrame, font=("Arial", 8), text = str(J4axisLimPos), style="Jointlim.TLabel")
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
J4jogslide = Scale(J4jogFrame, from_=-J4axisLimNeg, to=J4axisLimPos,  length=180, orient=HORIZONTAL,  command=J4sliderUpdate)
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
J5negLimLab = Label(J5jogFrame, font=("Arial", 8), text = str(-J5axisLimNeg), style="Jointlim.TLabel")
J5negLimLab.place(x=115, y=25)
J5posLimLab = Label(J5jogFrame, font=("Arial", 8), text = str(J5axisLimPos), style="Jointlim.TLabel")
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
J5jogslide = Scale(J5jogFrame, from_=-J5axisLimNeg, to=J5axisLimPos,  length=180, orient=HORIZONTAL,  command=J5sliderUpdate)
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
J6negLimLab = Label(J6jogFrame, font=("Arial", 8), text = str(-J6axisLimNeg), style="Jointlim.TLabel")
J6negLimLab.place(x=115, y=25)
J6posLimLab = Label(J6jogFrame, font=("Arial", 8), text = str(J6axisLimPos), style="Jointlim.TLabel")
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
J6jogslide = Scale(J6jogFrame, from_=-J6axisLimNeg, to=J6axisLimPos,  length=180, orient=HORIZONTAL,  command=J6sliderUpdate)
J6jogslide.bind("<ButtonRelease-1>", J6sliderExecute)
J6jogslide.place(x=115, y=7)

TRjogFrame = Frame(tab1, width=145, height=115)
TRjogFrame['relief'] = 'raised'
TRjogFrame.place(x=1340, y=350)
TRLab = Label(TRjogFrame, font=("Arial", 14), text = "7th Axis")
TRLab.place(x=15, y=5)
TRcurAngEntryField = Entry(TRjogFrame,width=5)
TRcurAngEntryField.place(x=90, y=9)
TRjogNegBut = Button(TRjogFrame,  text="-", width=3, command = lambda: TrackjogNeg(float(incrementEntryField.get())))
TRjogNegBut.place(x=10, y=65, width=30, height=25)
TRjogPosBut = Button(TRjogFrame, text="+",  width=3, command = lambda: TrackjogPos(float(incrementEntryField.get())))
TRjogPosBut.place(x=105, y=65, width=30, height=25)
TRstilljogNegBut = Button(TRjogFrame,  text="", width=3, command = lambda: TRstilljogNeg(float(incrementEntryField.get())))
TRstilljogNegBut.place(x=10, y=95, width=30, height=12)
TRstilljogPosBut = Button(TRjogFrame, text="",  width=3, command = lambda: TRstilljogPos(float(incrementEntryField.get())))
TRstilljogPosBut.place(x=105, y=95, width=30, height=12)
TRnegLimLab = Label(TRjogFrame, font=("Arial", 8), text = str(-TRaxisLimNeg), style="Jointlim.TLabel")
TRnegLimLab.place(x=10, y=30)
TRposLimLab = Label(TRjogFrame, font=("Arial", 8), text = str(TRaxisLimPos), style="Jointlim.TLabel")
TRposLimLab.place(x=110, y=30)
TRslidelabel = Label(TRjogFrame)
TRslidelabel.place(x=60, y=70)
def TRsliderUpdate(foo):
  TRslidelabel.config(text=round(float(TRjogslide.get()),2))   
def TRsliderExecute(foo): 
  TRdelta = float(TRjogslide.get()) - float(TRcurAngEntryField.get())
  if (TRdelta < 0):
    TrackjogNeg(abs(TRdelta))
  else:
    TrackjogPos(abs(TRdelta))       
TRjogslide = Scale(TRjogFrame, from_=-TRaxisLimNeg, to=TRaxisLimPos,  length=125, orient=HORIZONTAL,  command=TRsliderUpdate)
TRjogslide.bind("<ButtonRelease-1>", TRsliderExecute)
TRjogslide.place(x=10, y=43)


####ENTRY FIELDS##########################################################
##########################################################################

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

manInsBut = Button(tab1, text="  Insert  ",  command = manInsItem)
manInsBut.place(x=98, y=725)

manRepBut = Button(tab1,  text="Replace",  command = manReplItem)
manRepBut.place(x=164, y=725)

getSelBut = Button(tab1,  text="Get Selected",  command = getSel)
getSelBut.place(x=10, y=725)

speedOption=StringVar(tab1)
speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
speedMenu.place(x=412, y=76)



#single buttons

options=StringVar(tab1)
menu=OptionMenu(tab1, options, "Move J", "Move J", "OFF J", "Move L", "Move R", "Move A Mid", "Move A End", "Move C Center", "Move C Start", "Move C Plane", "Start Spline", "End Spline", "Move PR", "OFF PR ", "Teach PR", command=posRegFieldVisible)
menu.grid(row=2,column=2)
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

getVisBut = Button(tab1,  text="Get Vision",  width=22, command = getvision)
getVisBut.place(x=700, y=380)

#buttons with 1 entry

waitTimeBut = Button(tab1, text="Wait Time (seconds)",  width=22,  command = waitTime)
waitTimeBut.place(x=700, y=420)

waitInputOnBut = Button(tab1, text="Wait Input ON",  width=22,  command = waitInputOn)
waitInputOnBut.place(x=700, y=460)

waitInputOffBut = Button(tab1,  text="Wait Input OFF",  width=22,  command = waitInputOff)
waitInputOffBut.place(x=700, y=500)

setOutputOnBut = Button(tab1,  text="Set Output On",  width=22,  command = setOutputOn)
setOutputOnBut.place(x=700, y=540)

setOutputOffBut = Button(tab1,  text="Set Output OFF",  width=22,   command = setOutputOff)
setOutputOffBut.place(x=700, y=580)

tabNumBut = Button(tab1,  text="Create Tab",  width=22, command = tabNumber)
tabNumBut.place(x=700, y=620)

jumpTabBut = Button(tab1,  text="Jump to Tab",  width=22, command = jumpTab)
jumpTabBut.place(x=700, y=660)




waitTimeEntryField = Entry(tab1,width=5)
waitTimeEntryField.place(x=855, y=425)

waitInputEntryField = Entry(tab1,width=5)
waitInputEntryField.place(x=855, y=465)

waitInputOffEntryField = Entry(tab1,width=5)
waitInputOffEntryField.place(x=855, y=505)

outputOnEntryField = Entry(tab1,width=5)
outputOnEntryField.place(x=855, y=545)

outputOffEntryField = Entry(tab1,width=5)
outputOffEntryField.place(x=855, y=585)

tabNumEntryField = Entry(tab1,width=5)
tabNumEntryField.place(x=855, y=625)

jumpTabEntryField = Entry(tab1,width=5)
jumpTabEntryField.place(x=855, y=665)






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

storPosEqLab = Label(tab1,font=("Arial", 6), text = " StorPos      Element       (++/--)")
storPosEqLab.place(x=1107, y=549)








ProgBut = Button(tab1,  text="Load Program",   command = loadProg)
ProgBut.place(x=202, y=42)

ResetDriveBut = Button(tab1,  text="Reset Drives",   command = ResetDrives)
#ResetDriveBut.place(x=307, y=42)



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


ToolFrameLab = Label(tab2, text = "Tool Frame Offset")
ToolFrameLab.place(x=970, y=60)

UFxLab = Label(tab2, font=("Arial", 11), text = "X")
UFxLab.place(x=920, y=90)

UFyLab = Label(tab2, font=("Arial", 11), text = "Y")
UFyLab.place(x=960, y=90)

UFzLab = Label(tab2, font=("Arial", 11), text = "Z")
UFzLab.place(x=1000, y=90)

UFRxLab = Label(tab2, font=("Arial", 11), text = "Rz")
UFRxLab.place(x=1040, y=90)

UFRyLab = Label(tab2, font=("Arial", 11), text = "Ry")
UFRyLab.place(x=1080, y=90)

UFRzLab = Label(tab2, font=("Arial", 11), text = "Rx")
UFRzLab.place(x=1120, y=90)

comLab = Label(tab2, text = "Communication")
comLab.place(x=72, y=60)

jointCalLab = Label(tab2, text = "Joint Calibration")
jointCalLab.place(x=290, y=60)

axis7Lab = Label(tab2, text = "7th Axis Calibration")
axis7Lab.place(x=65, y=300)

axis7lengthLab = Label(tab2, text = "7th Axis Length:")
axis7lengthLab.place(x=51, y=340)

axis7rotLab = Label(tab2, text = "MM per Rotation:")
axis7rotLab.place(x=45, y=370)

axis7stepsLab = Label(tab2, text = "Drive Steps:")
axis7stepsLab.place(x=75, y=400)



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

CalibrationOffsetsLab = Label(tab2, text = "Encoder Control")
CalibrationOffsetsLab.place(x=715, y=60)

cmdSentLab = Label(tab2, text = "Last Command Sent to Controller")
cmdSentLab.place(x=10, y=565)

cmdRecLab = Label(tab2, text = "Last Response From Controller")
cmdRecLab.place(x=10, y=625)

ToolFrameLab = Label(tab2, text = "Theme")
ToolFrameLab.place(x=1225, y=60)


### 2 BUTTONS################################################################
#############################################################################

comPortBut = Button(tab2,  text="  Set Com Teensy  ",   command = setCom)
comPortBut.place(x=85, y=110)

comPortBut2 = Button(tab2,  text="Set Com IO Board",   command = setCom2)
comPortBut2.place(x=85, y=180)


lightBut = Button(tab2,  text="  Light  ",  command = lightTheme)
lightBut.place(x=1190, y=90)

darkBut = Button(tab2,  text="  Dark   ",  command = darkTheme)
darkBut.place(x=1250, y=90)


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





TRzerobut = Button(tab2, text=" Set Axis 7 Calibration to Zero ",    command = zeroAxis7)
TRzerobut.place(x=40, y=440)

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

CalJ5But = Button(tab2,   text="Calibrate J6 Only",   command = calRobotJ6)
CalJ5But.place(x=285, y=390)

CalZeroBut = Button(tab2,   text="Force Cal. to 0° Home",  width=20,   command = CalZeroPos)
CalZeroBut.place(x=270, y=425)

CalRestBut = Button(tab2,   text="Force Cal. to Vert. Rest",  width=20,   command = CalRestPos)
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

axis7lengthEntryField = Entry(tab2,width=6)
axis7lengthEntryField.place(x=150, y=340)

axis7rotEntryField = Entry(tab2,width=6)
axis7rotEntryField.place(x=150, y=370)

axis7stepsEntryField = Entry(tab2,width=6)
axis7stepsEntryField.place(x=150, y=400)



   ### Tool Frame ###

TFxEntryField = Entry(tab2,width=5)
TFxEntryField.place(x=910, y=115)
TFyEntryField = Entry(tab2,width=5)
TFyEntryField.place(x=950, y=115)
TFzEntryField = Entry(tab2,width=5)
TFzEntryField.place(x=990, y=115)
TFrzEntryField = Entry(tab2,width=5)
TFrzEntryField.place(x=1030, y=115)
TFryEntryField = Entry(tab2,width=5)
TFryEntryField.place(x=1070, y=115)
TFrxEntryField = Entry(tab2,width=5)
TFrxEntryField.place(x=1110, y=115)




TrackLengthEntryField = Entry(tab2,width=8)
#TrackLengthEntryField.place(x=380, y=630)


TrackStepLimEntryField = Entry(tab2,width=8)
#TrackStepLimEntryField.place(x=380, y=655)



####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 3



### 3 LABELS#################################################################
#############################################################################

servo0onequalsLab = Label(tab3, text = "=")
servo0onequalsLab.place(x=70, y=12)

servo0offequalsLab = Label(tab3, text = "=")
servo0offequalsLab.place(x=70, y=52)

servo1onequalsLab = Label(tab3, text = "=")
servo1onequalsLab.place(x=70, y=92)

servo1offequalsLab = Label(tab3, text = "=")
servo1offequalsLab.place(x=70, y=132)

servo2onequalsLab = Label(tab3, text = "=")
servo2onequalsLab.place(x=70, y=172)

servo2offequalsLab = Label(tab3, text = "=")
servo2offequalsLab.place(x=70, y=212)

servo3onequalsLab = Label(tab3, text = "=")
servo3onequalsLab.place(x=70, y=252)

servo3offequalsLab = Label(tab3, text = "=")
servo3offequalsLab.place(x=70, y=292)



Do1onequalsLab = Label(tab3, text = "=")
Do1onequalsLab.place(x=210, y=12)

Do1offequalsLab = Label(tab3, text = "=")
Do1offequalsLab.place(x=210, y=52)

Do2onequalsLab = Label(tab3, text = "=")
Do2onequalsLab.place(x=210, y=92)

Do2offequalsLab = Label(tab3, text = "=")
Do2offequalsLab.place(x=210, y=132)

Do3onequalsLab = Label(tab3, text = "=")
Do3onequalsLab.place(x=210, y=172)

Do3offequalsLab = Label(tab3, text = "=")
Do3offequalsLab.place(x=210, y=212)

Do4onequalsLab = Label(tab3, text = "=")
Do4onequalsLab.place(x=210, y=252)

Do4offequalsLab = Label(tab3, text = "=")
Do4offequalsLab.place(x=210, y=292)

Do5onequalsLab = Label(tab3, text = "=")
Do5onequalsLab.place(x=210, y=332)

Do5offequalsLab = Label(tab3, text = "=")
Do5offequalsLab.place(x=210, y=372)

Do6onequalsLab = Label(tab3, text = "=")
Do6onequalsLab.place(x=210, y=412)

Do6offequalsLab = Label(tab3, text = "=")
Do6offequalsLab.place(x=210, y=452)


inoutavailLab = Label(tab3, text = "NOTE: the following are available when using the default Nano board for IO:   Inputs = 2-7  /  Outputs = 8-13  /  Servos = A0-A7")
inoutavailLab.place(x=10, y=640)

inoutavailLab = Label(tab3, text = "If using IO on Teensy board:  Inputs = 32-36  /  Outputs = 37-41 - if using IO on Teensy you must manually change the command from 'Out On =' to 'ToutOn ='")
inoutavailLab.place(x=10, y=655)


### 3 BUTTONS################################################################
#############################################################################

servo0onBut = Button(tab3,  text="Servo 0",  command = Servo0on)
servo0onBut.place(x=10, y=10)

servo0offBut = Button(tab3,  text="Servo 0",  command = Servo0off)
servo0offBut.place(x=10, y=50)

servo1onBut = Button(tab3,  text="Servo 1",  command = Servo1on)
servo1onBut.place(x=10, y=90)

servo1offBut = Button(tab3,  text="Servo 1",  command = Servo1off)
servo1offBut.place(x=10, y=130)

servo2onBut = Button(tab3,  text="Servo 2",  command = Servo2on)
servo2onBut.place(x=10, y=170)

servo2offBut = Button(tab3,  text="Servo 2",  command = Servo2off)
servo2offBut.place(x=10, y=210)

servo3onBut = Button(tab3,  text="Servo 3",  command = Servo3on)
servo3onBut.place(x=10, y=250)

servo3offBut = Button(tab3,  text="Servo 3",  command = Servo3off)
servo3offBut.place(x=10, y=290)





DO1onBut = Button(tab3,  text="DO on",  command = DO1on)
DO1onBut.place(x=150, y=10)

DO1offBut = Button(tab3,  text="DO off",  command = DO1off)
DO1offBut.place(x=150, y=50)

DO2onBut = Button(tab3,  text="DO on",  command = DO2on)
DO2onBut.place(x=150, y=90)

DO2offBut = Button(tab3,  text="DO off",  command = DO2off)
DO2offBut.place(x=150, y=130)

DO3onBut = Button(tab3,  text="DO on",  command = DO3on)
DO3onBut.place(x=150, y=170)

DO3offBut = Button(tab3,  text="DO off",  command = DO3off)
DO3offBut.place(x=150, y=210)

DO4onBut = Button(tab3,  text="DO on",  command = DO4on)
DO4onBut.place(x=150, y=250)

DO4offBut = Button(tab3,  text="DO off",  command = DO4off)
DO4offBut.place(x=150, y=290)

DO5onBut = Button(tab3,  text="DO on",  command = DO5on)
DO5onBut.place(x=150, y=330)

DO5offBut = Button(tab3,  text="DO off",  command = DO5off)
DO5offBut.place(x=150, y=370)

DO6onBut = Button(tab3,  text="DO on",  command = DO6on)
DO6onBut.place(x=150, y=410)

DO6offBut = Button(tab3,  text="DO off",  command = DO6off)
DO6offBut.place(x=150, y=450)



#### 3 ENTRY FIELDS##########################################################
#############################################################################


servo0onEntryField = Entry(tab3,width=5)
servo0onEntryField.place(x=90, y=15)

servo0offEntryField = Entry(tab3,width=5)
servo0offEntryField.place(x=90, y=55)

servo1onEntryField = Entry(tab3,width=5)
servo1onEntryField.place(x=90, y=95)

servo1offEntryField = Entry(tab3,width=5)
servo1offEntryField.place(x=90, y=135)

servo2onEntryField = Entry(tab3,width=5)
servo2onEntryField.place(x=90, y=175)

servo2offEntryField = Entry(tab3,width=5)
servo2offEntryField.place(x=90, y=215)


servo3onEntryField = Entry(tab3,width=5)
servo3onEntryField.place(x=90, y=255)

servo3offEntryField = Entry(tab3,width=5)
servo3offEntryField.place(x=90, y=295)





DO1onEntryField = Entry(tab3,width=5)
DO1onEntryField.place(x=230, y=15)

DO1offEntryField = Entry(tab3,width=5)
DO1offEntryField.place(x=230, y=55)

DO2onEntryField = Entry(tab3,width=5)
DO2onEntryField.place(x=230, y=95)

DO2offEntryField = Entry(tab3,width=5)
DO2offEntryField.place(x=230, y=135)

DO3onEntryField = Entry(tab3,width=5)
DO3onEntryField.place(x=230, y=175)

DO3offEntryField = Entry(tab3,width=5)
DO3offEntryField.place(x=230, y=215)

DO4onEntryField = Entry(tab3,width=5)
DO4onEntryField.place(x=230, y=255)

DO4offEntryField = Entry(tab3,width=5)
DO4offEntryField.place(x=230, y=295)

DO5onEntryField = Entry(tab3,width=5)
DO5onEntryField.place(x=230, y=335)

DO5offEntryField = Entry(tab3,width=5)
DO5offEntryField.place(x=230, y=375)

DO6onEntryField = Entry(tab3,width=5)
DO6onEntryField.place(x=230, y=415)

DO6offEntryField = Entry(tab3,width=5)
DO6offEntryField.place(x=230, y=455)



####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 4




### 4 LABELS#################################################################
#############################################################################

R1Lab = Label(tab4, text = "R1")
R1Lab.place(x=70, y=30)

R2Lab = Label(tab4, text = "R2")
R2Lab.place(x=70, y=60)

R3Lab = Label(tab4, text = "R3")
R3Lab.place(x=70, y=90)

R4Lab = Label(tab4, text = "R4")
R4Lab.place(x=70, y=120)

R5Lab = Label(tab4, text = "R5")
R5Lab.place(x=70, y=150)

R6Lab = Label(tab4, text = "R6")
R6Lab.place(x=70, y=180)

R7Lab = Label(tab4, text = "R7")
R7Lab.place(x=70, y=210)

R8Lab = Label(tab4, text = "R8")
R8Lab.place(x=70, y=240)

R9Lab = Label(tab4, text = "R9")
R9Lab.place(x=70, y=270)

R10Lab = Label(tab4, text = "R10")
R10Lab.place(x=70, y=300)

R11Lab = Label(tab4, text = "R11")
R11Lab.place(x=70, y=330)

R12Lab = Label(tab4, text = "R12")
R12Lab.place(x=70, y=360)

R13Lab = Label(tab4, text = "R14")
R13Lab.place(x=70, y=390)

R14Lab = Label(tab4, text = "R14")
R14Lab.place(x=70, y=420)

R15Lab = Label(tab4, text = "R15")
R15Lab.place(x=70, y=450)

R16Lab = Label(tab4, text = "R16")
R16Lab.place(x=70, y=480)


SP1Lab = Label(tab4, text = "PR1 (vision)")
SP1Lab.place(x=640, y=30)

SP2Lab = Label(tab4, text = "PR2")
SP2Lab.place(x=640, y=60)

SP3Lab = Label(tab4, text = "PR3")
SP3Lab.place(x=640, y=90)

SP4Lab = Label(tab4, text = "PR4")
SP4Lab.place(x=640, y=120)

SP5Lab = Label(tab4, text = "PR5")
SP5Lab.place(x=640, y=150)

SP6Lab = Label(tab4, text = "PR6")
SP6Lab.place(x=640, y=180)

SP7Lab = Label(tab4, text = "PR7")
SP7Lab.place(x=640, y=210)

SP8Lab = Label(tab4, text = "PR8")
SP8Lab.place(x=640, y=240)

SP9Lab = Label(tab4, text = "PR9")
SP9Lab.place(x=640, y=270)

SP10Lab = Label(tab4, text = "PR10")
SP10Lab.place(x=640, y=300)

SP11Lab = Label(tab4, text = "PR11")
SP11Lab.place(x=640, y=330)

SP12Lab = Label(tab4, text = "PR12")
SP12Lab.place(x=640, y=360)

SP13Lab = Label(tab4, text = "PR14")
SP13Lab.place(x=640, y=390)

SP14Lab = Label(tab4, text = "PR14")
SP14Lab.place(x=640, y=420)

SP15Lab = Label(tab4, text = "PR15")
SP15Lab.place(x=640, y=450)

SP16Lab = Label(tab4, text = "PR16")
SP16Lab.place(x=640, y=480)


SP_E1_Lab = Label(tab4, text = "X")
SP_E1_Lab.place(x=410, y=10)

SP_E2_Lab = Label(tab4, text = "Y")
SP_E2_Lab.place(x=450, y=10)

SP_E3_Lab = Label(tab4, text = "Z")
SP_E3_Lab.place(x=490, y=10)

SP_E4_Lab = Label(tab4, text = "Rz")
SP_E4_Lab.place(x=530, y=10)

SP_E5_Lab = Label(tab4, text = "Ry")
SP_E5_Lab.place(x=570, y=10)

SP_E6_Lab = Label(tab4, text = "Rx")
SP_E6_Lab.place(x=610, y=10)



### 4 BUTTONS################################################################
#############################################################################




#### 4 ENTRY FIELDS##########################################################
#############################################################################

R1EntryField = Entry(tab4,width=5)
R1EntryField.place(x=30, y=30)

R2EntryField = Entry(tab4,width=5)
R2EntryField.place(x=30, y=60)

R3EntryField = Entry(tab4,width=5)
R3EntryField.place(x=30, y=90)

R4EntryField = Entry(tab4,width=5)
R4EntryField.place(x=30, y=120)

R5EntryField = Entry(tab4,width=5)
R5EntryField.place(x=30, y=150)

R6EntryField = Entry(tab4,width=5)
R6EntryField.place(x=30, y=180)

R7EntryField = Entry(tab4,width=5)
R7EntryField.place(x=30, y=210)

R8EntryField = Entry(tab4,width=5)
R8EntryField.place(x=30, y=240)

R9EntryField = Entry(tab4,width=5)
R9EntryField.place(x=30, y=270)

R10EntryField = Entry(tab4,width=5)
R10EntryField.place(x=30, y=300)

R11EntryField = Entry(tab4,width=5)
R11EntryField.place(x=30, y=330)

R12EntryField = Entry(tab4,width=5)
R12EntryField.place(x=30, y=360)

R13EntryField = Entry(tab4,width=5)
R13EntryField.place(x=30, y=390)

R14EntryField = Entry(tab4,width=5)
R14EntryField.place(x=30, y=420)

R15EntryField = Entry(tab4,width=5)
R15EntryField.place(x=30, y=450)

R16EntryField = Entry(tab4,width=5)
R16EntryField.place(x=30, y=480)




SP_1_E1_EntryField = Entry(tab4,width=5)
SP_1_E1_EntryField.place(x=400, y=30)

SP_2_E1_EntryField = Entry(tab4,width=5)
SP_2_E1_EntryField.place(x=400, y=60)

SP_3_E1_EntryField = Entry(tab4,width=5)
SP_3_E1_EntryField.place(x=400, y=90)

SP_4_E1_EntryField = Entry(tab4,width=5)
SP_4_E1_EntryField.place(x=400, y=120)

SP_5_E1_EntryField = Entry(tab4,width=5)
SP_5_E1_EntryField.place(x=400, y=150)

SP_6_E1_EntryField = Entry(tab4,width=5)
SP_6_E1_EntryField.place(x=400, y=180)

SP_7_E1_EntryField = Entry(tab4,width=5)
SP_7_E1_EntryField.place(x=400, y=210)

SP_8_E1_EntryField = Entry(tab4,width=5)
SP_8_E1_EntryField.place(x=400, y=240)

SP_9_E1_EntryField = Entry(tab4,width=5)
SP_9_E1_EntryField.place(x=400, y=270)

SP_10_E1_EntryField = Entry(tab4,width=5)
SP_10_E1_EntryField.place(x=400, y=300)

SP_11_E1_EntryField = Entry(tab4,width=5)
SP_11_E1_EntryField.place(x=400, y=330)

SP_12_E1_EntryField = Entry(tab4,width=5)
SP_12_E1_EntryField.place(x=400, y=360)

SP_13_E1_EntryField = Entry(tab4,width=5)
SP_13_E1_EntryField.place(x=400, y=390)

SP_14_E1_EntryField = Entry(tab4,width=5)
SP_14_E1_EntryField.place(x=400, y=420)

SP_15_E1_EntryField = Entry(tab4,width=5)
SP_15_E1_EntryField.place(x=400, y=450)

SP_16_E1_EntryField = Entry(tab4,width=5)
SP_16_E1_EntryField.place(x=400, y=480)





SP_1_E2_EntryField = Entry(tab4,width=5)
SP_1_E2_EntryField.place(x=440, y=30)

SP_2_E2_EntryField = Entry(tab4,width=5)
SP_2_E2_EntryField.place(x=440, y=60)

SP_3_E2_EntryField = Entry(tab4,width=5)
SP_3_E2_EntryField.place(x=440, y=90)

SP_4_E2_EntryField = Entry(tab4,width=5)
SP_4_E2_EntryField.place(x=440, y=120)

SP_5_E2_EntryField = Entry(tab4,width=5)
SP_5_E2_EntryField.place(x=440, y=150)

SP_6_E2_EntryField = Entry(tab4,width=5)
SP_6_E2_EntryField.place(x=440, y=180)

SP_7_E2_EntryField = Entry(tab4,width=5)
SP_7_E2_EntryField.place(x=440, y=210)

SP_8_E2_EntryField = Entry(tab4,width=5)
SP_8_E2_EntryField.place(x=440, y=240)

SP_9_E2_EntryField = Entry(tab4,width=5)
SP_9_E2_EntryField.place(x=440, y=270)

SP_10_E2_EntryField = Entry(tab4,width=5)
SP_10_E2_EntryField.place(x=440, y=300)

SP_11_E2_EntryField = Entry(tab4,width=5)
SP_11_E2_EntryField.place(x=440, y=330)

SP_12_E2_EntryField = Entry(tab4,width=5)
SP_12_E2_EntryField.place(x=440, y=360)

SP_13_E2_EntryField = Entry(tab4,width=5)
SP_13_E2_EntryField.place(x=440, y=390)

SP_14_E2_EntryField = Entry(tab4,width=5)
SP_14_E2_EntryField.place(x=440, y=420)

SP_15_E2_EntryField = Entry(tab4,width=5)
SP_15_E2_EntryField.place(x=440, y=450)

SP_16_E2_EntryField = Entry(tab4,width=5)
SP_16_E2_EntryField.place(x=440, y=480)




SP_1_E3_EntryField = Entry(tab4,width=5)
SP_1_E3_EntryField.place(x=480, y=30)

SP_2_E3_EntryField = Entry(tab4,width=5)
SP_2_E3_EntryField.place(x=480, y=60)

SP_3_E3_EntryField = Entry(tab4,width=5)
SP_3_E3_EntryField.place(x=480, y=90)

SP_4_E3_EntryField = Entry(tab4,width=5)
SP_4_E3_EntryField.place(x=480, y=120)

SP_5_E3_EntryField = Entry(tab4,width=5)
SP_5_E3_EntryField.place(x=480, y=150)

SP_6_E3_EntryField = Entry(tab4,width=5)
SP_6_E3_EntryField.place(x=480, y=180)

SP_7_E3_EntryField = Entry(tab4,width=5)
SP_7_E3_EntryField.place(x=480, y=210)

SP_8_E3_EntryField = Entry(tab4,width=5)
SP_8_E3_EntryField.place(x=480, y=240)

SP_9_E3_EntryField = Entry(tab4,width=5)
SP_9_E3_EntryField.place(x=480, y=270)

SP_10_E3_EntryField = Entry(tab4,width=5)
SP_10_E3_EntryField.place(x=480, y=300)

SP_11_E3_EntryField = Entry(tab4,width=5)
SP_11_E3_EntryField.place(x=480, y=330)

SP_12_E3_EntryField = Entry(tab4,width=5)
SP_12_E3_EntryField.place(x=480, y=360)

SP_13_E3_EntryField = Entry(tab4,width=5)
SP_13_E3_EntryField.place(x=480, y=390)

SP_14_E3_EntryField = Entry(tab4,width=5)
SP_14_E3_EntryField.place(x=480, y=420)

SP_15_E3_EntryField = Entry(tab4,width=5)
SP_15_E3_EntryField.place(x=480, y=450)

SP_16_E3_EntryField = Entry(tab4,width=5)
SP_16_E3_EntryField.place(x=480, y=480)




SP_1_E4_EntryField = Entry(tab4,width=5)
SP_1_E4_EntryField.place(x=520, y=30)

SP_2_E4_EntryField = Entry(tab4,width=5)
SP_2_E4_EntryField.place(x=520, y=60)

SP_3_E4_EntryField = Entry(tab4,width=5)
SP_3_E4_EntryField.place(x=520, y=90)

SP_4_E4_EntryField = Entry(tab4,width=5)
SP_4_E4_EntryField.place(x=520, y=120)

SP_5_E4_EntryField = Entry(tab4,width=5)
SP_5_E4_EntryField.place(x=520, y=150)

SP_6_E4_EntryField = Entry(tab4,width=5)
SP_6_E4_EntryField.place(x=520, y=180)

SP_7_E4_EntryField = Entry(tab4,width=5)
SP_7_E4_EntryField.place(x=520, y=210)

SP_8_E4_EntryField = Entry(tab4,width=5)
SP_8_E4_EntryField.place(x=520, y=240)

SP_9_E4_EntryField = Entry(tab4,width=5)
SP_9_E4_EntryField.place(x=520, y=270)

SP_10_E4_EntryField = Entry(tab4,width=5)
SP_10_E4_EntryField.place(x=520, y=300)

SP_11_E4_EntryField = Entry(tab4,width=5)
SP_11_E4_EntryField.place(x=520, y=330)

SP_12_E4_EntryField = Entry(tab4,width=5)
SP_12_E4_EntryField.place(x=520, y=360)

SP_13_E4_EntryField = Entry(tab4,width=5)
SP_13_E4_EntryField.place(x=520, y=390)

SP_14_E4_EntryField = Entry(tab4,width=5)
SP_14_E4_EntryField.place(x=520, y=420)

SP_15_E4_EntryField = Entry(tab4,width=5)
SP_15_E4_EntryField.place(x=520, y=450)

SP_16_E4_EntryField = Entry(tab4,width=5)
SP_16_E4_EntryField.place(x=520, y=480)

SP_1_E5_EntryField = Entry(tab4,width=5)
SP_1_E5_EntryField.place(x=560, y=30)

SP_2_E5_EntryField = Entry(tab4,width=5)
SP_2_E5_EntryField.place(x=560, y=60)

SP_3_E5_EntryField = Entry(tab4,width=5)
SP_3_E5_EntryField.place(x=560, y=90)

SP_4_E5_EntryField = Entry(tab4,width=5)
SP_4_E5_EntryField.place(x=560, y=120)

SP_5_E5_EntryField = Entry(tab4,width=5)
SP_5_E5_EntryField.place(x=560, y=150)

SP_6_E5_EntryField = Entry(tab4,width=5)
SP_6_E5_EntryField.place(x=560, y=180)

SP_7_E5_EntryField = Entry(tab4,width=5)
SP_7_E5_EntryField.place(x=560, y=210)

SP_8_E5_EntryField = Entry(tab4,width=5)
SP_8_E5_EntryField.place(x=560, y=240)

SP_9_E5_EntryField = Entry(tab4,width=5)
SP_9_E5_EntryField.place(x=560, y=270)

SP_10_E5_EntryField = Entry(tab4,width=5)
SP_10_E5_EntryField.place(x=560, y=300)

SP_11_E5_EntryField = Entry(tab4,width=5)
SP_11_E5_EntryField.place(x=560, y=330)

SP_12_E5_EntryField = Entry(tab4,width=5)
SP_12_E5_EntryField.place(x=560, y=360)

SP_13_E5_EntryField = Entry(tab4,width=5)
SP_13_E5_EntryField.place(x=560, y=390)

SP_14_E5_EntryField = Entry(tab4,width=5)
SP_14_E5_EntryField.place(x=560, y=420)

SP_15_E5_EntryField = Entry(tab4,width=5)
SP_15_E5_EntryField.place(x=560, y=450)

SP_16_E5_EntryField = Entry(tab4,width=5)
SP_16_E5_EntryField.place(x=560, y=480)




SP_1_E6_EntryField = Entry(tab4,width=5)
SP_1_E6_EntryField.place(x=600, y=30)

SP_2_E6_EntryField = Entry(tab4,width=5)
SP_2_E6_EntryField.place(x=600, y=60)

SP_3_E6_EntryField = Entry(tab4,width=5)
SP_3_E6_EntryField.place(x=600, y=90)

SP_4_E6_EntryField = Entry(tab4,width=5)
SP_4_E6_EntryField.place(x=600, y=120)

SP_5_E6_EntryField = Entry(tab4,width=5)
SP_5_E6_EntryField.place(x=600, y=150)

SP_6_E6_EntryField = Entry(tab4,width=5)
SP_6_E6_EntryField.place(x=600, y=180)

SP_7_E6_EntryField = Entry(tab4,width=5)
SP_7_E6_EntryField.place(x=600, y=210)

SP_8_E6_EntryField = Entry(tab4,width=5)
SP_8_E6_EntryField.place(x=600, y=240)

SP_9_E6_EntryField = Entry(tab4,width=5)
SP_9_E6_EntryField.place(x=600, y=270)

SP_10_E6_EntryField = Entry(tab4,width=5)
SP_10_E6_EntryField.place(x=600, y=300)

SP_11_E6_EntryField = Entry(tab4,width=5)
SP_11_E6_EntryField.place(x=600, y=330)

SP_12_E6_EntryField = Entry(tab4,width=5)
SP_12_E6_EntryField.place(x=600, y=360)

SP_13_E6_EntryField = Entry(tab4,width=5)
SP_13_E6_EntryField.place(x=600, y=390)

SP_14_E6_EntryField = Entry(tab4,width=5)
SP_14_E6_EntryField.place(x=600, y=420)

SP_15_E6_EntryField = Entry(tab4,width=5)
SP_15_E6_EntryField.place(x=600, y=450)

SP_16_E6_EntryField = Entry(tab4,width=5)
SP_16_E6_EntryField.place(x=600, y=480)











####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 5




### 5 LABELS#################################################################
#############################################################################

VisFileLocLab = Label(tab5, text = "Vision File Location:")
VisFileLocLab.place(x=10, y=12)

VisCalPixLab = Label(tab5, text = "Calibration Pixels:")
VisCalPixLab.place(x=10, y=75)

VisCalmmLab = Label(tab5, text = "Calibration Robot MM:")
VisCalmmLab.place(x=10, y=105)

VisCalOxLab = Label(tab5, text = "Orig: X")
VisCalOxLab.place(x=150, y=42)

VisCalOyLab = Label(tab5, text = "Orig: Y")
VisCalOyLab.place(x=210, y=42)

VisCalXLab = Label(tab5, text = "End: X")
VisCalXLab.place(x=270, y=42)

VisCalYLab = Label(tab5, text = "End: Y")
VisCalYLab.place(x=330, y=42)



VisInTypeLab = Label(tab5, text = "Choose Vision Format")
VisInTypeLab.place(x=500, y=38)

VisXfoundLab = Label(tab5, text = "X found position (mm)")
VisXfoundLab.place(x=540, y=100)

VisYfoundLab = Label(tab5, text = "Y found position (mm)")
VisYfoundLab.place(x=540, y=130)

VisRZfoundLab = Label(tab5, text = "R found position (ang)")
VisRZfoundLab.place(x=540, y=160)

VisXpixfoundLab = Label(tab5, text = "X pixes returned from camera")
VisXpixfoundLab.place(x=760, y=100)

VisYpixfoundLab = Label(tab5, text = "Y pixes returned from camera")
VisYpixfoundLab.place(x=760, y=130)

### 5 BUTTONS################################################################
#############################################################################

visoptions=StringVar(tab5)
menu=OptionMenu(tab5, visoptions, "Openvision", "Openvision", "Roborealm 1.7.5", "x,y,r")
menu.grid(row=2,column=2)
menu.place(x=500, y=60)


testvisBut = Button(tab5,  text="test",  width=15, command = testvis)
testvisBut.place(x=500, y=190)

saveCalBut = Button(tab5,  text="SAVE VISION DATA",  width=26, command = SaveAndApplyCalibration)
saveCalBut.place(x=1150, y=630)


#### 5 ENTRY FIELDS##########################################################
#############################################################################

VisFileLocEntryField = Entry(tab5,width=70)
VisFileLocEntryField.place(x=125, y=12)

VisPicOxPEntryField = Entry(tab5,width=5)
VisPicOxPEntryField.place(x=155, y=75)

VisPicOxMEntryField = Entry(tab5,width=5)
VisPicOxMEntryField.place(x=155, y=105)

VisPicOyPEntryField = Entry(tab5,width=5)
VisPicOyPEntryField.place(x=215, y=75)

VisPicOyMEntryField = Entry(tab5,width=5)
VisPicOyMEntryField.place(x=215, y=105)

VisPicXPEntryField = Entry(tab5,width=5)
VisPicXPEntryField.place(x=275, y=75)

VisPicXMEntryField = Entry(tab5,width=5)
VisPicXMEntryField.place(x=275, y=105)

VisPicYPEntryField = Entry(tab5,width=5)
VisPicYPEntryField.place(x=335, y=75)

VisPicYMEntryField = Entry(tab5,width=5)
VisPicYMEntryField.place(x=335, y=105)

VisXfindEntryField = Entry(tab5,width=5)
VisXfindEntryField.place(x=500, y=100)

VisYfindEntryField = Entry(tab5,width=5)
VisYfindEntryField.place(x=500, y=130)

VisRZfindEntryField = Entry(tab5,width=5)
VisRZfindEntryField.place(x=500, y=160)

VisXpixfindEntryField = Entry(tab5,width=5)
VisXpixfindEntryField.place(x=720, y=100)

VisYpixfindEntryField = Entry(tab5,width=5)
VisYpixfindEntryField.place(x=720, y=130)


####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 6

Elogframe=Frame(tab6)
Elogframe.place(x=40,y=15)
scrollbar = Scrollbar(Elogframe) 
scrollbar.pack(side=RIGHT, fill=Y)
tab6.ElogView = Listbox(Elogframe,width=150,height=40, yscrollcommand=scrollbar.set)
try:
  Elog = pickle.load(open("ErrorLog","rb"))
except:
  Elog = ['##BEGINNING OF LOG##']
  pickle.dump(Elog,open("ErrorLog","wb"))
time.sleep(.2)
for item in Elog:
  tab6.ElogView.insert(END,item) 
tab6.ElogView.pack()
scrollbar.config(command=tab6.ElogView.yview)

def clearLog():
 tab6.ElogView.delete(1,END)
 value=tab6.ElogView.get(0,END)
 pickle.dump(value,open("ErrorLog","wb"))

clearLogBut = Button(tab6,  text="Clear Log",  width=26, command = clearLog)
clearLogBut.place(x=1000, y=630)




####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 7

link = Label(tab7, font='12', text="https://www.anninrobotics.com/tutorials",  cursor="hand2")
link.bind("<Button-1>", lambda event: webbrowser.open(link.cget("text")))
link.place(x=10, y=9)

def callback():
    webbrowser.open_new(r"https://www.paypal.me/ChrisAnnin")

donateBut = Button(tab7,  command = callback)
donatePhoto=PhotoImage(file="pp.gif")
donateBut.config(image=donatePhoto)
donateBut.place(x=1250, y=2)


scroll = Scrollbar(tab7)
scroll.pack(side=RIGHT, fill=Y)
configfile = Text(tab7, wrap=WORD, width=166, height=40, yscrollcommand=scroll.set)
filename='information.txt'
with open(filename, 'r', encoding='utf-8-sig') as file:
  configfile.insert(INSERT, file.read())
configfile.pack(side="left")
scroll.config(command=configfile.yview)
configfile.place(x=10, y=40)


####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 10

### 10 LABELS#################################################################
#############################################################################

testSendLab = Label(tab10, text = "Test string to send to arduino")
testSendLab.place(x=10, y=20)

testRecLab = Label(tab10, text = "Message echoed back from arduino")
testRecLab.place(x=10, y=70)

### 10 BUTTONS################################################################
#############################################################################

testSendBut = Button(tab10,  text="SEND TO ARDUINO",   command = TestString)
testSendBut.place(x=10, y=120)

testClearBut = Button(tab10,  text="CLEAR RECEIVED",   command = ClearTestString)
testClearBut.place(x=180, y=120)

#### 10 ENTRY FIELDS##########################################################
#############################################################################

testSendEntryField = Entry(tab10,width=222)
testSendEntryField.place(x=10, y=40)

testRecEntryField = Entry(tab10,width=222)
testRecEntryField.place(x=10, y=90)



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
TrackcurPos =calibration.get("28")
TrackLength =calibration.get("29")
TrackStepLim=calibration.get("30")
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
TRlength    = calibration.get("61")
TRrotation  = calibration.get("62")
TRsteps     = calibration.get("63")
TRStepCur   = calibration.get("64")
J1CalStatVal2= calibration.get("65")
J2CalStatVal2= calibration.get("66")
J3CalStatVal2= calibration.get("67")
J4CalStatVal2= calibration.get("68")
J5CalStatVal2= calibration.get("69")
J6CalStatVal2= calibration.get("70")


####

comPortEntryField.insert(0,str(comPort))
com2PortEntryField.insert(0,str(com2Port))
incrementEntryField.insert(0,"10")
speedEntryField.insert(0,"25")
ACCspeedField.insert(0,"10")
DECspeedField.insert(0,"10")
ACCrampField.insert(0,"100")
roundEntryField.insert(0,"0")
ProgEntryField.insert(0,(Prog))
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
TRcurAngEntryField.insert(0,str(TrackcurPos))
TrackLengthEntryField.insert(0,str(TrackLength))
TrackStepLimEntryField.insert(0,str(TrackStepLim))
VisFileLocEntryField.insert(0,str(VisFileLoc))
visoptions.set(VisProg)
VisPicOxPEntryField.insert(0,str(VisOrigXpix))
VisPicOxMEntryField.insert(0,str(VisOrigXmm))
VisPicOyPEntryField.insert(0,str(VisOrigYpix))
VisPicOyMEntryField.insert(0,str(VisOrigYmm))
VisPicXPEntryField.insert(0,str(VisEndXpix))
VisPicXMEntryField.insert(0,str(VisEndXmm))
VisPicYPEntryField.insert(0,str(VisEndYpix))
VisPicYMEntryField.insert(0,str(VisEndYmm))
J1calOffEntryField.insert(0,str(J1calOff))
J2calOffEntryField.insert(0,str(J2calOff))
J3calOffEntryField.insert(0,str(J3calOff))
J4calOffEntryField.insert(0,str(J4calOff))
J5calOffEntryField.insert(0,str(J5calOff))
J6calOffEntryField.insert(0,str(J6calOff))
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
axis7lengthEntryField.insert(0,str(TRlength))
axis7rotEntryField.insert(0,str(TRrotation))
axis7stepsEntryField.insert(0,str(TRsteps))



setCom()
setCom2()


loadProg()

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