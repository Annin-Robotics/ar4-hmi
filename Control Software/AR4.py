#!/usr/bin/env python

############################################################################
## Version AR4 6.3 #########################################################
############################################################################
""" AR4 - robot control software
    Copyright (c) 2024, Chris Annin
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
  VERSION 4.3 1/21/24 Gcode to SD card.  Estop button interrupt.
  VERSION 4.3.1 2/1/24 bug fix - vision snap and find drop down
  VERSION 4.4 3/2/24 added kinematic error handling
  VERSION 4.4 6/29/24 simplified drive motors functions with arrays
  VERSION 5.0 7/14/24 updating kinematics
  VERSION 5.1 1/22/25 bug fix stopping after calibration from CMD window / added Modbus RS-485
  VERSION 5.2 3/23/25 add auto calibrate for individual axis
  VERSION 6.0 6/12/25 add virtual robot
  VERSION 6.1 8/29/25 updated accel and decel, auto calibrate & microsteps
  VERSION 6.2 9/12/25 changed bootstrap theme, xbox upgrade
  VERSION 6.2.1 9/24/25 fixed slider position update
  VERSION 6.3 10/8/25 - JK - changed COM entry to dropdown, added beta Linux support, added basic config module
'''
##########################################################################
##########################################################################

import logging
from multiprocessing.resource_sharer import stop
from os import execv
from tkinter import *
# Import ttkbootstrap widgets to replace ttk widgets
import ttkbootstrap as ttk_bootstrap
from ttkbootstrap import Style as BootstrapStyle
from ttkbootstrap import *  # This makes ttkbootstrap widgets available globally
from tkinter import ttk  # Keep for compatibility
from tkinter import simpledialog
from tkinter import messagebox
from PIL import Image, ImageTk
from matplotlib import pyplot as plt

from tkinter import filedialog as fd
from functools import partial
from vtkmodules.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor
import vtkmodules.vtkInteractionStyle as vtkIS
from queue import Queue
import ctypes

import sys
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
import vtk
import re
from numpy import mean
from os import path
import tkinter as tk
from tkinter import ttk
from threading import Lock
from threading import Thread
import robot_kinematics as robot

#####################################################################################
# Cross-Compat Patch
# We need platform awareness, port enumeration, and some typing imports

from pathlib import Path
import platform
from serial.tools import list_ports
from typing import List

from modules.ar_config import AR_Configuration

global Config, CE
Config = AR_Configuration()
CE = Config.Environment

if CE['Platform']['IS_WINDOWS']:
  from pygrabber.dshow_graph import FilterGraph

'''
class FilterGraph:
    """Minimal replacement for pygrabber.dshow_graph.FilterGraph (enumeration only).
    for cross-platform support and minimal re-write"""

    def __init__(self):
        # Pick a backend that works well per-OS
        sysname = platform.system()
        if sysname == "Windows":
            self._backend = cv2.CAP_DSHOW  # or cv2.CAP_MSMF
        elif sysname == "Linux":
            self._backend = cv2.CAP_V4L2
        else:
            self._backend = 0

    def get_input_devices(self, max_scan: int = 10) -> List[str]:
        """Return a list of camera *names* (best-effort)."""
        names: List[str] = []

        # On Linux, expand scan range based on /dev/video*
        if platform.system() == "Linux":
            try:
                devs = [
                    int(d[5:]) for d in os.listdir("/dev")
                    if d.startswith("video") and d[5:].isdigit()
                ]
                if devs:
                    max_scan = max(max_scan, max(devs) + 1)
            except Exception:
                pass

        for idx in range(max_scan):
            cap = cv2.VideoCapture(idx, self._backend)
            ok = cap.isOpened()
            cap.release()
            if not ok:
                continue

            # Default label
            name = f"Camera {idx}"
            names.append(name)

        return names
'''
################################################################################################
## Logging Configuration
logger = logging.getLogger("AR4_HMI:Main")
logger.setLevel(logging.DEBUG)

# Add console handler
console = logging.StreamHandler(sys.stdout)
console.setFormatter(logging.Formatter("%(name)s: %(asctime)s [%(levelname)s] %(message)s"))
logger.addHandler(console)

# Function to log to Pane 8
def pane8_log(message):
    if tab8 and hasattr(tab8, "ElogView"):
      Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
      try:
          # Schedule insertion on Tkinter main thread
          tab8.ElogView.after(0, lambda: tab8.ElogView.insert(END, f"{Curtime} - {message}"))
      except tk.TclError:
          pass  # widget likely gone

# Setup Pane8 as a logging handler and log there
from modules.ar_logging import CustomOutputHandler

pane8_handler = CustomOutputHandler(pane8_log)
pane8_handler.setFormatter(logging.Formatter("%(levelname)s - %(message)s"))
logger.addHandler(pane8_handler)

################################################################################################

robot.robot_set()


DIR = pathlib.Path(__file__).parent.resolve()
os.chdir(DIR)

cropping = False


root = Tk()
root.wm_title("AR4 Software Ver 6.3")

#####################################################################################
# Cross-Compat Patch
# Linux doesn't like iconbitmap so we will use iconphoto which requires a png

#root.iconbitmap(r'AR.ico')
root.iconphoto(True, tk.PhotoImage(file="AR.png"))
#####################################################################################

root.resizable(width=False, height=False)
root.geometry('1536x792+0+0')
root.runTrue = 0
root.GCrunTrue = 0

def on_closing():
  cv2.destroyAllWindows()
  root.quit()
  root.update()
  root.destroy()
        
root.wm_protocol("WM_DELETE_WINDOW", on_closing)



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
global J7CalStat
J7CalStat = IntVar()
global J8CalStat
J8CalStat = IntVar()
global J9CalStat
J9CalStat = IntVar()

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
global J7CalStat2
J7CalStat2 = IntVar()
global J8CalStat2
J8CalStat2 = IntVar()
global J9CalStat2
J9CalStat2 = IntVar()

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

global estopActive
estopActive = False

global posOutreach
posOutreach = False

global SplineTrue
SplineTrue = False

global gcodeSpeed
gcodeSpeed = "10"

global inchTrue
inchTrue = False

global moveInProc
moveInProc = 0

global liveJog
liveJog = False

global progRunning
progRunning = False

offlineMode = False

global setColor
global renderer

color_map = {}

J1StepM = None
J2StepM = None
J3StepM = None
J4StepM = None
J5StepM = None
J6StepM = None

oriImage = None
DHparams = None
StepMonitors = [0] * 6
rndSpeed = 0
estopActive = False
minSpeedDelay = 200  # µs
speedViolation = "0"
mainMode = 1

live_jog_lock = threading.Lock()
live_cartesian_lock = threading.Lock()
live_tool_lock = threading.Lock()
drive_lock = threading.Lock()
serial_lock = threading.Lock()



# Robot constants and placeholders (you should replace these with actual values)
ROBOT_nDOFs = 6
SolutionMatrix = np.zeros((6, 2))
joints_estimate = np.zeros(6)
xyzuvw_In = np.zeros(6)
KinematicError = 0

# Tool and base frame placeholders (4x4 matrices)
Robot_BaseFrame = np.eye(4)
Robot_ToolFrame = np.eye(4)
Robot_Data = np.zeros(66)  # Replace with actual DK values


#declare axis limit vars
J1PosLim = 0
J1NegLim = 0
J2PosLim = 0
J2NegLim = 0
J3PosLim = 0
J3NegLim = 0
J4PosLim = 0
J4NegLim = 0
J5PosLim = 0
J5NegLim = 0
J6PosLim = 0
J6NegLim = 0
J7PosLim = 0
J7NegLim = 0
J8PosLim = 0
J8NegLim = 0
J9PosLim = 0
J9NegLim = 0



############################################################################
### DEFINE TABS ############################################################
############################################################################

nb = ttk_bootstrap.Notebook(root, width=1536, height=792)
nb.place(x=0, y=0)

tab1 = ttk_bootstrap.Frame(nb)
nb.add(tab1, text=' Main Controls ')

tab2 = ttk_bootstrap.Frame(nb)
nb.add(tab2, text='  Config Settings  ')

tab3 = ttk_bootstrap.Frame(nb)
nb.add(tab3, text='   Kinematics    ')

tab4 = ttk_bootstrap.Frame(nb)
nb.add(tab4, text=' Inputs Outputs ')

tab5 = ttk_bootstrap.Frame(nb)
nb.add(tab5, text='   Registers    ')

tab6 = ttk_bootstrap.Frame(nb)
nb.add(tab6, text='   Vision    ')

tab7 = ttk_bootstrap.Frame(nb)
nb.add(tab7, text='    G-Code     ')

tab8 = ttk_bootstrap.Frame(nb)
nb.add(tab8, text='      Log      ')

tab9 = ttk_bootstrap.Frame(nb)
#nb.add(tab9, text='   Info    ')


cam_on = False
cap = None

#############################################################################################
### KINEMATICS FOR VIR ROBOT ################################################################
#############################################################################################

#DEG2RAD = np.pi / 180
#RAD2DEG = 180 / np.pi

def update_CPP_kin_from_entries():
    global DHparams
    global Robot_Data
    global J1PosLim
    global J1NegLim
    global J2PosLim
    global J2NegLim
    global J3PosLim
    global J3NegLim
    global J4PosLim
    global J4NegLim
    global J5PosLim
    global J5NegLim
    global J6PosLim
    global J6NegLim

    try:

        robot.set_dh_parameters_explicit(
          # θ (radians)
          np.radians(float(J1ΘEntryField.get())), np.radians(float(J2ΘEntryField.get())), np.radians(float(J3ΘEntryField.get())),
          np.radians(float(J4ΘEntryField.get())), np.radians(float(J5ΘEntryField.get())), np.radians(float(J6ΘEntryField.get())),

          # α (radians)
          np.radians(float(J1αEntryField.get())), np.radians(float(J2αEntryField.get())), np.radians(float(J3αEntryField.get())),
          np.radians(float(J4αEntryField.get())), np.radians(float(J5αEntryField.get())), np.radians(float(J6αEntryField.get())),

          # a (mm)
          float(J1aEntryField.get()), float(J2aEntryField.get()), float(J3aEntryField.get()),
          float(J4aEntryField.get()), float(J5aEntryField.get()), float(J6aEntryField.get()),

          # d (mm)
          float(J1dEntryField.get()), float(J2dEntryField.get()), float(J3dEntryField.get()),
          float(J4dEntryField.get()), float(J5dEntryField.get()), float(J6dEntryField.get())
      )
        
        PosLimits = [float(val) for val in [J1PosLim, J2PosLim, J3PosLim, J4PosLim, J5PosLim, J6PosLim]]
        NegLimits = [float(val) for val in [J1NegLim, J2NegLim, J3NegLim, J4NegLim, J5NegLim, J6NegLim]]
        robot.set_joint_limits(PosLimits, NegLimits)
        robot.set_robot_tool_frame(float(TFxEntryField.get()), 
                                   float(TFyEntryField.get()), 
                                   float(TFzEntryField.get()), 
                                   float(TFrxEntryField.get()), 
                                   float(TFryEntryField.get()), 
                                   float(TFrzEntryField.get()))
          

    except ValueError as e:
        logger.error(f"Invalid parameter input: {e}")
        return None 


def setStepMonitorsVR():
    global StepMonitors
    global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    global VR_angles
    StepMonitors[0] = (float(VR_angles[0]) + float(J1NegLim)) * float(J1StepDeg)
    StepMonitors[1] = (float(VR_angles[1]) + float(J2NegLim)) * float(J2StepDeg)
    StepMonitors[2] = (float(VR_angles[2]) + float(J3NegLim)) * float(J3StepDeg)
    StepMonitors[3] = (float(VR_angles[3]) + float(J4NegLim)) * float(J4StepDeg)                                              
    StepMonitors[4] = (float(VR_angles[4]) + float(J5NegLim)) * float(J5StepDeg)
    StepMonitors[5] = (float(VR_angles[5]) + float(J6NegLim)) * float(J6StepDeg)
    J1StepM = StepMonitors[0]
    J2StepM = StepMonitors[1]
    J3StepM = StepMonitors[2]
    J4StepM = StepMonitors[3]
    J5StepM = StepMonitors[4]
    J6StepM = StepMonitors[5]

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
def refresh_gui_from_joint_angles(joint_angles):
    global XcurPos, YcurPos, ZcurPos, RxcurPos, RycurPos, RzcurPos
    global J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur, WC
    setStepMonitorsVR()
    # Forward kinematics to get XYZ + orientation
    try:
        fk_xyzuvw = robot.forward_kinematics(joint_angles)
        xyzuvw = fk_xyzuvw[:3] + [math.degrees(v) for v in fk_xyzuvw[3:]]
    except Exception as e:
        logger.error(f"Forward kinematics failed: {e}")
        return

    # Cast and unpack as strings with 3 decimal places
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [f"{v:.3f}" for v in xyzuvw]

    # Update cartesian position fields
    XcurEntryField.delete(0, 'end'); XcurEntryField.insert(0, XcurPos)
    YcurEntryField.delete(0, 'end'); YcurEntryField.insert(0, YcurPos)
    ZcurEntryField.delete(0, 'end'); ZcurEntryField.insert(0, ZcurPos)
    RzcurEntryField.delete(0, 'end'); RzcurEntryField.insert(0, RzcurPos)
    RycurEntryField.delete(0, 'end'); RycurEntryField.insert(0, RycurPos)
    RxcurEntryField.delete(0, 'end'); RxcurEntryField.insert(0, RxcurPos)

    # Update jog sliders
    J1jogslide.set(joint_angles[0])
    J2jogslide.set(joint_angles[1])
    J3jogslide.set(joint_angles[2])
    J4jogslide.set(joint_angles[3])
    J5jogslide.set(joint_angles[4])
    J6jogslide.set(joint_angles[5])



    J1AngCur = str(joint_angles[0])
    J2AngCur = str(joint_angles[1])
    J3AngCur = str(joint_angles[2])
    J4AngCur = str(joint_angles[3])
    J5AngCur = str(joint_angles[4])
    J6AngCur = str(joint_angles[5])

    if J5AngCur != '' and float(J5AngCur) > 0:
      WC = "F"
    else:
      WC = "N"

    logger.info(J5AngCur)  

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

    J1jogslide.set(J1AngCur)
    J2jogslide.set(J2AngCur)
    J3jogslide.set(J3AngCur)
    J4jogslide.set(J4AngCur)
    J5jogslide.set(J5AngCur)
    J6jogslide.set(J6AngCur)

    # Update joint angle fields
    J1curAngEntryField.delete(0, 'end'); J1curAngEntryField.insert(0, J1AngCur)
    J2curAngEntryField.delete(0, 'end'); J2curAngEntryField.insert(0, J2AngCur)
    J3curAngEntryField.delete(0, 'end'); J3curAngEntryField.insert(0, J3AngCur)
    J4curAngEntryField.delete(0, 'end'); J4curAngEntryField.insert(0, J4AngCur)
    J5curAngEntryField.delete(0, 'end'); J5curAngEntryField.insert(0, J5AngCur)
    J6curAngEntryField.delete(0, 'end'); J6curAngEntryField.insert(0, J6AngCur)










#############################################################################################
### MOVE LOGIC FOR VIRTUAL ROBOT ############################################################
#############################################################################################


def start_driveMotorsJ_thread(*args):
    if drive_lock.locked():
        logger.info("Drive already in progress — ignoring new command.")
        return
    t = threading.Thread(target=run_driveMotorsJ_safe, args=args, daemon=True)
    t.start()

def run_driveMotorsJ_safe(*args):
    with drive_lock:
        driveMotorsJ(*args)    


def driveMotorsJ(
    J1step, J2step, J3step, J4step, J5step, J6step,
    J1dir, J2dir, J3dir, J4dir, J5dir, J6dir,
    SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp):

    
    global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    global xyzuvw_In, xyzuvw_Out, stepDeg, negLim, VR_angles

    limits = robot.get_joint_limits()

    steps = [int(round(J1step)), int(round(J2step)), int(round(J3step)),
             int(round(J4step)), int(round(J5step)), int(round(J6step))]
    dirs = [J1dir, J2dir, J3dir, J4dir, J5dir, J6dir]
    StepMonitors = [J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM]
    prev_StepMonitors = StepMonitors.copy()

    cur = [0] * 6
    PE = [0] * 6
    SE_1 = [0] * 6
    SE_2 = [0] * 6
    LO_1 = [0] * 6
    LO_2 = [0] * 6
    PEcur = [0] * 6
    SE_1cur = [0] * 6
    SE_2cur = [0] * 6

    HighStep = max(steps)
    time.sleep(15e-6)


    if live_cartesian_lock.locked():
        virOffset = 4.7
    elif live_tool_lock.locked():
        virOffset = 5.1
    else:
        virOffset = 4.5    

    # Keep your existing virOffset scaling
    SpeedVal = SpeedVal * virOffset
    #ACCspd   = ACCspd   * virOffset
    #DCCspd   = DCCspd   * virOffset
    #ACCramp  = ACCramp  * virOffset

    # Steps in each region
    ACCStep = HighStep * (ACCspd / 100.0)
    DCCStep = HighStep * (DCCspd / 100.0)
    NORStep = HighStep - ACCStep - DCCStep

    # Target total time in microseconds (no 1.2 fudge)
    speedSP = 0.0
    if SpeedType == "s":
        speedSP = SpeedVal * 1_000_000.0
    elif SpeedType == "m" and xyzuvw_In and xyzuvw_Out:
        dx = xyzuvw_In[0] - xyzuvw_Out[0]
        dy = xyzuvw_In[1] - xyzuvw_Out[1]
        dz = xyzuvw_In[2] - xyzuvw_Out[2]
        lineDist = math.sqrt(dx*dx + dy*dy + dz*dz)
        # seconds = distance / (mm/s)
        speedSP = (lineDist / SpeedVal) * 1_000_000.0

    # fixed ramp factors (start/end slower than cruise), same as Teensy:
    # if(ACCramp < 10){ ACCramp = 10; }  k_* = ACCramp / 10
    if ACCramp < 10.0:
        ACCramp = 10.0
    k_acc = ACCramp / 10.0
    k_dec = ACCramp / 10.0

    # Solve cruise delay for trapezoid
    if SpeedType in ("s", "m") and speedSP > 0.0:
        # T = cruise * [ NORStep + (ACCStep*(1+k_acc) + DCCStep*(1+k_dec))/2 ]
        denom = NORStep + 0.5 * (ACCStep * (1.0 + k_acc) + DCCStep * (1.0 + k_dec))
        if denom <= 0.0:
            calcStepGap = speedSP / max(float(HighStep), 1.0)
        else:
            calcStepGap = speedSP / denom

        if calcStepGap < minSpeedDelay:
            calcStepGap = minSpeedDelay
            try:
                speedViolation = "1"
            except NameError:
                pass  # only if your Python sim doesn't use this flag
    elif SpeedType == "p":
        calcStepGap = minSpeedDelay / (SpeedVal / 100.0)
    else:
        calcStepGap = minSpeedDelay

    # With cruise known, define start/end delays and per-step increments
    startDelay = calcStepGap * k_acc  # slower than cruise
    endDelay   = calcStepGap * k_dec  # slower than cruise

    # Linear ramps
    calcACCstepInc = ((startDelay - calcStepGap) / ACCStep) if ACCStep > 0.0 else 0.0  # subtract each accel step
    calcDCCstepInc = ((endDelay   - calcStepGap) / DCCStep) if DCCStep > 0.0 else 0.0  # add each decel step

    # Start delay
    calcACCstartDel = startDelay
    curDelay = calcACCstartDel
    highStepCur = 0


    while any(cur[i] < steps[i] for i in range(6)):
        

        if highStepCur <= ACCStep:
            curDelay -= calcACCstepInc
        elif highStepCur >= (HighStep - DCCStep):
            curDelay += calcDCCstepInc
        else:
            curDelay = calcStepGap

        distDelay = 30
        disDelayCur = 0

        for i in range(6):
            if cur[i] < steps[i]:
                PE[i] = HighStep // steps[i]
                LO_1[i] = HighStep - (steps[i] * PE[i])
                SE_1[i] = (HighStep // LO_1[i]) if LO_1[i] > 0 else 0
                LO_2[i] = (HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) // SE_1[i]))) if SE_1[i] > 0 else 0
                SE_2[i] = (HighStep // LO_2[i]) if LO_2[i] > 0 else 0

                if SE_2[i] == 0:
                    SE_2cur[i] = SE_2[i] + 1

                if SE_2cur[i] != SE_2[i]:
                    SE_2cur[i] += 1
                    if SE_1[i] == 0:
                        SE_1cur[i] = SE_1[i] + 1

                    if SE_1cur[i] != SE_1[i]:
                        SE_1cur[i] += 1
                        PEcur[i] += 1

                        if PEcur[i] == PE[i]:
                            cur[i] += 1
                            PEcur[i] = 0
                            time.sleep(distDelay / 1_000_000)
                            disDelayCur += distDelay
                            StepMonitors[i] += 1 if dirs[i] else -1
                            VR_angles[i] = (StepMonitors[i] / stepDeg[i]) - negLim[i]

                            if StepMonitors[i] != prev_StepMonitors[i]:
                                prev_StepMonitors[i] = StepMonitors[i]
                    else:
                        SE_1cur[i] = 0
                else:
                    SE_2cur[i] = 0

        highStepCur += 1
        time.sleep(max((curDelay - disDelayCur), 0) / 1_000_000)

    J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM = StepMonitors
    if offlineMode and not liveJog:
        refresh_gui_from_joint_angles(VR_angles)


def parse_mj_command(inData):
    pattern = r"X([-+]?[0-9.]+)Y([-+]?[0-9.]+)Z([-+]?[0-9.]+)Rz([-+]?[0-9.]+)Ry([-+]?[0-9.]+)Rx([-+]?[0-9.]+)Sp([-+]?[0-9.]+)Ac([-+]?[0-9.]+)Dc([-+]?[0-9.]+)Rm([-+]?[0-9.]+)"
    match = re.search(pattern, inData)
    if not match:
        logger.error("MJ command parse failed")
        return None

    vals = [float(v) for v in match.groups()]
    return {
        "xyzuvw": vals[:6],
        "Speed": vals[6],
        "Acc": vals[7],
        "Dec": vals[8],
        "Ramp": vals[9]
    }

def parse_mt_command(inData):
    axis_map = {
        'JTX': 0, 'JTY': 1, 'JTZ': 2,
        'JTR': 3, 'JTP': 4, 'JTW': 5
    }

    # Extract axis and direction (e.g., JTX1 or JTP0)
    axis_match = re.search(r'(JT[XYZRPW])([01])([-+]?[0-9.]+)', inData)
    if not axis_match:
        logger.error("Tool jog command parse failed (axis part)")
        return None

    axis_str = axis_match.group(1)
    direction = int(axis_match.group(2))
    value = float(axis_match.group(3))

    if axis_str not in axis_map:
        logger.warning(f"Unknown axis code: {axis_str}")
        return None

    axis_index = axis_map[axis_str]
    offset_vector = [0.0] * 6
    offset_vector[axis_index] = -value if direction == 1 else value

    # Extract speed and ramp values
    try:
        SpeedType = inData[inData.index("S") + 1]
        Speed = float(inData[inData.index("S") + 2 : inData.index("G")])
        Acc = float(inData[inData.index("G") + 1 : inData.index("H")])
        Dec = float(inData[inData.index("H") + 1 : inData.index("I")])
        Ramp = float(inData[inData.index("I") + 1 : inData.index("Lm")])
        LoopMode = inData.split("Lm")[1].strip()
    except Exception as e:
        logger.error(f"Tool jog command parse failed (parameters): {e}")
        return None

    return {
        "offset_vector": offset_vector,
        "SpeedType": SpeedType,
        "Speed": Speed,
        "Acc": Acc,
        "Dec": Dec,
        "Ramp": Ramp,
        "LoopMode": LoopMode
    }


def rj_command(in_data):
    global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    global J1StepDeg, J2StepDeg, J3StepDeg, J4StepDeg, J5StepDeg, J6StepDeg
    global J1axisLimNeg, J2axisLimNeg, J3axisLimNeg, J4axisLimNeg, J5axisLimNeg, J6axisLimNeg
    global J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim
    global cur_steps, Alarm

    # Find start positions
    Jidx = {label: in_data.find(label) for label in ['A', 'B', 'C', 'D', 'E', 'F']}
    SPstart = in_data.find("S")
    AcStart = in_data.find("Ac")
    DcStart = in_data.find("Dc")
    RmStart = in_data.find("Rm")
    WristConStart = in_data.find("W")
    LoopModeStart = in_data.find("Lm")

    # Parse joint angles
    Jangles = [
        float(in_data[Jidx['A']+1:Jidx['B']]),
        float(in_data[Jidx['B']+1:Jidx['C']]),
        float(in_data[Jidx['C']+1:Jidx['D']]),
        float(in_data[Jidx['D']+1:Jidx['E']]),
        float(in_data[Jidx['E']+1:Jidx['F']]),
        float(in_data[Jidx['F']+1:SPstart]),
    ]

    SpeedType = in_data[SPstart + 1]
    SpeedVal = float(in_data[SPstart + 2:AcStart])
    ACCspd = float(in_data[AcStart + 2:DcStart])
    DCCspd = float(in_data[DcStart + 2:RmStart])
    ACCramp = float(in_data[RmStart + 2:WristConStart])
    WristCon = in_data[WristConStart + 1:LoopModeStart]
    LoopMode = in_data[LoopModeStart + 2:].strip()
    LoopModes = list(map(int, list(LoopMode)))

    fut_steps = [
    int(round((Jangles[0] + J1axisLimNeg) * float(J1StepDeg))),
    int(round((Jangles[1] + J2axisLimNeg) * float(J2StepDeg))),
    int(round((Jangles[2] + J3axisLimNeg) * float(J3StepDeg))),
    int(round((Jangles[3] + J4axisLimNeg) * float(J4StepDeg))),
    int(round((Jangles[4] + J5axisLimNeg) * float(J5StepDeg))),
    int(round((Jangles[5] + J6axisLimNeg) * float(J6StepDeg))),
    ]


    cur_steps = [J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM]
    step_degs = [J1StepDeg, J2StepDeg, J3StepDeg, J4StepDeg, J5StepDeg, J6StepDeg]
    step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

    step_difs = [int(round(cur - fut)) for cur, fut in zip(cur_steps, fut_steps)]


    dirs = [1 if diff <= 0 else 0 for diff in step_difs]
    faults = []
    

    for i in range(6):
        if dirs[i] == 1 and (cur_steps[i] + abs(step_difs[i]) > step_lims[i]):
            faults.append(1)
        elif dirs[i] == 0 and (cur_steps[i] - abs(step_difs[i]) < 0):
            faults.append(1)
        else:
            faults.append(0)

    total_axis_fault = sum(faults)

    if total_axis_fault == 0:
        start_driveMotorsJ_thread(
            *[abs(d) for d in step_difs],
            *dirs,
            SpeedType,
            SpeedVal,
            ACCspd,
            DCCspd,
            ACCramp
        )
    else:
        if offlineMode:
          Alarm = "EL" + ''.join(str(f) for f in faults)
          ErrorHandler(Alarm)


def mj_command(inData):
    global xyzuvw_In, JstepCur, KinematicError, JointMin, JointMax, Robot_Data, JangleOut
    global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    global J1StepDeg, J2StepDeg, J3StepDeg, J4StepDeg, J5StepDeg, J6StepDeg
    global J1axisLimNeg, J2axisLimNeg, J3axisLimNeg, J4axisLimNeg, J5axisLimNeg, J6axisLimNeg
    global J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim
    global cur_steps, Alarm, VR_angles

    logger.info(inData)

    result = parse_mj_command(inData)
    if not result:
        if offlineMode:
          ErrorHandler("ER")
        return

    # Extract values
    xyzuvw_In[:] = result["xyzuvw"]
    SpeedVal = result["Speed"]
    ACCspd = result["Acc"]
    DCCspd = result["Dec"]
    ACCramp = result["Ramp"]
    SpeedType = inData[inData.find("S") + 1]

    xyzuvw_In = np.array(xyzuvw_In, dtype=float)

    # IK call
    JangleOut = robot.SolveInverseKinematics(xyzuvw_In, VR_angles)

    if JangleOut is None:
        if offlineMode:
          logger.error("Inverse kinematics failed. No solution found.")
          ErrorHandler("ER")
        return

 
    JangleOut = np.array(JangleOut, dtype=np.float64).flatten()

    # Convert angles to steps
    step_degs = [float(J1StepDeg), float(J2StepDeg), float(J3StepDeg),
             float(J4StepDeg), float(J5StepDeg), float(J6StepDeg)]
    axis_neg = [float(J1axisLimNeg), float(J2axisLimNeg), float(J3axisLimNeg),
            float(J4axisLimNeg), float(J5axisLimNeg), float(J6axisLimNeg)]
    fut_steps = [int(round((j + off) * deg)) for j, off, deg in zip(JangleOut, axis_neg, step_degs)]

    cur_steps = [J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM]
    step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

    step_difs = [int(round(cur - fut)) for cur, fut in zip(cur_steps, fut_steps)]

    dirs = [1 if diff <= 0 else 0 for diff in step_difs]
    faults = []

    for i in range(6):
        if dirs[i] == 1 and (cur_steps[i] + abs(step_difs[i]) > step_lims[i]):
            faults.append(1)
        elif dirs[i] == 0 and (cur_steps[i] - abs(step_difs[i]) < 0):
            faults.append(1)
        else:
            faults.append(0)

    total_axis_fault = sum(faults)

    if total_axis_fault == 0:
        start_driveMotorsJ_thread(
            *[abs(d) for d in step_difs],
            *dirs,
            SpeedType,
            SpeedVal,
            ACCspd,
            DCCspd,
            ACCramp
        )
    else:
        if offlineMode:
          Alarm = "EL" + ''.join(str(f) for f in faults)
          ErrorHandler(Alarm)
          logger.error(Alarm)




def mt_command(inData):
    global xyzuvw_In, JangleOut, KinematicError, Alarm, VR_angles
    global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    global J1StepDeg, J2StepDeg, J3StepDeg, J4StepDeg, J5StepDeg, J6StepDeg
    global J1axisLimNeg, J2axisLimNeg, J3axisLimNeg, J4axisLimNeg, J5axisLimNeg, J6axisLimNeg
    global J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim
    global cur_steps, offlineMode
    global XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos 

    result = parse_mt_command(inData)
    if not result:
        if offlineMode:
            ErrorHandler("ER")
        return
    
    offset = [float(v) for v in result["offset_vector"]]
    robot.set_robot_tool_frame(*offset)

    # Build xyzuvw_In from current pose
    xyzuvw_In = np.array([
        float(XcurPos),
        float(YcurPos),
        float(ZcurPos),
        float(RzcurPos),
        float(RycurPos),
        float(RxcurPos)
    ], dtype=float)


    # IK solve
    JangleOut = robot.SolveInverseKinematics(xyzuvw_In, VR_angles)

    # put tool frame back where it was
    robot.set_robot_tool_frame(float(TFxEntryField.get()), 
                                   float(TFyEntryField.get()), 
                                   float(TFzEntryField.get()), 
                                   float(TFrxEntryField.get()), 
                                   float(TFryEntryField.get()), 
                                   float(TFrzEntryField.get()))

    if JangleOut is None:
        if offlineMode:
            logger.error("Inverse kinematics failed. No solution found.")
            ErrorHandler("ER")
        return

    JangleOut = np.array(JangleOut, dtype=np.float64).flatten()

    # Convert angles to steps
    step_degs = [float(J1StepDeg), float(J2StepDeg), float(J3StepDeg),
                 float(J4StepDeg), float(J5StepDeg), float(J6StepDeg)]
    axis_neg = [float(J1axisLimNeg), float(J2axisLimNeg), float(J3axisLimNeg),
                float(J4axisLimNeg), float(J5axisLimNeg), float(J6axisLimNeg)]
    fut_steps = [int(round((j + off) * deg)) for j, off, deg in zip(JangleOut, axis_neg, step_degs)]

    cur_steps = [J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM]
    step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

    step_difs = [int(round(cur - fut)) for cur, fut in zip(cur_steps, fut_steps)]
    dirs = [1 if diff <= 0 else 0 for diff in step_difs]

    # Check limits
    faults = []
    for i in range(6):
        if dirs[i] == 1 and (cur_steps[i] + abs(step_difs[i]) > step_lims[i]):
            faults.append(1)
        elif dirs[i] == 0 and (cur_steps[i] - abs(step_difs[i]) < 0):
            faults.append(1)
        else:
            faults.append(0)

    if sum(faults) == 0:
        start_driveMotorsJ_thread(
            *[abs(d) for d in step_difs],
            *dirs,
            result["SpeedType"],
            result["Speed"],
            result["Acc"],
            result["Dec"],
            result["Ramp"]
        )
    else:
        if offlineMode:
            Alarm = "EL" + ''.join(str(f) for f in faults)
            ErrorHandler(Alarm)     
     


def live_joint_jog(in_data):
    global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    global VR_angles, J1axisLimNeg, J2axisLimNeg, J3axisLimNeg, J4axisLimNeg, J5axisLimNeg, J6axisLimNeg
    global J1StepDeg, J2StepDeg, J3StepDeg, J4StepDeg, J5StepDeg, J6StepDeg
    global J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim
    global KinematicError, Alarm, flag, liveJog

    # Parse jog command components
    Vector = float(in_data[in_data.index("V") + 1 : in_data.index("S")])
    SpeedType = in_data[in_data.index("S") + 1]
    SpeedVal = float(in_data[in_data.index("S") + 2 : in_data.index("Ac")])
    ACCspd = DCCspd = ACCramp = 100  # Simplified for now

    LoopModeStr = in_data.split("Lm")[1].strip()
    LoopModes = [int(c) for c in LoopModeStr]

    idx = int(Vector // 10) - 1
    direction = 1 if int(Vector) % 10 == 1 else -1

    if not (0 <= idx < 6):
        Alarm = "ER"
        ErrorHandler(Alarm)
        return

    liveJog = True
    while liveJog:
        while drive_lock.locked():
                    time.sleep(0.005)   
        try:
            Jangles = [float(a) for a in VR_angles[:6]]
        except Exception as e:
            if offlineMode:
              logger.error("Invalid VR_angles:", VR_angles[:6])
              Alarm = "ER"
              ErrorHandler(Alarm)
            return

        Jangles[idx] += direction * .1

        axis_lims = [
            float(J1axisLimNeg), float(J2axisLimNeg), float(J3axisLimNeg),
            float(J4axisLimNeg), float(J5axisLimNeg), float(J6axisLimNeg)
        ]
        step_degs = [
            float(J1StepDeg), float(J2StepDeg), float(J3StepDeg),
            float(J4StepDeg), float(J5StepDeg), float(J6StepDeg)
        ]
        step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]
        cur_steps = [J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM]

        fut_steps = [int(round((Jangles[i] + axis_lims[i]) * step_degs[i])) for i in range(6)]
        step_difs = [cur - fut for cur, fut in zip(cur_steps, fut_steps)]
        dirs = [1 if diff <= 0 else 0 for diff in step_difs]

        faults = []
        for i in range(6):
            if dirs[i] == 1 and (cur_steps[i] + abs(step_difs[i]) > step_lims[i]):
                faults.append(1)
            elif dirs[i] == 0 and (cur_steps[i] - abs(step_difs[i]) < 0):
                faults.append(1)
            else:
                faults.append(0)

        total_axis_fault = sum(faults)

        if total_axis_fault == 0:
            if not drive_lock.locked():
                start_driveMotorsJ_thread(
                    *[abs(d) for d in step_difs],
                    *dirs,
                    SpeedType,
                    SpeedVal,
                    ACCspd,
                    DCCspd,
                    ACCramp
            )

                
        else:
            if offlineMode:
              Alarm = "EL" + ''.join(str(f) for f in faults)
              ErrorHandler(Alarm)
              Alarm = "0"
            break



def live_cartesian_jog(in_data):
    global xyzuvw_In, xyzuvw_Out, VR_angles, JangleOut, KinematicError, Alarm
    global XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos
    global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    global J1StepDeg, J2StepDeg, J3StepDeg, J4StepDeg, J5StepDeg, J6StepDeg
    global J1axisLimNeg, J2axisLimNeg, J3axisLimNeg, J4axisLimNeg, J5axisLimNeg, J6axisLimNeg
    global J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim
    global liveJog

    # Parse command
    Vector = float(in_data[in_data.index("V") + 1:in_data.index("S")])
    SpeedType = in_data[in_data.index("S") + 1]
    SpeedVal = float(in_data[in_data.index("S") + 2:in_data.index("Ac")])
    ACCspd = DCCspd = ACCramp = 100  # fixed for now
    LoopModeStr = in_data.split("Lm")[1].strip()
    LoopModes = [int(c) for c in LoopModeStr]

    # Cartesian jog increment
    jog_step = 1  # mm or deg, depending on axis

    xyzuvw_In = np.array([
        float(XcurPos),
        float(YcurPos),
        float(ZcurPos),
        float(RzcurPos),
        float(RycurPos),
        float(RxcurPos)
    ], dtype=float)

    liveJog = True
    while liveJog:
        while drive_lock.locked():
            time.sleep(0.005)   

        idx = int(Vector // 10) - 1
        direction = 1 if int(Vector) % 10 == 1 else -1

        if 0 <= idx < 6:
            xyzuvw_In[idx] += direction * jog_step
        else:
            Alarm = "ER"
            #ErrorHandler(Alarm)
            break

        # Inverse Kinematics
        try:
            JangleOut = robot.SolveInverseKinematics(xyzuvw_In, VR_angles)
        except Exception as e:
            logger.error("IK Exception:", e)
            ErrorHandler("ER")
            break

        if JangleOut is None:
            if offlineMode:
              Alarm = "ER"
              ErrorHandler(Alarm)
            break
        
        JangleOut = np.array(JangleOut, dtype=np.float64).flatten()

        # Convert angles to steps
        step_degs = [
            float(J1StepDeg), float(J2StepDeg), float(J3StepDeg),
            float(J4StepDeg), float(J5StepDeg), float(J6StepDeg)
        ]
        axis_lims = [
            float(J1axisLimNeg), float(J2axisLimNeg), float(J3axisLimNeg),
            float(J4axisLimNeg), float(J5axisLimNeg), float(J6axisLimNeg)
        ]
        #fut_steps = [int(round((float(j) + float(off)) * float(deg))) for j, off, deg in zip(JangleOut, axis_neg, step_degs)]
        fut_steps = [int(round((JangleOut[i] + axis_lims[i]) * step_degs[i])) for i in range(6)]

        cur_steps = [J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM]
        step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

        step_difs = [cur - fut for cur, fut in zip(cur_steps, fut_steps)]
        dirs = [1 if diff <= 0 else 0 for diff in step_difs]

        # Check axis limits
        faults = []
        for i in range(6):
            if dirs[i] == 1 and (cur_steps[i] + abs(step_difs[i]) > step_lims[i]):
                faults.append(1)
            elif dirs[i] == 0 and (cur_steps[i] - abs(step_difs[i]) < 0):
                faults.append(1)
            else:
                faults.append(0)

        if sum(faults) == 0 and KinematicError == 0:
            if not drive_lock.locked():
                start_driveMotorsJ_thread(
                    *[abs(d) for d in step_difs],
                    *dirs,
                    SpeedType,
                    SpeedVal,
                    ACCspd,
                    DCCspd,
                    ACCramp
            )

                 
        else:
            Alarm = "EL" + ''.join(str(f) for f in faults)
            ErrorHandler(Alarm)
            break


def live_tool_jog(in_data):
    global xyzuvw_In, JangleOut, KinematicError, Alarm, VR_angles
    global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    global J1StepDeg, J2StepDeg, J3StepDeg, J4StepDeg, J5StepDeg, J6StepDeg
    global J1axisLimNeg, J2axisLimNeg, J3axisLimNeg, J4axisLimNeg, J5axisLimNeg, J6axisLimNeg
    global J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim
    global TFxEntryField, TFyEntryField, TFzEntryField, TFrxEntryField, TFryEntryField, TFrzEntryField
    global liveJog, offlineMode

    # Parse command
    Vector = float(in_data[in_data.index("V") + 1:in_data.index("S")])
    SpeedType = in_data[in_data.index("S") + 1]
    SpeedVal = float(in_data[in_data.index("S") + 2:in_data.index("Ac")])
    ACCspd = DCCspd = ACCramp = 100  # fixed acceleration values
    LoopModeStr = in_data.split("Lm")[1].strip()
    LoopModes = [int(c) for c in LoopModeStr]

    # Tool frame jog step size
    jog_step = 1  # mm or degrees depending on axis

    # Save original tool frame to restore later
    original_tool_frame = [
        float(TFxEntryField.get()),
        float(TFyEntryField.get()),
        float(TFzEntryField.get()),
        float(TFrxEntryField.get()),
        float(TFryEntryField.get()),
        float(TFrzEntryField.get())
    ]

    liveJog = True
    while liveJog:
        while drive_lock.locked():
            time.sleep(0.005)

        idx = int(Vector // 10) - 1
        if idx == 3:  # Rz → Trx
            idx = 5
        elif idx == 5:  # Rx → Trz
            idx = 3

        direction = 1 if int(Vector) % 10 == 1 else -1

        # Build pose from current position
        xyzuvw_In = robot.forward_kinematics(VR_angles)
        xyzuvw_In = xyzuvw_In[:3] + [math.degrees(v) for v in xyzuvw_In[3:]]   

        if 0 <= idx < 6:
            # Modify tool frame temporarily
            jogged_tool_frame = original_tool_frame.copy()
            jogged_tool_frame[idx] += direction * jog_step
            robot.set_robot_tool_frame(*jogged_tool_frame)
        else:
            Alarm = "ER"
            ErrorHandler(Alarm)
            break

        try:
            JangleOut = robot.SolveInverseKinematics(xyzuvw_In, VR_angles)
        except Exception as e:
            logger.error(f"IK Exception: {e}")
            ErrorHandler("ER")
            break

        # Restore original tool frame
        robot.set_robot_tool_frame(*original_tool_frame)

        if JangleOut is None:
            if offlineMode:
                Alarm = "ER"
                ErrorHandler(Alarm)
            break

        JangleOut = np.array(JangleOut, dtype=np.float64).flatten()

        step_degs = [
            float(J1StepDeg), float(J2StepDeg), float(J3StepDeg),
            float(J4StepDeg), float(J5StepDeg), float(J6StepDeg)
        ]
        axis_lims = [
            float(J1axisLimNeg), float(J2axisLimNeg), float(J3axisLimNeg),
            float(J4axisLimNeg), float(J5axisLimNeg), float(J6axisLimNeg)
        ]
        fut_steps = [int(round((JangleOut[i] + axis_lims[i]) * step_degs[i])) for i in range(6)]

        cur_steps = [J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM]
        step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

        step_difs = [cur - fut for cur, fut in zip(cur_steps, fut_steps)]
        dirs = [1 if diff <= 0 else 0 for diff in step_difs]

        # Axis limit check
        faults = []
        for i in range(6):
            if dirs[i] == 1 and (cur_steps[i] + abs(step_difs[i]) > step_lims[i]):
                faults.append(1)
            elif dirs[i] == 0 and (cur_steps[i] - abs(step_difs[i]) < 0):
                faults.append(1)
            else:
                faults.append(0)

        if sum(faults) == 0 and KinematicError == 0:
            if not drive_lock.locked():
                start_driveMotorsJ_thread(
                    *[abs(d) for d in step_difs],
                    *dirs,
                    SpeedType,
                    SpeedVal,
                    ACCspd,
                    DCCspd,
                    ACCramp
                )
        else:
            Alarm = "EL" + ''.join(str(f) for f in faults)
            ErrorHandler(Alarm)
            break
        



#############################################################################################
### VIRTUAL ROBOT ###########################################################################
#############################################################################################

# Global storage
vtk_running = False
actors = {}
assemblies = {}
base_transforms = {}
joint_transforms = {}
composite_transforms = {}


def toggle_offline_mode():
    global offlineMode
    global VR_angles, J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur
    offlineMode = not offlineMode
    if offlineMode:
        offline_button.config(text="Go Online", style="Offline.TButton")
        almStatusLab.config(text="SYSTEM IN OFFLINE MODE", style="Warn.TLabel")
        almStatusLab2.config(text="SYSTEM IN OFFLINE MODE", style="Warn.TLabel")
        VR_angles = [0.000, 0.000, 0.000, 0.000, 90.000, 0.000]
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
        refresh_gui_from_joint_angles(VR_angles)

    else:
        offline_button.config(text="Run Offline", style="Online.TButton")
        almStatusLab.config(text="SYSTEM IN ONLINE MODE", style="OK.TLabel")
        almStatusLab2.config(text="SYSTEM IN ONLINE MODE", style="OK.TLabel")
        requestPos()
        VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
        setStepMonitorsVR()
        
def request_render():
    render_window.Render()

def update_joint_transforms():
    angles = VR_angles  # List of 6 joint angles in degrees

    joint_stl_keys = [
        "Link 1-1.STL",
        "Link 2-1.STL",
        "Link 3-1.STL",
        "Link 4-1.STL",
        "Link 5-1.STL",
        "Link 6-1.STL",
    ]

    for i, stl_key in enumerate(joint_stl_keys):
        joint_tf = joint_transforms[stl_key]
        joint_tf.Identity()

        if i == 0:
            joint_tf.RotateZ(angles[i])
        elif i == 1:
            joint_tf.RotateY(angles[i])
        elif i == 2:
            joint_tf.RotateY(angles[i])
        elif i == 3:
            joint_tf.RotateX(angles[i])
        elif i == 4:
            joint_tf.RotateY(angles[i])
        elif i == 5:
            joint_tf.RotateX(angles[i])  


def build_robot_actors(renderer):
    global actors, assemblies, base_transforms, joint_transforms, composite_transforms, color_map
    # Named colors setup
    colors = vtk.vtkNamedColors()

    # STL files including Link 4-2.STL
    stl_files = [
        "Link Base-1.STL", "Link Base-2.STL", "Link Base-3.STL",
        "Link 1-1.STL", "Link 1-2.STL",
        "Link 2-1.STL", "Link 2-2.STL", "Link 2-3.STL",
        "Link 3-1.STL", "Link 3-2.STL",
        "Link 4-1.STL", "Link 4-2.STL", "Link 4-3.STL",
        "Link 5-1.STL", "Link 5-2.STL",
        "Link 6-1.STL", "Link 6-2.STL"
    ]

    # Clear and initialize the global color map
    color_map.clear()
    color_map.update({stl: "Silver" for stl in stl_files})
    color_map.update({
        "Link Base-2.STL": "Orange",
        "Link Base-3.STL": "DimGray",
        "Link 1-2.STL": "DimGray",
        "Link 2-2.STL": "Orange", "Link 2-3.STL": "DimGray",
        "Link 3-2.STL": "DimGray",
        "Link 4-2.STL": "Orange", "Link 4-3.STL": "DimGray",
        "Link 5-2.STL": "DimGray",
        "Link 6-2.STL": "DimGray"
    })

    # Storage reset
    actors.clear()
    assemblies.clear()
    base_transforms.clear()
    joint_transforms.clear()
    composite_transforms.clear()

    # Load STL files and create actors
    for stl in stl_files:
        reader = vtk.vtkSTLReader()
        reader.SetFileName(stl)
        reader.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        # Apply initial color from the shared color_map
        actor.GetProperty().SetColor(colors.GetColor3d(color_map[stl]))

        base_tf = vtk.vtkTransform()
        joint_tf = vtk.vtkTransform()
        comp_tf = vtk.vtkTransform()

        # Alignment transforms
        if stl == "Link 1-1.STL":
            base_tf.RotateX(180)
            base_tf.Translate(0, 0, -87.5)
        elif stl == "Link 2-1.STL":
            base_tf.RotateZ(180)
            base_tf.RotateX(270)
            base_tf.Translate(-64.15, 77.78, 8.87)
        elif stl == "Link 3-1.STL":
            base_tf.RotateZ(180)
            base_tf.RotateX(180)
            base_tf.Translate(0, 305, -27.84)
        elif stl == "Link 4-1.STL":
            base_tf.RotateY(90)
            base_tf.RotateX(180)
            base_tf.Translate(-36.7, 0, -75.94)
        elif stl == "Link 5-1.STL":
            base_tf.RotateZ(180)
            base_tf.RotateY(90)
            base_tf.Translate(147, 0, 44.88)
        elif stl == "Link 6-1.STL":
            base_tf.RotateY(90)
            base_tf.Translate(43.3, 0, 25)

        comp_tf.Concatenate(base_tf)
        comp_tf.Concatenate(joint_tf)

        asm = vtk.vtkAssembly()
        asm.AddPart(actor)
        asm.SetUserTransform(comp_tf)

        actors[stl] = actor
        assemblies[stl] = asm
        base_transforms[stl] = base_tf
        joint_transforms[stl] = joint_tf
        composite_transforms[stl] = comp_tf

    # Build hierarchy
    root = assemblies["Link Base-1.STL"]
    root.AddPart(assemblies["Link Base-2.STL"])
    assemblies["Link Base-2.STL"].AddPart(assemblies["Link Base-3.STL"])
    assemblies["Link Base-3.STL"].AddPart(assemblies["Link 1-1.STL"])
    assemblies["Link 1-1.STL"].AddPart(assemblies["Link 1-2.STL"])
    assemblies["Link 1-2.STL"].AddPart(assemblies["Link 2-1.STL"])
    assemblies["Link 2-1.STL"].AddPart(assemblies["Link 2-2.STL"])
    assemblies["Link 2-2.STL"].AddPart(assemblies["Link 2-3.STL"])
    assemblies["Link 2-3.STL"].AddPart(assemblies["Link 3-1.STL"])
    assemblies["Link 3-1.STL"].AddPart(assemblies["Link 3-2.STL"])
    assemblies["Link 3-2.STL"].AddPart(assemblies["Link 4-1.STL"])
    assemblies["Link 4-1.STL"].AddPart(assemblies["Link 4-2.STL"])
    assemblies["Link 4-2.STL"].AddPart(assemblies["Link 4-3.STL"])
    assemblies["Link 4-3.STL"].AddPart(assemblies["Link 5-1.STL"])
    assemblies["Link 5-1.STL"].AddPart(assemblies["Link 5-2.STL"])
    assemblies["Link 5-2.STL"].AddPart(assemblies["Link 6-1.STL"])
    assemblies["Link 6-1.STL"].AddPart(assemblies["Link 6-2.STL"])

    renderer.AddActor(root)


class CustomInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, renderer):
        self.AddObserver("LeftButtonReleaseEvent", self.on_left_button_up)
        self.renderer = renderer

    def on_left_button_up(self, obj, event):
        self.OnLeftButtonUp()  # <-- CORRECT way to call the base method

def update_joint_angles():
    angles = {
        "Link 1-1.STL": -VR_angles[0],
        "Link 2-1.STL": VR_angles[1],
        "Link 3-1.STL": -VR_angles[2],
        "Link 4-1.STL": -VR_angles[3],
        "Link 5-1.STL": -VR_angles[4],
        "Link 6-1.STL": VR_angles[5]
    }

    for stl, angle in angles.items():
        jt = joint_transforms[stl]
        jt.Identity()
        jt.RotateZ(angle)
        ct = composite_transforms[stl]
        ct.Identity()
        ct.Concatenate(base_transforms[stl])
        ct.Concatenate(jt)

def add_floor_grid(renderer, size=1000, spacing=50):
    grid = vtk.vtkPolyData()
    points = vtk.vtkPoints()
    lines = vtk.vtkCellArray()

    count = 0
    for i in range(-size, size + spacing, spacing):
        # lines parallel to X
        points.InsertNextPoint(i, -size, 0)
        points.InsertNextPoint(i, size, 0)
        lines.InsertNextCell(2)
        lines.InsertCellPoint(count)
        lines.InsertCellPoint(count + 1)
        count += 2

        # lines parallel to Y
        points.InsertNextPoint(-size, i, 0)
        points.InsertNextPoint(size, i, 0)
        lines.InsertNextCell(2)
        lines.InsertCellPoint(count)
        lines.InsertCellPoint(count + 1)
        count += 2

    grid.SetPoints(points)
    grid.SetLines(lines)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(grid)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.7, 0.7, 0.7)  # Light gray
    actor.GetProperty().SetLineWidth(1)

    renderer.AddActor(actor)        

def on_close_event(obj, event):
    global vtk_running
    vtk_running = False
    try:
        obj.GetRenderWindow().Finalize()
        obj.TerminateApp()
    except:
        pass

def update_vtk(render_window, root_widget):
    if vtk_running:
        update_joint_angles()
        tab1.after(0, request_render)
        root_widget.after(16, lambda: update_vtk(render_window, root_widget))


def add_reset_view_button(renderer, interactor, camera):
    # Create a text actor for the button
    text_actor = vtk.vtkTextActor()
    text_actor.SetInput("Reset View")
    text_actor.GetTextProperty().SetFontSize(24)
    text_actor.GetTextProperty().SetColor(1.0, 1.0, 1.0)  # white
    text_actor.SetDisplayPosition(20, 20)  # bottom-left corner
    renderer.AddActor2D(text_actor)

    # Get a rough width/height for click bounds (trial and error)
    click_bounds = {
        'x1': 20,
        'y1': 20,
        'x2': 20 + 150,  # width of text
        'y2': 20 + 40    # height of text
    }

    def click_callback(obj, event):
        click_pos = interactor.GetEventPosition()
        x, y = click_pos
        if click_bounds['x1'] <= x <= click_bounds['x2'] and click_bounds['y1'] <= y <= click_bounds['y2']:
            camera.Azimuth(45)
            camera.Elevation(35)
            camera.SetViewUp(0, 0, 1)
            renderer.ResetCamera()
            renderer.ResetCameraClippingRange()
            interactor.GetRenderWindow().Render()

    # Attach click handler
    interactor.AddObserver("LeftButtonPressEvent", click_callback)        

def launch_vtk_nonblocking(root_widget):
    global renderer, vtk_running, interactor, render_window, VR_angles
    global J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur

    vtk_running = True

    renderer = vtk.vtkRenderer()
    render_window = vtk.vtkRenderWindow() 
    render_window.SetWindowName("AR4 Virtual Robot Viewer")
    interactor = vtk.vtkRenderWindowInteractor()
    render_window.AddRenderer(renderer)
    interactor.SetRenderWindow(render_window)

    style = CustomInteractorStyle(renderer)
    interactor.SetInteractorStyle(style)

    render_window.SetSize(1024, 768)
    renderer.SetBackground(vtk.vtkNamedColors().GetColor3d("LightSlateGray"))

    build_robot_actors(renderer)
    add_floor_grid(renderer)

    camera = renderer.GetActiveCamera()
    renderer.ResetCamera()
    camera.Dolly(3)
    camera.Azimuth(65)
    camera.Elevation(55)
    camera.SetViewUp(0, 0, 1)
    renderer.ResetCameraClippingRange()

    #add_reset_view_button(renderer, interactor, camera)

    interactor.AddObserver("ExitEvent", on_close_event)
    interactor.Initialize()
    render_window.Render()

    # Embed periodic update and check render loop
    update_vtk(render_window, root_widget)

    VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
    setStepMonitorsVR()
    update_main_color()




def update_stl_transform():
    name = stl_name_var.get()
    if name not in imported_actors:
        logger.error("File not found in imported actors.")
        return

    actor = imported_actors[name]
    try:
        x = float(x_var.get())
        y = float(y_var.get())
        z = float(z_var.get())
        rot = float(rot_var.get())
    except ValueError:
        logger.error("Invalid number entered.")
        return

    transform = vtk.vtkTransform()
    transform.Translate(x, y, z)
    transform.RotateZ(rot)

    actor.SetUserTransform(transform)
    render_window.Render()





imported_actors = {}  # filename -> actor mapping

stl_name_var = tk.StringVar()
x_var = tk.StringVar(value="0")
y_var = tk.StringVar(value="0")
z_var = tk.StringVar(value="0")
rot_var = tk.StringVar(value="0")

def import_stl_file():
    file_path = fd.askopenfilename(filetypes=[("STL files", "*.stl")])
    if not file_path:
        return

    filename = os.path.basename(file_path)
    stl_name_var.set(filename)

    reader = vtk.vtkSTLReader()
    reader.SetFileName(file_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.254, 0.41, 0.882)
    actor.SetPosition(0, 0, 0)

    renderer.AddActor(actor)
    render_window.Render()

    imported_actors[filename] = actor


def load_stl_into_scene(stl_path, renderer, render_window):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(stl_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.254, 0.41, 0.882)  

    actor.SetPosition(0, 0, 0)  # Adjust as needed

    renderer.AddActor(actor)
    render_window.Render()

   
    




###############################################################################################################################################################
### STARTUP DEFS ################################################################################################################# 
###############################################################################################################################################################




def startup_spinner(root, message="Please wait…"):
    win = tk.Toplevel(root)
    win.title("")
    win.transient(root)
    win.resizable(False, False)
    win.grab_set()  # modal

    # Use same icon as main window
    #win.iconbitmap(r'AR.png')
    win.iconphoto(True, tk.PhotoImage(file="AR.png"))
    ttk.Label(win, text=message, padding=12).pack()
    pb = ttk.Progressbar(win, mode="indeterminate", length=220)
    pb.pack(padx=12, pady=(0, 12))
    pb.start(12)

    # Center on parent
    root.update_idletasks()
    x = root.winfo_rootx() + (root.winfo_width() - win.winfo_reqwidth()) // 2
    y = root.winfo_rooty() + (root.winfo_height() - win.winfo_reqheight()) // 2
    win.geometry(f"+{x}+{y}")

    win.update_idletasks()
    return win, pb


def startup_with_spinner(root, timeout=10.0):
    spinner, pb = startup_spinner(root, "Please Wait.. System Starting")
    q = Queue()

    def worker():
        try:
            q.put(startup())
        except Exception as e:
            q.put(e)

    Thread(target=worker, daemon=True).start()

    deadline = time.monotonic() + timeout
    while q.empty() and time.monotonic() < deadline:
        root.update()
        time.sleep(0.01)

    # close spinner (success or timeout)
    try: pb.stop()
    except: pass
    try: spinner.grab_release()
    except: pass
    spinner.destroy()

    if q.empty():
        logger.error("UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER (timed out after %.1fs)", timeout)
        raise TimeoutError(f"Startup timed out after {timeout:.1f}s")

    res = q.get()
    if isinstance(res, Exception):
        logger.exception("Startup failed while initializing Teensy 4.1 controller")
        raise res
    return res


def startup():
  setCom2()
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

###############################################################################################################################################################
# Change of field to support automatic comm detection and drop down
# Added exception output to log window
def setCom(misc=None):  # Requires an input parameter for element use / it's unused
  global ser
  Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
  port = com1SelectedValue.get()
  baud = 9600

  # If something was already open, close it first (prevents WinError 5 on switch)
  try:
    if 'ser' in globals() and ser and getattr(ser, "is_open", False):
      ser.close()
      time.sleep(0.2)  # give Windows a moment to release the handle
  except Exception:
    pass

  try:
    logger.info("COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER on Port %s", port)

    # Add small timeouts so reads/writes can’t hang forever
    ser = serial.Serial(
      port=port,
      baudrate=baud
    )

    almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
    logger.info("COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER")

    time.sleep(.1)
    # Prefer reset_input_buffer over deprecated flushInput
    try:
      ser.reset_input_buffer()
      ser.reset_output_buffer()
    except Exception:
      pass

    startup_with_spinner(root)  # if this raises, we’ll close port in except block below

    # persist log view
    value = tab8.ElogView.get(0, END)
    pickle.dump(value, open("ErrorLog", "wb"))

  except Exception as e:
    # Ensure the port is closed on ANY failure after open
    try:
      if 'ser' in globals() and ser and getattr(ser, "is_open", False):
        ser.close()
        time.sleep(0.2)
    except Exception:
      pass

    logger.exception("UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER")
    almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER", style="Alarm.TLabel")
    almStatusLab2.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER", style="Alarm.TLabel")

    # persist log view even on error
    try:
      value = tab8.ElogView.get(0, END)
      pickle.dump(value, open("ErrorLog", "wb"))
    except Exception:
      pass


def setCom2(misc=None): # Requires and input parameter for element use / its unused
  try:
    global ser2
    #port = "COM" + com2PortEntryField.get()
    port = com2SelectedValue.get()
    baud = 9600    
    ser2 = serial.Serial(port,baud)
    #almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
    #almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
    logger.info(f"COMMUNICATIONS STARTED WITH ARDUINO IO BOARD on port: {port}")
    #tab8.ElogView.insert(END, Curtime+f" - COMMUNICATIONS STARTED WITH ARDUINO IO BOARD on port: {port}")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
  except Exception as e:
    logger.error(f"{e}")
    #tab8.ElogView.insert(END, Curtime+f" - Error: {e}")
    #almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD", style="Alarm.TLabel")
    #almStatusLab2.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD", style="Alarm.TLabel")
    logger.error(f"UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD")
    #tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
    
def darkTheme():
  global curTheme
  curTheme = 0
  # Use the existing style instance and switch theme
  if hasattr(root, 'style'):
    style = root.style
    style.theme_use("darkly")
  else:
    style = BootstrapStyle(theme="darkly")
    root.style = style
  
  # Configure custom styles for the darkly theme
  style.configure("Alarm.TLabel", foreground="#dc3545", font = ('Arial','10','bold'))  # Bootstrap danger color
  style.configure("Warn.TLabel", foreground="#fd7e14", font = ('Arial','10','bold'))   # Bootstrap warning color
  style.configure("OK.TLabel", foreground="#198754", font = ('Arial','10','bold'))     # Bootstrap success color
  style.configure("Jointlim.TLabel", foreground="#0dcaf0", font = ('Arial','8'))      # Bootstrap info color
  style.configure('AlarmBut.TButton', foreground ='#dc3545')                          # Bootstrap danger color
  style.configure('Frame1.TFrame', background='#ffffff')
  style.configure("Offline.TButton", foreground="#198754", font = ('Arial','8','bold'))  # Bootstrap success color
  style.configure("Online.TButton")
  # Configure Entry widgets for better alignment with buttons
  style.configure("TEntry", 
                  fieldbackground="#495057",  # Dark background for darkly theme
                  borderwidth=1,
                  insertcolor="#ffffff",      # White cursor
                  padding=(1, 0, 1, 0))      # More aggressive padding reduction: left, top, right, bottom
  
  # Configure Button widgets for subtle size reduction
  style.configure("TButton", 
                  padding=(5, 3, 5, 3))     # Very subtle padding reduction: just slightly smaller than default
  
  # Configure OptionMenu widgets to match button proportions
  style.configure("TMenubutton", 
                  padding=(5, 3, 5, 3))     # Match button padding for proportional scaling


def lightTheme():
  global curTheme
  curTheme = 1
  # Use the existing style instance and switch theme
  if hasattr(root, 'style'):
    style = root.style
    style.theme_use("flatly")  # Changed to sandstone theme
  else:
    style = BootstrapStyle(theme="flatly")  # Changed to sandstone theme
    root.style = style
  
  # Configure custom styles for the light theme
  style.configure("Alarm.TLabel", foreground="#dc3545", font = ('Arial','10','bold'))  # Bootstrap danger color
  style.configure("Warn.TLabel", foreground="#fd7e14", font = ('Arial','10','bold'))   # Bootstrap warning color
  style.configure("OK.TLabel", foreground="#198754", font = ('Arial','10','bold'))     # Bootstrap success color
  style.configure("Jointlim.TLabel", foreground="#0d6efd", font = ('Arial','8'))      # Bootstrap primary color
  style.configure('AlarmBut.TButton', foreground ='#dc3545')                          # Bootstrap danger color
  style.configure('Frame1.TFrame', background='#000000')
  style.configure("Offline.TButton", foreground="#fd7e14")                            # Bootstrap warning color
  style.configure("Online.TButton", foreground="#000000")
  style.configure("Offline.TButton", foreground="#198754", font = ('Arial','8','bold'))  # Bootstrap success color
  style.configure("Online.TButton")
  # Configure Entry widgets for better alignment with buttons
  style.configure("TEntry", 
                  fieldbackground="#ffffff",  # White background for light theme
                  borderwidth=1,
                  insertcolor="#000000",      # Black cursor
                  padding=(1, 0, 1, 0))      # More aggressive padding reduction: left, top, right, bottom
  
  # Configure Button widgets for subtle size reduction
  style.configure("TButton", 
                  padding=(5, 3, 5, 3))     # Very subtle padding reduction: just slightly smaller than default
  
  # Configure OptionMenu widgets to match button proportions
  style.configure("TMenubutton", 
                  padding=(5, 3, 5, 3))     # Match button padding for proportional scaling



###############################################################################################################################################################  
### EXECUTION DEFS ######################################################################################################################### EXECUTION DEFS ###  
############################################################################################################################################################### 

def runProg():
  def threadProg():
    global rowinproc
    global stopQueue
    global splineActive
    global estopActive
    estopActive = False
    global posOutreach
    posOutreach = False
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
        if (estopActive == TRUE):
          almStatusLab.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
          almStatusLab2.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
        elif (posOutreach == TRUE):
          almStatusLab.config(text="Position Out of Reach",  style="Alarm.TLabel")
          almStatusLab2.config(text="Position Out of Reach",  style="Alarm.TLabel")  
        else:
          almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
          almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")  
      else:
        almStatusLab.config(text="PROGRAM RUNNING",  style="OK.TLabel")
        almStatusLab2.config(text="PROGRAM RUNNING",  style="OK.TLabel") 
      rowinproc = 1
      executeRow()
      while rowinproc == 1:
        time.sleep(.1)
        try:
          selRow = tab1.progView.curselection()[0]
        except:
          pass
      last = tab1.progView.index('end')
      #for row in range (0,selRow):
        #tab1.progView.itemconfig(row, {'fg': 'dodger blue'})
      #tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
      #for row in range (selRow+1,last):
        #tab1.progView.itemconfig(row, {'fg': 'black'})
      tab1.progView.selection_clear(0, END)
      try:
        selRow += 1
        tab1.progView.select_set(selRow)
        curRow += 1
      except:
        pass
      time.sleep(.1)
      try:
        selRow = tab1.progView.curselection()[0]
        curRowEntryField.delete(0, 'end')
        curRowEntryField.insert(0,selRow)
      except:
        curRowEntryField.delete(0, 'end')
        curRowEntryField.insert(0,"---") 
        tab1.runTrue = 0
        if (estopActive == TRUE):
          almStatusLab.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
          almStatusLab2.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
        elif (posOutreach == TRUE):
          almStatusLab.config(text="Position Out of Reach",  style="Alarm.TLabel")
          almStatusLab2.config(text="Position Out of Reach",  style="Alarm.TLabel")    
        else:
          almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
          almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel") 
  t = threading.Thread(target=threadProg)
  t.start()
  
def stepFwd():
    def threadProg():
      global estopActive
      estopActive = False
      global posOutreach
      posOutreach = False
      almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
      almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel") 
      executeRow() 
      selRow = tab1.progView.curselection()[0]
      last = tab1.progView.index('end')
      for row in range (0,selRow):
        tab1.progView.itemconfig(row, {'fg': 'dodger blue'})
      tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
      for row in range (selRow+1,last):
        tab1.progView.itemconfig(row, {'fg': 'gray'})
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
    t = threading.Thread(target=threadProg)
    t.start()

def stepRev():
    global estopActive
    estopActive = False
    global posOutreach
    posOutreach = False
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel") 
    executeRow()  
    selRow = tab1.progView.curselection()[0]
    last = tab1.progView.index('end')
    for row in range (0,selRow):
      tab1.progView.itemconfig(row, {'fg': 'gray'})
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
  global estopActive
  global posOutreach
  global stopQueue
  lastProg = ""
  tab1.runTrue = 0
  if (estopActive == TRUE):
    almStatusLab.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
    almStatusLab2.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
  elif (posOutreach == TRUE):
    almStatusLab.config(text="Position Out of Reach",  style="Alarm.TLabel")
    almStatusLab2.config(text="Position Out of Reach",  style="Alarm.TLabel")    
  else:        
    almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
    almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")

  
  
def executeRow():
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global rowinproc
  global LineDist
  global Xv
  global Yv
  global Zv
  global progRunning, VR_angles, stopQueue, splineActive, moveInProc, offlineMode
  progRunning = True
  selRow = tab1.progView.curselection()[0]
  tab1.progView.see(selRow+2)
  data = list(map(int, tab1.progView.curselection()))
  command=tab1.progView.get(data[0]).decode().strip()
  cmdType=command[:6]
  cmdTypeLong=command[:11]
  
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

  ##Run Gcode Program##
  if (cmdType == "Run Gc"):
    if offlineMode:
      almStatusLab.config(text="Gcode not supported in offline programming mode", style="Alarm.TLabel")
      return 
    if (moveInProc == 1):
      moveInProc == 2
    tab1.lastRow = tab1.progView.curselection()[0]
    tab1.lastProg = ProgEntryField.get()
    programIndex = command.find("Program -")
    filename = str(command[programIndex+10:])
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,filename)
    GCplayProg(filename)
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
    if offlineMode:
      almStatusLab.config(text="Test limit switches not supported in offline programming mode", style="Alarm.TLabel")
      return
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
    if offlineMode:
      almStatusLab.config(text="Encoder testing not supported in offline programming mode", style="Alarm.TLabel")
      return
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
    if offlineMode:
      almStatusLab.config(text="Read Encoders not supported in offline programming mode", style="Alarm.TLabel")
      return
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
    if offlineMode:
      almStatusLab.config(text="Servo control not supported in offline programming mode", style="Alarm.TLabel")
      return
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

  ##If Input##
  if (cmdType == "If Inp"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    inputNum = str(command[inputIndex+2:valIndex])
    valNum = int(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    ##querry board for IO value
    cmd = "JFX"+inputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,cmd)   
    ser2.write(cmd.encode())
    ser2.flushInput()
    time.sleep(.1)
    response = str(ser2.readline().strip(),'utf-8')
    if (response == "T"):
      querry = 1
    elif(response == "F"):
      querry = 0
    if(querry == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[progIndex+5:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()
         


  ##Read Com##
  if (cmdType == "Read C"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    comIndex = command.find("# ")
    charIndex = command.find("Char: ")
    actionIndex = command.find(": ")
    comNum = str(command[comIndex+2:charIndex-1])
    charNum = int(command[charIndex+6:])
    try:
      global ser3    
      port = "COM" + comNum   
      baud = 9600    
      ser3 = serial.Serial(port,baud,timeout=10)
    except:
      Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
      tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    ser3.flushInput()
    response = str(ser3.read(charNum).strip(),'utf-8')    
    com3outPortEntryField.delete(0, 'end')
    com3outPortEntryField.insert(0,response)
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)


  
  ##If Register##
  if (cmdType == "If Reg"):
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = int(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    ##querry board for IO value
    regEntry = "R"+inputNum+"EntryField"
    curRegVal = eval(regEntry).get()
    if (int(curRegVal) == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[progIndex+5:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()  

  ##If COM device##
  if (cmdType == "If COM"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    curCOMVal = com3outPortEntryField.get()
    if (curCOMVal == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()           

  ##If Modbus Coil##
  if (cmdType == "If MBc"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    slavestartIndex = command.find("(")
    slaveendIndex = command.find(")")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    slaveID = str(command[slavestartIndex+1:slaveendIndex])
    opVal = "1"
    subcommand = "BB"+"A"+slaveID+"B"+inputNum+"C"+opVal+"\n"
    ser.write(subcommand.encode())
    ser.flushInput()
    time.sleep(.1) 
    response = ser.readline().decode("utf-8").strip()  
    if (response == "Modbus Error"):
      ErrorHandler(response)  
    elif (response == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()   

  ##If Modbus Input##
  if (cmdType == "If MBi"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    slavestartIndex = command.find("(")
    slaveendIndex = command.find(")")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    slaveID = str(command[slavestartIndex+1:slaveendIndex])
    opVal = "1"
    subcommand = "BC"+"A"+slaveID+"B"+inputNum+"C"+opVal+"\n"
    ser.write(subcommand.encode())
    ser.flushInput()
    time.sleep(.1) 
    response = ser.readline().decode("utf-8").strip()  
    if (response == "Modbus Error"):
      ErrorHandler(response)  
    elif (response == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()

  ##If Modbus Holding Register##
  if (cmdType == "If MBh"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    slavestartIndex = command.find("SlaveID (")
    regNumstartIndex = command.find("Num Reg's (")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    slaveID = str(command[slavestartIndex+9:regNumstartIndex-2])
    opVal = str(command[regNumstartIndex+11:inputIndex-8])
    subcommand = "BD"+"A"+slaveID+"B"+inputNum+"C"+opVal+"\n"
    ser.write(subcommand.encode())
    ser.flushInput()
    time.sleep(.1) 
    response = ser.readline().decode("utf-8").strip()  
    if (response == "Modbus Error"):
      ErrorHandler(response)  
    elif (response == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()  

  ##If Modbus Input Register##
  if (cmdType == "If MBI"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    slavestartIndex = command.find("SlaveID (")
    regNumstartIndex = command.find("Num Reg's (")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    slaveID = str(command[slavestartIndex+9:regNumstartIndex-2])
    opVal = str(command[regNumstartIndex+11:inputIndex-14])
    subcommand = "BD"+"A"+slaveID+"B"+inputNum+"C"+opVal+"\n"
    ser.write(subcommand.encode())
    ser.flushInput()
    time.sleep(.1) 
    response = ser.readline().decode("utf-8").strip()  
    if (response == "Modbus Error"):
      ErrorHandler(response)  
    elif (response == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()

  ##Wait 5v IO board##
  if (cmdTypeLong == "Wait 5v Inp"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    timeoutIndex = command.find("Timeout =")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:timeoutIndex-3])
    timeout = str(command[timeoutIndex+10:])
    command = "WI"+"A"+inputNum+"B"+valNum+"C"+timeout+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser2.write(command.encode())
    ser2.flushInput()
    time.sleep(.1)
    ser2.read()
 

  ##Wait Modbus Coil##
  if (cmdTypeLong == "Wait MBcoil"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    timeoutIndex = command.find("Timeout =")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:timeoutIndex-3])
    timeout = str(command[timeoutIndex+10:])
    command = "WJ"+"A"+slaveID+"B"+inputNum+"C"+valNum+"D"+timeout+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.read()

  ##Wait Modbus Input##
  if (cmdTypeLong == "Wait MBinpu"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    timeoutIndex = command.find("Timeout =")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:timeoutIndex-3])
    timeout = str(command[timeoutIndex+10:])
    command = "WK"+"A"+slaveID+"B"+inputNum+"C"+valNum+"D"+timeout+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    ser.read()  
   
              

  

  ##Jump to Row##
  if (cmdType == "Jump T"):
    if (moveInProc == 1):
      moveInProc == 2
    tabIndex = command.find("Tab-")
    tabNum = ("Tab Number " + str(command[tabIndex+4:]) + "\r\n").encode('utf-8')
    index = tab1.progView.get(0, "end").index(tabNum)
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(index)  


  ##Set Output 5v IO Board##
  if (cmdTypeLong == "Set 5v Outp"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    outputIndex = command.find("# ")
    valueIndex = command.find("= ")
    output = str(command[outputIndex+2:valueIndex-1])
    value = str(command[valueIndex+2:])
    if (value == "1"):
      command = "ONX"+output+"\n"
    elif (value == "0"):
      command = "OFX"+output+"\n"  
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser2.write(command.encode())
    ser2.flushInput()
    time.sleep(.1)
    ser2.read()

  ##Set Modbus Coil##
  if (cmdTypeLong == "Set MBcoil "):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:])
    command = "SC"+"A"+slaveID+"B"+inputNum+"C"+valNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = ser.readline().decode("utf-8").strip()
    if (response == "-1"):
      ErrorHandler("Modbus Error")

  ##Set Modbus Register##
  if (cmdTypeLong == "Set MBoutpu"):
    if offlineMode:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:])
    command = "SO"+"A"+slaveID+"B"+inputNum+"C"+valNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = ser.readline().decode("utf-8").strip()
    if (response == "-1"):
      ErrorHandler("Modbus Error")       
 


  ##Wait Time Command##
  if (cmdType == "Wait T"):
    if offlineMode:
      almStatusLab.config(text="Wait time not supported in offline programming mode", style="Alarm.TLabel")
      return
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
    

  ##Calibrate Command##   
  if (cmdType == "Calibr"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotAll()

  ##Calibrate J1##   
  if (cmdType == "Cal_J1"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotJ1()

  ##Calibrate J2##   
  if (cmdType == "Cal_J2"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotJ2() 

  ##Calibrate J1##   
  if (cmdType == "Cal_J3"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotJ3() 

  ##Calibrate J1##   
  if (cmdType == "Cal_J4"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotJ4() 

  ##Calibrate J5##   
  if (cmdType == "Cal_J5"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotJ5() 

  ##Calibrate J6##   
  if (cmdType == "Cal_J6"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotJ6() 

  ##Calibrate J7##   
  if (cmdType == "Cal_J7"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotJ7() 

  ##Calibrate J8##   
  if (cmdType == "Cal_J8"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotJ8()

   ##Calibrate J9##   
  if (cmdType == "Cal_J9"):
    if offlineMode:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (moveInProc == 1):
      moveInProc == 2
    calRobotJ9()                   

  ##Set tool##  
  if (cmdType == "Tool S"):
    if offlineMode:
      almStatusLab.config(text="Set tool not supported in offline programming mode", style="Alarm.TLabel")
      return 
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
    if not vtk_running:
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
    else:
      if not offlineMode:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)







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
    if not vtk_running:
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
    else:
      if not offlineMode:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)  

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
    if not vtk_running:
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
    else:
      if not offlineMode:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)       

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
    if not vtk_running:
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
    else:
      if not offlineMode:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)   

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
    if not vtk_running:
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
    else:
      if not offlineMode:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)  

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
    if not vtk_running:
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
    else:
      if not offlineMode:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05) 




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
    if not vtk_running:
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
    else:
      if not offlineMode:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "RJ"+"A"+J1Val+"B"+J2Val+"C"+J3Val+"D"+J4Val+"E"+J5Val+"F"+J6Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
      rj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)
      
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
      if vtk_running:
        almStatusLab.config(text="Arc move not yet programmed for virtual robot playback", style="Alarm.TLabel")
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
      if vtk_running:
        almStatusLab.config(text="Circle move not yet programmed for virtual robot playback", style="Alarm.TLabel")
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


  VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
  setStepMonitorsVR() 
  progRunning = False  
  rowinproc = 0
  




  
##############################################################################################################################################################
### BUTTON JOGGING DEFS ############################################################################################################## BUTTON JOGGING DEFS ###
##############################################################################################################################################################  

##xbox  ######################################################################################################################################################

################################################################
# New XBOX updates require windows DLL files
# use old method when not on windows


if CE['Platform']['IS_WINDOWS']:
  # ---------- XINPUT (Xbox 360 / Xbox One) - Windows ----------
  for _dll in ("XInput1_4.dll", "XInput9_1_0.dll", "XInput1_3.dll"):
      try:
          _xinput = ctypes.WinDLL(_dll); break
      except OSError:
          _xinput = None
  if _xinput is None:
      raise OSError("XInput DLL not found")

  class XINPUT_GAMEPAD(ctypes.Structure):
      _fields_ = [
          ("wButtons", ctypes.c_ushort),
          ("bLeftTrigger", ctypes.c_ubyte),
          ("bRightTrigger", ctypes.c_ubyte),
          ("sThumbLX", ctypes.c_short),
          ("sThumbLY", ctypes.c_short),
          ("sThumbRX", ctypes.c_short),
          ("sThumbRY", ctypes.c_short),
      ]
  class XINPUT_STATE(ctypes.Structure):
      _fields_ = [("dwPacketNumber", ctypes.c_uint), ("Gamepad", XINPUT_GAMEPAD)]
  XInputGetState = _xinput.XInputGetState
  XInputGetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_STATE)]
  XInputGetState.restype  = ctypes.c_uint

  # ---------- Buttons / DPAD ----------
  BTN_A = 0x1000
  BTN_B = 0x2000
  BTN_X = 0x4000
  BTN_Y = 0x8000
  BTN_START = 0x0010
  BTN_LB = 0x0100
  BTN_RB = 0x0200
  DPAD_UP    = 0x0001
  DPAD_DOWN  = 0x0002
  DPAD_LEFT  = 0x0004
  DPAD_RIGHT = 0x0008

  # ---------- Stick robustness ----------
  DZ_LX = 7849; DZ_LY = 7849
  DZ_RX = 8689 + 1500; DZ_RY = 8689 + 1500
  START_THR_L = 0.18; STOP_THR_L = 0.12
  START_THR_R = 0.22; STOP_THR_R = 0.10
  LPF_ALPHA_L = 0.30; LPF_ALPHA_R = 0.35

  def _norm_axis(v, dz):
      if abs(v) < dz: return 0.0
      n = v / 32767.0
      return -1.0 if n < -1.0 else (1.0 if n > 1.0 else n)

  def _lbl(text, style="Warn.TLabel"):
      try:
          root.after(0, lambda: almStatusLab.config(text=text, style=style))
          root.after(0, lambda: almStatusLab2.config(text=text, style=style))
      except Exception:
          pass

  # ---------- Mode state (A=Joint, B=Cartesian) ----------
  _mainMode = 1
  def _show_mode_banner():
      try:
          _lbl("JOINT MODE" if _mainMode == 1 else "CARTESIAN MODE", style="Warn.TLabel")
      except Exception:
          pass

  # ---------- Tk-thread GUI calls ----------
  def _tk_call(fn, *args):
      if not callable(fn): return False
      try:
          root.after(0, (lambda f=fn, a=args: f(*a)))
          return True
      except Exception:
          return False

  def _gui_stop():
      if _tk_call(globals().get("StopJog"), None):
          return
      try:
          send_serial_command("S\n")
      except Exception:
          pass

  def _gui_start_joint(code):
      _tk_call(globals().get("LiveJointJog"), code)

  def _gui_start_cart(code):
      _tk_call(globals().get("LiveCarJog"), code)

  def _gui_start_tool(code):
      _tk_call(globals().get("LiveToolJog"), code)

  # ----- Teach (X button) -----
  def _teach_position():
      _tk_call(globals().get("teachInsertBelSelected"))

  # ----- Servo gripper toggle (Y button) over ser2 -----
  _grip_closed = False  # False = open; first press closes (SV0P0)

  def _nano_send(cmd):
      def worker():
          try:
              ser2.write(cmd.encode())
              ser2.flushInput()
              time.sleep(0.1)
              ser2.read()
          except Exception:
              pass
      threading.Thread(target=worker, daemon=True).start()

  def _toggle_servo_gripper():
      global _grip_closed
      if not _grip_closed:
          _nano_send("SV0P0\n")   # close
          _grip_closed = True
      else:
          _nano_send("SV0P50\n")  # open
          _grip_closed = False

  # ----- Pneumatic gripper toggle (START) over ser2 -----
  _pneu_open = False  # False = closed

  def _toggle_pneu_gripper():
      global _pneu_open
      if not _pneu_open:
          _nano_send("OFX8\n")   # OPEN
          _pneu_open = True
      else:
          _nano_send("ONX8\n")   # CLOSE
          _pneu_open = False

  # ----- Triggers adjust speedEntryField (smart stepping) -----
  def _bump_speed_smart(delta_hint):
      def do():
          try:
              val = int(speedEntryField.get())
          except Exception:
              val = 25
          if delta_hint < 0:
              # Decrease: above 5 → -5; at/under 5 → -1 (to a floor of 1)
              step = -5 if val > 5 else -1
          else:
              # Increase: below 5 → +1 up to 5; above 5 → +5
              step = +1 if val < 5 else +5
          newv = max(1, min(100, val + step))
          speedEntryField.delete(0, 'end')
          speedEntryField.insert(0, str(newv))
      try:
          root.after(0, do)
      except Exception:
          do()

  # ---------- Joint arbiter ----------
  _current = None
  _pending_start = None
  _last_input_time = 0.0
  SWITCH_DELAY_MS = 60
  WATCHDOG_MS     = 200

  def _lj_code(j, direction):  # J1- = 10, J1+ = 11; J2- = 20, J2+ = 21; ...
      return j*10 + (1 if direction > 0 else 0)

  def _request_switch(new_active):
      global _current, _pending_start
      old = _current
      if old == new_active:
          return

      def do_start_if_pending():
          global _current, _pending_start
          code = _pending_start; _pending_start = None
          if code is not None:
              _current = new_active
              _gui_start_joint(code)

      if old is not None:
          _current = None
          _pending_start = _lj_code(*new_active) if new_active else None
          _gui_stop()
          try: root.after(SWITCH_DELAY_MS, do_start_if_pending)
          except Exception: do_start_if_pending()
          return

      if new_active is not None:
          _current = new_active
          _gui_start_joint(_lj_code(*new_active))

  # ---------- Cartesian arbiter ----------
  _cart_current = None
  _cart_pending = None

  def _cart_code(axis, d):
      if axis == 'X':  return 10 if d < 0 else 11
      if axis == 'Y':  return 20 if d < 0 else 21
      if axis == 'Z':  return 30 if d < 0 else 31
      if axis == 'Rx': return 40 if d < 0 else 41
      if axis == 'Ry': return 50 if d < 0 else 51
      if axis == 'Rz': return 60 if d < 0 else 61
      return None

  def _request_switch_cart(new_active):
      global _cart_current, _cart_pending
      old = _cart_current
      if old == new_active:
          return

      def do_start_if_pending():
          global _cart_current, _cart_pending
          item = _cart_pending; _cart_pending = None
          if item is not None:
              _cart_current = item
              axis, d = item
              code = _cart_code(axis, d)
              if code is not None:
                  _gui_start_cart(code)

      if old is not None:
          _cart_current = None
          _cart_pending = new_active
          _gui_stop()
          try: root.after(SWITCH_DELAY_MS, do_start_if_pending)
          except Exception: do_start_if_pending()
          return

      if new_active is not None:
          _cart_current = new_active
          axis, d = new_active
          code = _cart_code(axis, d)
          if code is not None:
              _gui_start_cart(code)

  # ---------- Tool (Tz) arbiter (for bumpers) ----------
  _tool_current = None
  _tool_pending = None

  def _tool_code(axis, d):
      if axis == 'Tz': return 30 if d < 0 else 31   # per your LiveToolJog mapping
      return None

  def _request_switch_tool(new_active):
      global _tool_current, _tool_pending
      old = _tool_current
      if old == new_active:
          return

      def do_start_if_pending():
          global _tool_current, _tool_pending
          item = _tool_pending; _tool_pending = None
          if item is not None:
              _tool_current = item
              axis, d = item
              code = _tool_code(axis, d)
              if code is not None:
                  _gui_start_tool(code)

      if old is not None:
          _tool_current = None
          _tool_pending = new_active
          _gui_stop()
          try: root.after(SWITCH_DELAY_MS, do_start_if_pending)
          except Exception: do_start_if_pending()
          return

      if new_active is not None:
          _tool_current = new_active
          axis, d = new_active
          code = _tool_code(axis, d)
          if code is not None:
              _gui_start_tool(code)

  # ---------- Watchdog (covers all 3 arbiters) ----------
  def _watchdog_tick():
      global _current, _pending_start, _cart_current, _cart_pending, _tool_current, _tool_pending, _last_input_time
      try:
          now = time.monotonic()
          if (now - _last_input_time) * 1000.0 > WATCHDOG_MS:
              if _current is not None or _cart_current is not None or _tool_current is not None:
                  _current = None; _pending_start = None
                  _cart_current = None; _cart_pending = None
                  _tool_current = None; _tool_pending = None
                  _gui_stop()
      finally:
          try: root.after(WATCHDOG_MS, _watchdog_tick)
          except Exception: pass

  # ---------- Axis selection (one axis per stick) ----------
  _smooth = {'LX': 0, 'LY': 0, 'RX': 0, 'RY': 0}
  def _lp(prev, new, alpha): return int(prev + alpha * (new - prev))

  def _stick_to_axis(raw_x, raw_y, dz_x, dz_y, alpha, start_thr, stop_thr, tag):
      """
      Returns (axis, dir) with axis in {'X','Y',None}, dir in {-1,0,+1}
      (One axis per stick; picks stronger if diagonal.)
      """
      if tag == 'L':
          _smooth['LX'] = _lp(_smooth['LX'], raw_x, alpha)
          _smooth['LY'] = _lp(_smooth['LY'], raw_y, alpha)
          nx = _norm_axis(_smooth['LX'], dz_x); ny = _norm_axis(_smooth['LY'], dz_y)
      else:
          _smooth['RX'] = _lp(_smooth['RX'], raw_x, alpha)
          _smooth['RY'] = _lp(_smooth['RY'], raw_y, alpha)
          nx = _norm_axis(_smooth['RX'], dz_x); ny = _norm_axis(_smooth['RY'], dz_y)

      ix = 1 if nx >= start_thr else (-1 if nx <= -start_thr else 0)
      iy = 1 if ny >= start_thr else (-1 if ny <= -start_thr else 0)

      if ix == 0 and iy == 0:
          return None, 0
      if ix != 0 and iy != 0:
          return ('X', 1 if nx>0 else -1) if abs(nx) >= abs(ny) else ('Y', 1 if ny>0 else -1)
      return ('X', ix) if ix != 0 else ('Y', iy)

  # --- Dominant-axis lock for CARTESIAN left stick (prevents X<->Y flip mid-hold)
  _cartL_lock = {'which': None, 'dir': 0}
  def _cart_left_locked(raw_lx, raw_ly):
      global _smooth
      _smooth['LX'] = int(_smooth['LX'] + LPF_ALPHA_L * (raw_lx - _smooth['LX']))
      _smooth['LY'] = int(_smooth['LY'] + LPF_ALPHA_L * (raw_ly - _smooth['LY']))
      nx = _norm_axis(_smooth['LX'], DZ_LX)
      ny = _norm_axis(_smooth['LY'], DZ_LY)
      ix =  1 if nx >= START_THR_L else (-1 if nx <= -START_THR_L else 0)
      iy =  1 if ny >= START_THR_L else (-1 if ny <= -START_THR_L else 0)
      lock = _cartL_lock
      if lock['which'] == 'X':
          if abs(nx) > STOP_THR_L:
              lock['dir'] = 1 if nx > 0 else -1
              return 'X', lock['dir']
          else:
              lock['which'] = None; lock['dir'] = 0
      elif lock['which'] == 'Y':
          if abs(ny) > STOP_THR_L:
              lock['dir'] = 1 if ny > 0 else -1
              return 'Y', lock['dir']
          else:
              lock['which'] = None; lock['dir'] = 0
      if ix == 0 and iy == 0:
          return None, 0
      if ix != 0 and iy != 0:
          if abs(nx) >= abs(ny):
              lock['which'] = 'X'; lock['dir'] = 1 if nx > 0 else -1
          else:
              lock['which'] = 'Y'; lock['dir'] = 1 if ny > 0 else -1
      elif ix != 0:
          lock['which'] = 'X'; lock['dir'] = ix
      else:
          lock['which'] = 'Y'; lock['dir'] = iy
      return lock['which'], lock['dir']

  # ---------- Controller plumbing ----------
  def _find_controller():
      st = XINPUT_STATE()
      for i in range(4):
          if XInputGetState(i, ctypes.byref(st)) == 0:
              return i
      return None

  def _poll_loop():
      global _mainMode, _last_input_time
      idx = _find_controller()
      if idx is None:
          _lbl("No XInput controller detected"); return
      _lbl(f"Xbox connected (slot {idx})")
      try: root.after(WATCHDOG_MS, _watchdog_tick)
      except Exception: pass

      last_buttons = 0  # for edges X/Y/START/LB/RB
      lt_down = False
      rt_down = False
      TRIG_THR = 30  # analog threshold for a 'press'

      while True:
          st = XINPUT_STATE()
          if XInputGetState(idx, ctypes.byref(st)) != 0:
              _request_switch(None); _request_switch_cart(None); _request_switch_tool(None)
              _lbl("XBOX CONTROLLER NOT RESPONDING", style="Alarm.TLabel")
              time.sleep(0.2)
              idx = _find_controller()
              if idx is not None: _lbl(f"Xbox reconnected (slot {idx})")
              continue

          gp = st.Gamepad
          buttons = gp.wButtons

          # --- Button edges: X (teach), Y (servo gripper), START (pneumatic gripper) ---
          pressed = buttons & ~last_buttons
          if pressed & BTN_X:
              _teach_position()
          if pressed & BTN_Y:
              _toggle_servo_gripper()
          if pressed & BTN_START:
              _toggle_pneu_gripper()

          # --- Triggers: smart speed (edge) ---
          if gp.bLeftTrigger >= TRIG_THR and not lt_down:
              lt_down = True
              _bump_speed_smart(-1)
          elif gp.bLeftTrigger < TRIG_THR and lt_down:
              lt_down = False

          if gp.bRightTrigger >= TRIG_THR and not rt_down:
              rt_down = True
              _bump_speed_smart(+1)
          elif gp.bRightTrigger < TRIG_THR and rt_down:
              rt_down = False

          # --- Mode switching (A=Joint, B=Cartesian) ---
          if buttons & BTN_A:
              if _mainMode != 1:
                  _request_switch(None); _request_switch_cart(None); _request_switch_tool(None)
                  _mainMode = 1; _show_mode_banner()
          elif buttons & BTN_B:
              if _mainMode != 2:
                  _request_switch(None); _request_switch_cart(None); _request_switch_tool(None)
                  _mainMode = 2; _show_mode_banner()

          # --- Tool bumpers (priority over sticks/dpad) ---
          # LB => Tz−, RB => Tz+
          intended_tool = None
          if (buttons & BTN_LB) and not (buttons & BTN_RB):
              intended_tool = ('Tz', -1)
          elif (buttons & BTN_RB) and not (buttons & BTN_LB):
              intended_tool = ('Tz', +1)
          else:
              intended_tool = None

          if intended_tool is not None:
              # tool jog takes priority: stop other modes first
              _request_switch(None)
              _request_switch_cart(None)
              _request_switch_tool(intended_tool)
          else:
              _request_switch_tool(None)

              # --- Movement based on mode (only if no tool jog active) ---
              if _mainMode == 1:
                  # JOINT MODE (custom mapping)
                  axL, dirL = _stick_to_axis(gp.sThumbLX, gp.sThumbLY, DZ_LX, DZ_LY,
                                            LPF_ALPHA_L, START_THR_L, STOP_THR_L, 'L')
                  axR, dirR = _stick_to_axis(gp.sThumbRX, gp.sThumbRY, DZ_RX, DZ_RY,
                                            LPF_ALPHA_R, START_THR_R, STOP_THR_R, 'R')

                  # D-pad: J5 (Down=+1, Up=-1), J6 (Right=+1, Left=-1)
                  dJ5 = (+1 if (buttons & DPAD_DOWN) else -1 if (buttons & DPAD_UP) else 0)
                  dJ6 = (+1 if (buttons & DPAD_RIGHT) else -1 if (buttons & DPAD_LEFT) else 0)

                  intended = None
                  if dJ5 != 0:
                      intended = (5, dJ5)
                  elif dJ6 != 0:
                      intended = (6, dJ6)
                  elif axL is not None:
                      intended = (1, -dirL) if axL == 'X' else (2, -dirL)
                  elif axR is not None:
                      intended = (3, dirR) if axR == 'X' else (4, dirR)

                  _request_switch(intended)

              else:
                  # CARTESIAN MODE (left-stick axis lock)
                  axL, dirL = _cart_left_locked(gp.sThumbLX, gp.sThumbLY)
                  axR, dirR = _stick_to_axis(gp.sThumbRX, gp.sThumbRY, DZ_RX, DZ_RY,
                                            LPF_ALPHA_R, START_THR_R, STOP_THR_R, 'R')

                  # D-pad: Rx / Ry
                  dRx = (+1 if (buttons & DPAD_UP)    else -1 if (buttons & DPAD_DOWN) else 0)
                  dRy = (+1 if (buttons & DPAD_RIGHT) else -1 if (buttons & DPAD_LEFT) else 0)

                  intended_cart = None
                  if dRx != 0:
                      intended_cart = ('Rx', dRx)
                  elif dRy != 0:
                      intended_cart = ('Ry', dRy)
                  elif axL is not None:
                      intended_cart = ('X', dirL) if axL == 'Y' else ('Y', -dirL)
                  elif axR is not None:
                      intended_cart = ('Rz', dirR) if axR == 'X' else ('Z', dirR)

                  _request_switch_cart(intended_cart)

          _last_input_time = time.monotonic()
          last_buttons = buttons
          time.sleep(0.008)  # ~120 Hz

  # ---------- Public entry ----------
  def start_xbox():
      threading.Thread(target=_poll_loop, daemon=True).start()
      _lbl("Xbox ON / polling…", style="Warn.TLabel")

else:
  from inputs import get_gamepad
  def xbox():
    def threadxbox():
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


##end xbox ###################################################################################################################################################

def send_serial_command(cmd):
    global progRunning
    ser.write(cmd.encode())    
    ser.flushInput()
    time.sleep(0.1)
    response = str(ser.readline().strip(), 'utf-8')
    IncJogStatVal = int(IncJogStat.get())
    if IncJogStatVal or progRunning:
      if response[:1] == 'E':
        ErrorHandler(response)
      else:
        displayPosition(response) 		



def start_send_serial_thread(command):
    global progRunning
    if serial_lock.locked():
        logger.warning("Serial command already in progress — ignoring.")
        return
    t = threading.Thread(target=run_send_serial_safe, args=(command,), daemon=True)
    t.start()

def run_send_serial_safe(command):
    global progRunning
    with serial_lock:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        send_serial_command(command)
       

 
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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0])-value)+"B"+str(VR_angles[1])+"C"+str(VR_angles[2])+"D"+str(VR_angles[3])+"E"+str(VR_angles[4])+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)   
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
   

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0])+value)+"B"+str(VR_angles[1])+"C"+str(VR_angles[2])+"D"+str(VR_angles[3])+"E"+str(VR_angles[4])+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)  
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
     

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1]-value)+"C"+str(VR_angles[2])+"D"+str(VR_angles[3])+"E"+str(VR_angles[4])+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

     

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1]+value)+"C"+str(VR_angles[2])+"D"+str(VR_angles[3])+"E"+str(VR_angles[4])+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1])+"C"+str(VR_angles[2]-value)+"D"+str(VR_angles[3])+"E"+str(VR_angles[4])+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1])+"C"+str(VR_angles[2]+value)+"D"+str(VR_angles[3])+"E"+str(VR_angles[4])+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1])+"C"+str(VR_angles[2])+"D"+str(VR_angles[3]-value)+"E"+str(VR_angles[4])+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1])+"C"+str(VR_angles[2])+"D"+str(VR_angles[3]+value)+"E"+str(VR_angles[4])+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command) 

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1])+"C"+str(VR_angles[2])+"D"+str(VR_angles[3])+"E"+str(VR_angles[4]-value)+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1])+"C"+str(VR_angles[2])+"D"+str(VR_angles[3])+"E"+str(VR_angles[4]+value)+"F"+str(VR_angles[5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)   

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1])+"C"+str(VR_angles[2])+"D"+str(VR_angles[3])+"E"+str(VR_angles[4])+"F"+str(VR_angles[5]-value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

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
  global VR_angles
  global offlineMode
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
  commandVR = "RJ"+"A"+str(float(VR_angles[0]))+"B"+str(VR_angles[1])+"C"+str(VR_angles[2])+"D"+str(VR_angles[3])+"E"+str(VR_angles[4])+"F"+str(VR_angles[5]+value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)




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



def start_live_joint_jog_thread(command):
    if live_jog_lock.locked():
        logger.warning("Jog thread already in progress — ignoring.")
        return

    def thread_wrapper():
        with live_jog_lock:
            live_joint_jog(command)

    t = threading.Thread(target=thread_wrapper, daemon=True)
    t.start()


def LiveJointJog(value):
  global xboxUse, liveJog
  liveJog = True
  almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
  almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  checkSpeedVals()
  speedtype = speedOption.get()
  #dont allow mm/sec or sec - switch to percent
  if(speedtype == "mm per Sec" or speedtype == "Seconds"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"25")
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  #!! WC isn't defined prior to use here, at least sometimes
  WC = locals().get("WC", "")
  ############
  command = "LJ"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  start_live_joint_jog_thread(command)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  time.sleep(.1)


def start_live_cartesian_jog_thread(command):
    if live_cartesian_lock.locked():
        logger.warning("Jog thread already in progress — ignoring.")
        return

    def thread_wrapper():
        with live_cartesian_lock:
            live_cartesian_jog(command)

    t = threading.Thread(target=thread_wrapper, daemon=True)
    t.start()


def LiveCarJog(value):
  global xboxUse, liveJog
  liveJog = True
  almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
  almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  checkSpeedVals()
  speedtype = speedOption.get()
  #dont allow mm/sec or sec - switch to percent
  if(speedtype == "mm per Sec" or speedtype == "Seconds"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Sp" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"25")
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
  command = "LC"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
  start_live_cartesian_jog_thread(command)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  time.sleep(.1)


def start_live_tool_jog_thread(command):
    if live_tool_lock.locked():
        logger.warning("Jog thread already in progress — ignoring.")
        return

    def thread_wrapper():
        with live_tool_lock:
            live_tool_jog(command)

    t = threading.Thread(target=thread_wrapper, daemon=True)
    t.start()


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
  start_live_tool_jog_thread(command)
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  time.sleep(.1)



def wait_until_all_locks_free(min_hold_time, timeout=120):
    """
    Wait until all critical locks have been free for at least `min_hold_time` seconds.
    Timeout after `timeout` seconds.
    """
    start_time = time.time()
    stable_start = None

    while True:
        now = time.time()

        # Check all lock statuses
        all_free = not (
            live_jog_lock.locked() or
            live_cartesian_lock.locked() or
            live_tool_lock.locked() or
            drive_lock.locked() or
            serial_lock.locked()
        )

        if all_free:
            if stable_start is None:
                stable_start = now  # begin stability window
            elif now - stable_start >= min_hold_time:
                return  # done waiting
        else:
            stable_start = None  # reset stability window if any lock is active

        if now - start_time > timeout:
            logger.warning("Timeout waiting for locks to be free.")
            return

        time.sleep(0.01)  # poll at 10ms intervals

def wait_until_virtual_locks_free(min_hold_time, timeout=5):
    """
    Wait until all critical locks have been free for at least `min_hold_time` seconds.
    Timeout after `timeout` seconds.
    """
    start_time = time.time()
    stable_start = None

    while True:
        now = time.time()

        # Check all lock statuses
        all_free = not (
            live_jog_lock.locked() or
            live_cartesian_lock.locked() or
            drive_lock.locked()
        )

        if all_free:
            if stable_start is None:
                stable_start = now  # begin stability window
            elif now - stable_start >= min_hold_time:
                return  # done waiting
        else:
            stable_start = None  # reset stability window if any lock is active

        if now - start_time > timeout:
            logger.warning("Timeout waiting for locks to be free.")
            return

        time.sleep(0.01)  # poll at 10ms intervals




def StopJog(self):
  global liveJog, VR_angles
  global J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur
  if liveJog:
    liveJog = False
    time.sleep(.15) 
    if not offlineMode:
      command = "S\n"
      IncJogStatVal = int(IncJogStat.get())
      if (IncJogStatVal == 0):
        ser.write(command.encode()) 
        ser.flushInput()
        time.sleep(.05)
        response = str(ser.readline().strip(),'utf-8')
        if (response[:1] == 'E'):
          ErrorHandler(response)    
        else:
          displayPosition(response)
          VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
          setStepMonitorsVR()   
    else:
      refresh_gui_from_joint_angles(VR_angles)


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
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    XcurPos = XcurPos - value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)



  

def YjogNeg(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    YcurPos = YcurPos - value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)





def ZjogNeg(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    ZcurPos = ZcurPos - value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)  

def RxjogNeg(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    RxcurPos = RxcurPos - value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)  

def RyjogNeg(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    RycurPos = RycurPos - value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)    

def RzjogNeg(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    RzcurPos = RzcurPos - value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)  

def XjogPos(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    XcurPos = XcurPos + value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)   

def YjogPos(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    YcurPos = YcurPos + value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)   


def ZjogPos(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    ZcurPos = ZcurPos + value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)     

def RxjogPos(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    RxcurPos = RxcurPos + value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)   

def RyjogPos(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    RycurPos = RycurPos + value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)   

def RzjogPos(value):
  global xboxUse, XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos, WC, VR_angles
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
  if not offlineMode:
    command = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(VR_angles)
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos = [round(v, 3) for v in xyzuvw]
    RzcurPos = RzcurPos + value
    commandVR = (
        f"MJX{XcurPos:.3f}Y{YcurPos:.3f}Z{ZcurPos:.3f}"
        f"Rz{RzcurPos:.3f}Ry{RycurPos:.3f}Rx{RxcurPos:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{WC}Lm{LoopMode}\n"
    )
    mj_command(commandVR)  

   
  
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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)


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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)




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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

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
  if not offlineMode:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)


  
  
##############################################################################################################################################################  
### TEACH DEFS ################################################################################################################################ TEACH DEFS ###
##############################################################################################################################################################  

def teachInsertBelSelected():
  global XcurPos, YcurPos, ZcurPos, RxcurPos, RycurPos, RzcurPos 
  global J1AngCur, J2AngCur, J3AngCur, J4AngCur, J5AngCur, J6AngCur
  global WC
  global J7PosCur, J8PosCur, J9PosCur
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
    try:
      with open(file_path,'w', encoding='utf-8') as f:
        for item in items:
          f.write(str(item.strip(), encoding='utf-8'))
          f.write('\n')
        f.close()
    except:
      logger.error("No file specified")
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
    for name, val in [
        ("XcurPos", XcurPos),
        ("YcurPos", YcurPos),
        ("ZcurPos", ZcurPos),
        ("RzcurPos", RzcurPos),
        ("RycurPos", RycurPos),
        ("RxcurPos", RxcurPos),
        ("J7PosCur", J7PosCur),
        ("J8PosCur", J8PosCur),
        ("J9PosCur", J9PosCur),
        ("speedPrefix", speedPrefix),
        ("Speed", Speed),
        ("ACCspd", ACCspd),
        ("DECspd", DECspd),
        ("ACCramp", ACCramp),
        ("WC", WC),
    ]:
        if not isinstance(val, str):
            logger.warning(f"{name} is not a string — it is {type(val)}: {val}")
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


def MBreadHoldReg():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BA"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1) 
  response = ser.readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)

def MBreadCoil():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BB"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1) 
  response = ser.readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)

def MBreadInput():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BC"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1) 
  response = ser.readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)

def MBreadInputReg():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BD"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1) 
  response = ser.readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response) 

def MBwriteCoil():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BE"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1) 
  response = ser.readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response) 

   
def MBwriteReg():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BF"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1) 
  response = ser.readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)          

def QueryModbus():
  #command = "HD"+"\n"
  command = "MQ"+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1) 
  response = ser.readline().decode("utf-8").strip()
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,response)

def FaultReset():
  command = "FR"+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1) 
  response = str((ser.readline().strip(),'utf-8'))
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,response)    

  
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



#!! Appears Not to be used
'''
def setOutputOn(): #!! Is this used anywhere?
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
'''

#!! Appears Not to be used
'''
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
'''

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


def IfCMDInsert():
  localErrorFlag = False
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)

  option = iFoption.get()
  selection = iFselection.get()
  variable = IfVarEntryField.get()
  if (variable == ""):
    localErrorFlag = True
    message = "Please enter an input, register number or COM Port" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
  inputVal = IfInputEntryField.get()
  destVal = IfDestEntryField.get()
  if(option == "5v Input"):
    if(inputVal == "1" or inputVal == "0"):
      prefix = "If Input # " + variable + " = " + inputVal + " :"
    else:
      localErrorFlag = True
      message = "Please enter a 1 or 0 for the = value" 
      almStatusLab.config(text=message, style="Alarm.TLabel")

  elif (option == "Register"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter a register number" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If Register # " + variable + " = " + inputVal + " :"

  elif (option == "COM Device"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected COM device input" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If COM Device # " + variable + " = " + inputVal + " :"

  elif (option == "MB Coil"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Coil" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If MBcoil - SlaveID (1) - Coil # " + variable + " = " + inputVal + " :"

  elif (option == "MB Input"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Input" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If MBinput - SlaveID (1) - Input # " + variable + " = " + inputVal + " :"

  elif (option == "MB Hold Reg"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Holding Register" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If MBhold reg - SlaveID (1) Num Reg's (1) - Reg # " + variable + " = " + inputVal + " :"

  elif (option == "MB Input Reg"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Holding Register" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If MBInput Reg - SlaveID (1) Num Reg's (1) - Input Reg # " + variable + " = " + inputVal + " :"          

  if(selection == "Call Prog"):
    if (destVal == ""):
      localErrorFlag = True
      message = "Please enter a program name" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = prefix  + " Call Prog " + destVal
  elif(selection == "Jump Tab"):
    if (destVal == ""):
      localErrorFlag = True
      message = "Please enter a destination tab" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = prefix + " Jump to Tab " + destVal
  elif(selection == "Stop"):
    value = prefix + " Stop " 

  if(localErrorFlag == False):        
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



def WaitCMDInsert():
  localErrorFlag = False
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)

  option = waitoption.get()
  variable = waitVarEntryField.get()
  if (variable == ""):
    localErrorFlag = True
    message = "Please enter an input or Modbus address" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
  inputVal = waitInputEntryField.get()
  timoutVal = waitTimeoutEntryField.get()
  if(option == "5v Input"):
    if(inputVal == "1" or inputVal == "0"):
      value = "Wait 5v Input # " + variable + " = " + inputVal + " : Timeout = " + timoutVal 
    else:
      localErrorFlag = True
      message = "Please enter a 1 or 0 for the = value" 
      almStatusLab.config(text=message, style="Alarm.TLabel")

  elif (option == "MB Coil"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Coil" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = "Wait MBcoil - SlaveID (1) - Coil # " + variable + " = " + inputVal + " : Timeout = " + timoutVal 

  elif (option == "MB Input"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Input" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = "Wait MBinput - SlaveID (1) - Input # " + variable + " = " + inputVal + " : Timeout = " + timoutVal  

  if(localErrorFlag == False):        
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



def SetCMDInsert():
  localErrorFlag = False
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)

  option = setoption.get()
  variable = setVarEntryField.get()
  if (variable == ""):
    localErrorFlag = True
    message = "Please enter an input or Modbus address" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
  inputVal = setInputEntryField.get()
  if(option == "5v Output"):
    if(inputVal == "1" or inputVal == "0"):
      value = "Set 5v Output # " + variable + " = " + inputVal 
    else:
      localErrorFlag = True
      message = "Please enter a 1 or 0 for the = value" 
      almStatusLab.config(text=message, style="Alarm.TLabel")

  elif (option == "MB Coil"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Coil" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = "Set MBcoil - SlaveID (1) - Coil # " + variable + " = " + inputVal

  elif (option == "MB Register"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Register" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = "Set MBoutput - SlaveID (1) - Input # " + variable + " = " + inputVal

  if(localErrorFlag == False):        
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


def ReadAuxCom():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  comNum = auxPortEntryField .get()
  comChar = auxCharEntryField .get()
  servoins = "Read COM # "+comNum+" Char: "+comChar              
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


def TestAuxCom():
  try:
    global ser3    
    port = "COM" + com3PortEntryField.get()     
    baud = 9600    
    ser3 = serial.Serial(port,baud,timeout=5)
  except:
    Curtime = datetime.datetime.now().strftime("%B %d %Y - %I:%M%p")
    tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
  ser3.flushInput()
  numChar = int(com3charPortEntryField.get())
  response = str(ser3.read(numChar).strip(),'utf-8')    
  com3outPortEntryField .delete(0, 'end')
  com3outPortEntryField .insert(0,response)



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
  if getattr(sys, 'frozen', False):
    folder = os.path.dirname(sys.executable)
  elif __file__:
    folder = os.path.dirname(os.path.realpath(__file__))
  #folder = os.path.dirname(os.path.realpath(__file__))
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

def insertGCprog():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  newProg = PlayGCEntryField.get()
  GCProg = "Run Gcode Program - "+newProg            
  tab1.progView.insert(selRow, bytes(GCProg + '\n', 'utf-8')) 
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

#!! Appears not to be used
'''
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
'''

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

#!! Appears not to be used
'''
def TestString():
  message = testSendEntryField.get()
  command = "TM"+message+"\n"
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(0)
  echo = ser.readline()
  testRecEntryField.delete(0, 'end')
  testRecEntryField.insert(0,echo)  
'''

#!! Appears not to be used
'''
def ClearTestString():
  testRecEntryField.delete(0, 'end')
'''

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
  global VR_angles
  success = FALSE
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return 
  ##### STAGE 1 ########
  command = "LL"+"A"+str(J1CalStatVal)+"B"+str(J2CalStatVal)+"C"+str(J3CalStatVal)+"D"+str(J4CalStatVal)+"E"+str(J5CalStatVal)+"F"+str(J6CalStatVal)+"G"+str(J7CalStatVal)+"H"+str(J8CalStatVal)+"I"+str(J9CalStatVal)+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n" 
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
    VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
    setStepMonitorsVR()
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel")
    success = TRUE
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
  if (success == TRUE):
    CalStatVal2 = int(J1CalStatVal2)+int(J2CalStatVal2)+int(J3CalStatVal2)+int(J4CalStatVal2)+int(J5CalStatVal2)+int(J6CalStatVal2)
    if(CalStatVal2>0):
      command = "LL"+"A"+str(J1CalStatVal2)+"B"+str(J2CalStatVal2)+"C"+str(J3CalStatVal2)+"D"+str(J4CalStatVal2)+"E"+str(J5CalStatVal2)+"F"+str(J6CalStatVal2)+"G"+str(J7CalStatVal2)+"H"+str(J8CalStatVal2)+"I"+str(J9CalStatVal2)+"J"+str(J1calOff)+"K"+str(J2calOff)+"L"+str(J3calOff)+"M"+str(J4calOff)+"N"+str(J5calOff)+"O"+str(J6calOff)+"P"+str(J7calOff)+"Q"+str(J8calOff)+"R"+str(J9calOff)+"\n" 
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
        VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
        setStepMonitorsVR()
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
  global VR_angles
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
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
    VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
    setStepMonitorsVR()
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
  global VR_angles
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
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
    VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
    setStepMonitorsVR()
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
  global VR_angles
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
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
    VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
    setStepMonitorsVR()
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
  global VR_angles
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
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
    VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
    setStepMonitorsVR()
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
  global VR_angles
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
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
    VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
    setStepMonitorsVR()
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
  global VR_angles
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
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
    VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
    setStepMonitorsVR()
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
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
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
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
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
  if offlineMode:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
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
  global J1PosLim
  global J1NegLim
  global J2PosLim
  global J2NegLim
  global J3PosLim
  global J3NegLim
  global J4PosLim
  global J4NegLim
  global J5PosLim
  global J5NegLim
  global J6PosLim
  global J6NegLim
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

  update_CPP_kin_from_entries()

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
  ser.flush()
  time.sleep(.1)    
  ser.flushInput()
  time.sleep(.1)
  response = ser.read_all()

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
  global VR_angles
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
  VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
  setStepMonitorsVR()  

def CalRestPos():
  global VR_angles
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
  VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
  setStepMonitorsVR() 




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
  global WC, VR_angles 

  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  J1AngIndex = response.find('A')
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
  
  if J5AngCur.strip() != '' and float(J5AngCur) > 0:
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
  #J7PosCur = float(response[J7PosIndex+1:J8PosIndex].strip());
  #J8PosCur = float(response[J8PosIndex+1:J9PosIndex].strip());
  #J9PosCur = float(response[J9PosIndex+1:].strip());
  J7PosCur = response[J7PosIndex+1:J8PosIndex].strip();
  J8PosCur = response[J8PosIndex+1:J9PosIndex].strip();
  J9PosCur = response[J9PosIndex+1:].strip();
  
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

def LoadAR4Mk3default():
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
  J4PosLimEntryField.insert(0,str(180))
  J4NegLimEntryField.insert(0,str(180))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(180))
  J6NegLimEntryField.insert(0,str(180))  
  J1StepDegEntryField.insert(0,str(88.888))
  J2StepDegEntryField.insert(0,str(111.111)) 
  J3StepDegEntryField.insert(0,str(111.111)) 
  J4StepDegEntryField.insert(0,str(99.555)) 
  J5StepDegEntryField.insert(0,str(43.720)) 
  J6StepDegEntryField.insert(0,str(44.444))
  J1DriveMSEntryField.insert(0,str(800))
  J2DriveMSEntryField.insert(0,str(800))  
  J3DriveMSEntryField.insert(0,str(800))  
  J4DriveMSEntryField.insert(0,str(800))  
  J5DriveMSEntryField.insert(0,str(1600))  
  J6DriveMSEntryField.insert(0,str(800))
  J1EncCPREntryField.insert(0,str(4000))
  J2EncCPREntryField.insert(0,str(4000))
  J3EncCPREntryField.insert(0,str(4000))
  J4EncCPREntryField.insert(0,str(4000))
  J5EncCPREntryField.insert(0,str(4000))
  J6EncCPREntryField.insert(0,str(4000))
  J1ΘEntryField.insert(0,str(0))
  J2ΘEntryField.insert(0,str(-90))
  J3ΘEntryField.insert(0,str(0))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(180))
  J1αEntryField.insert(0,str(0))
  J2αEntryField.insert(0,str(-90))
  J3αEntryField.insert(0,str(0))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(-90))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(41))
  J1aEntryField.insert(0,str(0))
  J2aEntryField.insert(0,str(64.2))
  J3aEntryField.insert(0,str(305))
  J4aEntryField.insert(0,str(0))
  J5aEntryField.insert(0,str(0))
  J6aEntryField.insert(0,str(0)) 

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
  J3ΘEntryField.insert(0,str(0))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(180))
  J1αEntryField.insert(0,str(0))
  J2αEntryField.insert(0,str(-90))
  J3αEntryField.insert(0,str(0))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(-90))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(36.25))
  J1aEntryField.insert(0,str(0))
  J2aEntryField.insert(0,str(64.2))
  J3aEntryField.insert(0,str(305))
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
  J3ΘEntryField.insert(0,str(0))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(180))
  J1αEntryField.insert(0,str(0))
  J2αEntryField.insert(0,str(-90))
  J3αEntryField.insert(0,str(0))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(-90))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(36.25))
  J1aEntryField.insert(0,str(0))
  J2aEntryField.insert(0,str(64.2))
  J3aEntryField.insert(0,str(305))
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
  J3ΘEntryField.insert(0,str(0))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(180))
  J1αEntryField.insert(0,str(0))
  J2αEntryField.insert(0,str(-90))
  J3αEntryField.insert(0,str(0))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(-90))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(36.25))
  J1aEntryField.insert(0,str(0))
  J2aEntryField.insert(0,str(64.2))
  J3aEntryField.insert(0,str(305))
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
  global J7CalStatVal
  global J8CalStatVal
  global J9CalStatVal 
  global J1CalStatVal2
  global J2CalStatVal2
  global J3CalStatVal2
  global J4CalStatVal2
  global J5CalStatVal2
  global J6CalStatVal2
  global J7CalStatVal2
  global J8CalStatVal2
  global J9CalStatVal2
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
  J7CalStatVal = int(J7CalStat.get())
  J8CalStatVal = int(J8CalStat.get())
  J9CalStatVal = int(J9CalStat.get())
  J1CalStatVal2 = int(J1CalStat2.get())
  J2CalStatVal2 = int(J2CalStat2.get())
  J3CalStatVal2 = int(J3CalStat2.get())
  J4CalStatVal2 = int(J4CalStat2.get())
  J5CalStatVal2 = int(J5CalStat2.get())
  J6CalStatVal2 = int(J6CalStat2.get())
  J7CalStatVal2 = int(J7CalStat2.get())
  J8CalStatVal2 = int(J8CalStat2.get())
  J9CalStatVal2 = int(J9CalStat2.get())
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
    logger.error("no serial connection with Teensy board")  
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
  global setColor
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
  # Need the updted field value
  #calibration.insert(END, comPortEntryField.get())
  calibration.insert(END, com1SelectedValue.get())
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
  # Need the updted field value
  #calibration.insert(END, com2PortEntryField.get())
  calibration.insert(END, com2SelectedValue.get())  
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
  calibration.insert(END, GC_ST_WC_EntryField.get())
  calibration.insert(END, J7CalStatVal)
  calibration.insert(END, J8CalStatVal)
  calibration.insert(END, J9CalStatVal)
  calibration.insert(END, J7CalStatVal2)
  calibration.insert(END, J8CalStatVal2)
  calibration.insert(END, J9CalStatVal2)
  calibration.insert(END, setColor)
  

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
  global estopActive
  global posOutreach
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
    GCalmStatusLab.config(text=message, style="Alarm.TLabel")
    #Progstop()
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
      GCalmStatusLab.config(text=message, style="Alarm.TLabel")   
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
      GCalmStatusLab.config(text=message, style="Alarm.TLabel")
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
      GCalmStatusLab.config(text=message, style="Alarm.TLabel")
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
      GCalmStatusLab.config(text=message, style="Alarm.TLabel")
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
      GCalmStatusLab.config(text=message, style="Alarm.TLabel")
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
      GCalmStatusLab.config(text=message, style="Alarm.TLabel")

  ##REACH ERROR   
  elif (response[1:2] == 'R'):
    posOutreach = TRUE
    stopProg()
    message = "Position Out of Reach"
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    GCalmStatusLab.config(text=message, style="Alarm.TLabel")

  ##SPLINE ERROR   
  elif (response[1:2] == 'S'):  
    stopProg()
    message = "Spline Can Only Have Move L Types"
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    GCalmStatusLab.config(text=message, style="Alarm.TLabel")

  ##GCODE ERROR   
  elif (response[1:2] == 'G'):
    stopProg()
    message = "Gcode file not found"
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    GCalmStatusLab.config(text=message, style="Alarm.TLabel")

  ##ESTOP BUTTON   
  elif (response[1:2] == 'B'):
    estopActive = TRUE
    stopProg()
    message = "Estop Button was Pressed"
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    GCalmStatusLab.config(text=message, style="Alarm.TLabel")     

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
     
  ##MODBUS ERROR   
  elif (response == 'Modbus Error'):
    stopProg()
    message = "Modbus Error"
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb")) 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    GCalmStatusLab.config(text=message, style="Alarm.TLabel") 
  
  else:
    stopProg() 
    message = "Unknown Error"
    tab8.ElogView.insert(END, Curtime+" - "+message)
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    GCalmStatusLab.config(text=message, style="Alarm.TLabel")
      
	
	

###VISION DEFS###################################################################
#################################################################################	
 
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
      if (pickClosestVal == highangle and highangle>int(J6PosLim)):
        highangle=J6PosLim
      elif (pickClosestVal == 0 and highangle>int(J6PosLim)):    
        status = "fail"
      if (pickClosestVal == 1 and highangle<(int(J6NegLim)*-1)):
        highangle=J6NegLim*-1
      elif (pickClosestVal == 0 and highangle<(int(J6NegLim)*-1)):  
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
  if getattr(sys, 'frozen', False):
    folder = os.path.dirname(sys.executable)
  elif __file__:
    folder = os.path.dirname(os.path.realpath(__file__))
  #folder = os.path.dirname(os.path.realpath(__file__))
  filelist = [fname for fname in os.listdir(folder) if fname.endswith('.jpg')]
  Visoptmenu = ttk.Combobox(tab6, textvariable=selectedTemplate, values=filelist, state='readonly')
  Visoptmenu.place(x=390, y=52)
  Visoptmenu.bind("<<ComboboxSelected>>", VisOpUpdate)

def VisOpUpdate(foo):
  global selectedTemplate
  file = selectedTemplate.get()
  logger.info(file)
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
  GC_ST_WC_EntryField.delete(0, 'end')
  GC_ST_WC_EntryField.insert(0,str(WC))  

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
  WC = GC_ST_WC_EntryField.get()
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
      tab7.gcodeView.itemconfig(row, {'fg': 'gray'})
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

def GCdelete():
  if(GcodeFilenameField.get() != ""):
    Filename = GcodeFilenameField.get() + ".txt"
    command = "DG"+"Fn"+Filename+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      if(response == "P"):
        text = Filename + " has been deleted"
        GCalmStatusLab.config(text= text,  style="OK.TLabel")
        status = "no"
        GCread(status)
      elif(response == "F"):
        text = Filename + " was not found"
        GCalmStatusLab.config(text= text,  style="Alarm.TLabel")
  else:
    messagebox.showwarning("warning","Please Enter a Filename")

def GCread(status):
  command = "RG"+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  ser.write(command.encode())
  ser.flushInput()
  time.sleep(.1)
  response = str(ser.readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    if(status == "yes"):
      GCalmStatusLab.config(text= "FILES FOUND ON SD CARD:",  style="OK.TLabel")
    GcodeProgEntryField.delete(0, 'end')
    tab7.gcodeView.delete(0,END)
    for value in response.split(","):
      tab7.gcodeView.insert(END,value)
    tab7.gcodeView.pack()
    gcodescrollbar.config(command=tab7.gcodeView.yview)


def GCplay():
  Filename = GcodeFilenameField.get()
  GCplayProg(Filename)

  

def GCplayProg(Filename):
  GCalmStatusLab.config(text= "GCODE FILE RUNNING",  style="OK.TLabel")
  def GCthreadPlay():
    global estopActive
    Fn = Filename + ".txt"
    command = "PG"+"Fn"+Fn+"\n"
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
      if (estopActive == TRUE):
        GCalmStatusLab.config(text= "Estop Button was Pressed",  style="Alarm.TLabel")
      else:  
        GCalmStatusLab.config(text= "GCODE FILE COMPLETE",  style="Warn.TLabel") 
  GCplay = threading.Thread(target=GCthreadPlay)
  GCplay.start()   


def GCconvertProg():
  if(GcodeProgEntryField.get() == ""):
    messagebox.showwarning("warning","Please Load a Gcode Program") 
  elif (GcodeFilenameField.get() == ""):  
    messagebox.showwarning("warning","Please Enter a Filename") 
  else:
    Filename = GcodeFilenameField.get() + ".txt"
    command = "DG"+"Fn"+Filename+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    ser.write(command.encode())
    ser.flushInput()
    time.sleep(.1)
    response = str(ser.readline().strip(),'utf-8')  
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
          GCalmStatusLab.config(text="GCODE CONVERSION STOPPED",  style="Alarm.TLabel")
        else:
          GCalmStatusLab.config(text="GCODE CONVERSION RUNNING",  style="OK.TLabel")
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
          GCalmStatusLab.config(text="GCODE CONVERSION STOPPED",  style="Alarm.TLabel")
    GCt = threading.Thread(target=GCthreadProg)
    GCt.start()    

     


def GCstopProg():
    global cmdType
    global splineActive
    global GCstopQueue
    lastProg = ""
    tab7.GCrunTrue = 0
    GCalmStatusLab.config(text="GCODE CONVERSION STOPPED",  style="Alarm.TLabel")
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
  global GCrowinproc
  global LineDist
  global Xv
  global Yv
  global Zv
  global moveInProc
  global splineActive
  global stopQueue
  global gcodeSpeed
  global inchTrue
  global prevxVal
  global prevyVal
  global prevzVal
  global xVal
  global yVal
  global zVal
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
      WC = GC_ST_WC_EntryField.get()
      LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
      Filename = GcodeFilenameField.get() + ".txt"
      command = "WC"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+WC+"Lm"+LoopMode+"Fn"+Filename+"\n"
      cmdSentEntryField.delete(0, 'end') 


      cmdSentEntryField.insert(0,command)
      ser.write(command.encode())
      ser.flushInput()
      time.sleep(.1)
      response = str(ser.readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)
        GCstopProg()
        tab7.GCrunTrue = 0
        GCalmStatusLab.config(text="UNABLE TO WRITE TO SD CARD",  style="Alarm.TLabel")   
      else:
        displayPosition(response) 


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
      WC = GC_ST_WC_EntryField.get()
      #LoopMode = str(J1OpenLoopStat.get())+str(J2OpenLoopStat.get())+str(J3OpenLoopStat.get())+str(J4OpenLoopStat.get())+str(J5OpenLoopStat.get())+str(J6OpenLoopStat.get())
      LoopMode ="111111"
      #DisWrist = str(DisableWristRot.get())
      Filename = GcodeFilenameField.get() + ".txt"

      command = "WC"+"X"+xVal+"Y"+yVal+"Z"+zVal+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+WC+"Lm"+LoopMode+"Fn"+Filename+"\n"
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
        tab7.GCrunTrue = 0
        GCalmStatusLab.config(text="UNABLE TO WRITE TO SD CARD",  style="Alarm.TLabel")
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

CartjogFrame = Frame(tab1, width=1236, height=792,)
CartjogFrame.place(x=330, y=0)

curRowLab = Label(tab1, text = "Current Row:")
curRowLab.place(x=98, y=120)


almStatusLab = Label(tab1, text = "SYSTEM STARTING - PLEASE WAIT", style="OK.TLabel")
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
J1curAngEntryField = Entry(J1jogFrame,width=4,justify="center")
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
J2curAngEntryField = Entry(J2jogFrame,width=4,justify="center")
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
J3curAngEntryField = Entry(J3jogFrame,width=4,justify="center")
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
J4curAngEntryField = Entry(J4jogFrame,width=4,justify="center")
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
J5curAngEntryField = Entry(J5jogFrame,width=4,justify="center")
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
J6curAngEntryField = Entry(J6jogFrame,width=4,justify="center")
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
J7jogFrame.place(x=980, y=350)
J7Lab = Label(J7jogFrame, font=("Arial", 14), text = "7th Axis")
J7Lab.place(x=15, y=5)
J7curAngEntryField = Entry(J7jogFrame,width=4,justify="center")
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
J8jogFrame.place(x=1160, y=350)
J8Lab = Label(J8jogFrame, font=("Arial", 14), text = "8th Axis")
J8Lab.place(x=15, y=5)
J8curAngEntryField = Entry(J8jogFrame,width=4,justify="center")
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
J9jogFrame.place(x=1340, y=350)
J9Lab = Label(J9jogFrame, font=("Arial", 14), text = "9th Axis")
J9Lab.place(x=15, y=5)
J9curAngEntryField = Entry(J9jogFrame,width=4,justify="center")
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



incrementEntryField = Entry(tab1,width=4,justify="center")
incrementEntryField.place(x=380, y=45)

curRowEntryField = Entry(tab1,width=4,justify="center")
curRowEntryField.place(x=174, y=120)

manEntryField = Entry(tab1,width=105)
manEntryField.place(x=10, y=700)

ProgEntryField = Entry(tab1,width=20,justify="center")
ProgEntryField.place(x=70, y=45)



speedEntryField = Entry(tab1,width=4,justify="center")
speedEntryField.place(x=380, y=80)

ACCspeedField = Entry(tab1,width=4,justify="center")
ACCspeedField.place(x=380, y=100)

DECspeedField = Entry(tab1,width=4,justify="center")
DECspeedField.place(x=380, y=120)

ACCrampField = Entry(tab1,width=4,justify="center")
ACCrampField.place(x=380, y=140)

roundEntryField = Entry(tab1,width=4,justify="center")
roundEntryField.place(x=590, y=80)














  ### X ###

XcurEntryField = Entry(CartjogFrame,width=4,justify="center")
XcurEntryField.place(x=660, y=195)


   ### Y ###

YcurEntryField = Entry(CartjogFrame,width=4,justify="center")
YcurEntryField.place(x=750, y=195)


   ### Z ###

ZcurEntryField = Entry(CartjogFrame,width=4,justify="center")
ZcurEntryField.place(x=840, y=195)


   ### Rz ###

RzcurEntryField = Entry(CartjogFrame,width=4,justify="center")
RzcurEntryField.place(x=930, y=195)


   ### Ry ###

RycurEntryField = Entry(CartjogFrame,width=4,justify="center")
RycurEntryField.place(x=1020, y=195)


   ### Rx ###

RxcurEntryField = Entry(CartjogFrame,width=4,justify="center")
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
speedMenu.place(x=417, y=76)



#single buttons

options=StringVar(tab1)
menu=OptionMenu(tab1, options, "Move J", "Move J", "OFF J", "Move L", "Move R", "Move A Mid", "Move A End", "Move C Center", "Move C Start", "Move C Plane", "Start Spline", "End Spline", "Move PR", "OFF PR ", "Teach PR", "Move Vis", command=posRegFieldVisible)
menu.grid(row=2,column=2)
menu.config(width=18)
menu.place(x=700, y=180)

SavePosEntryField = Entry(tab1,width=4,justify="center")
#SavePosEntryField.place(x=800, y=183)


teachInsBut = Button(tab1,  text="Teach New Position", width=22, command = teachInsertBelSelected)
teachInsBut.place(x=700, y=220)

teachReplaceBut = Button(tab1, text="Modify Position", width=22,  command = teachReplaceSelected)
teachReplaceBut.place(x=700, y=260)

deleteBut = Button(tab1,  text="Delete", width=22,   command = deleteitem)
deleteBut.place(x=700, y=300)

returnBut = Button(tab1,  text="Return",  width=22,   command = insertReturn)
returnBut.place(x=700, y=340)

CalibrateBut = Button(tab1,  text="Auto Calibrate CMD", width=22,   command = insCalibrate)
CalibrateBut.place(x=700, y=380)

camOnBut = Button(tab1,  text="Camera On",  width=22, command = cameraOn)
camOnBut.place(x=700, y=420)

camOffBut = Button(tab1,  text="Camera Off",  width=22, command = cameraOff)
camOffBut.place(x=700, y=460)



#buttons with 1 entry

waitTimeBut = Button(tab1, text="Wait Time (seconds)",  width=22,  command = waitTime)
waitTimeBut.place(x=700, y=500)



#setOutputOnBut = Button(tab1,  text="Set Output On",  width=22,  command = setOutputOn)
#setOutputOnBut.place(x=700, y=580)

#setOutputOffBut = Button(tab1,  text="Set Output OFF",  width=22,   command = setOutputOff)
#setOutputOffBut.place(x=700, y=620)

tabNumBut = Button(tab1,  text="Create Tab",  width=22, command = tabNumber)
tabNumBut.place(x=700, y=540)

jumpTabBut = Button(tab1,  text="Jump to Tab",  width=22, command = jumpTab)
jumpTabBut.place(x=700, y=580)




waitTimeEntryField = Entry(tab1,width=4,justify="center")
waitTimeEntryField.place(x=855, y=505)

#waitInputEntryField = Entry(tab1,width=4,justify="center")
#waitInputEntryField.place(x=855, y=505)

#waitInputOffEntryField = Entry(tab1,width=4,justify="center")
#waitInputOffEntryField.place(x=855, y=545)

#outputOnEntryField = Entry(tab1,width=4,justify="center")
#outputOnEntryField.place(x=855, y=585)

#outputOffEntryField = Entry(tab1,width=4,justify="center")
#outputOffEntryField.place(x=855, y=625)

tabNumEntryField = Entry(tab1,width=4,justify="center")
tabNumEntryField.place(x=855, y=545)

jumpTabEntryField = Entry(tab1,width=4,justify="center")
jumpTabEntryField.place(x=855, y=585)

### SERVO BUTTON ###
servoBut = Button(tab1,  text="Servo",  width=22,   command = Servo)
servoBut.place(x=700, y=625)

servoNumEntryField = Entry(tab1,width=4,justify="center")
servoNumEntryField.place(x=855, y=630)

servoPosEntryField = Entry(tab1,width=4,justify="center")
servoPosEntryField.place(x=895, y=630)

servoLab = Label(tab1,font=("Arial", 6), text = "Number      Position")
servoLab.place(x=855, y=615)


### REGISTER BUTTON ###
RegNumBut = Button(tab1,  text="Register",  width=22,   command = insertRegister)
RegNumBut.place(x=700, y=665)

regNumEntryField = Entry(tab1,width=4,justify="center")
regNumEntryField.place(x=855, y=670)

regEqEntryField = Entry(tab1,width=4,justify="center")
regEqEntryField.place(x=895, y=670)

regEqLab = Label(tab1,font=("Arial", 6), text = "Register      (++/--)")
regEqLab.place(x=855, y=655)


### VISION FIND BUTTON ###
visFindBut = Button(tab1,  text="Vision Find",  width=22,   command = insertvisFind)
visFindBut.place(x=700, y=705)

visPassEntryField = Entry(tab1,width=4,justify="center")
visPassEntryField.place(x=855, y=710)

visFailEntryField = Entry(tab1,width=4,justify="center")
visFailEntryField.place(x=895, y=710)

visPassLab = Label(tab1,font=("Arial", 6), text = "Pass Tab     Fail Tab")
visPassLab.place(x=855, y=695)



### IF THEN STATEMENT ###
ifThenFrame = Frame(tab1, width=520, height=40,)
ifThenFrame.place(x=955, y=470)

ifSelLab = Label(ifThenFrame,font=("Arial 10 bold"), text = "IF")
ifSelLab.place(x=0, y=5)

iFoption=StringVar(ifThenFrame)
iFmenu=OptionMenu(ifThenFrame, iFoption, "5v Input", "5v Input", "Register", "COM Device", "MB Coil", "MB Input", "MB Hold Reg", "MB Input Reg")
iFmenu.grid(row=2,column=2)
iFmenu.config(width=12)
iFmenu.place(x=20, y=2)

IfVarEntryField = Entry(ifThenFrame,width=4,justify="center")
IfVarEntryField.place(x=140, y=5)

ifEqualLab = Label(ifThenFrame,font=("Arial 10 bold"), text = "=")
ifEqualLab.place(x=177, y=5)

IfInputEntryField = Entry(ifThenFrame,width=4,justify="center")
IfInputEntryField.place(x=195, y=5)

iFselection=StringVar(ifThenFrame)
iFSelmenu=OptionMenu(ifThenFrame, iFselection, "Call Prog", "Call Prog", "Jump Tab", "Stop")
iFSelmenu.grid(row=2,column=2)
iFSelmenu.config(width=9)
iFSelmenu.place(x=240, y=2)

IfDestEntryField = Entry(ifThenFrame,width=9,justify="center")
IfDestEntryField.place(x=340, y=5)

ifEqualLab = Label(ifThenFrame,font=("Arial 10 bold"), text = "•")
ifEqualLab.place(x=405, y=5)

insertIfCMDBut = Button(ifThenFrame,  text="Insert IF CMD",  width=12,   command = IfCMDInsert)
insertIfCMDBut.place(x=420, y=0)






### WAIT STATEMENT ###
waitFrame = Frame(tab1, width=550, height=40,)
waitFrame.place(x=935, y=510)

waitSelLab = Label(waitFrame,font=("Arial 10 bold"), text = "WAIT")
waitSelLab.place(x=0, y=5)

waitoption=StringVar(waitFrame)
waitmenu=OptionMenu(waitFrame, waitoption, "5v Input", "5v Input", "MB Coil", "MB Input")
waitmenu.grid(row=2,column=2)
waitmenu.config(width=12)
waitmenu.place(x=40, y=2)

waitVarEntryField = Entry(waitFrame,width=4,justify="center")
waitVarEntryField.place(x=160, y=5)

waitEqualLab = Label(waitFrame,font=("Arial 10 bold"), text = "=")
waitEqualLab.place(x=197, y=5)

waitInputEntryField = Entry(waitFrame,width=4,justify="center")
waitInputEntryField.place(x=215, y=5)

waitTimoutEqualLab = Label(waitFrame,font=("Arial 10 bold"), text = "Timeout =")
waitTimoutEqualLab.place(x=260, y=5)

waitTimeoutEntryField = Entry(waitFrame,width=5,justify="center")
waitTimeoutEntryField.place(x=335, y=5)

waitEqualLab = Label(waitFrame,font=("Arial 10 bold"), text = "•")
waitEqualLab.place(x=385, y=5)

insertwaitCMDBut = Button(waitFrame,  text="Insert WAIT CMD",  width=18,   command = WaitCMDInsert)
insertwaitCMDBut.place(x=403, y=0)






### SET STATEMENT ###
setFrame = Frame(tab1, width=500, height=40,)
setFrame.place(x=935, y=550)

setSelLab = Label(setFrame,font=("Arial 10 bold"), text = "SET")
setSelLab.place(x=7, y=5)

setoption=StringVar(setFrame)
setmenu=OptionMenu(setFrame, setoption, "5v Output", "5v Output", "MB Coil", "MB Register")
setmenu.grid(row=2,column=2)
setmenu.config(width=12)
setmenu.place(x=40, y=2)

setVarEntryField = Entry(setFrame,width=4,justify="center")
setVarEntryField.place(x=160, y=5)

setEqualLab = Label(setFrame,font=("Arial 10 bold"), text = "=")
setEqualLab.place(x=197, y=5)

setInputEntryField = Entry(setFrame,width=4,justify="center")
setInputEntryField.place(x=215, y=5)

setEqualLab = Label(setFrame,font=("Arial 10 bold"), text = "•")
setEqualLab.place(x=260, y=5)

insertsetCMDBut = Button(setFrame,  text="Insert set CMD",  width=18,   command = SetCMDInsert)
insertsetCMDBut.place(x=278, y=0)




#buttons with multiple entry

### READ COM DEVICE ###
readCOMFrame = Frame(tab1, width=450, height=45,)
readCOMFrame.place(x=975, y=692)

readAuxComBut = Button(readCOMFrame,  text="Read COM Device",  width=22,   command = ReadAuxCom)
readAuxComBut.place(x=0, y=13)

auxPortEntryField = Entry(readCOMFrame,width=4,justify="center")
auxPortEntryField.place(x=160, y=17)

auxCharEntryField = Entry(readCOMFrame,width=4,justify="center")
auxCharEntryField.place(x=200, y=17)

auxComLab = Label(readCOMFrame,font=("Arial", 6), text = "Port             Char")
auxComLab.place(x=160, y=3)



### READ POSITION REG ###
posRegFrame = Frame(tab1, width=450, height=45,)
posRegFrame.place(x=975, y=652)

StorPosBut = Button(posRegFrame,  text="Position Register",  width=22,   command = storPos)
StorPosBut.place(x=0, y=13)

storPosNumEntryField = Entry(posRegFrame,width=4,justify="center")
storPosNumEntryField.place(x=160, y=17)

storPosElEntryField = Entry(posRegFrame,width=4,justify="center")
storPosElEntryField.place(x=200, y=17)

storPosValEntryField = Entry(posRegFrame,width=4,justify="center")
storPosValEntryField.place(x=240, y=17)

storPosEqLab = Label(posRegFrame,font=("Arial", 6), text = "Pos Reg      Element       (++/--)")
storPosEqLab.place(x=158, y=3)







### CALL PROGRAM ###
callBut = Button(tab1,  text="Call Program",  width=16, command = insertCallProg)
callBut.place(x=975, y=625)

changeProgEntryField = Entry(tab1,width=14, justify="center")
changeProgEntryField.place(x=1095, y=630)



### PLAY GCODE BUTTON ###
GCplayBut = Button(tab1,  text="Play Gcode",  width=16, command = insertGCprog)
GCplayBut.place(x=1213, y=625)

PlayGCEntryField = Entry(tab1,width=14, justify="center")
PlayGCEntryField.place(x=1333, y=630)





manEntLab = Label(tab1, font=("Arial", 6), text = "Manual Program Entry")
manEntLab.place(x=10, y=685)

ProgBut = Button(tab1,  text="Load", width=10,  command = loadProg)
ProgBut.place(x=202, y=42)

CreateBut = Button(tab1,  text="New Prog", width=10,  command = CreateProg)
CreateBut.place(x=285, y=42)

VirtualRobBut = Button(tab1,  text="Virtual Robot", width=20,  command=lambda: launch_vtk_nonblocking(tab1))
VirtualRobBut.place(x=485, y=125)

offline_button = ttk.Button(tab1, text="Run Offline", width=20, command=toggle_offline_mode, style="Online.TButton")
offline_button.place(x=635, y=125)



runProgBut = Button(tab1,   command = runProg)
playPhoto=PhotoImage(file="play-icon.png")
runProgBut.config(image=playPhoto)
runProgBut.place(x=20, y=80)

if CE['Platform']['IS_WINDOWS']: # Use old Xbox method if not on Windows
  xboxBut = Button(tab1,  command = start_xbox)
else:
  xboxBut = Button(tab1,  command = xbox)

xboxPhoto=PhotoImage(file="xbox.png")
xboxBut.config(image=xboxPhoto)
xboxBut.place(x=700, y=40)

stopProgBut = Button(tab1,   command = stopProg)
stopPhoto=PhotoImage(file="stop-icon.png")
stopProgBut.config(image=stopPhoto)
stopProgBut.place(x=220, y=80)

revBut = Button(tab1,  text="REV ",  command = stepRev)
revBut.place(x=105, y=80)

fwdBut = Button(tab1,  text="FWD", command = stepFwd)
fwdBut.place(x=160, y=80)

IncJogCbut = Checkbutton(tab1, text="Incremental Jog",variable = IncJogStat)
IncJogCbut.place(x=417, y=48)


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

ComPortLab2 = Label(tab2, text = "5v IO BOARD COM PORT:")
ComPortLab2.place(x=60, y=160)

almStatusLab2 = Label(tab2, text = "SYSTEM STARTING - PLEASE WAIT", style="OK.TLabel")
almStatusLab2.place(x=25, y=20)




comLab = Label(tab2, font=("Arial 10 bold"), text = "Communication")
comLab.place(x=69, y=60)

jointCalLab = Label(tab2, font=("Arial 10 bold"), text = "Robot Calibration")
jointCalLab.place(x=280, y=60)

axis7Lab = Label(tab2, font=("Arial 10 bold"), text = "7th Axis Calibration")
axis7Lab.place(x=655, y=300)

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





axis8Lab = Label(tab2, font=("Arial 10 bold"), text = "8th Axis Calibration")
axis8Lab.place(x=855, y=300)

axis8lengthLab = Label(tab2, text = "8th Axis Length:")
axis8lengthLab.place(x=851, y=340)

axis8rotLab = Label(tab2, text = "MM per Rotation:")
axis8rotLab.place(x=845, y=370)

axis8stepsLab = Label(tab2, text = "Drive Steps:")
axis8stepsLab.place(x=875, y=400)


axis9Lab = Label(tab2, font=("Arial 10 bold"), text = "9th Axis Calibration")
axis9Lab.place(x=1055, y=300)

axis9lengthLab = Label(tab2, text = "9th Axis Length:")
axis9lengthLab.place(x=1051, y=340)

axis9rotLab = Label(tab2, text = "MM per Rotation:")
axis9rotLab.place(x=1045, y=370)

axis9stepsLab = Label(tab2, text = "Drive Steps:")
axis9stepsLab.place(x=1075, y=400)





CalibrationOffsetsLab = Label(tab2, font=("Arial 10 bold"), text = "Calibration Offsets")
CalibrationOffsetsLab.place(x=476, y=60)

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





CalibrationOffsetsLab = Label(tab2, font=("Arial 10 bold"), text = "Encoder Control")
CalibrationOffsetsLab.place(x=705, y=60)

cmdSentLab = Label(tab2, text = "Last Command Sent to Controller")
cmdSentLab.place(x=10, y=565)

cmdRecLab = Label(tab2, text = "Last Response From Controller")
cmdRecLab.place(x=10, y=625)

ThemeLab = Label(tab2, font=("Arial 10 bold"), text = "Theme")
ThemeLab.place(x=920, y=60)


### 2 BUTTONS################################################################
#############################################################################

#############################################################################
# Switching comport entry fields to self-populating dropdowns

#comPortBut = Button(tab2,  text="  Set Com Teensy  ",   command = setCom)
#comPortBut.place(x=85, y=110)


def detect_ports():
  # Imporve to actually query and detect boards on found com ports

  ports = list(list_ports.comports())
  choices = [p.device for p in ports]

  if 'comPort' in locals() and comPort not in ("", None):
    port1_default = comPort
  else:
    port1_default = None

  if 'com2Port' in locals() and com2Port not in ("", None):
    port2_default = com2Port
  else:
    port2_default = None

  return choices, port1_default, port2_default

port_choices, default_comport1, default_comport2 = detect_ports()

logger.debug(f"Available Comm Ports: {port_choices}")

com1SelectedValue = tk.StringVar(value=default_comport1)
com1Select = tk.OptionMenu(tab2, com1SelectedValue, *port_choices, command = setCom)
com1Select.place(x=75, y=110)

com2SelectedValue = tk.StringVar(value=default_comport2)
com2Select = tk.OptionMenu(tab2, com2SelectedValue, *port_choices, command = setCom2)
com2Select.place(x=75, y=180)

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

J7calCbut = Checkbutton(tab2, text="J7",variable = J7CalStat)
J7calCbut.place(x=285, y=165)

J8calCbut = Checkbutton(tab2, text="J8",variable = J8CalStat)
J8calCbut.place(x=320, y=165)

J9calCbut = Checkbutton(tab2, text="J9",variable = J9CalStat)
J9calCbut.place(x=355, y=165)


J1calCbut2 = Checkbutton(tab2, text="J1",variable = J1CalStat2)
J1calCbut2.place(x=285, y=200)

J2calCbut2 = Checkbutton(tab2, text="J2",variable = J2CalStat2)
J2calCbut2.place(x=320, y=200)

J3calCbut2 = Checkbutton(tab2, text="J3",variable = J3CalStat2)
J3calCbut2.place(x=355, y=200)

J4calCbut2 = Checkbutton(tab2, text="J4",variable = J4CalStat2)
J4calCbut2.place(x=285, y=220)

J5calCbut2 = Checkbutton(tab2, text="J5",variable = J5CalStat2)
J5calCbut2.place(x=320, y=220)

J6calCbut2 = Checkbutton(tab2, text="J6",variable = J6CalStat2)
J6calCbut2.place(x=355, y=220)

J7calCbut2 = Checkbutton(tab2, text="J7",variable = J7CalStat2)
J7calCbut2.place(x=285, y=240)

J8calCbut2 = Checkbutton(tab2, text="J8",variable = J8CalStat2)
J8calCbut2.place(x=320, y=240)

J9calCbut2 = Checkbutton(tab2, text="J9",variable = J9CalStat2)
J9calCbut2.place(x=355, y=240)





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
CalJ1But.place(x=285, y=275)

CalJ2But = Button(tab2,   text="Calibrate J2 Only",   command = calRobotJ2)
CalJ2But.place(x=285, y=310)

CalJ3But = Button(tab2,   text="Calibrate J3 Only",   command = calRobotJ3)
CalJ3But.place(x=285, y=345)

CalJ4But = Button(tab2,   text="Calibrate J4 Only",   command = calRobotJ4)
CalJ4But.place(x=285, y=380)

CalJ5But = Button(tab2,   text="Calibrate J5 Only",   command = calRobotJ5)
CalJ5But.place(x=285, y=415)

CalJ6But = Button(tab2,   text="Calibrate J6 Only",   command = calRobotJ6)
CalJ6But.place(x=285, y=450)

CalZeroBut = Button(tab2,   text="Force CaL to Home",  width=20,   command = CalZeroPos)
CalZeroBut.place(x=270, y=485)

CalRestBut = Button(tab2,   text="Force Cal to Rest",  width=20,   command = CalRestPos)
CalRestBut.place(x=270, y=520)




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


#comPortEntryField = Entry(tab2,width=4,justify="center")
#comPortEntryField.place(x=50, y=114)

#com2PortEntryField = Entry(tab2,width=4,justify="center")
#com2PortEntryField.place(x=50, y=184)


cmdSentEntryField = Entry(tab2,width=95,justify="center")
cmdSentEntryField.place(x=10, y=585)

cmdRecEntryField = Entry(tab2,width=95,justify="center")
cmdRecEntryField.place(x=10, y=645)


J1calOffEntryField = Entry(tab2,width=5,justify="center")
J1calOffEntryField.place(x=540, y=90)

J2calOffEntryField = Entry(tab2,width=5,justify="center")
J2calOffEntryField.place(x=540, y=120)

J3calOffEntryField = Entry(tab2,width=5,justify="center")
J3calOffEntryField.place(x=540, y=150)

J4calOffEntryField = Entry(tab2,width=5,justify="center")
J4calOffEntryField.place(x=540, y=180)

J5calOffEntryField = Entry(tab2,width=5,justify="center")
J5calOffEntryField.place(x=540, y=210)

J6calOffEntryField = Entry(tab2,width=5,justify="center")
J6calOffEntryField.place(x=540, y=240)

J7calOffEntryField = Entry(tab2,width=5,justify="center")
J7calOffEntryField.place(x=540, y=280)

J8calOffEntryField = Entry(tab2,width=5,justify="center")
J8calOffEntryField.place(x=540, y=310)

J9calOffEntryField = Entry(tab2,width=5,justify="center")
J9calOffEntryField.place(x=540, y=340)



axis7lengthEntryField = Entry(tab2,width=5,justify="center")
axis7lengthEntryField.place(x=750, y=340)

axis7rotEntryField = Entry(tab2,width=5,justify="center")
axis7rotEntryField.place(x=750, y=370)

axis7stepsEntryField = Entry(tab2,width=5,justify="center")
axis7stepsEntryField.place(x=750, y=400)

axis8lengthEntryField = Entry(tab2,width=5,justify="center")
axis8lengthEntryField.place(x=950, y=340)

axis8rotEntryField = Entry(tab2,width=5,justify="center")
axis8rotEntryField.place(x=950, y=370)

axis8stepsEntryField = Entry(tab2,width=5,justify="center")
axis8stepsEntryField.place(x=950, y=400)

axis9lengthEntryField = Entry(tab2,width=5,justify="center")
axis9lengthEntryField.place(x=1150, y=340)

axis9rotEntryField = Entry(tab2,width=5,justify="center")
axis9rotEntryField.place(x=1150, y=370)

axis9stepsEntryField = Entry(tab2,width=5,justify="center")
axis9stepsEntryField.place(x=1150, y=400)


#########################################################################################
#########################################################################################
############  VIRTUAL ROBOT COLORS
# Color map for STL files


# STL files including Link 4-2.STL
stl_files = [
    "Link Base-1.STL", "Link Base-2.STL", "Link Base-3.STL",
    "Link 1-1.STL", "Link 1-2.STL",
    "Link 2-1.STL", "Link 2-2.STL", "Link 2-3.STL",
    "Link 3-1.STL", "Link 3-2.STL",
    "Link 4-1.STL", "Link 4-2.STL", "Link 4-3.STL",
    "Link 5-1.STL", "Link 5-2.STL",
    "Link 6-1.STL", "Link 6-2.STL"
]



color_map = {stl: "Silver" for stl in stl_files}
color_map.update({
    "Link Base-2.STL": "Orange",
    "Link Base-3.STL": "DimGray",
    "Link 1-2.STL": "DimGray",
    "Link 2-2.STL": "Orange", "Link 2-3.STL": "DimGray",
    "Link 3-2.STL": "DimGray",
    "Link 4-2.STL": "Orange", "Link 4-3.STL": "DimGray",
    "Link 5-2.STL": "DimGray",
    "Link 6-2.STL": "DimGray"
})

main_color_parts = ["Link Base-2.STL", "Link 2-2.STL", "Link 4-2.STL"]
logo_color_parts = ["Link 2-3.STL", "Link 4-3.STL"]

def update_main_color(*args):
    global setColor
    selected = main_color_var.get()
    setColor = selected
    for part in main_color_parts:
        color_map[part] = selected 
        actors[part].GetProperty().SetColor(vtk.vtkNamedColors().GetColor3d(selected))
    render_window.Render()

# Color options
color_options = [
    "Red", "IndianRed", "Crimson", "FireBrick", "DarkRed", "Maroon",
    "RosyBrown", "MediumVioletRed", "DeepPink", "HotPink", "Orchid", "Magenta",
    "Orange", "DarkOrange", "Tomato", "Gold", "Yellow", "Chartreuse", "YellowGreen",
    "Green", "LimeGreen", "MediumSpringGreen", "DarkOliveGreen", 
    "Teal", "DarkTurquoise", "Turquoise", "CadetBlue",
    "DodgerBlue", "Blue", "RoyalBlue", "SlateBlue", "MediumSlateBlue", 
    "Navy", "MidnightBlue", "SteelBlue",
    "Black", "DimGray", "DarkGray", "Gray", "Silver", 
    "LightSlateGray", "LightSteelBlue",
    "White", "Gainsboro", "AntiqueWhite", "Cornsilk"
]
# Dropdowns
main_color_var = tk.StringVar(value="Royal Blue")

RobotColors = Label(tab2, font=("Arial 10 bold"), text = "Robot Color")
RobotColors.place(x=1090, y=60)

main_color_dropdown = ttk.OptionMenu(tab2, main_color_var, main_color_var.get(), *color_options, command=update_main_color)
main_color_dropdown.place(x=1090, y=90)




################### VIRTUAL ROBOT IMPORT STL ###################

ImportLabel = Label(tab2, font=("Arial 10 bold"), text="Import Models")
ImportLabel.place(x=1270, y=60)  # Unchanged

# Import STL button
ttk.Button(tab2, text="Import STL", command=import_stl_file).place(x=1270, y=90)

# File name label + entry
Label(tab2, text="File Name").place(x=1270, y=125)
Entry(tab2, textvariable=stl_name_var, width=20).place(x=1270, y=145)

# X position
Label(tab2, text="X Position").place(x=1270, y=175)
Entry(tab2, textvariable=x_var, width=10).place(x=1270, y=195)

# Y position
Label(tab2, text="Y Position").place(x=1270, y=225)
Entry(tab2, textvariable=y_var, width=10).place(x=1270, y=245)

# Z position
Label(tab2, text="Z Position").place(x=1270, y=275)
Entry(tab2, textvariable=z_var, width=10).place(x=1270, y=295)

# Rotation
Label(tab2, text="Z Rotation").place(x=1270, y=325)
Entry(tab2, textvariable=rot_var, width=10).place(x=1270, y=345)

# Update button
ttk.Button(tab2, text="Update Position", command=update_stl_transform).place(x=1270, y=380)



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

TFxEntryField = Entry(tab3,width=4,justify="center")
TFxEntryField.place(x=910, y=115)
TFyEntryField = Entry(tab3,width=4,justify="center")
TFyEntryField.place(x=950, y=115)
TFzEntryField = Entry(tab3,width=4,justify="center")
TFzEntryField.place(x=990, y=115)
TFrzEntryField = Entry(tab3,width=4,justify="center")
TFrzEntryField.place(x=1030, y=115)
TFryEntryField = Entry(tab3,width=4,justify="center")
TFryEntryField.place(x=1070, y=115)
TFrxEntryField = Entry(tab3,width=4,justify="center")
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

J1MotDirEntryField = Entry(tab3,width=5,justify="center")
J1MotDirEntryField.place(x=110, y=20)
J2MotDirEntryField = Entry(tab3,width=5,justify="center")
J2MotDirEntryField.place(x=110, y=45)
J3MotDirEntryField = Entry(tab3,width=5,justify="center")
J3MotDirEntryField.place(x=110, y=70)
J4MotDirEntryField = Entry(tab3,width=5,justify="center")
J4MotDirEntryField.place(x=110, y=95)
J5MotDirEntryField = Entry(tab3,width=5,justify="center")
J5MotDirEntryField.place(x=110, y=120)
J6MotDirEntryField = Entry(tab3,width=5,justify="center")
J6MotDirEntryField.place(x=110, y=145)
J7MotDirEntryField = Entry(tab3,width=5,justify="center")
J7MotDirEntryField.place(x=110, y=170)
J8MotDirEntryField = Entry(tab3,width=5,justify="center")
J8MotDirEntryField.place(x=110, y=195)
J9MotDirEntryField = Entry(tab3,width=5,justify="center")
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

J1CalDirEntryField = Entry(tab3,width=5,justify="center")
J1CalDirEntryField.place(x=110, y=280)
J2CalDirEntryField = Entry(tab3,width=5,justify="center")
J2CalDirEntryField.place(x=110, y=305)
J3CalDirEntryField = Entry(tab3,width=5,justify="center")
J3CalDirEntryField.place(x=110, y=330)
J4CalDirEntryField = Entry(tab3,width=5,justify="center")
J4CalDirEntryField.place(x=110, y=355)
J5CalDirEntryField = Entry(tab3,width=5,justify="center")
J5CalDirEntryField.place(x=110, y=380)
J6CalDirEntryField = Entry(tab3,width=5,justify="center")
J6CalDirEntryField.place(x=110, y=405)
J7CalDirEntryField = Entry(tab3,width=5,justify="center")
J7CalDirEntryField.place(x=110, y=430)
J8CalDirEntryField = Entry(tab3,width=5,justify="center")
J8CalDirEntryField.place(x=110, y=455)
J9CalDirEntryField = Entry(tab3,width=5,justify="center")
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

J1PosLimEntryField = Entry(tab3,width=5,justify="center")
J1PosLimEntryField.place(x=280, y=20)
J1NegLimEntryField = Entry(tab3,width=5,justify="center")
J1NegLimEntryField.place(x=280, y=45)
J2PosLimEntryField = Entry(tab3,width=5,justify="center")
J2PosLimEntryField.place(x=280, y=70)
J2NegLimEntryField = Entry(tab3,width=5,justify="center")
J2NegLimEntryField.place(x=280, y=95)
J3PosLimEntryField = Entry(tab3,width=5,justify="center")
J3PosLimEntryField.place(x=280, y=120)
J3NegLimEntryField = Entry(tab3,width=5,justify="center")
J3NegLimEntryField.place(x=280, y=145)
J4PosLimEntryField = Entry(tab3,width=5,justify="center")
J4PosLimEntryField.place(x=280, y=170)
J4NegLimEntryField = Entry(tab3,width=5,justify="center")
J4NegLimEntryField.place(x=280, y=195)
J5PosLimEntryField = Entry(tab3,width=5,justify="center")
J5PosLimEntryField.place(x=280, y=220)
J5NegLimEntryField = Entry(tab3,width=5,justify="center")
J5NegLimEntryField.place(x=280, y=245)
J6PosLimEntryField = Entry(tab3,width=5,justify="center")
J6PosLimEntryField.place(x=280, y=270)
J6NegLimEntryField = Entry(tab3,width=5,justify="center")
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

J1StepDegEntryField = Entry(tab3,width=5,justify="center")
J1StepDegEntryField.place(x=280, y=345)
J2StepDegEntryField = Entry(tab3,width=5,justify="center")
J2StepDegEntryField.place(x=280, y=370)
J3StepDegEntryField = Entry(tab3,width=5,justify="center")
J3StepDegEntryField.place(x=280, y=395)
J4StepDegEntryField = Entry(tab3,width=5,justify="center")
J4StepDegEntryField.place(x=280, y=420)
J5StepDegEntryField = Entry(tab3,width=5,justify="center")
J5StepDegEntryField.place(x=280, y=445)
J6StepDegEntryField = Entry(tab3,width=5,justify="center")
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

J1DriveMSEntryField = Entry(tab3,width=5,justify="center")
J1DriveMSEntryField.place(x=500, y=20)
J2DriveMSEntryField = Entry(tab3,width=5,justify="center")
J2DriveMSEntryField.place(x=500, y=45)
J3DriveMSEntryField = Entry(tab3,width=5,justify="center")
J3DriveMSEntryField.place(x=500, y=70)
J4DriveMSEntryField = Entry(tab3,width=5,justify="center")
J4DriveMSEntryField.place(x=500, y=95)
J5DriveMSEntryField = Entry(tab3,width=5,justify="center")
J5DriveMSEntryField.place(x=500, y=120)
J6DriveMSEntryField = Entry(tab3,width=5,justify="center")
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

J1EncCPREntryField = Entry(tab3,width=5,justify="center")
J1EncCPREntryField.place(x=500, y=195)
J2EncCPREntryField = Entry(tab3,width=5,justify="center")
J2EncCPREntryField.place(x=500, y=220)
J3EncCPREntryField = Entry(tab3,width=5,justify="center")
J3EncCPREntryField.place(x=500, y=245)
J4EncCPREntryField = Entry(tab3,width=5,justify="center")
J4EncCPREntryField.place(x=500, y=270)
J5EncCPREntryField = Entry(tab3,width=5,justify="center")
J5EncCPREntryField.place(x=500, y=295)
J6EncCPREntryField = Entry(tab3,width=5,justify="center")
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


J1ΘEntryField = Entry(tab3,width=5,justify="center")
J1ΘEntryField.place(x=630, y=45)
J2ΘEntryField = Entry(tab3,width=5,justify="center")
J2ΘEntryField.place(x=630, y=70)
J3ΘEntryField = Entry(tab3,width=5,justify="center")
J3ΘEntryField.place(x=630, y=95)
J4ΘEntryField = Entry(tab3,width=5,justify="center")
J4ΘEntryField.place(x=630, y=120)
J5ΘEntryField = Entry(tab3,width=5,justify="center")
J5ΘEntryField.place(x=630, y=145)
J6ΘEntryField = Entry(tab3,width=5,justify="center")
J6ΘEntryField.place(x=630, y=170)

J1αEntryField = Entry(tab3,width=5,justify="center")
J1αEntryField.place(x=685, y=45)
J2αEntryField = Entry(tab3,width=5,justify="center")
J2αEntryField.place(x=685, y=70)
J3αEntryField = Entry(tab3,width=5,justify="center")
J3αEntryField.place(x=685, y=95)
J4αEntryField = Entry(tab3,width=5,justify="center")
J4αEntryField.place(x=685, y=120)
J5αEntryField = Entry(tab3,width=5,justify="center")
J5αEntryField.place(x=685, y=145)
J6αEntryField = Entry(tab3,width=5,justify="center")
J6αEntryField.place(x=685, y=170)

J1dEntryField = Entry(tab3,width=5,justify="center")
J1dEntryField.place(x=740, y=45)
J2dEntryField = Entry(tab3,width=5,justify="center")
J2dEntryField.place(x=740, y=70)
J3dEntryField = Entry(tab3,width=5,justify="center")
J3dEntryField.place(x=740, y=95)
J4dEntryField = Entry(tab3,width=5,justify="center")
J4dEntryField.place(x=740, y=120)
J5dEntryField = Entry(tab3,width=5,justify="center")
J5dEntryField.place(x=740, y=145)
J6dEntryField = Entry(tab3,width=5,justify="center")
J6dEntryField.place(x=740, y=170)

J1aEntryField = Entry(tab3,width=5,justify="center")
J1aEntryField.place(x=795, y=45)
J2aEntryField = Entry(tab3,width=5,justify="center")
J2aEntryField.place(x=795, y=70)
J3aEntryField = Entry(tab3,width=5,justify="center")
J3aEntryField.place(x=795, y=95)
J4aEntryField = Entry(tab3,width=5,justify="center")
J4aEntryField.place(x=795, y=120)
J5aEntryField = Entry(tab3,width=5,justify="center")
J5aEntryField.place(x=795, y=145)
J6aEntryField = Entry(tab3,width=5,justify="center")
J6aEntryField.place(x=795, y=170)


### LOAD DEFAULT ###

loadAR4Mk2But = Button(tab3,  text="Load AR4-MK3 Defaults",  width=26, command = LoadAR4Mk3default)
loadAR4Mk2But.place(x=1150, y=470)

loadAR4Mk2But = Button(tab3,  text="Load AR4-MK2 Defaults",  width=26, command = LoadAR4Mk2default)
loadAR4Mk2But.place(x=1150, y=510)

loadAR4But = Button(tab3,  text="Load AR4 Defaults",  width=26, command = LoadAR4default)
loadAR4But.place(x=1150, y=550)

loadAR3But = Button(tab3,  text="Load AR3 Defaults",  width=26, command = LoadAR3default)
loadAR3But.place(x=1150, y=590)






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
servo0onequalsLab.place(x=70, y=42)

servo0offequalsLab = Label(tab4, text = "=")
servo0offequalsLab.place(x=70, y=82)

servo1onequalsLab = Label(tab4, text = "=")
servo1onequalsLab.place(x=70, y=122)

servo1offequalsLab = Label(tab4, text = "=")
servo1offequalsLab.place(x=70, y=162)

servo2onequalsLab = Label(tab4, text = "=")
servo2onequalsLab.place(x=70, y=202)

servo2offequalsLab = Label(tab4, text = "=")
servo2offequalsLab.place(x=70, y=242)

servo3onequalsLab = Label(tab4, text = "=")
servo3onequalsLab.place(x=70, y=282)

servo3offequalsLab = Label(tab4, text = "=")
servo3offequalsLab.place(x=70, y=322)



Do1onequalsLab = Label(tab4, text = "=")
Do1onequalsLab.place(x=210, y=42)

Do1offequalsLab = Label(tab4, text = "=")
Do1offequalsLab.place(x=210, y=82)

Do2onequalsLab = Label(tab4, text = "=")
Do2onequalsLab.place(x=210, y=122)

Do2offequalsLab = Label(tab4, text = "=")
Do2offequalsLab.place(x=210, y=162)

Do3onequalsLab = Label(tab4, text = "=")
Do3onequalsLab.place(x=210, y=202)

Do3offequalsLab = Label(tab4, text = "=")
Do3offequalsLab.place(x=210, y=242)

Do4onequalsLab = Label(tab4, text = "=")
Do4onequalsLab.place(x=210, y=282)

Do4offequalsLab = Label(tab4, text = "=")
Do4offequalsLab.place(x=210, y=322)

Do5onequalsLab = Label(tab4, text = "=")
Do5onequalsLab.place(x=210, y=362)

Do5offequalsLab = Label(tab4, text = "=")
Do5offequalsLab.place(x=210, y=402)

Do6onequalsLab = Label(tab4, text = "=")
Do6onequalsLab.place(x=210, y=442)

Do6offequalsLab = Label(tab4, text = "=")
Do6offequalsLab.place(x=210, y=482)

IOboardLab = Label(tab4, font=("Arial 10 bold"), text = "5v IO BOARD")
IOboardLab.place(x=95, y=10)

AuxComLab = Label(tab4, font=("Arial 10 bold"), text = "AUX COM DEVICE")
AuxComLab.place(x=400, y=10)

ModbusLab = Label(tab4, font=("Arial 10 bold"), text = "MODBUS DEVICE")
ModbusLab.place(x=700, y=10)

AuxPortNumLab= Label(tab4, text = "Aux Com Port")
AuxPortNumLab.place(x=440, y=42)

AuxPortCharLab= Label(tab4, text = "Char to Read")
AuxPortCharLab.place(x=440, y=82)

MBslaveLab= Label(tab4, text = "Slave ID")
MBslaveLab.place(x=750, y=42)

MBaddressLab= Label(tab4, text = "Modbus Address")
MBaddressLab.place(x=750, y=82)

MBwriteLab= Label(tab4, text = "Operation Value")
MBwriteLab.place(x=750, y=122)

MBoutputLab= Label(tab4, text = "Output Response:")
MBoutputLab.place(x=662, y=405)



inoutavailLab = Label(tab4, text = "The following IO are available when using the default 5v Nano board for IO:   Inputs = 2-7  /  Outputs = 8-13  /  Servos = A0-A7")
inoutavailLab.place(x=10, y=640)

inoutavailLab = Label(tab4, text = "The following IO are available when using the default 5v Mega board for IO:   Inputs = 0-27  /  Outputs = 28-53  /  Servos = A0-A7")
inoutavailLab.place(x=10, y=655)

inoutavailLab = Label(tab4, text = "Please review this tutorial video on using 5v IO boards:")
inoutavailLab.place(x=10, y=670)

inoutavailLab = Label(tab4, text = "5v board inputs are high impedance and susceptable to floating voltage - inputs use a pullup resistor and will read high when nothing is connected - its best to connect your input signal to GND and if/wait for the input signal to = 0")
inoutavailLab.place(x=10, y=685)



link2 = Label(tab4, font=("Arial", 8), text="https://youtu.be/76F6dS4ar8Y?si=Z6NstZy1zNeHgtCF", foreground="blue", cursor="hand2")
link2.bind("<Button-1>", lambda event: webbrowser.open(link2.cget("text")))
link2.place(x=300, y=671)



### 4 BUTTONS################################################################
#############################################################################

servo0onBut = Button(tab4,  text="Servo 0",  command = Servo0on)
servo0onBut.place(x=10, y=40)

servo0offBut = Button(tab4,  text="Servo 0",  command = Servo0off)
servo0offBut.place(x=10, y=80)

servo1onBut = Button(tab4,  text="Servo 1",  command = Servo1on)
servo1onBut.place(x=10, y=120)

servo1offBut = Button(tab4,  text="Servo 1",  command = Servo1off)
servo1offBut.place(x=10, y=160)

servo2onBut = Button(tab4,  text="Servo 2",  command = Servo2on)
servo2onBut.place(x=10, y=200)

servo2offBut = Button(tab4,  text="Servo 2",  command = Servo2off)
servo2offBut.place(x=10, y=240)

servo3onBut = Button(tab4,  text="Servo 3",  command = Servo3on)
servo3onBut.place(x=10, y=280)

servo3offBut = Button(tab4,  text="Servo 3",  command = Servo3off)
servo3offBut.place(x=10, y=320)





DO1onBut = Button(tab4,  text="DO on",  command = DO1on)
DO1onBut.place(x=150, y=40)

DO1offBut = Button(tab4,  text="DO off",  command = DO1off)
DO1offBut.place(x=150, y=80)

DO2onBut = Button(tab4,  text="DO on",  command = DO2on)
DO2onBut.place(x=150, y=120)

DO2offBut = Button(tab4,  text="DO off",  command = DO2off)
DO2offBut.place(x=150, y=160)

DO3onBut = Button(tab4,  text="DO on",  command = DO3on)
DO3onBut.place(x=150, y=200)

DO3offBut = Button(tab4,  text="DO off",  command = DO3off)
DO3offBut.place(x=150, y=240)

DO4onBut = Button(tab4,  text="DO on",  command = DO4on)
DO4onBut.place(x=150, y=280)

DO4offBut = Button(tab4,  text="DO off",  command = DO4off)
DO4offBut.place(x=150, y=320)

DO5onBut = Button(tab4,  text="DO on",  command = DO5on)
DO5onBut.place(x=150, y=360)

DO5offBut = Button(tab4,  text="DO off",  command = DO5off)
DO5offBut.place(x=150, y=400)

DO6onBut = Button(tab4,  text="DO on",  command = DO6on)
DO6onBut.place(x=150, y=440)

DO6offBut = Button(tab4,  text="DO off",  command = DO6off)
DO6offBut.place(x=150, y=480)


comPortBut3 = Button(tab4,  text="Test Aux COM Device",   command = TestAuxCom)
comPortBut3.place(x=395, y=120)

MBreadCoilBut = Button(tab4,  text="Read Coil", width=30, command = MBreadCoil)
MBreadCoilBut.place(x=665, y=160)

MBreadDinputBut = Button(tab4,  text="Read Discrete Input", width=30, command = MBreadInput)
MBreadDinputBut.place(x=665, y=200)

MBreadHoldRegBut = Button(tab4,  text="Read Holding Register", width=30, command = MBreadHoldReg)
MBreadHoldRegBut.place(x=665, y=240)

MBreadInputRegBut = Button(tab4,  text="Read Input Register", width=30, command = MBreadInputReg)
MBreadInputRegBut.place(x=665, y=280)

MBwriteCoilBut = Button(tab4,  text="Write Coil", width=30, command = MBwriteCoil)
MBwriteCoilBut.place(x=665, y=320)

MBwriteRegBut = Button(tab4,  text="Write Register", width=30, command = MBwriteReg)
MBwriteRegBut.place(x=665, y=360)





#### 4 ENTRY FIELDS##########################################################
#############################################################################


servo0onEntryField = Entry(tab4,width=4,justify="center")
servo0onEntryField.place(x=90, y=45)

servo0offEntryField = Entry(tab4,width=4,justify="center")
servo0offEntryField.place(x=90, y=85)

servo1onEntryField = Entry(tab4,width=4,justify="center")
servo1onEntryField.place(x=90, y=125)

servo1offEntryField = Entry(tab4,width=4,justify="center")
servo1offEntryField.place(x=90, y=165)

servo2onEntryField = Entry(tab4,width=4,justify="center")
servo2onEntryField.place(x=90, y=205)

servo2offEntryField = Entry(tab4,width=4,justify="center")
servo2offEntryField.place(x=90, y=245)


servo3onEntryField = Entry(tab4,width=4,justify="center")
servo3onEntryField.place(x=90, y=285)

servo3offEntryField = Entry(tab4,width=4,justify="center")
servo3offEntryField.place(x=90, y=325)





DO1onEntryField = Entry(tab4,width=4,justify="center")
DO1onEntryField.place(x=230, y=45)

DO1offEntryField = Entry(tab4,width=4,justify="center")
DO1offEntryField.place(x=230, y=85)

DO2onEntryField = Entry(tab4,width=4,justify="center")
DO2onEntryField.place(x=230, y=125)

DO2offEntryField = Entry(tab4,width=4,justify="center")
DO2offEntryField.place(x=230, y=165)

DO3onEntryField = Entry(tab4,width=4,justify="center")
DO3onEntryField.place(x=230, y=205)

DO3offEntryField = Entry(tab4,width=4,justify="center")
DO3offEntryField.place(x=230, y=245)

DO4onEntryField = Entry(tab4,width=4,justify="center")
DO4onEntryField.place(x=230, y=285)

DO4offEntryField = Entry(tab4,width=4,justify="center")
DO4offEntryField.place(x=230, y=325)

DO5onEntryField = Entry(tab4,width=4,justify="center")
DO5onEntryField.place(x=230, y=365)

DO5offEntryField = Entry(tab4,width=4,justify="center")
DO5offEntryField.place(x=230, y=405)

DO6onEntryField = Entry(tab4,width=4,justify="center")
DO6onEntryField.place(x=230, y=445)

DO6offEntryField = Entry(tab4,width=4,justify="center")
DO6offEntryField.place(x=230, y=485)



com3PortEntryField = Entry(tab4,width=4,justify="center")
com3PortEntryField.place(x=400, y=40)

com3charPortEntryField = Entry(tab4,width=4,justify="center")
com3charPortEntryField.place(x=400, y=80)

com3outPortEntryField = Entry(tab4,width=25,justify="center")
com3outPortEntryField.place(x=385, y=160)


MBslaveEntryField = Entry(tab4,width=4,justify="center")
MBslaveEntryField.place(x=710, y=40)

MBaddressEntryField = Entry(tab4,width=5,justify="center")
MBaddressEntryField.place(x=690, y=80)

MBoperValEntryField = Entry(tab4,width=5,justify="center")
MBoperValEntryField.place(x=690, y=120)

MBoutputEntryField = Entry(tab4,width=33,justify="center")
MBoutputEntryField.place(x=662, y=425)



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

R1EntryField = Entry(tab5,width=4,justify="center")
R1EntryField.place(x=30, y=30)

R2EntryField = Entry(tab5,width=4,justify="center")
R2EntryField.place(x=30, y=60)

R3EntryField = Entry(tab5,width=4,justify="center")
R3EntryField.place(x=30, y=90)

R4EntryField = Entry(tab5,width=4,justify="center")
R4EntryField.place(x=30, y=120)

R5EntryField = Entry(tab5,width=4,justify="center")
R5EntryField.place(x=30, y=150)

R6EntryField = Entry(tab5,width=4,justify="center")
R6EntryField.place(x=30, y=180)

R7EntryField = Entry(tab5,width=4,justify="center")
R7EntryField.place(x=30, y=210)

R8EntryField = Entry(tab5,width=4,justify="center")
R8EntryField.place(x=30, y=240)

R9EntryField = Entry(tab5,width=4,justify="center")
R9EntryField.place(x=30, y=270)

R10EntryField = Entry(tab5,width=4,justify="center")
R10EntryField.place(x=30, y=300)

R11EntryField = Entry(tab5,width=4,justify="center")
R11EntryField.place(x=30, y=330)

R12EntryField = Entry(tab5,width=4,justify="center")
R12EntryField.place(x=30, y=360)

R13EntryField = Entry(tab5,width=4,justify="center")
R13EntryField.place(x=30, y=390)

R14EntryField = Entry(tab5,width=4,justify="center")
R14EntryField.place(x=30, y=420)

R15EntryField = Entry(tab5,width=4,justify="center")
R15EntryField.place(x=30, y=450)

R16EntryField = Entry(tab5,width=4,justify="center")
R16EntryField.place(x=30, y=480)




SP_1_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_1_E1_EntryField.place(x=400, y=30)

SP_2_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_2_E1_EntryField.place(x=400, y=60)

SP_3_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_3_E1_EntryField.place(x=400, y=90)

SP_4_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_4_E1_EntryField.place(x=400, y=120)

SP_5_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_5_E1_EntryField.place(x=400, y=150)

SP_6_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_6_E1_EntryField.place(x=400, y=180)

SP_7_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_7_E1_EntryField.place(x=400, y=210)

SP_8_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_8_E1_EntryField.place(x=400, y=240)

SP_9_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_9_E1_EntryField.place(x=400, y=270)

SP_10_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_10_E1_EntryField.place(x=400, y=300)

SP_11_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_11_E1_EntryField.place(x=400, y=330)

SP_12_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_12_E1_EntryField.place(x=400, y=360)

SP_13_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_13_E1_EntryField.place(x=400, y=390)

SP_14_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_14_E1_EntryField.place(x=400, y=420)

SP_15_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_15_E1_EntryField.place(x=400, y=450)

SP_16_E1_EntryField = Entry(tab5,width=4,justify="center")
SP_16_E1_EntryField.place(x=400, y=480)





SP_1_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_1_E2_EntryField.place(x=440, y=30)

SP_2_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_2_E2_EntryField.place(x=440, y=60)

SP_3_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_3_E2_EntryField.place(x=440, y=90)

SP_4_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_4_E2_EntryField.place(x=440, y=120)

SP_5_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_5_E2_EntryField.place(x=440, y=150)

SP_6_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_6_E2_EntryField.place(x=440, y=180)

SP_7_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_7_E2_EntryField.place(x=440, y=210)

SP_8_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_8_E2_EntryField.place(x=440, y=240)

SP_9_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_9_E2_EntryField.place(x=440, y=270)

SP_10_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_10_E2_EntryField.place(x=440, y=300)

SP_11_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_11_E2_EntryField.place(x=440, y=330)

SP_12_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_12_E2_EntryField.place(x=440, y=360)

SP_13_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_13_E2_EntryField.place(x=440, y=390)

SP_14_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_14_E2_EntryField.place(x=440, y=420)

SP_15_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_15_E2_EntryField.place(x=440, y=450)

SP_16_E2_EntryField = Entry(tab5,width=4,justify="center")
SP_16_E2_EntryField.place(x=440, y=480)




SP_1_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_1_E3_EntryField.place(x=480, y=30)

SP_2_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_2_E3_EntryField.place(x=480, y=60)

SP_3_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_3_E3_EntryField.place(x=480, y=90)

SP_4_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_4_E3_EntryField.place(x=480, y=120)

SP_5_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_5_E3_EntryField.place(x=480, y=150)

SP_6_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_6_E3_EntryField.place(x=480, y=180)

SP_7_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_7_E3_EntryField.place(x=480, y=210)

SP_8_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_8_E3_EntryField.place(x=480, y=240)

SP_9_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_9_E3_EntryField.place(x=480, y=270)

SP_10_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_10_E3_EntryField.place(x=480, y=300)

SP_11_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_11_E3_EntryField.place(x=480, y=330)

SP_12_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_12_E3_EntryField.place(x=480, y=360)

SP_13_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_13_E3_EntryField.place(x=480, y=390)

SP_14_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_14_E3_EntryField.place(x=480, y=420)

SP_15_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_15_E3_EntryField.place(x=480, y=450)

SP_16_E3_EntryField = Entry(tab5,width=4,justify="center")
SP_16_E3_EntryField.place(x=480, y=480)




SP_1_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_1_E4_EntryField.place(x=520, y=30)

SP_2_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_2_E4_EntryField.place(x=520, y=60)

SP_3_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_3_E4_EntryField.place(x=520, y=90)

SP_4_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_4_E4_EntryField.place(x=520, y=120)

SP_5_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_5_E4_EntryField.place(x=520, y=150)

SP_6_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_6_E4_EntryField.place(x=520, y=180)

SP_7_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_7_E4_EntryField.place(x=520, y=210)

SP_8_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_8_E4_EntryField.place(x=520, y=240)

SP_9_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_9_E4_EntryField.place(x=520, y=270)

SP_10_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_10_E4_EntryField.place(x=520, y=300)

SP_11_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_11_E4_EntryField.place(x=520, y=330)

SP_12_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_12_E4_EntryField.place(x=520, y=360)

SP_13_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_13_E4_EntryField.place(x=520, y=390)

SP_14_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_14_E4_EntryField.place(x=520, y=420)

SP_15_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_15_E4_EntryField.place(x=520, y=450)

SP_16_E4_EntryField = Entry(tab5,width=4,justify="center")
SP_16_E4_EntryField.place(x=520, y=480)

SP_1_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_1_E5_EntryField.place(x=560, y=30)

SP_2_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_2_E5_EntryField.place(x=560, y=60)

SP_3_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_3_E5_EntryField.place(x=560, y=90)

SP_4_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_4_E5_EntryField.place(x=560, y=120)

SP_5_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_5_E5_EntryField.place(x=560, y=150)

SP_6_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_6_E5_EntryField.place(x=560, y=180)

SP_7_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_7_E5_EntryField.place(x=560, y=210)

SP_8_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_8_E5_EntryField.place(x=560, y=240)

SP_9_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_9_E5_EntryField.place(x=560, y=270)

SP_10_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_10_E5_EntryField.place(x=560, y=300)

SP_11_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_11_E5_EntryField.place(x=560, y=330)

SP_12_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_12_E5_EntryField.place(x=560, y=360)

SP_13_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_13_E5_EntryField.place(x=560, y=390)

SP_14_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_14_E5_EntryField.place(x=560, y=420)

SP_15_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_15_E5_EntryField.place(x=560, y=450)

SP_16_E5_EntryField = Entry(tab5,width=4,justify="center")
SP_16_E5_EntryField.place(x=560, y=480)




SP_1_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_1_E6_EntryField.place(x=600, y=30)

SP_2_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_2_E6_EntryField.place(x=600, y=60)

SP_3_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_3_E6_EntryField.place(x=600, y=90)

SP_4_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_4_E6_EntryField.place(x=600, y=120)

SP_5_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_5_E6_EntryField.place(x=600, y=150)

SP_6_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_6_E6_EntryField.place(x=600, y=180)

SP_7_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_7_E6_EntryField.place(x=600, y=210)

SP_8_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_8_E6_EntryField.place(x=600, y=240)

SP_9_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_9_E6_EntryField.place(x=600, y=270)

SP_10_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_10_E6_EntryField.place(x=600, y=300)

SP_11_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_11_E6_EntryField.place(x=600, y=330)

SP_12_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_12_E6_EntryField.place(x=600, y=360)

SP_13_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_13_E6_EntryField.place(x=600, y=390)

SP_14_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_14_E6_EntryField.place(x=600, y=420)

SP_15_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_15_E6_EntryField.place(x=600, y=450)

SP_16_E6_EntryField = Entry(tab5,width=4,justify="center")
SP_16_E6_EntryField.place(x=600, y=480)




####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 6




### 6 LABELS#################################################################
#############################################################################


#VisBackdropImg = ImageTk.PhotoImage(Image.open('VisBackdrop.png'))

VisBackdropImg = Image.open("VisBackdrop.png")
VisBackdropTk = ImageTk.PhotoImage(VisBackdropImg)
VisBackdromLbl = Label(tab6, image = VisBackdropTk)
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


template_frame = Frame(tab6,width=120,height=150)
template_frame.place(x=575, y=50)

template_lbl = Label(template_frame)
template_lbl.place(x=0, y=0)

FoundValuesLab = Label(tab6, text = "FOUND VALUES")
FoundValuesLab.place(x=750, y=30)

CalValuesLab = Label(tab6, text = "CALIBRATION VALUES")
CalValuesLab.place(x=900, y=30)





### 6 BUTTONS################################################################
#############################################################################

match CE['Platform']['OS']:
  case "Windows":
    graph = FilterGraph()
    try:
      camList = graph.get_input_devices()
    except:
      camList = ["Select a Camera"]
    visoptions=StringVar(tab6)
    visoptions.set("Select a Camera")
    try:
      vismenu=OptionMenu(tab6, visoptions, camList[0], *camList)
      vismenu.config(width=20)
      vismenu.place(x=10, y=10)
    except: 
      logger.error ("no camera")
  case "Linux":
    # Build label->id map
    label_to_id = {c.label: c.id for c in CE['Cameras']['Enum']}
    camList = list(label_to_id.keys())

    selected_label = tk.StringVar(value=(camList[0] if camList else "Select a Camera"))

    visoptions=StringVar(tab6)
    visoptions.set("Select a Camera")

    def on_camera_select(chosen_label):
      cam_id = label_to_id.get(chosen_label, "None")
      logger.debug("Debug - User picked:", chosen_label, " -> using id:", cam_id)
 
    try:
      vismenu=OptionMenu(tab6, visoptions, selected_label.get(), *camList, command=on_camera_select)
      vismenu.config(width=20)
      vismenu.place(x=10, y=10)
    except: 
      logger.error ("no camera")


StartCamBut = Button(tab6,  text="Start Camera",  width=12, command = start_vid)
StartCamBut.place(x=200, y=10)

StopCamBut = Button(tab6,  text="Stop Camera",  width=12, command = stop_vid)
StopCamBut.place(x=315, y=10)

CapImgBut = Button(tab6,  text="Snap Image",  width=12, command = take_pic)
CapImgBut.place(x=10, y=50)

TeachImgBut = Button(tab6,  text="Teach Object",  width=12, command = selectTemplate)
TeachImgBut.place(x=140, y=50)

FindVisBut = Button(tab6,  text="Snap & Find",  width=12, command = snapFind)
FindVisBut.place(x=270, y=50)


ZeroBrCnBut = Button(tab6, text="Zero",  width=4, command = zeroBrCn)
ZeroBrCnBut.place(x=10, y=110)

maskBut = Button(tab6, text="Mask",  width=4, command = selectMask)
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



VisBacColorEntryField = Entry(tab6,width=12,justify="center")
VisBacColorEntryField.place(x=390, y=100)
VisBacColorLab = Label(tab6, text = "Background Color")
VisBacColorLab.place(x=390, y=120)

bgAutoCbut = Checkbutton(tab6, command=checkAutoBG, text="Auto",variable = autoBG)
bgAutoCbut.place(x=490, y=101)

VisScoreEntryField = Entry(tab6,width=12,justify="center")
VisScoreEntryField.place(x=390, y=150)
VisScoreLab = Label(tab6, text = "Score Threshold")
VisScoreLab.place(x=390, y=170)




VisRetScoreEntryField = Entry(tab6,width=12,justify="center")
VisRetScoreEntryField.place(x=750, y=55)
VisRetScoreLab = Label(tab6, text = "Scored Value")
VisRetScoreLab.place(x=750, y=75)

VisRetAngleEntryField = Entry(tab6,width=12,justify="center")
VisRetAngleEntryField.place(x=750, y=105)
VisRetAngleLab = Label(tab6, text = "Found Angle")
VisRetAngleLab.place(x=750, y=125)

VisRetXpixEntryField = Entry(tab6,width=12,justify="center")
VisRetXpixEntryField.place(x=750, y=155)
VisRetXpixLab = Label(tab6, text = "Pixel X Position")
VisRetXpixLab.place(x=750, y=175)

VisRetYpixEntryField = Entry(tab6,width=12,justify="center")
VisRetYpixEntryField.place(x=750, y=205)
VisRetYpixLab = Label(tab6, text = "Pixel Y Position")
VisRetYpixLab.place(x=750, y=225)

VisRetXrobEntryField = Entry(tab6,width=12,justify="center")
VisRetXrobEntryField.place(x=750, y=255)
VisRetXrobLab = Label(tab6, text = "Robot X Position")
VisRetXrobLab.place(x=750, y=275)

VisRetYrobEntryField = Entry(tab6,width=12,justify="center")
VisRetYrobEntryField.place(x=750, y=305)
VisRetYrobLab = Label(tab6, text = "Robot Y Position")
VisRetYrobLab.place(x=750, y=325)







VisX1PixEntryField = Entry(tab6,width=12,justify="center")
VisX1PixEntryField.place(x=900, y=55)
VisX1PixLab = Label(tab6, text = "X1 Pixel Pos")
VisX1PixLab.place(x=900, y=75)

VisY1PixEntryField = Entry(tab6,width=12,justify="center")
VisY1PixEntryField.place(x=900, y=105)
VisY1PixLab = Label(tab6, text = "Y1 Pixel Pos")
VisY1PixLab.place(x=900, y=125)

VisX2PixEntryField = Entry(tab6,width=12,justify="center")
VisX2PixEntryField.place(x=900, y=155)
VisX2PixLab = Label(tab6, text = "X2 Pixel Pos")
VisX2PixLab.place(x=900, y=175)

VisY2PixEntryField = Entry(tab6,width=12,justify="center")
VisY2PixEntryField.place(x=900, y=205)
VisY2PixLab = Label(tab6, text = "Y2 Pixel Pos")
VisY2PixLab.place(x=900, y=225)


VisX1RobEntryField = Entry(tab6,width=12,justify="center")
VisX1RobEntryField.place(x=1010, y=55)
VisX1RobLab = Label(tab6, text = "X1 Robot Pos")
VisX1RobLab.place(x=1010, y=75)

VisY1RobEntryField = Entry(tab6,width=12,justify="center")
VisY1RobEntryField.place(x=1010, y=105)
VisY1RobLab = Label(tab6, text = "Y1 Robot Pos")
VisY1RobLab.place(x=1010, y=125)

VisX2RobEntryField = Entry(tab6,width=12,justify="center")
VisX2RobEntryField.place(x=1010, y=155)
VisX2RobLab = Label(tab6, text = "X2 Robot Pos")
VisX2RobLab.place(x=1010, y=175)

VisY2RobEntryField = Entry(tab6,width=12,justify="center")
VisY2RobEntryField.place(x=1010, y=205)
VisY2RobLab = Label(tab6, text = "Y2 Robot Pos")
VisY2RobLab.place(x=1010, y=225)





####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 7

GcodeProgEntryField = Entry(tab7,width=60,justify="center")
GcodeProgEntryField.place(x=20, y=55)

GcodCurRowEntryField = Entry(tab7,width=10,justify="center")
GcodCurRowEntryField.place(x=1175, y=20)

GC_ST_E1_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E1_EntryField.place(x=20, y=140)

GC_ST_E2_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E2_EntryField.place(x=75, y=140)

GC_ST_E3_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E3_EntryField.place(x=130, y=140)

GC_ST_E4_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E4_EntryField.place(x=185, y=140)

GC_ST_E5_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E5_EntryField.place(x=240, y=140)

GC_ST_E6_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E6_EntryField.place(x=295, y=140)

GC_ST_WC_EntryField = Entry(tab7,width=3,justify="center")
GC_ST_WC_EntryField.place(x=350, y=140)


GC_SToff_E1_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E1_EntryField.place(x=20, y=205)

GC_SToff_E2_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E2_EntryField.place(x=75, y=205)

GC_SToff_E3_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E3_EntryField.place(x=130, y=205)

GC_SToff_E4_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E4_EntryField.place(x=185, y=205)

GC_SToff_E5_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E5_EntryField.place(x=240, y=205)

GC_SToff_E6_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E6_EntryField.place(x=295, y=205)

GcodeFilenameField = Entry(tab7,width=40,justify="center")
GcodeFilenameField.place(x=20, y=340)


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

def GCcallback(event):
    selection = event.widget.curselection()
    try:
      if selection:
          index = selection[0]
          data = event.widget.get(index)
          data = data.replace('.txt','')
          GcodeFilenameField.delete(0, 'end')
          GcodeFilenameField.insert(0,data)
          PlayGCEntryField.delete(0, 'end')
          PlayGCEntryField.insert(0,data)    
      else:
          GcodeFilenameField.insert(0,"")  
    except:
      logger.error("not an SD file")
      
tab7.gcodeView.bind("<<ListboxSelect>>", GCcallback)


LoadGcodeBut = Button(tab7,  text="Load Program", width=25, command = loadGcodeProg)
LoadGcodeBut.place(x=20, y=20)

GcodeStartPosBut = Button(tab7,  text="Set Start Position", width=25, command = SetGcodeStartPos)
GcodeStartPosBut.place(x=20, y=100)

GcodeMoveStartPosBut = Button(tab7,  text="Move to Start Offset", width=25, command = MoveGcodeStartPos)
GcodeMoveStartPosBut.place(x=20, y=240)

runGcodeBut = Button(tab7, text="Convert & Upload to SD", width=25,   command = GCconvertProg)
#playGPhoto=PhotoImage(file="play-icon.gif")
#runGcodeBut.config(image=playGPhoto)
runGcodeBut.place(x=20, y=375)

stopGcodeBut = Button(tab7, text="Stop Conversion & Upload", width=25,  command = GCstopProg)
#stopGPhoto=PhotoImage(file="stop-icon.gif")
#stopGcodeBut.config(image=stopGPhoto)
stopGcodeBut.place(x=190, y=375)

delGcodeBut = Button(tab7, text="Delete File from SD", width=25,   command = GCdelete)
delGcodeBut.place(x=20, y=415)

readGcodeBut = Button(tab7, text="Read Files from SD", width=25,   command = partial(GCread, "yes"))
readGcodeBut.place(x=20, y=455)

playGPhoto=PhotoImage(file="play-icon.png")
readGcodeBut = Button(tab7, text="Play Gcode File", width=20,   command = GCplay, image = playGPhoto, compound=LEFT)
readGcodeBut.place(x=20, y=495)

#revGcodeBut = Button(tab7,  text="REV ",  command = stepRev)
#revGcodeBut.place(x=180, y=290)

#fwdGcodeBut = Button(tab7,  text="FWD", command = GCstepFwd)
#fwdGcodeBut.place(x=230, y=290)

saveGCBut = Button(tab7,  text="SAVE DATA",  width=26, command = SaveAndApplyCalibration)
saveGCBut.place(x=20, y=600)











gcodeCurRowLab = Label(tab7, text = "Current Row: ")
gcodeCurRowLab.place(x=1100, y=21)

gcodeStartPosOFfLab = Label(tab7, text = "Start Position Offset")
gcodeStartPosOFfLab.place(x=20, y=180)

gcodeFilenameLab = Label(tab7, text = "Filename:")
gcodeFilenameLab.place(x=20, y=320)







####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 8

Elogframe=Frame(tab8)
Elogframe.place(x=40,y=15)
scrollbar = Scrollbar(Elogframe) 
scrollbar.pack(side=RIGHT, fill=Y)
tab8.ElogView = Listbox(Elogframe,width=230,height=40, yscrollcommand=scrollbar.set)
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
clearLogBut.place(x=40, y=690)




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
global J1axisLimNeg, J2axisLimNeg, J3axisLimNeg, J4axisLimNeg, J5axisLimNeg, J6axisLimNeg 
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
#comPort     =calibration.get("12")
if calibration.get("12") in port_choices:
  com1SelectedValue.set(calibration.get("12"))
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
#com2Port     =calibration.get("53")
if calibration.get("53") in port_choices:
  com2SelectedValue.set(calibration.get("53"))
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
GC_ST_WC    = calibration.get("187")
J7CalStatVal= calibration.get("188")
J8CalStatVal= calibration.get("189")
J9CalStatVal= calibration.get("190")
J7CalStatVal2= calibration.get("191")
J8CalStatVal2= calibration.get("192")
J9CalStatVal2= calibration.get("193")
setColor      =calibration.get("194")


#print(f"DEBUG: setting com1Selected Value to: {str(comPort)} comPort is of type {str(type(comPort))}")
#print(f"DEBUG: setting com2Selected Value to: {str(com2Port)} com2Port is of type {str(type(com2Port))}")
#com1SelectedValue.set(value=str(comPort))
#com2SelectedValue.set(value=str(com2Port))
#comPortEntryField.insert(0,str(comPort))
#com2PortEntryField.insert(0,str(com2Port))
incrementEntryField.insert(0,"10")
speedEntryField.insert(0,"25")
ACCspeedField.insert(0,"15")
DECspeedField.insert(0,"15")
ACCrampField.insert(0,"80")
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
if (J7CalStatVal == 1):
  J7CalStat.set(True) 
if (J8CalStatVal == 1):
  J8CalStat.set(True) 
if (J9CalStatVal == 1):
  J9CalStat.set(True)         
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
if (J7CalStatVal2 == 1):
  J7CalStat2.set(True) 
if (J8CalStatVal2 == 1):
  J8CalStat2.set(True) 
if (J9CalStatVal2 == 1):
  J9CalStat2.set(True)           
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
GC_ST_WC_EntryField.insert(0,str(GC_ST_WC))
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

update_CPP_kin_from_entries()
VR_angles = [float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)]
JangleOut = np.array([float(J1AngCur), float(J2AngCur), float(J3AngCur), float(J4AngCur), float(J5AngCur), float(J6AngCur)])
negLim = [float(J1NegLim), float(J2NegLim), float(J3NegLim), float(J4NegLim), float(J5NegLim), float(J6NegLim)]

#axis limits in each direction
J1axisLimNeg = float(J1NegLim)
J2axisLimNeg = float(J2NegLim)
J3axisLimNeg = float(J3NegLim)
J4axisLimNeg = float(J4NegLim)
J5axisLimNeg = float(J5NegLim)
J6axisLimNeg = float(J6NegLim)
J1axisLimPos = float(J1PosLim)
J2axisLimPos = float(J2PosLim)
J3axisLimPos = float(J3PosLim)
J4axisLimPos = float(J4PosLim)
J5axisLimPos = float(J5PosLim)
J6axisLimPos = float(J6PosLim)


#degrees full movement of each axis
J1axisLim = J1axisLimPos + J1axisLimNeg;
J2axisLim = J2axisLimPos + J2axisLimNeg;
J3axisLim = J3axisLimPos + J3axisLimNeg;
J4axisLim = J4axisLimPos + J4axisLimNeg;
J5axisLim = J5axisLimPos + J5axisLimNeg;
J6axisLim = J6axisLimPos + J6axisLimNeg;
#steps full movement of each axis
J1StepLim = J1axisLim * float(J1StepDeg)
J2StepLim = J2axisLim * float(J2StepDeg)
J3StepLim = J3axisLim * float(J3StepDeg)
J4StepLim = J4axisLim * float(J4StepDeg)
J5StepLim = J5axisLim * float(J5StepDeg)
J6StepLim = J6axisLim * float(J6StepDeg)
stepDeg = [float(J1StepDeg), float(J2StepDeg), float(J3StepDeg), float(J4StepDeg), float(J5StepDeg), float(J6StepDeg)]
setStepMonitorsVR()
main_color_var.set(setColor)




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


#tkinter.messagebox.showwarning("AR4 License / Copyright notice", msg)
xboxUse = 0


tab1.after(100, setCom)

tab1.mainloop()




#manEntryField.delete(0, 'end')
#manEntryField.insert(0,value)
