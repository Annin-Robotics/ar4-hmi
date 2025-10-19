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
  VERSION 6.3 10/8/25 changed COM entry to dropdown, added beta Linux support, added basic config module
'''
##########################################################################

import sys
import os
from datetime import datetime

################################################################################################
## Logging Configuration
import logging
from ARrobots.Logging import CustomOutputHandler, ModuleFilter, dump_logger_info
'''
Check for debug mode from environment variable
Linux - 'export DEBUG=true'
Windows - 'set DEBUG=true'
'''
DEBUG = os.getenv("DEBUG", "").lower() in ("1", "true", "yes", "on") #Check if DEBUG env var is set

logger = logging.getLogger("ARrobots")
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

# Add console handler
console = logging.StreamHandler(sys.stdout)
console.setFormatter(logging.Formatter("%(name)s: %(asctime)s [%(levelname)s] %(message)s"))
logger.addHandler(console)
logger.propagate = False

# Function to log to Pane 8
def pane8_log(message):
    if tab8 and hasattr(tab8, "ElogView"):
      Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
      try:
          # Schedule insertion on Tkinter main thread
          tab8.ElogView.after(0, lambda: tab8.ElogView.insert(END, f"{Curtime} - {message}"))
      except tk.TclError:
          pass  # widget likely gone

# Setup Pane8 as a logging handler and log there
pane8_handler = CustomOutputHandler(pane8_log)
pane8_handler.setFormatter(logging.Formatter("%(levelname)s - %(message)s"))
# Pan8 should not try to log modules that init before pane8 is available.
pane8_handler.addFilter(ModuleFilter("ARrobots.AR4config"))
pane8_handler.addFilter(ModuleFilter("ARrobots.Logging"))
logger.addHandler(pane8_handler)

if DEBUG:
  dump_logger_info("ARrobots")
  dump_logger_info("ARrobots.AR4config")
  dump_logger_info("ARrobots.Calibration")
  dump_logger_info("ARrobots.Logging")

## End Logging Configuration #######################################################################################

from os import path, execv
import pathlib
import subprocess
from multiprocessing.resource_sharer import stop
import threading
from threading import Lock, Thread
from queue import Queue

import time

from functools import partial
import ctypes

import math
import numpy as np
from numpy import mean

import pickle
import serial

from matplotlib import pyplot as plt

from tkinter import *
# Import ttkbootstrap widgets to replace ttk widgets
import ttkbootstrap as ttk_bootstrap
from ttkbootstrap import Style as BootstrapStyle
from ttkbootstrap import *  # This makes ttkbootstrap widgets available globally
from tkinter import simpledialog, messagebox
import tkinter as tk
from tkinter import ttk, Misc
from tkinter import filedialog as fd
import tkinter.messagebox
from PIL import Image, ImageTk

import vtk
from vtkmodules.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor
import vtkmodules.vtkInteractionStyle as vtkIS

import webbrowser
import cv2

import re

import ARrobots.robot_kinematics as robot
from ARrobots.Calibration import load_calibration, save_calibration
from ARrobots.HMI.Calibration import apply_calibration

#####################################################################################
# Cross-Compat Patch
# We need platform awareness, port enumeration, and some typing imports

from pathlib import Path
import platform
from serial.tools import list_ports
from typing import List

from ARrobots.AR4config import AR4_Configuration

global Config, CE, CAL, RUN
Config = AR4_Configuration()
CE = Config.Environment
CAL = Config.Calibration
RUN = Config.RuntimeState # Not implemented yet

if CE['Platform']['IS_WINDOWS']:
  from pygrabber.dshow_graph import FilterGraph


robot.robot_set()

DIR = pathlib.Path(__file__).parent.resolve()
os.chdir(DIR)

RUN['cropping'] = False

root = Tk()
root.wm_title("AR4 Software Ver 6.3")
root.iconphoto(True, tk.PhotoImage(file="AR.png"))

# Make headless RPi fit app on screen better
if CE['Platform']['IS_RPI'] and CE['Platform']['IS_HEADLESS']:
  rpi_scale = 0.75
  rpi_x_size = 1590
  rpi_y_size = 800
  logger.debug(f"Running on headless Raspberry Pi - Adjusting scale to {rpi_scale} and window size to {rpi_x_size}x{rpi_y_size}")
  root.tk.call('tk', 'scaling', rpi_scale)
  root.geometry(f'{rpi_x_size}x{rpi_y_size}+0+0')
else:
  root.geometry('1600x900+0+0')  # Adjusted for RPI compatibility (1600x900 minimum)
  #root.geometry("1850x980+0+0")  # Original size

root.resizable(width=True, height=True)

#UI_SCALE = 1.25  

#_orig_place = tk.Widget.place
#def _place_scaled(self, *args, **kw):
#    # scale only absolute pixel arguments
#    for k in ("x", "y", "width", "height"):
#        if k in kw and kw[k] is not None:
#            try:
#                kw[k] = int(float(kw[k]) * UI_SCALE)
#            except Exception:
#                pass
#    return _orig_place(self, *args, **kw)

#tk.Widget.place = _place_scaled




nb = ttk_bootstrap.Notebook(root)

# Configure root window for resizing
root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=1)
nb.grid(row=0, column=0, sticky='nsew')

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

def on_closing():
  cv2.destroyAllWindows()
  root.quit()
  root.update()
  root.destroy()

root.wm_protocol("WM_DELETE_WINDOW", on_closing)

root.runTrue = 0
root.GCrunTrue = 0

#global JogStepsStat
#JogStepsStat = IntVar()
#RUN['JogStepsStat'] = IntVar()
#global J1OpenLoopStat
#J1OpenLoopStat = IntVar()
#global J2OpenLoopStat
#J2OpenLoopStat = IntVar()
#global J3OpenLoopStat
#J3OpenLoopStat = IntVar()
#global J4OpenLoopStat
#J4OpenLoopStat = IntVar()
#global J5OpenLoopStat
#J5OpenLoopStat = IntVar()
#global J6OpenLoopStat
#J6OpenLoopStat = IntVar()
#global DisableWristRot
#DisableWristRot = IntVar()
#global J1CalStat
#J1CalStat = IntVar()
#global J2CalStat
#J2CalStat = IntVar()
#global J3CalStat
#J3CalStat = IntVar()
#global J4CalStat
#J4CalStat = IntVar()
#global J5CalStat
#J5CalStat = IntVar()
#global J6CalStat
#J6CalStat = IntVar()
#global J7CalStat
#J7CalStat = IntVar()
#global J8CalStat
#J8CalStat = IntVar()
#global J9CalStat
#J9CalStat = IntVar()
#global J1CalStat2
#J1CalStat2 = IntVar()
#global J2CalStat2
#J2CalStat2 = IntVar()
#global J3CalStat2
#J3CalStat2 = IntVar()
#global J4CalStat2
#J4CalStat2 = IntVar()
#global J5CalStat2
#J5CalStat2 = IntVar()
#global J6CalStat2
#J6CalStat2 = IntVar()
#global J7CalStat2
#J7CalStat2 = IntVar()
#global J8CalStat2
#J8CalStat2 = IntVar()
#global J9CalStat2
#J9CalStat2 = IntVar()
#global IncJogStat
#IncJogStat = IntVar()
#global fullRot
#fullRot = IntVar()
#global pick180
#pick180 = IntVar()
#global pickClosest
#pickClosest = IntVar()
#global autoBG
#autoBG = IntVar()
#global estopActive
#estopActive = False
#global posOutreach
#posOutreach = False
#global SplineTrue
#SplineTrue = False
#global gcodeSpeed
#gcodeSpeed = "10"
#global inchTrue
#inchTrue = False
#global moveInProc
#moveInProc = 0
#global liveJog
#liveJog = False
#global progRunning
#progRunning = False
#offlineMode = False
#global setColor
#global renderer
#color_map = {}
#J1StepM = None
#J2StepM = None
#J3StepM = None
#J4StepM = None
#J5StepM = None
#J6StepM = None
#oriImage = None
#DHparams = None
#StepMonitors = [0] * 6
#rndSpeed = 0
#minSpeedDelay = 200  # µs
#speedViolation = "0"
#mainMode = 1
# Robot constants and placeholders (you should replace these with actual values)
#ROBOT_nDOFs = 6
#SolutionMatrix = np.zeros((6, 2))
#joints_estimate = np.zeros(6)
###
#xyzuvw_In = np.zeros(6)
#KinematicError = 0
# Tool and base frame placeholders (4x4 matrices)
#Robot_BaseFrame = np.eye(4)
#Robot_ToolFrame = np.eye(4)
#Robot_Data = np.zeros(66)  # Replace with actual DK values

#cam_on = False
#cap = None

# global RUN['xboxUse']
# global curTheme

# Vision Find variables (full implementation on Tab 6)
RUN['selectedTemplate'] = StringVar()
RUN['selectedTemplate'].set("")

live_jog_lock = threading.Lock()
live_cartesian_lock = threading.Lock()
live_tool_lock = threading.Lock()
drive_lock = threading.Lock()
serial_lock = threading.Lock()

'''
RUN['J1CalStat1'] = IntVar()
RUN['J2CalStat1'] = IntVar()
RUN['J3CalStat1'] = IntVar()
RUN['J4CalStat1'] = IntVar()
RUN['J5CalStat1'] = IntVar()
RUN['J6CalStat1'] = IntVar()
RUN['J8CalStat1'] = IntVar()
RUN['J9CalStat1'] = IntVar()
RUN['J7CalStat1'] = IntVar()

RUN['J1CalStat2'] = IntVar()
RUN['J2CalStat2'] = IntVar()
RUN['J3CalStat2'] = IntVar()
RUN['J4CalStat2'] = IntVar()
RUN['J5CalStat2'] = IntVar()
RUN['J6CalStat2'] = IntVar()
RUN['J7CalStat2'] = IntVar()
RUN['J8CalStat2'] = IntVar()
RUN['J9CalStat2'] = IntVar()
'''

RUN['IncJogStat'] = IntVar()
RUN['fullRot'] = IntVar()
RUN['pick180'] = IntVar()
RUN['pickClosest'] = IntVar()
RUN['autoBG'] = IntVar()
RUN['estopActive'] = False
RUN['posOutreach'] = False
RUN['gcodeSpeed'] = "10"

RUN['inchTrue'] = False
RUN['moveInProc'] = 0
RUN['liveJog'] = False
RUN['progRunning'] = False
RUN['offlineMode'] = False

RUN['color_map'] = {}

RUN['J1StepM'] = None
RUN['J2StepM'] = None
RUN['J3StepM'] = None
RUN['J4StepM'] = None
RUN['J5StepM'] = None
RUN['J6StepM'] = None

RUN['oriImage'] = None
RUN['StepMonitors'] = [0] * 6
RUN['minSpeedDelay'] = 200 #µs
RUN['speedViolation'] = "0"

RUN['xyzuvw_In'] = np.zeros(6)
RUN['KinematicError'] = 0

RUN['cam_on'] = False
RUN['cap'] = None

# Migrated global variables to RUN dictionary
# Robot State & Control
RUN['Alarm'] = None
RUN['VR_angles'] = None
RUN['JangleOut'] = None
RUN['JstepCur'] = None
RUN['JointMin'] = None
RUN['JointMax'] = None
RUN['cur_steps'] = None
RUN['J1axisLimNeg'] = None
RUN['J2axisLimNeg'] = None
RUN['J3axisLimNeg'] = None
RUN['J4axisLimNeg'] = None
RUN['J5axisLimNeg'] = None
RUN['J6axisLimNeg'] = None
RUN['negLim'] = None
RUN['stepDeg'] = None
RUN['flag'] = None
RUN['xyzuvw_Out'] = None
RUN['LineDist'] = None
RUN['Xv'] = None
RUN['Yv'] = None
RUN['Zv'] = None
RUN['xVal'] = None
RUN['yVal'] = None
RUN['zVal'] = None

# Serial Communication
RUN['ser'] = None
RUN['ser2'] = None
RUN['ser3'] = None

# Program Execution
RUN['rowinproc'] = None
RUN['stopQueue'] = None
RUN['splineActive'] = None
RUN['GCrowinproc'] = None
RUN['GCstopQueue'] = None

# Live Jog State
RUN['_current'] = None
RUN['_pending_start'] = None
RUN['_cart_current'] = None
RUN['_cart_pending'] = None
RUN['_tool_current'] = None
RUN['_tool_pending'] = None
RUN['_last_input_time'] = None
RUN['_mainMode'] = None
RUN['_smooth'] = None
RUN['_grip_closed'] = None
RUN['_pneu_open'] = None
RUN['cmdType'] = None

# Vision System
RUN['cropping'] = False
RUN['button_down'] = None
RUN['box_points'] = None
RUN['x_start'] = None
RUN['y_start'] = None
RUN['x_end'] = None
RUN['y_end'] = None
RUN['mX1'] = None
RUN['mY1'] = None
RUN['mX2'] = None
RUN['mY2'] = None
RUN['prevxVal'] = None
RUN['prevyVal'] = None
RUN['prevzVal'] = None
RUN['xMMpos'] = None
RUN['yMMpos'] = None
RUN['BGavg'] = None

# 3D Visualization
RUN['vtk_running'] = False
RUN['actors'] = {}
RUN['assemblies'] = {}
RUN['base_transforms'] = {}
RUN['joint_transforms'] = {}
RUN['composite_transforms'] = {}
RUN['interactor'] = None
RUN['render_window'] = None

# Input Devices
RUN['xboxUse'] = None
RUN['selectedTemplate'] = None
RUN['selectedCam'] = None


#declare axis limit vars
#! These are probably not necesary anymore but verify
CAL['J1PosLim'] = 0
CAL['J1NegLim'] = 0
CAL['J2PosLim'] = 0
CAL['J2NegLim'] = 0
CAL['J3PosLim'] = 0
CAL['J3NegLim'] = 0
CAL['J4PosLim'] = 0
CAL['J4NegLim'] = 0
CAL['J5PosLim'] = 0
CAL['J5NegLim'] = 0
CAL['J6PosLim'] = 0
CAL['J6NegLim'] = 0
CAL['J7PosLim'] = 0
CAL['J1CalStatVal'] = tk.IntVar(value=0)
CAL['J2CalStatVal'] = tk.IntVar(value=0)
CAL['J3CalStatVal'] = tk.IntVar(value=0)
CAL['J4CalStatVal'] = tk.IntVar(value=0)
CAL['J5CalStatVal'] = tk.IntVar(value=0)
CAL['J6CalStatVal'] = tk.IntVar(value=0)
CAL['J7CalStatVal'] = tk.IntVar(value=0)
CAL['J8CalStatVal'] = tk.IntVar(value=0)
CAL['J9CalStatVal'] = tk.IntVar(value=0)
CAL['J1CalStatVal2'] = tk.IntVar(value=0)
CAL['J2CalStatVal2'] = tk.IntVar(value=0)
CAL['J2CalStatVal2'] = tk.IntVar(value=0)
CAL['J3CalStatVal2'] = tk.IntVar(value=0)
CAL['J4CalStatVal2'] = tk.IntVar(value=0)
CAL['J5CalStatVal2'] = tk.IntVar(value=0)
CAL['J6CalStatVal2'] = tk.IntVar(value=0)
CAL['J7CalStatVal2'] = tk.IntVar(value=0)
CAL['J8CalStatVal2'] = tk.IntVar(value=0)
CAL['J9CalStatVal2'] = tk.IntVar(value=0)
CAL['J1OpenLoopVal'] = tk.IntVar(value=0)
CAL['J2OpenLoopVal'] = tk.IntVar(value=0)
CAL['J3OpenLoopVal'] = tk.IntVar(value=0)
CAL['J4OpenLoopVal'] = tk.IntVar(value=0)
CAL['J5OpenLoopVal'] = tk.IntVar(value=0)
CAL['J6OpenLoopVal'] = tk.IntVar(value=0)
CAL['DisableWristRotVal'] = tk.IntVar(value=0)



#J7NegLim = 0
#J8PosLim = 0
#J8NegLim = 0
#J9PosLim = 0
#J9NegLim = 0


#############################################################################################
### KINEMATICS FOR VIR ROBOT ################################################################
#############################################################################################

#DEG2RAD = np.pi / 180
#RAD2DEG = 180 / np.pi

def update_CPP_kin_from_entries():
    #global DHparams
    #global Robot_Data
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
        
        PosLimits = [float(val) for val in [CAL['J1PosLim'], CAL['J2PosLim'], CAL['J3PosLim'], CAL['J4PosLim'], CAL['J5PosLim'], CAL['J6PosLim']]]
        NegLimits = [float(val) for val in [CAL['J1NegLim'], CAL['J2NegLim'], CAL['J3NegLim'], CAL['J4NegLim'], CAL['J5NegLim'], CAL['J6NegLim']]]
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
    #global StepMonitors
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['VR_angles']
    RUN['StepMonitors'][0] = (float(RUN['VR_angles'][0]) + float(CAL['J1NegLim'])) * float(CAL['J1StepDeg'])
    RUN['StepMonitors'][1] = (float(RUN['VR_angles'][1]) + float(CAL['J2NegLim'])) * float(CAL['J2StepDeg'])
    RUN['StepMonitors'][2] = (float(RUN['VR_angles'][2]) + float(CAL['J3NegLim'])) * float(CAL['J3StepDeg'])
    RUN['StepMonitors'][3] = (float(RUN['VR_angles'][3]) + float(CAL['J4NegLim'])) * float(CAL['J4StepDeg'])                                              
    RUN['StepMonitors'][4] = (float(RUN['VR_angles'][4]) + float(CAL['J5NegLim'])) * float(CAL['J5StepDeg'])
    RUN['StepMonitors'][5] = (float(RUN['VR_angles'][5]) + float(CAL['J6NegLim'])) * float(CAL['J6StepDeg'])
    RUN['J1StepM'] = RUN['StepMonitors'][0]
    RUN['J2StepM'] = RUN['StepMonitors'][1]
    RUN['J3StepM'] = RUN['StepMonitors'][2]
    RUN['J4StepM'] = RUN['StepMonitors'][3]
    RUN['J5StepM'] = RUN['StepMonitors'][4]
    RUN['J6StepM'] = RUN['StepMonitors'][5]

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
def refresh_gui_from_joint_angles(joint_angles):
    setStepMonitorsVR()
    # Forward kinematics to get XYZ + orientation
    try:
        fk_xyzuvw = robot.forward_kinematics(joint_angles)
        xyzuvw = fk_xyzuvw[:3] + [math.degrees(v) for v in fk_xyzuvw[3:]]
    except Exception as e:
        logger.error(f"Forward kinematics failed: {e}")
        return

    # Cast and unpack as strings with 3 decimal places
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [f"{v:.3f}" for v in xyzuvw]

    # Update cartesian position fields
    XcurEntryField.delete(0, 'end'); XcurEntryField.insert(0, CAL['XcurPos'])
    YcurEntryField.delete(0, 'end'); YcurEntryField.insert(0, CAL['YcurPos'])
    ZcurEntryField.delete(0, 'end'); ZcurEntryField.insert(0, CAL['ZcurPos'])
    RzcurEntryField.delete(0, 'end'); RzcurEntryField.insert(0, CAL['RzcurPos'])
    RycurEntryField.delete(0, 'end'); RycurEntryField.insert(0, CAL['RycurPos'])
    RxcurEntryField.delete(0, 'end'); RxcurEntryField.insert(0, CAL['RxcurPos'])

    # Update jog sliders
    J1jogslide.set(joint_angles[0])
    J2jogslide.set(joint_angles[1])
    J3jogslide.set(joint_angles[2])
    J4jogslide.set(joint_angles[3])
    J5jogslide.set(joint_angles[4])
    J6jogslide.set(joint_angles[5])



    CAL['J1AngCur'] = str(joint_angles[0])
    CAL['J2AngCur'] = str(joint_angles[1])
    CAL['J3AngCur'] = str(joint_angles[2])
    CAL['J4AngCur'] = str(joint_angles[3])
    CAL['J5AngCur'] = str(joint_angles[4])
    CAL['J6AngCur'] = str(joint_angles[5])

    if CAL['J5AngCur'] != '' and float(CAL['J5AngCur']) > 0:
      RUN['WC'] = "F"
    else:
      RUN['WC'] = "N"

    logger.info(CAL['J5AngCur'])  

    J1curAngEntryField.delete(0, 'end')
    J1curAngEntryField.insert(0,CAL['J1AngCur'])
    J2curAngEntryField.delete(0, 'end')
    J2curAngEntryField.insert(0,CAL['J2AngCur'])
    J3curAngEntryField.delete(0, 'end')
    J3curAngEntryField.insert(0,CAL['J3AngCur'])
    J4curAngEntryField.delete(0, 'end')
    J4curAngEntryField.insert(0,CAL['J4AngCur'])
    J5curAngEntryField.delete(0, 'end')
    J5curAngEntryField.insert(0,CAL['J5AngCur'])
    J6curAngEntryField.delete(0, 'end')
    J6curAngEntryField.insert(0,CAL['J6AngCur'])   

    J1jogslide.set(CAL['J1AngCur'])
    J2jogslide.set(CAL['J2AngCur'])
    J3jogslide.set(CAL['J3AngCur'])
    J4jogslide.set(CAL['J4AngCur'])
    J5jogslide.set(CAL['J5AngCur'])
    J6jogslide.set(CAL['J6AngCur'])

    # Update joint angle fields
    J1curAngEntryField.delete(0, 'end'); J1curAngEntryField.insert(0, CAL['J1AngCur'])
    J2curAngEntryField.delete(0, 'end'); J2curAngEntryField.insert(0, CAL['J2AngCur'])
    J3curAngEntryField.delete(0, 'end'); J3curAngEntryField.insert(0, CAL['J3AngCur'])
    J4curAngEntryField.delete(0, 'end'); J4curAngEntryField.insert(0, CAL['J4AngCur'])
    J5curAngEntryField.delete(0, 'end'); J5curAngEntryField.insert(0, CAL['J5AngCur'])
    J6curAngEntryField.delete(0, 'end'); J6curAngEntryField.insert(0, CAL['J6AngCur'])










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

    
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    #global xyzuvw_In
    # global RUN['xyzuvw_Out'], RUN['stepDeg'], RUN['negLim'], RUN['VR_angles']

    limits = robot.get_joint_limits()

    steps = [int(round(J1step)), int(round(J2step)), int(round(J3step)),
             int(round(J4step)), int(round(J5step)), int(round(J6step))]
    dirs = [J1dir, J2dir, J3dir, J4dir, J5dir, J6dir]
    RUN['StepMonitors'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
    prev_StepMonitors = RUN['StepMonitors'].copy()

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
    elif SpeedType == "m" and RUN['xyzuvw_In'] and RUN['xyzuvw_Out']:
        dx = RUN['xyzuvw_In'][0] - RUN['xyzuvw_Out'][0]
        dy = RUN['xyzuvw_In'][1] - RUN['xyzuvw_Out'][1]
        dz = RUN['xyzuvw_In'][2] - RUN['xyzuvw_Out'][2]
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

        if calcStepGap < RUN['minSpeedDelay']:
            calcStepGap = RUN['minSpeedDelay']
            try:
                RUN['speedViolation'] = "1"
            except NameError:
                pass  # only if your Python sim doesn't use this flag
    elif SpeedType == "p":
        calcStepGap = RUN['minSpeedDelay'] / (SpeedVal / 100.0)
    else:
        calcStepGap = RUN['minSpeedDelay']

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
                            RUN['StepMonitors'][i] += 1 if dirs[i] else -1
                            RUN['VR_angles'][i] = (RUN['StepMonitors'][i] / RUN['stepDeg'][i]) - RUN['negLim'][i]

                            if RUN['StepMonitors'][i] != prev_StepMonitors[i]:
                                prev_StepMonitors[i] = RUN['StepMonitors'][i]
                    else:
                        SE_1cur[i] = 0
                else:
                    SE_2cur[i] = 0

        highStepCur += 1
        time.sleep(max((curDelay - disDelayCur), 0) / 1_000_000)

    RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM'] = RUN['StepMonitors']
    if RUN['offlineMode'] and not RUN['liveJog']:
        refresh_gui_from_joint_angles(RUN['VR_angles'])


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
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['cur_steps'], RUN['Alarm']

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
    int(round((Jangles[0] + RUN['J1axisLimNeg']) * float(CAL['J1StepDeg']))),
    int(round((Jangles[1] + RUN['J2axisLimNeg']) * float(CAL['J2StepDeg']))),
    int(round((Jangles[2] + RUN['J3axisLimNeg']) * float(CAL['J3StepDeg']))),
    int(round((Jangles[3] + RUN['J4axisLimNeg']) * float(CAL['J4StepDeg']))),
    int(round((Jangles[4] + RUN['J5axisLimNeg']) * float(CAL['J5StepDeg']))),
    int(round((Jangles[5] + RUN['J6axisLimNeg']) * float(CAL['J6StepDeg']))),
    ]


    RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
    step_degs = [CAL['J1StepDeg'], CAL['J2StepDeg'], CAL['J3StepDeg'], CAL['J4StepDeg'], CAL['J5StepDeg'], CAL['J6StepDeg']]
    step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

    step_difs = [int(round(cur - fut)) for cur, fut in zip(RUN['cur_steps'], fut_steps)]


    dirs = [1 if diff <= 0 else 0 for diff in step_difs]
    faults = []
    

    for i in range(6):
        if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
            faults.append(1)
        elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
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
        if RUN['offlineMode']:
          RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
          ErrorHandler(RUN['Alarm'])


def mj_command(inData):
    #global xyzuvw_In, KinematicError, Robot_Data
    # global RUN['JstepCur'], RUN['JointMin'], RUN['JointMax'], RUN['JangleOut']
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    # global RUN['cur_steps'], RUN['Alarm'], RUN['VR_angles']

    logger.info(inData)

    result = parse_mj_command(inData)
    if not result:
        if RUN['offlineMode']:
          ErrorHandler("ER")
        return

    # Extract values
    RUN['xyzuvw_In'][:] = result["xyzuvw"]
    SpeedVal = result["Speed"]
    ACCspd = result["Acc"]
    DCCspd = result["Dec"]
    ACCramp = result["Ramp"]
    SpeedType = inData[inData.find("S") + 1]

    RUN['xyzuvw_In'] = np.array(RUN['xyzuvw_In'], dtype=float)

    # IK call
    RUN['JangleOut'] = robot.SolveInverseKinematics(RUN['xyzuvw_In'], RUN['VR_angles'])

    if RUN['JangleOut'] is None:
        if RUN['offlineMode']:
          logger.error("Inverse kinematics failed. No solution found.")
          ErrorHandler("ER")
        return

 
    RUN['JangleOut'] = np.array(RUN['JangleOut'], dtype=np.float64).flatten()

    # Convert angles to steps
    step_degs = [float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
             float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])]
    axis_neg = [float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
            float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])]
    fut_steps = [int(round((j + off) * deg)) for j, off, deg in zip(RUN['JangleOut'], axis_neg, step_degs)]

    RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
    step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

    step_difs = [int(round(cur - fut)) for cur, fut in zip(RUN['cur_steps'], fut_steps)]

    dirs = [1 if diff <= 0 else 0 for diff in step_difs]
    faults = []

    for i in range(6):
        if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
            faults.append(1)
        elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
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
        if RUN['offlineMode']:
          RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
          ErrorHandler(RUN['Alarm'])
          logger.error(RUN['Alarm'])




def mt_command(inData):
    #global xyzuvw_In, KinematicError
    # global RUN['JangleOut'], RUN['Alarm'], RUN['VR_angles']
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    # global RUN['cur_steps']
    #global offlineMode

    result = parse_mt_command(inData)
    if not result:
        if RUN['offlineMode']:
            ErrorHandler("ER")
        return
    
    offset = [float(v) for v in result["offset_vector"]]
    robot.set_robot_tool_frame(*offset)

    # Build xyzuvw_In from current pose
    RUN['xyzuvw_In'] = np.array([
        float(CAL['XcurPos']),
        float(CAL['YcurPos']),
        float(CAL['ZcurPos']),
        float(CAL['RzcurPos']),
        float(CAL['RycurPos']),
        float(CAL['RxcurPos'])
    ], dtype=float)


    # IK solve
    RUN['JangleOut'] = robot.SolveInverseKinematics(RUN['xyzuvw_In'], RUN['VR_angles'])

    # put tool frame back where it was
    robot.set_robot_tool_frame(float(TFxEntryField.get()), 
                                   float(TFyEntryField.get()), 
                                   float(TFzEntryField.get()), 
                                   float(TFrxEntryField.get()), 
                                   float(TFryEntryField.get()), 
                                   float(TFrzEntryField.get()))

    if RUN['JangleOut'] is None:
        if RUN['offlineMode']:
            logger.error("Inverse kinematics failed. No solution found.")
            ErrorHandler("ER")
        return

    RUN['JangleOut'] = np.array(RUN['JangleOut'], dtype=np.float64).flatten()

    # Convert angles to steps
    step_degs = [float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
                 float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])]
    axis_neg = [float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
                float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])]
    fut_steps = [int(round((j + off) * deg)) for j, off, deg in zip(RUN['JangleOut'], axis_neg, step_degs)]

    RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
    step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

    step_difs = [int(round(cur - fut)) for cur, fut in zip(RUN['cur_steps'], fut_steps)]
    dirs = [1 if diff <= 0 else 0 for diff in step_difs]

    # Check limits
    faults = []
    for i in range(6):
        if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
            faults.append(1)
        elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
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
        if RUN['offlineMode']:
            RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
            ErrorHandler(RUN['Alarm'])     
     


def live_joint_jog(in_data):
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['VR_angles'], RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    # global RUN['Alarm'], RUN['flag']
    #global liveJog, KinematicError

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
        RUN['Alarm'] = "ER"
        ErrorHandler(RUN['Alarm'])
        return

    RUN['liveJog'] = True
    while RUN['liveJog']:
        while drive_lock.locked():
                    time.sleep(0.005)   
        try:
            Jangles = [float(a) for a in RUN['VR_angles'][:6]]
        except Exception as e:
            if RUN['offlineMode']:
              logger.error("Invalid RUN['VR_angles']:", RUN['VR_angles'][:6])
              RUN['Alarm'] = "ER"
              ErrorHandler(RUN['Alarm'])
            return

        Jangles[idx] += direction * .1

        axis_lims = [
            float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
            float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])
        ]
        step_degs = [
            float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
            float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])
        ]
        step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]
        RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]

        fut_steps = [int(round((Jangles[i] + axis_lims[i]) * step_degs[i])) for i in range(6)]
        step_difs = [cur - fut for cur, fut in zip(RUN['cur_steps'], fut_steps)]
        dirs = [1 if diff <= 0 else 0 for diff in step_difs]

        faults = []
        for i in range(6):
            if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
                faults.append(1)
            elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
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
            if RUN['offlineMode']:
              RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
              ErrorHandler(RUN['Alarm'])
              RUN['Alarm'] = "0"
            break



def live_cartesian_jog(in_data):
    #global xyzuvw_In, KinematicError
    # global RUN['xyzuvw_Out'], RUN['VR_angles'], RUN['JangleOut'], RUN['Alarm']
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    #global liveJog

    # Parse command
    Vector = float(in_data[in_data.index("V") + 1:in_data.index("S")])
    SpeedType = in_data[in_data.index("S") + 1]
    SpeedVal = float(in_data[in_data.index("S") + 2:in_data.index("Ac")])
    ACCspd = DCCspd = ACCramp = 100  # fixed for now
    LoopModeStr = in_data.split("Lm")[1].strip()
    LoopModes = [int(c) for c in LoopModeStr]

    # Cartesian jog increment
    jog_step = 1  # mm or deg, depending on axis

    RUN['xyzuvw_In'] = np.array([
        float(CAL['XcurPos']),
        float(CAL['YcurPos']),
        float(CAL['ZcurPos']),
        float(CAL['RzcurPos']),
        float(CAL['RycurPos']),
        float(CAL['RxcurPos'])
    ], dtype=float)

    RUN['liveJog'] = True
    while RUN['liveJog']:
        while drive_lock.locked():
            time.sleep(0.005)   

        idx = int(Vector // 10) - 1
        direction = 1 if int(Vector) % 10 == 1 else -1

        if 0 <= idx < 6:
            RUN['xyzuvw_In'][idx] += direction * jog_step
        else:
            RUN['Alarm'] = "ER"
            #ErrorHandler(Alarm)
            break

        # Inverse Kinematics
        try:
            RUN['JangleOut'] = robot.SolveInverseKinematics(RUN['xyzuvw_In'], RUN['VR_angles'])
        except Exception as e:
            logger.error("IK Exception:", e)
            ErrorHandler("ER")
            break

        if RUN['JangleOut'] is None:
            if RUN['offlineMode']:
              RUN['Alarm'] = "ER"
              ErrorHandler(RUN['Alarm'])
            break
        
        RUN['JangleOut'] = np.array(RUN['JangleOut'], dtype=np.float64).flatten()

        # Convert angles to steps
        step_degs = [
            float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
            float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])
        ]
        axis_lims = [
            float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
            float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])
        ]
        #fut_steps = [int(round((float(j) + float(off)) * float(deg))) for j, off, deg in zip(JangleOut, axis_neg, step_degs)]
        fut_steps = [int(round((RUN['JangleOut'][i] + axis_lims[i]) * step_degs[i])) for i in range(6)]

        RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
        step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

        step_difs = [cur - fut for cur, fut in zip(RUN['cur_steps'], fut_steps)]
        dirs = [1 if diff <= 0 else 0 for diff in step_difs]

        # Check axis limits
        faults = []
        for i in range(6):
            if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
                faults.append(1)
            elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
                faults.append(1)
            else:
                faults.append(0)

        if sum(faults) == 0 and RUN['KinematicError'] == 0:
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
            RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
            ErrorHandler(RUN['Alarm'])
            break


def live_tool_jog(in_data):
    #global xyzuvw_In, KinematicError
    # global RUN['JangleOut'], RUN['Alarm'], RUN['VR_angles']
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    # global TFxEntryField, TFyEntryField, TFzEntryField, TFrxEntryField, TFryEntryField, TFrzEntryField
    #global liveJog
    #global offlineMode

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

    RUN['liveJog'] = True
    while RUN['liveJog']:
        while drive_lock.locked():
            time.sleep(0.005)

        idx = int(Vector // 10) - 1
        if idx == 3:  # Rz → Trx
            idx = 5
        elif idx == 5:  # Rx → Trz
            idx = 3

        direction = 1 if int(Vector) % 10 == 1 else -1

        # Build pose from current position
        RUN['xyzuvw_In'] = robot.forward_kinematics(RUN['VR_angles'])
        RUN['xyzuvw_In'] = RUN['xyzuvw_In'][:3] + [math.degrees(v) for v in RUN['xyzuvw_In'][3:]]   

        if 0 <= idx < 6:
            # Modify tool frame temporarily
            jogged_tool_frame = original_tool_frame.copy()
            jogged_tool_frame[idx] += direction * jog_step
            robot.set_robot_tool_frame(*jogged_tool_frame)
        else:
            RUN['Alarm'] = "ER"
            ErrorHandler(RUN['Alarm'])
            break

        try:
            RUN['JangleOut'] = robot.SolveInverseKinematics(RUN['xyzuvw_In'], RUN['VR_angles'])
        except Exception as e:
            logger.error(f"IK Exception: {e}")
            ErrorHandler("ER")
            break

        # Restore original tool frame
        robot.set_robot_tool_frame(*original_tool_frame)

        if RUN['JangleOut'] is None:
            if RUN['offlineMode']:
                RUN['Alarm'] = "ER"
                ErrorHandler(RUN['Alarm'])
            break

        RUN['JangleOut'] = np.array(RUN['JangleOut'], dtype=np.float64).flatten()

        step_degs = [
            float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
            float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])
        ]
        axis_lims = [
            float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
            float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])
        ]
        fut_steps = [int(round((RUN['JangleOut'][i] + axis_lims[i]) * step_degs[i])) for i in range(6)]

        RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
        step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

        step_difs = [cur - fut for cur, fut in zip(RUN['cur_steps'], fut_steps)]
        dirs = [1 if diff <= 0 else 0 for diff in step_difs]

        # Axis limit check
        faults = []
        for i in range(6):
            if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
                faults.append(1)
            elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
                faults.append(1)
            else:
                faults.append(0)

        if sum(faults) == 0 and RUN['KinematicError'] == 0:
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
            RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
            ErrorHandler(RUN['Alarm'])
            break
        



#############################################################################################
### VIRTUAL ROBOT ###########################################################################
#############################################################################################

# Global storage
RUN['vtk_running'] = False
RUN['actors'] = {}
RUN['assemblies'] = {}
RUN['base_transforms'] = {}
RUN['joint_transforms'] = {}
RUN['composite_transforms'] = {}


def toggle_offline_mode():
    #global offlineMode
    # global RUN['VR_angles']
    RUN['offlineMode'] = not RUN['offlineMode']
    if RUN['offlineMode']:
        offline_button.config(text="Go Online", style="Offline.TButton")
        almStatusLab.config(text="SYSTEM IN OFFLINE MODE", style="Warn.TLabel")
        almStatusLab2.config(text="SYSTEM IN OFFLINE MODE", style="Warn.TLabel")
        RUN['VR_angles'] = [0.000, 0.000, 0.000, 0.000, 90.000, 0.000]
        J1negLimLab.config(text="-"+CAL['J1NegLim'], style="Jointlim.TLabel")
        J1posLimLab.config(text=CAL['J1PosLim'], style="Jointlim.TLabel")
        J1jogslide.config(from_=float("-"+CAL['J1NegLim']), to=float(CAL['J1PosLim']),  length=180, orient=HORIZONTAL,  command=J1sliderUpdate)
        J2negLimLab.config(text="-"+CAL['J2NegLim'], style="Jointlim.TLabel")
        J2posLimLab.config(text=CAL['J2PosLim'], style="Jointlim.TLabel")
        J2jogslide.config(from_=float("-"+CAL['J2NegLim']), to=float(CAL['J2PosLim']),  length=180, orient=HORIZONTAL,  command=J2sliderUpdate)
        J3negLimLab.config(text="-"+CAL['J3NegLim'], style="Jointlim.TLabel")
        J3posLimLab.config(text=CAL['J3PosLim'], style="Jointlim.TLabel")
        J3jogslide.config(from_=float("-"+CAL['J3NegLim']), to=float(CAL['J3PosLim']),  length=180, orient=HORIZONTAL,  command=J3sliderUpdate)
        J4negLimLab.config(text="-"+CAL['J4NegLim'], style="Jointlim.TLabel")
        J4posLimLab.config(text=CAL['J4PosLim'], style="Jointlim.TLabel")
        J4jogslide.config(from_=float("-"+CAL['J4NegLim']), to=float(CAL['J4PosLim']),  length=180, orient=HORIZONTAL,  command=J4sliderUpdate)
        J5negLimLab.config(text="-"+CAL['J5NegLim'], style="Jointlim.TLabel")
        J5posLimLab.config(text=CAL['J5PosLim'], style="Jointlim.TLabel")
        J5jogslide.config(from_=float("-"+CAL['J5NegLim']), to=float(CAL['J5PosLim']),  length=180, orient=HORIZONTAL,  command=J5sliderUpdate)
        J6negLimLab.config(text="-"+CAL['J6NegLim'], style="Jointlim.TLabel")
        J6posLimLab.config(text=CAL['J6PosLim'], style="Jointlim.TLabel")
        J6jogslide.config(from_=float("-"+CAL['J6NegLim']), to=float(CAL['J6PosLim']),  length=180, orient=HORIZONTAL,  command=J6sliderUpdate)
        refresh_gui_from_joint_angles(RUN['VR_angles'])

    else:
        offline_button.config(text="Run Offline", style="Online.TButton")
        almStatusLab.config(text="SYSTEM IN ONLINE MODE", style="OK.TLabel")
        almStatusLab2.config(text="SYSTEM IN ONLINE MODE", style="OK.TLabel")
        requestPos()
        RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
        setStepMonitorsVR()
        
def request_render():
    RUN['render_window'].Render()

def update_joint_transforms():
    angles = RUN['VR_angles']  # List of 6 joint angles in degrees

    joint_stl_keys = [
        "Link 1-1.STL",
        "Link 2-1.STL",
        "Link 3-1.STL",
        "Link 4-1.STL",
        "Link 5-1.STL",
        "Link 6-1.STL",
    ]

    for i, stl_key in enumerate(joint_stl_keys):
        joint_tf = RUN['joint_transforms'][stl_key]
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
    # global RUN['actors'], RUN['assemblies'], RUN['base_transforms'], RUN['joint_transforms'], RUN['composite_transforms']
    #global color_map

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
    RUN['color_map'].clear()
    RUN['color_map'].update({stl: "Silver" for stl in stl_files})
    RUN['color_map'].update({
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
    RUN['actors'].clear()
    RUN['assemblies'].clear()
    RUN['base_transforms'].clear()
    RUN['joint_transforms'].clear()
    RUN['composite_transforms'].clear()

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
        actor.GetProperty().SetColor(colors.GetColor3d(RUN['color_map'][stl]))

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

        RUN['actors'][stl] = actor
        RUN['assemblies'][stl] = asm
        RUN['base_transforms'][stl] = base_tf
        RUN['joint_transforms'][stl] = joint_tf
        RUN['composite_transforms'][stl] = comp_tf

    # Build hierarchy
    root = RUN['assemblies']["Link Base-1.STL"]
    root.AddPart(RUN['assemblies']["Link Base-2.STL"])
    RUN['assemblies']["Link Base-2.STL"].AddPart(RUN['assemblies']["Link Base-3.STL"])
    RUN['assemblies']["Link Base-3.STL"].AddPart(RUN['assemblies']["Link 1-1.STL"])
    RUN['assemblies']["Link 1-1.STL"].AddPart(RUN['assemblies']["Link 1-2.STL"])
    RUN['assemblies']["Link 1-2.STL"].AddPart(RUN['assemblies']["Link 2-1.STL"])
    RUN['assemblies']["Link 2-1.STL"].AddPart(RUN['assemblies']["Link 2-2.STL"])
    RUN['assemblies']["Link 2-2.STL"].AddPart(RUN['assemblies']["Link 2-3.STL"])
    RUN['assemblies']["Link 2-3.STL"].AddPart(RUN['assemblies']["Link 3-1.STL"])
    RUN['assemblies']["Link 3-1.STL"].AddPart(RUN['assemblies']["Link 3-2.STL"])
    RUN['assemblies']["Link 3-2.STL"].AddPart(RUN['assemblies']["Link 4-1.STL"])
    RUN['assemblies']["Link 4-1.STL"].AddPart(RUN['assemblies']["Link 4-2.STL"])
    RUN['assemblies']["Link 4-2.STL"].AddPart(RUN['assemblies']["Link 4-3.STL"])
    RUN['assemblies']["Link 4-3.STL"].AddPart(RUN['assemblies']["Link 5-1.STL"])
    RUN['assemblies']["Link 5-1.STL"].AddPart(RUN['assemblies']["Link 5-2.STL"])
    RUN['assemblies']["Link 5-2.STL"].AddPart(RUN['assemblies']["Link 6-1.STL"])
    RUN['assemblies']["Link 6-1.STL"].AddPart(RUN['assemblies']["Link 6-2.STL"])

    renderer.AddActor(root)


class CustomInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, renderer):
        self.AddObserver("LeftButtonReleaseEvent", self.on_left_button_up)
        self.renderer = renderer

    def on_left_button_up(self, obj, event):
        self.OnLeftButtonUp()  # <-- CORRECT way to call the base method

def update_joint_angles():
    angles = {
        "Link 1-1.STL": -RUN['VR_angles'][0],
        "Link 2-1.STL": RUN['VR_angles'][1],
        "Link 3-1.STL": -RUN['VR_angles'][2],
        "Link 4-1.STL": -RUN['VR_angles'][3],
        "Link 5-1.STL": -RUN['VR_angles'][4],
        "Link 6-1.STL": RUN['VR_angles'][5]
    }

    for stl, angle in angles.items():
        jt = RUN['joint_transforms'][stl]
        jt.Identity()
        jt.RotateZ(angle)
        ct = RUN['composite_transforms'][stl]
        ct.Identity()
        ct.Concatenate(RUN['base_transforms'][stl])
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

    RUN['renderer'].AddActor(actor)        

def on_close_event(obj, event):
    # global RUN['vtk_running']
    RUN['vtk_running'] = False
    try:
        obj.GetRenderWindow().Finalize()
        obj.TerminateApp()
    except:
        pass

def update_vtk(render_window, root_widget):
    if RUN['vtk_running']:
        update_joint_angles()
        tab1.after(0, request_render)
        root_widget.after(16, lambda: update_vtk(RUN['render_window'], root_widget))


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
        click_pos = RUN['interactor'].GetEventPosition()
        x, y = click_pos
        if click_bounds['x1'] <= x <= click_bounds['x2'] and click_bounds['y1'] <= y <= click_bounds['y2']:
            camera.Azimuth(45)
            camera.Elevation(35)
            camera.SetViewUp(0, 0, 1)
            RUN['renderer'].ResetCamera()
            RUN['renderer'].ResetCameraClippingRange()
            RUN['interactor'].GetRenderWindow().Render()

    # Attach click handler
    RUN['interactor'].AddObserver("LeftButtonPressEvent", click_callback)        

def launch_vtk_nonblocking(root_widget):
    #global renderer
    # global RUN['vtk_running'], RUN['interactor'], RUN['render_window'], RUN['VR_angles']

    RUN['vtk_running'] = True

    RUN['renderer'] = vtk.vtkRenderer()
    RUN['render_window'] = vtk.vtkRenderWindow() 
    RUN['render_window'].SetWindowName("AR4 Virtual Robot Viewer")
    RUN['interactor'] = vtk.vtkRenderWindowInteractor()
    RUN['render_window'].AddRenderer(RUN['renderer'])
    RUN['interactor'].SetRenderWindow(RUN['render_window'])

    style = CustomInteractorStyle(RUN['renderer'])
    RUN['interactor'].SetInteractorStyle(style)

    RUN['render_window'].SetSize(1024, 768)
    RUN['renderer'].SetBackground(vtk.vtkNamedColors().GetColor3d("LightSlateGray"))

    build_robot_actors(RUN['renderer'])
    add_floor_grid(RUN['renderer'])

    camera = RUN['renderer'].GetActiveCamera()
    RUN['renderer'].ResetCamera()
    camera.Dolly(3)
    camera.Azimuth(65)
    camera.Elevation(55)
    camera.SetViewUp(0, 0, 1)
    RUN['renderer'].ResetCameraClippingRange()

    #add_reset_view_button(RUN['renderer'], interactor, camera)

    RUN['interactor'].AddObserver("ExitEvent", on_close_event)
    RUN['interactor'].Initialize()
    RUN['render_window'].Render()
    
    # Set window to stay on top
    set_vtk_topmost_delayed(RUN['render_window'], root_widget)

    # Embed periodic update and check render loop
    update_vtk(RUN['render_window'], root_widget)

    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
    update_main_color()





def set_vtk_topmost_delayed(render_window, root_widget):
    '''Continuously keep VTK window on top - cross-platform'''
    def keep_on_top():
        import time
        import platform
        import subprocess
        time.sleep(0.5)  # Initial delay for window creation
        
        attempt = 0
        os_type = platform.system()
        
        while RUN['vtk_running']:
            try:
                if os_type == 'Windows':
                    import win32gui
                    import win32con
                    
                    # Find window by title
                    hwnd = win32gui.FindWindow(None, "AR4 Virtual Robot Viewer")
                    
                    if hwnd and hwnd != 0:
                        win32gui.SetWindowPos(hwnd, win32con.HWND_TOPMOST, 0, 0, 0, 0,
                                            win32con.SWP_NOMOVE | win32con.SWP_NOSIZE | win32con.SWP_NOACTIVATE)
                
                elif os_type == 'Linux':
                    # Use wmctrl to set window above others
                    # First try to find the window
                    try:
                        result = subprocess.run(['wmctrl', '-l'], capture_output=True, text=True, check=False)
                        if result.returncode == 0:
                            for line in result.stdout.split('\n'):
                                    window_id = line.split()[0]
                                    # Set window to stay on top
                                    subprocess.run(['wmctrl', '-i', '-r', window_id, '-b', 'add,above'], 
                                                 stderr=subprocess.DEVNULL, check=False)
                                    break
                    except FileNotFoundError:
                        # wmctrl not installed, try xdotool as fallback
                        try:
                            result = subprocess.run(['xdotool', 'search', '--name', 'AR4 Virtual Robot Viewer'], 
                                                  capture_output=True, text=True, check=False)
                            if result.returncode == 0 and result.stdout.strip():
                                window_id = result.stdout.strip().split()[0]
                                subprocess.run(['xdotool', 'windowraise', window_id], 
                                             stderr=subprocess.DEVNULL, check=False)
                        except FileNotFoundError:
                            pass  # Neither wmctrl nor xdotool available
                
                attempt += 1
            except Exception as e:
                if attempt % 10 == 0:
                    logger.debug(f"VTK topmost error: {e}")
            
            time.sleep(0.5)  # Re-apply every 500ms
    
    # Run in a thread to not block
    import threading
    threading.Thread(target=keep_on_top, daemon=True).start()

def set_vtk_topmost_delayed(render_window, root_widget):
    '''Continuously keep VTK window on top using window title'''
    def keep_on_top():
        import time
        import platform
        time.sleep(0.5)  # Initial delay for window creation
        
        attempt = 0
        while RUN['vtk_running']:
            try:
                if platform.system() == 'Windows':
                    import win32gui
                    import win32con
                    
                    # Find window by title
                    hwnd = win32gui.FindWindow(None, "AR4 Virtual Robot Viewer")
                    
                    # Debug output every 10 attempts (every 3 seconds)
                    if attempt % 10 == 0:
                        print(f"[VTK Topmost Debug] Attempt {attempt}: hwnd={hwnd}")
                    
                    if hwnd and hwnd != 0:
                        result = win32gui.SetWindowPos(hwnd, win32con.HWND_TOPMOST, 0, 0, 0, 0,
                                            win32con.SWP_NOMOVE | win32con.SWP_NOSIZE | win32con.SWP_NOACTIVATE)
                        if attempt % 10 == 0:
                            print(f"[VTK Topmost Debug] SetWindowPos result: {result}")
                    else:
                        if attempt % 10 == 0:
                            print(f"[VTK Topmost Debug] Window not found!")
                    
                    attempt += 1
            except Exception as e:
                print(f"[VTK Topmost Debug] Error: {e}")
            time.sleep(0.3)  # Re-apply every 300ms
    
    # Run in a thread to not block
    import threading
    threading.Thread(target=keep_on_top, daemon=True).start()

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
    RUN['render_window'].Render()





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

    RUN['renderer'].AddActor(actor)
    RUN['render_window'].Render()

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
    RUN['render_window'].Render()

   
    




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
  # global RUN['ser']
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  CAL['comPort'] = com1SelectedValue.get()
  baud = 9600

  # If something was already open, close it first (prevents WinError 5 on switch)
  try:
    if 'ser' in globals() and ser and getattr(ser, "is_open", False):
      RUN['ser'].close()
      time.sleep(0.2)  # give Windows a moment to release the handle
  except Exception:
    pass

  try:
    if CAL['comPort'] in (None, "", "None"):
      raise ValueError("No COM port selected")
    # Add small timeouts so reads/writes can’t hang forever
    RUN['ser'] = serial.Serial(
      port=CAL['comPort'],
      baudrate=baud
    )
    logger.info("COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER on Port %s", CAL['comPort'])

    almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
    logger.info("COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER")

    time.sleep(.1)
    # Prefer reset_input_buffer over deprecated flushInput
    try:
      RUN['ser'].reset_input_buffer()
      RUN['ser'].reset_output_buffer()
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
        RUN['ser'].close()
        time.sleep(0.2)
    except Exception:
      pass

    # logger.exception("UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER")
    # logger.exception raises a new unhandled exception instead of just logging the issue
    logger.error("UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER")
    
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
    # global RUN['ser2']
    #port = "COM" + com2PortEntryField.get()
    CAL['com2Port'] = com2SelectedValue.get()
    baud = 9600

    if CAL['com2Port'] in (None, "", "None"):
      raise ValueError("No COM port selected")
    
    RUN['ser2'] = serial.Serial(CAL['com2Port'],baud)
    #almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
    #almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
    logger.info(f"COMMUNICATIONS STARTED WITH ARDUINO IO BOARD on port: {CAL['com2Port']}")
    #tab8.ElogView.insert(END, Curtime+f" - COMMUNICATIONS STARTED WITH ARDUINO IO BOARD on port: {port}")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
  except Exception as e:
    logger.debug(f"{e}")
    #tab8.ElogView.insert(END, Curtime+f" - Error: {e}")
    #almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD", style="Alarm.TLabel")
    #almStatusLab2.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD", style="Alarm.TLabel")
    logger.error(f"UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD")
    #tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
    
def darkTheme():
  CAL['curTheme'] = 0
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
  CAL['curTheme'] = 1
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
    # global RUN['rowinproc']
    # global RUN['stopQueue']
    # global RUN['splineActive']
    #global estopActive
    RUN['estopActive'] = False
    #global posOutreach
    RUN['posOutreach'] = False
    RUN['stopQueue'] = "0"
    RUN['splineActive'] = "0"
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
        if (RUN['estopActive']):
          almStatusLab.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
          almStatusLab2.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
        elif (RUN['posOutreach']):
          almStatusLab.config(text="Position Out of Reach",  style="Alarm.TLabel")
          almStatusLab2.config(text="Position Out of Reach",  style="Alarm.TLabel")  
        else:
          almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
          almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")  
      else:
        almStatusLab.config(text="PROGRAM RUNNING",  style="OK.TLabel")
        almStatusLab2.config(text="PROGRAM RUNNING",  style="OK.TLabel") 
      RUN['rowinproc'] = 1
      executeRow()
      while RUN['rowinproc'] == 1:
        time.sleep(.1)
      selRow = tab1.progView.curselection()[0]
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
        if (RUN['estopActive']):
          almStatusLab.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
          almStatusLab2.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
        elif (RUN['posOutreach']):
          almStatusLab.config(text="Position Out of Reach",  style="Alarm.TLabel")
          almStatusLab2.config(text="Position Out of Reach",  style="Alarm.TLabel")    
        else:
          almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
          almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel") 
  t = threading.Thread(target=threadProg)
  t.start()
  
def stepFwd():
    def threadProg():
      #global estopActive
      RUN['estopActive'] = False
      #global posOutreach
      RUN['posOutreach'] = False
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
    #global estopActive
    RUN['estopActive'] = False
    #global posOutreach
    RUN['posOutreach'] = False
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
  # global RUN['cmdType']
  # global RUN['splineActive']
  #global estopActive
  #global posOutreach
  # global RUN['stopQueue']
  lastProg = ""
  tab1.runTrue = 0
  if (RUN['estopActive']):
    almStatusLab.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
    almStatusLab2.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
  elif (RUN['posOutreach']):
    almStatusLab.config(text="Position Out of Reach",  style="Alarm.TLabel")
    almStatusLab2.config(text="Position Out of Reach",  style="Alarm.TLabel")    
  else:        
    almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
    almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")

  
  
def executeRow():
  # global RUN['rowinproc']
  # global RUN['LineDist']
  # global RUN['Xv']
  # global RUN['Yv']
  # global RUN['Zv']
  #global progRunning, offlineMode
  # global RUN['VR_angles'], RUN['stopQueue'], RUN['splineActive']
  #global moveInProc
  RUN['progRunning'] = True
  selRow = tab1.progView.curselection()[0]
  tab1.progView.see(selRow+2)
  data = list(map(int, tab1.progView.curselection()))
  command=tab1.progView.get(data[0]).decode().strip()
  RUN['cmdType'] =command[:6]
  cmdTypeLong=command[:11]
  
  ##Call Program##
  if (RUN['cmdType'] == "Call P"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
  if (RUN['cmdType'] == "Run Gc"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Gcode not supported in offline programming mode", style="Alarm.TLabel")
      return 
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
  if (RUN['cmdType'] == "Return"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
  if (RUN['cmdType'] == "Test L"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Test limit switches not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = "TL\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.05)
    response = str(RUN['ser'].readline().strip(),'utf-8')
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)

  ##Set Encoders 1000
  if (RUN['cmdType'] == "Set En"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Encoder testing not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = "SE\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.05)
    RUN['ser'].read() 

  ##Read Encoders
  if (RUN['cmdType'] == "Read E"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Read Encoders not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = "RE\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.05)
    response = str(RUN['ser'].readline().strip(),'utf-8')
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)

  ##Servo Command##
  if (RUN['cmdType'] == "Servo "):
    if RUN['offlineMode']:
      almStatusLab.config(text="Servo control not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    servoIndex = command.find("number ")
    posIndex = command.find("position: ")
    servoNum = str(command[servoIndex+7:posIndex-4])
    servoPos = str(command[posIndex+10:])
    command = "SV"+servoNum+"P"+servoPos+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser2'].write(command.encode())
    RUN['ser2'].flushInput()
    time.sleep(.1)
    RUN['ser2'].read() 

  ##If Input##
  if (RUN['cmdType'] == "If Inp"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
    RUN['ser2'].write(cmd.encode())
    RUN['ser2'].flushInput()
    time.sleep(.1)
    response = str(RUN['ser2'].readline().strip(),'utf-8')
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
  if (RUN['cmdType'] == "Read C"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    comIndex = command.find("# ")
    charIndex = command.find("Char: ")
    actionIndex = command.find(": ")
    comNum = str(command[comIndex+2:charIndex-1])
    charNum = int(command[charIndex+6:])
    try:
      # global RUN['ser3']    
      port = "COM" + comNum   
      baud = 9600    
      RUN['ser3'] = serial.Serial(port,baud,timeout=10)
    except:
      #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
      #tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
      logger.error("UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    RUN['ser3'].flushInput()
    response = str(RUN['ser3'].read(charNum).strip(),'utf-8')    
    com3outPortEntryField.delete(0, 'end')
    com3outPortEntryField.insert(0,response)
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)


  
  ##If Register##
  if (RUN['cmdType'] == "If Reg"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
  if (RUN['cmdType'] == "If COM"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
  if (RUN['cmdType'] == "If MBc"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
    RUN['ser'].write(subcommand.encode())
    RUN['ser'].flushInput()
    time.sleep(.1) 
    response = RUN['ser'].readline().decode("utf-8").strip()  
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
  if (RUN['cmdType'] == "If MBi"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
    RUN['ser'].write(subcommand.encode())
    RUN['ser'].flushInput()
    time.sleep(.1) 
    response = RUN['ser'].readline().decode("utf-8").strip()  
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
  if (RUN['cmdType'] == "If MBh"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
    RUN['ser'].write(subcommand.encode())
    RUN['ser'].flushInput()
    time.sleep(.1) 
    response = RUN['ser'].readline().decode("utf-8").strip()  
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
  if (RUN['cmdType'] == "If MBI"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
    RUN['ser'].write(subcommand.encode())
    RUN['ser'].flushInput()
    time.sleep(.1) 
    response = RUN['ser'].readline().decode("utf-8").strip()  
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
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    timeoutIndex = command.find("Timeout =")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:timeoutIndex-3])
    timeout = str(command[timeoutIndex+10:])
    command = "WI"+"A"+inputNum+"B"+valNum+"C"+timeout+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser2'].write(command.encode())
    RUN['ser2'].flushInput()
    time.sleep(.1)
    RUN['ser2'].read()
 

  ##Wait Modbus Coil##
  if (cmdTypeLong == "Wait MBcoil"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    RUN['ser'].read()

  ##Wait Modbus Input##
  if (cmdTypeLong == "Wait MBinpu"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    RUN['ser'].read()  
   
              

### this will fail on first run without reloading program - the insertion inserts it as bytes (due to pickling the program save) but when reloaded its a listbox of strings - therefore this would only work after reload and looking for string
  ## long term fix refactor all 
  ''' 
  ##Jump to Row##
    if (RUN['moveInProc'] == 1):
        RUN['moveInProc'] = 2
    tabIndex = command.find("Tab-")
    tabNum = ("Tab Number " + str(command[tabIndex+4:]) + "\r\n").encode('utf-8')
    index = tab1.progView.get(0, "end").index(tabNum)
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(index) 
  ''' 
  
  ## Jump to Row (bytes/str + CRLF tolerant) ##
  if RUN['cmdType'] == "Jump T":
      if (RUN['moveInProc'] == 1):
        RUN['moveInProc'] = 2

      tabIndex = command.find("Tab-")
      if tabIndex == -1:
          print("[Jump T] Malformed command, missing 'Tab-':", repr(command))
          return

      # keep your original tabNum (bytes with CRLF)
      tabNum = ("Tab Number " + str(command[tabIndex+4:]) + "\r\n").encode('utf-8')

      def _norm(x):
          # bytes -> str; strip CR/LF and outer spaces; lower for case-insensitive match
          if isinstance(x, bytes):
              try:
                  x = x.decode("utf-8", "replace")
              except Exception:
                  x = str(x)
          return str(x).replace("\r", "").replace("\n", "").strip().lower()

      target_norm = _norm(tabNum)

      # Always read current items in the widget
      items = list(tab1.progView.get(0, tk.END))

      # 1) Try exact normalized match (works whether items are bytes or str)
      idx = next((i for i, it in enumerate(items) if _norm(it) == target_norm), None)

      if idx is None:
          # 2) Optional fallback: if your rows are like "Jump Tab-3", match by number
          #    Extract the number from tabNum and look for common forms
          m = re.search(r'\d+', _norm(tabNum))
          if m:
              n = m.group(0)
              forms = {f"tab number {n}", f"tab-{n}", f"tab {n}", f"tab: {n}", f"jump tab-{n}"}
              idx = next((i for i, it in enumerate(items) if _norm(it) in forms), None)

      if idx is None:
          print(f"[Jump T] Not found: {repr(tabNum)}")
      else:
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(idx)
          tab1.progView.see(idx)







  ##Set Output 5v IO Board##
  if (cmdTypeLong == "Set 5v Outp"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
    RUN['ser2'].write(command.encode())
    RUN['ser2'].flushInput()
    time.sleep(.1)
    RUN['ser2'].read()

  ##Set Modbus Coil##
  if (cmdTypeLong == "Set MBcoil "):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:])
    command = "SC"+"A"+slaveID+"B"+inputNum+"C"+valNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    response = RUN['ser'].readline().decode("utf-8").strip()
    if (response == "-1"):
      ErrorHandler("Modbus Error")

  ##Set Modbus Register##
  if (cmdTypeLong == "Set MBoutpu"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:])
    command = "SO"+"A"+slaveID+"B"+inputNum+"C"+valNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    response = RUN['ser'].readline().decode("utf-8").strip()
    if (response == "-1"):
      ErrorHandler("Modbus Error")       
 


  ##Wait Time Command##
  if (RUN['cmdType'] == "Wait T"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Wait time not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    timeIndex = command.find("Wait Time = ")
    timeSeconds = str(command[timeIndex+12:])
    command = "WTS"+timeSeconds+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    RUN['ser'].read() 

  ##Set Register##  
  if (RUN['cmdType'] == "Regist"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
  if (RUN['cmdType'] == "Positi"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
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
  if (RUN['cmdType'] == "Calibr"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotAll()

  ##Calibrate J1##   
  if (RUN['cmdType'] == "Cal_J1"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotJ1()

  ##Calibrate J2##   
  if (RUN['cmdType'] == "Cal_J2"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotJ2() 

  ##Calibrate J1##   
  if (RUN['cmdType'] == "Cal_J3"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotJ3() 

  ##Calibrate J1##   
  if (RUN['cmdType'] == "Cal_J4"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotJ4() 

  ##Calibrate J5##   
  if (RUN['cmdType'] == "Cal_J5"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotJ5() 

  ##Calibrate J6##   
  if (RUN['cmdType'] == "Cal_J6"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotJ6() 

  ##Calibrate J7##   
  if (RUN['cmdType'] == "Cal_J7"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotJ7() 

  ##Calibrate J8##   
  if (RUN['cmdType'] == "Cal_J8"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotJ8()

   ##Calibrate J9##   
  if (RUN['cmdType'] == "Cal_J9"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Calibration not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    calRobotJ9()                   

  ##Set tool##  
  if (RUN['cmdType'] == "Tool S"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Set tool not supported in offline programming mode", style="Alarm.TLabel")
      return 
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel") 
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    RUN['xVal'] = command[xIndex+3:yIndex]
    RUN['yVal'] = command[yIndex+3:zIndex]
    RUN['zVal'] = command[zIndex+3:rzIndex]
    rzVal = command[rzIndex+4:ryIndex]
    ryVal = command[ryIndex+4:rxIndex]
    rxVal = command[rxIndex+4:]
    TFxEntryField.delete(0,'end')
    TFyEntryField.delete(0,'end')
    TFzEntryField.delete(0,'end')
    TFrzEntryField.delete(0,'end')
    TFryEntryField.delete(0,'end')
    TFrxEntryField.delete(0,'end')
    TFxEntryField.insert(0,str(RUN['xVal']))
    TFyEntryField.insert(0,str(RUN['yVal']))
    TFzEntryField.insert(0,str(RUN['zVal']))
    TFrzEntryField.insert(0,str(rzVal))
    TFryEntryField.insert(0,str(ryVal))
    TFrxEntryField.insert(0,str(rxVal))
    command = "TF"+"A"+RUN['xVal']+"B"+RUN['yVal']+"C"+RUN['zVal']+"D"+rzVal+"E"+ryVal+"F"+rxVal+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    RUN['ser'].read()
     
  
  ##Move J Command##  
  if (RUN['cmdType'] == "Move J"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
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
    RUN['xVal'] = command[xIndex+3:yIndex]
    RUN['yVal'] = command[yIndex+3:zIndex]
    RUN['zVal'] = command[zIndex+3:rzIndex]
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
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    if not RUN['vtk_running']:
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)







 ##Offs J Command##  
  if (RUN['cmdType'] == "OFF J "): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
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
    RUN['xVal'] = str(float(cx) + float(command[xIndex+3:yIndex]))
    RUN['yVal'] = str(float(cy) + float(command[yIndex+3:zIndex]))
    RUN['zVal'] = str(float(cz) + float(command[zIndex+3:rzIndex]))
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
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    if not RUN['vtk_running']:
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)  

  ##Move Vis Command##  
  if (RUN['cmdType'] == "Move V"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
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
    RUN['xVal'] = str(float(cx) + float(VisRetXrobEntryField.get()))
    RUN['yVal'] = str(float(cy) + float(VisRetYrobEntryField.get()))
    RUN['zVal'] = str(float(cz) + float(command[zIndex+3:rzIndex]))
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
    RUN['WC'] = command[WristConfIndex+3:]
    visRot = VisRetAngleEntryField.get()
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MV"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Vr"+visRot+"Lm"+LoopMode+"\n"
    if not RUN['vtk_running']:
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)       

  ##Move PR Command##  
  if (RUN['cmdType'] == "Move P"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
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
    RUN['xVal'] = str(float(cx))
    RUN['yVal'] = str(float(cy))
    RUN['zVal'] = str(float(cz))
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
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    if not RUN['vtk_running']:
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)   

  ##OFFS PR Command##  
  if (RUN['cmdType'] == "OFF PR"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
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
    RUN['xVal'] = str(float(eval("SP_"+SP+"_E1_EntryField").get()) + float(eval("SP_"+SP2+"_E1_EntryField").get()))
    RUN['yVal'] = str(float(eval("SP_"+SP+"_E2_EntryField").get()) + float(eval("SP_"+SP2+"_E2_EntryField").get()))
    RUN['zVal'] = str(float(eval("SP_"+SP+"_E3_EntryField").get()) + float(eval("SP_"+SP2+"_E3_EntryField").get()))
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
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    if not RUN['vtk_running']:
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)  

  ##Move L Command##  
  if (RUN['cmdType'] == "Move L"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
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
    RUN['xVal'] = command[xIndex+3:yIndex]
    RUN['yVal'] = command[yIndex+3:zIndex]
    RUN['zVal'] = command[zIndex+3:rzIndex]
    rzVal = command[rzIndex+4:ryIndex]
    if (np.sign(float(rzVal)) != np.sign(float(CAL['RzcurPos']))):
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
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    DisWrist = str(CAL['DisableWristRotVal'].get())
    command = "ML"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+RUN['WC']+"Lm"+LoopMode+"Q"+DisWrist+"\n"
    if not RUN['vtk_running']:
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"  
      mj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05) 




  ##Move R Command##  
  if (RUN['cmdType'] == "Move R"):
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1 
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
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "RJ"+"A"+J1Val+"B"+J2Val+"C"+J3Val+"D"+J4Val+"E"+J5Val+"F"+J6Val+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    if not RUN['vtk_running']:
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      commandVR = "RJ"+"A"+J1Val+"B"+J2Val+"C"+J3Val+"D"+J4Val+"E"+J5Val+"F"+J6Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
      rj_command(commandVR)
      wait_until_all_locks_free(min_hold_time=0.05)
      
  ##Move A Command##  
  if (RUN['cmdType'] == "Move A"):
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
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
      RUN['xVal'] = command[xIndex+3:yIndex]
      RUN['yVal'] = command[yIndex+3:zIndex]
      RUN['zVal'] = command[zIndex+3:rzIndex]
      rzVal = command[rzIndex+4:ryIndex]
      ryVal = command[ryIndex+4:rxIndex]
      rxVal = command[rxIndex+4:trIndex]
      trVal = command[trIndex+4:SpeedIndex]
      speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
      Speed = command[SpeedIndex+4:ACCspdIndex]
      ACCspd = command[ACCspdIndex+4:DECspdIndex]
      DECspd = command[DECspdIndex+4:ACCrampIndex]
      ACCramp = command[ACCrampIndex+4:WristConfIndex]
      RUN['WC'] = command[WristConfIndex+3:]
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
      RUN['WC'] = command[WristConfIndex+3:]
      TCX = 0
      TCY = 0 
      TCZ = 0
      TCRx = 0
      TCRy = 0
      TCRz = 0
      #move arc command
      if RUN['vtk_running']:
        almStatusLab.config(text="Arc move not yet programmed for virtual robot playback", style="Alarm.TLabel")
      LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
      command = "MA"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Ex"+Xend+"Ey"+Yend+"Ez"+Zend+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response) 

  ##Move C Command##  
  if (RUN['cmdType'] == "Move C"):
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
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
      RUN['xVal'] = command[xIndex+3:yIndex]
      RUN['yVal'] = command[yIndex+3:zIndex]
      RUN['zVal'] = command[zIndex+3:rzIndex]
      rzVal = command[rzIndex+4:ryIndex]
      ryVal = command[ryIndex+4:rxIndex]
      rxVal = command[rxIndex+4:trIndex]
      trVal = command[trIndex+4:SpeedIndex]
      speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
      Speed = command[SpeedIndex+4:ACCspdIndex]
      ACCspd = command[ACCspdIndex+4:DECspdIndex]
      DECspd = command[DECspdIndex+4:ACCrampIndex]
      ACCramp = command[ACCrampIndex+4:WristConfIndex]
      RUN['WC'] = command[WristConfIndex+3:]
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
      LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
      command = "MJ"+"X"+Xmid+"Y"+Ymid+"Z"+Zmid+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      #move circle command
      start = time.time()
      if RUN['vtk_running']:
        almStatusLab.config(text="Circle move not yet programmed for virtual robot playback", style="Alarm.TLabel")
      LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
      command = "MC"+"Cx"+RUN['xVal']+"Cy"+RUN['yVal']+"Cz"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"Bx"+Xmid+"By"+Ymid+"Bz"+Zmid+"Px"+Xend+"Py"+Yend+"Pz"+Zend+"Tr"+trVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
      end = time.time()
      #manEntryField.delete(0, 'end')
      #manEntryField.insert(0,end-start) 
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response) 

  ##Start Spline
  if (RUN['cmdType'] == "Start "):
    RUN['splineActive'] = "1"
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = "SL\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    RUN['ser'].read() 

  ##End Spline
  if (RUN['cmdType'] == "End Sp"):
    RUN['splineActive'] = "0"
    if(RUN['stopQueue'] == "1"):
      RUN['stopQueue'] = "0"
      stop()
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = "SS\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    response = str(RUN['ser'].readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response) 

  ##Camera On
  if(RUN['cmdType'] == "Cam On"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    start_vid()

  ##Camera Off
  if(RUN['cmdType'] == "Cam Of"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    stop_vid()  

  ##Vision Find
  if(RUN['cmdType'] == "Vis Fi"):
    #if (RUN['moveInProc'] == 1):
      #RUN['moveInProc'] = 2
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


  RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
  setStepMonitorsVR() 
  RUN['progRunning'] = False  
  RUN['rowinproc'] = 0
  





  
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
  RUN['_mainMode'] = 1
  def _show_mode_banner():
      try:
          _lbl("JOINT MODE" if RUN['_mainMode'] == 1 else "CARTESIAN MODE", style="Warn.TLabel")
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
  RUN['_grip_closed'] = False  # False = open; first press closes (SV0P0)

  def _nano_send(cmd):
      def worker():
          try:
              RUN['ser2'].write(cmd.encode())
              RUN['ser2'].flushInput()
              time.sleep(0.1)
              RUN['ser2'].read()
          except Exception:
              pass
      threading.Thread(target=worker, daemon=True).start()

  def _toggle_servo_gripper():
      # global RUN['_grip_closed']
      if not RUN['_grip_closed']:
          _nano_send("SV0P0\n")   # close
          RUN['_grip_closed'] = True
      else:
          _nano_send("SV0P50\n")  # open
          RUN['_grip_closed'] = False

  # ----- Pneumatic gripper toggle (START) over ser2 -----
  RUN['_pneu_open'] = False  # False = closed

  def _toggle_pneu_gripper():
      # global RUN['_pneu_open']
      if not RUN['_pneu_open']:
          _nano_send("OFX8\n")   # OPEN
          RUN['_pneu_open'] = True
      else:
          _nano_send("ONX8\n")   # CLOSE
          RUN['_pneu_open'] = False

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
  RUN['_current'] = None
  RUN['_pending_start'] = None
  RUN['_last_input_time'] = 0.0
  SWITCH_DELAY_MS = 60
  WATCHDOG_MS     = 200

  def _lj_code(j, direction):  # J1- = 10, J1+ = 11; J2- = 20, J2+ = 21; ...
      return j*10 + (1 if direction > 0 else 0)

  def _request_switch(new_active):
      # global RUN['_current'], RUN['_pending_start']
      old = RUN['_current']
      if old == new_active:
          return

      def do_start_if_pending():
          # global RUN['_current'], RUN['_pending_start']
          code = _pending_start; RUN['_pending_start'] = None
          if code is not None:
              RUN['_current'] = new_active
              _gui_start_joint(code)

      if old is not None:
          RUN['_current'] = None
          RUN['_pending_start'] = _lj_code(*new_active) if new_active else None
          _gui_stop()
          try: root.after(SWITCH_DELAY_MS, do_start_if_pending)
          except Exception: do_start_if_pending()
          return

      if new_active is not None:
          RUN['_current'] = new_active
          _gui_start_joint(_lj_code(*new_active))

  # ---------- Cartesian arbiter ----------
  RUN['_cart_current'] = None
  RUN['_cart_pending'] = None

  def _cart_code(axis, d):
      if axis == 'X':  return 10 if d < 0 else 11
      if axis == 'Y':  return 20 if d < 0 else 21
      if axis == 'Z':  return 30 if d < 0 else 31
      if axis == 'Rx': return 40 if d < 0 else 41
      if axis == 'Ry': return 50 if d < 0 else 51
      if axis == 'Rz': return 60 if d < 0 else 61
      return None

  def _request_switch_cart(new_active):
      # global RUN['_cart_current'], RUN['_cart_pending']
      old = RUN['_cart_current']
      if old == new_active:
          return

      def do_start_if_pending():
          # global RUN['_cart_current'], RUN['_cart_pending']
          item = _cart_pending; RUN['_cart_pending'] = None
          if item is not None:
              RUN['_cart_current'] = item
              axis, d = item
              code = _cart_code(axis, d)
              if code is not None:
                  _gui_start_cart(code)

      if old is not None:
          RUN['_cart_current'] = None
          RUN['_cart_pending'] = new_active
          _gui_stop()
          try: root.after(SWITCH_DELAY_MS, do_start_if_pending)
          except Exception: do_start_if_pending()
          return

      if new_active is not None:
          RUN['_cart_current'] = new_active
          axis, d = new_active
          code = _cart_code(axis, d)
          if code is not None:
              _gui_start_cart(code)

  # ---------- Tool (Tz) arbiter (for bumpers) ----------
  RUN['_tool_current'] = None
  RUN['_tool_pending'] = None

  def _tool_code(axis, d):
      if axis == 'Tz': return 30 if d < 0 else 31   # per your LiveToolJog mapping
      return None

  def _request_switch_tool(new_active):
      # global RUN['_tool_current'], RUN['_tool_pending']
      old = RUN['_tool_current']
      if old == new_active:
          return

      def do_start_if_pending():
          # global RUN['_tool_current'], RUN['_tool_pending']
          item = _tool_pending; RUN['_tool_pending'] = None
          if item is not None:
              RUN['_tool_current'] = item
              axis, d = item
              code = _tool_code(axis, d)
              if code is not None:
                  _gui_start_tool(code)

      if old is not None:
          RUN['_tool_current'] = None
          RUN['_tool_pending'] = new_active
          _gui_stop()
          try: root.after(SWITCH_DELAY_MS, do_start_if_pending)
          except Exception: do_start_if_pending()
          return

      if new_active is not None:
          RUN['_tool_current'] = new_active
          axis, d = new_active
          code = _tool_code(axis, d)
          if code is not None:
              _gui_start_tool(code)

  # ---------- Watchdog (covers all 3 arbiters) ----------
  def _watchdog_tick():
      # global RUN['_current'], RUN['_pending_start'], RUN['_cart_current'], RUN['_cart_pending'], RUN['_tool_current'], RUN['_tool_pending'], RUN['_last_input_time']
      try:
          now = time.monotonic()
          if (now - RUN['_last_input_time']) * 1000.0 > WATCHDOG_MS:
              if RUN['_current'] is not None or RUN['_cart_current'] is not None or RUN['_tool_current'] is not None:
                  RUN['_current'] = None; RUN['_pending_start'] = None
                  RUN['_cart_current'] = None; RUN['_cart_pending'] = None
                  RUN['_tool_current'] = None; RUN['_tool_pending'] = None
                  _gui_stop()
      finally:
          try: root.after(WATCHDOG_MS, _watchdog_tick)
          except Exception: pass

  # ---------- Axis selection (one axis per stick) ----------
  RUN['_smooth'] = {'LX': 0, 'LY': 0, 'RX': 0, 'RY': 0}
  def _lp(prev, new, alpha): return int(prev + alpha * (new - prev))

  def _stick_to_axis(raw_x, raw_y, dz_x, dz_y, alpha, start_thr, stop_thr, tag):
      """
      Returns (axis, dir) with axis in {'X','Y',None}, dir in {-1,0,+1}
      (One axis per stick; picks stronger if diagonal.)
      """
      if tag == 'L':
          RUN['_smooth']['LX'] = _lp(RUN['_smooth']['LX'], raw_x, alpha)
          RUN['_smooth']['LY'] = _lp(RUN['_smooth']['LY'], raw_y, alpha)
          nx = _norm_axis(RUN['_smooth']['LX'], dz_x); ny = _norm_axis(RUN['_smooth']['LY'], dz_y)
      else:
          RUN['_smooth']['RX'] = _lp(RUN['_smooth']['RX'], raw_x, alpha)
          RUN['_smooth']['RY'] = _lp(RUN['_smooth']['RY'], raw_y, alpha)
          nx = _norm_axis(RUN['_smooth']['RX'], dz_x); ny = _norm_axis(RUN['_smooth']['RY'], dz_y)

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
      # global RUN['_smooth']
      RUN['_smooth']['LX'] = int(RUN['_smooth']['LX'] + LPF_ALPHA_L * (raw_lx - RUN['_smooth']['LX']))
      RUN['_smooth']['LY'] = int(RUN['_smooth']['LY'] + LPF_ALPHA_L * (raw_ly - RUN['_smooth']['LY']))
      nx = _norm_axis(RUN['_smooth']['LX'], DZ_LX)
      ny = _norm_axis(RUN['_smooth']['LY'], DZ_LY)
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
      # global RUN['_mainMode'], RUN['_last_input_time']
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
              if RUN['_mainMode'] != 1:
                  _request_switch(None); _request_switch_cart(None); _request_switch_tool(None)
                  RUN['_mainMode'] = 1; _show_mode_banner()
          elif buttons & BTN_B:
              if RUN['_mainMode'] != 2:
                  _request_switch(None); _request_switch_cart(None); _request_switch_tool(None)
                  RUN['_mainMode'] = 2; _show_mode_banner()

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
              if RUN['_mainMode'] == 1:
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

          RUN['_last_input_time'] = time.monotonic()
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
      # global RUN['xboxUse']
      jogMode = 1
      if RUN['xboxUse'] == 0:
        RUN['xboxUse'] = 1
        mainMode = 1
        jogMode = 1
        grip = 0
        almStatusLab.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
        almStatusLab2.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
        #xbcStatusLab.config(text='Xbox ON', )
        ChgDis(2)
      else:
        RUN['xboxUse'] = 0
        almStatusLab.config(text='XBOX CONTROLLER OFF', style="Warn.TLabel")
        almStatusLab2.config(text='XBOX CONTROLLER OFF', style="Warn.TLabel")
        #xbcStatusLab.config(text='Xbox OFF', )
      while RUN['xboxUse'] == 1:
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
                RUN['ser2'].write(command.encode())
                RUN['ser2'].flushInput()
                time.sleep(.1)
                RUN['ser2'].read() 
              else:
                grip = 0
                outputNum = DO1onEntryField.get()
                command = "ONX"+outputNum+"\n"
                RUN['ser2'].write(command.encode())
                RUN['ser2'].flushInput()
                time.sleep(.1)
                RUN['ser2'].read()     
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
    #global progRunning
    RUN['ser'].write(cmd.encode())    
    RUN['ser'].flushInput()
    time.sleep(0.1)
    response = str(RUN['ser'].readline().strip(), 'utf-8')
    IncJogStatVal = int(RUN['IncJogStat'].get())
    if IncJogStatVal or RUN['progRunning']:
      if response[:1] == 'E':
        ErrorHandler(response)
      else:
        displayPosition(response) 		



def start_send_serial_thread(command):
    #global progRunning
    if serial_lock.locked():
        logger.warning("Serial command already in progress — ignoring.")
        return
    t = threading.Thread(target=run_send_serial_safe, args=(command,), daemon=True)
    t.start()

def run_send_serial_safe(command):
    #global progRunning
    with serial_lock:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        send_serial_command(command)
       

 
def J1jogNeg(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+str(float(CAL['J1AngCur'])-value)+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0])-value)+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)   
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
   

def J1jogPos(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+str(float(CAL['J1AngCur'])+value)+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0])+value)+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)  
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
     

def J2jogNeg(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+str(float(CAL['J2AngCur'])-value)+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1]-value)+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

     

def J2jogPos(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+str(float(CAL['J2AngCur'])+value)+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1]+value)+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J3jogNeg(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+str(float(CAL['J3AngCur'])-value)+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2]-value)+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J3jogPos(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+str(float(CAL['J3AngCur'])+value)+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2]+value)+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J4jogNeg(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+str(float(CAL['J4AngCur'])-value)+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3]-value)+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J4jogPos(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+str(float(CAL['J4AngCur'])+value)+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3]+value)+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command) 

def J5jogNeg(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+str(float(CAL['J5AngCur'])-value)+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4]-value)+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J5jogPos(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+str(float(CAL['J5AngCur'])+value)+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4]+value)+"F"+str(RUN['VR_angles'][5])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)   

def J6jogNeg(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+str(float(CAL['J6AngCur'])-value)+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5]-value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J6jogPos(value):
  # global RUN['xboxUse']
  # global RUN['VR_angles']
  #global offlineMode
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+str(float(CAL['J6AngCur'])+value)+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "RJ"+"A"+str(float(RUN['VR_angles'][0]))+"B"+str(RUN['VR_angles'][1])+"C"+str(RUN['VR_angles'][2])+"D"+str(RUN['VR_angles'][3])+"E"+str(RUN['VR_angles'][4])+"F"+str(RUN['VR_angles'][5]+value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  #send command to virtual robot
  rj_command(commandVR)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)




def J7jogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(float(CAL['J7PosCur'])-value)+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J7jogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(float(CAL['J7PosCur'])+value)+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 



def J8jogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(float(CAL['J8PosCur'])-value)+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)



def J8jogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(float(CAL['J8PosCur'])+value)+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)  


def J9jogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(float(CAL['J9PosCur'])-value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)



def J9jogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(float(CAL['J9PosCur'])+value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
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
  # global RUN['xboxUse']
  #global liveJog
  RUN['liveJog'] = True
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  #!! WC isn't defined prior to use here, at least sometimes
  RUN['WC'] = locals().get("RUN['WC']", "")
  ############
  command = "LJ"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  start_live_joint_jog_thread(command)
  if not RUN['offlineMode']:
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
  # global RUN['xboxUse']
  #global liveJog
  RUN['liveJog'] = True
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "LC"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  start_live_cartesian_jog_thread(command)
  if not RUN['offlineMode']:
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
  # global RUN['xboxUse']
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "LT"+"V"+str(value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  start_live_tool_jog_thread(command)
  if not RUN['offlineMode']:
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
  #global liveJog
  # global RUN['VR_angles']
  if RUN['liveJog']:
    RUN['liveJog'] = False
    time.sleep(.15) 
    if not RUN['offlineMode']:
      command = "S\n"
      IncJogStatVal = int(RUN['IncJogStat'].get())
      if (IncJogStatVal == 0):
        RUN['ser'].write(command.encode()) 
        RUN['ser'].flushInput()
        time.sleep(.05)
        response = str(RUN['ser'].readline().strip(),'utf-8')
        if (response[:1] == 'E'):
          ErrorHandler(response)    
        else:
          displayPosition(response)
          RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
          setStepMonitorsVR()   
    else:
      refresh_gui_from_joint_angles(RUN['VR_angles'])


def J7jogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(float(CAL['J7PosCur'])-value)+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J7jogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(float(CAL['J7PosCur'])+value)+"J8"+str(CAL['J8PosCur'])+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 


def J8jogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(float(CAL['J8PosCur'])-value)+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J8jogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(float(CAL['J8PosCur'])+value)+"J9"+str(CAL['J9PosCur'])+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response) 



def J9jogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(float(CAL['J9PosCur'])-value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J9jogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "RJ"+"A"+CAL['J1AngCur']+"B"+CAL['J2AngCur']+"C"+CAL['J3AngCur']+"D"+CAL['J4AngCur']+"E"+CAL['J5AngCur']+"F"+CAL['J6AngCur']+"J7"+str(CAL['J7PosCur'])+"J8"+str(CAL['J8PosCur'])+"J9"+str(float(CAL['J9PosCur'])+value)+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  if (response[:1] == 'E'):
    ErrorHandler(response)    
  else:
    displayPosition(response)     
    




def XjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = str(float(CAL['XcurPos']) - value)
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['XcurPos'] = CAL['XcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)



  

def YjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = str(float(CAL['YcurPos']) - value)
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['YcurPos'] = CAL['YcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)





def ZjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = str(float(CAL['ZcurPos']) - value)
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['ZcurPos'] = CAL['ZcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)  

def RxjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal =  str(float(CAL['RxcurPos']) - value)
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RxcurPos'] = CAL['RxcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)  

def RyjogNeg(value):
  # global RUN['xboxUse']
  # global XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = str(float(CAL['RycurPos']) - value)
  rxVal =  CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RycurPos'] = CAL['RycurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)    

def RzjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal =  str(float(CAL['RzcurPos']) - value)
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RzcurPos'] = CAL['RzcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)  

def XjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = str(float(CAL['XcurPos']) + value)
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['XcurPos'] = CAL['XcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)   

def YjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = str(float(CAL['YcurPos']) + value)
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['YcurPos'] = CAL['YcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)   


def ZjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = str(float(CAL['ZcurPos']) + value)
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['ZcurPos'] = CAL['ZcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)     

def RxjogPos(value):
  # global RUN['xboxUse']
  # global XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal =  str(float(CAL['RxcurPos']) + value)
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RxcurPos'] = CAL['RxcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)   

def RyjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = str(float(CAL['RycurPos']) + value)
  rxVal =  CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RycurPos'] = CAL['RycurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)   

def RzjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal =  str(float(CAL['RzcurPos']) + value)
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
    mj_command(commandVR)
  else:
    xyzuvw = robot.forward_kinematics(RUN['VR_angles'])
    xyzuvw = xyzuvw[:3] + [math.degrees(v) for v in xyzuvw[3:]]
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RzcurPos'] = CAL['RzcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    mj_command(commandVR)  

   
  
def TXjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTX1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)


def TYjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTY1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

def TZjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTZ1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)




def TRxjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTW1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TRyjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTP1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TRzjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTR1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TXjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTX0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TYjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTY0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

def TZjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTZ0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

def TRxjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTW0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TRyjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTP0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

def TRzjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
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
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTR0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)


  
  
##############################################################################################################################################################  
### TEACH DEFS ################################################################################################################################ TEACH DEFS ###
##############################################################################################################################################################  

def teachInsertBelSelected():
  # global WC
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
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']              
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
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']              
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
    newPos = movetype + " [*]"+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']          
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
    newPos = movetype + " [*]"+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']
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
        ("XcurPos", CAL['XcurPos']),
        ("YcurPos", CAL['YcurPos']),
        ("ZcurPos", CAL['ZcurPos']),
        ("RzcurPos", CAL['RzcurPos']),
        ("RycurPos", CAL['RycurPos']),
        ("RxcurPos", CAL['RxcurPos']),
        ("J7PosCur", CAL['J7PosCur']),
        ("J8PosCur", CAL['J8PosCur']),
        ("J9PosCur", CAL['J9PosCur']),
        ("speedPrefix", speedPrefix),
        ("Speed", Speed),
        ("ACCspd", ACCspd),
        ("DECspd", DECspd),
        ("ACCramp", ACCramp),
        ("WC", RUN['WC']),
    ]:
        if not isinstance(val, str):
            logger.warning(f"{name} is not a string — it is {type(val)}: {val}")
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']              
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
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" Rnd "+Rounding+" $ "+RUN['WC'] 
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
    newPos = movetype + " [*] J1 "+CAL['J1AngCur']+" J2 "+CAL['J2AngCur']+" J3 "+CAL['J3AngCur']+" J4 "+CAL['J4AngCur']+" J5 "+CAL['J5AngCur']+" J6 "+CAL['J6AngCur']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']            
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
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']             
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
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']             
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
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']              
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
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']                 
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
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']
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
    SPE6 = "Position Register "+PR+" Element 6 = "+CAL['RxcurPos']         
    tab1.progView.insert(selRow, bytes(SPE6 + '\n', 'utf-8')) 
    SPE5 = "Position Register "+PR+" Element 5 = "+CAL['RycurPos']            
    tab1.progView.insert(selRow, bytes(SPE5 + '\n', 'utf-8')) 
    SPE4 = "Position Register "+PR+" Element 4 = "+CAL['RzcurPos']           
    tab1.progView.insert(selRow, bytes(SPE4 + '\n', 'utf-8')) 	
    SPE3 = "Position Register "+PR+" Element 3 = "+CAL['ZcurPos']       
    tab1.progView.insert(selRow, bytes(SPE3 + '\n', 'utf-8')) 	
    SPE2 = "Position Register "+PR+" Element 2 = "+CAL['YcurPos']            
    tab1.progView.insert(selRow, bytes(SPE2 + '\n', 'utf-8')) 
    SPE1 = "Position Register "+PR+" Element 1 = "+CAL['XcurPos']         
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
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1) 
  response = RUN['ser'].readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)

def MBreadCoil():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BB"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1) 
  response = RUN['ser'].readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)

def MBreadInput():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BC"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1) 
  response = RUN['ser'].readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)

def MBreadInputReg():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BD"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1) 
  response = RUN['ser'].readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response) 

def MBwriteCoil():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BE"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1) 
  response = RUN['ser'].readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response) 

   
def MBwriteReg():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BF"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1) 
  response = RUN['ser'].readline().decode("utf-8").strip()
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)          

def QueryModbus():
  #command = "HD"+"\n"
  command = "MQ"+"\n"
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1) 
  response = RUN['ser'].readline().decode("utf-8").strip()
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,response)

def FaultReset():
  command = "FR"+"\n"
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1) 
  response = str((RUN['ser'].readline().strip(),'utf-8'))
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
  seconds = waitSecField.get()
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

  if(not localErrorFlag):        
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

  if(not localErrorFlag):        
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

  if(not localErrorFlag):        
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
    # global RUN['ser3']    
    port = "COM" + com3PortEntryField.get()     
    baud = 9600    
    RUN['ser3'] = serial.Serial(port,baud,timeout=5)
  except:
    #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
    #tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
    logger.error("UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
  RUN['ser3'].flushInput()
  numChar = int(com3charPortEntryField.get())
  response = str(RUN['ser3'].read(numChar).strip(),'utf-8')    
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
  if filename == "":
    return
  try:
    Prog = open(filename,"rb")
    time.sleep(.1)
    for item in Prog:
      tab1.progView.insert(END,item)
    tab1.progView.pack()
    scrollbar.config(command=tab1.progView.yview)
    save_calibration(CAL)
  except FileNotFoundError:
    logger.warning("File not found. Please check the file path and try again.")
  except Exception as e:
    logger.error(f"An error occurred: {e}")

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
  save_calibration(CAL) 



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
  
  match CE['Platform']['OS']:
    case 'Windows':
      os.startfile(file_path)
    case 'Linux':
      try:
        subprocess.run(["xdg-open", file_path], check=False)
      except FileNotFoundError:
          logger.error("xdg-open not found. Please install xdg-utils package.")
    case _:
      logger.error("Unsupported OS on File Open")

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
  save_calibration(CAL)      


def insertvisFind():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  template = RUN['selectedTemplate'].get()
  if (template == ""):
    template = "None_Selected.jpg"
  CAL['autoBGVal'] = int(RUN['autoBG'].get())  
  if (CAL['autoBGVal'] == 1):
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
  try:
    selRow = tab1.progView.curselection()[0]
    curRowEntryField.delete(0, 'end')
    curRowEntryField.insert(0,selRow)
  except Exception as e:
    logger.debug(f"No selection available: {e}")
 
def getSel():
  selRow = tab1.progView.curselection()[0]
  tab1.progView.see(selRow+2)
  data = list(map(int, tab1.progView.curselection()))
  command=tab1.progView.get(data[0]).decode()
  manEntryField.delete(0, 'end')
  manEntryField.insert(0, command)  
  
def Servo0on():
  save_calibration(CAL) 
  servoPos = servo0onEntryField.get()
  command = "SV0P"+servoPos+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read()


def Servo0off():
  save_calibration(CAL) 
  servoPos = servo0offEntryField.get()
  command = "SV0P"+servoPos+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read()


def Servo1on():
  save_calibration(CAL) 
  servoPos = servo1onEntryField.get()
  command = "SV1P"+servoPos+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 


def Servo1off():
  save_calibration(CAL) 
  servoPos = servo1offEntryField.get()
  command = "SV1P"+servoPos+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read()
 

def Servo2on():
  save_calibration(CAL) 
  servoPos = servo2onEntryField.get()
  command = "SV2P"+servoPos+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 


def Servo2off():
  save_calibration(CAL) 
  servoPos = servo2offEntryField.get()
  command = "SV2P"+servoPos+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read()

def Servo3on():
  save_calibration(CAL) 
  servoPos = servo3onEntryField.get()
  command = "SV3P"+servoPos+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 

def Servo3off():
  save_calibration(CAL) 
  servoPos = servo3offEntryField.get()
  command = "SV3P"+servoPos+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read()

def DO1on():
  outputNum = DO1onEntryField.get()
  command = "ONX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 


def DO1off():
  outputNum = DO1offEntryField.get()
  command = "OFX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 
 

def DO2on():
  outputNum = DO2onEntryField.get()
  command = "ONX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read()
 

def DO2off():
  outputNum = DO2offEntryField.get()
  command = "OFX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 


def DO3on():
  outputNum = DO3onEntryField.get()
  command = "ONX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 


def DO3off():
  outputNum = DO3offEntryField.get()
  command = "OFX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 
 

def DO4on():
  outputNum = DO4onEntryField.get()
  command = "ONX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read()
 

def DO4off():
  outputNum = DO4offEntryField.get()
  command = "OFX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 


def DO5on():
  outputNum = DO5onEntryField.get()
  command = "ONX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 


def DO5off():
  outputNum = DO5offEntryField.get()
  command = "OFX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 
 

def DO6on():
  outputNum = DO6onEntryField.get()
  command = "ONX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read()
 

def DO6off():
  outputNum = DO6offEntryField.get()
  command = "OFX"+outputNum+"\n"
  RUN['ser2'].write(command.encode())
  RUN['ser2'].flushInput()
  time.sleep(.1)
  RUN['ser2'].read() 

#!! Appears not to be used
'''
def TestString():
  message = testSendEntryField.get()
  command = "TM"+message+"\n"
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(0)
  echo = RUN['ser'].readline()
  testRecEntryField.delete(0, 'end')
  testRecEntryField.insert(0,echo)  
'''

#!! Appears not to be used
'''
def ClearTestString():
  testRecEntryField.delete(0, 'end')
'''

def CalcLinDist(X2,Y2,Z2):
  # global RUN['LineDist']
  X1 = CAL['XcurPos']
  Y1 = CAL['YcurPos']
  Z1 = CAL['ZcurPos']
  RUN['LineDist'] = (((X2-X1)**2)+((Y2-Y1)**2)+((Z2-Z1)**2))**.5
  return (RUN['LineDist'])

def CalcLinVect(X2,Y2,Z2):
  # global RUN['Xv']
  # global RUN['Yv']
  # global RUN['Zv']
  X1 = CAL['XcurPos']
  Y1 = CAL['YcurPos']
  Z1 = CAL['ZcurPos']
  RUN['Xv'] = X2-X1
  RUN['Yv'] = Y2-Y1
  RUN['Zv'] = Z2-Z1
  return (RUN['Xv'],RUN['Yv'],RUN['Zv'])  

''' not used
def CalcLinWayPt(CX,CY,CZ,curWayPt,):
  return
'''
 


	
	
##############################################################################################################################################################	
### CALIBRATION & SAVE DEFS ###################################################################################################### CALIBRATION & SAVE DEFS ###
##############################################################################################################################################################	

def calRobotAll():
  # global RUN['VR_angles']
  success = FALSE
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return 
  ##### STAGE 1 ########
  command = "LL"+"A"+str(CAL['J1CalStatVal'])+"B"+str(CAL['J2CalStatVal'])+"C"+str(CAL['J3CalStatVal'])+"D"+str(CAL['J4CalStatVal'])+"E"+str(CAL['J5CalStatVal'])+"F"+str(CAL['J6CalStatVal'])+"G"+str(CAL['J7CalStatVal'])+"H"+str(CAL['J8CalStatVal'])+"I"+str(CAL['J9CalStatVal'])+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "Auto Calibration Stage 1 Successful"
    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel")
    success = TRUE
  else:
    message = "Auto Calibration Stage 1 Failed - See Log" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)

  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb")) 
  ##### STAGE 2 ########
  if (success):
    CalStatVal2 = int(CAL['J1CalStatVal2'])+int(CAL['J2CalStatVal2'])+int(CAL['J3CalStatVal2'])+int(CAL['J4CalStatVal2'])+int(CAL['J5CalStatVal2'])+int(CAL['J6CalStatVal2'])
    if(CalStatVal2>0):
      command = "LL"+"A"+str(CAL['J1CalStatVal2'])+"B"+str(CAL['J2CalStatVal2'])+"C"+str(CAL['J3CalStatVal2'])+"D"+str(CAL['J4CalStatVal2'])+"E"+str(CAL['J5CalStatVal2'])+"F"+str(CAL['J6CalStatVal2'])+"G"+str(CAL['J7CalStatVal2'])+"H"+str(CAL['J8CalStatVal2'])+"I"+str(CAL['J9CalStatVal2'])+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n" 
      RUN['ser'].write(command.encode())
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].flushInput()
      response = str(RUN['ser'].readline().strip(),'utf-8')
      cmdRecEntryField.delete(0, 'end')
      cmdRecEntryField.insert(0,response)
      if (response[:1] == 'A'):
        displayPosition(response)  
        message = "Auto Calibration Stage 2 Successful"
        RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
        setStepMonitorsVR()
        almStatusLab.config(text=message, style="OK.TLabel")
        almStatusLab2.config(text=message, style="OK.TLabel") 
      else:
        message = "Auto Calibration Stage 2 Failed - See Log" 
        almStatusLab.config(text=message, style="Alarm.TLabel")
        almStatusLab2.config(text=message, style="Alarm.TLabel")
        ErrorHandler(response)
      if "success" in message.strip().lower():
        logger.info(message)
      else:
        logger.error(message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb")) 


def calRobotJ1():
  # global RUN['VR_angles']
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = "LLA1B0C0D0E0F0G0H0I0"+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J1 Calibrated Successfully"
    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J1 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ2():
  # global RUN['VR_angles']
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = "LLA0B1C0D0E0F0G0H0I0"+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J2 Calibrated Successfully"
    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J2 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ3():
  # global RUN['VR_angles']
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = "LLA0B0C1D0E0F0G0H0I0"+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J3 Calibrated Successfully"
    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J3 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ4():
  # global RUN['VR_angles']
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = "LLA0B0C0D1E0F0G0H0I0"+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n" 
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J4 Calibrated Successfully"
    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel" ) 
  else:
    message = "J4 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ5():
  # global RUN['VR_angles']
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = "LLA0B0C0D0E1F0G0H0I0"+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n"  
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J5 Calibrated Successfully"
    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J5 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ6():
  # global RUN['VR_angles']
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = "LLA0B0C0D0E0F1G0H0I0"+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n"  
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:1] == 'A'):
    displayPosition(response)  
    message = "J6 Calibrated Successfully"
    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
    almStatusLab.config(text=message, style="OK.TLabel")
    almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J6 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))   

def calRobotJ7():
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = "LLA0B0C0D0E0F0G1H0I0"+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n"  
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
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
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb")) 

def calRobotJ8():
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = "LLA0B0C0D0E0F0G0H1I0"+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n"  
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
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
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))    

def calRobotJ9():
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = "LLA0B0C0D0E0F0G0H0I1"+"J"+str(CAL['J1calOff'])+"K"+str(CAL['J2calOff'])+"L"+str(CAL['J3calOff'])+"M"+str(CAL['J4calOff'])+"N"+str(CAL['J5calOff'])+"O"+str(CAL['J6calOff'])+"P"+str(CAL['J7calOff'])+"Q"+str(CAL['J8calOff'])+"R"+str(CAL['J9calOff'])+"\n"  
  RUN['ser'].write(command.encode())
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].flushInput()
  response = str(RUN['ser'].readline().strip(),'utf-8')
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
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))             
	



def correctPos():
  command = "CP\n"
  RUN['ser'].write(command.encode())    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  displayPosition(response) 

def requestPos():
  command = "RP\n"
  RUN['ser'].write(command.encode())    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
  displayPosition(response) 

def updateParams():
  CAL['TFx']  = TFxEntryField.get()
  CAL['TFy']  = TFyEntryField.get()
  CAL['TFz']  = TFzEntryField.get()
  CAL['TFrz'] = TFrzEntryField.get()
  CAL['TFry'] = TFryEntryField.get()
  CAL['TFrx'] = TFrxEntryField.get()
  CAL['J1MotDir'] = J1MotDirEntryField.get()
  CAL['J2MotDir'] = J2MotDirEntryField.get()
  CAL['J3MotDir'] = J3MotDirEntryField.get()
  CAL['J4MotDir'] = J4MotDirEntryField.get()
  CAL['J5MotDir'] = J5MotDirEntryField.get()
  CAL['J6MotDir'] = J6MotDirEntryField.get()
  CAL['J7MotDir'] = J7MotDirEntryField.get()
  CAL['J8MotDir'] = J8MotDirEntryField.get()
  CAL['J9MotDir'] = J9MotDirEntryField.get()
  CAL['J1CalDir'] = J1CalDirEntryField.get()
  CAL['J2CalDir'] = J2CalDirEntryField.get()
  CAL['J3CalDir'] = J3CalDirEntryField.get()
  CAL['J4CalDir'] = J4CalDirEntryField.get()
  CAL['J5CalDir'] = J5CalDirEntryField.get()
  CAL['J6CalDir'] = J6CalDirEntryField.get()
  CAL['J7CalDir'] = J7CalDirEntryField.get()
  CAL['J8CalDir'] = J8CalDirEntryField.get()
  CAL['J9CalDir'] = J9CalDirEntryField.get()
  CAL['J1PosLim'] = J1PosLimEntryField.get()
  CAL['J1NegLim'] = J1NegLimEntryField.get()
  CAL['J2PosLim'] = J2PosLimEntryField.get()
  CAL['J2NegLim'] = J2NegLimEntryField.get()
  CAL['J3PosLim'] = J3PosLimEntryField.get()
  CAL['J3NegLim'] = J3NegLimEntryField.get()
  CAL['J4PosLim'] = J4PosLimEntryField.get()
  CAL['J4NegLim'] = J4NegLimEntryField.get()
  CAL['J5PosLim'] = J5PosLimEntryField.get()
  CAL['J5NegLim'] = J5NegLimEntryField.get()
  CAL['J6PosLim'] = J6PosLimEntryField.get()
  CAL['J6NegLim'] = J6NegLimEntryField.get()
  CAL['J1StepDeg'] = J1StepDegEntryField.get()
  CAL['J2StepDeg'] = J2StepDegEntryField.get()
  CAL['J3StepDeg'] = J3StepDegEntryField.get()
  CAL['J4StepDeg'] = J4StepDegEntryField.get()
  CAL['J5StepDeg'] = J5StepDegEntryField.get()
  CAL['J6StepDeg'] = J6StepDegEntryField.get()
  J1EncMult = str(float(J1EncCPREntryField.get())/float(J1DriveMSEntryField.get()))
  J2EncMult = str(float(J2EncCPREntryField.get())/float(J2DriveMSEntryField.get()))
  J3EncMult = str(float(J3EncCPREntryField.get())/float(J3DriveMSEntryField.get()))
  J4EncMult = str(float(J4EncCPREntryField.get())/float(J4DriveMSEntryField.get()))
  J5EncMult = str(float(J5EncCPREntryField.get())/float(J5DriveMSEntryField.get()))
  J6EncMult = str(float(J6EncCPREntryField.get())/float(J6DriveMSEntryField.get()))
  CAL['J1ΘDHpar'] = J1ΘEntryField.get()
  CAL['J2ΘDHpar'] = J2ΘEntryField.get()
  CAL['J3ΘDHpar'] = J3ΘEntryField.get()
  CAL['J4ΘDHpar'] = J4ΘEntryField.get()
  CAL['J5ΘDHpar'] = J5ΘEntryField.get()
  CAL['J6ΘDHpar'] = J6ΘEntryField.get()
  CAL['J1αDHpar'] = J1αEntryField.get()
  CAL['J2αDHpar'] = J2αEntryField.get()
  CAL['J3αDHpar'] = J3αEntryField.get()
  CAL['J4αDHpar'] = J4αEntryField.get()
  CAL['J5αDHpar'] = J5αEntryField.get()
  CAL['J6αDHpar'] = J6αEntryField.get()
  CAL['J1dDHpar'] = J1dEntryField.get()
  CAL['J2dDHpar'] = J2dEntryField.get()
  CAL['J3dDHpar'] = J3dEntryField.get()
  CAL['J4dDHpar'] = J4dEntryField.get()
  CAL['J5dDHpar'] = J5dEntryField.get()
  CAL['J6dDHpar'] = J6dEntryField.get()
  CAL['J1aDHpar'] = J1aEntryField.get()
  CAL['J2aDHpar'] = J2aEntryField.get()
  CAL['J3aDHpar'] = J3aEntryField.get()
  CAL['J4aDHpar'] = J4aEntryField.get()
  CAL['J5aDHpar'] = J5aEntryField.get()
  CAL['J6aDHpar'] = J6aEntryField.get()

  update_CPP_kin_from_entries()

  J1negLimLab.config(text="-"+CAL['J1NegLim'], style="Jointlim.TLabel")
  J1posLimLab.config(text=CAL['J1PosLim'], style="Jointlim.TLabel")
  J1jogslide.config(from_=float("-"+CAL['J1NegLim']), to=float(CAL['J1PosLim']),  length=180, orient=HORIZONTAL,  command=J1sliderUpdate)
  J2negLimLab.config(text="-"+CAL['J2NegLim'], style="Jointlim.TLabel")
  J2posLimLab.config(text=CAL['J2PosLim'], style="Jointlim.TLabel")
  J2jogslide.config(from_=float("-"+CAL['J2NegLim']), to=float(CAL['J2PosLim']),  length=180, orient=HORIZONTAL,  command=J2sliderUpdate)
  J3negLimLab.config(text="-"+CAL['J3NegLim'], style="Jointlim.TLabel")
  J3posLimLab.config(text=CAL['J3PosLim'], style="Jointlim.TLabel")
  J3jogslide.config(from_=float("-"+CAL['J3NegLim']), to=float(CAL['J3PosLim']),  length=180, orient=HORIZONTAL,  command=J3sliderUpdate)
  J4negLimLab.config(text="-"+CAL['J4NegLim'], style="Jointlim.TLabel")
  J4posLimLab.config(text=CAL['J4PosLim'], style="Jointlim.TLabel")
  J4jogslide.config(from_=float("-"+CAL['J4NegLim']), to=float(CAL['J4PosLim']),  length=180, orient=HORIZONTAL,  command=J4sliderUpdate)
  J5negLimLab.config(text="-"+CAL['J5NegLim'], style="Jointlim.TLabel")
  J5posLimLab.config(text=CAL['J5PosLim'], style="Jointlim.TLabel")
  J5jogslide.config(from_=float("-"+CAL['J5NegLim']), to=float(CAL['J5PosLim']),  length=180, orient=HORIZONTAL,  command=J5sliderUpdate)
  J6negLimLab.config(text="-"+CAL['J6NegLim'], style="Jointlim.TLabel")
  J6posLimLab.config(text=CAL['J6PosLim'], style="Jointlim.TLabel")
  J6jogslide.config(from_=float("-"+CAL['J6NegLim']), to=float(CAL['J6PosLim']),  length=180, orient=HORIZONTAL,  command=J6sliderUpdate)

  command = "UP"+"A"+CAL['TFx']+"B"+CAL['TFy']+"C"+CAL['TFz']+"D"+CAL['TFrz']+"E"+CAL['TFry']+"F"+CAL['TFrx']+\
  "G"+CAL['J1MotDir']+"H"+CAL['J2MotDir']+"I"+CAL['J3MotDir']+"J"+CAL['J4MotDir']+"K"+CAL['J5MotDir']+"L"+CAL['J6MotDir']+"M"+CAL['J7MotDir']+"N"+CAL['J8MotDir']+"O"+CAL['J9MotDir']+\
  "P"+CAL['J1CalDir']+"Q"+CAL['J2CalDir']+"R"+CAL['J3CalDir']+"S"+CAL['J4CalDir']+"T"+CAL['J5CalDir']+"U"+CAL['J6CalDir']+"V"+CAL['J7CalDir']+"W"+CAL['J8CalDir']+"X"+CAL['J9CalDir']+\
  "Y"+CAL['J1PosLim']+"Z"+CAL['J1NegLim']+"a"+CAL['J2PosLim']+"b"+CAL['J2NegLim']+"c"+CAL['J3PosLim']+"d"+CAL['J3NegLim']+"e"+CAL['J4PosLim']+"f"+CAL['J4NegLim']+"g"+CAL['J5PosLim']+"h"+CAL['J5NegLim']+"i"+CAL['J6PosLim']+"j"+CAL['J6NegLim']+\
  "k"+CAL['J1StepDeg']+"l"+CAL['J2StepDeg']+"m"+CAL['J3StepDeg']+"n"+CAL['J4StepDeg']+"o"+CAL['J5StepDeg']+"p"+CAL['J6StepDeg']+\
  "q"+J1EncMult+"r"+J2EncMult+"s"+J3EncMult+"t"+J4EncMult+"u"+J5EncMult+"v"+J6EncMult+\
  "w"+CAL['J1ΘDHpar']+"x"+CAL['J2ΘDHpar']+"y"+CAL['J3ΘDHpar']+"z"+CAL['J4ΘDHpar']+"!"+CAL['J5ΘDHpar']+"@"+CAL['J6ΘDHpar']+\
  "#"+CAL['J1αDHpar']+"$"+CAL['J2αDHpar']+"%"+CAL['J3αDHpar']+"^"+CAL['J4αDHpar']+"&"+CAL['J5αDHpar']+"*"+CAL['J6αDHpar']+\
  "("+CAL['J1dDHpar']+")"+CAL['J2dDHpar']+"+"+CAL['J3dDHpar']+"="+CAL['J4dDHpar']+","+CAL['J5dDHpar']+"_"+CAL['J6dDHpar']+\
  "<"+CAL['J1aDHpar']+">"+CAL['J2aDHpar']+"?"+CAL['J3aDHpar']+"{"+CAL['J4aDHpar']+"}"+CAL['J5aDHpar']+"~"+CAL['J6aDHpar']+\
  "\n"
  try:
    RUN['ser'].write(command.encode())
    RUN['ser'].flush()
    time.sleep(.1)    
    RUN['ser'].flushInput()
    time.sleep(.1)
    response = RUN['ser'].read_all()
  except Exception as e:
    if RUN['ser'] in locals():
      logger.error("Serial error: "+str(e))
    else:
      logger.error("Serial port not open")

def calExtAxis():
  J7NegLim = 0
  J8NegLim = 0
  J9NegLim = 0

  CAL['J7PosLim'] = float(axis7lengthEntryField.get())
  J8PosLim = float(axis8lengthEntryField.get())
  J9PosLim = float(axis9lengthEntryField.get())

  J7negLimLab.config(text=str(-J7NegLim), style="Jointlim.TLabel")
  J8negLimLab.config(text=str(-J8NegLim), style="Jointlim.TLabel")
  J9negLimLab.config(text=str(-J9NegLim), style="Jointlim.TLabel")

  J7posLimLab.config(text=str(CAL['J7PosLim']), style="Jointlim.TLabel")
  J8posLimLab.config(text=str(J8PosLim), style="Jointlim.TLabel")
  J9posLimLab.config(text=str(J9PosLim), style="Jointlim.TLabel")

  J7jogslide.config(from_=-J7NegLim, to=CAL['J7PosLim'],  length=125, orient=HORIZONTAL,  command=J7sliderUpdate)
  J8jogslide.config(from_=-J8NegLim, to=J8PosLim,  length=125, orient=HORIZONTAL,  command=J8sliderUpdate)
  J9jogslide.config(from_=-J9NegLim, to=J9PosLim,  length=125, orient=HORIZONTAL,  command=J9sliderUpdate)

  command = "CE"+"A"+str(CAL['J7PosLim'])+"B"+str(CAL['J7rotation'])+"C"+str(CAL['J7steps'])+"D"+str(J8PosLim)+"E"+str(CAL['J8rotation'])+"F"+str(CAL['J8steps'])+"G"+str(J9PosLim)+"H"+str(CAL['J9rotation'])+"I"+str(CAL['J9steps'])+"\n"
  RUN['ser'].write(command.encode())    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = RUN['ser'].read()

def zeroAxis7():
  command = "Z7"+"\n"
  RUN['ser'].write(command.encode())    
  RUN['ser'].flushInput()
  time.sleep(.1)
  almStatusLab.config(text="J7 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J7 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J7 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  
  response = str(RUN['ser'].readline().strip(),'utf-8')
  displayPosition(response) 

def zeroAxis8():
  command = "Z8"+"\n"
  RUN['ser'].write(command.encode())    
  RUN['ser'].flushInput()
  time.sleep(.1)
  almStatusLab.config(text="J8 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J8 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J8 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  
  response = str(RUN['ser'].readline().strip(),'utf-8')
  displayPosition(response) 

def zeroAxis9():
  command = "Z9"+"\n"
  RUN['ser'].write(command.encode())    
  RUN['ser'].flushInput()
  time.sleep(.1)
  almStatusLab.config(text="J9 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J9 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J9 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  
  response = str(RUN['ser'].readline().strip(),'utf-8')
  displayPosition(response)   


def sendPos():
  command = "SP"+"A"+str(CAL['J1AngCur'])+"B"+str(CAL['J2AngCur'])+"C"+str(CAL['J3AngCur'])+"D"+str(CAL['J4AngCur'])+"E"+str(CAL['J5AngCur'])+"F"+str(CAL['J6AngCur'])+"G"+str(CAL['J7PosCur'])+"H"+str(CAL['J8PosCur'])+"I"+str(CAL['J9PosCur'])+"\n"
  RUN['ser'].write(command.encode())    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = RUN['ser'].read()

def CalZeroPos():
  # global RUN['VR_angles']
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  command = "SPA0B0C0D0E90F0\n"
  RUN['ser'].write(command.encode())    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = RUN['ser'].read()
  requestPos()
  almStatusLab.config(text="Calibration Forced to Home", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Home", style="Warn.TLabel")
  message = "Calibration Forced to Home - this is for commissioning and testing - be careful!"
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))
  RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
  setStepMonitorsVR()  

def CalRestPos():
  # global RUN['VR_angles']
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  command = "SPA0B0C-89D0E0F0\n"
  RUN['ser'].write(command.encode())    
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = RUN['ser'].read()
  requestPos()
  almStatusLab.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")
  message = "Calibration Forced to Vertical - this is for commissioning and testing - be careful!"
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb")) 
  RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
  setStepMonitorsVR() 




def displayPosition(response):
  # global WC, RUN['VR_angles'] 

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
  CAL['J1AngCur'] = response[J1AngIndex+1:J2AngIndex].strip();
  CAL['J2AngCur'] = response[J2AngIndex+1:J3AngIndex].strip();
  CAL['J3AngCur'] = response[J3AngIndex+1:J4AngIndex].strip();
  CAL['J4AngCur'] = response[J4AngIndex+1:J5AngIndex].strip();
  CAL['J5AngCur'] = response[J5AngIndex+1:J6AngIndex].strip();
  CAL['J6AngCur'] = response[J6AngIndex+1:XposIndex].strip();
  
  if CAL['J5AngCur'].strip() != '' and float(CAL['J5AngCur']) > 0:
    RUN['WC'] = "F"
  else:
    RUN['WC'] = "N"
  CAL['XcurPos'] = response[XposIndex+1:YposIndex].strip();
  CAL['YcurPos'] = response[YposIndex+1:ZposIndex].strip();
  CAL['ZcurPos'] = response[ZposIndex+1:RzposIndex].strip();
  CAL['RzcurPos'] = response[RzposIndex+1:RyposIndex].strip();
  CAL['RycurPos'] = response[RyposIndex+1:RxposIndex].strip();
  CAL['RxcurPos'] = response[RxposIndex+1:SpeedVioIndex].strip();
  SpeedVioation = response[SpeedVioIndex+1:DebugIndex].strip();
  Debug = response[DebugIndex+1:FlagIndex].strip();
  Flag = response[FlagIndex+1:J7PosIndex].strip();
  #J7PosCur = float(response[J7PosIndex+1:J8PosIndex].strip());
  #J8PosCur = float(response[J8PosIndex+1:J9PosIndex].strip());
  #J9PosCur = float(response[J9PosIndex+1:].strip());
  CAL['J7PosCur'] = response[J7PosIndex+1:J8PosIndex].strip();
  CAL['J8PosCur'] = response[J8PosIndex+1:J9PosIndex].strip();
  CAL['J9PosCur'] = response[J9PosIndex+1:].strip();
  
  J1curAngEntryField.delete(0, 'end')
  J1curAngEntryField.insert(0,CAL['J1AngCur'])
  J2curAngEntryField.delete(0, 'end')
  J2curAngEntryField.insert(0,CAL['J2AngCur'])
  J3curAngEntryField.delete(0, 'end')
  J3curAngEntryField.insert(0,CAL['J3AngCur'])
  J4curAngEntryField.delete(0, 'end')
  J4curAngEntryField.insert(0,CAL['J4AngCur'])
  J5curAngEntryField.delete(0, 'end')
  J5curAngEntryField.insert(0,CAL['J5AngCur'])
  J6curAngEntryField.delete(0, 'end')
  J6curAngEntryField.insert(0,CAL['J6AngCur'])
  XcurEntryField.delete(0, 'end')
  XcurEntryField.insert(0,CAL['XcurPos'])
  YcurEntryField.delete(0, 'end')
  YcurEntryField.insert(0,CAL['YcurPos'])
  ZcurEntryField.delete(0, 'end')
  ZcurEntryField.insert(0,CAL['ZcurPos'])
  RzcurEntryField.delete(0, 'end')
  RzcurEntryField.insert(0,CAL['RzcurPos'])
  RycurEntryField.delete(0, 'end')
  RycurEntryField.insert(0,CAL['RycurPos'])
  RxcurEntryField.delete(0, 'end')
  RxcurEntryField.insert(0,CAL['RxcurPos'])
  J7curAngEntryField.delete(0, 'end')
  J7curAngEntryField.insert(0,CAL['J7PosCur'])
  J8curAngEntryField.delete(0, 'end')
  J8curAngEntryField.insert(0,CAL['J8PosCur'])
  J9curAngEntryField.delete(0, 'end')
  J9curAngEntryField.insert(0,CAL['J9PosCur'])
  J1jogslide.set(CAL['J1AngCur'])
  J2jogslide.set(CAL['J2AngCur'])
  J3jogslide.set(CAL['J3AngCur'])
  J4jogslide.set(CAL['J4AngCur'])
  J5jogslide.set(CAL['J5AngCur'])
  J6jogslide.set(CAL['J6AngCur'])
  J7jogslide.set(CAL['J7PosCur'])
  J8jogslide.set(CAL['J8PosCur'])
  J9jogslide.set(CAL['J9PosCur'])
  manEntryField.delete(0, 'end')
  manEntryField.insert(0,Debug)

  save_calibration(CAL)
  if (Flag != ""):
      ErrorHandler(Flag) 
  if (SpeedVioation=='1'):
      #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
      message = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"
      #tab8.ElogView.insert(END, Curtime+" - "+message)
      logger.warning(message)
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
  
def save_custom_calibration():
  sync_fields_to_calibration()
  save_calibration(calibration_file='custom.json', calibration_data=CAL)

def load_custom_calibration():
  loaded_calibration = load_calibration(calibration_file='custom.json')
  logger.debug(f"Value of J1DriveMS collected from file is: {loaded_calibration['J1DriveMS']}")
  apply_calibration(loaded_calibration, CAL)

  sync_calibration_to_fields()

def sync_calibration_to_fields():
  '''Update Kinematics fields from current CAL'''
  ClearKinTabFields()
  J1MotDirEntryField.insert(0, str(CAL['J1MotDir']))
  J2MotDirEntryField.insert(0, str(CAL['J2MotDir']))
  J3MotDirEntryField.insert(0, str(CAL['J3MotDir']))
  J4MotDirEntryField.insert(0, str(CAL['J4MotDir']))
  J5MotDirEntryField.insert(0, str(CAL['J5MotDir']))
  J6MotDirEntryField.insert(0, str(CAL['J6MotDir']))
  J7MotDirEntryField.insert(0, str(CAL['J7MotDir']))
  J8MotDirEntryField.insert(0, str(CAL['J8MotDir']))
  J9MotDirEntryField.insert(0, str(CAL['J9MotDir']))
  J1CalDirEntryField.insert(0, str(CAL['J1CalDir']))
  J2CalDirEntryField.insert(0, str(CAL['J2CalDir']))
  J3CalDirEntryField.insert(0, str(CAL['J3CalDir']))
  J4CalDirEntryField.insert(0, str(CAL['J4CalDir']))
  J5CalDirEntryField.insert(0, str(CAL['J5CalDir']))
  J6CalDirEntryField.insert(0, str(CAL['J6CalDir']))
  J7CalDirEntryField.insert(0, str(CAL['J7CalDir']))
  J8CalDirEntryField.insert(0, str(CAL['J8CalDir']))
  J9CalDirEntryField.insert(0, str(CAL['J9CalDir']))
  J1PosLimEntryField.insert(0, str(CAL['J1PosLim']))
  J1NegLimEntryField.insert(0, str(CAL['J1NegLim']))
  J2PosLimEntryField.insert(0, str(CAL['J2PosLim']))
  J2NegLimEntryField.insert(0, str(CAL['J2NegLim']))
  J3PosLimEntryField.insert(0, str(CAL['J3PosLim']))
  J3NegLimEntryField.insert(0, str(CAL['J3NegLim']))
  J4PosLimEntryField.insert(0, str(CAL['J4PosLim']))
  J4NegLimEntryField.insert(0, str(CAL['J4NegLim']))
  J5PosLimEntryField.insert(0, str(CAL['J5PosLim']))
  J5NegLimEntryField.insert(0, str(CAL['J5NegLim']))
  J6PosLimEntryField.insert(0, str(CAL['J6PosLim']))
  J6NegLimEntryField.insert(0, str(CAL['J6NegLim']))
  J1StepDegEntryField.insert(0, str(CAL['J1StepDeg']))
  J2StepDegEntryField.insert(0, str(CAL['J2StepDeg']))
  J3StepDegEntryField.insert(0, str(CAL['J3StepDeg']))
  J4StepDegEntryField.insert(0, str(CAL['J4StepDeg']))
  J5StepDegEntryField.insert(0, str(CAL['J5StepDeg']))
  J6StepDegEntryField.insert(0, str(CAL['J6StepDeg']))
  logger.debug(f"Loaded value of J1DriveMS is: {CAL['J1DriveMS']}")
  J1DriveMSEntryField.insert(0, str(CAL['J1DriveMS']))
  J2DriveMSEntryField.insert(0, str(CAL['J2DriveMS']))
  J3DriveMSEntryField.insert(0, str(CAL['J3DriveMS']))
  J4DriveMSEntryField.insert(0, str(CAL['J4DriveMS']))
  J5DriveMSEntryField.insert(0, str(CAL['J5DriveMS']))
  J6DriveMSEntryField.insert(0, str(CAL['J6DriveMS']))
  J1EncCPREntryField.insert(0, str(CAL['J1EncCPR']))
  J2EncCPREntryField.insert(0, str(CAL['J2EncCPR']))
  J3EncCPREntryField.insert(0, str(CAL['J3EncCPR']))
  J4EncCPREntryField.insert(0, str(CAL['J4EncCPR']))
  J5EncCPREntryField.insert(0, str(CAL['J5EncCPR']))
  J6EncCPREntryField.insert(0, str(CAL['J6EncCPR']))
  J1ΘEntryField.insert(0,str(CAL['J1ΘDHpar']))
  J2ΘEntryField.insert(0,str(CAL['J2ΘDHpar']))
  J3ΘEntryField.insert(0,str(CAL['J3ΘDHpar']))
  J4ΘEntryField.insert(0,str(CAL['J4ΘDHpar']))
  J5ΘEntryField.insert(0,str(CAL['J5ΘDHpar']))
  J6ΘEntryField.insert(0,str(CAL['J6ΘDHpar']))
  J1αEntryField.insert(0,str(CAL['J1αDHpar']))
  J2αEntryField.insert(0,str(CAL['J2αDHpar']))
  J3αEntryField.insert(0,str(CAL['J3αDHpar']))
  J4αEntryField.insert(0,str(CAL['J4αDHpar']))
  J5αEntryField.insert(0,str(CAL['J5αDHpar']))
  J6αEntryField.insert(0,str(CAL['J6αDHpar']))
  J1dEntryField.insert(0,str(CAL['J1dDHpar']))
  J2dEntryField.insert(0,str(CAL['J2dDHpar']))
  J3dEntryField.insert(0,str(CAL['J3dDHpar']))
  J4dEntryField.insert(0,str(CAL['J4dDHpar']))
  J5dEntryField.insert(0,str(CAL['J5dDHpar']))
  J6dEntryField.insert(0,str(CAL['J6dDHpar']))
  J1aEntryField.insert(0,str(CAL['J1aDHpar']))
  J2aEntryField.insert(0,str(CAL['J2aDHpar']))
  J3aEntryField.insert(0,str(CAL['J3aDHpar']))
  J4aEntryField.insert(0,str(CAL['J4aDHpar']))
  J5aEntryField.insert(0,str(CAL['J5aDHpar']))
  J6aEntryField.insert(0,str(CAL['J6aDHpar']))


  # Add Encoder control checkboxes
  # Add auto-calibration settings

  updateParams()

def sync_fields_to_calibration():
  ''' synchronize the running CAL from field values '''
  CAL['comPort'] = com1SelectedValue.get()
  CAL['com2Port'] = com2SelectedValue.get()
  CAL['J7PosCur'] = J7curAngEntryField.get()
  CAL['J8PosCur'] = J8curAngEntryField.get()
  CAL['J9PosCur'] = J9curAngEntryField.get()
  CAL['VisProg'] = visoptions.get()
  CAL['J1calOff']    = float(J1calOffEntryField.get())
  CAL['J2calOff']    = float(J2calOffEntryField.get())
  CAL['J3calOff']    = float(J3calOffEntryField.get())
  CAL['J4calOff']    = float(J4calOffEntryField.get())
  CAL['J5calOff']    = float(J5calOffEntryField.get())
  CAL['J6calOff']    = float(J6calOffEntryField.get())
  CAL['J7calOff']    = float(J7calOffEntryField.get())
  CAL['J8calOff']    = float(J8calOffEntryField.get())
  CAL['J9calOff']    = float(J9calOffEntryField.get())
  CAL['J7PosLim']     = float(axis7lengthEntryField.get())
  CAL['J7rotation']   = float(axis7rotEntryField.get())
  CAL['J7steps']      = float(axis7stepsEntryField.get())
  CAL['J8length']     = float(axis8lengthEntryField.get())
  CAL['J8rotation']   = float(axis8rotEntryField.get())
  CAL['J8steps']      = float(axis8stepsEntryField.get())
  CAL['J9length']     = float(axis9lengthEntryField.get())
  CAL['J9rotation']   = float(axis9rotEntryField.get())
  CAL['J9steps']      = float(axis9stepsEntryField.get())

  # Checkboxes directly manipulate CAL variable and don't need to be sync'd

  logger.debug(f"Current value of J1DriveMS is: {CAL['J1DriveMS']}")
  logger.debug(f"Current value of J1DriveMSEntryField is: {J1DriveMSEntryField.get()}")
  CAL['J1DriveMS'] = int(J1DriveMSEntryField.get())
  logger.debug(f"Value of J1DriveMS updated to: {CAL['J1DriveMS']}")
  CAL['J2DriveMS'] = int(J2DriveMSEntryField.get())
  CAL['J3DriveMS'] = int(J3DriveMSEntryField.get())
  CAL['J4DriveMS'] = int(J4DriveMSEntryField.get())
  CAL['J5DriveMS'] = int(J5DriveMSEntryField.get())
  CAL['J6DriveMS'] = int(J6DriveMSEntryField.get())
  CAL['J1EncCPR'] = int(J1EncCPREntryField.get())
  CAL['J2EncCPR'] = int(J2EncCPREntryField.get())
  CAL['J3EncCPR'] = int(J3EncCPREntryField.get())
  CAL['J4EncCPR'] = int(J4EncCPREntryField.get())
  CAL['J5EncCPR'] = int(J5EncCPREntryField.get())
  CAL['J6EncCPR'] = int(J6EncCPREntryField.get())


def SaveAndApplyCalibration():
  try:
    sync_fields_to_calibration()
    updateParams()
    time.sleep(.1)
    calExtAxis()
  except:
    logger.error("no serial connection with Teensy board")  
  save_calibration(CAL)


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
  #global estopActive
  #global posOutreach
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  messages = [] #list to hold possible multiple error messages
  ##AXIS LIMIT ERROR
  if (response[1:2] == 'L'):
    if (response[2:3] == '1'):
      messages.append("J1 Axis Limit")
    if (response[3:4] == '1'):
      messages.append("J2 Axis Limit")
    if (response[4:5] == '1'):
      messages.append("J3 Axis Limit")
    if (response[5:6] == '1'):
      messages.append("J4 Axis Limit")
    if (response[6:7] == '1'):
      messages.append("J5 Axis Limit")
    if (response[7:8] == '1'):
      messages.append("J6 Axis Limit")
    if (response[8:9] == '1'):
      messages.append("J7 Axis Limit")
    if (response[9:10] == '1'):
      messages.append("J8 Axis Limit")
    if (response[10:11] == '1'):
      messages.append("J9 Axis Limit")

    # Actions to take on axis limit error
    cmdRecEntryField.delete(0, 'end')
    cmdRecEntryField.insert(0,response)            
    alarm_message = "Axis Limit Error - See Log"
    #Progstop()

  ##COLLISION ERROR   
  elif (response[1:2] == 'C'):
    if (response[2:3] == '1'):
      messages.append("J1 Collision or Motor Error")
    if (response[3:4] == '1'):
      messages.append("J2 Collision or Motor Error")
    if (response[4:5] == '1'):
      messages.append("J3 Collision or Motor Error")
    if (response[5:6] == '1'):
      messages.append("J4 Collision or Motor Error")
    if (response[6:7] == '1'):
      messages.append("J5 Collision or Motor Error")
    if (response[7:8] == '1'):
      messages.append("J6 Collision or Motor Error")

    # Actions to take on collision all errors
    correctPos()
    stopProg()        
    alarm_message = "Collision or Motor Error - See Log"

  ##REACH ERROR   
  elif (response[1:2] == 'R'):
    RUN['posOutreach'] = TRUE
    stopProg()
    message = "Position Out of Reach"
    messages.append(message)
    alarm_message = message

  ##SPLINE ERROR   
  elif (response[1:2] == 'S'):  
    stopProg()
    message = "Spline Can Only Have Move L Types"
    messages.append(message)
    alarm_message = message

  ##GCODE ERROR   
  elif (response[1:2] == 'G'):
    stopProg()
    message = "Gcode file not found"
    messages.append(message)
    alarm_message = message

  ##ESTOP BUTTON   
  elif (response[1:2] == 'B'):
    RUN['estopActive'] = TRUE
    stopProg()
    message = "Estop Button was Pressed"
    messages.append(message)
    alarm_message = message    

  ##CALIBRATION ERROR 
  elif (response[1:2] == 'A'):  
    if (response[2:3] == '1'):
      messages.append("J1 CALIBRATION ERROR")
    if (response[2:3] == '2'):
      messages.append("J2 CALIBRATION ERROR")
    if (response[2:3] == '3'):
      messages.append("J3 CALIBRATION ERROR")
    if (response[2:3] == '4'):
      messages.append("J4 CALIBRATION ERROR")
    if (response[2:3] == '5'):
      messages.append("J5 CALIBRATION ERROR")
    if (response[2:3] == '6'):
      messages.append("J6 CALIBRATION ERROR") 
    if (response[2:3] == '7'):
      messages.append("J7 CALIBRATION ERROR")
    if (response[2:3] == '8'):
      messages.append("J8 CALIBRATION ERROR")
    if (response[2:3] == '9'):
      messages.append("J9 CALIBRATION ERROR")

    alarm_message = "Calibration Error - See Log"             
     
  ##MODBUS ERROR   
  elif (response == 'Modbus Error'):
    stopProg()
    message = "Modbus Error"
    messages.append(message)
    alarm_message = message
  
  else:
    stopProg() 
    message = "Unknown Error"
    messages.append(message)
    alarm_message = message

  # After taking actions for each error type, log all messages
  for msg in messages:
    logger.error(msg)

  # After logging all message, save the error log
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))

  # Update the alarm status label once
  almStatusLab.config(text=alarm_message, style="Alarm.TLabel")
  almStatusLab2.config(text=alarm_message, style="Alarm.TLabel")
  GCalmStatusLab.config(text=alarm_message, style="Alarm.TLabel")
      
	
	

###VISION DEFS###################################################################
#################################################################################	
 
def viscalc():
  # global RUN['xMMpos']
  # global RUN['yMMpos']
  #origin x1 y1
  CAL['VisOrigXpix'] = float(VisX1PixEntryField.get())
  CAL['VisOrigXmm'] = float(VisX1RobEntryField.get()) 
  CAL['VisOrigYpix'] = float(VisY1PixEntryField.get()) 
  CAL['VisOrigYmm'] = float(VisY1RobEntryField.get()) 
  # x2 y2
  CAL['VisEndXpix'] = float(VisX2PixEntryField.get())
  CAL['VisEndXmm'] = float(VisX2RobEntryField.get()) 
  CAL['VisEndYpix'] = float(VisY2PixEntryField.get()) 
  CAL['VisEndYmm'] = float(VisY2RobEntryField.get())

  x = float(VisRetXpixEntryField.get()) 
  y = float(VisRetYpixEntryField.get()) 

  XPrange = float(CAL['VisEndXpix']) - float(CAL['VisOrigXpix'])
  XPratio = (x-float(CAL['VisOrigXpix'])) / XPrange
  XMrange = float(CAL['VisEndXmm']) - float(CAL['VisOrigXmm'])
  XMpos = float(XMrange) * float(XPratio)
  RUN['xMMpos'] = float(CAL['VisOrigXmm']) + XMpos
  ##
  YPrange = float(CAL['VisEndYpix']) - float(CAL['VisOrigYpix'])
  YPratio = (y-float(CAL['VisOrigYpix'])) / YPrange
  YMrange = float(CAL['VisEndYmm']) - float(CAL['VisOrigYmm'])
  YMpos = float(YMrange) * float(YPratio)
  RUN['yMMpos'] = float(CAL['VisOrigYmm']) + YMpos
  return (RUN['xMMpos'],RUN['yMMpos'])





# Define function to show frame
def show_frame():

    if RUN['cam_on']:

        ret, frame = RUN['cap'].read()    

        if ret:
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)    
            img = Image.fromarray(cv2image).resize((480,320))
            imgtk = ImageTk.PhotoImage(image=img)        
            live_lbl.imgtk = imgtk    
            live_lbl.configure(image=imgtk)    
        
        live_lbl.after(10, show_frame)

def start_vid():
    #global cam_on, cap
    #global cap
    stop_vid()
    RUN['cam_on'] = True
    curVisStingSel = visoptions.get()
    l = len(camList)
    for i in range(l):
      if (visoptions.get() == camList[i]):
        RUN['selectedCam'] = i
    RUN['cap'] = cv2.VideoCapture(RUN['selectedCam']) 
    show_frame()




def stop_vid():
    #global cam_on
    RUN['cam_on'] = False
    
    if RUN['cap']:
        RUN['cap'].release()

#vismenu.size

def take_pic():
  # global RUN['selectedCam']
  #global cap
  # global RUN['BGavg']
  # global RUN['mX1']
  # global RUN['mY1']
  # global RUN['mX2']
  # global RUN['mY2']

  if(RUN['cam_on']):
    ret, frame = RUN['cap'].read()
  else:
    curVisStingSel = visoptions.get()
    l = len(camList)
    for i in range(l):
      if (visoptions.get() == camList[i]):
        RUN['selectedCam'] = i
    RUN['cap'] = cv2.VideoCapture(RUN['selectedCam']) 
    ret, frame = RUN['cap'].read()

  brightness = int(VisBrightSlide.get())
  contrast = int(VisContrastSlide.get())
  CAL['zoom'] = int(VisZoomSlide.get())

  frame = np.int16(frame)
  frame = frame * (contrast/127+1) - contrast + brightness
  frame = np.clip(frame, 0, 255)
  frame = np.uint8(frame) 
  cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
  

  #get the webcam size
  height, width = cv2image.shape

  #prepare the crop
  centerX,centerY=int(height/2),int(width/2)
  radiusX,radiusY= int(CAL['zoom']*height/100),int(CAL['zoom']*width/100)

  minX,maxX=centerX-radiusX,centerX+radiusX
  minY,maxY=centerY-radiusY,centerY+radiusY

  cropped = cv2image[minX:maxX, minY:maxY]
  cv2image = cv2.resize(cropped, (width, height))

  CAL['autoBGVal'] = int(RUN['autoBG'].get())
  if(CAL['autoBGVal']==1):
    BG1 = cv2image[int(VisX1PixEntryField.get())][int(VisY1PixEntryField.get())]
    BG2 = cv2image[int(VisX1PixEntryField.get())][int(VisY2PixEntryField.get())]
    BG3 = cv2image[int(VisX2PixEntryField.get())][int(VisY2PixEntryField.get())]
    avg = int(mean([BG1,BG2,BG3]))
    RUN['BGavg'] = (avg,avg,avg) 
    background = avg
    VisBacColorEntryField.configure(state='enabled')  
    VisBacColorEntryField.delete(0, 'end')
    VisBacColorEntryField.insert(0,str(RUN['BGavg']))
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
      cv2image[y, x] = background if x >= RUN['mX2'] or x <= RUN['mX1'] or y <= RUN['mY1'] or y >= RUN['mY2'] else cv2image[y, x]  

  img = Image.fromarray(cv2image).resize((640,480))

  

  imgtk = ImageTk.PhotoImage(image=img) 
  vid_lbl.imgtk = imgtk    
  vid_lbl.configure(image=imgtk) 
  filename = 'curImage.jpg'
  cv2.imwrite(filename, cv2image)


def mask_pic():
  # global RUN['selectedCam']
  #global cap
  # global RUN['BGavg']
  # global RUN['mX1']
  # global RUN['mY1']
  # global RUN['mX2']
  # global RUN['mY2']

  if(RUN['cam_on']):
    ret, frame = RUN['cap'].read()
  else:
    curVisStingSel = visoptions.get()
    l = len(camList)
    for i in range(l):
      if (visoptions.get() == camList[i]):
        RUN['selectedCam'] = i
    RUN['cap'] = cv2.VideoCapture(RUN['selectedCam']) 
    ret, frame = RUN['cap'].read()
  brightness = int(VisBrightSlide.get())
  contrast = int(VisContrastSlide.get())
  CAL['zoom'] = int(VisZoomSlide.get())
  frame = np.int16(frame)
  frame = frame * (contrast/127+1) - contrast + brightness
  frame = np.clip(frame, 0, 255)
  frame = np.uint8(frame) 
  cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
  #get the webcam size
  height, width = cv2image.shape
  #prepare the crop
  centerX,centerY=int(height/2),int(width/2)
  radiusX,radiusY= int(CAL['zoom']*height/100),int(CAL['zoom']*width/100)
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
    # global RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'], RUN['cropping']
    #global oriImage
    # global RUN['box_points']
    # global RUN['button_down']
    # global RUN['mX1']
    # global RUN['mY1']
    # global RUN['mX2']
    # global RUN['mY2']


    cropDone = False
    

    if (not RUN['button_down']) and (event == cv2.EVENT_LBUTTONDOWN):
        RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'] = x, y, x, y
        RUN['cropping'] = True
        RUN['button_down'] = True
        RUN['box_points'] = [(x, y)]
        
    # Mouse is Moving
    elif (RUN['button_down']) and (event == cv2.EVENT_MOUSEMOVE):
        if RUN['cropping']:
            image_copy = RUN['oriImage'].copy()
            RUN['x_end'], RUN['y_end'] = x, y
            point = (x, y)
            cv2.rectangle(image_copy, RUN['box_points'][0], point, (0, 255, 0), 2)
            cv2.imshow("image", image_copy)

    # if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        RUN['button_down'] = False
        RUN['box_points'].append((x, y))
        cv2.rectangle(RUN['oriImage'], RUN['box_points'][0], RUN['box_points'][1], (0, 255, 0), 2)
        cv2.imshow("image", RUN['oriImage'])
        # record the ending (x, y) coordinates
        RUN['x_end'], RUN['y_end'] = x, y
        RUN['cropping'] = False # cropping is finished

        RUN['mX1'] = RUN['x_start']+3
        RUN['mY1'] = RUN['y_start']+3
        RUN['mX2'] = RUN['x_end']-3
        RUN['mY2'] = RUN['y_end']-3

        CAL['autoBGVal'] = int(RUN['autoBG'].get())
        if(CAL['autoBGVal']==1):
          BG1 = RUN['oriImage'][int(VisX1PixEntryField.get())][int(VisY1PixEntryField.get())]
          BG2 = RUN['oriImage'][int(VisX1PixEntryField.get())][int(VisY2PixEntryField.get())]
          BG3 = RUN['oriImage'][int(VisX2PixEntryField.get())][int(VisY2PixEntryField.get())]
          avg = int(mean([BG1,BG2,BG3]))
          RUN['BGavg'] = (avg,avg,avg) 
          background = avg
          VisBacColorEntryField.configure(state='enabled')  
          VisBacColorEntryField.delete(0, 'end')
          VisBacColorEntryField.insert(0,str(RUN['BGavg']))
          VisBacColorEntryField.configure(state='disabled')   
        else:  
          background = eval(VisBacColorEntryField.get())

        h = RUN['oriImage'].shape[0]
        w = RUN['oriImage'].shape[1]
        # loop over the image
        for y in range(0, h):
            for x in range(0, w):
                # change the pixel
                RUN['oriImage'][y, x] = background if x >= RUN['mX2'] or x <= RUN['mX1'] or y <= RUN['mY1'] or y >= RUN['mY2'] else RUN['oriImage'][y, x]

        img = Image.fromarray(RUN['oriImage'])
        imgtk = ImageTk.PhotoImage(image=img) 
        vid_lbl.imgtk = imgtk    
        vid_lbl.configure(image=imgtk) 
        filename = 'curImage.jpg'
        cv2.imwrite(filename, RUN['oriImage'])
        cv2.destroyAllWindows()



def selectMask():
  #global oriImage
  # global RUN['button_down']
  RUN['button_down'] = False
  RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'] = 0, 0, 0, 0
  mask_pic()

  image = cv2.imread('curImage.jpg')
  RUN['oriImage'] = image.copy()
  
  cv2.namedWindow("image")
  cv2.setMouseCallback("image", mask_crop)
  cv2.imshow("image", image)



def mouse_crop(event, x, y, flags, param):
    # global RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'], RUN['cropping']
    #global oriImage
    # global RUN['box_points']
    # global RUN['button_down']

    cropDone = False
    

    if (not RUN['button_down']) and (event == cv2.EVENT_LBUTTONDOWN):
        RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'] = x, y, x, y
        RUN['cropping'] = True
        RUN['button_down'] = True
        RUN['box_points'] = [(x, y)]
        
    # Mouse is Moving
    elif (RUN['button_down']) and (event == cv2.EVENT_MOUSEMOVE):
        if RUN['cropping']:
            image_copy = RUN['oriImage'].copy()
            RUN['x_end'], RUN['y_end'] = x, y
            point = (x, y)
            cv2.rectangle(image_copy, RUN['box_points'][0], point, (0, 255, 0), 2)
            cv2.imshow("image", image_copy)

    # if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        RUN['button_down'] = False
        RUN['box_points'].append((x, y))
        cv2.rectangle(RUN['oriImage'], RUN['box_points'][0], RUN['box_points'][1], (0, 255, 0), 2)
        cv2.imshow("image", RUN['oriImage'])
        # record the ending (x, y) coordinates
        RUN['x_end'], RUN['y_end'] = x, y
        RUN['cropping'] = False # cropping is finished

        refPoint = [(RUN['x_start']+3, RUN['y_start']+3), (RUN['x_end']-3, RUN['y_end']-3)]

        if len(refPoint) == 2: #when two points were found
            roi = RUN['oriImage'][refPoint[0][1]:refPoint[1][1], refPoint[0][0]:refPoint[1][0]]
            
            cv2.imshow("Cropped", roi)
            USER_INP = simpledialog.askstring(title="Teach Vision Object",
                                  prompt="Save Object As:")
            templateName = USER_INP+".jpg"                      
            cv2.imwrite(templateName, roi)
            cv2.destroyAllWindows()
            updateVisOp()  



def selectTemplate():
  #global oriImage
  # global RUN['button_down']
  RUN['button_down'] = False
  RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'] = 0, 0, 0, 0
  image = cv2.imread('curImage.jpg')
  RUN['oriImage'] = image.copy()
  
  cv2.namedWindow("image")
  cv2.setMouseCallback("image", mouse_crop)
  cv2.imshow("image", image)




def snapFind():
  # global RUN['selectedTemplate']
  # global RUN['BGavg']
  take_pic()
  template = RUN['selectedTemplate'].get()
  min_score = float(VisScoreEntryField.get())*.01
  CAL['autoBGVal'] = int(RUN['autoBG'].get())
  if(CAL['autoBGVal']==1):
    background = RUN['BGavg']
    VisBacColorEntryField.configure(state='enabled')  
    VisBacColorEntryField.delete(0, 'end')
    VisBacColorEntryField.insert(0,str(RUN['BGavg']))
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
    # global RUN['xMMpos']
    # global RUN['yMMpos']
    #global autoBG

    if(background == "Auto"):
      background = RUN['BGavg']
      VisBacColorEntryField.configure(state='enabled')  
      VisBacColorEntryField.delete(0, 'end')
      VisBacColorEntryField.insert(0,str(RUN['BGavg']))
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

    CAL['fullRotVal'] = int(RUN['fullRot'].get())

    for i in range (1):
      if(i==0):
        method = cv2.TM_CCOEFF_NORMED
      else:
        #method = cv2.TM_CCOEFF_NORMED
        method = cv2.TM_CCORR_NORMED  

      #USE 1/3 - EACH SIDE SEARCH
      if (CAL['fullRotVal'] == 0): 
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
      CAL['pick180Val'] = int(RUN['pick180'].get())  
      if (CAL['pick180Val'] == 1):
        if (highangle>90):
          highangle = -180 + highangle
        elif (highangle<-90):
          highangle = 180 + highangle
      #try closest
      CAL['pickClosestVal'] = int(RUN['pickClosest'].get())
      if (CAL['pickClosestVal'] == highangle and highangle>int(CAL['J6PosLim'])):
        highangle=CAL['J6PosLim']
      elif (CAL['pickClosestVal'] == 0 and highangle>int(CAL['J6PosLim'])):    
        status = "fail"
      if (CAL['pickClosestVal'] == 1 and highangle<(int(CAL['J6NegLim'])*-1)):
        highangle=CAL['J6NegLim']*-1
      elif (CAL['pickClosestVal'] == 0 and highangle<(int(CAL['J6NegLim'])*-1)):  
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
      VisRetXrobEntryField .insert(0,str(round(RUN['xMMpos'],2)))  
      VisRetYrobEntryField .delete(0, 'end')
      VisRetYrobEntryField .insert(0,str(round(RUN['yMMpos'],2)))  

      


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
  # global RUN['selectedTemplate']
  RUN['selectedTemplate'] = StringVar()
  if getattr(sys, 'frozen', False):
    folder = os.path.dirname(sys.executable)
  elif __file__:
    folder = os.path.dirname(os.path.realpath(__file__))
  #folder = os.path.dirname(os.path.realpath(__file__))
  filelist = [fname for fname in os.listdir(folder) if fname.endswith('.jpg')]
  Visoptmenu = ttk.Combobox(tab6, textvariable=RUN['selectedTemplate'], values=filelist, state='readonly')
  Visoptmenu.place(x=390, y=52)
  Visoptmenu.bind("<<ComboboxSelected>>", VisOpUpdate)

def VisOpUpdate(foo):
  # global RUN['selectedTemplate']
  file = RUN['selectedTemplate'].get()
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
  # global RUN['mX1']
  # global RUN['mY1']
  # global RUN['mX2']
  # global RUN['mY2']
  RUN['mX1'] = 0
  RUN['mY1'] = 0
  RUN['mX2'] = 640
  RUN['mY2'] = 480
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
  CAL['autoBGVal'] = int(RUN['autoBG'].get())
  if(CAL['autoBGVal']==1):
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
  GC_ST_E1_EntryField.insert(0,str(CAL['XcurPos']))
  GC_ST_E2_EntryField.delete(0, 'end')
  GC_ST_E2_EntryField.insert(0,str(CAL['YcurPos']))  
  GC_ST_E3_EntryField.delete(0, 'end')
  GC_ST_E3_EntryField.insert(0,str(CAL['ZcurPos']))  
  GC_ST_E4_EntryField.delete(0, 'end')
  GC_ST_E4_EntryField.insert(0,str(CAL['RzcurPos']))  
  GC_ST_E5_EntryField.delete(0, 'end')
  GC_ST_E5_EntryField.insert(0,str(CAL['RycurPos']))  
  GC_ST_E6_EntryField.delete(0, 'end')
  GC_ST_E6_EntryField.insert(0,str(CAL['RxcurPos']))
  GC_ST_WC_EntryField.delete(0, 'end')
  GC_ST_WC_EntryField.insert(0,str(RUN['WC']))  

def MoveGcodeStartPos():
  RUN['xVal'] = str(float(GC_ST_E1_EntryField.get())+float(GC_SToff_E1_EntryField.get()))
  RUN['yVal'] = str(float(GC_ST_E2_EntryField.get())+float(GC_SToff_E2_EntryField.get()))
  RUN['zVal'] = str(float(GC_ST_E3_EntryField.get())+float(GC_SToff_E3_EntryField.get()))
  rzVal = str(float(GC_ST_E4_EntryField.get())+float(GC_SToff_E4_EntryField.get()))
  ryVal = str(float(GC_ST_E5_EntryField.get())+float(GC_SToff_E5_EntryField.get()))
  rxVal = str(float(GC_ST_E6_EntryField.get())+float(GC_SToff_E6_EntryField.get()))
  J7Val = str(CAL['J7PosCur'])
  J8Val = str(CAL['J8PosCur'])
  J9Val = str(CAL['J9PosCur'])
  speedPrefix = "Sm"
  Speed = "25"
  ACCspd = "10"
  DECspd = "10"
  ACCramp = "100"
  RUN['WC'] = GC_ST_WC_EntryField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
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
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    response = str(RUN['ser'].readline().strip(),'utf-8')
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
  RUN['ser'].write(command.encode())
  RUN['ser'].flushInput()
  time.sleep(.1)
  response = str(RUN['ser'].readline().strip(),'utf-8')
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
    #global estopActive
    Fn = Filename + ".txt"
    command = "PG"+"Fn"+Fn+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    response = str(RUN['ser'].readline().strip(),'utf-8')
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      displayPosition(response)
      if (RUN['estopActive']):
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
    RUN['ser'].write(command.encode())
    RUN['ser'].flushInput()
    time.sleep(.1)
    response = str(RUN['ser'].readline().strip(),'utf-8')  
    last = tab7.gcodeView.index('end')
    for row in range (0,last):
      tab7.gcodeView.itemconfig(row, {'fg': 'black'})
    def GCthreadProg():
      # global RUN['GCrowinproc']
      # global RUN['GCstopQueue']
      # global RUN['splineActive']
      # global RUN['prevxVal']
      # global RUN['prevyVal']
      # global RUN['prevzVal']
      RUN['prevxVal'] = 0
      RUN['prevyVal'] = 0
      RUN['prevzVal'] = 0
      RUN['GCstopQueue'] = "0"
      RUN['splineActive'] = "0"
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
        RUN['GCrowinproc'] = 1
        GCexecuteRow()
        while RUN['GCrowinproc'] == 1:
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
    # global RUN['cmdType']
    # global RUN['splineActive']
    # global RUN['GCstopQueue']
    lastProg = ""
    tab7.GCrunTrue = 0
    GCalmStatusLab.config(text="GCODE CONVERSION STOPPED",  style="Alarm.TLabel")
    if(RUN['splineActive'] ==1):
      RUN['splineActive'] = "0"
      if(RUN['stopQueue'] == "1"):
        RUN['stopQueue'] = "0"
        stop()
      if (RUN['moveInProc'] == 1):
        RUN['moveInProc'] == 2
      command = "SS\n" 
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        ErrorHandler(response)   
      else:
        displayPosition(response)         

def GCexecuteRow():
  # global RUN['GCrowinproc']
  # global RUN['LineDist']
  # global RUN['Xv']
  # global RUN['Yv']
  # global RUN['Zv']
  #global moveInProc
  # global RUN['splineActive']
  # global RUN['stopQueue']
  #global gcodeSpeed
  #global inchTrue
  # global RUN['prevxVal']
  # global RUN['prevyVal']
  # global RUN['prevzVal']
  # global RUN['xVal']
  # global RUN['yVal']
  # global RUN['zVal']
  GCstartTime = time.time()
  GCselRow = tab7.gcodeView.curselection()[0]
  tab7.gcodeView.see(GCselRow+2)
  data = list(map(int, tab7.gcodeView.curselection()))
  command=tab7.gcodeView.get(data[0]).decode()
  RUN['cmdType'] =command[:1]
  subCmd=command[1:command.find(" ")].rstrip()


  ## F ##
  if (RUN['cmdType'] == "F"):
    RUN['gcodeSpeed']=command[command.find("F")+1:]


  ## G ##
  if (RUN['cmdType'] == "G"):

    #IMPERIAL
    if (subCmd == "20"):
      RUN['inchTrue'] = True; 
    
    #METRIC
    if (subCmd == "21"):
      RUN['inchTrue'] = False;
    
    #ABSOLUTE / INCREMENTAL - HOME (absolute is forced and moves to start position offset)
    if (subCmd == "90" or subCmd == "91" or subCmd == "28"):
      
      RUN['xVal'] = str(float(GC_ST_E1_EntryField.get())+float(GC_SToff_E1_EntryField.get()))
      RUN['yVal'] = str(float(GC_ST_E2_EntryField.get())+float(GC_SToff_E2_EntryField.get()))
      RUN['zVal'] = str(float(GC_ST_E3_EntryField.get())+float(GC_SToff_E3_EntryField.get()))
      rzVal = str(float(GC_ST_E4_EntryField.get())+float(GC_SToff_E4_EntryField.get()))
      ryVal = str(float(GC_ST_E5_EntryField.get())+float(GC_SToff_E5_EntryField.get()))
      rxVal = str(float(GC_ST_E6_EntryField.get())+float(GC_SToff_E6_EntryField.get()))
      J7Val = str(CAL['J7PosCur'])
      J8Val = str(CAL['J8PosCur'])
      J9Val = str(CAL['J9PosCur'])
      speedPrefix = "Sm"
      Speed = "25"
      ACCspd = "10"
      DECspd = "10"
      ACCramp = "100"
      RUN['WC'] = GC_ST_WC_EntryField.get()
      LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
      Filename = GcodeFilenameField.get() + ".txt"
      command = "WC"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"Fn"+Filename+"\n"
      cmdSentEntryField.delete(0, 'end') 


      cmdSentEntryField.insert(0,command)
      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.1)
      response = str(RUN['ser'].readline().strip(),'utf-8')
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
        RUN['xVal'] =xtemp[:xtemp.find(" ")]
        RUN['xVal'] =str(round(float(xVal),3))
      else:
        RUN['xVal'] =""  
      if("Y" in command):
        ytemp=command[command.find("Y")+1:]     
        RUN['yVal'] =ytemp[:ytemp.find(" ")]
        RUN['yVal'] =str(round(float(yVal),3))
      else:
        RUN['yVal'] =""
      if("Z" in command):
        ztemp=command[command.find("Z")+1:]     
        RUN['zVal'] =ztemp[:ztemp.find(" ")]
        RUN['zVal'] =str(round(float(zVal),3))
      else:
        RUN['zVal'] =""
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
       


      if(RUN['xVal'] != ""):
        if(RUN['inchTrue']):
          RUN['xVal'] =str(float(xVal)*25.4)
        RUN['xVal'] = str(round((float(GC_ST_E1_EntryField.get())+float(xVal)),3))
      else:
        try:
          if(RUN['prevxVal'] != 0):
            RUN['xVal'] = RUN['prevxVal']
          else:  
            RUN['xVal'] = str(CAL['XcurPos'])
        except:
          RUN['xVal'] = str(CAL['XcurPos'])   


      if(RUN['yVal'] != ""):
        if(RUN['inchTrue']):
          RUN['yVal'] =str(float(yVal)*25.4)
        RUN['yVal'] = str(round((float(GC_ST_E2_EntryField.get())+float(yVal)),3))
      else:
        try:
          if(RUN['prevyVal'] != 0):
            RUN['yVal'] = RUN['prevyVal']
          else: 
            RUN['yVal'] = str(CAL['YcurPos'])
        except:
          RUN['yVal'] = str(CAL['YcurPos'])  
        
      if(RUN['zVal'] != ""):
        if(RUN['inchTrue']):
          RUN['zVal'] =str(float(zVal)*25.4)
        RUN['zVal'] = str(round((float(GC_ST_E3_EntryField.get())+float(zVal)),3))
      else:
        try:
          if(RUN['prevzVal'] != 0):
            RUN['zVal'] = RUN['prevzVal']
          else: 
            RUN['zVal'] = str(CAL['ZcurPos'])
        except:
          RUN['zVal'] = str(CAL['ZcurPos'])          

      if(aVal != ""):
        rzVal = str(float(GC_ST_E4_EntryField.get())+float(aVal))
        if (np.sign(float(rzVal)) != np.sign(float(CAL['RzcurPos']))):
          rzVal=str(round((float(rzVal)*-1),3))
      else:
        rzVal = str(CAL['RzcurPos'])
      
      if(bVal != ""):
        ryVal = str(round((float(GC_ST_E5_EntryField.get())+float(bVal))),3)
      else:
        ryVal = str(CAL['RycurPos'])

      if(cVal != ""):
        rxVal = str(round((float(GC_ST_E6_EntryField.get())+float(cVal)),3))
      else:
        rxVal = str(CAL['RxcurPos'])

      if(eVal != ""):
        J7Val = eVal
      else:
        J7Val = str(CAL['J7PosCur'])
      
      J8Val = str(CAL['J8PosCur'])
      J9Val = str(CAL['J9PosCur'])
      
      if(fVal != ""):
        if(RUN['inchTrue']):
          RUN['gcodeSpeed'] = str(round((float(fVal)/25.4),2))
        else:
          RUN['gcodeSpeed'] = str(round((float(fVal)/60),2))  
      speedPrefix = "Sm"
      Speed = RUN['gcodeSpeed']



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
      RUN['WC'] = GC_ST_WC_EntryField.get()
      #LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
      LoopMode ="111111"
      #DisWrist = str(CAL['DisableWristRotVal'].get())
      Filename = GcodeFilenameField.get() + ".txt"

      command = "WC"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+RUN['WC']+"Lm"+LoopMode+"Fn"+Filename+"\n"
      RUN['prevxVal'] = RUN['xVal']
      RUN['prevyVal'] = RUN['yVal']
      RUN['prevzVal'] = RUN['zVal']
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)

      #tab8.ElogView.insert(END, command)
      #value=tab8.ElogView.get(0,END)
      #pickle.dump(value,open("ErrorLog","wb"))

      RUN['ser'].write(command.encode())
      RUN['ser'].flushInput()
      time.sleep(.05)
      #ser.read()
      response = str(RUN['ser'].readline().strip(),'utf-8')
      if (response[:1] == 'E'):
        tab7.GCrunTrue = 0
        GCalmStatusLab.config(text="UNABLE TO WRITE TO SD CARD",  style="Alarm.TLabel")
        ErrorHandler(response)   
      else:
        displayPosition(response)

  RUN['GCrowinproc'] = 0

  

   


        



  
####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
"""
COMPLETE TAB 1 REFACTORING - FROM .place() TO .grid()
This replaces lines 11249-12534 in the original AR4.py file
"""

#####TAB 1
##########################################################################


def posRegFieldVisible(self):
  """Show/hide position register field based on move type selection"""
  curCmdtype = options.get()
  if (curCmdtype=="Move PR" or curCmdtype=="OFF PR " or curCmdtype=="Teach PR"):
    SavePosEntryField.grid()  # Show the field
  else:
    SavePosEntryField.grid_remove()  # Hide the field

# Tkinter variables for Tab 1
speedOption = StringVar(tab1)
options = StringVar(tab1)

# Entry fields for command builders (created here, placed in frames later)
waitTimeEntryField = Entry(tab1, width=4, justify="center")
tabNumEntryField = Entry(tab1, width=4, justify="center")
jumpTabEntryField = Entry(tab1, width=4, justify="center")
servoNumEntryField = Entry(tab1, width=4, justify="center")
servoPosEntryField = Entry(tab1, width=4, justify="center")
regNumEntryField = Entry(tab1, width=4, justify="center")
regEqEntryField = Entry(tab1, width=4, justify="center")
visPassEntryField = Entry(tab1, width=4, justify="center")
visFailEntryField = Entry(tab1, width=4, justify="center")
changeProgEntryField = Entry(tab1, width=14, justify="center")
PlayGCEntryField = Entry(tab1, width=14, justify="center")
auxPortEntryField = Entry(tab1, width=4, justify="center")
auxCharEntryField = Entry(tab1, width=4, justify="center")
storPosNumEntryField = Entry(tab1, width=4, justify="center")
storPosElEntryField = Entry(tab1, width=4, justify="center")
storPosValEntryField = Entry(tab1, width=4, justify="center")
waitTimeoutEntryField = Entry(tab1, width=4, justify="center")
waitInputEntryField = Entry(tab1, width=4, justify="center")
waitInputOffEntryField = Entry(tab1, width=4, justify="center")
waitVarEntryField = Entry(tab1, width=4, justify="center")
outputOnEntryField = Entry(tab1, width=4, justify="center")
outputOffEntryField = Entry(tab1, width=4, justify="center")
setInputEntryField = Entry(tab1, width=4, justify="center")
setVarEntryField = Entry(tab1, width=4, justify="center")
IfInputEntryField = Entry(tab1, width=4, justify="center")
IfVarEntryField = Entry(tab1, width=4, justify="center")
IfDestEntryField = Entry(tab1, width=4, justify="center")

IfVarEntryField = Entry(tab1, width=4, justify="center")
IfInputEntryField = Entry(tab1, width=4, justify="center")
waitVarEntryField = Entry(tab1, width=4, justify="center")
waitInputEntryField = Entry(tab1, width=4, justify="center")
waitTimeoutEntryField = Entry(tab1, width=5, justify="center")
setVarEntryField = Entry(tab1, width=4, justify="center")
setInputEntryField = Entry(tab1, width=4, justify="center")
auxPortEntryField = Entry(tab1, width=4, justify="center")
auxCharEntryField = Entry(tab1, width=4, justify="center")
storPosNumEntryField = Entry(tab1, width=4, justify="center")
storPosElEntryField = Entry(tab1, width=4, justify="center")
storPosValEntryField = Entry(tab1, width=4, justify="center")



# All Entry Fields for Tab 1


### TAB 1 - MAIN CONTROLS - REFACTORED TO USE GRID LAYOUT
##########################################################################

# Configure tab1 main grid layout (3-column design)
tab1.grid_rowconfigure(0, weight=1)  # Main content area expands
tab1.grid_columnconfigure(0, weight=0, minsize=200)  # Left panel - fixed minimum width (narrower)
tab1.grid_columnconfigure(1, weight=1, minsize=550)  # Center panel - expands
tab1.grid_columnconfigure(2, weight=0, minsize=800)  # Right panel - fixed minimum width

# ============================================================================
# LEFT PANEL - Program Controls
# ============================================================================
leftPanel = Frame(tab1, relief="raised", borderwidth=1)
leftPanel.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)

# Configure left panel grid
leftPanel.grid_rowconfigure(20, weight=1)  # Spacer row at bottom to push controls to top
leftPanel.grid_columnconfigure(0, weight=1)
leftPanel.grid_columnconfigure(1, weight=1)

# Row 0: Program label and entry (with more top padding)
ProgLab = Label(leftPanel, text="Program:")
ProgLab.grid(row=0, column=0, sticky="w", padx=5, pady=(8, 2))

ProgEntryField = Entry(leftPanel, width=15, justify="center")
ProgEntryField.grid(row=0, column=1, sticky="ew", padx=5, pady=(8, 2))

# Row 1: Load button
loadBut = ttk.Button(leftPanel, text="Load", command=loadProg)
loadBut.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

# Row 2: New Prog button
savProg = ttk.Button(leftPanel, text="New Prog", command=CreateProg)
savProg.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

# Row 3: Incremental Jog checkbox and entry field
IncJogCbut = Checkbutton(leftPanel, text="Inc Jog", variable=RUN['IncJogStat'])
IncJogCbut.grid(row=3, column=0, sticky="w", padx=5, pady=2)

incrementEntryField = Entry(leftPanel, width=6, justify="center")
incrementEntryField.grid(row=3, column=1, sticky="ew", padx=5, pady=2)

# Row 4: Current Row
curRowLab = Label(leftPanel, text="Current Row:")
curRowLab.grid(row=4, column=0, sticky="w", padx=5, pady=2)

curRowEntryField = Entry(leftPanel, width=6, justify="center")
curRowEntryField.grid(row=4, column=1, sticky="ew", padx=5, pady=2)

# Row 5: Motion controls frame (with Xbox button at bottom)
speedFrame = LabelFrame(leftPanel, text="Motion", padding=5)
speedFrame.grid(row=5, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

speedFrame.grid_columnconfigure(1, weight=1)

speedLab = Label(speedFrame, text="Speed")
speedLab.grid(row=0, column=0, sticky="w", padx=2, pady=1)

speedEntryField = Entry(speedFrame, width=4, justify="center")
speedEntryField.grid(row=0, column=1, sticky="ew", padx=2, pady=1)

speedOptionMenu = OptionMenu(speedFrame, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
speedOptionMenu.grid(row=0, column=2, sticky="ew", padx=2, pady=1)

ACCLab = Label(speedFrame, text="Acceleration")
ACCLab.grid(row=1, column=0, sticky="w", padx=2, pady=1)

ACCspeedField = Entry(speedFrame, width=4, justify="center")
ACCspeedField.grid(row=1, column=1, sticky="ew", padx=2, pady=1)

Label(speedFrame, text="%").grid(row=1, column=2, sticky="w", padx=2)

DECLab = Label(speedFrame, text="Deceleration")
DECLab.grid(row=2, column=0, sticky="w", padx=2, pady=1)

DECspeedField = Entry(speedFrame, width=4, justify="center")
DECspeedField.grid(row=2, column=1, sticky="ew", padx=2, pady=1)

Label(speedFrame, text="%").grid(row=2, column=2, sticky="w", padx=2)

RampLab = Label(speedFrame, text="Ramp")
RampLab.grid(row=3, column=0, sticky="w", padx=2, pady=1)

ACCrampField = Entry(speedFrame, width=4, justify="center")
ACCrampField.grid(row=3, column=1, sticky="ew", padx=2, pady=1)

Label(speedFrame, text="%").grid(row=3, column=2, sticky="w", padx=2)

RoundLab = Label(speedFrame, text="Rounding")
RoundLab.grid(row=4, column=0, sticky="w", padx=2, pady=1)

roundEntryField = Entry(speedFrame, width=4, justify="center")
roundEntryField.grid(row=4, column=1, sticky="ew", padx=2, pady=1)

Label(speedFrame, text="mm").grid(row=4, column=2, sticky="w", padx=2)

# Xbox button at bottom of Motion frame (centered)
if CE['Platform']['IS_WINDOWS']:
    xboxBut = Button(speedFrame, command=start_xbox)
else:
    xboxBut = Button(speedFrame, command=xbox)
xboxPhoto = PhotoImage(file="xbox.png")
xboxBut.config(image=xboxPhoto)
xboxBut.grid(row=5, column=0, columnspan=3, pady=(5, 0))

# Row 6: Virtual controls frame
virtualFrame = LabelFrame(leftPanel, text="Virtual", padding=5)
virtualFrame.grid(row=6, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

virtualFrame.grid_columnconfigure(0, weight=1)

virtRobBut = ttk.Button(virtualFrame, text="Virtual Robot", command=lambda: launch_vtk_nonblocking(tab1))
virtRobBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

offline_button = ttk.Button(virtualFrame, text="Run Offline", width=20, command=toggle_offline_mode, style="Online.TButton")
offline_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
# Row 7: Position Commands frame
posFrame = LabelFrame(leftPanel, text="Position Commands", padding=5)
posFrame.grid(row=7, column=0, columnspan=2, sticky="new", padx=5, pady=(2, 5))

posFrame.grid_columnconfigure(0, weight=1)

moveSelMenu = OptionMenu(posFrame, options, "Move J", "Move J", "OFF J", "Move L", "Move R", "Move A Mid", "Move A End", "Move C Center", "Move C Start", "Move C Plane", "Start Spline", "End Spline", "Move PR", "OFF PR ", "Teach PR", "Move Vis", command=posRegFieldVisible)
moveSelMenu.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

# Position Register Entry Field (hidden by default, shown when PR moves selected)
SavePosEntryField = Entry(posFrame, width=4, justify="center")
SavePosEntryField.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
SavePosEntryField.grid_remove()  # Hidden by default, shown by posRegFieldVisible()

teachPosBut = ttk.Button(posFrame, text="Teach New Position", command=teachInsertBelSelected)
teachPosBut.grid(row=2, column=0, sticky="ew", padx=2, pady=2)

modPosBut = ttk.Button(posFrame, text="Modify Position", command=teachReplaceSelected)
modPosBut.grid(row=3, column=0, sticky="ew", padx=2, pady=2)

deleteBut = ttk.Button(posFrame, text="Delete", command=deleteitem)
deleteBut.grid(row=4, column=0, sticky="ew", padx=2, pady=2)

returnBut = ttk.Button(posFrame, text="Return", command=insertReturn)
returnBut.grid(row=5, column=0, sticky="ew", padx=2, pady=2)

autoCalBut = ttk.Button(posFrame, text="Auto Calibrate CMD", command=insCalibrate)
CalibrateBut = autoCalBut  # Alias for compatibility
autoCalBut.grid(row=6, column=0, sticky="ew", padx=2, pady=2)

# Row 8: Vision container
visionFrame = LabelFrame(leftPanel, text="Vision", padding=5)
visionFrame.grid(row=8, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

visionFrame.grid_columnconfigure(0, weight=1)
visionFrame.grid_columnconfigure(1, weight=1)
visionFrame.grid_columnconfigure(2, weight=1)
visionFrame.grid_columnconfigure(3, weight=1)

# Row 0: Camera On and Camera Off buttons
camOnBut = ttk.Button(visionFrame, text="Camera On", command=cameraOn)
camOnBut.grid(row=0, column=0, columnspan=2, sticky="ew", padx=2, pady=2)

camOffBut = ttk.Button(visionFrame, text="Camera Off", command=cameraOff)
camOffBut.grid(row=0, column=2, columnspan=2, sticky="ew", padx=2, pady=2)

# Row 1: Vision Find button
visFindBut = ttk.Button(visionFrame, text="Vision Find", command=insertvisFind)
visFindBut.grid(row=1, column=0, columnspan=2, sticky="ew", padx=2, pady=(5, 2))

# Row 2: Pass Tab and Fail Tab labels
Label(visionFrame, text="Pass Tab", font=("Arial", 8)).grid(row=2, column=0, sticky="e", padx=(2, 1))
Label(visionFrame, text="Fail Tab", font=("Arial", 8)).grid(row=2, column=1, sticky="w", padx=(1, 2))

# Row 3: Pass Tab and Fail Tab entry fields
visPassEntryField = Entry(visionFrame, width=6, justify="center")
visPassEntryField.grid(row=3, column=0, sticky="ew", padx=(2, 1), pady=(0, 2))

visFailEntryField = Entry(visionFrame, width=6, justify="center")
visFailEntryField.grid(row=3, column=1, sticky="ew", padx=(1, 2), pady=(0, 2))

# Vision Find additional fields (placeholders - full UI on Tab 6)
VisBacColorEntryField = Entry(tab1, width=6)  # Hidden, used by insertvisFind
VisScoreEntryField = Entry(tab1, width=6)  # Hidden, used by insertvisFind
VisBacColorEntryField.insert(0, "116, 116, 116")  # Default background color
VisScoreEntryField.insert(0, "85")  # Default score threshold

# Row 9: Wait container
waitContainer = LabelFrame(leftPanel, text="Wait", padding=5)
waitContainer.grid(row=9, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

waitContainer.grid_columnconfigure(0, weight=1)
waitContainer.grid_columnconfigure(1, weight=0)

waitSecBut = ttk.Button(waitContainer, text="Wait Time (seconds)", command=waitTime)
waitSecBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

waitSecField = Entry(waitContainer, width=8, justify="center")
waitSecField.grid(row=0, column=1, sticky="w", padx=2, pady=2)

# ============================================================================
# CENTER PANEL - Program Display and Controls
# ============================================================================
centerPanel = Frame(tab1)
centerPanel.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)

# Configure center panel grid
centerPanel.grid_rowconfigure(0, weight=0)  # Status message
centerPanel.grid_rowconfigure(1, weight=0)  # Play controls
centerPanel.grid_rowconfigure(2, weight=3)  # Program view (more weight) (expands)
centerPanel.grid_rowconfigure(3, weight=0)  # Manual entry (at bottom)
centerPanel.grid_rowconfigure(4, weight=0)  # Position controls
centerPanel.grid_rowconfigure(5, weight=0)  # Command builders
centerPanel.grid_rowconfigure(6, weight=0)  # Bottom buttons
centerPanel.grid_columnconfigure(0, weight=1)

# Row 0: Status message
runStatusLab = Label(centerPanel, text="SYSTEM STARTING - PLEASE WAIT", font=("Arial", 10, "bold"), style="OK.TLabel")
runStatusLab.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

# Create alias for compatibility with existing code
almStatusLab = runStatusLab

# Row 0.5: Play controls
playFrame = Frame(centerPanel)
playFrame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

for i in range(4):
    playFrame.grid_columnconfigure(i, weight=1)

# Load play/stop button images
try:
    playPhoto = PhotoImage(file="play-icon.png")
    stopPhoto = PhotoImage(file="stop-icon.png")
    use_images = True
except:
    use_images = False

# Import tk to get regular Button (not ttk.Button)
import tkinter as tk_base

playBut = tk_base.Button(playFrame, command=runProg)
if use_images:
    playBut.config(image=playPhoto)
    playBut.image = playPhoto  # Keep reference
else:
    playBut.config(text="▶", width=8, height=2)
playBut.grid(row=0, column=0, sticky="nsew", padx=2, pady=0)

revBut = tk_base.Button(playFrame, text="REV", command=stepRev, height=2, font=("Arial", 10))
revBut.grid(row=0, column=1, sticky="nsew", padx=2, pady=0)

fwdBut = tk_base.Button(playFrame, text="FWD", command=stepFwd, height=2, font=("Arial", 10))
fwdBut.grid(row=0, column=2, sticky="nsew", padx=2, pady=0)

stopBut = tk_base.Button(playFrame, command=stopProg)
if use_images:
    stopBut.config(image=stopPhoto)
    stopBut.image = stopPhoto  # Keep reference
else:
    stopBut.config(text="⬛", width=8, height=2)
stopBut.grid(row=0, column=3, sticky="nsew", padx=2, pady=0)

# Row 2: Program view with scrollbar
progframe = Frame(centerPanel, relief="sunken", borderwidth=1)
progframe.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)

scrollbar = Scrollbar(progframe)
scrollbar.pack(side=RIGHT, fill=Y)

tab1.progView = Listbox(progframe, exportselection=0, width=70, height=28, yscrollcommand=scrollbar.set)
tab1.progView.bind('<<ListboxSelect>>', progViewselect)
tab1.progView.pack(side=LEFT, fill=BOTH, expand=True)

scrollbar.config(command=tab1.progView.yview)

# Row 3: Manual Program Entry
manEntryFrame = LabelFrame(centerPanel, text="Manual Program Entry", padding=5)
manEntryFrame.grid(row=3, column=0, sticky="ew", padx=5, pady=5)

# Configure equal width columns for buttons
for i in range(5):
    manEntryFrame.grid_columnconfigure(i, weight=1)

# Entry field at top (full width)
manEntryField = Entry(manEntryFrame, width=60)
manEntryField.grid(row=0, column=0, columnspan=5, sticky="ew", padx=2, pady=(2, 5))

# Five buttons below entry field (equal width)
getSelBut = ttk.Button(manEntryFrame, text="Get Selected", command=getSel)
getSelBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

insertBut = ttk.Button(manEntryFrame, text="Insert", command=manInsItem)
insertBut.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

replaceBut = ttk.Button(manEntryFrame, text="Replace", command=manReplItem)
replaceBut.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

openTextBut = ttk.Button(manEntryFrame, text="Open Text", command=openText)
openTextBut.grid(row=1, column=3, sticky="ew", padx=2, pady=2)

reloadBut = ttk.Button(manEntryFrame, text="Reload", command=reloadProg)
reloadBut.grid(row=1, column=4, sticky="ew", padx=2, pady=2)

# Row 3: Position controls
# Position Commands moved to left panel

# Wait Time moved to left panel Wait container

# Row 5: Tab and Servo controls
# Duplicate buttons removed - now in proper containers

# ============================================================================
# RIGHT PANEL - Joint and Cartesian Controls
# ============================================================================
rightPanel = Frame(tab1)
rightPanel.grid(row=0, column=2, sticky="nsew", padx=2, pady=2)

# Configure right panel grid
rightPanel.grid_rowconfigure(0, weight=0)  # Joint controls - fixed height
rightPanel.grid_rowconfigure(1, weight=0)  # Cartesian controls - fixed height
rightPanel.grid_rowconfigure(2, weight=0)  # Tool controls - fixed height
rightPanel.grid_rowconfigure(3, weight=0)  # Command builders - fixed height
rightPanel.grid_rowconfigure(4, weight=0)  # Navigation - fixed height
rightPanel.grid_rowconfigure(5, weight=0)  # Register commands - fixed height
rightPanel.grid_rowconfigure(6, weight=0)  # Device commands - fixed height
rightPanel.grid_rowconfigure(7, weight=1)  # Additional axes - compresses first when height reduced
rightPanel.grid_rowconfigure(8, weight=2)  # Spacer - expands most
rightPanel.grid_columnconfigure(0, weight=1)
rightPanel.grid_columnconfigure(1, weight=1)

# Joint controls container (J1-J6)
jointFrame = LabelFrame(rightPanel, text="Joint Control (J1-J6)", padding=5)
jointFrame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

jointFrame.grid_columnconfigure(0, weight=1)
jointFrame.grid_columnconfigure(1, weight=1)

# Helper function to create joint control widgets
def create_joint_jog_frame(parent, row, col, joint_name, joint_num):
    """Create a joint jog control frame with label, entry, buttons, and slider"""
    frame = Frame(parent)
    frame.grid(row=row, column=col, sticky="ew", padx=2, pady=2)
    
    # Configure internal grid
    frame.grid_columnconfigure(0, weight=0)  # Label
    frame.grid_columnconfigure(1, weight=0)  # Entry
    frame.grid_columnconfigure(2, weight=0)  # Neg button
    frame.grid_columnconfigure(3, weight=1)  # Slider
    frame.grid_columnconfigure(4, weight=0)  # Pos button
    
    # Joint label
    lab = Label(frame, font=("Arial", 14), text=joint_name)
    lab.grid(row=0, column=0, padx=2)
    
    # Current angle entry
    entry = Entry(frame, width=6, justify="center")
    entry.grid(row=0, column=1, padx=2)
    
    # Negative jog button
    neg_but = Button(frame, text="-", width=3)
    neg_but.grid(row=0, column=2, padx=2)
    
    # Slider
    slider = Scale(frame, from_=-170, to=170, orient=HORIZONTAL, length=150)
    slider.grid(row=0, column=3, sticky="ew", padx=2)
    
    # Positive jog button
    pos_but = Button(frame, text="+", width=3)
    pos_but.grid(row=0, column=4, padx=2)
    
    # Limit labels (second row)
    neg_lim_lab = Label(frame, font=("Arial", 8), text="-170", style="Jointlim.TLabel")
    neg_lim_lab.grid(row=1, column=2, sticky="w")
    
    pos_lim_lab = Label(frame, font=("Arial", 8), text="170", style="Jointlim.TLabel")
    pos_lim_lab.grid(row=1, column=4, sticky="e")
    
    slide_label = Label(frame, font=("Arial", 8))
    slide_label.grid(row=1, column=3)
    
    return frame, entry, neg_but, pos_but, slider, slide_label, neg_lim_lab, pos_lim_lab

# Create J1-J6 frames (2 columns x 3 rows)
##J1
J1jogFrame, J1curAngEntryField, J1jogNegBut, J1jogPosBut, J1jogslide, J1slidelabel, J1negLimLab, J1posLimLab = create_joint_jog_frame(jointFrame, 0, 0, "J1", 1)

# Bind J1 button events
def SelJ1jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J1jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(10)  
J1jogNegBut.bind("<ButtonPress>", SelJ1jogNeg)
J1jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ1jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J1jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(11)  
J1jogPosBut.bind("<ButtonPress>", SelJ1jogPos)
J1jogPosBut.bind("<ButtonRelease>", StopJog)

def J1sliderUpdate(foo):
  J1slidelabel.config(text=round(float(J1jogslide.get()),2))   
def J1sliderExecute(foo): 
  J1delta = float(J1jogslide.get()) - float(J1curAngEntryField.get())
  if (J1delta < 0):
    J1jogNeg(abs(J1delta))
  else:
    J1jogPos(abs(J1delta))       
J1jogslide.config(command=J1sliderUpdate)
J1jogslide.bind("<ButtonRelease-1>", J1sliderExecute)

##J2
J2jogFrame, J2curAngEntryField, J2jogNegBut, J2jogPosBut, J2jogslide, J2slidelabel, J2negLimLab, J2posLimLab = create_joint_jog_frame(jointFrame, 1, 0, "J2", 2)

def SelJ2jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J2jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(20)  
J2jogNegBut.bind("<ButtonPress>", SelJ2jogNeg)
J2jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ2jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J2jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(21)  
J2jogPosBut.bind("<ButtonPress>", SelJ2jogPos)
J2jogPosBut.bind("<ButtonRelease>", StopJog)

def J2sliderUpdate(foo):
  J2slidelabel.config(text=round(float(J2jogslide.get()),2))   
def J2sliderExecute(foo): 
  J2delta = float(J2jogslide.get()) - float(J2curAngEntryField.get())
  if (J2delta < 0):
    J2jogNeg(abs(J2delta))
  else:
    J2jogPos(abs(J2delta))       
J2jogslide.config(command=J2sliderUpdate)
J2jogslide.bind("<ButtonRelease-1>", J2sliderExecute)

##J3
J3jogFrame, J3curAngEntryField, J3jogNegBut, J3jogPosBut, J3jogslide, J3slidelabel, J3negLimLab, J3posLimLab = create_joint_jog_frame(jointFrame, 2, 0, "J3", 3)

def SelJ3jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J3jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(30)  
J3jogNegBut.bind("<ButtonPress>", SelJ3jogNeg)
J3jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ3jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J3jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(31)  
J3jogPosBut.bind("<ButtonPress>", SelJ3jogPos)
J3jogPosBut.bind("<ButtonRelease>", StopJog)

def J3sliderUpdate(foo):
  J3slidelabel.config(text=round(float(J3jogslide.get()),2))   
def J3sliderExecute(foo): 
  J3delta = float(J3jogslide.get()) - float(J3curAngEntryField.get())
  if (J3delta < 0):
    J3jogNeg(abs(J3delta))
  else:
    J3jogPos(abs(J3delta))       
J3jogslide.config(command=J3sliderUpdate)
J3jogslide.bind("<ButtonRelease-1>", J3sliderExecute)

##J4
J4jogFrame, J4curAngEntryField, J4jogNegBut, J4jogPosBut, J4jogslide, J4slidelabel, J4negLimLab, J4posLimLab = create_joint_jog_frame(jointFrame, 0, 1, "J4", 4)

def SelJ4jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J4jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(40)  
J4jogNegBut.bind("<ButtonPress>", SelJ4jogNeg)
J4jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ4jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J4jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(41)  
J4jogPosBut.bind("<ButtonPress>", SelJ4jogPos)
J4jogPosBut.bind("<ButtonRelease>", StopJog)

def J4sliderUpdate(foo):
  J4slidelabel.config(text=round(float(J4jogslide.get()),2))   
def J4sliderExecute(foo): 
  J4delta = float(J4jogslide.get()) - float(J4curAngEntryField.get())
  if (J4delta < 0):
    J4jogNeg(abs(J4delta))
  else:
    J4jogPos(abs(J4delta))       
J4jogslide.config(command=J4sliderUpdate)
J4jogslide.bind("<ButtonRelease-1>", J4sliderExecute)

##J5
J5jogFrame, J5curAngEntryField, J5jogNegBut, J5jogPosBut, J5jogslide, J5slidelabel, J5negLimLab, J5posLimLab = create_joint_jog_frame(jointFrame, 1, 1, "J5", 5)

def SelJ5jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J5jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(50)  
J5jogNegBut.bind("<ButtonPress>", SelJ5jogNeg)
J5jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ5jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J5jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(51)  
J5jogPosBut.bind("<ButtonPress>", SelJ5jogPos)
J5jogPosBut.bind("<ButtonRelease>", StopJog)

def J5sliderUpdate(foo):
  J5slidelabel.config(text=round(float(J5jogslide.get()),2))   
def J5sliderExecute(foo): 
  J5delta = float(J5jogslide.get()) - float(J5curAngEntryField.get())
  if (J5delta < 0):
    J5jogNeg(abs(J5delta))
  else:
    J5jogPos(abs(J5delta))       
J5jogslide.config(command=J5sliderUpdate)
J5jogslide.bind("<ButtonRelease-1>", J5sliderExecute)

##J6
J6jogFrame, J6curAngEntryField, J6jogNegBut, J6jogPosBut, J6jogslide, J6slidelabel, J6negLimLab, J6posLimLab = create_joint_jog_frame(jointFrame, 2, 1, "J6", 6)

def SelJ6jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J6jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(60)  
J6jogNegBut.bind("<ButtonPress>", SelJ6jogNeg)
J6jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ6jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J6jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(61)  
J6jogPosBut.bind("<ButtonPress>", SelJ6jogPos)
J6jogPosBut.bind("<ButtonRelease>", StopJog)

def J6sliderUpdate(foo):
  J6slidelabel.config(text=round(float(J6jogslide.get()),2))   
def J6sliderExecute(foo): 
  J6delta = float(J6jogslide.get()) - float(J6curAngEntryField.get())
  if (J6delta < 0):
    J6jogNeg(abs(J6delta))
  else:
    J6jogPos(abs(J6delta))       
J6jogslide.config(command=J6sliderUpdate)
J6jogslide.bind("<ButtonRelease-1>", J6sliderExecute)

# Cartesian jog controls
CartjogFrame = LabelFrame(rightPanel, text="Cartesian Control (X Y Z Rz Ry Rx)", padding=5)
CartjogFrame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

CartjogFrame.grid_columnconfigure(0, weight=1)
CartjogFrame.grid_columnconfigure(1, weight=1)
CartjogFrame.grid_columnconfigure(2, weight=1)
CartjogFrame.grid_columnconfigure(3, weight=1)
CartjogFrame.grid_columnconfigure(4, weight=1)
CartjogFrame.grid_columnconfigure(5, weight=1)


# Helper function for tool frame controls (no entry field)
def create_tool_control(parent, row, col, label_text):
    """Create a tool jog control column with horizontal buttons (no entry field)"""
    Label(parent, font=("Arial", 14), text=label_text).grid(row=row, column=col, pady=2)
    
    # Create frame for horizontal button layout
    button_frame = Frame(parent)
    button_frame.grid(row=row+1, column=col, pady=2)
    
    neg_but = Button(button_frame, text="-", width=3)
    neg_but.grid(row=0, column=0, padx=1)
    
    pos_but = Button(button_frame, text="+", width=3)
    pos_but.grid(row=0, column=1, padx=1)
    
    return neg_but, pos_but

# Helper function for cartesian controls
def create_cart_control(parent, row, col, label_text):
    """Create a cartesian jog control column with horizontal buttons"""
    Label(parent, font=("Arial", 14), text=label_text).grid(row=row, column=col, pady=2)
    
    entry = Entry(parent, width=6, justify="center")
    entry.grid(row=row+1, column=col, pady=2)
    
    # Create frame for horizontal button layout
    button_frame = Frame(parent)
    button_frame.grid(row=row+2, column=col, pady=2)
    
    neg_but = Button(button_frame, text="-", width=3)
    neg_but.grid(row=0, column=0, padx=1)
    
    pos_but = Button(button_frame, text="+", width=3)
    pos_but.grid(row=0, column=1, padx=1)
    
    return entry, neg_but, pos_but

# Create cartesian controls
XcurEntryField, XjogNegBut, XjogPosBut = create_cart_control(CartjogFrame, 0, 0, "X")
YcurEntryField, YjogNegBut, YjogPosBut = create_cart_control(CartjogFrame, 0, 1, "Y")
ZcurEntryField, ZjogNegBut, ZjogPosBut = create_cart_control(CartjogFrame, 0, 2, "Z")
RzcurEntryField, RzjogNegBut, RzjogPosBut = create_cart_control(CartjogFrame, 0, 3, "Rz")
RycurEntryField, RyjogNegBut, RyjogPosBut = create_cart_control(CartjogFrame, 0, 4, "Ry")
RxcurEntryField, RxjogNegBut, RxjogPosBut = create_cart_control(CartjogFrame, 0, 5, "Rx")

# Bind cartesian button events
def SelXjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    XjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(10)  
XjogNegBut.bind("<ButtonPress>", SelXjogNeg)
XjogNegBut.bind("<ButtonRelease>", StopJog)

def SelXjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    XjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(11)  
XjogPosBut.bind("<ButtonPress>", SelXjogPos)
XjogPosBut.bind("<ButtonRelease>", StopJog)

def SelYjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    YjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(20)  
YjogNegBut.bind("<ButtonPress>", SelYjogNeg)
YjogNegBut.bind("<ButtonRelease>", StopJog)

def SelYjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    YjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(21)  
YjogPosBut.bind("<ButtonPress>", SelYjogPos)
YjogPosBut.bind("<ButtonRelease>", StopJog)

def SelZjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    ZjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(30)  
ZjogNegBut.bind("<ButtonPress>", SelZjogNeg)
ZjogNegBut.bind("<ButtonRelease>", StopJog)

def SelZjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    ZjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(31)  
ZjogPosBut.bind("<ButtonPress>", SelZjogPos)
ZjogPosBut.bind("<ButtonRelease>", StopJog)

def SelRzjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RzjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(40)  
RzjogNegBut.bind("<ButtonPress>", SelRzjogNeg)
RzjogNegBut.bind("<ButtonRelease>", StopJog)

def SelRzjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RzjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(41)  
RzjogPosBut.bind("<ButtonPress>", SelRzjogPos)
RzjogPosBut.bind("<ButtonRelease>", StopJog)

def SelRyjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RyjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(50)  
RyjogNegBut.bind("<ButtonPress>", SelRyjogNeg)
RyjogNegBut.bind("<ButtonRelease>", StopJog)

def SelRyjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RyjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(51)  
RyjogPosBut.bind("<ButtonPress>", SelRyjogPos)
RyjogPosBut.bind("<ButtonRelease>", StopJog)

def SelRxjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RxjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(60)  
RxjogNegBut.bind("<ButtonPress>", SelRxjogNeg)
RxjogNegBut.bind("<ButtonRelease>", StopJog)

def SelRxjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RxjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(61)  
RxjogPosBut.bind("<ButtonPress>", SelRxjogPos)
RxjogPosBut.bind("<ButtonRelease>", StopJog)

# Tool frame controls
TooljogFrame = LabelFrame(rightPanel, text="Tool Frame Control (Tx Ty Tz Trz Try Trx)", padding=5)
TooljogFrame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

TooljogFrame.grid_columnconfigure(0, weight=1)
TooljogFrame.grid_columnconfigure(1, weight=1)
TooljogFrame.grid_columnconfigure(2, weight=1)
TooljogFrame.grid_columnconfigure(3, weight=1)
TooljogFrame.grid_columnconfigure(4, weight=1)
TooljogFrame.grid_columnconfigure(5, weight=1)

# Helper function for tool frame controls (buttons only, no entry fields)
def create_tool_control(parent, col, label_text):
    """Create a tool frame jog control with label and horizontal buttons (no entry field)"""
    Label(parent, font=("Arial", 14), text=label_text).grid(row=0, column=col, pady=2)
    
    # Create frame for horizontal button layout
    button_frame = Frame(parent)
    button_frame.grid(row=1, column=col, pady=2)
    
    neg_but = Button(button_frame, text="-", width=3)
    neg_but.grid(row=0, column=0, padx=1)
    
    pos_but = Button(button_frame, text="+", width=3)
    pos_but.grid(row=0, column=1, padx=1)
    
    return neg_but, pos_but

# Create tool frame controls (no entry fields)
TXjogNegBut, TXjogPosBut = create_tool_control(TooljogFrame, 0, "Tx")
TYjogNegBut, TYjogPosBut = create_tool_control(TooljogFrame, 1, "Ty")
TZjogNegBut, TZjogPosBut = create_tool_control(TooljogFrame, 2, "Tz")
TRzjogNegBut, TRzjogPosBut = create_tool_control(TooljogFrame, 3, "Trz")
TRyjogNegBut, TRyjogPosBut = create_tool_control(TooljogFrame, 4, "Try")
TRxjogNegBut, TRxjogPosBut = create_tool_control(TooljogFrame, 5, "Trx")

# Bind tool frame button events
def SelTXjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TXjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(10)  
TXjogNegBut.bind("<ButtonPress>", SelTXjogNeg)
TXjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTXjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TXjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(11)  
TXjogPosBut.bind("<ButtonPress>", SelTXjogPos)
TXjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTYjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TYjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(20)  
TYjogNegBut.bind("<ButtonPress>", SelTYjogNeg)
TYjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTYjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TYjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(21)  
TYjogPosBut.bind("<ButtonPress>", SelTYjogPos)
TYjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTZjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TZjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(30)  
TZjogNegBut.bind("<ButtonPress>", SelTZjogNeg)
TZjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTZjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TZjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(31)  
TZjogPosBut.bind("<ButtonPress>", SelTZjogPos)
TZjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTRzjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRzjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(40)  
TRzjogNegBut.bind("<ButtonPress>", SelTRzjogNeg)
TRzjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTRzjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRzjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(41)  
TRzjogPosBut.bind("<ButtonPress>", SelTRzjogPos)
TRzjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTRyjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRyjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(50)  
TRyjogNegBut.bind("<ButtonPress>", SelTRyjogNeg)
TRyjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTRyjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRyjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(51)  
TRyjogPosBut.bind("<ButtonPress>", SelTRyjogPos)
TRyjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTRxjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRxjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(60)  
TRxjogNegBut.bind("<ButtonPress>", SelTRxjogNeg)
TRxjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTRxjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRxjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(61)  
TRxjogPosBut.bind("<ButtonPress>", SelTRxjogPos)
TRxjogPosBut.bind("<ButtonRelease>", StopJog)


# Extra axes (J7, J8, J9)
extraAxesFrame = LabelFrame(rightPanel, text="Additional Axes", padding=5)
extraAxesFrame.grid(row=7, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

extraAxesFrame.grid_columnconfigure(0, weight=1)
extraAxesFrame.grid_columnconfigure(1, weight=1)
extraAxesFrame.grid_columnconfigure(2, weight=1)

# J7 Frame
J7jogFrame = Frame(extraAxesFrame, relief="raised", borderwidth=1, padding=5)
J7jogFrame.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)

J7jogFrame.grid_columnconfigure(0, weight=1)
J7jogFrame.grid_columnconfigure(1, weight=0)
J7jogFrame.grid_columnconfigure(2, weight=1)

J7Lab = Label(J7jogFrame, font=("Arial", 12), text="7th Axis")
J7Lab.grid(row=0, column=0, columnspan=3, pady=2)

J7negLimLab = Label(J7jogFrame, font=("Arial", 8), text="0.00", style="Jointlim.TLabel")
J7negLimLab.grid(row=1, column=0, sticky="w", padx=2)
J7curAngEntryField = Entry(J7jogFrame, width=8, justify="center")
J7curAngEntryField.grid(row=1, column=1, padx=5)
J7posLimLab = Label(J7jogFrame, font=("Arial", 8), text="0", style="Jointlim.TLabel")
J7posLimLab.grid(row=1, column=2, sticky="e", padx=2)

J7jogslide = Scale(J7jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120)
J7jogslide.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

# Create frame for button layout (- entry +)
J7buttonFrame = Frame(J7jogFrame)
J7buttonFrame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

J7buttonFrame.grid_columnconfigure(0, weight=1)
J7buttonFrame.grid_columnconfigure(1, weight=0)
J7buttonFrame.grid_columnconfigure(2, weight=1)

J7jogNegBut = Button(J7buttonFrame, text="-", width=3)
J7jogNegBut.grid(row=0, column=0, sticky="w", padx=(0, 2))

J7slideLimLab = Entry(J7buttonFrame, width=8, justify="center", state="readonly")
J7slideLimLab.grid(row=0, column=1, padx=2)

J7jogPosBut = Button(J7buttonFrame, text="+", width=3)
J7jogPosBut.grid(row=0, column=2, sticky="e", padx=(2, 0))

# Bind J7 events
def SelJ7jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J7jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(70) 
J7jogNegBut.bind("<ButtonPress>", SelJ7jogNeg)
J7jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ7jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J7jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(71)  
J7jogPosBut.bind("<ButtonPress>", SelJ7jogPos)
J7jogPosBut.bind("<ButtonRelease>", StopJog)

def J7sliderUpdate(foo):
  J7slideLimLab.config(state="normal")
  J7slideLimLab.delete(0, END)
  J7slideLimLab.insert(0, round(float(J7jogslide.get()),2))
  J7slideLimLab.config(state="readonly")   
def J7sliderExecute(foo): 
  J7delta = float(J7jogslide.get()) - float(J7curAngEntryField.get())
  if (J7delta < 0):
    J7jogNeg(abs(J7delta))
  else:
    J7jogPos(abs(J7delta))       
J7jogslide.config(command=J7sliderUpdate)
J7jogslide.bind("<ButtonRelease-1>", J7sliderExecute)

# J8 Frame
J8jogFrame = Frame(extraAxesFrame, relief="raised", borderwidth=1, padding=5)
J8jogFrame.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)

J8jogFrame.grid_columnconfigure(0, weight=1)
J8jogFrame.grid_columnconfigure(1, weight=0)
J8jogFrame.grid_columnconfigure(2, weight=1)

J8Lab = Label(J8jogFrame, font=("Arial", 12), text="8th Axis")
J8Lab.grid(row=0, column=0, columnspan=3, pady=2)

J8negLimLab = Label(J8jogFrame, font=("Arial", 8), text="0.00", style="Jointlim.TLabel")
J8negLimLab.grid(row=1, column=0, sticky="w", padx=2)
J8curAngEntryField = Entry(J8jogFrame, width=8, justify="center")
J8curAngEntryField.grid(row=1, column=1, padx=5)
J8posLimLab = Label(J8jogFrame, font=("Arial", 8), text="0", style="Jointlim.TLabel")
J8posLimLab.grid(row=1, column=2, sticky="e", padx=2)

J8jogslide = Scale(J8jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120)
J8jogslide.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

# Create frame for button layout (- entry +)
J8buttonFrame = Frame(J8jogFrame)
J8buttonFrame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

J8buttonFrame.grid_columnconfigure(0, weight=1)
J8buttonFrame.grid_columnconfigure(1, weight=0)
J8buttonFrame.grid_columnconfigure(2, weight=1)

J8jogNegBut = Button(J8buttonFrame, text="-", width=3)
J8jogNegBut.grid(row=0, column=0, sticky="w", padx=(0, 2))

J8slideLimLab = Entry(J8buttonFrame, width=8, justify="center", state="readonly")
J8slideLimLab.grid(row=0, column=1, padx=2)

J8jogPosBut = Button(J8buttonFrame, text="+", width=3)
J8jogPosBut.grid(row=0, column=2, sticky="e", padx=(2, 0))

# Bind J8 events
def SelJ8jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J8jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(80) 
J8jogNegBut.bind("<ButtonPress>", SelJ8jogNeg)
J8jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ8jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J8jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(81)  
J8jogPosBut.bind("<ButtonPress>", SelJ8jogPos)
J8jogPosBut.bind("<ButtonRelease>", StopJog)

def J8sliderUpdate(foo):
  J8slideLimLab.config(state="normal")
  J8slideLimLab.delete(0, END)
  J8slideLimLab.insert(0, round(float(J8jogslide.get()),2))
  J8slideLimLab.config(state="readonly")   
def J8sliderExecute(foo): 
  J8delta = float(J8jogslide.get()) - float(J8curAngEntryField.get())
  if (J8delta < 0):
    J8jogNeg(abs(J8delta))
  else:
    J8jogPos(abs(J8delta))       
J8jogslide.config(command=J8sliderUpdate)
J8jogslide.bind("<ButtonRelease-1>", J8sliderExecute)

# J9 Frame
J9jogFrame = Frame(extraAxesFrame, relief="raised", borderwidth=1, padding=5)
J9jogFrame.grid(row=0, column=2, sticky="nsew", padx=2, pady=2)

J9jogFrame.grid_columnconfigure(0, weight=1)
J9jogFrame.grid_columnconfigure(1, weight=0)
J9jogFrame.grid_columnconfigure(2, weight=1)

J9Lab = Label(J9jogFrame, font=("Arial", 12), text="9th Axis")
J9Lab.grid(row=0, column=0, columnspan=3, pady=2)

J9negLimLab = Label(J9jogFrame, font=("Arial", 8), text="0.00", style="Jointlim.TLabel")
J9negLimLab.grid(row=1, column=0, sticky="w", padx=2)
J9curAngEntryField = Entry(J9jogFrame, width=8, justify="center")
J9curAngEntryField.grid(row=1, column=1, padx=5)
J9posLimLab = Label(J9jogFrame, font=("Arial", 8), text="0", style="Jointlim.TLabel")
J9posLimLab.grid(row=1, column=2, sticky="e", padx=2)

J9jogslide = Scale(J9jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120)
J9jogslide.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

# Create frame for button layout (- entry +)
J9buttonFrame = Frame(J9jogFrame)
J9buttonFrame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

J9buttonFrame.grid_columnconfigure(0, weight=1)
J9buttonFrame.grid_columnconfigure(1, weight=0)
J9buttonFrame.grid_columnconfigure(2, weight=1)

J9jogNegBut = Button(J9buttonFrame, text="-", width=3)
J9jogNegBut.grid(row=0, column=0, sticky="w", padx=(0, 2))

J9slideLimLab = Entry(J9buttonFrame, width=8, justify="center", state="readonly")
J9slideLimLab.grid(row=0, column=1, padx=2)

J9jogPosBut = Button(J9buttonFrame, text="+", width=3)
J9jogPosBut.grid(row=0, column=2, sticky="e", padx=(2, 0))

# Bind J9 events
def SelJ9jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J9jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(90) 
J9jogNegBut.bind("<ButtonPress>", SelJ9jogNeg)
J9jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ9jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J9jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(91)  
J9jogPosBut.bind("<ButtonPress>", SelJ9jogPos)
J9jogPosBut.bind("<ButtonRelease>", StopJog)

def J9sliderUpdate(foo):
  J9slideLimLab.config(state="normal")
  J9slideLimLab.delete(0, END)
  J9slideLimLab.insert(0, round(float(J9jogslide.get()),2))
  J9slideLimLab.config(state="readonly")   
def J9sliderExecute(foo): 
  J9delta = float(J9jogslide.get()) - float(J9curAngEntryField.get())
  if (J9delta < 0):
    J9jogNeg(abs(J9delta))
  else:
    J9jogPos(abs(J9delta))       
J9jogslide.config(command=J9sliderUpdate)
J9jogslide.bind("<ButtonRelease-1>", J9sliderExecute)

# Command builders (IF, SET, WAIT - reordered and aligned)
cmdFrame = LabelFrame(rightPanel, text="Command Builders", padding=5)
cmdFrame.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=(5, 2))

# Configure columns for proper alignment
cmdFrame.grid_columnconfigure(0, weight=0, minsize=45)   # Label
cmdFrame.grid_columnconfigure(1, weight=0, minsize=130)  # Type dropdown
cmdFrame.grid_columnconfigure(2, weight=0, minsize=60)   # Var entry
cmdFrame.grid_columnconfigure(3, weight=0, minsize=25)   # "="
cmdFrame.grid_columnconfigure(4, weight=0, minsize=60)   # Value entry
cmdFrame.grid_columnconfigure(5, weight=0, minsize=100)  # Action dropdown (IF only)
cmdFrame.grid_columnconfigure(6, weight=0, minsize=80)   # Dest entry / Timeout entry
cmdFrame.grid_columnconfigure(7, weight=0, minsize=25)   # "•"
cmdFrame.grid_columnconfigure(8, weight=1, minsize=120)  # Insert button

# Create StringVars for OptionMenus
iFoption = StringVar(cmdFrame)
iFoption.set("5v Input")
iFselection = StringVar(cmdFrame)
iFselection.set("Call Prog")
waitoption = StringVar(cmdFrame)
waitoption.set("5v Input")
setoption = StringVar(cmdFrame)
setoption.set("5v Output")

# Row 0: IF command - IF [Type] [Var#] = [Value] [Action] [Dest] • [Insert]
Label(cmdFrame, text="IF", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=(2, 5), pady=2)

iFmenu = OptionMenu(cmdFrame, iFoption, "5v Input", "5v Input", "Register", "COM Device", "MB Coil", "MB Input", "MB Hold Reg", "MB Input Reg")
iFmenu.config(width=12)
iFmenu.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

IfVarEntryField = Entry(cmdFrame, width=6, justify="center")
IfVarEntryField.grid(row=0, column=2, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="=").grid(row=0, column=3, padx=2)

IfInputEntryField = Entry(cmdFrame, width=6, justify="center")
IfInputEntryField.grid(row=0, column=4, sticky="ew", padx=2, pady=2)

iFSelmenu = OptionMenu(cmdFrame, iFselection, "Call Prog", "Call Prog", "Jump Tab", "Stop")
iFSelmenu.config(width=9)
iFSelmenu.grid(row=0, column=5, sticky="ew", padx=2, pady=2)

IfDestEntryField = Entry(cmdFrame, width=8, justify="center")
IfDestEntryField.grid(row=0, column=6, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="•").grid(row=0, column=7, padx=2)

insertIFBut = ttk.Button(cmdFrame, text="Insert IF CMD", command=IfCMDInsert)
insertIFBut.grid(row=0, column=8, sticky="ew", padx=2, pady=2)

# Row 1: SET command - SET [Type] [Var#] = [Value] • [Insert]
Label(cmdFrame, text="SET", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="w", padx=(2, 5), pady=2)

setmenu = OptionMenu(cmdFrame, setoption, "5v Output", "5v Output", "MB Coil", "MB Register")
setmenu.config(width=12)
setmenu.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

setVarEntryField = Entry(cmdFrame, width=6, justify="center")
setVarEntryField.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="=").grid(row=1, column=3, padx=2)

setInputEntryField = Entry(cmdFrame, width=6, justify="center")
setInputEntryField.grid(row=1, column=4, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="•").grid(row=1, column=7, padx=2)

insertSetBut = ttk.Button(cmdFrame, text="Insert set CMD", command=SetCMDInsert)
insertSetBut.grid(row=1, column=8, sticky="ew", padx=2, pady=2)

# Row 2: WAIT command - WAIT [Type] [Var#] = [Value] Timeout = [Time] • [Insert]
Label(cmdFrame, text="WAIT", font=("Arial", 10, "bold")).grid(row=2, column=0, sticky="w", padx=(2, 5), pady=2)

waitmenu = OptionMenu(cmdFrame, waitoption, "5v Input", "5v Input", "MB Coil", "MB Input")
waitmenu.config(width=12)
waitmenu.grid(row=2, column=1, sticky="ew", padx=2, pady=2)

waitVarEntryField = Entry(cmdFrame, width=6, justify="center")
waitVarEntryField.grid(row=2, column=2, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="=").grid(row=2, column=3, padx=2)

waitInputEntryField = Entry(cmdFrame, width=6, justify="center")
waitInputEntryField.grid(row=2, column=4, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="Timeout =").grid(row=2, column=5, sticky="e", padx=2)

waitTimeoutEntryField = Entry(cmdFrame, width=6, justify="center")
waitTimeoutEntryField.grid(row=2, column=6, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="•").grid(row=2, column=7, padx=2)

insertWaitBut = ttk.Button(cmdFrame, text="Insert WAIT CMD", command=WaitCMDInsert)
insertWaitBut.grid(row=2, column=8, sticky="ew", padx=2, pady=2)

# Navigation container (2x2 grid layout)
navFrame = LabelFrame(rightPanel, text="Navigation", padding=5)
navFrame.grid(row=4, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

# Configure 4 columns for 2x2 grid (button, entry, button, entry)
navFrame.grid_columnconfigure(0, weight=1, minsize=100)  # Button 1
navFrame.grid_columnconfigure(1, weight=1, minsize=80)   # Entry 1
navFrame.grid_columnconfigure(2, weight=1, minsize=100)  # Button 2
navFrame.grid_columnconfigure(3, weight=1, minsize=80)   # Entry 2

# Row 0: Create Tab | Call Program
createTabBut = ttk.Button(navFrame, text="Create Tab", command=tabNumber)
createTabBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

tabNumEntryField = Entry(navFrame, width=8, justify="center")
tabNumEntryField.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

callProgBut = ttk.Button(navFrame, text="Call Program", command=insertCallProg)
callProgBut.grid(row=0, column=2, sticky="ew", padx=2, pady=2)

changeProgEntryField = Entry(navFrame, width=8, justify="center")
changeProgEntryField.grid(row=0, column=3, sticky="ew", padx=2, pady=2)

# Row 1: Jump to Tab | Play Gcode
jumpTabBut = ttk.Button(navFrame, text="Jump to Tab", command=jumpTab)
jumpTabBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

jumpTabEntryField = Entry(navFrame, width=8, justify="center")
jumpTabEntryField.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

playGcodeBut = ttk.Button(navFrame, text="Play Gcode", command=insertGCprog)
playGcodeBut.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

PlayGCEntryField = Entry(navFrame, width=8, justify="center")
PlayGCEntryField.grid(row=1, column=3, sticky="ew", padx=2, pady=2)

# Register Commands container
regFrame = LabelFrame(rightPanel, text="Register Commands", padding=5)
regFrame.grid(row=5, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

# Configure columns for side-by-side layout
regFrame.grid_columnconfigure(0, weight=1, minsize=120)  # Register button
regFrame.grid_columnconfigure(1, weight=0, minsize=60)   # Register entry
regFrame.grid_columnconfigure(2, weight=0, minsize=60)   # (++/--) entry
regFrame.grid_columnconfigure(3, weight=1, minsize=140)  # Position Register button
regFrame.grid_columnconfigure(4, weight=0, minsize=60)   # Pos Reg entry
regFrame.grid_columnconfigure(5, weight=0, minsize=60)   # Element entry
regFrame.grid_columnconfigure(6, weight=0, minsize=60)   # (++/--) entry

# Row 0: Labels
Label(regFrame, text="Register", font=("Arial", 8)).grid(row=0, column=1, sticky="w", padx=2)
Label(regFrame, text="(++/--)", font=("Arial", 8)).grid(row=0, column=2, sticky="w", padx=2)
Label(regFrame, text="Pos Reg", font=("Arial", 8)).grid(row=0, column=4, sticky="w", padx=2)
Label(regFrame, text="Element", font=("Arial", 8)).grid(row=0, column=5, sticky="w", padx=2)
Label(regFrame, text="(++/--)", font=("Arial", 8)).grid(row=0, column=6, sticky="w", padx=2)

# Row 1: Buttons and entry fields
RegNumBut = ttk.Button(regFrame, text="Register", command=insertRegister)
RegNumBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

regNumEntryField = Entry(regFrame, width=6, justify="center")
regNumEntryField.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

regEqEntryField = Entry(regFrame, width=6, justify="center")
regEqEntryField.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

StorPosBut = ttk.Button(regFrame, text="Position Register", command=storPos)
StorPosBut.grid(row=1, column=3, sticky="ew", padx=(10, 2), pady=2)

storPosNumEntryField = Entry(regFrame, width=6, justify="center")
storPosNumEntryField.grid(row=1, column=4, sticky="ew", padx=2, pady=2)

storPosElEntryField = Entry(regFrame, width=6, justify="center")
storPosElEntryField.grid(row=1, column=5, sticky="ew", padx=2, pady=2)

storPosValEntryField = Entry(regFrame, width=6, justify="center")
storPosValEntryField.grid(row=1, column=6, sticky="ew", padx=2, pady=2)

# Aliases for compatibility
posRegBut = StorPosBut
posRegEntryField = storPosNumEntryField

# Device Commands container
devFrame = LabelFrame(rightPanel, text="Device Commands", padding=5)
devFrame.grid(row=6, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

# Configure columns
devFrame.grid_columnconfigure(0, weight=1, minsize=100)  # Servo button
devFrame.grid_columnconfigure(1, weight=0, minsize=60)   # Number entry
devFrame.grid_columnconfigure(2, weight=0, minsize=60)   # Position entry
devFrame.grid_columnconfigure(3, weight=1, minsize=140)  # Read COM button
devFrame.grid_columnconfigure(4, weight=0, minsize=60)   # Port entry
devFrame.grid_columnconfigure(5, weight=0, minsize=60)   # Char entry

# Row 0: Labels
Label(devFrame, text="Number", font=("Arial", 8)).grid(row=0, column=1, sticky="w", padx=2)
Label(devFrame, text="Position", font=("Arial", 8)).grid(row=0, column=2, sticky="w", padx=2)
Label(devFrame, text="Port", font=("Arial", 8)).grid(row=0, column=4, sticky="w", padx=2)
Label(devFrame, text="Char", font=("Arial", 8)).grid(row=0, column=5, sticky="w", padx=2)

# Row 1: Buttons and entry fields
servoBut = ttk.Button(devFrame, text="Servo", command=Servo)
servoBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

servoNumEntryField = Entry(devFrame, width=6, justify="center")
servoNumEntryField.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

servoPosEntryField = Entry(devFrame, width=6, justify="center")
servoPosEntryField.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

readCOMBut = ttk.Button(devFrame, text="Read COM Device", command=ReadAuxCom)
readCOMBut.grid(row=1, column=3, sticky="ew", padx=(10, 2), pady=2)

auxPortEntryField = Entry(devFrame, width=6, justify="center")
auxPortEntryField.grid(row=1, column=4, sticky="ew", padx=2, pady=2)

auxCharEntryField = Entry(devFrame, width=6, justify="center")
auxCharEntryField.grid(row=1, column=5, sticky="ew", padx=2, pady=2)

##########################################################################
### END OF TAB 1 REFACTORING







####TAB 2

# Configure tab2 main grid
tab2.grid_rowconfigure(0, weight=0)  # Status bar
tab2.grid_rowconfigure(1, weight=1)  # Main content (expands)
tab2.grid_rowconfigure(2, weight=0)  # Commands and Save row

tab2.grid_columnconfigure(0, weight=0, minsize=180)  # Communication
tab2.grid_columnconfigure(1, weight=0, minsize=180)  # Robot Calibration
tab2.grid_columnconfigure(2, weight=0, minsize=150)  # Calibration Offsets
tab2.grid_columnconfigure(3, weight=0, minsize=150)  # Encoder Control
tab2.grid_columnconfigure(4, weight=0, minsize=180)  # External Axes
tab2.grid_columnconfigure(5, weight=0, minsize=150)  # Theme
tab2.grid_columnconfigure(6, weight=0, minsize=180)  # Virtual Import
tab2.grid_columnconfigure(7, weight=0, minsize=120)  # Save
tab2.grid_columnconfigure(8, weight=1)  # Spacer (expands)

# ============================================================================
# ROW 0: Status/Alarm Label (spans all columns)
# ============================================================================
almStatusLab2 = Label(tab2, text="SYSTEM STARTING - PLEASE WAIT", style="OK.TLabel")
almStatusLab2.grid(row=0, column=0, columnspan=9, sticky="w", padx=25, pady=20)

# ============================================================================
# ROW 1, COLUMN 0: Communication Frame
# ============================================================================
commFrame = LabelFrame(tab2, text="Communication", padding=10)
commFrame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

commFrame.grid_columnconfigure(0, weight=1)

# COM port detection function (from original)
def detect_ports():
  from serial.tools import list_ports
  ports = list(list_ports.comports())
  choices = [p.device for p in ports]
  choices.insert(0, "None")
  if globals().get('CAL', {}).get('comPort') not in ("", None, "None"):
    port1_default = CAL['comPort']
  else:
    port1_default = "None"
  
  if globals().get('CAL', {}).get('com2ort') not in ("", None, "None"):
    port2_default = CAL['com2Port']
  else:
    port2_default = "None"
  
  return choices, port1_default, port2_default

port_choices, default_comport1, default_comport2 = detect_ports()
logger.debug(f"Available Comm Ports: {port_choices}")

# Teensy COM Port
ComPortLab = Label(commFrame, text="TEENSY COM PORT:")
ComPortLab.grid(row=0, column=0, sticky="w", padx=5, pady=(5, 2))

com1SelectedValue = tk.StringVar(value=default_comport1 or "None")
com1Select = tk.OptionMenu(commFrame, com1SelectedValue, *port_choices, command=setCom)
com1Select.grid(row=1, column=0, sticky="ew", padx=5, pady=2)

# 5v IO Board COM Port
ComPortLab2 = Label(commFrame, text="5v IO BOARD COM PORT:")
ComPortLab2.grid(row=2, column=0, sticky="w", padx=5, pady=(15, 2))

com2SelectedValue = tk.StringVar(value=default_comport2 or "None")
com2Select = tk.OptionMenu(commFrame, com2SelectedValue, *port_choices, command=setCom2)
com2Select.grid(row=3, column=0, sticky="ew", padx=5, pady=2)

# ============================================================================
# ROW 1, COLUMN 1: Robot Calibration Frame
# ============================================================================
calFrame = LabelFrame(tab2, text="Robot Calibration", padding=10)
calFrame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)

calFrame.grid_columnconfigure(0, weight=1)

# Auto Calibrate button
autoCalBut = Button(calFrame, text="  Auto Calibrate  ", command=calRobotAll)
autoCalBut.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

# First set of checkboxes (J1-J9)
checkFrame1 = Frame(calFrame)
checkFrame1.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

checkFrame1.grid_columnconfigure(0, weight=1)
checkFrame1.grid_columnconfigure(1, weight=1)
checkFrame1.grid_columnconfigure(2, weight=1)

J1calCbut = Checkbutton(checkFrame1, text="J1", variable=CAL['J1CalStatVal'])
J1calCbut.grid(row=0, column=0, sticky="w", padx=2)

J2calCbut = Checkbutton(checkFrame1, text="J2", variable=CAL['J2CalStatVal'])
J2calCbut.grid(row=0, column=1, sticky="w", padx=2)

J3calCbut = Checkbutton(checkFrame1, text="J3", variable=CAL['J3CalStatVal'])
J3calCbut.grid(row=0, column=2, sticky="w", padx=2)

J4calCbut = Checkbutton(checkFrame1, text="J4", variable=CAL['J4CalStatVal'])
J4calCbut.grid(row=1, column=0, sticky="w", padx=2)

J5calCbut = Checkbutton(checkFrame1, text="J5", variable=CAL['J5CalStatVal'])
J5calCbut.grid(row=1, column=1, sticky="w", padx=2)

J6calCbut = Checkbutton(checkFrame1, text="J6", variable=CAL['J6CalStatVal'])
J6calCbut.grid(row=1, column=2, sticky="w", padx=2)

J7calCbut = Checkbutton(checkFrame1, text="J7", variable=CAL['J7CalStatVal'])
J7calCbut.grid(row=2, column=0, sticky="w", padx=2)

J8calCbut = Checkbutton(checkFrame1, text="J8", variable=CAL['J8CalStatVal'])
J8calCbut.grid(row=2, column=1, sticky="w", padx=2)

J9calCbut = Checkbutton(checkFrame1, text="J9", variable=CAL['J9CalStatVal'])
J9calCbut.grid(row=2, column=2, sticky="w", padx=2)

# Second set of checkboxes (J1-J9)
checkFrame2 = Frame(calFrame)
checkFrame2.grid(row=2, column=0, sticky="ew", padx=5, pady=(10, 5))

checkFrame2.grid_columnconfigure(0, weight=1)
checkFrame2.grid_columnconfigure(1, weight=1)
checkFrame2.grid_columnconfigure(2, weight=1)

J1calCbut2 = Checkbutton(checkFrame2, text="J1", variable=CAL['J1CalStatVal2'])
J1calCbut2.grid(row=0, column=0, sticky="w", padx=2)

J2calCbut2 = Checkbutton(checkFrame2, text="J2", variable=CAL['J2CalStatVal2'])
J2calCbut2.grid(row=0, column=1, sticky="w", padx=2)

J3calCbut2 = Checkbutton(checkFrame2, text="J3", variable=CAL['J3CalStatVal2'])
J3calCbut2.grid(row=0, column=2, sticky="w", padx=2)

J4calCbut2 = Checkbutton(checkFrame2, text="J4", variable=CAL['J4CalStatVal2'])
J4calCbut2.grid(row=1, column=0, sticky="w", padx=2)

J5calCbut2 = Checkbutton(checkFrame2, text="J5", variable=CAL['J5CalStatVal2'])
J5calCbut2.grid(row=1, column=1, sticky="w", padx=2)

J6calCbut2 = Checkbutton(checkFrame2, text="J6", variable=CAL['J6CalStatVal2'])
J6calCbut2.grid(row=1, column=2, sticky="w", padx=2)

J7calCbut2 = Checkbutton(checkFrame2, text="J7", variable=CAL['J7CalStatVal2'])
J7calCbut2.grid(row=2, column=0, sticky="w", padx=2)

J8calCbut2 = Checkbutton(checkFrame2, text="J8", variable=CAL['J8CalStatVal2'])
J8calCbut2.grid(row=2, column=1, sticky="w", padx=2)

J9calCbut2 = Checkbutton(checkFrame2, text="J9", variable=CAL['J9CalStatVal2'])
J9calCbut2.grid(row=2, column=2, sticky="w", padx=2)

# Individual calibration buttons (EXACT names from original)
CalJ1But = Button(calFrame, text="Calibrate J1 Only", command=calRobotJ1)
CalJ1But.grid(row=3, column=0, sticky="ew", padx=5, pady=2)

CalJ2But = Button(calFrame, text="Calibrate J2 Only", command=calRobotJ2)
CalJ2But.grid(row=4, column=0, sticky="ew", padx=5, pady=2)

CalJ3But = Button(calFrame, text="Calibrate J3 Only", command=calRobotJ3)
CalJ3But.grid(row=5, column=0, sticky="ew", padx=5, pady=2)

CalJ4But = Button(calFrame, text="Calibrate J4 Only", command=calRobotJ4)
CalJ4But.grid(row=6, column=0, sticky="ew", padx=5, pady=2)

CalJ5But = Button(calFrame, text="Calibrate J5 Only", command=calRobotJ5)
CalJ5But.grid(row=7, column=0, sticky="ew", padx=5, pady=2)

CalJ6But = Button(calFrame, text="Calibrate J6 Only", command=calRobotJ6)
CalJ6But.grid(row=8, column=0, sticky="ew", padx=5, pady=(5,20))

ForceCalHomeBut = Button(calFrame, text="Force Cal Home", command=CalZeroPos)
ForceCalHomeBut.grid(row=9, column=0, sticky="ew", padx=5, pady=2)

ForceCalHomeBut = Button(calFrame, text="Force Cal Rest", command=CalRestPos)
ForceCalHomeBut.grid(row=10, column=0, sticky="ew", padx=5, pady=2)


# ============================================================================
# ROW 1, COLUMN 2: Calibration Offsets Frame
# ============================================================================
calOffsetFrame = LabelFrame(tab2, text="Calibration Offsets", padding=10)
calOffsetFrame.grid(row=1, column=2, sticky="nsew", padx=5, pady=5)

calOffsetFrame.grid_columnconfigure(0, weight=1)
calOffsetFrame.grid_columnconfigure(1, weight=1)

# J1 Offset
J1calLab = Label(calOffsetFrame, text="J1 Offset")
J1calLab.grid(row=0, column=0, sticky="e", padx=2, pady=2)
J1calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J1calOffEntryField.grid(row=0, column=1, sticky="w", padx=2, pady=2)

# J2 Offset
J2calLab = Label(calOffsetFrame, text="J2 Offset")
J2calLab.grid(row=1, column=0, sticky="e", padx=2, pady=2)
J2calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J2calOffEntryField.grid(row=1, column=1, sticky="w", padx=2, pady=2)

# J3 Offset
J3calLab = Label(calOffsetFrame, text="J3 Offset")
J3calLab.grid(row=2, column=0, sticky="e", padx=2, pady=2)
J3calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J3calOffEntryField.grid(row=2, column=1, sticky="w", padx=2, pady=2)

# J4 Offset
J4calLab = Label(calOffsetFrame, text="J4 Offset")
J4calLab.grid(row=3, column=0, sticky="e", padx=2, pady=2)
J4calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J4calOffEntryField.grid(row=3, column=1, sticky="w", padx=2, pady=2)

# J5 Offset
J5calLab = Label(calOffsetFrame, text="J5 Offset")
J5calLab.grid(row=4, column=0, sticky="e", padx=2, pady=2)
J5calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J5calOffEntryField.grid(row=4, column=1, sticky="w", padx=2, pady=2)

# J6 Offset
J6calLab = Label(calOffsetFrame, text="J6 Offset")
J6calLab.grid(row=5, column=0, sticky="e", padx=2, pady=2)
J6calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J6calOffEntryField.grid(row=5, column=1, sticky="w", padx=2, pady=2)

# J7 Offset
J7calLab = Label(calOffsetFrame, text="J7 Offset")
J7calLab.grid(row=6, column=0, sticky="e", padx=2, pady=2)
J7calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J7calOffEntryField.grid(row=6, column=1, sticky="w", padx=2, pady=2)

# J8 Offset
J8calLab = Label(calOffsetFrame, text="J8 Offset")
J8calLab.grid(row=7, column=0, sticky="e", padx=2, pady=2)
J8calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J8calOffEntryField.grid(row=7, column=1, sticky="w", padx=2, pady=2)

# J9 Offset
J9calLab = Label(calOffsetFrame, text="J9 Offset")
J9calLab.grid(row=8, column=0, sticky="e", padx=2, pady=2)
J9calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J9calOffEntryField.grid(row=8, column=1, sticky="w", padx=2, pady=2)


# ============================================================================
# ROW 1, COLUMN 3: Encoder Control Frame
# ============================================================================
encoderFrame = LabelFrame(tab2, text="Encoder Control", padding=10)
encoderFrame.grid(row=1, column=3, sticky="nsew", padx=5, pady=5)

encoderFrame.grid_columnconfigure(0, weight=1)

# J1 Open Loop
J1OpenLoopCbut = Checkbutton(encoderFrame, text="J1 Open Loop (disable encoder)", variable=CAL['J1OpenLoopVal'])
J1OpenLoopCbut.grid(row=0, column=0, sticky="w", padx=5, pady=2)

# J2 Open Loop
J2OpenLoopCbut = Checkbutton(encoderFrame, text="J2 Open Loop (disable encoder)", variable=CAL['J2OpenLoopVal'])
J2OpenLoopCbut.grid(row=1, column=0, sticky="w", padx=5, pady=2)

# J3 Open Loop
J3OpenLoopCbut = Checkbutton(encoderFrame, text="J3 Open Loop (disable encoder)", variable=CAL['J3OpenLoopVal'])
J3OpenLoopCbut.grid(row=2, column=0, sticky="w", padx=5, pady=2)

# J4 Open Loop
J4OpenLoopCbut = Checkbutton(encoderFrame, text="J4 Open Loop (disable encoder)", variable=CAL['J4OpenLoopVal'])
J4OpenLoopCbut.grid(row=3, column=0, sticky="w", padx=5, pady=2)

# J5 Open Loop
J5OpenLoopCbut = Checkbutton(encoderFrame, text="J5 Open Loop (disable encoder)", variable=CAL['J5OpenLoopVal'])
J5OpenLoopCbut.grid(row=4, column=0, sticky="w", padx=5, pady=2)

# J6 Open Loop
J6OpenLoopCbut = Checkbutton(encoderFrame, text="J6 Open Loop (disable encoder)", variable=CAL['J6OpenLoopVal'])
J6OpenLoopCbut.grid(row=5, column=0, sticky="w", padx=5, pady=2)



# ============================================================================
# Color Configuration for Robot Visualization
# ============================================================================

main_color_parts = ["Link Base-2.STL", "Link 2-2.STL", "Link 4-2.STL"]
logo_color_parts = ["Link 2-3.STL", "Link 4-3.STL"]

def update_main_color(*args):
    selected = main_color_var.get()
    CAL['setColor'] = selected
    for part in main_color_parts:
        RUN['color_map'][part] = selected 
        RUN['actors'][part].GetProperty().SetColor(vtk.vtkNamedColors().GetColor3d(selected))
    RUN['render_window'].Render()

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

# Initialize color variable
main_color_var = tk.StringVar(value="Royal Blue")


# ============================================================================
# ROW 1, COLUMN 5: Theme Frame
# ============================================================================
themeFrame = LabelFrame(tab2, text="Theme", padding=10)
themeFrame.grid(row=1, column=5, sticky="nsew", padx=5, pady=5)

themeFrame.grid_columnconfigure(0, weight=1)
themeFrame.grid_columnconfigure(1, weight=1)

# Theme buttons
lightBut = Button(themeFrame, text="  Light  ", command=lightTheme)
lightBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

darkBut = Button(themeFrame, text="  Dark   ", command=darkTheme)
darkBut.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

# Robot Color label and dropdown
robotColorLab = Label(themeFrame, text="Robot Color", font=("Arial", 10, "bold"))
robotColorLab.grid(row=1, column=0, columnspan=2, sticky="w", padx=2, pady=(10, 2))

main_color_dropdown = ttk.OptionMenu(themeFrame, main_color_var, main_color_var.get(), *color_options, command=update_main_color)
main_color_dropdown.grid(row=2, column=0, columnspan=2, sticky="ew", padx=2, pady=2)




# ============================================================================
# ROW 1, COLUMN 4: External Axes Frame
# ============================================================================
externalAxesFrame = LabelFrame(tab2, text="External Axes", padding=10)
externalAxesFrame.grid(row=1, column=4, sticky="nsew", padx=5, pady=5)

externalAxesFrame.grid_columnconfigure(0, weight=0)  # Labels
externalAxesFrame.grid_columnconfigure(1, weight=1)  # Entry fields

# --- 7th Axis Calibration ---
axis7Lab = Label(externalAxesFrame, font=("Arial 10 bold"), text="7th Axis Calibration")
axis7Lab.grid(row=0, column=0, columnspan=2, sticky="w", padx=5, pady=(5, 10))

axis7lengthLab = Label(externalAxesFrame, text="7th Axis Length:")
axis7lengthLab.grid(row=1, column=0, sticky="e", padx=5, pady=2)
axis7lengthEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis7lengthEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

axis7rotLab = Label(externalAxesFrame, text="MM per Rotation:")
axis7rotLab.grid(row=2, column=0, sticky="e", padx=5, pady=2)
axis7rotEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis7rotEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

axis7stepsLab = Label(externalAxesFrame, text="Drive Steps:")
axis7stepsLab.grid(row=3, column=0, sticky="e", padx=5, pady=2)
axis7stepsEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis7stepsEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J7zerobut = Button(externalAxesFrame, text="Set Axis 7 Calibration to Zero", width=28, command=zeroAxis7)
J7zerobut.grid(row=4, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

J7calbut = Button(externalAxesFrame, text="Autocalibrate Axis 7", width=28, command=calRobotJ7)
J7calbut.grid(row=5, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

axis7pinsetLab = Label(externalAxesFrame, font=("Arial", 8), text="StepPin = 12 / DirPin = 13 / CalPin = 36")
axis7pinsetLab.grid(row=6, column=0, columnspan=2, sticky="w", padx=5, pady=(2, 15))

# --- 8th Axis Calibration ---
axis8Lab = Label(externalAxesFrame, font=("Arial 10 bold"), text="8th Axis Calibration")
axis8Lab.grid(row=7, column=0, columnspan=2, sticky="w", padx=5, pady=(5, 10))

axis8lengthLab = Label(externalAxesFrame, text="8th Axis Length:")
axis8lengthLab.grid(row=8, column=0, sticky="e", padx=5, pady=2)
axis8lengthEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis8lengthEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)

axis8rotLab = Label(externalAxesFrame, text="MM per Rotation:")
axis8rotLab.grid(row=9, column=0, sticky="e", padx=5, pady=2)
axis8rotEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis8rotEntryField.grid(row=9, column=1, sticky="w", padx=5, pady=2)

axis8stepsLab = Label(externalAxesFrame, text="Drive Steps:")
axis8stepsLab.grid(row=10, column=0, sticky="e", padx=5, pady=2)
axis8stepsEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis8stepsEntryField.grid(row=10, column=1, sticky="w", padx=5, pady=2)

J8zerobut = Button(externalAxesFrame, text="Set Axis 8 Calibration to Zero", width=28, command=zeroAxis8)
J8zerobut.grid(row=11, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

J8calbut = Button(externalAxesFrame, text="Autocalibrate Axis 8", width=28, command=calRobotJ8)
J8calbut.grid(row=12, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

axis8pinsetLab = Label(externalAxesFrame, font=("Arial", 8), text="StepPin = 32 / DirPin = 33 / CalPin = 37")
axis8pinsetLab.grid(row=13, column=0, columnspan=2, sticky="w", padx=5, pady=(2, 15))

# --- 9th Axis Calibration ---
axis9Lab = Label(externalAxesFrame, font=("Arial 10 bold"), text="9th Axis Calibration")
axis9Lab.grid(row=14, column=0, columnspan=2, sticky="w", padx=5, pady=(5, 10))

axis9lengthLab = Label(externalAxesFrame, text="9th Axis Length:")
axis9lengthLab.grid(row=15, column=0, sticky="e", padx=5, pady=2)
axis9lengthEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis9lengthEntryField.grid(row=15, column=1, sticky="w", padx=5, pady=2)

axis9rotLab = Label(externalAxesFrame, text="MM per Rotation:")
axis9rotLab.grid(row=16, column=0, sticky="e", padx=5, pady=2)
axis9rotEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis9rotEntryField.grid(row=16, column=1, sticky="w", padx=5, pady=2)

axis9stepsLab = Label(externalAxesFrame, text="Drive Steps:")
axis9stepsLab.grid(row=17, column=0, sticky="e", padx=5, pady=2)
axis9stepsEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis9stepsEntryField.grid(row=17, column=1, sticky="w", padx=5, pady=2)

J9zerobut = Button(externalAxesFrame, text="Set Axis 9 Calibration to Zero", width=28, command=zeroAxis9)
J9zerobut.grid(row=18, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

J9calbut = Button(externalAxesFrame, text="Autocalibrate Axis 9", width=28, command=calRobotJ9)
J9calbut.grid(row=19, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

axis9pinsetLab = Label(externalAxesFrame, font=("Arial", 8), text="StepPin = 34 / DirPin = 35 / CalPin = 38")
axis9pinsetLab.grid(row=20, column=0, columnspan=2, sticky="w", padx=5, pady=(2, 5))


# ============================================================================
# ROW 1, COLUMN 6: Virtual Import Frame
# ============================================================================
virtualImportFrame = LabelFrame(tab2, text="Virtual Import", padding=10)
virtualImportFrame.grid(row=1, column=6, sticky="nsew", padx=5, pady=5)

virtualImportFrame.grid_columnconfigure(0, weight=1)

# Import STL button
importSTLBut = ttk.Button(virtualImportFrame, text="Import STL", command=import_stl_file)
importSTLBut.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

# File Name
fileNameLab = Label(virtualImportFrame, text="File Name")
fileNameLab.grid(row=1, column=0, sticky="w", padx=5, pady=(10, 2))
stl_name_entry = Entry(virtualImportFrame, textvariable=stl_name_var, width=20)
stl_name_entry.grid(row=2, column=0, sticky="ew", padx=5, pady=2)

# X Position
xPosLab = Label(virtualImportFrame, text="X Position")
xPosLab.grid(row=3, column=0, sticky="w", padx=5, pady=(10, 2))
x_entry = Entry(virtualImportFrame, textvariable=x_var, width=10)
x_entry.grid(row=4, column=0, sticky="w", padx=5, pady=2)

# Y Position
yPosLab = Label(virtualImportFrame, text="Y Position")
yPosLab.grid(row=5, column=0, sticky="w", padx=5, pady=(10, 2))
y_entry = Entry(virtualImportFrame, textvariable=y_var, width=10)
y_entry.grid(row=6, column=0, sticky="w", padx=5, pady=2)

# Z Position
zPosLab = Label(virtualImportFrame, text="Z Position")
zPosLab.grid(row=7, column=0, sticky="w", padx=5, pady=(10, 2))
z_entry = Entry(virtualImportFrame, textvariable=z_var, width=10)
z_entry.grid(row=8, column=0, sticky="w", padx=5, pady=2)

# Z Rotation
zRotLab = Label(virtualImportFrame, text="Z Rotation")
zRotLab.grid(row=9, column=0, sticky="w", padx=5, pady=(10, 2))
rot_entry = Entry(virtualImportFrame, textvariable=rot_var, width=10)
rot_entry.grid(row=10, column=0, sticky="w", padx=5, pady=2)

# Update Position button
updatePosBut = ttk.Button(virtualImportFrame, text="Update Position", command=update_stl_transform)
updatePosBut.grid(row=11, column=0, sticky="ew", padx=5, pady=(10, 5))

# ============================================================================
# ROW 2, COLUMN 5: Save Frame (below and right of Commands)
# ============================================================================
saveFrame = LabelFrame(tab2, text="Save", padding=10)
saveFrame.grid(row=2, column=5, columnspan=2, sticky="ew", padx=5, pady=5)

saveFrame.grid_columnconfigure(0, weight=1)
saveFrame.grid_rowconfigure(0, weight=1)  # Center vertically

# Save All button
saveCalBut = Button(saveFrame, text="SAVE ALL", width=15, command=SaveAndApplyCalibration)
saveCalBut.grid(row=0, column=0, sticky="", padx=5, pady=5)

# ============================================================================
# ROW 2: Commands Frame (spans all columns)
# ============================================================================
cmdFrame = LabelFrame(tab2, text="Commands", padding=10)
cmdFrame.grid(row=2, column=0, columnspan=5, sticky="ew", padx=5, pady=5)

cmdFrame.grid_columnconfigure(0, weight=1)

cmdSentLab = Label(cmdFrame, text="Last Command Sent to Controller")
cmdSentLab.grid(row=0, column=0, sticky="w", padx=5, pady=(0, 2))

cmdSentEntryField = Entry(cmdFrame, width=120, justify="center")
cmdSentEntryField.grid(row=1, column=0, sticky="ew", padx=5, pady=2)

cmdRecLab = Label(cmdFrame, text="Last Response From Controller")
cmdRecLab.grid(row=2, column=0, sticky="w", padx=5, pady=(10, 2))

cmdRecEntryField = Entry(cmdFrame, width=120, justify="center")
cmdRecEntryField.grid(row=3, column=0, sticky="ew", padx=5, pady=2)

####TAB 3

# ============================================================================
# Tab 3 Grid Layout Configuration
# ============================================================================
tab3.grid_rowconfigure(0, weight=1)
tab3.grid_rowconfigure(1, weight=1)
tab3.grid_columnconfigure(0, weight=0, minsize=180)  # Motor Dir, Cal Dir
tab3.grid_columnconfigure(1, weight=0, minsize=180)  # Pos Limits, Steps/Deg
tab3.grid_columnconfigure(2, weight=0, minsize=220)  # Drive MS, Encoder CPR
tab3.grid_columnconfigure(3, weight=0, minsize=280)  # DH Parameters, Tool Frame
tab3.grid_columnconfigure(4, weight=0, minsize=200)  # Defaults
tab3.grid_columnconfigure(5, weight=1)  # Remaining .place() widgets

# ============================================================================
# Motor Direction Frame (Row 0, Column 0)
# ============================================================================
motorDirFrame = LabelFrame(tab3, text="Motor Direction", padding=10)
motorDirFrame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
motorDirFrame.grid_columnconfigure(0, weight=0)
motorDirFrame.grid_columnconfigure(1, weight=1)

J1MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J1 Motor Direction")
J1MotDirLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J1MotDirEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J2 Motor Direction")
J2MotDirLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J2MotDirEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J3 Motor Direction")
J3MotDirLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J3MotDirEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J4 Motor Direction")
J4MotDirLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J4MotDirEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J5 Motor Direction")
J5MotDirLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J5MotDirEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J6 Motor Direction")
J6MotDirLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J6MotDirEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J7MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J7 Motor Direction")
J7MotDirLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J7MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J7MotDirEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J8MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J8 Motor Direction")
J8MotDirLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J8MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J8MotDirEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J9MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J9 Motor Direction")
J9MotDirLab_grid.grid(row=8, column=0, sticky="w", padx=5, pady=2)
J9MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J9MotDirEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Calibration Direction Frame (Row 1, Column 0)
# ============================================================================
calDirFrame = LabelFrame(tab3, text="Calibration Direction", padding=10)
calDirFrame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
calDirFrame.grid_columnconfigure(0, weight=0)
calDirFrame.grid_columnconfigure(1, weight=1)

J1CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J1 Calibration Dir.")
J1CalDirLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J1CalDirEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J2 Calibration Dir.")
J2CalDirLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J2CalDirEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J3 Calibration Dir.")
J3CalDirLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J3CalDirEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J4 Calibration Dir.")
J4CalDirLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J4CalDirEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J5 Calibration Dir.")
J5CalDirLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J5CalDirEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J6 Calibration Dir.")
J6CalDirLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J6CalDirEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J7CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J7 Calibration Dir.")
J7CalDirLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J7CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J7CalDirEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J8CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J8 Calibration Dir.")
J8CalDirLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J8CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J8CalDirEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J9CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J9 Calibration Dir.")
J9CalDirLab_grid.grid(row=8, column=0, sticky="w", padx=5, pady=2)
J9CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J9CalDirEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Position Limits Frame (Row 0, Column 1)
# ============================================================================
posLimFrame = LabelFrame(tab3, text="Position Limits", padding=10)
posLimFrame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
posLimFrame.grid_columnconfigure(0, weight=0)
posLimFrame.grid_columnconfigure(1, weight=1)

J1PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J1 Pos Limit")
J1PosLimLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J1PosLimEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J1NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J1 Neg Limit")
J1NegLimLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J1NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J1NegLimEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J2PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J2 Pos Limit")
J2PosLimLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J2PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J2PosLimEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J2NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J2 Neg Limit")
J2NegLimLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J2NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J2NegLimEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J3PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J3 Pos Limit")
J3PosLimLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J3PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J3PosLimEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J3NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J3 Neg Limit")
J3NegLimLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J3NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J3NegLimEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J4PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J4 Pos Limit")
J4PosLimLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J4PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J4PosLimEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J4NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J4 Neg Limit")
J4NegLimLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J4NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J4NegLimEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J5PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J5 Pos Limit")
J5PosLimLab_grid.grid(row=8, column=0, sticky="w", padx=5, pady=2)
J5PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J5PosLimEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)

J5NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J5 Neg Limit")
J5NegLimLab_grid.grid(row=9, column=0, sticky="w", padx=5, pady=2)
J5NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J5NegLimEntryField.grid(row=9, column=1, sticky="w", padx=5, pady=2)

J6PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J6 Pos Limit")
J6PosLimLab_grid.grid(row=10, column=0, sticky="w", padx=5, pady=2)
J6PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J6PosLimEntryField.grid(row=10, column=1, sticky="w", padx=5, pady=2)

J6NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J6 Neg Limit")
J6NegLimLab_grid.grid(row=11, column=0, sticky="w", padx=5, pady=2)
J6NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J6NegLimEntryField.grid(row=11, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Steps per Degree Frame (Row 1, Column 1)
# ============================================================================
stepDegFrame = LabelFrame(tab3, text="Steps per Degree", padding=10)
stepDegFrame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
stepDegFrame.grid_columnconfigure(0, weight=0)
stepDegFrame.grid_columnconfigure(1, weight=1)

J1StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J1 Step/Deg")
J1StepDegLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1StepDegEntryField = Entry(stepDegFrame, width=5, justify="center")
J1StepDegEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J2 Step/Deg")
J2StepDegLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2StepDegEntryField = Entry(stepDegFrame, width=5, justify="center")
J2StepDegEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J3 Step/Deg")
J3StepDegLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3StepDegEntryField = Entry(stepDegFrame, width=5, justify="center")
J3StepDegEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J4 Step/Deg")
J4StepDegLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4StepDegEntryField = Entry(stepDegFrame, width=5, justify="center")
J4StepDegEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J5 Step/Deg")
J5StepDegLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5StepDegEntryField = Entry(stepDegFrame, width=5, justify="center")
J5StepDegEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J6 Step/Deg")
J6StepDegLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6StepDegEntryField = Entry(stepDegFrame, width=5, justify="center")
J6StepDegEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Drive Microsteps Frame (Row 0, Column 2)
# ============================================================================
driveMSFrame = LabelFrame(tab3, text="Drive Microsteps", padding=10)
driveMSFrame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
driveMSFrame.grid_columnconfigure(0, weight=0)
driveMSFrame.grid_columnconfigure(1, weight=1)

J1DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J1 Drive Microstep")
J1DriveMSLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J1DriveMSEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J2 Drive Microstep")
J2DriveMSLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J2DriveMSEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J3 Drive Microstep")
J3DriveMSLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J3DriveMSEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J4 Drive Microstep")
J4DriveMSLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J4DriveMSEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J5 Drive Microstep")
J5DriveMSLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J5DriveMSEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J6 Drive Microstep")
J6DriveMSLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J6DriveMSEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Encoder CPR Frame (Row 1, Column 2)
# ============================================================================
encCPRFrame = LabelFrame(tab3, text="Encoder CPR", padding=10)
encCPRFrame.grid(row=1, column=2, sticky="nsew", padx=5, pady=5)
encCPRFrame.grid_columnconfigure(0, weight=0)
encCPRFrame.grid_columnconfigure(1, weight=1)

J1EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J1 Encoder CPR")
J1EncCPRLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J1EncCPREntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J2 Encoder CPR")
J2EncCPRLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J2EncCPREntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J3 Encoder CPR")
J3EncCPRLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J3EncCPREntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J4 Encoder CPR")
J4EncCPRLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J4EncCPREntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J5 Encoder CPR")
J5EncCPRLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J5EncCPREntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J6 Encoder CPR")
J6EncCPRLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J6EncCPREntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# DH Parameters Frame (Row 0, Column 3)
# ============================================================================
dhParamsFrame = LabelFrame(tab3, text="DH Parameters", padding=10)
dhParamsFrame.grid(row=0, column=3, sticky="nsew", padx=5, pady=5)

# Column headers
dhParamsFrame.grid_columnconfigure(0, weight=0, minsize=30)   # J1-J6 labels
dhParamsFrame.grid_columnconfigure(1, weight=0, minsize=50)   # DH-Θ
dhParamsFrame.grid_columnconfigure(2, weight=0, minsize=50)   # DH-α
dhParamsFrame.grid_columnconfigure(3, weight=0, minsize=50)   # DH-d
dhParamsFrame.grid_columnconfigure(4, weight=0, minsize=50)   # DH-a

# Header row
Label(dhParamsFrame, font=("Arial", 8), text="").grid(row=0, column=0)
Label(dhParamsFrame, font=("Arial", 8), text="DH-Θ").grid(row=0, column=1)
Label(dhParamsFrame, font=("Arial", 8), text="DH-α").grid(row=0, column=2)
Label(dhParamsFrame, font=("Arial", 8), text="DH-d").grid(row=0, column=3)
Label(dhParamsFrame, font=("Arial", 8), text="DH-a").grid(row=0, column=4)

# J1 row
Label(dhParamsFrame, font=("Arial", 8), text="J1").grid(row=1, column=0, sticky="w", padx=5, pady=2)
J1ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1ΘEntryField.grid(row=1, column=1, padx=2, pady=2)
J1αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1αEntryField.grid(row=1, column=2, padx=2, pady=1)
J1dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1dEntryField.grid(row=1, column=3, padx=2, pady=2)
J1aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1aEntryField.grid(row=1, column=4, padx=2, pady=2)

# J2 row
Label(dhParamsFrame, font=("Arial", 8), text="J2").grid(row=2, column=0, sticky="w", padx=5, pady=2)
J2ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2ΘEntryField.grid(row=2, column=1, padx=2, pady=2)
J2αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2αEntryField.grid(row=2, column=2, padx=2, pady=1)
J2dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2dEntryField.grid(row=2, column=3, padx=2, pady=2)
J2aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2aEntryField.grid(row=2, column=4, padx=2, pady=2)

# J3 row
Label(dhParamsFrame, font=("Arial", 8), text="J3").grid(row=3, column=0, sticky="w", padx=5, pady=2)
J3ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3ΘEntryField.grid(row=3, column=1, padx=2, pady=2)
J3αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3αEntryField.grid(row=3, column=2, padx=2, pady=1)
J3dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3dEntryField.grid(row=3, column=3, padx=2, pady=2)
J3aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3aEntryField.grid(row=3, column=4, padx=2, pady=2)

# J4 row
Label(dhParamsFrame, font=("Arial", 8), text="J4").grid(row=4, column=0, sticky="w", padx=5, pady=2)
J4ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4ΘEntryField.grid(row=4, column=1, padx=2, pady=2)
J4αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4αEntryField.grid(row=4, column=2, padx=2, pady=1)
J4dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4dEntryField.grid(row=4, column=3, padx=2, pady=2)
J4aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4aEntryField.grid(row=4, column=4, padx=2, pady=2)

# J5 row
Label(dhParamsFrame, font=("Arial", 8), text="J5").grid(row=5, column=0, sticky="w", padx=5, pady=2)
J5ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5ΘEntryField.grid(row=5, column=1, padx=2, pady=2)
J5αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5αEntryField.grid(row=5, column=2, padx=2, pady=1)
J5dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5dEntryField.grid(row=5, column=3, padx=2, pady=2)
J5aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5aEntryField.grid(row=5, column=4, padx=2, pady=2)

# J6 row
Label(dhParamsFrame, font=("Arial", 8), text="J6").grid(row=6, column=0, sticky="w", padx=5, pady=2)
J6ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6ΘEntryField.grid(row=6, column=1, padx=2, pady=2)
J6αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6αEntryField.grid(row=6, column=2, padx=2, pady=1)
J6dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6dEntryField.grid(row=6, column=3, padx=2, pady=2)
J6aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6aEntryField.grid(row=6, column=4, padx=2, pady=2)

# ============================================================================
# Tool Frame Offset Frame (Row 1, Column 3)
# ============================================================================
toolFrameFrame = LabelFrame(tab3, text="Tool Frame Offset", padding=10)
toolFrameFrame.grid(row=1, column=3, sticky="nsew", padx=5, pady=5)

toolFrameFrame.grid_columnconfigure(0, weight=1)
toolFrameFrame.grid_columnconfigure(1, weight=1)
toolFrameFrame.grid_columnconfigure(2, weight=1)
toolFrameFrame.grid_columnconfigure(3, weight=1)
toolFrameFrame.grid_columnconfigure(4, weight=1)
toolFrameFrame.grid_columnconfigure(5, weight=1)

# Header row
Label(toolFrameFrame, font=("Arial", 11), text="X").grid(row=0, column=0, padx=2, pady=2)
Label(toolFrameFrame, font=("Arial", 11), text="Y").grid(row=0, column=1, padx=2, pady=2)
Label(toolFrameFrame, font=("Arial", 11), text="Z").grid(row=0, column=2, padx=2, pady=1)
Label(toolFrameFrame, font=("Arial", 11), text="Rz").grid(row=0, column=3, padx=2, pady=2)
Label(toolFrameFrame, font=("Arial", 11), text="Ry").grid(row=0, column=4, padx=2, pady=2)
Label(toolFrameFrame, font=("Arial", 11), text="Rx").grid(row=0, column=5, padx=2, pady=2)

# Entry fields row
TFxEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFxEntryField.grid(row=1, column=0, padx=2, pady=2)
TFyEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFyEntryField.grid(row=1, column=1, padx=2, pady=2)
TFzEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFzEntryField.grid(row=1, column=2, padx=2, pady=1)
TFrzEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFrzEntryField.grid(row=1, column=3, padx=2, pady=2)
TFryEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFryEntryField.grid(row=1, column=4, padx=2, pady=2)
TFrxEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFrxEntryField.grid(row=1, column=5, padx=2, pady=2)

# Checkbox row
DisableWristCbut = Checkbutton(toolFrameFrame, text="Disable Wrist Rotation - Linear Moves", variable=CAL['DisableWristRotVal'])
DisableWristCbut.grid(row=2, column=0, columnspan=6, sticky="w", padx=5, pady=5)

# ============================================================================
# Defaults Frame (Row 0-1, Column 4)
# ============================================================================
defaultsFrame = LabelFrame(tab3, text="Defaults", padding=10)
defaultsFrame.grid(row=0, column=4, rowspan=2, sticky="nsew", padx=5, pady=5)

defaultsFrame.grid_columnconfigure(0, weight=1)

loadAR4Mk2But = Button(defaultsFrame, text="Load AR4-MK3 Defaults", width=26, command=LoadAR4Mk3default)
loadAR4Mk2But.grid(row=0, column=0, padx=5, pady=5)

loadAR4Mk2But = Button(defaultsFrame, text="Load AR4-MK2 Defaults", width=26, command=LoadAR4Mk2default)
loadAR4Mk2But.grid(row=1, column=0, padx=5, pady=5)

loadAR4But = Button(defaultsFrame, text="Load AR4 Defaults", width=26, command=LoadAR4default)
loadAR4But.grid(row=2, column=0, padx=5, pady=5)

loadAR3But = Button(defaultsFrame, text="Load AR3 Defaults", width=26, command=LoadAR3default)
loadAR3But.grid(row=3, column=0, padx=5, pady=5)

saveCalBut = Button(defaultsFrame, text="SAVE", width=26, command=SaveAndApplyCalibration)
saveCalBut.grid(row=4, column=0, padx=5, pady=(10, 30)) # 10 pixels above, 30 below


loadCustomBut = Button(defaultsFrame, text="Load Custom Calibration", width=26, command=load_custom_calibration)
loadCustomBut .grid(row=5, column=0, padx=5, pady=(30, 5))

saveCustomBut = Button(defaultsFrame, text="Save Custom Calibration", width=26, command=save_custom_calibration)
saveCustomBut.grid(row=6, column=0, padx=5, pady=5)



# #### TOOL FRAME ####
# ToolFrameLab = Label(tab3, text = "Tool Frame Offset")
# ToolFrameLab.place(x=970, y=60)
# 
# UFxLab = Label(tab3, font=("Arial", 11), text = "X")
# UFxLab.place(x=920, y=90)
# 
# UFyLab = Label(tab3, font=("Arial", 11), text = "Y")
# UFyLab.place(x=960, y=90)
# 
# UFzLab = Label(tab3, font=("Arial", 11), text = "Z")
# UFzLab.place(x=1000, y=90)
# 
# UFRxLab = Label(tab3, font=("Arial", 11), text = "Rz")
# UFRxLab.place(x=1040, y=90)
# 
# UFRyLab = Label(tab3, font=("Arial", 11), text = "Ry")
# UFRyLab.place(x=1080, y=90)
# 
# UFRzLab = Label(tab3, font=("Arial", 11), text = "Rx")
# UFRzLab.place(x=1120, y=90)
# 
# TFxEntryField = Entry(tab3,width=4,justify="center")
# TFxEntryField.place(x=910, y=115)
# TFyEntryField = Entry(tab3,width=4,justify="center")
# TFyEntryField.place(x=950, y=115)
# TFzEntryField = Entry(tab3,width=4,justify="center")
# TFzEntryField.place(x=990, y=115)
# TFrzEntryField = Entry(tab3,width=4,justify="center")
# TFrzEntryField.place(x=1030, y=115)
# TFryEntryField = Entry(tab3,width=4,justify="center")
# TFryEntryField.place(x=1070, y=115)
# TFrxEntryField = Entry(tab3,width=4,justify="center")
# TFrxEntryField.place(x=1110, y=115)
# 
# DisableWristCbut = Checkbutton(tab3, text="Disable Wrist Rotation - Linear Moves",variable = CAL['DisableWristRotVal'])
# DisableWristCbut.place(x=910, y=150)


# # ####  MOTOR DIRECTIONS ####

# # J1MotDirLab = Label(tab3, font=("Arial", 8), text = "J1 Motor Direction")
# # J1MotDirLab.place(x=10, y=20)
# # J2MotDirLab = Label(tab3, font=("Arial", 8), text = "J2 Motor Direction")
# # J2MotDirLab.place(x=10, y=45)
# # J3MotDirLab = Label(tab3, font=("Arial", 8), text = "J3 Motor Direction")
# # J3MotDirLab.place(x=10, y=70)
# # J4MotDirLab = Label(tab3, font=("Arial", 8), text = "J4 Motor Direction")
# # J4MotDirLab.place(x=10, y=95)
# # J5MotDirLab = Label(tab3, font=("Arial", 8), text = "J5 Motor Direction")
# # J5MotDirLab.place(x=10, y=120)
# # J6MotDirLab = Label(tab3, font=("Arial", 8), text = "J6 Motor Direction")
# # J6MotDirLab.place(x=10, y=145)
# # J7MotDirLab = Label(tab3, font=("Arial", 8), text = "J7 Motor Direction")
# # J7MotDirLab.place(x=10, y=170)
# # J8MotDirLab = Label(tab3, font=("Arial", 8), text = "J8 Motor Direction")
# # J8MotDirLab.place(x=10, y=195)
# # J9MotDirLab = Label(tab3, font=("Arial", 8), text = "J9 Motor Direction")
# # J9MotDirLab.place(x=10, y=220)

# # J1MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J1MotDirEntryField.place(x=110, y=20)
# # J2MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J2MotDirEntryField.place(x=110, y=45)
# # J3MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J3MotDirEntryField.place(x=110, y=70)
# # J4MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J4MotDirEntryField.place(x=110, y=95)
# # J5MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J5MotDirEntryField.place(x=110, y=120)
# # J6MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J6MotDirEntryField.place(x=110, y=145)
# # J7MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J7MotDirEntryField.place(x=110, y=170)
# # J8MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J8MotDirEntryField.place(x=110, y=195)
# # J9MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J9MotDirEntryField.place(x=110, y=220)


# # ####  CALIBRATION DIRECTIONS ####

# # J1CalDirLab = Label(tab3, font=("Arial", 8), text = "J1 Calibration Dir.")
# # J1CalDirLab.place(x=10, y=280)
# # J2CalDirLab = Label(tab3, font=("Arial", 8), text = "J2 Calibration Dir.")
# # J2CalDirLab.place(x=10, y=305)
# # J3CalDirLab = Label(tab3, font=("Arial", 8), text = "J3 Calibration Dir.")
# # J3CalDirLab.place(x=10, y=330)
# # J4CalDirLab = Label(tab3, font=("Arial", 8), text = "J4 Calibration Dir.")
# # J4CalDirLab.place(x=10, y=355)
# # J5CalDirLab = Label(tab3, font=("Arial", 8), text = "J5 Calibration Dir.")
# # J5CalDirLab.place(x=10, y=380)
# # J6CalDirLab = Label(tab3, font=("Arial", 8), text = "J6 Calibration Dir.")
# # J6CalDirLab.place(x=10, y=405)
# # J7CalDirLab = Label(tab3, font=("Arial", 8), text = "J7 Calibration Dir.")
# # J7CalDirLab.place(x=10, y=430)
# # J8CalDirLab = Label(tab3, font=("Arial", 8), text = "J8 Calibration Dir.")
# # J8CalDirLab.place(x=10, y=455)
# # J9CalDirLab = Label(tab3, font=("Arial", 8), text = "J9 Calibration Dir.")
# # J9CalDirLab.place(x=10, y=480)

# # J1CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J1CalDirEntryField.place(x=110, y=280)
# # J2CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J2CalDirEntryField.place(x=110, y=305)
# # J3CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J3CalDirEntryField.place(x=110, y=330)
# # J4CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J4CalDirEntryField.place(x=110, y=355)
# # J5CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J5CalDirEntryField.place(x=110, y=380)
# # J6CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J6CalDirEntryField.place(x=110, y=405)
# # J7CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J7CalDirEntryField.place(x=110, y=430)
# # J8CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J8CalDirEntryField.place(x=110, y=455)
# # J9CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J9CalDirEntryField.place(x=110, y=480)

# # ### axis limits
# # J1PosLimLab = Label(tab3, font=("Arial", 8), text = "J1 Pos Limit")
# # J1PosLimLab.place(x=200, y=20)
# # J1NegLimLab = Label(tab3, font=("Arial", 8), text = "J1 Neg Limit")
# # J1NegLimLab.place(x=200, y=45)
# # J2PosLimLab = Label(tab3, font=("Arial", 8), text = "J2 Pos Limit")
# # J2PosLimLab.place(x=200, y=70)
# # J2NegLimLab = Label(tab3, font=("Arial", 8), text = "J2 Neg Limit")
# # J2NegLimLab.place(x=200, y=95)
# # J3PosLimLab = Label(tab3, font=("Arial", 8), text = "J3 Pos Limit")
# # J3PosLimLab.place(x=200, y=120)
# # J3NegLimLab = Label(tab3, font=("Arial", 8), text = "J3 Neg Limit")
# # J3NegLimLab.place(x=200, y=145)
# # J4PosLimLab = Label(tab3, font=("Arial", 8), text = "J4 Pos Limit")
# # J4PosLimLab.place(x=200, y=170)
# # J4NegLimLab = Label(tab3, font=("Arial", 8), text = "J4 Neg Limit")
# # J4NegLimLab.place(x=200, y=195)
# # J5PosLimLab = Label(tab3, font=("Arial", 8), text = "J5 Pos Limit")
# # J5PosLimLab.place(x=200, y=220)
# # J5NegLimLab = Label(tab3, font=("Arial", 8), text = "J5 Neg Limit")
# # J5NegLimLab.place(x=200, y=245)
# # J6PosLimLab = Label(tab3, font=("Arial", 8), text = "J6 Pos Limit")
# # J6PosLimLab.place(x=200, y=270)
# # J6NegLimLab = Label(tab3, font=("Arial", 8), text = "J6 Neg Limit")
# # J6NegLimLab.place(x=200, y=295)

# # J1PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J1PosLimEntryField.place(x=280, y=20)
# # J1NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J1NegLimEntryField.place(x=280, y=45)
# # J2PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J2PosLimEntryField.place(x=280, y=70)
# # J2NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J2NegLimEntryField.place(x=280, y=95)
# # J3PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J3PosLimEntryField.place(x=280, y=120)
# # J3NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J3NegLimEntryField.place(x=280, y=145)
# # J4PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J4PosLimEntryField.place(x=280, y=170)
# # J4NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J4NegLimEntryField.place(x=280, y=195)
# # J5PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J5PosLimEntryField.place(x=280, y=220)
# # J5NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J5NegLimEntryField.place(x=280, y=245)
# # J6PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J6PosLimEntryField.place(x=280, y=270)
# # J6NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J6NegLimEntryField.place(x=280, y=295)


### steps per degress
# # J1StepDegLab = Label(tab3, font=("Arial", 8), text = "J1 Step/Deg")
# # J1StepDegLab.place(x=200, y=345)
# # J2StepDegLab = Label(tab3, font=("Arial", 8), text = "J2 Step/Deg")
# # J2StepDegLab.place(x=200, y=370)
# # J3StepDegLab = Label(tab3, font=("Arial", 8), text = "J3 Step/Deg")
# # J3StepDegLab.place(x=200, y=395)
# # J4StepDegLab = Label(tab3, font=("Arial", 8), text = "J4 Step/Deg")
# # J4StepDegLab.place(x=200, y=420)
# # J5StepDegLab = Label(tab3, font=("Arial", 8), text = "J5 Step/Deg")
# # J5StepDegLab.place(x=200, y=445)
# # J6StepDegLab = Label(tab3, font=("Arial", 8), text = "J6 Step/Deg")
# # J6StepDegLab.place(x=200, y=470)

# # J1StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J1StepDegEntryField.place(x=280, y=345)
# # J2StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J2StepDegEntryField.place(x=280, y=370)
# # J3StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J3StepDegEntryField.place(x=280, y=395)
# # J4StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J4StepDegEntryField.place(x=280, y=420)
# # J5StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J5StepDegEntryField.place(x=280, y=445)
# # J6StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J6StepDegEntryField.place(x=280, y=470)


### DRIVER STEPS
# # J1DriveMSLab = Label(tab3, font=("Arial", 8), text = "J1 Drive Microstep")
# # J1DriveMSLab.place(x=390, y=20)
# # J2DriveMSLab = Label(tab3, font=("Arial", 8), text = "J2 Drive Microstep")
# # J2DriveMSLab.place(x=390, y=45)
# # J3DriveMSLab = Label(tab3, font=("Arial", 8), text = "J3 Drive Microstep")
# # J3DriveMSLab.place(x=390, y=70)
# # J4DriveMSLab = Label(tab3, font=("Arial", 8), text = "J4 Drive Microstep")
# # J4DriveMSLab.place(x=390, y=95)
# # J5DriveMSLab = Label(tab3, font=("Arial", 8), text = "J5 Drive Microstep")
# # J5DriveMSLab.place(x=390, y=120)
# # J6DriveMSLab = Label(tab3, font=("Arial", 8), text = "J6 Drive Microstep")
# # J6DriveMSLab.place(x=390, y=145)

# # J1DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J1DriveMSEntryField.place(x=500, y=20)
# # J2DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J2DriveMSEntryField.place(x=500, y=45)
# # J3DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J3DriveMSEntryField.place(x=500, y=70)
# # J4DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J4DriveMSEntryField.place(x=500, y=95)
# # J5DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J5DriveMSEntryField.place(x=500, y=120)
# # J6DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J6DriveMSEntryField.place(x=500, y=145)


###ENCODER CPR
# # J1EncCPRLab = Label(tab3, font=("Arial", 8), text = "J1 Encoder CPR")
# # J1EncCPRLab.place(x=390, y=195)
# # J2EncCPRLab = Label(tab3, font=("Arial", 8), text = "J2 Encoder CPR")
# # J2EncCPRLab.place(x=390, y=220)
# # J3EncCPRLab = Label(tab3, font=("Arial", 8), text = "J3 Encoder CPR")
# # J3EncCPRLab.place(x=390, y=245)
# # J4EncCPRLab = Label(tab3, font=("Arial", 8), text = "J4 Encoder CPR")
# # J4EncCPRLab.place(x=390, y=270)
# # J5EncCPRLab = Label(tab3, font=("Arial", 8), text = "J5 Encoder CPR")
# # J5EncCPRLab.place(x=390, y=295)
# # J6EncCPRLab = Label(tab3, font=("Arial", 8), text = "J6 Encoder CPR")
# # J6EncCPRLab.place(x=390, y=320)

# # J1EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J1EncCPREntryField.place(x=500, y=195)
# # J2EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J2EncCPREntryField.place(x=500, y=220)
# # J3EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J3EncCPREntryField.place(x=500, y=245)
# # J4EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J4EncCPREntryField.place(x=500, y=270)
# # J5EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J5EncCPREntryField.place(x=500, y=295)
# # J6EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J6EncCPREntryField.place(x=500, y=320)


# ### DH PARAMS
# 
# ### DRIVER STEPS
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J1")
# J1DHparamLab.place(x=600, y=45)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J2")
# J1DHparamLab.place(x=600, y=70)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J3")
# J1DHparamLab.place(x=600, y=95)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J4")
# J1DHparamLab.place(x=600, y=120)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J5")
# J1DHparamLab.place(x=600, y=145)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J6")
# J1DHparamLab.place(x=600, y=170)
# 
# ΘDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-Θ")
# ΘDHparamLab.place(x=645, y=20)
# αDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-α")
# αDHparamLab.place(x=700, y=20)
# dDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-d")
# dDHparamLab.place(x=755, y=20)
# aDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-a")
# aDHparamLab.place(x=810, y=20)
# 
# 
# J1ΘEntryField = Entry(tab3,width=5,justify="center")
# J1ΘEntryField.place(x=630, y=45)
# J2ΘEntryField = Entry(tab3,width=5,justify="center")
# J2ΘEntryField.place(x=630, y=70)
# J3ΘEntryField = Entry(tab3,width=5,justify="center")
# J3ΘEntryField.place(x=630, y=95)
# J4ΘEntryField = Entry(tab3,width=5,justify="center")
# J4ΘEntryField.place(x=630, y=120)
# J5ΘEntryField = Entry(tab3,width=5,justify="center")
# J5ΘEntryField.place(x=630, y=145)
# J6ΘEntryField = Entry(tab3,width=5,justify="center")
# J6ΘEntryField.place(x=630, y=170)
# 
# J1αEntryField = Entry(tab3,width=5,justify="center")
# J1αEntryField.place(x=685, y=45)
# J2αEntryField = Entry(tab3,width=5,justify="center")
# J2αEntryField.place(x=685, y=70)
# J3αEntryField = Entry(tab3,width=5,justify="center")
# J3αEntryField.place(x=685, y=95)
# J4αEntryField = Entry(tab3,width=5,justify="center")
# J4αEntryField.place(x=685, y=120)
# J5αEntryField = Entry(tab3,width=5,justify="center")
# J5αEntryField.place(x=685, y=145)
# J6αEntryField = Entry(tab3,width=5,justify="center")
# J6αEntryField.place(x=685, y=170)
# 
# J1dEntryField = Entry(tab3,width=5,justify="center")
# J1dEntryField.place(x=740, y=45)
# J2dEntryField = Entry(tab3,width=5,justify="center")
# J2dEntryField.place(x=740, y=70)
# J3dEntryField = Entry(tab3,width=5,justify="center")
# J3dEntryField.place(x=740, y=95)
# J4dEntryField = Entry(tab3,width=5,justify="center")
# J4dEntryField.place(x=740, y=120)
# J5dEntryField = Entry(tab3,width=5,justify="center")
# J5dEntryField.place(x=740, y=145)
# J6dEntryField = Entry(tab3,width=5,justify="center")
# J6dEntryField.place(x=740, y=170)
# 
# J1aEntryField = Entry(tab3,width=5,justify="center")
# J1aEntryField.place(x=795, y=45)
# J2aEntryField = Entry(tab3,width=5,justify="center")
# J2aEntryField.place(x=795, y=70)
# J3aEntryField = Entry(tab3,width=5,justify="center")
# J3aEntryField.place(x=795, y=95)
# J4aEntryField = Entry(tab3,width=5,justify="center")
# J4aEntryField.place(x=795, y=120)
# J5aEntryField = Entry(tab3,width=5,justify="center")
# J5aEntryField.place(x=795, y=145)
# J6aEntryField = Entry(tab3,width=5,justify="center")
# J6aEntryField.place(x=795, y=170)


# ### LOAD DEFAULT ###
# 
# loadAR4Mk2But = Button(tab3,  text="Load AR4-MK3 Defaults",  width=26, command = LoadAR4Mk3default)
# loadAR4Mk2But.place(x=1150, y=470)
# 
# loadAR4Mk2But = Button(tab3,  text="Load AR4-MK2 Defaults",  width=26, command = LoadAR4Mk2default)
# loadAR4Mk2But.place(x=1150, y=510)
# 
# loadAR4But = Button(tab3,  text="Load AR4 Defaults",  width=26, command = LoadAR4default)
# loadAR4But.place(x=1150, y=550)
# 
# loadAR3But = Button(tab3,  text="Load AR3 Defaults",  width=26, command = LoadAR3default)
# loadAR3But.place(x=1150, y=590)
# 
# 
# 
# 
# 
# 
# #### SAVE ####
# 
# saveCalBut = Button(tab3,  text="SAVE",  width=26, command = SaveAndApplyCalibration)
# saveCalBut.place(x=1150, y=630)



####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 4

# ============================================================================
# Tab 4 Grid Layout Configuration
# ============================================================================
tab4.grid_rowconfigure(0, weight=1)
tab4.grid_rowconfigure(1, weight=0)
tab4.grid_columnconfigure(0, weight=0, minsize=300)  # 5v IO BOARD
tab4.grid_columnconfigure(1, weight=0, minsize=250)  # AUX COM DEVICE
tab4.grid_columnconfigure(2, weight=0, minsize=400)  # MODBUS DEVICE
tab4.grid_columnconfigure(3, weight=1)  # Remaining space

# ============================================================================
# 5v IO BOARD Frame (Row 0, Column 0)
# ============================================================================
ioBoardFrame = LabelFrame(tab4, text="5v IO BOARD", padding=10)
ioBoardFrame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

ioBoardFrame.grid_columnconfigure(0, weight=0, minsize=60)   # Servo buttons
ioBoardFrame.grid_columnconfigure(1, weight=0, minsize=10)   # = labels
ioBoardFrame.grid_columnconfigure(2, weight=0, minsize=35)   # Entry fields
ioBoardFrame.grid_columnconfigure(3, weight=0, minsize=10)   # Spacer between columns
ioBoardFrame.grid_columnconfigure(4, weight=0, minsize=60)   # DO buttons
ioBoardFrame.grid_columnconfigure(5, weight=0, minsize=10)   # = labels
ioBoardFrame.grid_columnconfigure(6, weight=0, minsize=35)   # Entry fields (no expansion)

# Configure all rows to have uniform minimum height
for row in range(12):
    ioBoardFrame.grid_rowconfigure(row, minsize=25)

# Servo 0 on
servo0onBut = Button(ioBoardFrame, text="Servo 0", command=Servo0on)
servo0onBut.grid(row=0, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=0, column=1, padx=2, pady=1)
servo0onEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo0onEntryField.grid(row=0, column=2, padx=2, pady=1)

# DO on (row 0)
DO1onBut = Button(ioBoardFrame, text="DO on", command=DO1on)
DO1onBut.grid(row=0, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=0, column=5, padx=2, pady=1)
DO1onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO1onEntryField.grid(row=0, column=6, padx=2, pady=1)

# Servo 0 off
servo0offBut = Button(ioBoardFrame, text="Servo 0", command=Servo0off)
servo0offBut.grid(row=1, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=1, column=1, padx=2, pady=1)
servo0offEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo0offEntryField.grid(row=1, column=2, padx=2, pady=1)

# DO off (row 1)
DO1offBut = Button(ioBoardFrame, text="DO off", command=DO1off)
DO1offBut.grid(row=1, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=1, column=5, padx=2, pady=1)
DO1offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO1offEntryField.grid(row=1, column=6, padx=2, pady=1)

# Servo 1 on
servo1onBut = Button(ioBoardFrame, text="Servo 1", command=Servo1on)
servo1onBut.grid(row=2, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=2, column=1, padx=2, pady=1)
servo1onEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo1onEntryField.grid(row=2, column=2, padx=2, pady=1)

# DO on (row 2)
DO2onBut = Button(ioBoardFrame, text="DO on", command=DO2on)
DO2onBut.grid(row=2, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=2, column=5, padx=2, pady=1)
DO2onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO2onEntryField.grid(row=2, column=6, padx=2, pady=1)

# Servo 1 off
servo1offBut = Button(ioBoardFrame, text="Servo 1", command=Servo1off)
servo1offBut.grid(row=3, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=3, column=1, padx=2, pady=1)
servo1offEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo1offEntryField.grid(row=3, column=2, padx=2, pady=1)

# DO off (row 3)
DO2offBut = Button(ioBoardFrame, text="DO off", command=DO2off)
DO2offBut.grid(row=3, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=3, column=5, padx=2, pady=1)
DO2offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO2offEntryField.grid(row=3, column=6, padx=2, pady=1)

# Servo 2 on
servo2onBut = Button(ioBoardFrame, text="Servo 2", command=Servo2on)
servo2onBut.grid(row=4, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=4, column=1, padx=2, pady=1)
servo2onEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo2onEntryField.grid(row=4, column=2, padx=2, pady=1)

# DO on (row 4)
DO3onBut = Button(ioBoardFrame, text="DO on", command=DO3on)
DO3onBut.grid(row=4, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=4, column=5, padx=2, pady=1)
DO3onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO3onEntryField.grid(row=4, column=6, padx=2, pady=1)

# Servo 2 off
servo2offBut = Button(ioBoardFrame, text="Servo 2", command=Servo2off)
servo2offBut.grid(row=5, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=5, column=1, padx=2, pady=1)
servo2offEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo2offEntryField.grid(row=5, column=2, padx=2, pady=1)

# DO off (row 5)
DO3offBut = Button(ioBoardFrame, text="DO off", command=DO3off)
DO3offBut.grid(row=5, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=5, column=5, padx=2, pady=1)
DO3offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO3offEntryField.grid(row=5, column=6, padx=2, pady=1)

# Servo 3 on
servo3onBut = Button(ioBoardFrame, text="Servo 3", command=Servo3on)
servo3onBut.grid(row=6, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=6, column=1, padx=2, pady=1)
servo3onEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo3onEntryField.grid(row=6, column=2, padx=2, pady=1)

# DO on (row 6)
DO4onBut = Button(ioBoardFrame, text="DO on", command=DO4on)
DO4onBut.grid(row=6, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=6, column=5, padx=2, pady=1)
DO4onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO4onEntryField.grid(row=6, column=6, padx=2, pady=1)

# Servo 3 off
servo3offBut = Button(ioBoardFrame, text="Servo 3", command=Servo3off)
servo3offBut.grid(row=7, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=7, column=1, padx=2, pady=1)
servo3offEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo3offEntryField.grid(row=7, column=2, padx=2, pady=1)

# DO off (row 7)
DO4offBut = Button(ioBoardFrame, text="DO off", command=DO4off)
DO4offBut.grid(row=7, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=7, column=5, padx=2, pady=1)
DO4offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO4offEntryField.grid(row=7, column=6, padx=2, pady=1)



# DO on (row 8) - no servo
DO5onBut = Button(ioBoardFrame, text="DO on", command=DO5on)
DO5onBut.grid(row=8, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=8, column=5, padx=2, pady=1)
DO5onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO5onEntryField.grid(row=8, column=6, padx=2, pady=1)



# DO off (row 9)
DO5offBut = Button(ioBoardFrame, text="DO off", command=DO5off)
DO5offBut.grid(row=9, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=9, column=5, padx=2, pady=1)
DO5offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO5offEntryField.grid(row=9, column=6, padx=2, pady=1)



# DO on (row 10)
DO6onBut = Button(ioBoardFrame, text="DO on", command=DO6on)
DO6onBut.grid(row=10, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=10, column=5, padx=2, pady=1)
DO6onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO6onEntryField.grid(row=10, column=6, padx=2, pady=1)



# DO off (row 11)
DO6offBut = Button(ioBoardFrame, text="DO off", command=DO6off)
DO6offBut.grid(row=11, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=11, column=5, padx=2, pady=1)
DO6offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO6offEntryField.grid(row=11, column=6, padx=2, pady=1)

# ============================================================================
# AUX COM DEVICE Frame (Row 0, Column 1)
# ============================================================================
auxComFrame = LabelFrame(tab4, text="AUX COM DEVICE", padding=10)
auxComFrame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

auxComFrame.grid_columnconfigure(0, weight=1)

# Aux Com Port
Label(auxComFrame, text="Aux Com Port").grid(row=0, column=0, sticky="w", padx=(5,2), pady=5)
com3PortEntryField = Entry(auxComFrame, width=10, justify="left")
com3PortEntryField.grid(row=0, column=1, sticky="w", padx=(2,5), pady=5)

# Char to Read
Label(auxComFrame, text="Char to Read").grid(row=1, column=0, sticky="w", padx=(5,2), pady=5)
com3charPortEntryField = Entry(auxComFrame, width=10, justify="left")
com3charPortEntryField.grid(row=1, column=1, sticky="w", padx=(2,5), pady=5)

# Test button
comPortBut3 = Button(auxComFrame, text="Test Aux COM Device", command=TestAuxCom)
comPortBut3.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

# Output
com3outPortEntryField = Entry(auxComFrame, width=25, justify="center")
com3outPortEntryField.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

# ============================================================================
# MODBUS DEVICE Frame (Row 0, Column 2)
# ============================================================================
modbusFrame = LabelFrame(tab4, text="MODBUS DEVICE", padding=10)
modbusFrame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)

modbusFrame.grid_columnconfigure(0, weight=1)

# Slave ID
Label(modbusFrame, text="Slave ID").grid(row=0, column=0, sticky="w", padx=(5,2), pady=5)
MBslaveEntryField = Entry(modbusFrame, width=10, justify="left")
MBslaveEntryField.grid(row=0, column=1, sticky="w", padx=(2,5), pady=5)

# Modbus Address
Label(modbusFrame, text="Modbus Address").grid(row=1, column=0, sticky="w", padx=(5,2), pady=5)
MBaddressEntryField = Entry(modbusFrame, width=10, justify="left")
MBaddressEntryField.grid(row=1, column=1, sticky="w", padx=(2,5), pady=5)

# Operation Value
Label(modbusFrame, text="Operation Value").grid(row=2, column=0, sticky="w", padx=(5,2), pady=5)
MBoperValEntryField = Entry(modbusFrame, width=10, justify="left")
MBoperValEntryField.grid(row=2, column=1, sticky="w", padx=(2,5), pady=5)

# Buttons
MBreadCoilBut = Button(modbusFrame, text="Read Coil", width=30, command=MBreadCoil)
MBreadCoilBut.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBreadDinputBut = Button(modbusFrame, text="Read Discrete Input", width=30, command=MBreadInput)
MBreadDinputBut.grid(row=4, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBreadHoldRegBut = Button(modbusFrame, text="Read Holding Register", width=30, command=MBreadHoldReg)
MBreadHoldRegBut.grid(row=5, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBreadInputRegBut = Button(modbusFrame, text="Read Input Register", width=30, command=MBreadInputReg)
MBreadInputRegBut.grid(row=6, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBwriteCoilBut = Button(modbusFrame, text="Write Coil", width=30, command=MBwriteCoil)
MBwriteCoilBut.grid(row=7, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBwriteRegBut = Button(modbusFrame, text="Write Register", width=30, command=MBwriteReg)
MBwriteRegBut.grid(row=8, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

# Output Response
Label(modbusFrame, text="Output Response:").grid(row=9, column=0, columnspan=2, sticky="w", padx=5, pady=(10,2))
MBoutputEntryField = Entry(modbusFrame, width=33, justify="center")
MBoutputEntryField.grid(row=10, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

# ============================================================================
# Information Frame (Row 1, Column 0-2, spans 3 columns)
# ============================================================================
infoFrame = LabelFrame(tab4, text="Information", padding=10)
infoFrame.grid(row=1, column=0, columnspan=3, sticky="ew", padx=5, pady=5)

infoFrame.grid_columnconfigure(0, weight=1)

Label(infoFrame, text="The following IO are available when using the default 5v Nano board for IO:   Inputs = 2-7  /  Outputs = 8-13  /  Servos = A0-A7").grid(row=0, column=0, sticky="w", padx=5, pady=2)

Label(infoFrame, text="The following IO are available when using the default 5v Mega board for IO:   Inputs = 0-27  /  Outputs = 28-53  /  Servos = A0-A7").grid(row=1, column=0, sticky="w", padx=5, pady=2)

Label(infoFrame, text="Please review this tutorial video on using 5v IO boards:").grid(row=2, column=0, sticky="w", padx=5, pady=2)

link2 = Label(infoFrame, font=("Arial", 8), text="https://youtu.be/76F6dS4ar8Y?si=Z6NstZy1zNeHgtCF", foreground="blue", cursor="hand2")
link2.bind("<Button-1>", lambda event: webbrowser.open(link2.cget("text")))
link2.grid(row=3, column=0, sticky="w", padx=5, pady=2)

Label(infoFrame, text="5v board inputs are high impedance and susceptable to floating voltage - inputs use a pullup resistor and will read high when nothing is connected - its best to connect your input signal to GND and if/wait for the input signal to = 0").grid(row=4, column=0, sticky="w", padx=5, pady=2)





# ### 4 LABELS#################################################################
# #############################################################################
# 
# servo0onequalsLab = Label(tab4, text = "=")
# servo0onequalsLab.place(x=70, y=42)
# 
# servo0offequalsLab = Label(tab4, text = "=")
# servo0offequalsLab.place(x=70, y=82)
# 
# servo1onequalsLab = Label(tab4, text = "=")
# servo1onequalsLab.place(x=70, y=122)
# 
# servo1offequalsLab = Label(tab4, text = "=")
# servo1offequalsLab.place(x=70, y=162)
# 
# servo2onequalsLab = Label(tab4, text = "=")
# servo2onequalsLab.place(x=70, y=202)
# 
# servo2offequalsLab = Label(tab4, text = "=")
# servo2offequalsLab.place(x=70, y=242)
# 
# servo3onequalsLab = Label(tab4, text = "=")
# servo3onequalsLab.place(x=70, y=282)
# 
# servo3offequalsLab = Label(tab4, text = "=")
# servo3offequalsLab.place(x=70, y=322)
# 
# 
# 
# Do1onequalsLab = Label(tab4, text = "=")
# Do1onequalsLab.place(x=210, y=42)
# 
# Do1offequalsLab = Label(tab4, text = "=")
# Do1offequalsLab.place(x=210, y=82)
# 
# Do2onequalsLab = Label(tab4, text = "=")
# Do2onequalsLab.place(x=210, y=122)
# 
# Do2offequalsLab = Label(tab4, text = "=")
# Do2offequalsLab.place(x=210, y=162)
# 
# Do3onequalsLab = Label(tab4, text = "=")
# Do3onequalsLab.place(x=210, y=202)
# 
# Do3offequalsLab = Label(tab4, text = "=")
# Do3offequalsLab.place(x=210, y=242)
# 
# Do4onequalsLab = Label(tab4, text = "=")
# Do4onequalsLab.place(x=210, y=282)
# 
# Do4offequalsLab = Label(tab4, text = "=")
# Do4offequalsLab.place(x=210, y=322)
# 
# Do5onequalsLab = Label(tab4, text = "=")
# Do5onequalsLab.place(x=210, y=362)
# 
# Do5offequalsLab = Label(tab4, text = "=")
# Do5offequalsLab.place(x=210, y=402)
# 
# Do6onequalsLab = Label(tab4, text = "=")
# Do6onequalsLab.place(x=210, y=442)
# 
# Do6offequalsLab = Label(tab4, text = "=")
# Do6offequalsLab.place(x=210, y=482)
# 
# IOboardLab = Label(tab4, font=("Arial 10 bold"), text = "5v IO BOARD")
# IOboardLab.place(x=95, y=10)
# 
# AuxComLab = Label(tab4, font=("Arial 10 bold"), text = "AUX COM DEVICE")
# AuxComLab.place(x=400, y=10)
# 
# ModbusLab = Label(tab4, font=("Arial 10 bold"), text = "MODBUS DEVICE")
# ModbusLab.place(x=700, y=10)
# 
# AuxPortNumLab= Label(tab4, text = "Aux Com Port")
# AuxPortNumLab.place(x=440, y=42)
# 
# AuxPortCharLab= Label(tab4, text = "Char to Read")
# AuxPortCharLab.place(x=440, y=82)
# 
# MBslaveLab= Label(tab4, text = "Slave ID")
# MBslaveLab.place(x=750, y=42)
# 
# MBaddressLab= Label(tab4, text = "Modbus Address")
# MBaddressLab.place(x=750, y=82)
# 
# MBwriteLab= Label(tab4, text = "Operation Value")
# MBwriteLab.place(x=750, y=122)
# 
# MBoutputLab= Label(tab4, text = "Output Response:")
# MBoutputLab.place(x=662, y=405)
# 
# 
# 
# inoutavailLab = Label(tab4, text = "The following IO are available when using the default 5v Nano board for IO:   Inputs = 2-7  /  Outputs = 8-13  /  Servos = A0-A7")
# inoutavailLab.place(x=10, y=640)
# 
# inoutavailLab = Label(tab4, text = "The following IO are available when using the default 5v Mega board for IO:   Inputs = 0-27  /  Outputs = 28-53  /  Servos = A0-A7")
# inoutavailLab.place(x=10, y=655)
# 
# inoutavailLab = Label(tab4, text = "Please review this tutorial video on using 5v IO boards:")
# inoutavailLab.place(x=10, y=670)
# 
# inoutavailLab = Label(tab4, text = "5v board inputs are high impedance and susceptable to floating voltage - inputs use a pullup resistor and will read high when nothing is connected - its best to connect your input signal to GND and if/wait for the input signal to = 0")
# inoutavailLab.place(x=10, y=685)
# 
# 
# 
# link2 = Label(tab4, font=("Arial", 8), text="https://youtu.be/76F6dS4ar8Y?si=Z6NstZy1zNeHgtCF", foreground="blue", cursor="hand2")
# link2.bind("<Button-1>", lambda event: webbrowser.open(link2.cget("text")))
# link2.place(x=300, y=671)
# 
# 
# 
# ### 4 BUTTONS################################################################
# #############################################################################
# 
# servo0onBut = Button(tab4,  text="Servo 0",  command = Servo0on)
# servo0onBut.place(x=10, y=40)
# 
# servo0offBut = Button(tab4,  text="Servo 0",  command = Servo0off)
# servo0offBut.place(x=10, y=80)
# 
# servo1onBut = Button(tab4,  text="Servo 1",  command = Servo1on)
# servo1onBut.place(x=10, y=120)
# 
# servo1offBut = Button(tab4,  text="Servo 1",  command = Servo1off)
# servo1offBut.place(x=10, y=160)
# 
# servo2onBut = Button(tab4,  text="Servo 2",  command = Servo2on)
# servo2onBut.place(x=10, y=200)
# 
# servo2offBut = Button(tab4,  text="Servo 2",  command = Servo2off)
# servo2offBut.place(x=10, y=240)
# 
# servo3onBut = Button(tab4,  text="Servo 3",  command = Servo3on)
# servo3onBut.place(x=10, y=280)
# 
# servo3offBut = Button(tab4,  text="Servo 3",  command = Servo3off)
# servo3offBut.place(x=10, y=320)
# 
# 
# 
# 
# 
# DO1onBut = Button(tab4,  text="DO on",  command = DO1on)
# DO1onBut.place(x=150, y=40)
# 
# DO1offBut = Button(tab4,  text="DO off",  command = DO1off)
# DO1offBut.place(x=150, y=80)
# 
# DO2onBut = Button(tab4,  text="DO on",  command = DO2on)
# DO2onBut.place(x=150, y=120)
# 
# DO2offBut = Button(tab4,  text="DO off",  command = DO2off)
# DO2offBut.place(x=150, y=160)
# 
# DO3onBut = Button(tab4,  text="DO on",  command = DO3on)
# DO3onBut.place(x=150, y=200)
# 
# DO3offBut = Button(tab4,  text="DO off",  command = DO3off)
# DO3offBut.place(x=150, y=240)
# 
# DO4onBut = Button(tab4,  text="DO on",  command = DO4on)
# DO4onBut.place(x=150, y=280)
# 
# DO4offBut = Button(tab4,  text="DO off",  command = DO4off)
# DO4offBut.place(x=150, y=320)
# 
# DO5onBut = Button(tab4,  text="DO on",  command = DO5on)
# DO5onBut.place(x=150, y=360)
# 
# DO5offBut = Button(tab4,  text="DO off",  command = DO5off)
# DO5offBut.place(x=150, y=400)
# 
# DO6onBut = Button(tab4,  text="DO on",  command = DO6on)
# DO6onBut.place(x=150, y=440)
# 
# DO6offBut = Button(tab4,  text="DO off",  command = DO6off)
# DO6offBut.place(x=150, y=480)
# 
# 
# comPortBut3 = Button(tab4,  text="Test Aux COM Device",   command = TestAuxCom)
# comPortBut3.place(x=395, y=120)
# 
# MBreadCoilBut = Button(tab4,  text="Read Coil", width=30, command = MBreadCoil)
# MBreadCoilBut.place(x=665, y=160)
# 
# MBreadDinputBut = Button(tab4,  text="Read Discrete Input", width=30, command = MBreadInput)
# MBreadDinputBut.place(x=665, y=200)
# 
# MBreadHoldRegBut = Button(tab4,  text="Read Holding Register", width=30, command = MBreadHoldReg)
# MBreadHoldRegBut.place(x=665, y=240)
# 
# MBreadInputRegBut = Button(tab4,  text="Read Input Register", width=30, command = MBreadInputReg)
# MBreadInputRegBut.place(x=665, y=280)
# 
# MBwriteCoilBut = Button(tab4,  text="Write Coil", width=30, command = MBwriteCoil)
# MBwriteCoilBut.place(x=665, y=320)
# 
# MBwriteRegBut = Button(tab4,  text="Write Register", width=30, command = MBwriteReg)
# MBwriteRegBut.place(x=665, y=360)
# 
# 
# 
# 
# 
# #### 4 ENTRY FIELDS##########################################################
# #############################################################################
# 
# 
# servo0onEntryField = Entry(tab4,width=4,justify="center")
# servo0onEntryField.place(x=90, y=45)
# 
# servo0offEntryField = Entry(tab4,width=4,justify="center")
# servo0offEntryField.place(x=90, y=85)
# 
# servo1onEntryField = Entry(tab4,width=4,justify="center")
# servo1onEntryField.place(x=90, y=125)
# 
# servo1offEntryField = Entry(tab4,width=4,justify="center")
# servo1offEntryField.place(x=90, y=165)
# 
# servo2onEntryField = Entry(tab4,width=4,justify="center")
# servo2onEntryField.place(x=90, y=205)
# 
# servo2offEntryField = Entry(tab4,width=4,justify="center")
# servo2offEntryField.place(x=90, y=245)
# 
# 
# servo3onEntryField = Entry(tab4,width=4,justify="center")
# servo3onEntryField.place(x=90, y=285)
# 
# servo3offEntryField = Entry(tab4,width=4,justify="center")
# servo3offEntryField.place(x=90, y=325)
# 
# 
# 
# 
# 
# DO1onEntryField = Entry(tab4,width=4,justify="center")
# DO1onEntryField.place(x=230, y=45)
# 
# DO1offEntryField = Entry(tab4,width=4,justify="center")
# DO1offEntryField.place(x=230, y=85)
# 
# DO2onEntryField = Entry(tab4,width=4,justify="center")
# DO2onEntryField.place(x=230, y=125)
# 
# DO2offEntryField = Entry(tab4,width=4,justify="center")
# DO2offEntryField.place(x=230, y=165)
# 
# DO3onEntryField = Entry(tab4,width=4,justify="center")
# DO3onEntryField.place(x=230, y=205)
# 
# DO3offEntryField = Entry(tab4,width=4,justify="center")
# DO3offEntryField.place(x=230, y=245)
# 
# DO4onEntryField = Entry(tab4,width=4,justify="center")
# DO4onEntryField.place(x=230, y=285)
# 
# DO4offEntryField = Entry(tab4,width=4,justify="center")
# DO4offEntryField.place(x=230, y=325)
# 
# DO5onEntryField = Entry(tab4,width=4,justify="center")
# DO5onEntryField.place(x=230, y=365)
# 
# DO5offEntryField = Entry(tab4,width=4,justify="center")
# DO5offEntryField.place(x=230, y=405)
# 
# DO6onEntryField = Entry(tab4,width=4,justify="center")
# DO6onEntryField.place(x=230, y=445)
# 
# DO6offEntryField = Entry(tab4,width=4,justify="center")
# DO6offEntryField.place(x=230, y=485)
# 
# 
# 
# com3PortEntryField = Entry(tab4,width=4,justify="center")
# com3PortEntryField.place(x=400, y=40)
# 
# com3charPortEntryField = Entry(tab4,width=4,justify="center")
# com3charPortEntryField.place(x=400, y=80)
# 
# com3outPortEntryField = Entry(tab4,width=25,justify="center")
# com3outPortEntryField.place(x=385, y=160)
# 
# 
# MBslaveEntryField = Entry(tab4,width=4,justify="center")
# MBslaveEntryField.place(x=710, y=40)
# 
# MBaddressEntryField = Entry(tab4,width=5,justify="center")
# MBaddressEntryField.place(x=690, y=80)
# 
# MBoperValEntryField = Entry(tab4,width=5,justify="center")
# MBoperValEntryField.place(x=690, y=120)
# 
# MBoutputEntryField = Entry(tab4,width=33,justify="center")
# MBoutputEntryField.place(x=662, y=425)
# 
# 
# 
# ####################################################################################################################################################
# ####################################################################################################################################################
# ####################################################################################################################################################
####TAB 5

# ============================================================================
# Tab 5 Grid Layout Configuration
# ============================================================================
tab5.grid_rowconfigure(0, weight=1)
tab5.grid_columnconfigure(0, weight=0, minsize=150)  # Registers
tab5.grid_columnconfigure(1, weight=0, minsize=300)  # Position Registers

# ============================================================================
# Registers Container (Column 0)
# ============================================================================
registersFrame = LabelFrame(tab5, text="Registers", padding=10)
registersFrame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

registersFrame.grid_columnconfigure(0, weight=0)  # Entry field column
registersFrame.grid_columnconfigure(1, weight=1)  # Label column

# R1
R1EntryField = Entry(registersFrame, width=4, justify="center")
R1EntryField.grid(row=0, column=0, padx=2, pady=2)
R1Lab = Label(registersFrame, text="R1")
R1Lab.grid(row=0, column=1, sticky="w", padx=5, pady=2)

# R2
R2EntryField = Entry(registersFrame, width=4, justify="center")
R2EntryField.grid(row=1, column=0, padx=2, pady=2)
R2Lab = Label(registersFrame, text="R2")
R2Lab.grid(row=1, column=1, sticky="w", padx=5, pady=2)

# R3
R3EntryField = Entry(registersFrame, width=4, justify="center")
R3EntryField.grid(row=2, column=0, padx=2, pady=2)
R3Lab = Label(registersFrame, text="R3")
R3Lab.grid(row=2, column=1, sticky="w", padx=5, pady=2)

# R4
R4EntryField = Entry(registersFrame, width=4, justify="center")
R4EntryField.grid(row=3, column=0, padx=2, pady=2)
R4Lab = Label(registersFrame, text="R4")
R4Lab.grid(row=3, column=1, sticky="w", padx=5, pady=2)

# R5
R5EntryField = Entry(registersFrame, width=4, justify="center")
R5EntryField.grid(row=4, column=0, padx=2, pady=2)
R5Lab = Label(registersFrame, text="R5")
R5Lab.grid(row=4, column=1, sticky="w", padx=5, pady=2)

# R6
R6EntryField = Entry(registersFrame, width=4, justify="center")
R6EntryField.grid(row=5, column=0, padx=2, pady=2)
R6Lab = Label(registersFrame, text="R6")
R6Lab.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# R7
R7EntryField = Entry(registersFrame, width=4, justify="center")
R7EntryField.grid(row=6, column=0, padx=2, pady=2)
R7Lab = Label(registersFrame, text="R7")
R7Lab.grid(row=6, column=1, sticky="w", padx=5, pady=2)

# R8
R8EntryField = Entry(registersFrame, width=4, justify="center")
R8EntryField.grid(row=7, column=0, padx=2, pady=2)
R8Lab = Label(registersFrame, text="R8")
R8Lab.grid(row=7, column=1, sticky="w", padx=5, pady=2)

# R9
R9EntryField = Entry(registersFrame, width=4, justify="center")
R9EntryField.grid(row=8, column=0, padx=2, pady=2)
R9Lab = Label(registersFrame, text="R9")
R9Lab.grid(row=8, column=1, sticky="w", padx=5, pady=2)

# R10
R10EntryField = Entry(registersFrame, width=4, justify="center")
R10EntryField.grid(row=9, column=0, padx=2, pady=2)
R10Lab = Label(registersFrame, text="R10")
R10Lab.grid(row=9, column=1, sticky="w", padx=5, pady=2)

# R11
R11EntryField = Entry(registersFrame, width=4, justify="center")
R11EntryField.grid(row=10, column=0, padx=2, pady=2)
R11Lab = Label(registersFrame, text="R11")
R11Lab.grid(row=10, column=1, sticky="w", padx=5, pady=2)

# R12
R12EntryField = Entry(registersFrame, width=4, justify="center")
R12EntryField.grid(row=11, column=0, padx=2, pady=2)
R12Lab = Label(registersFrame, text="R12")
R12Lab.grid(row=11, column=1, sticky="w", padx=5, pady=2)

# R13
R13EntryField = Entry(registersFrame, width=4, justify="center")
R13EntryField.grid(row=12, column=0, padx=2, pady=2)
R13Lab = Label(registersFrame, text="R13")
R13Lab.grid(row=12, column=1, sticky="w", padx=5, pady=2)

# R14
R14EntryField = Entry(registersFrame, width=4, justify="center")
R14EntryField.grid(row=13, column=0, padx=2, pady=2)
R14Lab = Label(registersFrame, text="R14")
R14Lab.grid(row=13, column=1, sticky="w", padx=5, pady=2)

# R15
R15EntryField = Entry(registersFrame, width=4, justify="center")
R15EntryField.grid(row=14, column=0, padx=2, pady=2)
R15Lab = Label(registersFrame, text="R15")
R15Lab.grid(row=14, column=1, sticky="w", padx=5, pady=2)

# R16
R16EntryField = Entry(registersFrame, width=4, justify="center")
R16EntryField.grid(row=15, column=0, padx=2, pady=2)
R16Lab = Label(registersFrame, text="R16")
R16Lab.grid(row=15, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Position Registers Container (Column 1)
# ============================================================================
posRegistersFrame = LabelFrame(tab5, text="Position Registers", padding=10)
posRegistersFrame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

posRegistersFrame.grid_columnconfigure(0, weight=0, minsize=35)  # X
posRegistersFrame.grid_columnconfigure(1, weight=0, minsize=35)  # Y
posRegistersFrame.grid_columnconfigure(2, weight=0, minsize=35)  # Z
posRegistersFrame.grid_columnconfigure(3, weight=0, minsize=35)  # Rz
posRegistersFrame.grid_columnconfigure(4, weight=0, minsize=35)  # Ry
posRegistersFrame.grid_columnconfigure(5, weight=0, minsize=35)  # Rx
posRegistersFrame.grid_columnconfigure(6, weight=0, minsize=40)  # PR label

# Header row
Label(posRegistersFrame, text="X").grid(row=0, column=0, padx=1, pady=2)
Label(posRegistersFrame, text="Y").grid(row=0, column=1, padx=1, pady=2)
Label(posRegistersFrame, text="Z").grid(row=0, column=2, padx=1, pady=2)
Label(posRegistersFrame, text="Rz").grid(row=0, column=3, padx=1, pady=2)
Label(posRegistersFrame, text="Ry").grid(row=0, column=4, padx=1, pady=2)
Label(posRegistersFrame, text="Rx").grid(row=0, column=5, padx=1, pady=2)

# PR1
SP_1_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E1_EntryField.grid(row=1, column=0, padx=1, pady=2)
SP_1_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E2_EntryField.grid(row=1, column=1, padx=1, pady=2)
SP_1_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E3_EntryField.grid(row=1, column=2, padx=1, pady=2)
SP_1_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E4_EntryField.grid(row=1, column=3, padx=1, pady=2)
SP_1_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E5_EntryField.grid(row=1, column=4, padx=1, pady=2)
SP_1_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E6_EntryField.grid(row=1, column=5, padx=1, pady=2)
SP1Lab = Label(posRegistersFrame, text="PR1")
SP1Lab.grid(row=1, column=6, sticky="w", padx=2, pady=2)

# PR2
SP_2_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E1_EntryField.grid(row=2, column=0, padx=1, pady=2)
SP_2_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E2_EntryField.grid(row=2, column=1, padx=1, pady=2)
SP_2_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E3_EntryField.grid(row=2, column=2, padx=1, pady=2)
SP_2_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E4_EntryField.grid(row=2, column=3, padx=1, pady=2)
SP_2_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E5_EntryField.grid(row=2, column=4, padx=1, pady=2)
SP_2_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E6_EntryField.grid(row=2, column=5, padx=1, pady=2)
SP2Lab = Label(posRegistersFrame, text="PR2")
SP2Lab.grid(row=2, column=6, sticky="w", padx=2, pady=2)

# PR3
SP_3_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E1_EntryField.grid(row=3, column=0, padx=1, pady=2)
SP_3_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E2_EntryField.grid(row=3, column=1, padx=1, pady=2)
SP_3_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E3_EntryField.grid(row=3, column=2, padx=1, pady=2)
SP_3_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E4_EntryField.grid(row=3, column=3, padx=1, pady=2)
SP_3_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E5_EntryField.grid(row=3, column=4, padx=1, pady=2)
SP_3_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E6_EntryField.grid(row=3, column=5, padx=1, pady=2)
SP3Lab = Label(posRegistersFrame, text="PR3")
SP3Lab.grid(row=3, column=6, sticky="w", padx=2, pady=2)

# PR4
SP_4_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E1_EntryField.grid(row=4, column=0, padx=1, pady=2)
SP_4_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E2_EntryField.grid(row=4, column=1, padx=1, pady=2)
SP_4_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E3_EntryField.grid(row=4, column=2, padx=1, pady=2)
SP_4_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E4_EntryField.grid(row=4, column=3, padx=1, pady=2)
SP_4_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E5_EntryField.grid(row=4, column=4, padx=1, pady=2)
SP_4_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E6_EntryField.grid(row=4, column=5, padx=1, pady=2)
SP4Lab = Label(posRegistersFrame, text="PR4")
SP4Lab.grid(row=4, column=6, sticky="w", padx=2, pady=2)

# PR5
SP_5_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E1_EntryField.grid(row=5, column=0, padx=1, pady=2)
SP_5_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E2_EntryField.grid(row=5, column=1, padx=1, pady=2)
SP_5_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E3_EntryField.grid(row=5, column=2, padx=1, pady=2)
SP_5_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E4_EntryField.grid(row=5, column=3, padx=1, pady=2)
SP_5_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E5_EntryField.grid(row=5, column=4, padx=1, pady=2)
SP_5_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E6_EntryField.grid(row=5, column=5, padx=1, pady=2)
SP5Lab = Label(posRegistersFrame, text="PR5")
SP5Lab.grid(row=5, column=6, sticky="w", padx=2, pady=2)

# PR6
SP_6_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E1_EntryField.grid(row=6, column=0, padx=1, pady=2)
SP_6_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E2_EntryField.grid(row=6, column=1, padx=1, pady=2)
SP_6_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E3_EntryField.grid(row=6, column=2, padx=1, pady=2)
SP_6_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E4_EntryField.grid(row=6, column=3, padx=1, pady=2)
SP_6_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E5_EntryField.grid(row=6, column=4, padx=1, pady=2)
SP_6_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E6_EntryField.grid(row=6, column=5, padx=1, pady=2)
SP6Lab = Label(posRegistersFrame, text="PR6")
SP6Lab.grid(row=6, column=6, sticky="w", padx=2, pady=2)

# PR7
SP_7_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E1_EntryField.grid(row=7, column=0, padx=1, pady=2)
SP_7_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E2_EntryField.grid(row=7, column=1, padx=1, pady=2)
SP_7_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E3_EntryField.grid(row=7, column=2, padx=1, pady=2)
SP_7_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E4_EntryField.grid(row=7, column=3, padx=1, pady=2)
SP_7_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E5_EntryField.grid(row=7, column=4, padx=1, pady=2)
SP_7_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E6_EntryField.grid(row=7, column=5, padx=1, pady=2)
SP7Lab = Label(posRegistersFrame, text="PR7")
SP7Lab.grid(row=7, column=6, sticky="w", padx=2, pady=2)

# PR8
SP_8_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E1_EntryField.grid(row=8, column=0, padx=1, pady=2)
SP_8_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E2_EntryField.grid(row=8, column=1, padx=1, pady=2)
SP_8_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E3_EntryField.grid(row=8, column=2, padx=1, pady=2)
SP_8_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E4_EntryField.grid(row=8, column=3, padx=1, pady=2)
SP_8_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E5_EntryField.grid(row=8, column=4, padx=1, pady=2)
SP_8_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E6_EntryField.grid(row=8, column=5, padx=1, pady=2)
SP8Lab = Label(posRegistersFrame, text="PR8")
SP8Lab.grid(row=8, column=6, sticky="w", padx=2, pady=2)

# PR9
SP_9_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E1_EntryField.grid(row=9, column=0, padx=1, pady=2)
SP_9_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E2_EntryField.grid(row=9, column=1, padx=1, pady=2)
SP_9_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E3_EntryField.grid(row=9, column=2, padx=1, pady=2)
SP_9_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E4_EntryField.grid(row=9, column=3, padx=1, pady=2)
SP_9_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E5_EntryField.grid(row=9, column=4, padx=1, pady=2)
SP_9_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E6_EntryField.grid(row=9, column=5, padx=1, pady=2)
SP9Lab = Label(posRegistersFrame, text="PR9")
SP9Lab.grid(row=9, column=6, sticky="w", padx=2, pady=2)

# PR10
SP_10_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E1_EntryField.grid(row=10, column=0, padx=1, pady=2)
SP_10_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E2_EntryField.grid(row=10, column=1, padx=1, pady=2)
SP_10_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E3_EntryField.grid(row=10, column=2, padx=1, pady=2)
SP_10_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E4_EntryField.grid(row=10, column=3, padx=1, pady=2)
SP_10_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E5_EntryField.grid(row=10, column=4, padx=1, pady=2)
SP_10_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E6_EntryField.grid(row=10, column=5, padx=1, pady=2)
SP10Lab = Label(posRegistersFrame, text="PR10")
SP10Lab.grid(row=10, column=6, sticky="w", padx=2, pady=2)

# PR11
SP_11_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E1_EntryField.grid(row=11, column=0, padx=1, pady=2)
SP_11_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E2_EntryField.grid(row=11, column=1, padx=1, pady=2)
SP_11_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E3_EntryField.grid(row=11, column=2, padx=1, pady=2)
SP_11_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E4_EntryField.grid(row=11, column=3, padx=1, pady=2)
SP_11_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E5_EntryField.grid(row=11, column=4, padx=1, pady=2)
SP_11_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E6_EntryField.grid(row=11, column=5, padx=1, pady=2)
SP11Lab = Label(posRegistersFrame, text="PR11")
SP11Lab.grid(row=11, column=6, sticky="w", padx=2, pady=2)

# PR12
SP_12_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E1_EntryField.grid(row=12, column=0, padx=1, pady=2)
SP_12_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E2_EntryField.grid(row=12, column=1, padx=1, pady=2)
SP_12_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E3_EntryField.grid(row=12, column=2, padx=1, pady=2)
SP_12_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E4_EntryField.grid(row=12, column=3, padx=1, pady=2)
SP_12_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E5_EntryField.grid(row=12, column=4, padx=1, pady=2)
SP_12_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E6_EntryField.grid(row=12, column=5, padx=1, pady=2)
SP12Lab = Label(posRegistersFrame, text="PR12")
SP12Lab.grid(row=12, column=6, sticky="w", padx=2, pady=2)

# PR13
SP_13_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E1_EntryField.grid(row=13, column=0, padx=1, pady=2)
SP_13_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E2_EntryField.grid(row=13, column=1, padx=1, pady=2)
SP_13_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E3_EntryField.grid(row=13, column=2, padx=1, pady=2)
SP_13_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E4_EntryField.grid(row=13, column=3, padx=1, pady=2)
SP_13_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E5_EntryField.grid(row=13, column=4, padx=1, pady=2)
SP_13_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E6_EntryField.grid(row=13, column=5, padx=1, pady=2)
SP13Lab = Label(posRegistersFrame, text="PR13")
SP13Lab.grid(row=13, column=6, sticky="w", padx=2, pady=2)

# PR14
SP_14_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E1_EntryField.grid(row=14, column=0, padx=1, pady=2)
SP_14_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E2_EntryField.grid(row=14, column=1, padx=1, pady=2)
SP_14_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E3_EntryField.grid(row=14, column=2, padx=1, pady=2)
SP_14_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E4_EntryField.grid(row=14, column=3, padx=1, pady=2)
SP_14_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E5_EntryField.grid(row=14, column=4, padx=1, pady=2)
SP_14_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E6_EntryField.grid(row=14, column=5, padx=1, pady=2)
SP14Lab = Label(posRegistersFrame, text="PR14")
SP14Lab.grid(row=14, column=6, sticky="w", padx=2, pady=2)

# PR15
SP_15_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E1_EntryField.grid(row=15, column=0, padx=1, pady=2)
SP_15_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E2_EntryField.grid(row=15, column=1, padx=1, pady=2)
SP_15_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E3_EntryField.grid(row=15, column=2, padx=1, pady=2)
SP_15_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E4_EntryField.grid(row=15, column=3, padx=1, pady=2)
SP_15_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E5_EntryField.grid(row=15, column=4, padx=1, pady=2)
SP_15_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E6_EntryField.grid(row=15, column=5, padx=1, pady=2)
SP15Lab = Label(posRegistersFrame, text="PR15")
SP15Lab.grid(row=15, column=6, sticky="w", padx=2, pady=2)

# PR16
SP_16_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E1_EntryField.grid(row=16, column=0, padx=1, pady=2)
SP_16_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E2_EntryField.grid(row=16, column=1, padx=1, pady=2)
SP_16_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E3_EntryField.grid(row=16, column=2, padx=1, pady=2)
SP_16_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E4_EntryField.grid(row=16, column=3, padx=1, pady=2)
SP_16_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E5_EntryField.grid(row=16, column=4, padx=1, pady=2)
SP_16_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E6_EntryField.grid(row=16, column=5, padx=1, pady=2)
SP16Lab = Label(posRegistersFrame, text="PR16")
SP16Lab.grid(row=16, column=6, sticky="w", padx=2, pady=2)


####TAB 6




### 6 LABELS#################################################################
#############################################################################


#VisBackdropImg = ImageTk.PhotoImage(Image.open('VisBackdrop.png'))

VisBackdropImg = Image.open("VisBackdrop.png")
VisBackdropTk = ImageTk.PhotoImage(VisBackdropImg)
VisBackdromLbl = Label(tab6, image = VisBackdropTk)
VisBackdromLbl.place(x=15, y=215)


#RUN['cam_on']= cv2.VideoCapture(0)
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
      allcams = graph.get_input_devices()
    except Exception:
      allcams = ["Select a Camera"]

    # --- filter out virtual/fake cameras (case-insensitive) ---
    _ban = ("iriun", "droidcam", "obs", "virtual", "manycam", "snap camera", "ndi", "xsplit", "mmhmm")
    camList = [n for n in allcams if not any(b in n.lower() for b in _ban)]
    if not camList:
      camList = allcams[:]  # fallback if everything got filtered
    # ----------------------------------------------------------

    visoptions = StringVar(tab6)
    visoptions.set("Select a Camera")

    try:
      # If we have real cams, preselect the first real one; otherwise keep the placeholder
      if camList and camList[0] != "Select a Camera":
        vismenu = OptionMenu(tab6, visoptions, camList[0], *camList)
        visoptions.set(camList[0])  # ensures the real cam (e.g., Logi C270) is selected
      else:
        vismenu = OptionMenu(tab6, visoptions, "Select a Camera")
      vismenu.config(width=20)
      vismenu.place(x=10, y=10)
    except Exception:
      logger.error("no camera")

  case "Linux":
    # Build label->id map
    label_to_id = {c.label: c.id for c in CE['Cameras']['Enum']}
    camList = list(label_to_id.keys())

    selected_label = tk.StringVar(value=(camList[0] if camList else "Select a Camera"))

    visoptions = StringVar(tab6)
    visoptions.set("Select a Camera")

    def on_camera_select(chosen_label):
      cam_id = label_to_id.get(chosen_label, "None")
      logger.debug("Debug - User picked:", chosen_label, " -> using id:", cam_id)

    try:
      vismenu = OptionMenu(tab6, visoptions, selected_label.get(), *camList, command=on_camera_select)
      vismenu.config(width=20)
      vismenu.place(x=10, y=10)
    except Exception:
      logger.error("no camera")





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


fullRotCbut = Checkbutton(tab6, text="Full Rotation Search",variable = RUN['fullRot'])
fullRotCbut.place(x=900, y=255)

pick180Cbut = Checkbutton(tab6, text="Pick Closest 180°",variable = RUN['pick180'])
pick180Cbut.place(x=900, y=275)

pickClosestCbut = Checkbutton(tab6, text="Try Closest When Out of Range",variable = RUN['pickClosest'])
pickClosestCbut.place(x=900, y=295)





saveCalBut = Button(tab6,  text="SAVE VISION DATA",  width=26, command = SaveAndApplyCalibration)
saveCalBut.place(x=915, y=340)





#### 6 ENTRY FIELDS##########################################################
#############################################################################



VisBacColorEntryField = Entry(tab6,width=12,justify="center")
VisBacColorEntryField.place(x=390, y=100)
VisBacColorLab = Label(tab6, text = "Background Color")
VisBacColorLab.place(x=390, y=120)

bgAutoCbut = Checkbutton(tab6, command=checkAutoBG, text="Auto",variable = RUN['autoBG'])
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


loaded_calibration = load_calibration()
apply_calibration(loaded_calibration, CAL)

logger.debug(f"Comport 1 restored value is: {CAL['comPort']}")
logger.debug(f"Comport 1 restored value is: {CAL['com2Port']}")

if CAL['comPort'] in port_choices:
  com1SelectedValue.set(CAL['comPort'])

if CAL['com2Port'] in port_choices:
  com2SelectedValue.set(CAL['com2Port'])


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
servo0onEntryField.insert(0,str(CAL['Servo0on']))
servo0offEntryField.insert(0,str(CAL['Servo0off']))
servo1onEntryField.insert(0,str(CAL['Servo1on']))
servo1offEntryField.insert(0,str(CAL['Servo1off']))
DO1onEntryField.insert(0,str(CAL['DO1on']))
DO1offEntryField.insert(0,str(CAL['DO1off']))
DO2onEntryField.insert(0,str(CAL['DO2on']))
DO2offEntryField.insert(0,str(CAL['DO2off']))
TFxEntryField.insert(0,str(CAL['TFx']))
TFyEntryField.insert(0,str(CAL['TFy']))
TFzEntryField.insert(0,str(CAL['TFz']))
TFrxEntryField.insert(0,str(CAL['TFrx']))
TFryEntryField.insert(0,str(CAL['TFry']))
TFrzEntryField.insert(0,str(CAL['TFrz']))
J7curAngEntryField.insert(0,str(CAL['J7PosCur']))
J8curAngEntryField.insert(0,str(CAL['J8PosCur']))
J9curAngEntryField.insert(0,str(CAL['J9PosCur']))
J1calOffEntryField.insert(0,str(CAL['J1calOff']))
J2calOffEntryField.insert(0,str(CAL['J2calOff']))
J3calOffEntryField.insert(0,str(CAL['J3calOff']))
J4calOffEntryField.insert(0,str(CAL['J4calOff']))
J5calOffEntryField.insert(0,str(CAL['J5calOff']))
J6calOffEntryField.insert(0,str(CAL['J6calOff']))
J7calOffEntryField.insert(0,str(CAL['J7calOff']))
J8calOffEntryField.insert(0,str(CAL['J8calOff']))
J9calOffEntryField.insert(0,str(CAL['J9calOff']))

if (CAL['curTheme'] == 1): 
  lightTheme()
else:
  darkTheme()
'''
if (CAL['J1CalStatVal'] == 1):
  RUN['J1CalStat1'].set(True)
if (CAL['J2CalStatVal'] == 1):
  RUN['J2CalStat1'].set(True)
if (CAL['J3CalStatVal'] == 1):
  RUN['J3CalStat1'].set(True)
if (CAL['J4CalStatVal'] == 1):
  RUN['J4CalStat1'].set(True)
if (CAL['J5CalStatVal'] == 1):
  RUN['J5CalStat1'].set(True)
if (CAL['J6CalStatVal'] == 1):
  RUN['J6CalStat1'].set(True)
if (CAL['J7CalStatVal'] == 1):
  RUN['J7CalStat1'].set(True) 
if (CAL['J8CalStatVal'] == 1):
  RUN['J8CalStat1'].set(True) 
if (CAL['J9CalStatVal'] == 1):
  RUN['J9CalStat1'].set(True)         
if (CAL['J1CalStatVal2'] == 1):
  RUN['J1CalStat2'].set(True)
if (CAL['J2CalStatVal2'] == 1):
  RUN['J2CalStat2'].set(True)
if (CAL['J3CalStatVal2'] == 1):
  RUN['J3CalStat2'].set(True)
if (CAL['J4CalStatVal2'] == 1):
  RUN['J4CalStat2'].set(True)
if (CAL['J5CalStatVal2'] == 1):
  RUN['J5CalStat2'].set(True)
if (CAL['J6CalStatVal2'] == 1):
  RUN['J6CalStat2'].set(True)
if (CAL['J7CalStatVal2'] == 1):
  RUN['J7CalStat2'].set(True) 
if (CAL['J8CalStatVal2'] == 1):
  RUN['J8CalStat2'].set(True) 
if (CAL['J9CalStatVal2'] == 1):
  RUN['J9CalStat2'].set(True)
'''          
axis7lengthEntryField.insert(0,str(CAL['J7PosLim']))
axis7rotEntryField.insert(0,str(CAL['J7rotation']))
axis7stepsEntryField.insert(0,str(CAL['J7steps']))
VisBrightSlide.set(CAL['VisBrightVal'])
VisContrastSlide.set(CAL['VisContVal'])
VisBacColorEntryField.insert(0,str(CAL['VisBacColor']))
VisScoreEntryField.insert(0,str(CAL['VisScore']))
VisX1PixEntryField.insert(0,str(CAL['VisX1Val']))
VisY1PixEntryField.insert(0,str(CAL['VisY1Val']))
VisX2PixEntryField.insert(0,str(CAL['VisX2Val']))
VisY2PixEntryField.insert(0,str(CAL['VisY2Val']))
VisX1RobEntryField.insert(0,str(CAL['VisRobX1Val']))
VisY1RobEntryField.insert(0,str(CAL['VisRobY1Val']))
VisX2RobEntryField.insert(0,str(CAL['VisRobX2Val']))
VisY2RobEntryField.insert(0,str(CAL['VisRobY2Val']))
VisZoomSlide.set(CAL['zoom'])
if (CAL['pickClosestVal'] == 1):
  RUN['pickClosest'].set(True)
if (CAL['pick180Val'] == 1):
  RUN['pick180'].set(True)  
visoptions.set(CAL['curCam'])
if (CAL['fullRotVal'] == 1):
  RUN['fullRot'].set(True)
if (CAL['autoBGVal'] == 1):
  RUN['autoBG'].set(True)  
RUN['mX1'] = CAL['mX1val']
RUN['mY1'] = CAL['mY1val']
RUN['mX2'] = CAL['mX2val']
RUN['mY2'] = CAL['mY2val']
axis8lengthEntryField.insert(0,str(CAL['J8length']))
axis8rotEntryField.insert(0,str(CAL['J8rotation']))
axis8stepsEntryField.insert(0,str(CAL['J8steps']))
axis9lengthEntryField.insert(0,str(CAL['J9length']))
axis9rotEntryField.insert(0,str(CAL['J9rotation']))
axis9stepsEntryField.insert(0,str(CAL['J9steps']))
GC_ST_E1_EntryField.insert(0,str(CAL['GC_ST_E1']))
GC_ST_E2_EntryField.insert(0,str(CAL['GC_ST_E2']))
GC_ST_E3_EntryField.insert(0,str(CAL['GC_ST_E3']))
GC_ST_E4_EntryField.insert(0,str(CAL['GC_ST_E4']))
GC_ST_E5_EntryField.insert(0,str(CAL['GC_ST_E5']))
GC_ST_E6_EntryField.insert(0,str(CAL['GC_ST_E6']))
GC_ST_WC_EntryField.insert(0,str(CAL['GC_ST_WC']))
GC_SToff_E1_EntryField.insert(0,str(CAL['GC_SToff_E1']))
GC_SToff_E2_EntryField.insert(0,str(CAL['GC_SToff_E2']))
GC_SToff_E3_EntryField.insert(0,str(CAL['GC_SToff_E3']))
GC_SToff_E4_EntryField.insert(0,str(CAL['GC_SToff_E4']))
GC_SToff_E5_EntryField.insert(0,str(CAL['GC_SToff_E5']))
GC_SToff_E6_EntryField.insert(0,str(CAL['GC_SToff_E6']))
J1MotDirEntryField.insert(0,str(CAL['J1MotDir']))
J2MotDirEntryField.insert(0,str(CAL['J2MotDir']))
J3MotDirEntryField.insert(0,str(CAL['J3MotDir']))
J4MotDirEntryField.insert(0,str(CAL['J4MotDir']))
J5MotDirEntryField.insert(0,str(CAL['J5MotDir']))
J6MotDirEntryField.insert(0,str(CAL['J6MotDir']))
J7MotDirEntryField.insert(0,str(CAL['J7MotDir']))
J8MotDirEntryField.insert(0,str(CAL['J8MotDir']))
J9MotDirEntryField.insert(0,str(CAL['J9MotDir']))
J1CalDirEntryField.insert(0,str(CAL['J1CalDir']))
J2CalDirEntryField.insert(0,str(CAL['J2CalDir']))
J3CalDirEntryField.insert(0,str(CAL['J3CalDir']))
J4CalDirEntryField.insert(0,str(CAL['J4CalDir']))
J5CalDirEntryField.insert(0,str(CAL['J5CalDir']))
J6CalDirEntryField.insert(0,str(CAL['J6CalDir']))
J7CalDirEntryField.insert(0,str(CAL['J7CalDir']))
J8CalDirEntryField.insert(0,str(CAL['J8CalDir']))
J9CalDirEntryField.insert(0,str(CAL['J9CalDir']))
J1PosLimEntryField.insert(0,str(CAL['J1PosLim']))
J1NegLimEntryField.insert(0,str(CAL['J1NegLim']))
J2PosLimEntryField.insert(0,str(CAL['J2PosLim']))
J2NegLimEntryField.insert(0,str(CAL['J2NegLim']))
J3PosLimEntryField.insert(0,str(CAL['J3PosLim']))
J3NegLimEntryField.insert(0,str(CAL['J3NegLim']))
J4PosLimEntryField.insert(0,str(CAL['J4PosLim']))
J4NegLimEntryField.insert(0,str(CAL['J4NegLim']))
J5PosLimEntryField.insert(0,str(CAL['J5PosLim']))
J5NegLimEntryField.insert(0,str(CAL['J5NegLim']))
J6PosLimEntryField.insert(0,str(CAL['J6PosLim']))
J6NegLimEntryField.insert(0,str(CAL['J6NegLim']))  
J1StepDegEntryField.insert(0,str(CAL['J1StepDeg']))
J2StepDegEntryField.insert(0,str(CAL['J2StepDeg'])) 
J3StepDegEntryField.insert(0,str(CAL['J3StepDeg'])) 
J4StepDegEntryField.insert(0,str(CAL['J4StepDeg'])) 
J5StepDegEntryField.insert(0,str(CAL['J5StepDeg'])) 
J6StepDegEntryField.insert(0,str(CAL['J6StepDeg']))
J1DriveMSEntryField.insert(0,str(CAL['J1DriveMS']))
J2DriveMSEntryField.insert(0,str(CAL['J2DriveMS']))  
J3DriveMSEntryField.insert(0,str(CAL['J3DriveMS']))  
J4DriveMSEntryField.insert(0,str(CAL['J4DriveMS']))  
J5DriveMSEntryField.insert(0,str(CAL['J5DriveMS']))  
J6DriveMSEntryField.insert(0,str(CAL['J6DriveMS']))
J1EncCPREntryField.insert(0,str(CAL['J1EncCPR']))
J2EncCPREntryField.insert(0,str(CAL['J2EncCPR']))
J3EncCPREntryField.insert(0,str(CAL['J3EncCPR']))
J4EncCPREntryField.insert(0,str(CAL['J4EncCPR']))
J5EncCPREntryField.insert(0,str(CAL['J5EncCPR']))
J6EncCPREntryField.insert(0,str(CAL['J6EncCPR']))
J1ΘEntryField.insert(0,str(CAL['J1ΘDHpar']))
J2ΘEntryField.insert(0,str(CAL['J2ΘDHpar']))
J3ΘEntryField.insert(0,str(CAL['J3ΘDHpar']))
J4ΘEntryField.insert(0,str(CAL['J4ΘDHpar']))
J5ΘEntryField.insert(0,str(CAL['J5ΘDHpar']))
J6ΘEntryField.insert(0,str(CAL['J6ΘDHpar']))
J1αEntryField.insert(0,str(CAL['J1αDHpar']))
J2αEntryField.insert(0,str(CAL['J2αDHpar']))
J3αEntryField.insert(0,str(CAL['J3αDHpar']))
J4αEntryField.insert(0,str(CAL['J4αDHpar']))
J5αEntryField.insert(0,str(CAL['J5αDHpar']))
J6αEntryField.insert(0,str(CAL['J6αDHpar']))
J1dEntryField.insert(0,str(CAL['J1dDHpar']))
J2dEntryField.insert(0,str(CAL['J2dDHpar']))
J3dEntryField.insert(0,str(CAL['J3dDHpar']))
J4dEntryField.insert(0,str(CAL['J4dDHpar']))
J5dEntryField.insert(0,str(CAL['J5dDHpar']))
J6dEntryField.insert(0,str(CAL['J6dDHpar']))
J1aEntryField.insert(0,str(CAL['J1aDHpar']))
J2aEntryField.insert(0,str(CAL['J2aDHpar']))
J3aEntryField.insert(0,str(CAL['J3aDHpar']))
J4aEntryField.insert(0,str(CAL['J4aDHpar']))
J5aEntryField.insert(0,str(CAL['J5aDHpar']))
J6aEntryField.insert(0,str(CAL['J6aDHpar']))

update_CPP_kin_from_entries()
RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
RUN['JangleOut'] = np.array([float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])])
RUN['negLim'] = [float(CAL['J1NegLim']), float(CAL['J2NegLim']), float(CAL['J3NegLim']), float(CAL['J4NegLim']), float(CAL['J5NegLim']), float(CAL['J6NegLim'])]

#axis limits in each direction
RUN['J1axisLimNeg'] = float(CAL['J1NegLim'])
RUN['J2axisLimNeg'] = float(CAL['J2NegLim'])
RUN['J3axisLimNeg'] = float(CAL['J3NegLim'])
RUN['J4axisLimNeg'] = float(CAL['J4NegLim'])
RUN['J5axisLimNeg'] = float(CAL['J5NegLim'])
RUN['J6axisLimNeg'] = float(CAL['J6NegLim'])
J1axisLimPos = float(CAL['J1PosLim'])
J2axisLimPos = float(CAL['J2PosLim'])
J3axisLimPos = float(CAL['J3PosLim'])
J4axisLimPos = float(CAL['J4PosLim'])
J5axisLimPos = float(CAL['J5PosLim'])
J6axisLimPos = float(CAL['J6PosLim'])


#degrees full movement of each axis
J1axisLim = J1axisLimPos + RUN['J1axisLimNeg'];
J2axisLim = J2axisLimPos + RUN['J2axisLimNeg'];
J3axisLim = J3axisLimPos + RUN['J3axisLimNeg'];
J4axisLim = J4axisLimPos + RUN['J4axisLimNeg'];
J5axisLim = J5axisLimPos + RUN['J5axisLimNeg'];
J6axisLim = J6axisLimPos + RUN['J6axisLimNeg'];
#steps full movement of each axis
J1StepLim = J1axisLim * float(CAL['J1StepDeg'])
J2StepLim = J2axisLim * float(CAL['J2StepDeg'])
J3StepLim = J3axisLim * float(CAL['J3StepDeg'])
J4StepLim = J4axisLim * float(CAL['J4StepDeg'])
J5StepLim = J5axisLim * float(CAL['J5StepDeg'])
J6StepLim = J6axisLim * float(CAL['J6StepDeg'])
RUN['stepDeg'] = [float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']), float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])]
setStepMonitorsVR()
main_color_var.set(CAL['setColor'])




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
RUN['xboxUse'] = 0


tab1.after(100, setCom)

#tab1.mainloop()
root.mainloop()



#manEntryField.delete(0, 'end')
#manEntryField.insert(0,value)
