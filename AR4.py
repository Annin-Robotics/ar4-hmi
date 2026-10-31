#!/usr/bin/env python

############################################################################
## Version AR4 8.1                                         ############
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
  VERSION 6.4 1/3/26 MK4 update, fixed tool jog, re-added 2 step calibration, add servo amp test
  VERSION 6.4.1 1/24/26 added path resolve for sys.executable
  VERSION 6.5 2/15/26 - gcode bug fix / update program flow with run once and run program loop
  VERSION 6.6 2/22/26 - update kinematic solver to reduce J4/6 wrap | reimplement wrist N/F config
  VERSION 6.7 3/11/26 - fix MB read hold reg bug
  VERSION 6.8 3/31/26 - MK5 update - hall effect cal high low
  VERSION 7.0 4/5/26 - Teensy CMD json update
  VERSION 8.0 10/7/26 - AR4/TST unification Phase 2: model reboot handling, reconnect flow (pairs with firmware 8.0)
                        motion streaming: consecutive motion rows run without pauses, Rnd blends Move L corners,
                        Start/End Spline rows no longer required, soft stop (pairs with firmware 8.0)
  VERSION 8.1 10/30/26 - kinematics: IK solves at the wrist singularity (J5 = 0 found no solution), accurate J4/J6
                        near J5 = 0, no 360 deg J4/J6 wrap at +/-180, jump guard for interpolated moves and live jogs,
                        Cartesian Rx/Ry/Rz jogs rotate about the world axes (pairs with firmware 8.1). Pure-Python
                        kinematics fallback when the compiled module is missing or stale. Splash runs in its own
                        process (keeps animating while loading / calibrating). Linux COM list shows USB ports only.
                        File dialogs, CAD deselect-on-click and the viewer no longer fight the rest of the UI.
'''
##########################################################################

import sys

if len(sys.argv) > 1 and sys.argv[1] == "--splash-child":
    # ar4_ui starts a second copy of this program (the .exe, when frozen) to show the splash in
    # its own process so the animation keeps running while this one is busy. Nothing below
    # this point is imported for that copy.
    import ar4_splash
    sys.exit(ar4_splash.main(sys.argv[2:]))

import os
import pathlib
from datetime import datetime

# CROSS-PLATFORM PATCH v1
# Resolve the app folder and chdir before anything opens a relative path (STLs, icons,
# config, CAD folders), so the program works no matter where it is launched from.
if getattr(sys, "frozen", False):
    DIR = pathlib.Path(sys.executable).resolve().parent
else:
    DIR = pathlib.Path(__file__).resolve().parent
os.chdir(DIR)
IS_WINDOWS = sys.platform.startswith("win")

from ARrobots.ARrobot import ARRobot, ARNano

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
          lines = str(message).splitlines() or [""]
          rows = [f"{Curtime} - {lines[0]}"] + ["      " + l for l in lines[1:]]
          tab8.ElogView.after(0, lambda: (tab8.ElogView.insert(END, *rows), tab8.ElogView.see(END)))
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



import tkinter as tk
from tkinter import ttk
from tkinter import simpledialog, messagebox, Misc
from tkinter import filedialog as fd
from tkinter.constants import *
from tkinter import Listbox

import ttkbootstrap
import ttkbootstrap as ttk_bootstrap          # ← keeps the alias line 248 needs
from ttkbootstrap import *                    # ← overrides LabelFrame etc. last
from ttkbootstrap import Style as BootstrapStyle

LabelFrame = ttk.LabelFrame

# MODERN UI PATCH v1
import ar4_ui as ui   # themes, fonts, header bar, splash (see ar4_ui.py / assets/)

import tkinter.messagebox
from PIL import Image, ImageTk

import vtk
try:
  from vtkmodules.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor  # unused, optional
except Exception:
  vtkTkRenderWindowInteractor = None
import vtkmodules.vtkInteractionStyle as vtkIS

import webbrowser
import cv2

import re

# Kinematics: the compiled module (ARrobots/src, built per platform) when it is current,
# otherwise the pure-Python twin ARrobots/robot_kinematics_py.py (same functions, same
# answers, slower - ~40 us per IK call, which is still fine for the virtual robot).
_KIN_API_REQUIRED = 2
try:
  import ARrobots.robot_kinematics as kinematics
  if getattr(kinematics, "API_VERSION", 0) < _KIN_API_REQUIRED:
    raise ImportError(
      f"compiled robot_kinematics is API {getattr(kinematics, 'API_VERSION', 0)}, "
      f"source is API {_KIN_API_REQUIRED} (rebuild ARrobots/src or use the fallback)")
  logger.debug("Using compiled kinematics module")
except Exception as _kin_err:
  import ARrobots.robot_kinematics_py as kinematics
  logger.warning(f"Compiled kinematics module not used ({_kin_err}); using the pure-Python solver")
from ARrobots.Calibration import load_calibration, save_calibration
from ARrobots.HMI.Calibration import apply_calibration
import json

from ARrobots.ARrobot import ARRobot
robot = ARRobot()

print(robot)

#####################################################################################
# Cross-Compat Patch
# We need platform awareness, port enumeration, and some typing imports

from pathlib import Path
import platform
from serial.tools import list_ports
from typing import List

from ARrobots.AR4config import AR4_Configuration

import shutil

# ---- Virtual Import / Tool Import CAD folders (from V7) ----
CAD_IMPORTED_DIR = os.path.join(os.getcwd(), "ARrobots", "CAD Imported")
CAD_IMPORTED_SCENE_FILE = os.path.join(CAD_IMPORTED_DIR, "imported_cad_scene.json")
CAD_SCENE_FILE = CAD_IMPORTED_SCENE_FILE

CAD_EOAT_DIR = os.path.join(os.getcwd(), "ARrobots", "CAD EOAT")
CAD_EOAT_SCENE_FILE = os.path.join(CAD_EOAT_DIR, "eoat_scene.json")

os.makedirs(CAD_IMPORTED_DIR, exist_ok=True)
os.makedirs(CAD_EOAT_DIR, exist_ok=True)

imported_actors = {}  # filename -> actor
selected_imported_filename = None

global Config, CE, CAL, RUN
Config = AR4_Configuration()
CE = Config.Environment
CAL = Config.Calibration
RUN = Config.RuntimeState # Not implemented yet
# Always have a robot object: until setCom connects it, every command returns the
# "not connected" error reply instead of crashing on a missing serial port.
if RUN.get('robot') is None:
  RUN['robot'] = ARRobot()

FilterGraph = None
if CE['Platform']['IS_WINDOWS']:
  try:
    from pygrabber.dshow_graph import FilterGraph
  except ImportError:
    logger.warning("pygrabber not installed - camera names unavailable (pip install pygrabber)")


#############################################################################################
### CROSS-PLATFORM HELPERS ##################################################################
#############################################################################################

CAMERA_LABEL_TO_ID = {}   # camera menu label -> OpenCV source (index on Windows, index or /dev path on Linux)


def resolve_serial_port(value):
  """Turn a port entry into a real device name for this OS.
  '3' -> COM3 on Windows, /dev/ttyUSB3 on Linux; 'ttyACM0' -> /dev/ttyACM0 on Linux;
  full names (COM5, /dev/ttyACM0, /dev/serial/by-id/...) are used as-is."""
  v = str(value).strip()
  if not v:
    return v
  if IS_WINDOWS:
    return ("COM" + v) if v.isdigit() else v
  if v.isdigit():
    return "/dev/ttyUSB" + v
  if not v.startswith("/") and v.startswith("tty"):
    return "/dev/" + v
  return v


def ask_open_file(**kw):
  """filedialog.askopenfilename with the virtual-robot viewer paused.

  While the native dialog is up the 60 Hz viewer loop kept rendering and (on Windows) the
  topmost thread kept re-stacking the viewer window, so the dialog lost activation and its
  Open button needed a second click. The main window is also passed as parent."""
  RUN['dialog_open'] = True
  try:
    try:
      root.update_idletasks()
      root.grab_release()
    except Exception:
      pass
    kw.setdefault("parent", root)
    return fd.askopenfilename(**kw)
  finally:
    RUN['dialog_open'] = False


def open_camera(source):
  """Open a camera with the best backend for this OS (DirectShow on Windows, V4L2 on
  Linux), falling back to OpenCV's default backend."""
  if IS_WINDOWS and isinstance(source, int) and hasattr(cv2, "CAP_DSHOW"):
    cap = cv2.VideoCapture(source, cv2.CAP_DSHOW)
    if cap.isOpened():
      return cap
    cap.release()
  elif not IS_WINDOWS and hasattr(cv2, "CAP_V4L2"):
    cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if cap.isOpened():
      return cap
    cap.release()
  return cv2.VideoCapture(source)


def selected_camera_source():
  """OpenCV source for the camera currently picked in the Vision tab menu."""
  label = visoptions.get()
  src = CAMERA_LABEL_TO_ID.get(label)
  if src is None:
    try:
      src = camList.index(label)
    except (ValueError, NameError):
      src = 0
  if isinstance(src, str) and src.isdigit():
    src = int(src)
  return src


def _norm_prog_row(x):
  if isinstance(x, bytes):
    x = x.decode("utf-8", "replace")
  return str(x).replace("\r", "").replace("\n", "").strip()


def find_prog_row(text):
  """Index of a program row, ignoring CRLF vs LF and bytes vs str.
  Program files saved on Linux end lines with \\n only, so exact matches against
  '...\\r\\n' failed there (and on any OS right after inserting a new row)."""
  target = _norm_prog_row(text)
  for i, item in enumerate(tab1.progView.get(0, "end")):
    if _norm_prog_row(item) == target:
      return i
  raise ValueError(f"Program row not found: {target!r}")


def resolve_imported_name(name):
  """Find an imported CAD by name regardless of extension case (part / part.stl / part.STL).
  Linux filenames are case-sensitive, so blindly appending '.STL' missed 'part.stl'."""
  if name in imported_actors:
    return name
  low = name.lower()
  base = os.path.splitext(low)[0] if low.endswith((".stl", ".step", ".stp")) else low
  for key in imported_actors:
    k = key.lower()
    if k == low or os.path.splitext(k)[0] == base:
      return key
  return name if low.endswith(".stl") else name + '.STL'


kinematics.robot_set()

#DIR = pathlib.Path(__file__).parent.resolve()
#os.chdir(DIR)

if getattr(sys, "frozen", False):
    DIR = pathlib.Path(sys.executable).resolve().parent   # folder containing the .exe
else:
    DIR = pathlib.Path(__file__).resolve().parent         # folder containing AR4.py

os.chdir(DIR)    

RUN['cropping'] = False

root = Tk(className="AR4")
root.wm_title("AR4 Robot Control 8.1")
ui.init(root)                            # fonts, icon, themes + styles: before any widget
splash = ui.show_launch_splash(root)     # main window stays hidden until startup is done

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




def _header_status():
  def up(dev):
    try:
      return bool(dev) and dev.is_connected()
    except Exception:
      return False
  def port(key):
    try:
      p = CAL[key]
      return "" if p in (None, "", "None") else str(p)
    except Exception:
      return ""
  return {
    "teensy": (up(RUN.get('robot')), port('comPort')),
    "io": (up(RUN.get('nano')), port('com2Port')),
    "offline": bool(RUN.get('offlineMode')),
    "running": getattr(tab1, 'runTrue', 0) == 1,
  }

header = ui.Header(root, _header_status)   # logo + page tabs + connection status
header.grid(row=0, column=0, sticky='ew')

nb = ttk_bootstrap.Notebook(root, style="Bare.TNotebook")   # its tabs are drawn by the header

# Configure root window for resizing
root.grid_rowconfigure(1, weight=1)
root.grid_columnconfigure(0, weight=1)
nb.grid(row=1, column=0, sticky='nsew')

tab1 = ttk_bootstrap.Frame(nb)
nb.add(tab1, text='Main Controls')

tab2 = ui.scroll_frame(nb)
nb.add(ui.page(tab2), text='System Setup')

tab3 = ui.scroll_frame(nb)
nb.add(ui.page(tab3), text='Robot Parameters')

tab4 = ui.scroll_frame(nb)
nb.add(ui.page(tab4), text='Inputs & Outputs')

tab5 = ui.scroll_frame(nb)
nb.add(ui.page(tab5), text='Registers')

tab6 = ttk_bootstrap.Frame(nb)
nb.add(tab6, text='Vision')

tab7 = ttk_bootstrap.Frame(nb)
nb.add(tab7, text='G-Code')

tab8 = ttk_bootstrap.Frame(nb)
nb.add(tab8, text='Log')
header.attach(nb)

tab9 = ttk_bootstrap.Frame(nb)
#nb.add(tab9, text='   Info    ')

def detach_gripper_on_exit():
    try:
        if 'nano' in RUN and RUN['nano'] and RUN['nano'].is_connected():
            try:
                response = RUN['nano'].gripper_detach()
                print(response)
                logger.info("Gripper detach on exit response: %s", response)
            except Exception as e:
                logger.warning("Unable to detach gripper on exit: %s", e)

            try:
                RUN['nano'].disconnect()
            except Exception as e:
                logger.warning("Unable to disconnect Nano: %s", e)

    except Exception as e:
        logger.warning("Nano cleanup skipped: %s", e)

def on_closing():
  try:
      detach_all_imported_cad_on_close()
  except Exception as e:
      logger.error(f"Error detaching imported CAD during shutdown: {e}")
  
  detach_gripper_on_exit()

  # Only save if the scene was actually loaded this session; otherwise an
  # empty imported_actors dict would overwrite the saved scene with [].
  if RUN.get('cad_scene_loaded'):
      save_imported_cad_scene()

  RUN['vtk_running'] = False

  try:
    cv2.destroyAllWindows()
  except Exception:
    pass

  try:
    root.quit()
    root.update()
    root.destroy()
  except Exception:
    pass

root.wm_protocol("WM_DELETE_WINDOW", on_closing)

root.runTrue = 0
root.GCrunTrue = 0


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
RUN['EOAT_DEFAULT_X_OFFSET'] = 0.0
RUN['EOAT_DEFAULT_Y_OFFSET'] = 0.0
RUN['EOAT_DEFAULT_Z_OFFSET'] = 16.5
RUN['EOAT_DEFAULT_RX_OFFSET'] = 0.0
RUN['EOAT_DEFAULT_RY_OFFSET'] = 0.0
RUN['EOAT_DEFAULT_RZ_OFFSET'] = 90.0
RUN['attached_imported_actors'] = set()
RUN['cad_scene_loaded'] = False
RUN['tool_actor'] = None
RUN['tool_filename'] = ""

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
RUN['streamActive'] = False   # firmware motion queue (spline mode) is open for this program run
RUN['autoBlend'] = True       # stream consecutive motion rows (needs firmware 8.0+, online)

# Robot models offered in the model menu and accepted from saved settings / the Teensy.
# TST-MK1 is an internal test configuration, hidden unless SHOW_TEST_MODEL is True.
SHOW_TEST_MODEL = False
PUBLIC_MODELS = ["AR3", "AR4-MK1", "AR4-MK2", "AR4-MK3", "AR4-MK4", "AR4-MK5"]
VALID_MODELS = PUBLIC_MODELS + (["TST-MK1"] if SHOW_TEST_MODEL else [])
RUN['fwVersion'] = ""
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
RUN['cmdTypeLong'] = None

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

### SPEED PREFIX ###
def map_spd_type(prefix):
    return {
        "Sp": "percent",
        "Ss": "seconds",
        "Sm": "mmps"
    }.get(prefix, "percent")


#############################################################################################
### KINEMATICS FOR VIR ROBOT ################################################################
#############################################################################################

#DEG2RAD = np.pi / 180
#RAD2DEG = 180 / np.pi

def update_CPP_kin_from_entries():
    #global DHparams
    #global Robot_Data
    try:

        kinematics.set_dh_parameters_explicit(
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
        kinematics.set_joint_limits(PosLimits, NegLimits)
        kinematics.set_robot_tool_frame(float(TFxEntryField.get()), 
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
        fk_xyzuvw = kinematics.forward_kinematics(joint_angles)
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

    limits = kinematics.get_joint_limits()

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

    virtual_speed_scale = .4

    if SpeedType == "p":
        # percent mode correction
        # >1.0 makes virtual slower
        # <1.0 makes virtual faster
        virtual_speed_scale = .4
        SpeedVal = SpeedVal * 2

    elif SpeedType == "m":
        virtual_speed_scale = .4
        SpeedVal = SpeedVal * 2

    elif SpeedType == "s":
        virtual_speed_scale = .4
        SpeedVal = SpeedVal * 2

    # Steps in each region
    ACCStep = HighStep * (ACCspd / 100.0)
    DCCStep = HighStep * (DCCspd / 100.0)
    NORStep = HighStep - ACCStep - DCCStep

    # Target total time in microseconds (no 1.2 fudge)
    speedSP = SpeedVal
    if SpeedType == "s":
        speedSP = SpeedVal * 1000000 
    elif SpeedType == "m" and RUN['xyzuvw_In'] and RUN['xyzuvw_Out']:
        dx = RUN['xyzuvw_In'][0] - RUN['xyzuvw_Out'][0]
        dy = RUN['xyzuvw_In'][1] - RUN['xyzuvw_Out'][1]
        dz = RUN['xyzuvw_In'][2] - RUN['xyzuvw_Out'][2]
        lineDist = math.sqrt(dx*dx + dy*dy + dz*dz)
        # seconds = distance / (mm/s)
        speedSP = (lineDist / SpeedVal) * 1000000

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

    calcStepGap = calcStepGap * virtual_speed_scale    

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

        distDelay = 0
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
    if RUN['offlineMode']:
        _set_status_ready()    # the virtual robot completed a move


def parse_mj_command(inData):
    import json

    try:
        doc = json.loads(inData.strip())

        spd_type = str(doc.get("spd_type", "percent")).lower()

        if spd_type == "percent":
            SpeedType = "p"
        elif spd_type == "seconds":
            SpeedType = "s"
        elif spd_type == "mmps":
            SpeedType = "m"
        else:
            SpeedType = "p"

        return {
            "xyzuvw": [
                float(doc["pose"][0]),
                float(doc["pose"][1]),
                float(doc["pose"][2]),
                float(doc["pose"][3]),
                float(doc["pose"][4]),
                float(doc["pose"][5])
            ],
            "SpeedType": SpeedType,
            "Speed": float(doc.get("spd", 25)),
            "Acc": float(doc.get("acc", 100)),
            "Dec": float(doc.get("dec", 100)),
            "Ramp": float(doc.get("ramp", 100)),
            "WristConfig": str(doc.get("w", "A")),
            "LoopMode": str(doc.get("loop_mode", "000000"))
        }

    except Exception as e:
        logger.error(f"move_j JSON parse failed: {inData} | Error: {e}")
        return None

def parse_mt_command(inData):
    import json

    try:
        doc = json.loads(inData.strip())

        axis_map = {
            "X": 0,
            "Y": 1,
            "Z": 2,
            "RX": 3,
            "RY": 4,
            "RZ": 5
        }

        axis = str(doc["axis"]).upper()
        axis_index = axis_map[axis]

        direction = 1 if str(doc["dir"]).lower() in ("pos", "1", "+") else -1
        step = float(doc.get("step", 1.0))

        offset_vector = [0.0] * 6
        offset_vector[axis_index] = direction * step

        spd_type_in = str(doc.get("spd_type", "percent")).lower()

        if spd_type_in == "percent":
            SpeedType = "p"
        elif spd_type_in == "seconds":
            SpeedType = "s"
        elif spd_type_in == "mmps":
            SpeedType = "m"
        else:
            SpeedType = "p"

        return {
            "offset_vector": offset_vector,
            "SpeedType": SpeedType,
            "Speed": float(doc.get("spd", 25)),
            "Acc": float(doc.get("acc", 100)),
            "Dec": float(doc.get("dec", 100)),
            "Ramp": float(doc.get("ramp", 100)),
            "LoopMode": str(doc.get("loop_mode", "000000")).strip()
        }

    except Exception as e:
        logger.error(f"Tool jog JSON parse failed: {inData} | Error: {e}")
        return None





def rj_command(in_data):
    import json

    try:
        doc = json.loads(in_data.strip())

        Jangles = [
            float(doc["j"][0]),
            float(doc["j"][1]),
            float(doc["j"][2]),
            float(doc["j"][3]),
            float(doc["j"][4]),
            float(doc["j"][5])
        ]

        spd_type_in = str(doc.get("spd_type", "percent")).lower()

        if spd_type_in == "percent":
            SpeedType = "p"
        elif spd_type_in == "seconds":
            SpeedType = "s"
        elif spd_type_in == "mmps":
            SpeedType = "m"
        else:
            SpeedType = "p"

        SpeedVal = float(doc.get("spd", 25))
        ACCspd = float(doc.get("acc", 100))
        DCCspd = float(doc.get("dec", 100))
        ACCramp = float(doc.get("ramp", 100))

        WristCon = str(doc.get("wc", "A"))
        LoopMode = str(doc.get("loop_mode", "000000")).strip()
        LoopModes = list(map(int, list(LoopMode)))

    except Exception as e:
        logger.error(f"move_joints JSON parse failed: {in_data} | Error: {e}")
        RUN['Alarm'] = "ER"
        ErrorHandler(RUN['Alarm'])
        return

    fut_steps = [
        int(round((Jangles[0] + RUN['J1axisLimNeg']) * float(CAL['J1StepDeg']))),
        int(round((Jangles[1] + RUN['J2axisLimNeg']) * float(CAL['J2StepDeg']))),
        int(round((Jangles[2] + RUN['J3axisLimNeg']) * float(CAL['J3StepDeg']))),
        int(round((Jangles[3] + RUN['J4axisLimNeg']) * float(CAL['J4StepDeg']))),
        int(round((Jangles[4] + RUN['J5axisLimNeg']) * float(CAL['J5StepDeg']))),
        int(round((Jangles[5] + RUN['J6axisLimNeg']) * float(CAL['J6StepDeg']))),
    ]

    RUN['cur_steps'] = [
        RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'],
        RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']
    ]

    step_lims = [
        J1StepLim, J2StepLim, J3StepLim,
        J4StepLim, J5StepLim, J6StepLim
    ]

    step_difs = [
        int(round(cur - fut))
        for cur, fut in zip(RUN['cur_steps'], fut_steps)
    ]

    dirs = [
        1 if diff <= 0 else 0
        for diff in step_difs
    ]

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
    RUN['JangleOut'] = kinematics.SolveInverseKinematics(RUN['xyzuvw_In'], RUN['VR_angles'])

    if RUN['JangleOut'] is None or len(RUN['JangleOut']) == 0:
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
    import math
    import numpy as np

    def rotvec_to_matrix(rx, ry, rz):
        theta = math.sqrt(rx * rx + ry * ry + rz * rz)

        if theta < 1e-12:
            return np.identity(3)

        kx = rx / theta
        ky = ry / theta
        kz = rz / theta

        K = np.array([
            [0.0, -kz, ky],
            [kz, 0.0, -kx],
            [-ky, kx, 0.0]
        ], dtype=float)

        I = np.identity(3)
        return I + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)

    def matrix_to_rotvec(R):
        trace_val = np.trace(R)
        cos_theta = (trace_val - 1.0) / 2.0
        cos_theta = max(-1.0, min(1.0, cos_theta))
        theta = math.acos(cos_theta)

        if theta < 1e-12:
            return [0.0, 0.0, 0.0]

        denom = 2.0 * math.sin(theta)

        rx = ((R[2, 1] - R[1, 2]) / denom) * theta
        ry = ((R[0, 2] - R[2, 0]) / denom) * theta
        rz = ((R[1, 0] - R[0, 1]) / denom) * theta

        return [rx, ry, rz]

    def pose_from_xyzuvw(xyzuvw_deg):
        T = np.identity(4)

        T[0:3, 3] = [
            float(xyzuvw_deg[0]),
            float(xyzuvw_deg[1]),
            float(xyzuvw_deg[2])
        ]

        rx = math.radians(float(xyzuvw_deg[3]))
        ry = math.radians(float(xyzuvw_deg[4]))
        rz = math.radians(float(xyzuvw_deg[5]))

        T[0:3, 0:3] = rotvec_to_matrix(rx, ry, rz)
        return T

    def xyzuvw_from_pose(T):
        rvec = matrix_to_rotvec(T[0:3, 0:3])

        return [
            float(T[0, 3]),
            float(T[1, 3]),
            float(T[2, 3]),
            math.degrees(rvec[0]),
            math.degrees(rvec[1]),
            math.degrees(rvec[2])
        ]

    result = parse_mt_command(inData)

    if not result:
        if RUN['offlineMode']:
            ErrorHandler("ER")
        return

    offset = [float(v) for v in result["offset_vector"]]

    RUN['xyzuvw_In'] = np.array([
        float(CAL['XcurPos']),
        float(CAL['YcurPos']),
        float(CAL['ZcurPos']),
        float(CAL['RzcurPos']),
        float(CAL['RycurPos']),
        float(CAL['RxcurPos'])
    ], dtype=float)

    command_pose = pose_from_xyzuvw(RUN['xyzuvw_In'])

    # Linear tool-frame offset
    command_pose[0:3, 3] += command_pose[0:3, 0] * offset[0]
    command_pose[0:3, 3] += command_pose[0:3, 1] * offset[1]
    command_pose[0:3, 3] += command_pose[0:3, 2] * offset[2]

    # Rotational tool-frame offset
    if offset[3] != 0 or offset[4] != 0 or offset[5] != 0:
        delta_pose = pose_from_xyzuvw([
            0.0,
            0.0,
            0.0,
            offset[3],
            offset[4],
            offset[5]
        ])

        command_pose = command_pose @ delta_pose

    RUN['xyzuvw_In'] = np.array(xyzuvw_from_pose(command_pose), dtype=float)

    try:
        RUN['JangleOut'] = kinematics.SolveInverseKinematics(
            RUN['xyzuvw_In'],
            RUN['VR_angles']
        )
    except Exception as e:
        logger.error(f"IK Exception: {e}")
        RUN['Alarm'] = "ER"
        ErrorHandler(RUN['Alarm'])
        return

    if RUN['JangleOut'] is None:
        if RUN['offlineMode']:
            logger.error("Inverse kinematics failed. No solution found.")
            ErrorHandler("ER")
        return

    RUN['JangleOut'] = np.array(RUN['JangleOut'], dtype=np.float64).flatten()

    step_degs = [
        float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
        float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])
    ]

    axis_neg = [
        float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
        float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])
    ]

    fut_steps = [
        int(round((j + off) * deg))
        for j, off, deg in zip(RUN['JangleOut'], axis_neg, step_degs)
    ]

    RUN['cur_steps'] = [
        RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'],
        RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']
    ]

    step_lims = [
        J1StepLim, J2StepLim, J3StepLim,
        J4StepLim, J5StepLim, J6StepLim
    ]

    step_difs = [
        int(round(cur - fut))
        for cur, fut in zip(RUN['cur_steps'], fut_steps)
    ]

    dirs = [
        1 if diff <= 0 else 0
        for diff in step_difs
    ]

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
    import json
    import time

    try:
        doc = json.loads(in_data.strip())

        idx = int(doc["axis"]) - 1
        direction = 1 if int(doc["dir"]) > 0 else -1

        spd_type_in = str(doc.get("spd_type", "percent")).lower()

        if spd_type_in == "percent":
            SpeedType = "p"
        elif spd_type_in == "seconds":
            SpeedType = "s"
        elif spd_type_in == "mmps":
            SpeedType = "m"
        else:
            SpeedType = "p"

        SpeedVal = float(doc.get("spd", 25))
        ACCspd = float(doc.get("acc", 100))
        DCCspd = float(doc.get("dec", 100))
        ACCramp = float(doc.get("ramp", 100))

        LoopModeStr = str(doc.get("loop_mode", "111111")).strip()
        LoopModes = [int(c) for c in LoopModeStr]

    except Exception as e:
        logger.error(f"Invalid live_joint_jog JSON command: {in_data} | Error: {e}")
        RUN['Alarm'] = "ER"
        ErrorHandler(RUN['Alarm'])
        return

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
        except Exception:
            if RUN['offlineMode']:
                logger.error(f"Invalid RUN['VR_angles']: {RUN['VR_angles'][:6]}")
                RUN['Alarm'] = "ER"
                ErrorHandler(RUN['Alarm'])
            return

        Jangles[idx] += direction * 0.1

        axis_lims = [
            float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
            float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])
        ]

        step_degs = [
            float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
            float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])
        ]

        step_lims = [
            J1StepLim, J2StepLim, J3StepLim,
            J4StepLim, J5StepLim, J6StepLim
        ]

        RUN['cur_steps'] = [
            RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'],
            RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']
        ]

        fut_steps = [
            int(round((Jangles[i] + axis_lims[i]) * step_degs[i]))
            for i in range(6)
        ]

        step_difs = [
            cur - fut
            for cur, fut in zip(RUN['cur_steps'], fut_steps)
        ]

        dirs = [
            1 if diff <= 0 else 0
            for diff in step_difs
        ]

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



# A live jog moves the target 1 mm / 1 deg per cycle; a solution that moves a joint farther
# than this is a wrist flip or a +/-180 wrap, not a jog, and ends the jog (as on the Teensy).
LIVE_JOG_MAX_JOINT_JUMP_DEG = 90.0


def live_cartesian_jog(in_data):
    import json
    import time
    import numpy as np

    try:
        doc = json.loads(in_data.strip())

        axis_map = {
            "X": 0,
            "Y": 1,
            "Z": 2,
            "RZ": 3,
            "RY": 4,
            "RX": 5
        }

        axis = str(doc["axis"]).upper()
        idx = axis_map[axis]

        direction = 1 if str(doc["dir"]).lower() in ("pos", "1", "+") else -1

        spd_type_in = str(doc.get("spd_type", "percent")).lower()

        if spd_type_in == "percent":
            SpeedType = "p"
        elif spd_type_in == "seconds":
            SpeedType = "s"
        elif spd_type_in == "mmps":
            SpeedType = "m"
        else:
            SpeedType = "p"

        SpeedVal = float(doc.get("spd", 25))
        ACCspd = float(doc.get("acc", 100))
        DCCspd = float(doc.get("dec", 100))
        ACCramp = float(doc.get("ramp", 100))

        LoopModeStr = str(doc.get("loop_mode", "111111")).strip()
        LoopModes = [int(c) for c in LoopModeStr]

    except Exception as e:
        logger.error(f"Invalid live_cartesian_jog JSON command: {in_data} | Error: {e}")
        RUN['Alarm'] = "ER"
        ErrorHandler(RUN['Alarm'])
        return

    if not (0 <= idx < 6):
        RUN['Alarm'] = "ER"
        ErrorHandler(RUN['Alarm'])
        return

    jog_step = 1  # mm or deg

    cur = [float(CAL['XcurPos']), float(CAL['YcurPos']), float(CAL['ZcurPos']),
           float(CAL['RzcurPos']), float(CAL['RycurPos']), float(CAL['RxcurPos'])]
    jog_pos = np.array(cur[:3], dtype=float)
    jog_R = ar4_to_R(*cur[3:])                  # commanded orientation as a matrix
    RUN['xyzuvw_In'] = np.array(cur, dtype=float)

    RUN['liveJog'] = True

    while RUN['liveJog']:
        while drive_lock.locked():
            time.sleep(0.005)

        # Translate along, or rotate about, a WORLD axis (same as the Teensy liveCartJog).
        # The pose fields hold a rotation vector: adding degrees to one of them is not a
        # rotation about anything and reversed direction past 180 deg (tool-down pose).
        if idx < 3:
            jog_pos[idx] += direction * jog_step
        else:
            world_axis = np.eye(3)[5 - idx]             # slot 3 = Rz, 4 = Ry, 5 = Rx
            jog_R = _rot_axis(world_axis, math.radians(direction * jog_step)) @ jog_R
        RUN['xyzuvw_In'] = np.array(
            list(jog_pos) + list(R_to_ar4(jog_R, ref=np.asarray(RUN['xyzuvw_In'][3:], dtype=float))),
            dtype=float)

        try:
            RUN['JangleOut'] = kinematics.SolveInverseKinematics(
                RUN['xyzuvw_In'],
                RUN['VR_angles'],
                LIVE_JOG_MAX_JOINT_JUMP_DEG
            )
        except Exception as e:
            logger.error(f"IK Exception: {e}")
            RUN['Alarm'] = "ER"
            ErrorHandler(RUN['Alarm'])
            break

        if RUN['JangleOut'] is None or len(RUN['JangleOut']) == 0:
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

        fut_steps = [
            int(round((RUN['JangleOut'][i] + axis_lims[i]) * step_degs[i]))
            for i in range(6)
        ]

        RUN['cur_steps'] = [
            RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'],
            RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']
        ]

        step_lims = [
            J1StepLim, J2StepLim, J3StepLim,
            J4StepLim, J5StepLim, J6StepLim
        ]

        step_difs = [
            cur - fut
            for cur, fut in zip(RUN['cur_steps'], fut_steps)
        ]

        dirs = [
            1 if diff <= 0 else 0
            for diff in step_difs
        ]

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
    import json
    import time
    import math
    import numpy as np

    def rotvec_to_matrix(rx, ry, rz):
        theta = math.sqrt(rx * rx + ry * ry + rz * rz)

        if theta < 1e-12:
            return np.identity(3)

        kx = rx / theta
        ky = ry / theta
        kz = rz / theta

        K = np.array([
            [0.0, -kz, ky],
            [kz, 0.0, -kx],
            [-ky, kx, 0.0]
        ], dtype=float)

        I = np.identity(3)
        return I + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)

    def matrix_to_rotvec(R):
        trace_val = np.trace(R)
        cos_theta = (trace_val - 1.0) / 2.0
        cos_theta = max(-1.0, min(1.0, cos_theta))
        theta = math.acos(cos_theta)

        if theta < 1e-12:
            return [0.0, 0.0, 0.0]

        denom = 2.0 * math.sin(theta)

        rx = ((R[2, 1] - R[1, 2]) / denom) * theta
        ry = ((R[0, 2] - R[2, 0]) / denom) * theta
        rz = ((R[1, 0] - R[0, 1]) / denom) * theta

        return [rx, ry, rz]

    def pose_from_xyzuvw(xyzuvw_rad):
        T = np.identity(4)

        T[0:3, 3] = [
            float(xyzuvw_rad[0]),
            float(xyzuvw_rad[1]),
            float(xyzuvw_rad[2])
        ]

        R = rotvec_to_matrix(
            float(xyzuvw_rad[3]),
            float(xyzuvw_rad[4]),
            float(xyzuvw_rad[5])
        )

        T[0:3, 0:3] = R
        return T

    def xyzuvw_from_pose(T):
        rvec = matrix_to_rotvec(T[0:3, 0:3])

        return [
            float(T[0, 3]),
            float(T[1, 3]),
            float(T[2, 3]),
            math.degrees(rvec[0]),
            math.degrees(rvec[1]),
            math.degrees(rvec[2])
        ]

    try:
        doc = json.loads(in_data.strip())

        axis = str(doc["axis"])
        axis_upper = axis.upper()

        direction = 1 if str(doc["dir"]).lower() == "pos" else -1

        spd_type_in = str(doc.get("spd_type", "percent")).lower()

        if spd_type_in == "percent":
            SpeedType = "p"
        elif spd_type_in == "seconds":
            SpeedType = "s"
        elif spd_type_in == "mmps":
            SpeedType = "m"
        else:
            SpeedType = "p"

        SpeedVal = float(doc.get("spd", 25))
        ACCspd = float(doc.get("acc", 100))
        DCCspd = float(doc.get("dec", 100))
        ACCramp = float(doc.get("ramp", 100))

        LoopModeStr = str(doc.get("loop_mode", "111111")).strip()
        LoopModes = [int(c) for c in LoopModeStr]

    except Exception as e:
        logger.error(f"Invalid live_tool_jog JSON command: {in_data} | Error: {e}")
        RUN['Alarm'] = "ER"
        ErrorHandler(RUN['Alarm'])
        return

    jog_step_lin = 1.0
    jog_step_rot = math.radians(1.0)

    try:
        fk_pose = kinematics.forward_kinematics(RUN['VR_angles'])
        jog_command_pose = pose_from_xyzuvw(fk_pose)
    except Exception as e:
        logger.error(f"Tool jog FK seed failed: {e}")
        RUN['Alarm'] = "ER"
        ErrorHandler(RUN['Alarm'])
        return

    jog_step_err = [0.0] * 6

    RUN['liveJog'] = True

    while RUN['liveJog']:
        while drive_lock.locked():
            time.sleep(0.005)

        signed_lin = jog_step_lin * direction
        signed_rot = jog_step_rot * direction

        if axis_upper == "X":
            jog_command_pose[0:3, 3] += jog_command_pose[0:3, 0] * signed_lin

        elif axis_upper == "Y":
            jog_command_pose[0:3, 3] += jog_command_pose[0:3, 1] * signed_lin

        elif axis_upper == "Z":
            jog_command_pose[0:3, 3] += jog_command_pose[0:3, 2] * signed_lin

        elif axis_upper in ("RX", "RY", "RZ"):
            delta_xyzuvw = [0.0] * 6

            if axis_upper == "RX":
                delta_xyzuvw[3] = signed_rot
            elif axis_upper == "RY":
                delta_xyzuvw[4] = signed_rot
            elif axis_upper == "RZ":
                delta_xyzuvw[5] = signed_rot

            delta_pose = pose_from_xyzuvw(delta_xyzuvw)

            # Match Teensy:
            # jogCommandPose = jogCommandPose * deltaPose
            jog_command_pose = jog_command_pose @ delta_pose

        else:
            RUN['Alarm'] = "ER"
            ErrorHandler(RUN['Alarm'])
            break

        RUN['xyzuvw_In'] = xyzuvw_from_pose(jog_command_pose)

        try:
            RUN['JangleOut'] = kinematics.SolveInverseKinematics(
                RUN['xyzuvw_In'],
                RUN['VR_angles'],
                LIVE_JOG_MAX_JOINT_JUMP_DEG
            )
        except Exception as e:
            logger.error(f"IK Exception: {e}")
            RUN['Alarm'] = "ER"
            ErrorHandler(RUN['Alarm'])
            break

        if RUN['JangleOut'] is None or len(RUN['JangleOut']) == 0:
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

        desired_steps = [
            (RUN['JangleOut'][i] + axis_lims[i]) * step_degs[i] + jog_step_err[i]
            for i in range(6)
        ]

        fut_steps = [
            int(round(desired_steps[i]))
            for i in range(6)
        ]

        jog_step_err = [
            desired_steps[i] - fut_steps[i]
            for i in range(6)
        ]

        RUN['cur_steps'] = [
            RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'],
            RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']
        ]

        step_lims = [
            J1StepLim, J2StepLim, J3StepLim,
            J4StepLim, J5StepLim, J6StepLim
        ]

        step_difs = [
            cur - fut
            for cur, fut in zip(RUN['cur_steps'], fut_steps)
        ]

        dirs = [
            1 if diff <= 0 else 0
            for diff in step_difs
        ]

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
        ### TEMP GRIPPER ###
        , "Servo Gripper.STL"
        , "Weld Gripper.STL" 
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
            base_tf.RotateZ(-90)
            base_tf.Translate(0, 0, -92)
        elif stl == "Link 2-1.STL":
            base_tf.RotateZ(-90)
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

        ### TEMP GRIPPER ###    
        elif stl == "Servo Gripper.STL":
            base_tf.RotateZ(90)
            base_tf.Translate(0, 0, 17)
        elif stl == "Weld Gripper.STL":
            base_tf.RotateZ(90)
            base_tf.Translate(0, 0, 17)         

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

    ### TEMP GRIPPER ###
    #RUN['assemblies']["Link 6-1.STL"].AddPart(RUN['assemblies']["Servo Gripper.STL"])
    #RUN['assemblies']["Link 6-1.STL"].AddPart(RUN['assemblies']["Weld Gripper.STL"])

    renderer.AddActor(root)

#############################################################################################
### CLICKED NODES + MOVE/ROTATE TRIAD FOR VIRTUAL ROBOT #####################################
#############################################################################################

RUN['vr_nodes'] = []            # list of {'pos': np(3), 'R': np(3x3), 'actor', 'arrow', 'label'}
RUN['node_renderer'] = None     # layer-1 renderer drawn on top of the robot
RUN['node_panel'] = None        # clickable 2D node list
RUN['node_picker'] = None       # picks robot/floor/imported geometry
RUN['node_sphere_picker'] = None
RUN['vr_selected'] = None       # index of highlighted node
RUN['gizmo'] = None             # move/rotate triad state
RUN['vr_rnd'] = None            # rounding (mm) used by Teach ML; seeded from the main Rnd field
RUN['vr_rnd_edit'] = None       # text being typed into the rounding field, None when not editing

VR_EVENTS = Queue()             # filled by VTK callbacks, drained on the Tk loop
VR_DEBUG = False

NODE_RADIUS = 5.0
NODE_COLOR = (1.0, 0.85, 0.0)
NODE_SEL_COLOR = (0.0, 1.0, 1.0)
NODE_ARROW_COLOR = (1.0, 0.35, 1.0)
NODE_ARROW_LEN = 45.0           # mm
NODE_ORIENT_FROM_SURFACE = True # new nodes face into the clicked surface (else: current tool orientation)
MAX_LISTED_NODES = 15
NODE_ROW_H = 20
NODE_PANEL_X = 10
NODE_PANEL_TOP = 12
NODE_FONT = 14
NODE_ROW_HIT_W = 720
NODE_CHAR_W = 9                 # approx. px per character of the panel font (button hit widths)
NODE_BTN_GAP = 12               # px between buttons on one row
VR_RND_STEP = 1.0               # mm per click of [-] / [+]
VR_RND_MAX = 500.0

CLICK_POINTS_FILE = str(DIR / "clickPoints.json")
NODE_APPROACH_OFFSET = 0.0      # mm to back off along the arrow when teaching (e.g. 25.0)

GIZMO_SCALE = 0.25              # triad size relative to view
GIZMO_SNAP_MM = 5.0
GIZMO_SNAP_DEG = 15.0
GIZMO_AXIS_COLORS = ((1.0, 0.25, 0.25), (0.25, 1.0, 0.25), (0.35, 0.55, 1.0))
GIZMO_ACTIVE_COLOR = (1.0, 1.0, 0.2)

GIZMO_TOOL_NAMES = ("Servo Gripper.STL", "Weld Gripper.STL")
TOOL_PARENT_CHAIN = [
    "Link Base-1.STL", "Link Base-2.STL", "Link Base-3.STL",
    "Link 1-1.STL", "Link 1-2.STL", "Link 2-1.STL", "Link 2-2.STL", "Link 2-3.STL",
    "Link 3-1.STL", "Link 3-2.STL", "Link 4-1.STL", "Link 4-2.STL", "Link 4-3.STL",
    "Link 5-1.STL", "Link 5-2.STL", "Link 6-1.STL",
]

_NODE_ARROW_PD = None

TCP_TRIAD_LEN = 30.0                    # mm, length of each TCP triad axis
TCP_TRIAD_WIDTH = 3.0                   # line width in pixels
TCP_TRIAD_OPACITY = 0.5
RUN['tcp_marker'] = None
RUN['tcp_key'] = None                   # joint angles the marker was last placed for
RUN['tf_applied'] = None                # tool frame last pushed to the kinematics


# ---------------------------------------------------------------- math helpers

def _np_from_vtk(m):
    return np.array([[m.GetElement(i, j) for j in range(4)] for i in range(4)], dtype=float)


def _vtk_from_np(a):
    m = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            m.SetElement(i, j, float(a[i, j]))
    return m


def _H(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def _T(p):
    return _H(np.eye(3), p)


def _rot_axis(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def _orthonormal(R):
    U, _, Vt = np.linalg.svd(np.asarray(R, float))
    Q = U @ Vt
    if np.linalg.det(Q) < 0:
        U[:, -1] *= -1
        Q = U @ Vt
    return Q


def euler_zyx_to_R(rz, ry, rx):
    """AR4 Rz/Ry/Rx (degrees) -> rotation matrix, R = Rz * Ry * Rx."""
    z, y, x = map(math.radians, (rz, ry, rx))
    Rz = np.array([[math.cos(z), -math.sin(z), 0], [math.sin(z), math.cos(z), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(y), 0, math.sin(y)], [0, 1, 0], [-math.sin(y), 0, math.cos(y)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(x), -math.sin(x)], [0, math.sin(x), math.cos(x)]])
    return Rz @ Ry @ Rx


def R_to_euler_zyx(R):
    """Rotation matrix -> (rz, ry, rx) degrees, inverse of euler_zyx_to_R."""
    sy = math.hypot(R[0, 0], R[1, 0])
    if sy > 1e-9:
        rz = math.atan2(R[1, 0], R[0, 0])
        ry = math.atan2(-R[2, 0], sy)
        rx = math.atan2(R[2, 1], R[2, 2])
    else:  # gimbal lock
        rz = math.atan2(-R[0, 1], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rx = 0.0
    return tuple(math.degrees(v) for v in (rz, ry, rx))


# robot_kinematics.forward_kinematics / SolveInverseKinematics do NOT use Euler angles for
# xyzuvw[3:6]: those three slots are a rotation vector (axis * angle, like UR robots), with
# slot 3 = x component, slot 4 = y, slot 5 = z.  The GUI labels them "Rz Ry Rx", so the
# helpers below keep that slot order but use the rotation-vector math the solver uses.

def ar4_to_R(a3, a4, a5):
    """AR4 pose angles in GUI order (the 'Rz Ry Rx' fields, degrees) -> rotation matrix."""
    v = np.radians([float(a3), float(a4), float(a5)])
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    return _rot_axis(v / th, th)


def _ar4_ref():
    try:
        return np.array([float(CAL['RzcurPos']), float(CAL['RycurPos']), float(CAL['RxcurPos'])])
    except Exception:
        return None


def R_to_ar4(R, ref=None):
    """Rotation matrix -> AR4 pose angles in GUI order (Rz, Ry, Rx fields), degrees.

    Near 180 degrees a rotation vector can be written two ways (k*th or k*(th-360)); both
    give the same orientation, so pick the one closest to `ref` (default: the robot's current
    pose) to keep the numbers looking like what the robot displays.
    """
    R = _orthonormal(R)
    # quaternion (Shepperd) - stable at 0 and 180 degrees
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    if q[0] < 0:
        q = -q
    n = float(np.linalg.norm(q[1:]))
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    k = q[1:] / n
    th = math.degrees(2.0 * math.atan2(n, q[0]))          # 0..180
    v = k * th
    if ref is None:
        ref = _ar4_ref()
    if ref is not None and th > 150.0:
        alt = k * (th - 360.0)                              # same orientation, other side
        if np.linalg.norm(alt - ref) < np.linalg.norm(v - ref):
            v = alt
    return tuple(float(a) for a in v)


def _current_tool_R():
    try:
        return ar4_to_R(float(CAL['RzcurPos']), float(CAL['RycurPos']), float(CAL['RxcurPos']))
    except Exception:
        return np.eye(3)


def _frame_from_z(z, x_ref):
    """Right-handed frame whose Z is `z` and whose X is as close as possible to `x_ref`."""
    z = np.asarray(z, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(x_ref, float) - z * (z @ x_ref)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, [0, 0, 1]) if abs(z[2]) < 0.9 else np.cross(z, [0, 1, 0])
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _display_ray(x, y):
    ren = RUN['renderer']
    ren.SetDisplayPoint(x, y, 0.0)
    ren.DisplayToWorld()
    a = np.array(ren.GetWorldPoint(), float)
    ren.SetDisplayPoint(x, y, 1.0)
    ren.DisplayToWorld()
    b = np.array(ren.GetWorldPoint(), float)
    a = a[:3] / a[3]
    b = b[:3] / b[3]
    d = b - a
    return a, d / np.linalg.norm(d)


def _axis_param(c, a, o, d):
    """Parameter t of the point on line c + t*a closest to the view ray o + s*d."""
    w0 = c - o
    b = a @ d
    den = 1.0 - b * b
    if den < 1e-6:
        return None     # axis points straight at the camera
    return (b * (d @ w0) - (a @ w0)) / den


def _plane_vec(c, n, o, d):
    """Unit vector from c to where the view ray hits the plane through c with normal n."""
    den = n @ d
    if abs(den) < 1e-6:
        return None     # ring seen edge-on
    s = (n @ (c - o)) / den
    v = o + s * d - c
    v = v - n * (n @ v)
    L = np.linalg.norm(v)
    return None if L < 1e-9 else v / L


# ---------------------------------------------------------------- event queue

def _vr_queue():
    return VR_EVENTS


def process_vr_events():
    """Runs on the Tk loop (from update_vtk). Safe to touch Tk/logger here."""
    q = _vr_queue()
    while not q.empty():
        ev = q.get()
        kind = ev[0]
        if kind == 'pick':
            if not node_panel_click(ev[1], ev[2]):
                pick_node(ev[1], ev[2])
        elif kind == 'select3d':
            select3d(ev[1], ev[2])
        elif kind == 'gizmo_done':
            on_gizmo_done()
        elif kind == 'deselect':
            deselect_all()
        elif kind == 'snap':
            snap_selected_to_tool()
        elif kind == 'undo':
            undo_last_node()
        elif kind == 'clear':
            clear_nodes()
        elif kind == 'rnd_key':
            rnd_edit_key(ev[1])


# ---------------------------------------------------------------- persistence

def save_click_points():
    pts = []
    for i, nd in enumerate(RUN['vr_nodes']):
        rz, ry, rx = R_to_ar4(nd['R'])
        p = nd['pos']
        pts.append({"id": i + 1,
                    "x": round(float(p[0]), 3), "y": round(float(p[1]), 3), "z": round(float(p[2]), 3),
                    "rz": round(rz, 3), "ry": round(ry, 3), "rx": round(rx, 3)})
    tmp = CLICK_POINTS_FILE + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"version": 3, "points": pts}, f, indent=2)
        os.replace(tmp, CLICK_POINTS_FILE)
    except Exception as e:
        logger.error(f"Could not save click points to {CLICK_POINTS_FILE}: {e}")


def load_click_points():
    """Return [(pos, R or None), ...] from clickPoints.json. Empty/missing file -> []."""
    try:
        with open(CLICK_POINTS_FILE, "r", encoding="utf-8") as f:
            text = f.read().strip()
    except FileNotFoundError:
        return []
    except Exception as e:
        logger.error(f"Could not read {CLICK_POINTS_FILE}: {e}")
        return []
    if not text:
        return []
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        bad = CLICK_POINTS_FILE + ".bad"
        try:
            os.replace(CLICK_POINTS_FILE, bad)
        except Exception:
            pass
        logger.error(f"clickPoints.json is not valid JSON ({e}); moved to {bad}, starting with no points")
        return []

    pts = data.get("points", []) if isinstance(data, dict) else data
    version = data.get("version", 1) if isinstance(data, dict) else 1
    to_R = ar4_to_R if version >= 3 else euler_zyx_to_R     # v1/v2 files stored ZYX Euler
    out = []
    for p in pts if isinstance(pts, list) else []:
        try:
            pos = (float(p["x"]), float(p["y"]), float(p["z"]))
        except (KeyError, TypeError, ValueError):
            logger.warning(f"Skipping invalid point in clickPoints.json: {p}")
            continue
        R = None
        if all(k in p for k in ("rz", "ry", "rx")):
            try:
                R = to_R(float(p["rz"]), float(p["ry"]), float(p["rx"]))
            except (TypeError, ValueError):
                R = None
        out.append((pos, R))
    return out


# ---------------------------------------------------------------- panel

def _make_text(color=(1.0, 1.0, 1.0), bold=False):
    t = vtk.vtkTextActor()
    tp = t.GetTextProperty()
    tp.SetFontFamilyToCourier()
    tp.SetFontSize(NODE_FONT)
    tp.SetColor(*color)
    tp.SetBold(bold)
    tp.SetBackgroundColor(0.0, 0.0, 0.0)
    tp.SetBackgroundOpacity(0.55)
    tp.SetJustificationToLeft()
    tp.SetVerticalJustificationToTop()
    t.GetPositionCoordinate().SetCoordinateSystemToDisplay()
    t.PickableOff()
    return t


def setup_node_overlay():
    """Overlay renderer (layer 1) sharing the main camera, panel, triad, and saved points."""
    rw = RUN['render_window']
    base = RUN['renderer']

    rw.SetNumberOfLayers(2)
    base.SetLayer(0)

    overlay = vtk.vtkRenderer()
    overlay.SetLayer(1)
    overlay.InteractiveOff()
    overlay.SetActiveCamera(base.GetActiveCamera())
    rw.AddRenderer(overlay)
    RUN['node_renderer'] = overlay
    setup_tcp_marker(overlay)

    panel = {
        'header':   _make_text(bold=True),
        'clear':    _make_text(color=(1.0, 0.6, 0.2), bold=True),
        'teach_mj':    _make_text(color=(0.4, 1.0, 0.4), bold=True),
        'teach_ml':    _make_text(color=(0.4, 0.85, 1.0), bold=True),
        'teachall_mj': _make_text(color=(0.4, 1.0, 0.4), bold=True),
        'teachall_ml': _make_text(color=(0.4, 0.85, 1.0), bold=True),
        'rnd_label':   _make_text(color=(0.4, 0.85, 1.0)),
        'rnd_minus':   _make_text(color=(0.4, 0.85, 1.0), bold=True),
        'rnd_value':   _make_text(bold=True),
        'rnd_plus':    _make_text(color=(0.4, 0.85, 1.0), bold=True),
        'more':     _make_text(color=(0.8, 0.8, 0.8)),
        'rows': [], 'hits': [], 'size': None, 'prog_loaded': None,
    }
    panel['clear'].SetInput(" [ Clear all ] ")
    panel['rnd_label'].SetInput(" ML rounding: ")
    panel['rnd_minus'].SetInput(" [-] ")
    panel['rnd_plus'].SetInput(" [+] ")
    for key in ('header', 'clear', 'teach_mj', 'teach_ml', 'teachall_mj', 'teachall_ml',
                'rnd_label', 'rnd_minus', 'rnd_value', 'rnd_plus', 'more'):
        overlay.AddViewProp(panel[key])
    RUN['node_panel'] = panel

    picker = vtk.vtkCellPicker()
    picker.SetTolerance(0.0005)
    RUN['node_picker'] = picker

    sp = vtk.vtkCellPicker()
    sp.SetTolerance(0.002)
    sp.PickFromListOn()
    RUN['node_sphere_picker'] = sp

    RUN['gizmo'] = _gizmo_create(overlay)

    while not VR_EVENTS.empty():
        VR_EVENTS.get()

    RUN['vr_nodes'] = []
    RUN['vr_selected'] = None
    saved = load_click_points()
    for pos, R in saved:
        add_node(pos, R, save=False, quiet=True)
    if saved:
        logger.info(f"Loaded {len(saved)} click point(s) from clickPoints.json")
    refresh_node_list()


# ---------------------------------------------------------------- TCP marker

def setup_tcp_marker(overlay):
    """X/Y/Z triad (red/green/blue) on the robot's TCP, drawn on the overlay so it shows
    through the tool model."""
    L = TCP_TRIAD_LEN
    pts = vtk.vtkPoints()
    for xyz in ((0, 0, 0), (L, 0, 0), (0, L, 0), (0, 0, L)):
        pts.InsertNextPoint(*xyz)
    lines = vtk.vtkCellArray()
    colors = vtk.vtkUnsignedCharArray()
    colors.SetNumberOfComponents(3)
    colors.SetName("Colors")
    for end, rgb in ((1, (255, 0, 0)), (2, (0, 200, 0)), (3, (0, 90, 255))):
        lines.InsertNextCell(2)
        lines.InsertCellPoint(0)
        lines.InsertCellPoint(end)
        colors.InsertNextTuple3(*rgb)
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetLines(lines)
    pd.GetCellData().SetScalars(colors)

    m = vtk.vtkPolyDataMapper()
    m.SetInputData(pd)
    m.SetScalarModeToUseCellData()
    a = vtk.vtkActor()
    a.SetMapper(m)
    prop = a.GetProperty()
    prop.SetLineWidth(TCP_TRIAD_WIDTH)
    prop.SetOpacity(TCP_TRIAD_OPACITY)
    prop.LightingOff()                      # flat, true colors from any angle
    a.PickableOff()
    overlay.AddActor(a)
    RUN['tcp_marker'] = a
    RUN['tcp_key'] = None
    RUN['tf_applied'] = None


def _sync_tool_frame_from_entries():
    """Push the tool frame fields on the parameters screen to the kinematics when they change.
    Half-typed values (e.g. '-' or '') are ignored until they parse. True if it changed."""
    try:
        vals = tuple(float(e.get()) for e in (TFxEntryField, TFyEntryField, TFzEntryField,
                                              TFrxEntryField, TFryEntryField, TFrzEntryField))
    except Exception:
        return False
    if vals == RUN.get('tf_applied'):
        return False
    try:
        kinematics.set_robot_tool_frame(*vals)
    except Exception as e:
        logger.debug(f"Tool frame update failed: {e}")
        return False
    RUN['tf_applied'] = vals
    return True


def update_tcp_marker():
    """Called every viewer tick: follow jogging and tool-frame edits. FK runs only on change."""
    a = RUN.get('tcp_marker')
    if a is None:
        return
    tf_changed = _sync_tool_frame_from_entries()
    try:
        key = tuple(float(v) for v in RUN['VR_angles'])
    except Exception:
        return
    if not tf_changed and key == RUN.get('tcp_key'):
        return
    try:
        fk = kinematics.forward_kinematics(RUN['VR_angles'])
        R = ar4_to_R(*[math.degrees(v) for v in fk[3:6]])
        a.SetUserMatrix(_vtk_from_np(_H(R, np.array(fk[:3], float))))
        RUN['tcp_key'] = key
    except Exception as e:
        logger.debug(f"TCP marker FK failed: {e}")


def _program_loaded():
    try:
        return bool(ProgEntryField.get().strip()) and tab1.progView.size() > 0
    except Exception:
        return False


def update_teach_visibility():
    panel = RUN.get('node_panel')
    if not panel:
        return
    loaded = _program_loaded()
    if loaded != panel.get('prog_loaded'):
        panel['prog_loaded'] = loaded
        panel['size'] = None
        layout_node_panel()


def _teach_target_index():
    n = len(RUN['vr_nodes'])
    sel = RUN.get('vr_selected')
    if sel is not None and 0 <= sel < n:
        return sel
    return n - 1 if n else None


def refresh_node_list():
    panel = RUN.get('node_panel')
    overlay = RUN.get('node_renderer')
    if not panel or overlay is None:
        return

    for row, btn, _ in panel['rows']:
        overlay.RemoveViewProp(row)
        overlay.RemoveViewProp(btn)
    panel['rows'] = []

    nodes = RUN['vr_nodes']
    panel['header'].SetInput(f" Nodes: {len(nodes)}   click=add  ctrl+click=select  "
                             f"shift=snap  t=to tool  z=undo  esc=deselect ")
    tgt = _teach_target_index()
    num = f" #{tgt + 1}" if tgt is not None else ""
    panel['teach_mj'].SetInput(f" [ Teach MJ{num} ] ")
    panel['teach_ml'].SetInput(f" [ Teach ML{num} ] ")
    panel['teachall_mj'].SetInput(f" [ Teach all MJ ({len(nodes)}) ] ")
    panel['teachall_ml'].SetInput(f" [ Teach all ML ({len(nodes)}) ] ")
    _set_rnd_field_text()

    first = max(0, len(nodes) - MAX_LISTED_NODES)
    panel['more'].SetInput(f" ... {first} earlier node(s) " if first else "")

    for i in range(first, len(nodes)):
        x, y, z = nodes[i]['pos']
        rz, ry, rx = R_to_ar4(nodes[i]['R'])
        sel = (RUN.get('vr_selected') == i)
        row = _make_text(color=NODE_SEL_COLOR if sel else (1.0, 1.0, 1.0))
        row.SetInput(f" {i+1:>3}: X{x:8.1f} Y{y:8.1f} Z{z:8.1f}  Rz{rz:7.1f} Ry{ry:7.1f} Rx{rx:7.1f} ")
        btn = _make_text(color=(1.0, 0.35, 0.35), bold=True)
        btn.SetInput(" X ")
        overlay.AddViewProp(row)
        overlay.AddViewProp(btn)
        panel['rows'].append((row, btn, i))

    panel['size'] = None
    layout_node_panel()


def layout_node_panel():
    panel = RUN.get('node_panel')
    rw = RUN.get('render_window')
    if not panel or rw is None:
        return
    w, h = rw.GetSize()
    if panel['size'] == (w, h):
        return
    panel['size'] = (w, h)

    hits = []
    y = h - NODE_PANEL_TOP

    panel['header'].SetPosition(NODE_PANEL_X, y)
    hits.append((NODE_PANEL_X, y - NODE_ROW_H, NODE_PANEL_X + 800, y, 'swallow', None))
    y -= NODE_ROW_H

    panel['clear'].SetPosition(NODE_PANEL_X, y)
    hits.append((NODE_PANEL_X, y - NODE_ROW_H, NODE_PANEL_X + 160, y, 'clear', None))
    y -= NODE_ROW_H

    show_teach = bool(panel.get('prog_loaded')) and len(RUN['vr_nodes']) > 0
    teach_rows = (
        (('teach_mj', 'teach_mj'), ('teach_ml', 'teach_ml')),
        (('teachall_mj', 'teachall_mj'), ('teachall_ml', 'teachall_ml')),
        (('rnd_label', 'swallow'), ('rnd_minus', 'rnd_minus'),
         ('rnd_value', 'rnd_value'), ('rnd_plus', 'rnd_plus')),
    )
    for row_items in teach_rows:
        for key, _ in row_items:
            panel[key].SetVisibility(1 if show_teach else 0)
    if show_teach:
        for row_items in teach_rows:
            x = NODE_PANEL_X
            for key, action in row_items:
                w = len(panel[key].GetInput() or "") * NODE_CHAR_W
                panel[key].SetPosition(x, y)
                hits.append((x, y - NODE_ROW_H, x + w, y, action, None))
                # labels and the -/value/+ group sit tight; buttons get a gap
                x += w + (0 if key.startswith('rnd_') else NODE_BTN_GAP)
            y -= NODE_ROW_H

    if panel['more'].GetInput():
        panel['more'].SetPosition(NODE_PANEL_X, y)
        hits.append((NODE_PANEL_X, y - NODE_ROW_H, NODE_PANEL_X + 300, y, 'swallow', None))
        y -= NODE_ROW_H
    else:
        panel['more'].SetPosition(-1000, -1000)

    for row, btn, idx in panel['rows']:
        btn.SetPosition(NODE_PANEL_X, y)
        row.SetPosition(NODE_PANEL_X + 40, y)
        hits.append((NODE_PANEL_X,      y - NODE_ROW_H, NODE_PANEL_X + 38, y, 'delete', idx))
        hits.append((NODE_PANEL_X + 40, y - NODE_ROW_H, NODE_PANEL_X + NODE_ROW_HIT_W, y, 'select', idx))
        y -= NODE_ROW_H

    panel['hits'] = hits


def node_panel_click(x, y):
    panel = RUN.get('node_panel')
    if not panel:
        return False
    layout_node_panel()
    if VR_DEBUG:
        print(f"[VR click] x={x} y={y} window={RUN['render_window'].GetSize()}")
    hit = None
    for h in panel.get('hits', []):
        if h[0] <= x <= h[2] and h[1] <= y <= h[3]:
            hit = h
            break

    # Clicking anywhere else finishes typing in the rounding field. A click in the
    # 3D view only confirms the value (it doesn't also drop a new node).
    if RUN.get('vr_rnd_edit') is not None and (hit is None or hit[4] != 'rnd_value'):
        rnd_edit_commit()
        if hit is None:
            return True

    if hit is not None:
        x0, y0, x1, y1, action, idx = hit
        if VR_DEBUG:
            print(f"[VR click]   -> {action} {'' if idx is None else idx + 1}")
        if action == 'delete':
            delete_node(idx)
        elif action == 'select':
            select_node(idx)
        elif action == 'clear':
            clear_nodes()
        elif action in ('teach_mj', 'teach_ml'):
            tgt = _teach_target_index()
            if tgt is not None:
                teach_node(tgt, "Move L" if action == 'teach_ml' else "Move J")
        elif action == 'teachall_mj':
            teach_all_nodes("Move J")
        elif action == 'teachall_ml':
            teach_all_nodes("Move L")
        elif action == 'rnd_minus':
            _set_vr_rounding(_vr_rounding() - VR_RND_STEP)
        elif action == 'rnd_plus':
            _set_vr_rounding(_vr_rounding() + VR_RND_STEP)
        elif action == 'rnd_value':
            if RUN.get('vr_rnd_edit') is None:
                rnd_edit_begin()
            else:
                rnd_edit_commit()
        return True
    return False


# ---------------------------------------------------------------- triad (gizmo)

def _gizmo_create(overlay):
    arrow = vtk.vtkArrowSource()
    arrow.SetShaftRadius(0.025)
    arrow.SetTipRadius(0.07)
    arrow.SetTipLength(0.25)
    arrow.SetShaftResolution(16)
    arrow.SetTipResolution(16)
    arrow.Update()

    torus = vtk.vtkParametricTorus()
    torus.SetRingRadius(0.65)
    torus.SetCrossSectionRadius(0.02)
    tsrc = vtk.vtkParametricFunctionSource()
    tsrc.SetParametricFunction(torus)
    tsrc.SetUResolution(72)
    tsrc.SetVResolution(8)
    tsrc.Update()

    actors = {}

    def _mk(pd, tf, color, name):
        f = vtk.vtkTransformPolyDataFilter()
        f.SetInputData(pd)
        f.SetTransform(tf)
        f.Update()
        m = vtk.vtkPolyDataMapper()
        m.SetInputData(f.GetOutput())
        a = vtk.vtkActor()
        a.SetMapper(m)
        prop = a.GetProperty()
        prop.SetColor(*color)
        prop.SetAmbient(0.6)
        prop.SetDiffuse(0.5)
        a.SetVisibility(0)
        a.PickableOn()
        overlay.AddActor(a)
        actors[name] = a

    for i, ax in enumerate('xyz'):
        tf_t = vtk.vtkTransform()   # arrow source points along +X
        tf_r = vtk.vtkTransform()   # torus axis is +Z
        if ax == 'x':
            tf_r.RotateY(90)
        elif ax == 'y':
            tf_t.RotateZ(90)
            tf_r.RotateX(90)
        else:
            tf_t.RotateY(-90)
        _mk(arrow.GetOutput(), tf_t, GIZMO_AXIS_COLORS[i], 't' + ax)
        _mk(tsrc.GetOutput(), tf_r, GIZMO_AXIS_COLORS[i], 'r' + ax)

    picker = vtk.vtkCellPicker()
    picker.SetTolerance(0.003)
    picker.PickFromListOn()
    for a in actors.values():
        picker.AddPickList(a)

    return {'actors': actors, 'picker': picker, 'target': None, 'drag': None, 'frame': None}


def _gizmo_frame(t):
    W = t['get']()
    R = _orthonormal(W[:3, :3])
    c = (W @ np.append(t['pivot'], 1.0))[:3]
    return R, c


def gizmo_set_target(t):
    g = RUN.get('gizmo')
    if not g:
        return
    g['target'] = t
    g['drag'] = None
    g['frame'] = None
    for a in g['actors'].values():
        a.SetVisibility(1 if t else 0)
    gizmo_sync()


def gizmo_sync():
    """Place the triad on its target, sized to stay roughly constant on screen. VTK-only."""
    g = RUN.get('gizmo')
    if not g or not g['target']:
        return
    t = g['target']
    if not t['alive']():
        gizmo_set_target(None)
        return
    R, c = _gizmo_frame(t)
    cam = RUN['renderer'].GetActiveCamera()
    if cam.GetParallelProjection():
        s = GIZMO_SCALE * cam.GetParallelScale()
    else:
        dist = np.linalg.norm(np.array(cam.GetPosition()) - c)
        s = GIZMO_SCALE * dist * math.tan(math.radians(cam.GetViewAngle()) / 2.0)
    M = _vtk_from_np(_H(R * s, c))
    for a in g['actors'].values():
        a.SetUserMatrix(M)
    g['frame'] = (R, c)


def gizmo_try_begin_drag(x, y):
    """Called from the VTK press callback (VTK/numpy only). True if a triad handle was grabbed."""
    g = RUN.get('gizmo')
    if not g or not g['target'] or RUN.get('node_renderer') is None:
        return False
    gizmo_sync()
    if g['frame'] is None or not g['picker'].Pick(x, y, 0, RUN['node_renderer']):
        return False
    hit = g['picker'].GetActor()
    name = next((n for n, a in g['actors'].items() if a == hit), None)
    if name is None:
        return False
    R, c = g['frame']
    axis = R[:, 'xyz'.index(name[1])].copy()
    o, d = _display_ray(x, y)
    ref = _axis_param(c, axis, o, d) if name[0] == 't' else _plane_vec(c, axis, o, d)
    if ref is None:
        return False
    g['drag'] = {'name': name, 'axis': axis, 'c': c.copy(), 'ref': ref,
                 'W0': g['target']['get']().copy()}
    g['actors'][name].GetProperty().SetColor(*GIZMO_ACTIVE_COLOR)
    return True


def gizmo_drag(x, y, snap=False):
    """Called from the VTK mouse-move callback (VTK/numpy only)."""
    g = RUN.get('gizmo')
    dr = g['drag'] if g else None
    if not dr:
        return
    o, d = _display_ray(x, y)
    a, c, W0 = dr['axis'], dr['c'], dr['W0']
    if dr['name'][0] == 't':
        t = _axis_param(c, a, o, d)
        if t is None:
            return
        delta = t - dr['ref']
        if snap:
            delta = round(delta / GIZMO_SNAP_MM) * GIZMO_SNAP_MM
        W = _T(a * delta) @ W0
    else:
        v = _plane_vec(c, a, o, d)
        if v is None:
            return
        v0 = dr['ref']
        ang = math.atan2(a @ np.cross(v0, v), v0 @ v)
        if snap:
            step = math.radians(GIZMO_SNAP_DEG)
            ang = round(ang / step) * step
        W = _T(c) @ _H(_rot_axis(a, ang), np.zeros(3)) @ _T(-c) @ W0
    g['target']['set'](W)
    gizmo_sync()
    if RUN.get('render_window') is not None:
        RUN['render_window'].Render()


def gizmo_end_drag():
    """Called from the VTK release callback. Hands follow-up work to the Tk loop."""
    g = RUN.get('gizmo')
    if not g or not g['drag']:
        return
    name = g['drag']['name']
    g['actors'][name].GetProperty().SetColor(*GIZMO_AXIS_COLORS['xyz'.index(name[1])])
    g['drag'] = None
    _vr_queue().put(('gizmo_done',))


def _bounds_center(actor):
    try:
        b = actor.GetMapper().GetInput().GetBounds()
        return np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
    except Exception:
        return np.zeros(3)


def _target_for_node(nd):
    return {'kind': 'node', 'name': None, 'obj': nd,
            'get': lambda: _H(nd['R'], nd['pos']),
            'set': lambda W: _node_set_matrix(nd, W),
            'pivot': np.zeros(3),
            'alive': lambda: any(n is nd for n in RUN['vr_nodes'])}


def _imported_is_attached(name):
    return name in RUN.get('attached_imported_actors', set())


def _target_for_imported(name):
    """Triad target for a Virtual Import CAD. Free parts use Position/Orientation (what
    the Virtual Import fields and imported_cad_scene.json use); parts attached to the
    gripper keep a flange-local UserMatrix, matching attachtogripper()."""
    act = imported_actors[name]

    def _get():
        if _imported_is_attached(name):
            return _np_from_vtk(get_world_matrix("Link 6-1.STL")) @ _np_from_vtk(act.GetMatrix())
        return _np_from_vtk(act.GetMatrix())

    def _set(W):
        act.SetOrigin(0, 0, 0)
        act.SetScale(1, 1, 1)
        if _imported_is_attached(name):
            local = np.linalg.inv(_np_from_vtk(get_world_matrix("Link 6-1.STL"))) @ W
            act.SetPosition(0, 0, 0)
            act.SetOrientation(0, 0, 0)
            act.SetUserMatrix(_vtk_from_np(local))
        else:
            tf = vtk.vtkTransform()
            tf.SetMatrix(_vtk_from_np(W))
            act.SetUserMatrix(None)
            act.SetUserTransform(None)
            act.SetPosition(*tf.GetPosition())
            act.SetOrientation(*tf.GetOrientation())

    return {'kind': 'imported', 'name': name, 'obj': act,
            'get': _get,
            'set': _set,
            'pivot': _bounds_center(act),
            'alive': lambda: imported_actors.get(name) is act}


def _tool_parent_world():
    W = np.eye(4)
    for link in TOOL_PARENT_CHAIN:
        W = W @ _np_from_vtk(RUN['assemblies'][link].GetMatrix())
    return W


def _target_for_tool(name):
    asm = RUN['assemblies'][name]
    base_tf = RUN['base_transforms'][name]

    def _get():
        return _tool_parent_world() @ _np_from_vtk(asm.GetMatrix())

    def _set(W):
        joint = _np_from_vtk(RUN['joint_transforms'][name].GetMatrix())
        local = np.linalg.inv(_tool_parent_world()) @ W @ np.linalg.inv(joint)
        base_tf.SetMatrix(_vtk_from_np(local))   # mount offset on the flange

    return {'kind': 'tool', 'name': name, 'obj': asm,
            'get': _get, 'set': _set,
            'pivot': _bounds_center(RUN['actors'][name]),
            'alive': lambda: name in RUN['assemblies']}


def _robot_tcp_world():
    """World matrix of the virtual robot's TCP (same maths as the TCP marker)."""
    fk = kinematics.forward_kinematics(RUN['VR_angles'])
    return _H(ar4_to_R(*[math.degrees(v) for v in fk[3:6]]), np.array(fk[:3], float))


def _target_for_robot():
    """Triad target that drives the robot itself: dragging the handle moves the TCP in
    Cartesian space. Each mouse move is solved with IK from the current joints; a pose the
    arm cannot reach (or that needs a wrist flip) is ignored, so the handle simply stops at
    the last reachable pose. When the drag ends the pose is sent to the real robot as a
    Move J (see on_gizmo_done)."""
    def _set(W):
        if drive_lock.locked():
            return                                  # a move is already playing
        cur = RUN['VR_angles']
        ref = np.array(_tcp_pose_deg(cur)[3:], float)
        pose = list(W[:3, 3]) + list(R_to_ar4(_orthonormal(W[:3, :3]), ref=ref))
        try:
            sol = kinematics.SolveInverseKinematics(pose, cur, LIVE_JOG_MAX_JOINT_JUMP_DEG)
        except Exception as e:
            logger.debug(f"TCP drag IK failed: {e}")
            return
        if sol is None or len(sol) == 0:
            return
        RUN['VR_angles'] = [float(v) for v in sol]
        setStepMonitorsVR()
        RUN['robot_drag_pose'] = pose

    return {'kind': 'robot', 'name': 'robot', 'obj': None,
            'get': _robot_tcp_world,
            'set': _set,
            'pivot': np.zeros(3),
            'alive': lambda: bool(RUN.get('vtk_running'))}


def _tcp_pose_deg(angles):
    fk = kinematics.forward_kinematics(angles)
    return list(fk[:3]) + [math.degrees(v) for v in fk[3:]]


def select3d(x, y):
    """Ctrl+click: select a node sphere, imported STL, gripper tool, or the robot itself
    (any arm link) for the triad."""
    if node_panel_click(x, y):
        return
    overlay = RUN.get('node_renderer')

    sp = RUN.get('node_sphere_picker')
    if sp is not None and overlay is not None and RUN['vr_nodes']:
        sp.InitializePickList()
        for nd in RUN['vr_nodes']:
            sp.AddPickList(nd['actor'])
        if sp.Pick(x, y, 0, overlay):
            hit = sp.GetActor()
            for i, nd in enumerate(RUN['vr_nodes']):
                if nd['actor'] == hit:
                    select_node(i, force=True)
                    return

    picker = RUN.get('node_picker')
    if picker is not None and picker.Pick(x, y, 0, RUN['renderer']):
        path_ = picker.GetPath()
        prop = path_.GetLastNode().GetViewProp() if path_ is not None else picker.GetActor()
        for name, act in imported_actors.items():
            if act == prop:
                select_imported_cad(name)
                _select_other(_target_for_imported(name), f"Selected {name} - drag the triad")
                return
        for name in GIZMO_TOOL_NAMES:
            if RUN['actors'].get(name) == prop:
                _select_other(_target_for_tool(name), f"Selected {name} - drag the triad")
                return
        for name, act in RUN['actors'].items():
            if act == prop and name not in GIZMO_TOOL_NAMES:
                _select_other(_target_for_robot(),
                              "Robot selected - drag the triad to move the TCP (shift = snap, Esc = done)")
                return
    deselect_all()


def _select_other(target, msg):
    RUN['vr_selected'] = None
    _apply_node_colors()
    refresh_node_list()
    gizmo_set_target(target)
    _status(msg)
    _render_vr()


def deselect_all():
    RUN['vr_selected'] = None
    gizmo_set_target(None)
    if selected_imported_filename:
        deselect_imported_cad()
    _apply_node_colors()
    refresh_node_list()
    _render_vr()


def on_gizmo_done():
    """Tk-side follow-up after a triad drag."""
    g = RUN.get('gizmo')
    t = g['target'] if g else None
    if not t:
        return
    W = t['get']()
    rz, ry, rx = R_to_ar4(W[:3, :3])
    if t['kind'] == 'node':
        save_click_points()
        refresh_node_list()
    elif t['kind'] == 'imported':
        select_imported_cad(t['name'])          # fills X/Y/Z + X/Y/Z rotation fields
        if _imported_is_attached(t['name']):
            logger.info(f"{t['name']} (attached) moved to X {W[0,3]:.2f} Y {W[1,3]:.2f} Z {W[2,3]:.2f}")
        else:
            save_imported_cad_scene()
    elif t['kind'] == 'robot':
        pose = RUN.pop('robot_drag_pose', None)
        if pose is not None:
            if RUN['offlineMode']:
                refresh_gui_from_joint_angles(RUN['VR_angles'])
            else:
                _move_j_to_pose(pose)               # the real robot follows the handle
    elif t['kind'] == 'tool':
        local = _np_from_vtk(RUN['base_transforms'][t['name']].GetMatrix())
        lrz, lry, lrx = R_to_euler_zyx(_orthonormal(local[:3, :3]))
        logger.info(f"{t['name']} mount offset: X {local[0,3]:.2f} Y {local[1,3]:.2f} Z {local[2,3]:.2f} "
                    f"Rz {lrz:.2f} Ry {lry:.2f} Rx {lrx:.2f}")
    _render_vr()


def snap_selected_to_tool():
    g = RUN.get('gizmo')
    t = g['target'] if g else None
    if not t or t['kind'] != 'node':
        _status("Select a node first (click its row or ctrl+click it)", "Warn.TLabel")
        return
    try:
        fk = kinematics.forward_kinematics(RUN['VR_angles'])
        pos = np.array(fk[:3], float)
        R = ar4_to_R(*[math.degrees(v) for v in fk[3:6]])
    except Exception as e:
        logger.error(f"Forward kinematics failed: {e}")
        return
    _node_set_matrix(t['obj'], _H(R, pos))
    save_click_points()
    refresh_node_list()
    gizmo_sync()
    _render_vr()


# ---------------------------------------------------------------- teaching

def _status(msg, style="OK.TLabel"):
    try:
        almStatusLab.config(text=msg, style=style)
        almStatusLab2.config(text=msg, style=style)
    except Exception:
        pass


def _vr_rounding():
    """Rounding (mm) for Teach ML. First use takes the value from the main Rnd field."""
    v = RUN.get('vr_rnd')
    if v is None:
        try:
            v = max(0.0, min(VR_RND_MAX, float(roundEntryField.get())))
        except Exception:
            v = 0.0
        RUN['vr_rnd'] = v
    return v


def _fmt_rnd(v):
    return f"{v:g}"


def _set_rnd_field_text():
    panel = RUN.get('node_panel')
    if not panel:
        return
    buf = RUN.get('vr_rnd_edit')
    tp = panel['rnd_value'].GetTextProperty()
    if buf is not None:
        panel['rnd_value'].SetInput(f" {buf}_ mm ")
        tp.SetColor(1.0, 1.0, 0.3)
    else:
        panel['rnd_value'].SetInput(f" {_fmt_rnd(_vr_rounding())} mm ")
        tp.SetColor(1.0, 1.0, 1.0)


def _refresh_rnd_field():
    _set_rnd_field_text()
    panel = RUN.get('node_panel')
    if panel:
        panel['size'] = None
        layout_node_panel()
    _render_vr()


def _set_vr_rounding(v):
    RUN['vr_rnd'] = max(0.0, min(VR_RND_MAX, round(float(v), 2)))
    _refresh_rnd_field()
    _status(f"Teach ML rounding: {_fmt_rnd(RUN['vr_rnd'])} mm")


def rnd_edit_begin():
    RUN['vr_rnd_edit'] = ""
    _refresh_rnd_field()
    _status("Type the rounding in mm - Enter to set, Esc to cancel")


def rnd_edit_commit():
    buf = RUN.get('vr_rnd_edit')
    RUN['vr_rnd_edit'] = None
    if buf:
        try:
            _set_vr_rounding(float(buf))
            return
        except ValueError:
            pass
    _refresh_rnd_field()


def rnd_edit_cancel():
    RUN['vr_rnd_edit'] = None
    _refresh_rnd_field()


def rnd_edit_key(key):
    buf = RUN.get('vr_rnd_edit')
    if buf is None:
        return
    if key in ('return', 'kp_enter'):
        rnd_edit_commit()
        return
    if key == 'escape':
        rnd_edit_cancel()
        return
    if key in ('backspace', 'delete'):
        buf = buf[:-1]
    else:
        ch = key[3:] if key.startswith('kp_') else key       # numeric keypad
        if ch in ('period', 'decimal'):
            ch = '.'
        if len(buf) < 6 and (ch.isdigit() and len(ch) == 1 or (ch == '.' and '.' not in buf)):
            buf += ch
    RUN['vr_rnd_edit'] = buf
    _refresh_rnd_field()


def _build_move_line(nd, move_type="Move J", rounding=0.0):
    """'Move J' or 'Move L' program line for a node using its own pose, or None if unreachable.
    Only the end pose is checked for reach; a Move L path itself is not checked."""
    R = nd['R']
    p = nd['pos'] - NODE_APPROACH_OFFSET * R[:, 2]     # back off along the arrow
    rz, ry, rx = R_to_ar4(R)
    try:
        target = np.array([p[0], p[1], p[2], rz, ry, rx], dtype=float)
        sol = kinematics.SolveInverseKinematics(target, RUN['VR_angles'])
        if sol is None or not np.all(np.isfinite(np.asarray(sol, dtype=float))):
            return None
    except Exception as e:
        logger.debug(f"IK check failed for node at {p}: {e}")
        return None

    checkSpeedVals()
    speedPrefix = {"Seconds": "Ss", "mm per Sec": "Sm"}.get(speedOption.get(), "Sp")
    if move_type == "Move L":
        # Same layout as a Move L taught from the main window (wrist config "A")
        return (f"Move L [*] X {p[0]:.3f} Y {p[1]:.3f} Z {p[2]:.3f} "
                f"Rz {rz:.3f} Ry {ry:.3f} Rx {rx:.3f} "
                f"J7 {CAL['J7PosCur']} J8 {CAL['J8PosCur']} J9 {CAL['J9PosCur']} "
                f"{speedPrefix} {speedEntryField.get()} Ac {ACCspeedField.get()} "
                f"Dc {DECspeedField.get()} Rm {ACCrampField.get()} Rnd {_fmt_rnd(rounding)} $ A")
    wc = RUN.get('WC') or "N"
    return (f"Move J [*] X {p[0]:.3f} Y {p[1]:.3f} Z {p[2]:.3f} "
            f"Rz {rz:.3f} Ry {ry:.3f} Rx {rx:.3f} "
            f"J7 {CAL['J7PosCur']} J8 {CAL['J8PosCur']} J9 {CAL['J9PosCur']} "
            f"{speedPrefix} {speedEntryField.get()} Ac {ACCspeedField.get()} "
            f"Dc {DECspeedField.get()} Rm {ACCrampField.get()} $ {wc}")


def _append_program_lines(lines):
    for line in lines:
        tab1.progView.insert(END, bytes(line + '\n', 'utf-8'))
    last = tab1.progView.size() - 1
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(last)
    tab1.progView.see(last)
    curRowEntryField.delete(0, 'end')
    curRowEntryField.insert(0, last)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path, 'w', encoding='utf-8') as f:
        for item in tab1.progView.get(0, END):
            if isinstance(item, bytes):
                item = item.decode('utf-8')
            f.write(item.strip())
            f.write('\n')


def teach_node(idx, move_type="Move J"):
    if not _program_loaded():
        _status("Load a program before teaching nodes", "Warn.TLabel")
        return
    if not (0 <= idx < len(RUN['vr_nodes'])):
        return
    line = _build_move_line(RUN['vr_nodes'][idx], move_type, _vr_rounding())
    if line is None:
        msg = f"Node {idx + 1} is out of reach - not taught"
        logger.warning(msg)
        _status(msg, "Warn.TLabel")
        return
    try:
        _append_program_lines([line])
    except Exception as e:
        logger.error(f"Could not write program file: {e}")
        _status("Could not write program file - see log", "Alarm.TLabel")
        return
    kind = f"Move L (Rnd {_fmt_rnd(_vr_rounding())})" if move_type == "Move L" else "Move J"
    msg = f"Node {idx + 1} taught as {kind} to end of {ProgEntryField.get()}"
    logger.info(msg)
    _status(msg)


def teach_all_nodes(move_type="Move J"):
    """Teach every node. For Move L, all nodes get the field's rounding except the
    last one, which gets Rnd 0 so the sequence ends exactly on the final point."""
    if not _program_loaded():
        _status("Load a program before teaching nodes", "Warn.TLabel")
        return
    lines, skipped = [], []
    nodes = RUN['vr_nodes']
    for i, nd in enumerate(nodes):
        rnd = _vr_rounding() if i < len(nodes) - 1 else 0.0
        line = _build_move_line(nd, move_type, rnd)
        if line is None:
            skipped.append(i + 1)
        else:
            lines.append(line)
    if lines:
        try:
            _append_program_lines(lines)
        except Exception as e:
            logger.error(f"Could not write program file: {e}")
            _status("Could not write program file - see log", "Alarm.TLabel")
            return
    kind = (f"Move L (Rnd {_fmt_rnd(_vr_rounding())}, last 0)" if move_type == "Move L" else "Move J")
    msg = f"Taught {len(lines)} node(s) as {kind} to end of {ProgEntryField.get()}"
    if skipped:
        msg += f" - skipped unreachable: {', '.join(map(str, skipped))}"
    logger.info(msg)
    _status(msg, "Warn.TLabel" if skipped else "OK.TLabel")


# ---------------------------------------------------------------- nodes

def _render_vr():
    if RUN.get('vtk_running') and RUN.get('render_window') is not None:
        RUN['render_window'].Render()


def _node_arrow_polydata():
    """Arrow along +Z ending at the origin (tip on the point), NODE_ARROW_LEN long."""
    global _NODE_ARROW_PD
    if _NODE_ARROW_PD is None:
        src = vtk.vtkArrowSource()
        src.SetShaftRadius(0.04)
        src.SetTipRadius(0.12)
        src.SetTipLength(0.3)
        src.SetShaftResolution(16)
        src.SetTipResolution(16)
        tf = vtk.vtkTransform()
        tf.PostMultiply()
        tf.RotateY(-90)                       # +X -> +Z
        tf.Translate(0, 0, -1)                # tip at origin
        tf.Scale(NODE_ARROW_LEN, NODE_ARROW_LEN, NODE_ARROW_LEN)
        f = vtk.vtkTransformPolyDataFilter()
        f.SetInputConnection(src.GetOutputPort())
        f.SetTransform(tf)
        f.Update()
        _NODE_ARROW_PD = f.GetOutput()
    return _NODE_ARROW_PD


def _update_node_actors(nd):
    p = nd['pos']
    nd['actor'].SetPosition(*p)
    nd['arrow'].SetUserMatrix(_vtk_from_np(_H(nd['R'], p)))
    nd['label'].SetPosition(p[0], p[1], p[2] + NODE_RADIUS * 2)


def _node_set_matrix(nd, W):
    nd['R'] = _orthonormal(W[:3, :3])
    nd['pos'] = np.array(W[:3, 3], float)
    _update_node_actors(nd)


def _apply_node_colors():
    sel = RUN.get('vr_selected')
    for i, nd in enumerate(RUN['vr_nodes']):
        c = NODE_SEL_COLOR if i == sel else NODE_COLOR
        nd['actor'].GetProperty().SetColor(*c)
        nd['label'].GetTextProperty().SetColor(*c)


def add_node(pos, R=None, save=True, quiet=False):
    overlay = RUN.get('node_renderer')
    if overlay is None:
        return
    pos = np.array(pos, float).reshape(3)
    R = _current_tool_R() if R is None else _orthonormal(R)
    n = len(RUN['vr_nodes']) + 1

    src = vtk.vtkSphereSource()
    src.SetRadius(NODE_RADIUS)
    src.SetThetaResolution(16)
    src.SetPhiResolution(16)
    m = vtk.vtkPolyDataMapper()
    m.SetInputConnection(src.GetOutputPort())
    sphere = vtk.vtkActor()
    sphere.SetMapper(m)
    sphere.GetProperty().SetColor(*NODE_COLOR)
    sphere.PickableOn()                       # for ctrl+click selection

    am = vtk.vtkPolyDataMapper()
    am.SetInputData(_node_arrow_polydata())
    arrow = vtk.vtkActor()
    arrow.SetMapper(am)
    arrow.GetProperty().SetColor(*NODE_ARROW_COLOR)
    arrow.PickableOff()

    label = vtk.vtkBillboardTextActor3D()
    label.SetInput(str(n))
    label.GetTextProperty().SetFontSize(16)
    label.GetTextProperty().SetColor(*NODE_COLOR)
    label.GetTextProperty().BoldOn()
    label.PickableOff()

    nd = {'pos': pos, 'R': R, 'actor': sphere, 'arrow': arrow, 'label': label}
    _update_node_actors(nd)
    overlay.AddActor(sphere)
    overlay.AddActor(arrow)
    overlay.AddActor(label)
    RUN['vr_nodes'].append(nd)

    if save:
        save_click_points()
    if not quiet:
        rz, ry, rx = R_to_ar4(R)
        logger.info(f"VR node {n}: X {pos[0]:.2f} Y {pos[1]:.2f} Z {pos[2]:.2f} "
                    f"Rz {rz:.2f} Ry {ry:.2f} Rx {rx:.2f}")
        refresh_node_list()
        _render_vr()


def delete_node(idx):
    overlay = RUN.get('node_renderer')
    nodes = RUN['vr_nodes']
    if overlay is None or not (0 <= idx < len(nodes)):
        return
    nd = nodes.pop(idx)
    for key in ('actor', 'arrow', 'label'):
        overlay.RemoveActor(nd[key])

    g = RUN.get('gizmo')
    if g and g['target'] and g['target'].get('obj') is nd:
        gizmo_set_target(None)

    sel = RUN.get('vr_selected')
    if sel == idx:
        RUN['vr_selected'] = None
    elif sel is not None and sel > idx:
        RUN['vr_selected'] = sel - 1

    for i, n in enumerate(nodes, 1):
        n['label'].SetInput(str(i))
    _apply_node_colors()
    save_click_points()
    logger.info(f"VR node {idx + 1} deleted ({len(nodes)} remaining)")
    refresh_node_list()
    _render_vr()


def select_node(idx, force=False):
    if not (0 <= idx < len(RUN['vr_nodes'])):
        return
    if force or RUN.get('vr_selected') != idx:
        RUN['vr_selected'] = idx
        gizmo_set_target(_target_for_node(RUN['vr_nodes'][idx]))
    else:
        RUN['vr_selected'] = None
        gizmo_set_target(None)
    _apply_node_colors()
    refresh_node_list()
    _render_vr()


def undo_last_node():
    if RUN['vr_nodes']:
        delete_node(len(RUN['vr_nodes']) - 1)


def clear_nodes():
    overlay = RUN.get('node_renderer')
    if overlay is not None:
        for nd in RUN['vr_nodes']:
            for key in ('actor', 'arrow', 'label'):
                overlay.RemoveActor(nd[key])
    g = RUN.get('gizmo')
    if g and g['target'] and g['target']['kind'] == 'node':
        gizmo_set_target(None)
    RUN['vr_nodes'] = []
    RUN['vr_selected'] = None
    save_click_points()
    refresh_node_list()
    _render_vr()


def pick_node(x, y):
    picker = RUN.get('node_picker')
    if picker is None:
        return
    if not picker.Pick(x, y, 0, RUN['renderer']) or picker.GetCellId() == -1:
        return
    p = np.array(picker.GetPickPosition(), float)

    R = None
    if NODE_ORIENT_FROM_SURFACE:
        n = np.array(picker.GetPickNormal(), float)
        try:
            cell = picker.GetDataSet().GetCell(picker.GetCellId())
            if cell.GetCellDimension() < 2:          # floor grid lines have no real normal
                n = np.array([0.0, 0.0, 1.0])
        except Exception:
            n = np.array([0.0, 0.0, 1.0])
        if np.linalg.norm(n) < 1e-9:
            n = np.array([0.0, 0.0, 1.0])
        cam_pos = np.array(RUN['renderer'].GetActiveCamera().GetPosition())
        if n @ (cam_pos - p) < 0:                    # make the normal face the viewer
            n = -n
        R = _frame_from_z(-n, _current_tool_R()[:, 0])   # arrow points into the surface
    add_node(p, R)


# ---------------------------------------------------------------- interactor style

class CustomInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    """VTK callbacks: VTK/numpy work only - anything touching Tk goes through VR_EVENTS."""
    CLICK_TOL_PX = 4

    # Class-level defaults: if Python drops its reference to this object, VTK re-wraps the
    # C++ style as a new instance WITHOUT calling __init__, and the callbacks then ran into
    # "no attribute '_dragging_gizmo'". RUN['vtk_style'] keeps the reference alive as well.
    renderer = None
    _press_pos = None
    _ctrl = False
    _dragging_gizmo = False

    def __init__(self, renderer):
        super().__init__()
        self.renderer = renderer
        self._press_pos = None
        self._ctrl = False
        self._dragging_gizmo = False
        self.AddObserver("LeftButtonPressEvent", self.on_left_button_down)
        self.AddObserver("LeftButtonReleaseEvent", self.on_left_button_up)
        self.AddObserver("MouseMoveEvent", self.on_mouse_move)
        self.AddObserver("KeyPressEvent", self.on_key_press)
        self.AddObserver("CharEvent", self.on_char)

    def on_char(self, obj, event):
        # VTK's built-in key actions are never wanted here: 'e'/'q' close the viewer, 'w'
        # wireframes the robot, 's'/'3'/'f'/'p' change the view - and a stray keystroke
        # that lands on the viewer instead of a text box did exactly that. Only 'r'
        # (reset the camera) is kept; the teach keys are handled in on_key_press.
        if RUN.get('vr_rnd_edit') is not None:
            return
        key = (self.GetInteractor().GetKeySym() or "").lower()
        if key == "r":
            self.OnChar()

    def on_left_button_down(self, obj, event):
        iren = self.GetInteractor()
        x, y = iren.GetEventPosition()
        if gizmo_try_begin_drag(x, y):
            self._dragging_gizmo = True
            self._press_pos = None
            return                                  # don't rotate the camera
        self._press_pos = (x, y)
        self._ctrl = bool(iren.GetControlKey())
        self.OnLeftButtonDown()

    def on_mouse_move(self, obj, event):
        if self._dragging_gizmo:
            iren = self.GetInteractor()
            x, y = iren.GetEventPosition()
            gizmo_drag(x, y, snap=bool(iren.GetShiftKey()))
            return
        self.OnMouseMove()

    def on_left_button_up(self, obj, event):
        if self._dragging_gizmo:
            self._dragging_gizmo = False
            gizmo_end_drag()
            return
        self.OnLeftButtonUp()
        if self._press_pos is None:
            return
        x0, y0 = self._press_pos
        x, y = self.GetInteractor().GetEventPosition()
        self._press_pos = None
        if abs(x - x0) <= self.CLICK_TOL_PX and abs(y - y0) <= self.CLICK_TOL_PX:
            _vr_queue().put(('select3d' if self._ctrl else 'pick', x, y))

    def on_key_press(self, obj, event):
        key = (self.GetInteractor().GetKeySym() or "").lower()
        if RUN.get('vr_rnd_edit') is not None:
            _vr_queue().put(('rnd_key', key))      # typing into the rounding field
            return
        if key == "z":
            _vr_queue().put(('undo',))
        elif key == "c":
            _vr_queue().put(('clear',))
        elif key == "t":
            _vr_queue().put(('snap',))
        elif key == "escape":
            _vr_queue().put(('deselect',))

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
    actor.GetProperty().SetColor(*ui.vtk_colors()['grid'])
    actor.GetProperty().SetLineWidth(1)

    RUN['renderer'].AddActor(actor)        

def on_close_event(obj, event):
    RUN['vtk_running'] = False
    RUN['node_renderer'] = None
    RUN['node_panel'] = None
    RUN['node_picker'] = None
    RUN['node_sphere_picker'] = None
    RUN['gizmo'] = None
    RUN['vr_nodes'] = []
    RUN['vr_selected'] = None
    try:
        obj.GetRenderWindow().Finalize()
        obj.TerminateApp()
    except:
        pass
    # forget the closed window so the Virtual Robot button opens a fresh viewer next time
    RUN['render_window'] = None
    RUN['interactor'] = None
    RUN['vtk_style'] = None
    
def update_vtk(render_window, root_widget):
    if RUN['vtk_running'] and not IS_WINDOWS and RUN.get('interactor') is not None:
        # On X11 Tk's event loop doesn't service VTK's own display connection, so the
        # viewer would never see mouse/keyboard/close events. Pump them here.
        # (On Windows they arrive through the shared message loop already.)
        try:
            RUN['interactor'].ProcessEvents()
        except Exception as e:
            logger.debug(f"VTK ProcessEvents failed: {e}")
    if RUN['vtk_running']:
        if RUN.get('dialog_open'):
            # a native file dialog is up: leave the message loop to it (see ask_open_file)
            root_widget.after(100, lambda: update_vtk(RUN['render_window'], root_widget))
            return
        process_vr_events()
        update_teach_visibility()
        layout_node_panel()

        # Only redraw when the robot moved (or once a second as a catch-all). Rendering the
        # full STL scene every 16 ms starved Tk: when a frame took longer than the tick the
        # main window never got idle time and stopped responding to clicks and typing.
        # Camera moves, gizmo drags and selections render themselves through VTK/_render_vr.
        try:
            key = tuple(round(float(v), 4) for v in RUN['VR_angles'])
        except Exception:
            key = None
        now = time.monotonic()
        dt = 0.0
        if key != RUN.get('vr_last_key') or now - RUN.get('vr_last_render', 0.0) > 1.0:
            RUN['vr_last_key'] = key
            update_joint_angles()
            update_tcp_marker()
            gizmo_sync()
            t0 = time.monotonic()
            try:
                request_render()
            except Exception as e:
                logger.debug(f"VTK render failed: {e}")
            RUN['vr_last_render'] = time.monotonic()
            dt = RUN['vr_last_render'] - t0
        # leave Tk at least as much time as the render took (never faster than ~60 Hz)
        delay = max(16, int(dt * 2000))
        root_widget.after(delay, lambda: update_vtk(RUN['render_window'], root_widget))

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

    if RUN.get('vtk_running') and RUN.get('render_window') is not None:
        # Already open: a second click used to build a second viewer on top of the first
        # (two render windows, two 60 Hz update loops) and the program froze. Just bring
        # the existing window forward.
        try:
            RUN['render_window'].Render()
            if IS_WINDOWS:
                import win32gui, win32con
                hwnd = win32gui.FindWindow(None, "AR4 Virtual Robot Viewer")
                if hwnd:
                    win32gui.ShowWindow(hwnd, win32con.SW_RESTORE)
                    win32gui.SetForegroundWindow(hwnd)
        except Exception as e:
            logger.debug(f"Could not raise the virtual robot window: {e}")
        return

    RUN['vtk_running'] = True

    RUN['renderer'] = vtk.vtkRenderer()
    RUN['render_window'] = vtk.vtkRenderWindow() 
    RUN['render_window'].SetWindowName("AR4 Virtual Robot Viewer")
    RUN['interactor'] = vtk.vtkRenderWindowInteractor()
    RUN['render_window'].AddRenderer(RUN['renderer'])
    RUN['interactor'].SetRenderWindow(RUN['render_window'])

    style = CustomInteractorStyle(RUN['renderer'])
    RUN['vtk_style'] = style                      # keep the Python wrapper (and its state) alive
    RUN['interactor'].SetInteractorStyle(style)

    RUN['render_window'].SetSize(1024, 768)
    _vc = ui.vtk_colors()
    RUN['renderer'].SetBackground(*_vc['bottom'])
    RUN['renderer'].SetBackground2(*_vc['top'])
    RUN['renderer'].GradientBackgroundOn()

    build_robot_actors(RUN['renderer'])
    add_floor_grid(RUN['renderer'])
    # Load previously imported CAD models and EOAT tool
    load_imported_cad_scene()
    load_eoat_scene()
    RUN['cad_scene_loaded'] = True

    setup_node_overlay()

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
    '''Continuously keep VTK window on top using window title'''
    if not IS_WINDOWS:
        return  # uses win32gui; on Linux the window manager handles stacking
    def keep_on_top():
        import time
        import platform

        time.sleep(0.5)  # Initial delay for window creation

        # The old loop re-stacked the viewer every 0.3 s. Re-asserting TOPMOST on a window
        # that already has it is not free: it shuffles the z-order under whatever the user
        # is working in (file dialogs, text boxes). Set it once and only repair it when
        # the flag has actually been lost, never while a dialog is open.
        while RUN['vtk_running']:
            try:
                if platform.system() == "Windows" and not RUN.get('dialog_open'):
                    import win32gui
                    import win32con

                    hwnd = win32gui.FindWindow(None, "AR4 Virtual Robot Viewer")

                    if hwnd:
                        ex = win32gui.GetWindowLong(hwnd, win32con.GWL_EXSTYLE)
                        if not (ex & win32con.WS_EX_TOPMOST):
                            win32gui.SetWindowPos(
                                hwnd,
                                win32con.HWND_TOPMOST,
                                0, 0, 0, 0,
                                win32con.SWP_NOMOVE |
                                win32con.SWP_NOSIZE |
                                win32con.SWP_NOACTIVATE
                            )

            except Exception as e:
                logger.debug(f"VTK topmost update failed: {e}")

            time.sleep(2.0)

    import threading
    threading.Thread(target=keep_on_top, daemon=True).start()

def save_imported_cad_scene():
    scene_data = []

    for filename, actor in imported_actors.items():
        x, y, z = actor.GetPosition()
        rx, ry, rz = actor.GetOrientation()

        scene_data.append({
            "filename": filename,
            "x": x,
            "y": y,
            "z": z,
            "rx": rx,
            "ry": ry,
            "rz": rz
        })

    os.makedirs(CAD_IMPORTED_DIR, exist_ok=True)

    with open(CAD_SCENE_FILE, "w") as f:
        json.dump(scene_data, f, indent=4)



def update_stl_transform():
    if not selected_imported_filename:
        messagebox.showwarning("No CAD Selected", "Please select an imported CAD file first.")
        return

    actor = imported_actors.get(selected_imported_filename)
    if not actor:
        return

    try:
        x = float(x_var.get())
        y = float(y_var.get())
        z = float(z_var.get())
        rx = float(rx_var.get())
        ry = float(ry_var.get())
        rz = float(rz_var.get())
    except ValueError:
        messagebox.showerror("Invalid Entry", "Position and rotation values must be numbers.")
        return

    actor.SetPosition(x, y, z)
    actor.SetOrientation(rx, ry, rz)

    RUN['render_window'].Render()

    save_imported_cad_scene()

    




stl_name_var = tk.StringVar()
x_var = tk.StringVar(value="0")
y_var = tk.StringVar(value="0")
z_var = tk.StringVar(value="0")
rx_var = tk.StringVar(value="0")
ry_var = tk.StringVar(value="0")
rz_var = tk.StringVar(value="0")
tool_name_var = StringVar()
tool_x_var = StringVar(value="0")
tool_y_var = StringVar(value="0")
tool_z_var = StringVar(value="0")
tool_rx_var = StringVar(value="0")
tool_ry_var = StringVar(value="0")
tool_rz_var = StringVar(value="0")



def refresh_imported_cad_list():
    importedCadList.delete(0, END)

    for filename in imported_actors.keys():
        importedCadList.insert(END, filename)


def _gizmo_follow_imported(name):
    """Put the 3D move/rotate triad on an imported CAD (or hide it when name is None)."""
    if not RUN.get('vtk_running') or RUN.get('gizmo') is None:
        return
    g = RUN['gizmo']
    cur = g.get('target')
    if name is None:
        if cur and cur['kind'] == 'imported':
            gizmo_set_target(None)
        return
    if name not in imported_actors:
        return
    if cur and cur['kind'] == 'imported' and cur['name'] == name:
        return
    if RUN.get('vr_selected') is not None:
        RUN['vr_selected'] = None
        _apply_node_colors()
        refresh_node_list()
    gizmo_set_target(_target_for_imported(name))


def select_imported_cad(filename):
    global selected_imported_filename

    selected_imported_filename = filename

    importedCadList.selection_clear(0, END)

    filenames = list(imported_actors.keys())
    if filename in filenames:
        index = filenames.index(filename)
        importedCadList.selection_set(index)
        importedCadList.activate(index)

    load_selected_cad_transform()
    update_imported_cad_highlight()
    _gizmo_follow_imported(filename)


def on_imported_cad_selected(event=None):
    global selected_imported_filename

    selection = importedCadList.curselection()
    if not selection:
        return

    selected_imported_filename = importedCadList.get(selection[0])
    load_selected_cad_transform()
    update_imported_cad_highlight()
    _gizmo_follow_imported(selected_imported_filename)


def load_selected_cad_transform():
    if not selected_imported_filename:
        return

    actor = imported_actors.get(selected_imported_filename)
    if not actor:
        return

    x, y, z = actor.GetPosition()
    rx, ry, rz = actor.GetOrientation()

    stl_name_var.set(selected_imported_filename)
    x_var.set(round(x, 3))
    y_var.set(round(y, 3))
    z_var.set(round(z, 3))
    rx_var.set(round(rx, 3))
    ry_var.set(round(ry, 3))
    rz_var.set(round(rz, 3))


def set_imported_cad_appearance(actor):
    prop = actor.GetProperty()
    prop.SetColor(0.70, 0.70, 0.70)
    prop.SetAmbient(0.25)
    prop.SetDiffuse(0.75)
    prop.SetSpecular(0.08)
    prop.SetSpecularPower(8)


def set_EOAT_cad_appearance(actor):
    prop = actor.GetProperty()
    prop.SetColor(0.45, 0.45, 0.45)
    prop.SetAmbient(0.25)
    prop.SetDiffuse(0.75)
    prop.SetSpecular(0.08)
    prop.SetSpecularPower(8)


def set_selected_cad_appearance(actor):
    prop = actor.GetProperty()
    prop.SetColor(1.0, 0.75, 0.15)  # gold/orange highlight
    prop.SetAmbient(0.35)
    prop.SetDiffuse(0.75)
    prop.SetSpecular(0.15)
    prop.SetSpecularPower(12)

def deselect_imported_cad():

    global selected_imported_filename

    selected_imported_filename = None

    importedCadList.selection_clear(0, END)

    stl_name_var.set("")
    x_var.set("")
    y_var.set("")
    z_var.set("")
    rx_var.set("")
    ry_var.set("")
    rz_var.set("")

    update_imported_cad_highlight()
    _gizmo_follow_imported(None)


def update_imported_cad_highlight():
    for filename, actor in imported_actors.items():

        if selected_imported_filename == filename:
            set_selected_cad_appearance(actor)
        else:
            set_imported_cad_appearance(actor)

    if RUN.get('vtk_running') and RUN.get('render_window') is not None:
      RUN['render_window'].Render()



def load_imported_cad_actor(filename, x=250, y=0, z=0, rx=0, ry=0, rz=0):
    file_path = os.path.join(CAD_IMPORTED_DIR, filename)

    if not os.path.exists(file_path):
        print(f"[CAD Import] Missing file: {file_path}")
        return

    reader = vtk.vtkSTLReader()
    reader.SetFileName(file_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    set_imported_cad_appearance(actor)

    actor.SetPosition(x, y, z)
    actor.SetOrientation(rx, ry, rz)

    RUN['renderer'].AddActor(actor)

    imported_actors[filename] = actor

def load_imported_cad_scene():
    if not os.path.exists(CAD_SCENE_FILE):
        return

    try:
        with open(CAD_SCENE_FILE, "r") as f:
            scene_data = json.load(f)

        for item in scene_data:
            filename = item.get("filename")

            x = item.get("x", 250)
            y = item.get("y", 0)
            z = item.get("z", 0)
            rx = item.get("rx", 0)
            ry = item.get("ry", 0)
            rz = item.get("rz", 0)

            load_imported_cad_actor(filename, x, y, z, rx, ry, rz)

        refresh_imported_cad_list()
        deselect_imported_cad()

        RUN['render_window'].Render()

    except Exception as e:
        print(f"[CAD Import] Failed to load saved CAD scene: {e}")



def save_eoat_scene():
    actor = RUN.get('tool_actor')

    if actor is None:
        data = {
            "filename": "",
            "x": 0,
            "y": 0,
            "z": 0,
            "rx": 0,
            "ry": 0,
            "rz": 0
        }
    else:
        data = {
            "filename": RUN.get('tool_filename', ""),
            "x": float(tool_x_var.get() or 0),
            "y": float(tool_y_var.get() or 0),
            "z": float(tool_z_var.get() or 0),
            "rx": float(tool_rx_var.get() or 0),
            "ry": float(tool_ry_var.get() or 0),
            "rz": float(tool_rz_var.get() or 0)
        }

    os.makedirs(CAD_EOAT_DIR, exist_ok=True)

    with open(CAD_EOAT_SCENE_FILE, "w") as f:
        json.dump(data, f, indent=4)



def load_eoat_scene():
    if not os.path.exists(CAD_EOAT_SCENE_FILE):
        return

    try:
        with open(CAD_EOAT_SCENE_FILE, "r") as f:
            data = json.load(f)

        filename = data.get("filename", "")

        if not filename:
            return

        x = data.get("x", 0)
        y = data.get("y", 0)
        z = data.get("z", 0)
        rx = data.get("rx", 0)
        ry = data.get("ry", 0)
        rz = data.get("rz", 0)

        load_eoat_actor(filename, x, y, z, rx, ry, rz)

        tool_name_var.set(filename)
        tool_x_var.set(str(x))
        tool_y_var.set(str(y))
        tool_z_var.set(str(z))
        tool_rx_var.set(str(rx))
        tool_ry_var.set(str(ry))
        tool_rz_var.set(str(rz))

        RUN['render_window'].Render()

    except Exception as e:
        print(f"[EOAT Import] Failed to load saved EOAT scene: {e}")


def load_eoat_actor(filename, x=0, y=0, z=0, rx=0, ry=0, rz=0):
    file_path = os.path.join(CAD_EOAT_DIR, filename)

    if not os.path.exists(file_path):
        print(f"[EOAT Import] Missing file: {file_path}")
        return

    reader = vtk.vtkSTLReader()
    reader.SetFileName(file_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    set_EOAT_cad_appearance(actor)

    actor.SetPosition(
    x + RUN['EOAT_DEFAULT_X_OFFSET'],
    y + RUN['EOAT_DEFAULT_Y_OFFSET'],
    z + RUN['EOAT_DEFAULT_Z_OFFSET']
    )

    actor.SetOrientation(
        rx + RUN['EOAT_DEFAULT_RX_OFFSET'],
        ry + RUN['EOAT_DEFAULT_RY_OFFSET'],
        rz + RUN['EOAT_DEFAULT_RZ_OFFSET']
    )

    assembly = RUN['assemblies']["Link 6-1.STL"]
    assembly.AddPart(actor)

    RUN['tool_actor'] = actor
    RUN['tool_filename'] = filename


def delete_selected_imported_cad():
    global selected_imported_filename

    filename = selected_imported_filename

    if not filename:
        print("[CAD Delete] No imported CAD selected.")
        return

    actor = imported_actors.get(filename)

    if actor is None:
        print(f"[CAD Delete] No actor found for: {filename}")
        return

    # Remove from world renderer
    try:
        RUN['renderer'].RemoveActor(actor)
    except Exception:
        pass

    # Remove from Link 6 assembly too, in case it was imported as tool
    try:
        RUN['assemblies']["Link 6-1.STL"].RemovePart(actor)
    except Exception:
        pass

    del imported_actors[filename]
    RUN['attached_imported_actors'].discard(filename)

    selected_imported_filename = None

    save_imported_cad_scene()
    refresh_imported_cad_list()

    if RUN.get('render_window') is not None:
        RUN['render_window'].Render()

    print(f"[CAD Delete] Removed imported CAD: {filename}")


def import_eoat_file():
    file_path = ask_open_file(
        initialdir=CAD_EOAT_DIR,
        filetypes=[
            ("3D files", "*.stl *.step *.stp"),
            ("STL files", "*.stl"),
            ("STEP files", "*.step *.stp")
        ]
    )

    if not file_path:
        return

    filename = os.path.basename(file_path)
    copied_path = os.path.join(CAD_EOAT_DIR, filename)

    if os.path.abspath(file_path) != os.path.abspath(copied_path):
        shutil.copy2(file_path, copied_path)

    file_path = copied_path
    filename = os.path.basename(file_path)

    if file_path.lower().endswith((".step", ".stp")):
        stl_path = os.path.splitext(file_path)[0] + "_converted.stl"

        try:
            import cadquery as cq
            shape = cq.importers.importStep(file_path)
            cq.exporters.export(shape, stl_path)

            file_path = stl_path
            filename = os.path.basename(stl_path)

        except Exception as e:
            messagebox.showerror("Conversion Error", f"Failed to convert STEP file:\n{e}")
            return

    remove_tool_cad()

    reader = vtk.vtkSTLReader()
    reader.SetFileName(file_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    set_EOAT_cad_appearance(actor)

    # User-visible offsets
    user_x = 0.0
    user_y = 0.0
    user_z = 0.0
    user_rx = 0.0
    user_ry = 0.0
    user_rz = 0.0

    # Actual actor transform = user offset + hidden EOAT mounting offset
    actor.SetPosition(
        user_x + RUN['EOAT_DEFAULT_X_OFFSET'],
        user_y + RUN['EOAT_DEFAULT_Y_OFFSET'],
        user_z + RUN['EOAT_DEFAULT_Z_OFFSET']
    )

    actor.SetOrientation(
        user_rx + RUN['EOAT_DEFAULT_RX_OFFSET'],
        user_ry + RUN['EOAT_DEFAULT_RY_OFFSET'],
        user_rz + RUN['EOAT_DEFAULT_RZ_OFFSET']
    )

    assembly = RUN['assemblies']["Link 6-1.STL"]
    assembly.AddPart(actor)

    RUN['tool_actor'] = actor
    RUN['tool_filename'] = filename

    # GUI always shows user offsets only, not hidden offsets
    tool_name_var.set(filename)
    tool_x_var.set(str(user_x))
    tool_y_var.set(str(user_y))
    tool_z_var.set(str(user_z))
    tool_rx_var.set(str(user_rx))
    tool_ry_var.set(str(user_ry))
    tool_rz_var.set(str(user_rz))

    if RUN.get('render_window') is not None:
        RUN['render_window'].Render()

    save_eoat_scene()






def import_3d_file(astool):
    file_path = ask_open_file(
        initialdir=CAD_IMPORTED_DIR,
        filetypes=[
            ("3D files", "*.stl *.step *.stp"),
            ("STL files", "*.stl"),
            ("STEP files", "*.step *.stp")
        ]
    )

    if not file_path:
        return

    original_filename = os.path.basename(file_path)
    copied_path = os.path.join(CAD_IMPORTED_DIR, original_filename)

    # Copy selected CAD into ARrobots/CAD Imported
    if os.path.abspath(file_path) != os.path.abspath(copied_path):
        shutil.copy2(file_path, copied_path)

    file_path = copied_path
    filename = os.path.basename(file_path)

    # Convert STEP/STP to STL inside CAD Imported folder
    if file_path.lower().endswith((".step", ".stp")):
        stl_path = os.path.splitext(file_path)[0] + "_converted.stl"

        try:
            import cadquery as cq
            shape = cq.importers.importStep(file_path)
            cq.exporters.export(shape, stl_path)

            file_path = stl_path
            filename = os.path.basename(stl_path)

        except Exception as e:
            messagebox.showerror("Conversion Error", f"Failed to convert STEP file:\n{e}")
            return

    reader = vtk.vtkSTLReader()
    reader.SetFileName(file_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()


    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    prop = actor.GetProperty()
    prop.SetColor(0.70, 0.70, 0.70)
    prop.SetAmbient(0.25)
    prop.SetDiffuse(0.75)
    prop.SetSpecular(0.08)
    prop.SetSpecularPower(8)

    actor.SetPosition(250, 0, 0)
    actor.SetOrientation(0, 0, 0)

    if astool:
        assembly = RUN['assemblies']["Link 6-1.STL"]
        assembly.AddPart(actor)
    else:
        RUN['renderer'].AddActor(actor)

    RUN['render_window'].Render()
    imported_actors[filename] = actor

    save_imported_cad_scene()

    refresh_imported_cad_list()
    select_imported_cad(filename)

    if RUN.get('render_window') is not None:
        RUN['render_window'].Render()


def get_world_matrix(stl_key):
    chain = [
        "Link Base-1.STL", "Link Base-2.STL", "Link Base-3.STL",
        "Link 1-1.STL", "Link 1-2.STL",
        "Link 2-1.STL", "Link 2-2.STL", "Link 2-3.STL",
        "Link 3-1.STL", "Link 3-2.STL",
        "Link 4-1.STL", "Link 4-2.STL", "Link 4-3.STL",
        "Link 5-1.STL", "Link 5-2.STL",
        "Link 6-1.STL", "Link 6-2.STL",
    ]
    world = vtk.vtkMatrix4x4()
    world.Identity()
    for key in chain:
        local = RUN['assemblies'][key].GetMatrix()
        temp = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(world, local, temp)
        world.DeepCopy(temp)
        if key == stl_key:
            break
    return world


def attachtogripper(actorname):
    # VTK changes and rendering must run on the Tkinter/main thread
    if threading.current_thread() is not threading.main_thread():
        root.after(0, lambda name=actorname: attachtogripper(name))
        return
    # Allow callers to omit the extension; matches .stl/.STL either way (Linux is case-sensitive)
    actorname = resolve_imported_name(actorname)

    if actorname not in imported_actors:
        logger.error(f"attachtogripper: '{actorname}' not found in imported_actors")
        return

    actor = imported_actors[actorname]

    actor_world = vtk.vtkMatrix4x4()
    actor_world.DeepCopy(actor.GetMatrix())

    gripper_world = get_world_matrix("Link 6-1.STL")
    inv_gripper = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Invert(gripper_world, inv_gripper)

    local = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Multiply4x4(inv_gripper, actor_world, local)

    actor.SetPosition(0, 0, 0)
    actor.SetOrientation(0, 0, 0)
    actor.SetScale(1, 1, 1)
    actor.SetOrigin(0, 0, 0)
    actor.SetUserMatrix(local)

    RUN['renderer'].RemoveActor(actor)
    RUN['assemblies']["Link 6-1.STL"].AddPart(actor)
    RUN['attached_imported_actors'].add(actorname)
    RUN['render_window'].Render()


def detachfromgripper(actorname):
    # VTK changes and rendering must run on the Tkinter/main thread
    if threading.current_thread() is not threading.main_thread():
        root.after(0, lambda name=actorname: detachfromgripper(name))
        return
    # Allow callers to omit the extension; matches .stl/.STL either way (Linux is case-sensitive)
    actorname = resolve_imported_name(actorname)

    if actorname not in imported_actors:
        logger.error(
            f"detachfromgripper: '{actorname}' not found in imported_actors"
        )
        return

    actor = imported_actors[actorname]
    gripper_assembly = RUN['assemblies']["Link 6-1.STL"]

    # --------------------------------------------------------
    # Calculate the actor's true WORLD matrix before detaching.
    #
    # actor.GetMatrix() is local to the gripper while attached.
    # Therefore:
    #
    # world = gripper_world * actor_local
    # --------------------------------------------------------
    actor_local = vtk.vtkMatrix4x4()
    actor_local.DeepCopy(actor.GetMatrix())

    gripper_world = get_world_matrix("Link 6-1.STL")

    actor_world = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Multiply4x4(
        gripper_world,
        actor_local,
        actor_world
    )

    # Remove from the robot/gripper hierarchy
    gripper_assembly.RemovePart(actor)

    # Clear the gripper-relative transform
    actor.SetUserMatrix(None)
    actor.SetUserTransform(None)

    actor.SetOrigin(0, 0, 0)
    actor.SetPosition(0, 0, 0)
    actor.SetOrientation(0, 0, 0)
    actor.SetScale(1, 1, 1)

    # Decompose the true world matrix into normal actor values
    world_transform = vtk.vtkTransform()
    world_transform.SetMatrix(actor_world)

    position = world_transform.GetPosition()
    orientation = world_transform.GetOrientation()
    scale = world_transform.GetScale()

    actor.SetPosition(
        position[0],
        position[1],
        position[2]
    )

    actor.SetOrientation(
        orientation[0],
        orientation[1],
        orientation[2]
    )

    actor.SetScale(
        scale[0],
        scale[1],
        scale[2]
    )

    # Put the actor back into the main scene
    RUN['renderer'].AddActor(actor)

    # Update the Virtual Import position fields
    stl_name_var.set(actorname)

    x_var.set(f"{position[0]:.3f}")
    y_var.set(f"{position[1]:.3f}")
    z_var.set(f"{position[2]:.3f}")

    rx_var.set(f"{orientation[0]:.3f}")
    ry_var.set(f"{orientation[1]:.3f}")
    rz_var.set(f"{orientation[2]:.3f}")

    RUN['attached_imported_actors'].discard(actorname)

    # Save the detached world position
    save_imported_cad_scene()

    RUN['render_window'].Render()

    logger.info(
        f"Detached '{actorname}' at world position: "
        f"X={position[0]:.3f}, "
        f"Y={position[1]:.3f}, "
        f"Z={position[2]:.3f}, "
        f"Rx={orientation[0]:.3f}, "
        f"Ry={orientation[1]:.3f}, "
        f"Rz={orientation[2]:.3f}"
    )


def detach_all_imported_cad_on_close():
    attached_names = list(
        RUN.get('attached_imported_actors', set())
    )

    for actorname in attached_names:
        try:
            detachfromgripper(actorname)
        except Exception as e:
            logger.error(
                f"Failed to detach '{actorname}' during shutdown: {e}"
            )    


def load_stl_into_scene(stl_path, renderer, render_window):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(stl_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.254, 0.41, 0.882)  

    actor.SetPosition(0, 0, 0)

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
    ui.set_window_icon(win, default=False)
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
    try:
      port = CAL['comPort']
    except Exception:
      port = ""
    progress = ui.busy(root, f"Connecting to Teensy on {port}" if port else "Connecting to Teensy")
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

    # close progress (success or timeout); the launch splash closes itself
    progress.close()

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
  time.sleep(.1)
  updateVisOp()




###############################################################################################################################################################
### COMMUNICATION DEFS ################################################################################################################# COMMUNICATION DEFS ###
###############################################################################################################################################################

###############################################################################################################################################################
# Change of field to support automatic comm detection and drop down
# Added exception output to log window

def sendHelloToTeensy(checkMismatch=True):
  """Send hello command to Teensy, log response, check for model mismatch or unconfigured board."""
  try:
    response = RUN['robot'].command({"cmd": "hello"})

    # The Teensy may emit boot/debug lines before the JSON hello (e.g. right
    # after a reboot). Skip non-JSON lines and keep reading until we find the
    # JSON response, so stray lines can't desync the serial protocol. The port
    # uses timeout=None (blocking), so apply a short timeout while we scan.
    hello = None
    deadline = time.time() + 3
    ser = RUN['robot'].serial_connection
    old_timeout = ser.timeout
    try:
      ser.timeout = 0.3
      while True:
        try:
          hello = json.loads(response)
          break
        except (json.JSONDecodeError, TypeError):
          logger.debug(f"Skipping non-JSON line while waiting for hello: {response}")
        if time.time() > deadline:
          logger.warning(f"Could not get JSON hello response; last line: {response}")
          return
        try:
          line = ser.readline()
        except Exception:
          logger.warning("Serial error while waiting for hello response")
          return
        if not line:
          continue
        response = line.decode(errors="replace").strip()
        if not response:
          continue
    finally:
      ser.timeout = old_timeout

    logger.info(f"Teensy hello: {response}")
    RUN['fwVersion'] = str(hello.get("FirmwareVersion", ""))

    # Check if board is unconfigured
    configured = hello.get("Configured", False)
    if not configured:
      messagebox.showwarning(
        "Unconfigured Board",
        "The Teensy board has no robot model configured in EEPROM.\n\n"
        "Please select a robot model from the Defaults panel and click Save to configure the board."
      )
      return

    # Track the Teensy's reported model so the HMI can detect model changes
    eepromModel = hello.get("RobotModel", "")
    eepromVersion = hello.get("RobotVersion", "")
    if eepromVersion:
      eepromFull = eepromModel + "-" + eepromVersion
    else:
      eepromFull = eepromModel
    RUN['teensyModel'] = eepromFull

    # Check for model mismatch between HMI combo box and EEPROM
    if checkMismatch:
      hmiModel = modelSelectVar.get()

      if eepromFull != hmiModel:
        result = messagebox.askyesno(
          "Model Mismatch",
          f"The Teensy EEPROM reports: {eepromFull}\n"
          f"The HMI is set to: {hmiModel}\n\n"
          "Would you like to update the HMI selection to match the Teensy?"
        )
        if result:
          if eepromFull in VALID_MODELS:
            modelSelectVar.set(eepromFull)
          else:
            messagebox.showwarning(
              "Unknown Model",
              f"The Teensy reports model '{eepromFull}' which is not a recognized model.\n"
              "Please select the correct model and save to update the Teensy."
            )

  except Exception:
    logger.exception("Unable to get hello response from Teensy")

def setCom(misc=None):
  CAL['comPort'] = com1SelectedValue.get()

  try:
    if 'robot' in RUN and RUN['robot']:
      RUN['robot'].disconnect()
      time.sleep(0.2)
  except Exception:
    pass

  try:
    if CAL['comPort'] in (None, "", "None"):
      raise ValueError("No COM port selected")

    RUN['robot'] = ARRobot(port=CAL['comPort'])
    RUN['robot'].connect()

    logger.info(
      "COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER on Port %s",
      CAL['comPort']
    )

    almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

    time.sleep(.1)

    sendHelloToTeensy()

    startup_with_spinner(root)

    value = tab8.ElogView.get(0, END)
    pickle.dump(value, open("ErrorLog", "wb"))

  except Exception:

    try:
      if 'robot' in RUN and RUN['robot']:
        RUN['robot'].disconnect()
        time.sleep(0.2)
    except Exception:
      pass

    logger.exception(
      "UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER"
    )

    almStatusLab.config(
      text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER",
      style="Alarm.TLabel"
    )

    almStatusLab2.config(
      text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER",
      style="Alarm.TLabel"
    )

    try:
      value = tab8.ElogView.get(0, END)
      pickle.dump(value, open("ErrorLog", "wb"))
    except Exception:
      pass


def setCom2(misc=None):
  CAL['com2Port'] = com2SelectedValue.get()

  try:
    if 'nano' in RUN and RUN['nano']:
      RUN['nano'].disconnect()
      time.sleep(0.2)
  except Exception:
    pass

  try:
    if CAL['com2Port'] in (None, "", "None"):
      raise ValueError("No COM port selected")

    RUN['nano'] = ARNano(port=CAL['com2Port'])
    RUN['nano'].connect()

    logger.info(
      "COMMUNICATIONS STARTED WITH ARDUINO IO BOARD on Port %s",
      CAL['com2Port']
    )

    value = tab8.ElogView.get(0, END)
    pickle.dump(value, open("ErrorLog", "wb"))

  except Exception:
    logger.exception("UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD")

    try:
      value = tab8.ElogView.get(0, END)
      pickle.dump(value, open("ErrorLog", "wb"))
    except Exception:
      pass
  try:
    response = RUN['nano'].command({"cmd": "hello"})
    print(response)
  except Exception:
    logger.exception("Unable to communicate with Arduino IO board")
    
def darkTheme():
  CAL['curTheme'] = 0
  ui.set_theme(root, "dark")      # graphite


def lightTheme():
  CAL['curTheme'] = 1
  ui.set_theme(root, "light")     # aluminium



###############################################################################################################################################################  
### EXECUTION DEFS ######################################################################################################################### EXECUTION DEFS ###  
############################################################################################################################################################### 

##############################################################################
### MOTION STREAMING (8.1) ###################################################
##############################################################################
# Consecutive motion rows are sent into the firmware's motion queue (spline mode)
# instead of one-at-a-time. Each queued move is acknowledged on receipt, so the
# robot never waits on the PC between moves, and Move L rows with Rnd > 0 blend
# into the next Move L. Any other row (gripper, wait, IO, If/Jump/Call, registers,
# vision...) is a barrier: the queue is closed with spline_end, which only returns
# once the robot has actually stopped at the last point, and then the row runs.
#
# With the virtual robot open, a streamed run does not animate moves itself: the firmware
# sends live joint angles ({"msg":"tele"}, ~20/s) and the virtual robot mirrors the real
# arm, so it shows the true path including rounded corners. Offline, the virtual robot
# still simulates its own moves exactly as before.

STREAM_MOTION_TYPES = ("Move J", "OFF J ", "Move L", "Move R", "Move P", "OFF PR", "Move A", "Move C")


def _fw_supports_streaming():
  try:
    parts = [int(x) for x in str(RUN.get('fwVersion', '')).split('.')[:2]]
    while len(parts) < 2:
      parts.append(0)
    return tuple(parts) >= (8, 0)
  except ValueError:
    return False


def stream_available():
  return (
    RUN['autoBlend']
    and not RUN['offlineMode']
    and RUN.get('robot') is not None
    and RUN['robot'].is_connected()
    and _fw_supports_streaming()
  )


def _selected_row_text():
  try:
    sel = tab1.progView.curselection()[0]
    return tab1.progView.get(sel).decode().strip()
  except Exception:
    return ""


def _is_passthrough_row(text):
  # Rows that neither move the robot nor need it stopped
  return (
    text == ""
    or text.startswith("#")
    or text.startswith("Tab Number")
    or text.startswith("Start Spline")
  )


def _on_robot_telemetry(angles):
  """Live joint angles from the firmware while streaming (reader thread). The VTK loop
  redraws from RUN['VR_angles'] every tick, so assigning a new list is all it needs."""
  if RUN['vtk_running']:
    RUN['VR_angles'] = angles


def _sync_virtual_to_robot():
  """Put the virtual robot exactly on the real robot's last reported position."""
  if not RUN['vtk_running']:
    return
  try:
    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']),
                        float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
  except Exception as e:
    logger.debug(f"Virtual robot resync failed: {e}")


def stream_begin():
  try:
    response = RUN['robot'].command('{"cmd":"spline_start"}')
  except Exception as e:
    logger.warning(f"spline_start failed, running without blending: {e}")
    return
  if str(response).strip() == "SL":
    RUN['streamActive'] = True
    RUN['splineActive'] = "1"
    RUN['robot'].telemetry_callback = _on_robot_telemetry
  else:
    logger.warning(f"Unexpected spline_start reply, running without blending: {response}")


def stream_end():
  """Close the motion queue and wait until the robot has stopped at the last point."""
  if not RUN['streamActive']:
    return
  RUN['streamActive'] = False
  RUN['splineActive'] = "0"
  try:
    response = RUN['robot'].command('{"cmd":"spline_end"}')
  except Exception as e:
    logger.warning(f"spline_end failed: {e}")
    return
  finally:
    RUN['robot'].telemetry_callback = None
  if is_error_response(response):
    ErrorHandler(response)
  else:
    displayPosition(response)
  _sync_virtual_to_robot()


def stream_prepare_row():
  """Open or close the motion queue for the row that is about to execute."""
  text = _selected_row_text()
  if text.startswith(STREAM_MOTION_TYPES):
    if not RUN['streamActive'] and stream_available():
      stream_begin()
  elif _is_passthrough_row(text):
    pass
  else:
    stream_end()


def robot_move(command):
  """Send a motion command: queued while streaming, otherwise request/response."""
  if RUN['streamActive']:
    return RUN['robot'].queue_move(command)
  return RUN['robot'].command(command)


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

    last = tab1.progView.index('end')
    for row in range (0,last):
      tab1.progView.itemconfig(row, {'fg': ui.C['fg']})

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
      try:
        selRow = tab1.progView.curselection()[0] 
      except:
        if(tab1.lastProg == ""):
          selRow = 1
          progLoop = ("## START PROGRAM LOOP ##\r\n").encode('utf-8')
          try:
            index = find_prog_row(progLoop)
            tab1.progView.selection_clear(0, END)
            tab1.progView.select_set(index)
          except:
            stopProg() 
        else:
          lastRow = tab1.lastRow + 1
          lastProg = tab1.lastProg
          ProgEntryField.delete(0, 'end')
          ProgEntryField.insert(0,lastProg)
          callProg(lastProg)
          time.sleep(.4) 
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(lastRow) 
          curRowEntryField.delete(0, 'end')
          curRowEntryField.insert(0,lastRow)
          tab1.lastProg = ""
      if(tab1.runTrue == 1):
        stream_prepare_row()
        executeRow()
      
        while RUN['rowinproc'] == 1:
          time.sleep(.1)
        #last = tab1.progView.index('end')
        #for row in range (0,selRow):
          #tab1.progView.itemconfig(row, {'fg': 'dodger blue'})
        #tab1.progView.itemconfig(selRow, {'fg': 'blue2'})
        #for row in range (selRow+1,last):
          #tab1.progView.itemconfig(row, {'fg': 'black'})
        
        try:
          selRow = tab1.progView.curselection()[0]
          selRow += 1
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(selRow)
          curRowEntryField.delete(0, 'end')
          curRowEntryField.insert(0,selRow)
        except:
          curRow=1
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(curRow)
        time.sleep(.01 if RUN['streamActive'] else .1)

        if (RUN['estopActive']):
          almStatusLab.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
          almStatusLab2.config(text="Estop Button was Pressed",  style="Alarm.TLabel")
        elif (RUN['posOutreach']):
          almStatusLab.config(text="Position Out of Reach",  style="Alarm.TLabel")
          almStatusLab2.config(text="Position Out of Reach",  style="Alarm.TLabel")    
        else:
          almStatusLab.config(text="PROGRAM STOPPED",  style="Alarm.TLabel")
          almStatusLab2.config(text="PROGRAM STOPPED",  style="Alarm.TLabel") 

  def threadProgSafe():
    try:
      threadProg()
    finally:
      # Always close the motion queue (also after Stop or an error)
      stream_end()

  t = threading.Thread(target=threadProgSafe)
  t.start()
  
def stepFwd():
    def threadProg():
      #global estopActive
      RUN['estopActive'] = False
      #global posOutreach
      RUN['posOutreach'] = False
      try:
        selRow = tab1.progView.curselection()[0]
      except:
        if(tab1.lastProg == ""):
          selRow = 1
          progLoop = ("## START PROGRAM LOOP ##\r\n").encode('utf-8')
          try:
            index = find_prog_row(progLoop)
            tab1.progView.selection_clear(0, END)
            tab1.progView.select_set(index)
          except:
            stopProg()  
        else:
          lastRow = tab1.lastRow + 1
          lastProg = tab1.lastProg
          ProgEntryField.delete(0, 'end')
          ProgEntryField.insert(0,lastProg)
          callProg(lastProg)
          time.sleep(.4) 
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(lastRow) 
          curRowEntryField.delete(0, 'end')
          curRowEntryField.insert(0,lastRow)
          tab1.lastProg = ""
      executeRow() 
      try:
        last = tab1.progView.index('end')
        selRow = tab1.progView.curselection()[0]
        for row in range (0,selRow):
          tab1.progView.itemconfig(row, {'fg': ui.C['info']})
        tab1.progView.itemconfig(selRow, {'fg': ui.C['accent']})
        for row in range (selRow+1,last):
          tab1.progView.itemconfig(row, {'fg': ui.C['muted']})
        try:
          selRow = tab1.progView.curselection()[0]
          selRow += 1
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(selRow)
          curRowEntryField.delete(0, 'end')
          curRowEntryField.insert(0,selRow)
        except:
          curRow=1
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(curRow)
        time.sleep(.1)
      except Exception:
        pass  
    t = threading.Thread(target=threadProg)
    t.start()

def stepRev():
    #global estopActive
    RUN['estopActive'] = False
    #global posOutreach
    RUN['posOutreach'] = False
    try:
      selRow = tab1.progView.curselection()[0]
    except:
      selRow = 1
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(selRow) 
    executeRow()  
    last = tab1.progView.index('end')
    for row in range (0,selRow):
      tab1.progView.itemconfig(row, {'fg': ui.C['muted']})
    tab1.progView.itemconfig(selRow, {'fg': ui.C['danger']})
    for row in range (selRow+1,last):
      tab1.progView.itemconfig(row, {'fg': ui.C['danger']})
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
  # Halt the robot and drop queued moves. Sent without the command lock so it
  # interrupts the program thread; the firmware replies once to whoever is waiting.
  if RUN['streamActive']:
    try:
      RUN['robot'].stop()
    except Exception as e:
      logger.warning(f"Robot stop failed: {e}")

  try:
    command = '{"cmd":"stop"}\n'

    #cmdSentEntryField.delete(0, 'end')
    #cmdSentEntryField.insert(0,command)

    response = RUN['nano'].command(command)

    #cmdRecEntryField.delete(0, 'end')
    #cmdRecEntryField.insert(0,response)

  except:
    print("Error IO Board Not Connected")

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
  RUN['progRunning'] = True
  try:
    selRow = tab1.progView.curselection()[0]
    tab1.progView.see(selRow+2)
  except Exception:
    pass

  try: 
    data = list(map(int, tab1.progView.curselection()))
    command=tab1.progView.get(data[0]).decode().strip()
    RUN['cmdType'] =command[:6]
    RUN['cmdTypeLong']=command[:11]
  except:
    RUN['cmdType'] = "Stop P"
    RUN['cmdTypeLong'] = "Stop P"

  ##Call Program##
  if (RUN['cmdType'] == "Call P"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    tab1.lastRow = tab1.progView.curselection()[0]
    tab1.lastProg = ProgEntryField.get()
    programIndex = command.find("Program -")
    progName = str(command[programIndex+10:])
    ProgEntryField.delete(0, 'end')
    ProgEntryField.insert(0,progName)
    callProg(progName)
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

  ##Stop Program##
  if (RUN['cmdType'] == "Stop P"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    stopProg()


 ##Test Limit Switches
  if (RUN['cmdType'] == "Test L"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Test limit switches not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = '{"cmd":"test_limit_switches"}\n'
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    response = RUN['robot'].command(command)
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)


  ##Test Gripper Amperage
  if (RUN['cmdType'] == "Test G"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Test gripper amperage not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = '{"cmd":"test_gripper_amps"}\n'
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    response = RUN['nano'].command(command)
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)



  ##Set Encoders 1000
  if (RUN['cmdType'] == "Set En"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Encoder testing not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = '{"cmd":"set_encoders"}\n'
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    response = RUN['robot'].command(command)

  ##Read Encoders
  if (RUN['cmdType'] == "Read E"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Read Encoders not supported in offline programming mode", style="Alarm.TLabel")
      return
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = '{"cmd":"read_encoders"}\n'
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    response = RUN['robot'].command(command)
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
    command = '{"cmd":"servo","num":' + servoNum + ',"pos":' + servoPos + '}\n'
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    response = RUN['nano'].command(command)

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

    ## query board for IO value
    cmd = '{"cmd":"input_read","pin":' + inputNum + '}\n'

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, cmd)

    response = RUN['nano'].command(cmd)

    cmdRecEntryField.delete(0, 'end')
    cmdRecEntryField.insert(0, str(response))

    if (response == "T"):
      querry = 1
    elif(response == "F"):
      querry = 0
    else:
      almStatusLab.config(text=f"Unexpected IO response: {response}", style="Alarm.TLabel")
      return

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
        index = find_prog_row(tabNum)
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
      port = resolve_serial_port(comNum)
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
        index = find_prog_row(tabNum)
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
        index = find_prog_row(tabNum)
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
    subcommand = (
      f'{{"cmd":"modbus_read_coil",'
      f'"slave_id":{slaveID},'
      f'"address":{inputNum},'
      f'"count":1'
      f'}}\n'
    )
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, subcommand)
    response = RUN['robot'].command(subcommand) 
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
        index = find_prog_row(tabNum)
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
    subcommand = (
      f'{{"cmd":"modbus_read_discrete_input",'
      f'"slave_id":{slaveID},'
      f'"address":{inputNum},'
      f'"count":{opVal}'
      f'}}\n'
    )
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, subcommand)
    response = RUN['robot'].command(subcommand)
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
        index = find_prog_row(tabNum)
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
    subcommand = (
      f'{{"cmd":"modbus_read_holding_register",'
      f'"slave_id":{slaveID},'
      f'"address":{inputNum},'
      f'"count":{opVal}'
      f'}}\n'
    )
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, subcommand)
    response = RUN['robot'].command(subcommand)
    if (response == "Modbus Error"):
      almStatusLab.config(text=response, style="Alarm.TLabel")
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
        index = find_prog_row(tabNum)
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
    subcommand = (
      f'{{"cmd":"modbus_read_input_register",'
      f'"slave_id":{slaveID},'
      f'"address":{inputNum},'
      f'"count":{opVal}'
      f'}}\n'
    )
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, subcommand)
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, subcommand)
    response = RUN['robot'].command(subcommand)
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
        index = find_prog_row(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()

  ##Wait 5v IO board##
  if (RUN['cmdTypeLong'] == "Wait 5v Inp"):
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

    command = '{"cmd":"wait_input","pin":' + inputNum + ',"state":' + valNum + ',"timeout":' + timeout + '}\n'

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['nano'].command(command)
 

  ##Wait Modbus Coil##
  if (RUN['cmdTypeLong'] == "Wait MBcoil"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return

    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2

    slavestartIndex = command.find("SlaveID (")
    slaveendIndex = command.find(")", slavestartIndex)
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    timeoutIndex = command.find("Timeout =")

    slaveID = str(command[slavestartIndex+9:slaveendIndex]).strip()
    inputNum = str(command[inputIndex+2:valIndex-1]).strip()
    valNum = str(command[valIndex+2:timeoutIndex-3]).strip()
    timeout = str(command[timeoutIndex+10:]).strip()

    command = (
      f'{{"cmd":"wait_modbus_coil",'
      f'"slave_id":{slaveID},'
      f'"address":{inputNum},'
      f'"expected":{valNum},'
      f'"timeout":{timeout}'
      f'}}\n'
    )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)

    response = RUN['robot'].command(command)

    if (response == "-1"):
      ErrorHandler("Modbus Error")

  ##Wait Modbus Input##
  if (RUN['cmdTypeLong'] == "Wait MBhold"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return

    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2

    slavestartIndex = command.find("SlaveID (")
    slaveendIndex = command.find(")", slavestartIndex)
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    timeoutIndex = command.find("Timeout =")

    slaveID = str(command[slavestartIndex+9:slaveendIndex]).strip()
    inputNum = str(command[inputIndex+2:valIndex-1]).strip()
    valNum = str(command[valIndex+2:timeoutIndex-3]).strip()
    timeout = str(command[timeoutIndex+10:]).strip()

    command = (
      f'{{"cmd":"wait_modbus_holding_register",'
      f'"slave_id":{slaveID},'
      f'"address":{inputNum},'
      f'"expected":{valNum},'
      f'"timeout":{timeout}'
      f'}}\n'
  )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['robot'].command(command)

    if (response == "Modbus Error"):
      ErrorHandler("Modbus Error")
    elif (response == "Timeout"):
      ErrorHandler("Modbus Timeout")
   
              

  ### this will fail on first run without reloading program - the insertion inserts it as bytes (due to pickling the program save) but when reloaded its a listbox of strings - therefore this would only work after reload and looking for string
  ## long term fix refactor all 
  ''' 
  ##Jump to Row##
    if (RUN['moveInProc'] == 1):
        RUN['moveInProc'] = 2
    tabIndex = command.find("Tab-")
    tabNum = ("Tab Number " + str(command[tabIndex+4:]) + "\r\n").encode('utf-8')
    index = find_prog_row(tabNum)
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
  if (RUN['cmdTypeLong'] == "Set 5v Outp"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return

    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2

    outputIndex = command.find("# ")
    valueIndex = command.find("= ")

    output = str(command[outputIndex+2:valueIndex-1])
    value = str(command[valueIndex+2:])

    command = '{"cmd":"set_output","pin":' + output + ',"state":' + value + '}\n'

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['nano'].command(command)



  ##Set Modbus Coil##
  if (RUN['cmdTypeLong'] == "Set MBcoil "):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return

    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2

    slavestartIndex = command.find("SlaveID (")
    slaveendIndex = command.find(")", slavestartIndex)
    inputIndex = command.find("# ")
    valIndex = command.find("= ")

    slaveID = str(command[slavestartIndex+9:slaveendIndex]).strip()
    inputNum = str(command[inputIndex+2:valIndex-1]).strip()
    valNum = str(command[valIndex+2:]).strip()

    command = (
      f'{{"cmd":"modbus_write_coil",'
      f'"slave_id":{slaveID},'
      f'"address":{inputNum},'
      f'"value":{valNum}'
      f'}}\n'
    )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)

    response = RUN['robot'].command(command)

    if (response == "-1"):
      ErrorHandler("Modbus Error")




  ##Set Modbus Register##
  if (RUN['cmdTypeLong'] == "Set MBoutpu"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      return

    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2

    slavestartIndex = command.find("SlaveID (")
    slaveendIndex = command.find(")", slavestartIndex)
    inputIndex = command.find("# ")
    valIndex = command.find("= ")

    slaveID = str(command[slavestartIndex+9:slaveendIndex]).strip()
    inputNum = str(command[inputIndex+2:valIndex-1]).strip()
    valNum = str(command[valIndex+2:]).strip()

    command = (
      f'{{"cmd":"modbus_write_register",'
      f'"slave_id":{slaveID},'
      f'"address":{inputNum},'
      f'"value":{valNum}'
      f'}}\n'
    )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['robot'].command(command)

    if (response == "Modbus Error"):
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
    command = (
      f'{{"cmd":"wait_time",'
      f'"seconds":{timeSeconds}'
      f'}}\n'
    )
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    response = RUN['robot'].command(command)


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

  # ### Set Tool no Longer used
  # ##Set tool##  
  # if (RUN['cmdType'] == "Tool S"):
  #   if RUN['offlineMode']:
  #     almStatusLab.config(text="Set tool not supported in offline programming mode", style="Alarm.TLabel")
  #     return 
  #   if (RUN['moveInProc'] == 1):
  #     RUN['moveInProc'] = 2
  #   almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
  #   almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel") 
  #   xIndex = command.find(" X ")
  #   yIndex = command.find(" Y ")
  #   zIndex = command.find(" Z ")
  #   rzIndex = command.find(" Rz ")
  #   ryIndex = command.find(" Ry ")
  #   rxIndex = command.find(" Rx ")
  #   RUN['xVal'] = command[xIndex+3:yIndex]
  #   RUN['yVal'] = command[yIndex+3:zIndex]
  #   RUN['zVal'] = command[zIndex+3:rzIndex]
  #   rzVal = command[rzIndex+4:ryIndex]
  #   ryVal = command[ryIndex+4:rxIndex]
  #   rxVal = command[rxIndex+4:]
  #   TFxEntryField.delete(0,'end')
  #   TFyEntryField.delete(0,'end')
  #   TFzEntryField.delete(0,'end')
  #   TFrzEntryField.delete(0,'end')
  #   TFryEntryField.delete(0,'end')
  #   TFrxEntryField.delete(0,'end')
  #   TFxEntryField.insert(0,str(RUN['xVal']))
  #   TFyEntryField.insert(0,str(RUN['yVal']))
  #   TFzEntryField.insert(0,str(RUN['zVal']))
  #   TFrzEntryField.insert(0,str(rzVal))
  #   TFryEntryField.insert(0,str(ryVal))
  #   TFrxEntryField.insert(0,str(rxVal))
  #   command = "TF"+"A"+RUN['xVal']+"B"+RUN['yVal']+"C"+RUN['zVal']+"D"+rzVal+"E"+ryVal+"F"+rxVal+"\n"
  #   cmdSentEntryField.delete(0, 'end')
  #   cmdSentEntryField.insert(0,command)
  #   RUN['ser'].write(command.encode())
  #   RUN['ser'].flushInput()
  #   time.sleep(.1)
  #   RUN['ser'].write(command.encode())
  #   RUN['ser'].flushInput()
  #   time.sleep(.1)
  #   RUN['ser'].read()
    
  
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
    command = (
      f'{{"cmd":"move_j",'
      f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal},{J7Val},{J8Val},{J9Val}],'
      f'"spd":{Speed},'
      f'"spd_type":"{map_spd_type(speedPrefix)}",'
      f'"acc":{ACCspd},'
      f'"dec":{DECspd},'
      f'"ramp":{ACCramp},'
      f'"w":"{RUN["WC"]}",'
      f'"loop_mode":"{LoopMode}"'
      f'}}\n'
    )
    if RUN['streamActive'] or not RUN['vtk_running']:   # streamed runs: virtual robot mirrors the real one
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      response = robot_move(command)
      if is_error_response(response):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      mj_command(command)
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
    command = (
      f'{{"cmd":"move_j",'
      f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal},{J7Val},{J8Val},{J9Val}],'
      f'"spd":{Speed},'
      f'"spd_type":"{map_spd_type(speedPrefix)}",'
      f'"acc":{ACCspd},'
      f'"dec":{DECspd},'
      f'"ramp":{ACCramp},'
      f'"w":"{RUN["WC"]}",'
      f'"loop_mode":"{LoopMode}"'
      f'}}\n'
    )
    if RUN['streamActive'] or not RUN['vtk_running']:   # streamed runs: virtual robot mirrors the real one
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      response = robot_move(command)
      if is_error_response(response):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      mj_command(command)
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
    command = (
      '{"cmd":"move_vis",'
      f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal},{J7Val},{J8Val},{J9Val}],'
      f'"spd_type":"{speedPrefix.strip()}",'
      f'"spd":{Speed},'
      f'"acc":{ACCspd},'
      f'"dec":{DECspd},'
      f'"ramp":{ACCramp},'
      f'"w":"{RUN["WC"].strip()}",'
      f'"vis_rot":{visRot},'
      f'"loop_mode":"{LoopMode}"'
      '}\n'
    )
    if not RUN['vtk_running']:
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      response = RUN['robot'].command(command)
      if is_error_response(response):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      mj_command(command)
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
    command = (
      f'{{"cmd":"move_j",'
      f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal},{J7Val},{J8Val},{J9Val}],'
      f'"spd":{Speed},'
      f'"spd_type":"{map_spd_type(speedPrefix)}",'
      f'"acc":{ACCspd},'
      f'"dec":{DECspd},'
      f'"ramp":{ACCramp},'
      f'"w":"{RUN["WC"]}",'
      f'"loop_mode":"{LoopMode}"'
      f'}}\n'
    )
    if RUN['streamActive'] or not RUN['vtk_running']:   # streamed runs: virtual robot mirrors the real one
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      response = robot_move(command)
      if is_error_response(response):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      mj_command(command)
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
    command = (
      f'{{"cmd":"move_j",'
      f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal},{J7Val},{J8Val},{J9Val}],'
      f'"spd":{Speed},'
      f'"spd_type":"{map_spd_type(speedPrefix)}",'
      f'"acc":{ACCspd},'
      f'"dec":{DECspd},'
      f'"ramp":{ACCramp},'
      f'"w":"{RUN["WC"]}",'
      f'"loop_mode":"{LoopMode}"'
      f'}}\n'
    )
    if RUN['streamActive'] or not RUN['vtk_running']:   # streamed runs: virtual robot mirrors the real one
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      response = robot_move(command)
      if is_error_response(response):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      mj_command(command)
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
    #if (np.sign(float(rzVal)) != np.sign(float(CAL['RzcurPos']))):
    #  rzVal=str(float(rzVal)*-1)
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
    command = (
    # Orientation order must match move_j and the robot_pos reply: [X, Y, Z, Rz, Ry, Rx]
    # (previously Rx/Rz were swapped here, which only went unnoticed when Rz ~= Rx)
    '{"cmd":"move_l",'
      f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal}],'
      f'"ext":[{J7Val},{J8Val},{J9Val}],'
      f'"spd":{Speed},'
      f'"spd_type":"{map_spd_type(speedPrefix)}",'
      f'"acc":{ACCspd},'
      f'"dec":{DECspd},'
      f'"ramp":{ACCramp},'
      f'"rounding":{Rounding},'
      f'"w":"{RUN["WC"]}",'
      f'"loop_mode":"{LoopMode}",'
      f'"dis_wrist":"{DisWrist}"'
      '}\n'
    )
    if RUN['streamActive'] or not RUN['vtk_running']:   # streamed runs: virtual robot mirrors the real one
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      response = robot_move(command)
      if is_error_response(response):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      mj_command(command)
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
    command = (
      '{"cmd":"move_joints",'
      f'"j":[{J1Val},{J2Val},{J3Val},{J4Val},{J5Val},{J6Val},{J7Val},{J8Val},{J9Val}],'
      f'"spd":{Speed},'
      f'"spd_type":"{map_spd_type(speedPrefix)}",'
      f'"acc":{ACCspd},'
      f'"dec":{DECspd},'
      f'"ramp":{ACCramp},'
      f'"wc":"{RUN["WC"]}",'
      f'"loop_mode":"{LoopMode}"'
      '}\n'
    )
    if RUN['streamActive'] or not RUN['vtk_running']:   # streamed runs: virtual robot mirrors the real one
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      response = robot_move(command)
      if is_error_response(response):
        ErrorHandler(response)   
      else:
        displayPosition(response)  
    else:
      if not RUN['offlineMode']:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        start_send_serial_thread(command)
      rj_command(command)
      wait_until_all_locks_free(min_hold_time=0.05)
      
  ##Move A Command##
  if (RUN['cmdType'] == "Move A"):
    if (RUN['moveInProc'] == 0):
        RUN['moveInProc'] = 1

    subCmd = command[:10]

    if (subCmd == "Move A End"):
        almStatusLab.config(text="Move A must start with a Mid followed by End", style="Alarm.TLabel")
        almStatusLab2.config(text="Move A must start with a Mid followed by End", style="Alarm.TLabel")
    else:
        # -----------------------------
        # Parse MID row
        # -----------------------------
        xIndex = command.find(" X ")
        yIndex = command.find(" Y ")
        zIndex = command.find(" Z ")
        rzIndex = command.find(" Rz ")
        ryIndex = command.find(" Ry ")
        rxIndex = command.find(" Rx ")
        j7Index = command.find(" J7 ")
        SpeedIndex = command.find(" Sp ")
        ACCspdIndex = command.find(" Ac ")
        DECspdIndex = command.find(" Dc ")
        ACCrampIndex = command.find(" Rm ")
        WristConfIndex = command.find(" $")

        RUN['xVal'] = command[xIndex+3:yIndex].strip()
        RUN['yVal'] = command[yIndex+3:zIndex].strip()
        RUN['zVal'] = command[zIndex+3:rzIndex].strip()

        rzVal = command[rzIndex+4:ryIndex].strip()
        ryVal = command[ryIndex+4:rxIndex].strip()
        rxVal = command[rxIndex+4:j7Index].strip()

        speedPrefix = command[SpeedIndex+1:SpeedIndex+3].strip()
        Speed = command[SpeedIndex+4:ACCspdIndex].strip()
        ACCspd = command[ACCspdIndex+4:DECspdIndex].strip()
        DECspd = command[DECspdIndex+4:ACCrampIndex].strip()
        ACCramp = command[ACCrampIndex+4:WristConfIndex].strip()
        RUN['WC'] = command[WristConfIndex+3:].strip()

        # -----------------------------
        # Read next row for END position only
        # -----------------------------
        curRow = tab1.progView.curselection()[0]
        selRow = tab1.progView.curselection()[0]

        tab1.progView.selection_clear(0, END)
        selRow += 1
        tab1.progView.select_set(selRow)
        curRow += 1
        selRow = tab1.progView.curselection()[0]
        tab1.progView.see(selRow + 2)

        data = list(map(int, tab1.progView.curselection()))
        end_command = tab1.progView.get(data[0]).decode()

        xIndex = end_command.find(" X ")
        yIndex = end_command.find(" Y ")
        zIndex = end_command.find(" Z ")
        rzIndex = end_command.find(" Rz ")

        Xend = end_command[xIndex+3:yIndex].strip()
        Yend = end_command[yIndex+3:zIndex].strip()
        Zend = end_command[zIndex+3:rzIndex].strip()

        # -----------------------------
        # Build JSON command
        # mid = [X,Y,Z,Rx,Ry,Rz]
        # end = [X,Y,Z]
        # -----------------------------
        if RUN['vtk_running'] and not RUN['streamActive']:
            almStatusLab.config(text="Arc move not yet programmed for virtual robot playback", style="Alarm.TLabel")

        LoopMode = (
            str(CAL['J1OpenLoopVal'].get()) +
            str(CAL['J2OpenLoopVal'].get()) +
            str(CAL['J3OpenLoopVal'].get()) +
            str(CAL['J4OpenLoopVal'].get()) +
            str(CAL['J5OpenLoopVal'].get()) +
            str(CAL['J6OpenLoopVal'].get())
        )

        command = (
            '{"cmd":"move_a",'
              f'"mid":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal}],'
              f'"end":[{Xend},{Yend},{Zend}],'
              f'"spd":{Speed},'
              f'"spd_type":"{map_spd_type(speedPrefix)}",'
              f'"acc":{ACCspd},'
              f'"dec":{DECspd},'
              f'"ramp":{ACCramp},'
              f'"w":"{RUN["WC"]}",'
              f'"loop_mode":"{LoopMode}"'
              '}\n'
        )

        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        response = robot_move(command)
        if is_error_response(response):
            ErrorHandler(response)
        else:
            displayPosition(response)

  ##Move C Command##
  if (RUN['cmdType'] == "Move C"):
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] = 1

    subCmd = command[:10]

    if (subCmd == "Move C Sta" or subCmd == "Move C Pla"):
      almStatusLab.config(text="Move C must start with a Center followed by Start & Plane", style="Alarm.TLabel")
      almStatusLab2.config(text="Move C must start with a Center followed by Start & Plane", style="Alarm.TLabel")

    else:
      # -----------------------------
      # Parse Center row
      # -----------------------------
      xIndex = command.find(" X ")
      yIndex = command.find(" Y ")
      zIndex = command.find(" Z ")
      rzIndex = command.find(" Rz ")
      ryIndex = command.find(" Ry ")
      rxIndex = command.find(" Rx ")
      j7Index = command.find(" J7 ")
      j8Index = command.find(" J8 ")
      j9Index = command.find(" J9 ")
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
      rxVal = command[rxIndex+4:j7Index]

      j7Val = command[j7Index+4:j8Index]
      j8Val = command[j8Index+4:j9Index]
      j9Val = command[j9Index+4:SpeedIndex]

      speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
      Speed = command[SpeedIndex+4:ACCspdIndex]
      ACCspd = command[ACCspdIndex+4:DECspdIndex]
      DECspd = command[DECspdIndex+4:ACCrampIndex]
      ACCramp = command[ACCrampIndex+4:WristConfIndex]
      RUN['WC'] = command[WristConfIndex+3:]

      # -----------------------------
      # Read next row = Start position
      # -----------------------------
      curRow = tab1.progView.curselection()[0]
      selRow = tab1.progView.curselection()[0]
      last = tab1.progView.index('end')

      for row in range(0, selRow):
        tab1.progView.itemconfig(row, {'fg': ui.C['info']})
      tab1.progView.itemconfig(selRow, {'fg': ui.C['accent']})
      for row in range(selRow+1, last):
        tab1.progView.itemconfig(row, {'fg': ui.C['fg']})

      tab1.progView.selection_clear(0, END)
      selRow += 1
      tab1.progView.select_set(selRow)
      curRow += 1
      selRow = tab1.progView.curselection()[0]
      tab1.progView.see(selRow+2)

      data = list(map(int, tab1.progView.curselection()))
      command = tab1.progView.get(data[0]).decode()

      xIndex = command.find(" X ")
      yIndex = command.find(" Y ")
      zIndex = command.find(" Z ")
      rzIndex = command.find(" Rz ")

      Xstart = command[xIndex+3:yIndex]
      Ystart = command[yIndex+3:zIndex]
      Zstart = command[zIndex+3:rzIndex]

      # -----------------------------
      # Read next row = Plane position
      # -----------------------------
      curRow = tab1.progView.curselection()[0]
      selRow = tab1.progView.curselection()[0]
      last = tab1.progView.index('end')

      for row in range(0, selRow):
        tab1.progView.itemconfig(row, {'fg': ui.C['info']})
      tab1.progView.itemconfig(selRow, {'fg': ui.C['accent']})
      for row in range(selRow+1, last):
        tab1.progView.itemconfig(row, {'fg': ui.C['fg']})

      tab1.progView.selection_clear(0, END)
      selRow += 1
      tab1.progView.select_set(selRow)
      curRow += 1
      selRow = tab1.progView.curselection()[0]
      tab1.progView.see(selRow+2)

      data = list(map(int, tab1.progView.curselection()))
      command = tab1.progView.get(data[0]).decode()

      xIndex = command.find(" X ")
      yIndex = command.find(" Y ")
      zIndex = command.find(" Z ")
      rzIndex = command.find(" Rz ")

      Xplane = command[xIndex+3:yIndex]
      Yplane = command[yIndex+3:zIndex]
      Zplane = command[zIndex+3:rzIndex]

      LoopMode = (
        str(CAL['J1OpenLoopVal'].get()) +
        str(CAL['J2OpenLoopVal'].get()) +
        str(CAL['J3OpenLoopVal'].get()) +
        str(CAL['J4OpenLoopVal'].get()) +
        str(CAL['J5OpenLoopVal'].get()) +
        str(CAL['J6OpenLoopVal'].get())
      )

      # -----------------------------
      # Stage 1: Move J to Start
      # -----------------------------
      command = (
        f'{{"cmd":"move_j",'
        f'"pose":[{Xstart},{Ystart},{Zstart},{rzVal},{ryVal},{rxVal},{j7Val},{j8Val},{j9Val}],'
        f'"spd":{Speed},'
        f'"spd_type":"{map_spd_type(speedPrefix)}",'
        f'"acc":{ACCspd},'
        f'"dec":{DECspd},'
        f'"ramp":{ACCramp},'
        f'"w":"{RUN["WC"]}",'
        f'"loop_mode":"{LoopMode}"'
        f'}}\n'
      )

      response = robot_move(command)

      # -----------------------------
      # Stage 2: Move C using Center + Plane
      # Teensy uses current position as Start
      # -----------------------------
      start = time.time()

      if RUN['vtk_running'] and not RUN['streamActive']:
        almStatusLab.config(text="Circle move not yet programmed for virtual robot playback", style="Alarm.TLabel")

      command = (
        f'{{"cmd":"move_c",'
        f'"center":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]}],'
        f'"plane":[{Xplane},{Yplane},{Zplane}],'
        f'"pose":[{rzVal},{ryVal},{rxVal}],'
        f'"spd":{Speed},'
        f'"spd_type":"{map_spd_type(speedPrefix)}",'
        f'"acc":{ACCspd},'
        f'"dec":{DECspd},'
        f'"ramp":{ACCramp},'
        f'"w":"{RUN["WC"]}",'
        f'"loop_mode":"{LoopMode}"'
        f'}}\n'
      )

      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0, command)

      response = robot_move(command)
      end = time.time()

      if is_error_response(response):
        ErrorHandler(response)
      else:
        displayPosition(response)



  ##Start Spline
  # Kept so older programs still load and run. Motion rows are streamed
  # automatically now, so this row does nothing.
  if (RUN['cmdType'] == "Start "):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2

  ##End Spline
  # Acts as an explicit stop point: closes the motion queue and waits for the robot.
  if (RUN['cmdType'] == "End Sp"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    stream_end()

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
      index = find_prog_row(tabNum)
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index)  
    elif (status == "fail"): 
      tabNum = ("Tab Number " + str(command[failIndex+6:]) + "\r\n").encode('utf-8')
      index = find_prog_row(tabNum)
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index) 
  
  ##Virtual Part Pick##
  if (RUN['cmdTypeLong'] == "Virtual Pic"):
    if RUN['vtk_running']:

        if (RUN['moveInProc'] == 1):
          RUN['moveInProc'] = 2

        fileNameIndex = command.find(" - ")
        filename = str(command[fileNameIndex+3:]).strip()
        attachtogripper(filename)

  ##Virtual Part Place##
  if (RUN['cmdTypeLong'] == "Virtual Pla"):
    if RUN['vtk_running']:

        if (RUN['moveInProc'] == 1):
          RUN['moveInProc'] = 2

        fileNameIndex = command.find(" - ")
        filename = str(command[fileNameIndex+3:]).strip()
        detachfromgripper(filename)      



  # Keep the virtual robot on the last reported position after each row. Not while
  # streaming: the stored position only updates when the stream closes, so this would
  # snap the virtual robot back to where the stream started (live telemetry drives it
  # then, and stream_end() syncs it exactly).
  if not RUN['streamActive']:
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
      logger.warning("XInput DLL not found - Xbox controller support disabled")

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
  if _xinput is not None:
      XInputGetState = _xinput.XInputGetState
      XInputGetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_STATE)]
      XInputGetState.restype  = ctypes.c_uint
  else:
      def XInputGetState(_idx, _state):
          return 1167  # ERROR_DEVICE_NOT_CONNECTED -> shows 'No XInput controller detected'

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
      axis = code // 10
      direction = 1 if code % 10 == 1 else -1
      _tk_call(globals().get("LiveJointJog"), axis, direction)

  def _gui_start_cart(code):
      axis_map = {
          10: ("X", "neg"),
          11: ("X", "pos"),
          20: ("Y", "neg"),
          21: ("Y", "pos"),
          30: ("Z", "neg"),
          31: ("Z", "pos"),
          40: ("Rx", "neg"),
          41: ("Rx", "pos"),
          50: ("Ry", "neg"),
          51: ("Ry", "pos"),
          60: ("Rz", "neg"),
          61: ("Rz", "pos"),
      }

      axis, direction = axis_map[code]
      _tk_call(globals().get("LiveCarJog"), axis, direction)

  def _gui_start_tool(code):
      axis_map = {
          30: ("Z", "neg"),
          31: ("Z", "pos"),
      }

      if code not in axis_map:
          return

      axis, direction = axis_map[code]
      _tk_call(globals().get("LiveToolJog"), axis, direction)

  # ----- Teach (X button) -----
  def _teach_position():
      _tk_call(globals().get("teachInsertBelSelected"))

  # ----- Servo gripper toggle (Y button) -----

  RUN['_grip_closed'] = False

  def _toggle_servo_gripper():
      def worker():
          try:
              if RUN['_grip_closed']:
                  RUN['nano'].gripper_open()
              else:
                  RUN['nano'].gripper_close()

              RUN['_grip_closed'] = not RUN['_grip_closed']

          except Exception as e:
              print(e)

      threading.Thread(target=worker, daemon=True).start()

    # ----- Pneumatic gripper toggle (START) -----

  RUN['_pneu_open'] = False    # False = closed

  def _toggle_pneu_gripper():

      def worker():
          try:
              if RUN['_pneu_open']:
                  # Close gripper
                  RUN['nano'].set_output(8, 1)
              else:
                  # Open gripper
                  RUN['nano'].set_output(8, 0)

              RUN['_pneu_open'] = not RUN['_pneu_open']

          except Exception as e:
              print("Pneumatic Gripper Error:", e)

      threading.Thread(target=worker, daemon=True).start()

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
        code = RUN['_pending_start']
        RUN['_pending_start'] = None

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
          item = RUN['_cart_pending']
          RUN['_cart_pending'] = None
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
      if axis == "Tz":
          return 30 if d < 0 else 31
      return None

  def _request_switch_tool(new_active):
      old = RUN['_tool_current']

      if old == new_active:
          return

      def do_start_if_pending():
          item = RUN['_tool_pending']
          RUN['_tool_pending'] = None

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

          try:
              root.after(SWITCH_DELAY_MS, do_start_if_pending)
          except Exception:
              do_start_if_pending()

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
          if idx is None:
              _request_switch(None)
              _request_switch_cart(None)
              _request_switch_tool(None)
              RUN['_xbox_enabled'] = False
              _lbl("XBOX CONTROLLER NOT RESPONDING - XBOX CONTROL OFF", style="Alarm.TLabel")
              return

          st = XINPUT_STATE()

          if XInputGetState(idx, ctypes.byref(st)) != 0:
              _request_switch(None)
              _request_switch_cart(None)
              _request_switch_tool(None)
              RUN['_xbox_enabled'] = False
              _lbl("XBOX CONTROLLER NOT RESPONDING - XBOX CONTROL OFF", style="Alarm.TLabel")
              return

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
      RUN['_xbox_enabled'] = True
      threading.Thread(target=_poll_loop, daemon=True).start()
      _lbl("Xbox ON / polling…", style="Warn.TLabel")

else:
  try:
    from inputs import get_gamepad
  except ImportError:
    get_gamepad = None
    logger.warning("'inputs' package not installed - Xbox controller disabled (pip install inputs)")
  def xbox():
    if get_gamepad is None:
      almStatusLab.config(text="XBOX: install the 'inputs' package (pip install inputs)", style="Warn.TLabel")
      almStatusLab2.config(text="XBOX: install the 'inputs' package (pip install inputs)", style="Warn.TLabel")
      return
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
                command = {
                    "cmd": "set_output",
                    "pin": int(outputNum),
                    "state": 0
                }
                response = RUN['nano'].command(command)
              else:
                grip = 0
                outputNum = DO1onEntryField.get()
                command = {
                    "cmd": "set_output",
                    "pin": int(outputNum),
                    "state": 1
                }
                response = RUN['nano'].command(command)
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



########################################
###  JOG THREADING ####
########################################

def send_serial_command(cmd):
    response = RUN['robot'].command(cmd)

    if response is None:
        return

    response = str(response).strip()

    if response == "":
        return

    IncJogStatVal = int(RUN['IncJogStat'].get())
    forceDisplay = RUN.get('forceDisplayPosition', False)

    # Errors are always reported (JSON errors start with "{" too, so check them first)
    if is_error_response(response):
        ErrorHandler(response)

    elif IncJogStatVal or RUN['progRunning'] or forceDisplay:
        if response.startswith("E"):          # legacy error strings
            ErrorHandler(response)

        elif response.startswith("{"):
            displayPosition(response)

        else:
            logger.info(f"Robot response: {response}")

    if forceDisplay:
        RUN['forceDisplayPosition'] = False


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

       

### JOG COMMANDS ###
 
def J1jogNeg(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{round(float(CAL["J1AngCur"]) - value, 3)},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)   
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
   

def J1jogPos(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{round(float(CAL["J1AngCur"]) + value, 3)},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)  
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
     

def J2jogNeg(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{round(float(CAL["J2AngCur"]) - value, 3)},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

     

def J2jogPos(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{round(float(CAL["J2AngCur"]) + value, 3)},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J3jogNeg(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{round(float(CAL["J3AngCur"]) - value, 3)},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J3jogPos(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{round(float(CAL["J3AngCur"]) + value, 3)},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J4jogNeg(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{round(float(CAL["J4AngCur"]) - value, 3)},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J4jogPos(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{round(float(CAL["J4AngCur"]) + value, 3)},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command) 

def J5jogNeg(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{round(float(CAL["J5AngCur"]) - value, 3)},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J5jogPos(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{round(float(CAL["J5AngCur"]) + value, 3)},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)   

def J6jogNeg(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{round(float(CAL["J6AngCur"]) - value, 3)},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)

def J6jogPos(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{round(float(CAL["J6AngCur"]) + value, 3)},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  #send command to virtual robot
  rj_command(command)
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)


def J7jogNeg(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{round(float(CAL["J7PosCur"]) - value, 3)},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  response = RUN['robot'].command(command)
  if is_error_response(response):
    ErrorHandler(response)    
  else:
    displayPosition(response)

def J7jogPos(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{round(float(CAL["J7PosCur"]) + value, 3)},'
    f'{CAL["J8PosCur"]},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  response = RUN['robot'].command(command)
  if is_error_response(response):
    ErrorHandler(response)    
  else:
    displayPosition(response) 



def J8jogNeg(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{round(float(CAL["J8PosCur"]) - value, 3)},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  response = RUN['robot'].command(command)
  if is_error_response(response):
    ErrorHandler(response)    
  else:
    displayPosition(response)



def J8jogPos(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{round(float(CAL["J8PosCur"]) + value, 3)},'
    f'{CAL["J9PosCur"]}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  response = RUN['robot'].command(command)
  if is_error_response(response):
    ErrorHandler(response)    
  else:
    displayPosition(response)  


def J9jogNeg(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{round(float(CAL["J9PosCur"]) - value, 3)}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  response = RUN['robot'].command(command)
  if is_error_response(response):
    ErrorHandler(response)    
  else:
    displayPosition(response)



def J9jogPos(value):
  value = round(float(value), 3)
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
  command = (
    f'{{"cmd":"move_joints",'
    f'"j":[{CAL["J1AngCur"]},'
    f'{CAL["J2AngCur"]},'
    f'{CAL["J3AngCur"]},'
    f'{CAL["J4AngCur"]},'
    f'{CAL["J5AngCur"]},'
    f'{CAL["J6AngCur"]},'
    f'{CAL["J7PosCur"]},'
    f'{CAL["J8PosCur"]},'
    f'{round(float(CAL["J9PosCur"]) - value, 3)}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)    
  response = RUN['robot'].command(command)
  if is_error_response(response):
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


def LiveJointJog(axis, direction):
  # global RUN['xboxUse']
  #global liveJog
  RUN['liveJog'] = True
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
  command = (
    f'{{"cmd":"live_joint_jog",'
    f'"axis":{axis},'
    f'"dir":{direction},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  if (RUN['vtk_running'] == True):
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


def LiveCarJog(axis, direction):
  RUN['liveJog'] = True
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
  command = (
    f'{{"cmd":"live_cart_jog",'
    f'"axis":"{axis}",'
    f'"dir":"{direction}",'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  if (RUN['vtk_running'] == True):
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


def LiveToolJog(axis, direction):
  RUN['liveJog'] = True
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
  command = (
    f'{{"cmd":"live_tool_jog",'
    f'"axis":"{axis}",'
    f'"dir":"{direction}",'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  if (RUN['vtk_running'] == True):
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
  if RUN['liveJog']:
    RUN['liveJog'] = False

    if not RUN['offlineMode']:
      time.sleep(.15)

      command = "\n"
      IncJogStatVal = int(RUN['IncJogStat'].get())

      if IncJogStatVal == 0:
        response = RUN['robot'].command_raw(command)

        if is_error_response(response):
          ErrorHandler(response)
        else:
          displayPosition(response)
          RUN['VR_angles'] = [
            float(CAL['J1AngCur']),
            float(CAL['J2AngCur']),
            float(CAL['J3AngCur']),
            float(CAL['J4AngCur']),
            float(CAL['J5AngCur']),
            float(CAL['J6AngCur'])
          ]
          setStepMonitorsVR()

    else:
      def refresh_when_virtual_jog_stops():
        if (
          drive_lock.locked() or
          live_jog_lock.locked() or
          live_cartesian_lock.locked() or
          live_tool_lock.locked()
        ):
          root.after(20, refresh_when_virtual_jog_stops)
          return

        refresh_gui_from_joint_angles(RUN['VR_angles'])
        RUN['KinematicError'] = 0

      root.after(20, refresh_when_virtual_jog_stops)


   
    
def _cart_jog_pose(axis, delta):
  """Current pose moved by `delta` along (X/Y/Z, mm) or about (Rx/Ry/Rz, deg) a WORLD axis.
  Rotations turn the tool about the world axis through the TCP. The pose fields hold a
  rotation vector, so simply adding degrees to one field (the old code) was not a rotation
  about any fixed axis and reversed direction near the usual tool-down orientation."""
  if RUN['offlineMode']:
    fk = kinematics.forward_kinematics(RUN['VR_angles'])
    cur = list(fk[:3]) + [math.degrees(v) for v in fk[3:]]
  else:
    cur = [float(CAL['XcurPos']), float(CAL['YcurPos']), float(CAL['ZcurPos']),
           float(CAL['RzcurPos']), float(CAL['RycurPos']), float(CAL['RxcurPos'])]
  pose = list(cur)
  if axis in ("X", "Y", "Z"):
    pose["XYZ".index(axis)] += delta
  else:
    R = ar4_to_R(*cur[3:])
    world_axis = np.eye(3)[("Rx", "Ry", "Rz").index(axis)]
    pose[3:] = list(R_to_ar4(_rot_axis(world_axis, math.radians(delta)) @ R, ref=np.array(cur[3:])))
  return pose


def _cart_inc_jog(axis, delta):
  """One incremental Cartesian jog (the +/- buttons with 'increment' ticked): a Move J to the
  current pose shifted by `delta`. Sent to the robot and run on the virtual robot."""
  _move_j_to_pose(_cart_jog_pose(axis, float(delta)))


def _move_j_to_pose(pose):
  """Move J to `pose` [x, y, z, rz, ry, rx] with the speed settings from the main screen.
  Sent to the robot (when connected) and run on the virtual robot."""
  checkSpeedVals()
  speedPrefix = {"mm per Sec": "Sm", "Seconds": "Ss"}.get(speedOption.get(), "Sp")
  Speed = speedEntryField.get()
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'], RUN['yVal'], RUN['zVal'] = (f"{v:.3f}" for v in pose[:3])
  rzVal, ryVal, rxVal = (f"{v:.3f}" for v in pose[3:])
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = (
    f'{{"cmd":"move_j",'
    f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal},{j7Val},{j8Val},{j9Val}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mj_command(command)


def XjogNeg(value):  _cart_inc_jog("X", -float(value))
def XjogPos(value):  _cart_inc_jog("X", float(value))
def YjogNeg(value):  _cart_inc_jog("Y", -float(value))
def YjogPos(value):  _cart_inc_jog("Y", float(value))
def ZjogNeg(value):  _cart_inc_jog("Z", -float(value))
def ZjogPos(value):  _cart_inc_jog("Z", float(value))
def RxjogNeg(value): _cart_inc_jog("Rx", -float(value))
def RxjogPos(value): _cart_inc_jog("Rx", float(value))
def RyjogNeg(value): _cart_inc_jog("Ry", -float(value))
def RyjogPos(value): _cart_inc_jog("Ry", float(value))
def RzjogNeg(value): _cart_inc_jog("Rz", -float(value))
def RzjogPos(value): _cart_inc_jog("Rz", float(value))


def TXjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  speedtype = speedOption.get()
  # dont allow mm/sec - switch to sec
  if speedtype == "mm per Sec":
    speedMenu = OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss"
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0, "50")
  # seconds
  if speedtype == "Seconds":
    speedPrefix = "Ss"
  # percent
  if speedtype == "Percent":
    speedPrefix = "Sp"
  Speed = speedEntryField.get()
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = (
    str(CAL['J1OpenLoopVal'].get()) +
    str(CAL['J2OpenLoopVal'].get()) +
    str(CAL['J3OpenLoopVal'].get()) +
    str(CAL['J4OpenLoopVal'].get()) +
    str(CAL['J5OpenLoopVal'].get()) +
    str(CAL['J6OpenLoopVal'].get())
  )
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"X",'
    f'"dir":"neg",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)


def TYjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Y",'
    f'"dir":"neg",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

def TZjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Z",'
    f'"dir":"neg",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)




def TRxjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Rx",'
    f'"dir":"neg",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  ) 
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TRyjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Ry",'
    f'"dir":"neg",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  ) 
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TRzjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Rz",'
    f'"dir":"neg",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  ) 
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TXjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"X",'
    f'"dir":"pos",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  ) 
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TYjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Y",'
    f'"dir":"pos",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  ) 
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

def TZjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Z",'
    f'"dir":"pos",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  ) 
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

def TRxjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Rx",'
    f'"dir":"pos",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  ) 
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command)

def TRyjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Ry",'
    f'"dir":"pos",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  ) 
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    start_send_serial_thread(command)
  mt_command(command) 

def TRzjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
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
  command = (
    f'{{"cmd":"jog_t",'
    f'"axis":"Rz",'
    f'"dir":"pos",'
    f'"step":{value},'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"A",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  ) 
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
    #newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" Rnd "+Rounding+" $ "+RUN['WC'] 
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" Rnd "+Rounding+" $ A" 
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
  elif(movetype == "Virtual Pick" or movetype == "Virtual Place"):
    newPos = movetype
    # Prefer the CAD selected in Virtual Import; fall back to the first saved part
    if selected_imported_filename:
      filename = os.path.splitext(selected_imported_filename)[0]
    else:
      try:
        with open(CAD_IMPORTED_SCENE_FILE, "r") as f:
          filename = os.path.splitext(json.load(f)[0]["filename"])[0]
      except Exception:
        filename = "Part Name Here"
    tab1.progView.insert(selRow, bytes(newPos + " - " + filename + '\n', 'utf-8')) 
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
    slaveID = MBslaveEntryField.get().strip()
    address = MBaddressEntryField.get().strip()
    opVal = MBoperValEntryField.get().strip()

    if not slaveID:
        messagebox.showwarning("Input Error", "Please enter a Slave ID.")
        return

    if not address:
        messagebox.showwarning("Input Error", "Please enter a Register Address.")
        return

    if not opVal:
        messagebox.showwarning("Input Error", "Please enter the number of Registers to read.")
        return

    if int(opVal) < 1:
        messagebox.showwarning("Input Error", "Register count must be 1 or greater.")
        return

    command = (
        f'{{"cmd":"modbus_read_holding_register",'
        f'"slave_id":{slaveID},'
        f'"address":{address},'
        f'"count":{opVal}'
        f'}}\n'
    )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['robot'].command(command)

    MBoutputEntryField.delete(0, 'end')
    MBoutputEntryField.insert(0, response)

def MBreadCoil():
    slaveID = MBslaveEntryField.get().strip()
    address = MBaddressEntryField.get().strip()
    opVal = MBoperValEntryField.get().strip()

    if not slaveID:
        messagebox.showwarning("Input Error", "Please enter a Slave ID.")
        return

    if not address:
        messagebox.showwarning("Input Error", "Please enter a Coil Address.")
        return

    if not opVal:
        messagebox.showwarning("Input Error", "Please enter the number of Coils to read.")
        return

    if int(opVal) < 1:
        messagebox.showwarning("Input Error", "Coil count must be 1 or greater.")
        return

    command = (
        f'{{"cmd":"modbus_read_coil",'
        f'"slave_id":{slaveID},'
        f'"address":{address},'
        f'"count":{opVal}'
        f'}}\n'
    )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['robot'].command(command)

    MBoutputEntryField.delete(0, 'end')
    MBoutputEntryField.insert(0, response)

def MBreadInput():
    slaveID = MBslaveEntryField.get().strip()
    address = MBaddressEntryField.get().strip()
    opVal = MBoperValEntryField.get().strip()

    if not slaveID:
        messagebox.showwarning("Input Error", "Please enter a Slave ID.")
        return

    if not address:
        messagebox.showwarning("Input Error", "Please enter an Input Address.")
        return

    if not opVal:
        messagebox.showwarning("Input Error", "Please enter the number of Inputs to read.")
        return

    if int(opVal) < 1:
        messagebox.showwarning("Input Error", "Input count must be 1 or greater.")
        return

    command = (
        f'{{"cmd":"modbus_read_discrete_input",'
        f'"slave_id":{slaveID},'
        f'"address":{address},'
        f'"count":{opVal}'
        f'}}\n'
    )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['robot'].command(command)

    MBoutputEntryField.delete(0, 'end')
    MBoutputEntryField.insert(0, response)

def MBreadInputReg():
    slaveID = MBslaveEntryField.get().strip()
    address = MBaddressEntryField.get().strip()
    opVal = MBoperValEntryField.get().strip()

    if not slaveID:
        messagebox.showwarning("Input Error", "Please enter a Slave ID.")
        return

    if not address:
        messagebox.showwarning("Input Error", "Please enter an Input Register Address.")
        return

    if not opVal:
        messagebox.showwarning("Input Error", "Please enter the number of Input Registers to read.")
        return

    if int(opVal) < 1:
        messagebox.showwarning("Input Error", "Input Register count must be 1 or greater.")
        return

    command = (
        f'{{"cmd":"modbus_read_input_register",'
        f'"slave_id":{slaveID},'
        f'"address":{address},'
        f'"count":{opVal}'
        f'}}\n'
    )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['robot'].command(command)

    MBoutputEntryField.delete(0, 'end')
    MBoutputEntryField.insert(0, response)


def MBwriteCoil():
    slaveID = MBslaveEntryField.get().strip()
    address = MBaddressEntryField.get().strip()
    opVal = MBoperValEntryField.get().strip()

    if not slaveID:
        messagebox.showwarning("Input Error", "Please enter a Slave ID.")
        return

    if not address:
        messagebox.showwarning("Input Error", "Please enter a Coil Address.")
        return

    if not opVal:
        messagebox.showwarning("Input Error", "Please enter a Coil Value.")
        return

    if opVal not in ("0", "1"):
        messagebox.showwarning("Input Error", "Coil Value must be 0 or 1.")
        return

    command = (
        f'{{"cmd":"modbus_write_coil",'
        f'"slave_id":{slaveID},'
        f'"address":{address},'
        f'"value":{opVal}'
        f'}}\n'
    )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['robot'].command(command)

    MBoutputEntryField.delete(0, 'end')
    MBoutputEntryField.insert(0, response)

   
def MBwriteReg():
    slaveID = MBslaveEntryField.get().strip()
    address = MBaddressEntryField.get().strip()
    opVal = MBoperValEntryField.get().strip()

    if not slaveID:
        messagebox.showwarning("Input Error", "Please enter a Slave ID.")
        return

    if not address:
        messagebox.showwarning("Input Error", "Please enter a Register Address.")
        return

    if not opVal:
        messagebox.showwarning("Input Error", "Please enter a Register Value.")
        return

    command = (
        f'{{"cmd":"modbus_write_register",'
        f'"slave_id":{slaveID},'
        f'"address":{address},'
        f'"value":{opVal}'
        f'}}\n'
    )

    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)

    response = RUN['robot'].command(command)

    MBoutputEntryField.delete(0, 'end')
    MBoutputEntryField.insert(0, response)



def QueryModbus():
  command = (
    f'{{"cmd":"modbus_query_test"}}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0, command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0, response)
  

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
  tab1.progView.itemconfig(selRow, {'fg': ui.C['edited']})
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
  tab1.progView.itemconfig(selRow, {'fg': ui.C['edited']})  
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

  elif (option == "MB Hold Reg"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Input" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = "Wait MBhold - SlaveID (1) - Input # " + variable + " = " + inputVal + " : Timeout = " + timoutVal  

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

  elif (option == "MB Hold Reg"):
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
    port = resolve_serial_port(com3PortEntryField.get())
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
  filetypes = (('robot program', '*.ar4 *.AR4'),("all files", "*"))
  filename = ask_open_file(title='Open files',initialdir=folder,filetypes=filetypes)
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
    f.write("## RUN ONCE ##")
    f.write('\n')
    f.write('\n')
    f.write("## START PROGRAM LOOP ##")
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

    

def insertStop():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  value = "Stop Program"           
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
  if not str(servoPos).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"servo","num":0,"pos":' + servoPos + '}\n'
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['nano'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  


def Servo0off():
  save_calibration(CAL)
  servoPos = servo0offEntryField.get()
  if not str(servoPos).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"servo","num":0,"pos":' + servoPos + '}\n'
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['nano'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)


def Servo1on():
  save_calibration(CAL)
  servoPos = servo1onEntryField.get()
  if not str(servoPos).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"servo","num":1,"pos":' + servoPos + '}\n'
  response = RUN['nano'].command(command)


def Servo1off():
  save_calibration(CAL)
  servoPos = servo1offEntryField.get()
  if not str(servoPos).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"servo","num":1,"pos":' + servoPos + '}\n'
  response = RUN['nano'].command(command)


def Servo2on():
  save_calibration(CAL)
  servoPos = servo2onEntryField.get()
  if not str(servoPos).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"servo","num":2,"pos":' + servoPos + '}\n'
  response = RUN['nano'].command(command)


def Servo2off():
  save_calibration(CAL)
  servoPos = servo2offEntryField.get()
  if not str(servoPos).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"servo","num":2,"pos":' + servoPos + '}\n'
  response = RUN['nano'].command(command)


def Servo3on():
  save_calibration(CAL)
  servoPos = servo3onEntryField.get()
  if not str(servoPos).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"servo","num":3,"pos":' + servoPos + '}\n'
  response = RUN['nano'].command(command)


def Servo3off():
  save_calibration(CAL)
  servoPos = servo3offEntryField.get()
  if not str(servoPos).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"servo","num":3,"pos":' + servoPos + '}\n'
  response = RUN['nano'].command(command)


def DO1on():
  outputNum = DO1onEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":1}\n'
  response = RUN['nano'].command(command)


def DO1off():
  outputNum = DO1offEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":0}\n'
  response = RUN['nano'].command(command)


def DO2on():
  outputNum = DO2onEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":1}\n'
  response = RUN['nano'].command(command)


def DO2off():
  outputNum = DO2offEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":0}\n'
  response = RUN['nano'].command(command)


def DO3on():
  outputNum = DO3onEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":1}\n'
  response = RUN['nano'].command(command)


def DO3off():
  outputNum = DO3offEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":0}\n'
  response = RUN['nano'].command(command)


def DO4on():
  outputNum = DO4onEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":1}\n'
  response = RUN['nano'].command(command)

def DO4off():
  outputNum = DO4offEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":0}\n'
  response = RUN['nano'].command(command)


def DO5on():
  outputNum = DO5onEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":1}\n'
  response = RUN['nano'].command(command)


def DO5off():
  outputNum = DO5offEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":0}\n'
  response = RUN['nano'].command(command)


def DO6on():
  outputNum = DO6onEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":1}\n'
  response = RUN['nano'].command(command)


def DO6off():
  outputNum = DO6offEntryField.get()
  if not str(outputNum).strip():
    almStatusLab.config(text="Please Enter Value", style="Alarm.TLabel")
    almStatusLab2.config(text="Please Enter Value", style="Alarm.TLabel")
    return
  command = '{"cmd":"set_output","pin":' + outputNum + ',"state":0}\n'
  response = RUN['nano'].command(command)

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
  RUN['estopActive'] = False
  message = "Auto Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()
  tab1.update()
  tab2.update_idletasks()
  tab2.update() 
  success = FALSE
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return 
  ##### STAGE 1 ########
  command = (
    '{"cmd":"calibrate",'
    f'"calibrate":[{CAL["J1CalStatVal"].get()},{CAL["J2CalStatVal"].get()},{CAL["J3CalStatVal"].get()},'
    f'{CAL["J4CalStatVal"].get()},{CAL["J5CalStatVal"].get()},{CAL["J6CalStatVal"].get()},'
    f'{CAL["J7CalStatVal"].get()},{CAL["J8CalStatVal"].get()},{CAL["J9CalStatVal"].get()}],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)
    if (not RUN['estopActive']):  
      message = "Auto Calibration Stage 1 Successful"
      RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
      setStepMonitorsVR()
      almStatusLab.config(text=message, style="OK.TLabel")
      almStatusLab2.config(text=message, style="OK.TLabel")
      tab1.update_idletasks()
      tab1.update()
      tab2.update_idletasks()
      tab2.update()
      success = TRUE
  else:
    message = "Auto Calibration Stage 1 Failed - See Log" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    tab1.update_idletasks()
    tab1.update()
    tab2.update_idletasks()
    tab2.update()
    ErrorHandler(response)
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb")) 
  ##### STAGE 2 ########
  if (success and not RUN['estopActive']):
    CalStatVal2 = int(CAL['J1CalStatVal2'].get())+int(CAL['J2CalStatVal2'].get())+int(CAL['J3CalStatVal2'].get())+int(CAL['J4CalStatVal2'].get())+int(CAL['J5CalStatVal2'].get())+int(CAL['J6CalStatVal2'].get())
    if(CalStatVal2>0):
      command = (
        '{"cmd":"calibrate_stage2",'
        f'"calibrate":[{CAL["J1CalStatVal2"].get()},{CAL["J2CalStatVal2"].get()},{CAL["J3CalStatVal2"].get()},'
        f'{CAL["J4CalStatVal2"].get()},{CAL["J5CalStatVal2"].get()},{CAL["J6CalStatVal2"].get()},'
        f'{CAL["J7CalStatVal2"].get()},{CAL["J8CalStatVal2"].get()},{CAL["J9CalStatVal2"].get()}],'
        f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
        f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
        f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
        '}\n'
      )
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      response = RUN['robot'].command(command)
      cmdRecEntryField.delete(0, 'end')
      cmdRecEntryField.insert(0,response)
      if (response[:18] == '{"msg":"robot_pos"'):
        displayPosition(response)
        if (not RUN['estopActive']):   
          message = "Auto Calibration Stage 2 Successful"
          RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
          setStepMonitorsVR()
          almStatusLab.config(text=message, style="OK.TLabel")
          almStatusLab2.config(text=message, style="OK.TLabel")
          tab1.update_idletasks()
          tab1.update()
          tab2.update_idletasks()
          tab2.update() 
      else:
        message = "Auto Calibration Stage 2 Failed - See Log" 
        almStatusLab.config(text=message, style="Alarm.TLabel")
        almStatusLab2.config(text=message, style="Alarm.TLabel")
        tab1.update_idletasks()
        tab1.update()
        tab2.update_idletasks()
        tab2.update()
        ErrorHandler(response)
      if "success" in message.strip().lower():
        logger.info(message)
      else:
        logger.error(message)
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb")) 


def calRobotJ1():
  RUN['estopActive'] = False
  message = "J1 Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()  
  tab1.update()
  tab2.update_idletasks()
  tab2.update()
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = (
    '{"cmd":"calibrate",'
    '"calibrate":[1,0,0,0,0,0,0,0,0],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)  
    if (not RUN['estopActive']):
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
  RUN['estopActive'] = False
  message = "J2 Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()
  tab1.update()
  tab2.update_idletasks()
  tab2.update()
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = (
    '{"cmd":"calibrate",'
    '"calibrate":[0,1,0,0,0,0,0,0,0],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)  
    if (not RUN['estopActive']):
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
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))       

def calRobotJ3():
  RUN['estopActive'] = False
  message = "J3 Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()
  tab1.update()
  tab2.update_idletasks()
  tab2.update()
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = (
    '{"cmd":"calibrate",'
    '"calibrate":[0,0,1,0,0,0,0,0,0],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)  
    if (not RUN['estopActive']):
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
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))       

def calRobotJ4():
  RUN['estopActive'] = False
  message = "J4 Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()
  tab1.update()
  tab2.update_idletasks()
  tab2.update()
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = (
    '{"cmd":"calibrate",'
    '"calibrate":[0,0,0,1,0,0,0,0,0],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)  
    if (not RUN['estopActive']):
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
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))      

def calRobotJ5():
  RUN['estopActive'] = False
  message = "J5 Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()
  tab1.update()
  tab2.update_idletasks()
  tab2.update()
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = (
    '{"cmd":"calibrate",'
    '"calibrate":[0,0,0,0,1,0,0,0,0],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)  
    if (not RUN['estopActive']):
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
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))      

def calRobotJ6():
  RUN['estopActive'] = False
  message = "J6 Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()
  tab1.update()
  tab2.update_idletasks()
  tab2.update()
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = (
    '{"cmd":"calibrate",'
    '"calibrate":[0,0,0,0,0,1,0,0,0],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)  
    if (not RUN['estopActive']):
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
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))   

def calRobotJ7():
  RUN['estopActive'] = False
  message = "J7 Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()
  tab1.update()
  tab2.update_idletasks()
  tab2.update()
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    almStatusLab2.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = (
    '{"cmd":"calibrate",'
    '"calibrate":[0,0,0,0,0,0,1,0,0],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)  
    if (not RUN['estopActive']):
      message = "J7 Calibrated Successfully"
      almStatusLab.config(text=message, style="OK.TLabel")
      almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J7 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  

def calRobotJ8():
  RUN['estopActive'] = False
  message = "J8 Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()
  tab1.update()
  tab2.update_idletasks()
  tab2.update()
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = (
    '{"cmd":"calibrate",'
    '"calibrate":[0,0,0,0,0,0,0,1,0],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)  
    if (not RUN['estopActive']):
      message = "J8 Calibrated Successfully"
      almStatusLab.config(text=message, style="OK.TLabel")
      almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J8 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))     

def calRobotJ9():
  RUN['estopActive'] = False
  message = "J9 Calibration In Process"
  almStatusLab.config(text=message, style="OK.TLabel")
  almStatusLab2.config(text=message, style="OK.TLabel")
  tab1.update_idletasks()
  tab1.update()
  tab2.update_idletasks()
  tab2.update()
  if RUN['offlineMode']:
    almStatusLab.config(text="Calibration not supported in offline mode", style="Alarm.TLabel")
    return
  command = (
    '{"cmd":"calibrate",'
    '"calibrate":[0,0,0,0,0,0,0,0,1],'
    f'"offsets":[{CAL["J1calOff"]},{CAL["J2calOff"]},{CAL["J3calOff"]},'
    f'{CAL["J4calOff"]},{CAL["J5calOff"]},{CAL["J6calOff"]},'
    f'{CAL["J7calOff"]},{CAL["J8calOff"]},{CAL["J9calOff"]}]'
    '}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  if (response[:18] == '{"msg":"robot_pos"'):
    displayPosition(response)  
    if (not RUN['estopActive']):
      message = "J9 Calibrated Successfully"
      almStatusLab.config(text=message, style="OK.TLabel")
      almStatusLab2.config(text=message, style="OK.TLabel") 
  else:
    message = "J9 Calibrated Failed" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    ErrorHandler(response)
  if "success" in message.strip().lower():
    logger.info(message)
  else:
    logger.error(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))             
	



def correctPos():
  command = '{"cmd":"correct_position"}\n'
  response = RUN['robot'].command(command)
  displayPosition(response)

def _with_calibration_splash(title, fn):
  """Run a calibration routine behind the animated splash. Calibration blocks this program on
  the serial port for up to a minute; the splash animates in its own process, so the user sees
  the program is alive instead of a frozen window."""
  def wrapped(*a, **k):
    if RUN['offlineMode'] or RUN.get('robot') is None:
      return fn(*a, **k)
    progress = ui.busy(root, f"{title} - the robot is moving to its limit switches")
    try:
      return fn(*a, **k)
    finally:
      progress.close()
  wrapped.__name__ = fn.__name__
  wrapped.__wrapped__ = fn
  return wrapped


calRobotAll = _with_calibration_splash("Auto calibration", calRobotAll)
for _j in range(1, 10):
  globals()[f"calRobotJ{_j}"] = _with_calibration_splash(f"Calibrating J{_j}", globals()[f"calRobotJ{_j}"])
del _j


def requestPos():
  command = '{"cmd":"get_position"}\n'
  response = RUN['robot'].command(command)
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

    CAL['J1CalSwitch'] = J1CalHighLowEntryField.get()
    CAL['J2CalSwitch'] = J2CalHighLowEntryField.get()
    CAL['J3CalSwitch'] = J3CalHighLowEntryField.get()
    CAL['J4CalSwitch'] = J4CalHighLowEntryField.get()
    CAL['J5CalSwitch'] = J5CalHighLowEntryField.get()
    CAL['J6CalSwitch'] = J6CalHighLowEntryField.get()
    CAL['J7CalSwitch'] = J7CalHighLowEntryField.get()
    CAL['J8CalSwitch'] = J8CalHighLowEntryField.get()
    CAL['J9CalSwitch'] = J9CalHighLowEntryField.get()

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

    J1EncMult = str(float(J1EncCPREntryField.get()) / float(J1DriveMSEntryField.get()))
    J2EncMult = str(float(J2EncCPREntryField.get()) / float(J2DriveMSEntryField.get()))
    J3EncMult = str(float(J3EncCPREntryField.get()) / float(J3DriveMSEntryField.get()))
    J4EncMult = str(float(J4EncCPREntryField.get()) / float(J4DriveMSEntryField.get()))
    J5EncMult = str(float(J5EncCPREntryField.get()) / float(J5DriveMSEntryField.get()))
    J6EncMult = str(float(J6EncCPREntryField.get()) / float(J6DriveMSEntryField.get()))

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

    J1negLimLab.config(text="-" + CAL['J1NegLim'], style="Jointlim.TLabel")
    J1posLimLab.config(text=CAL['J1PosLim'], style="Jointlim.TLabel")
    J1jogslide.config(from_=float("-" + CAL['J1NegLim']), to=float(CAL['J1PosLim']), length=180, orient=HORIZONTAL, command=J1sliderUpdate)

    J2negLimLab.config(text="-" + CAL['J2NegLim'], style="Jointlim.TLabel")
    J2posLimLab.config(text=CAL['J2PosLim'], style="Jointlim.TLabel")
    J2jogslide.config(from_=float("-" + CAL['J2NegLim']), to=float(CAL['J2PosLim']), length=180, orient=HORIZONTAL, command=J2sliderUpdate)

    J3negLimLab.config(text="-" + CAL['J3NegLim'], style="Jointlim.TLabel")
    J3posLimLab.config(text=CAL['J3PosLim'], style="Jointlim.TLabel")
    J3jogslide.config(from_=float("-" + CAL['J3NegLim']), to=float(CAL['J3PosLim']), length=180, orient=HORIZONTAL, command=J3sliderUpdate)

    J4negLimLab.config(text="-" + CAL['J4NegLim'], style="Jointlim.TLabel")
    J4posLimLab.config(text=CAL['J4PosLim'], style="Jointlim.TLabel")
    J4jogslide.config(from_=float("-" + CAL['J4NegLim']), to=float(CAL['J4PosLim']), length=180, orient=HORIZONTAL, command=J4sliderUpdate)

    J5negLimLab.config(text="-" + CAL['J5NegLim'], style="Jointlim.TLabel")
    J5posLimLab.config(text=CAL['J5PosLim'], style="Jointlim.TLabel")
    J5jogslide.config(from_=float("-" + CAL['J5NegLim']), to=float(CAL['J5PosLim']), length=180, orient=HORIZONTAL, command=J5sliderUpdate)

    J6negLimLab.config(text="-" + CAL['J6NegLim'], style="Jointlim.TLabel")
    J6posLimLab.config(text=CAL['J6PosLim'], style="Jointlim.TLabel")
    J6jogslide.config(from_=float("-" + CAL['J6NegLim']), to=float(CAL['J6PosLim']), length=180, orient=HORIZONTAL, command=J6sliderUpdate)

    # Build one ordered payload list
    payload = [
    CAL['TFx'], CAL['TFy'], CAL['TFz'], CAL['TFrz'], CAL['TFry'], CAL['TFrx'],

    CAL['J1MotDir'], CAL['J2MotDir'], CAL['J3MotDir'], CAL['J4MotDir'], CAL['J5MotDir'], CAL['J6MotDir'], CAL['J7MotDir'], CAL['J8MotDir'], CAL['J9MotDir'],

    CAL['J1CalDir'], CAL['J2CalDir'], CAL['J3CalDir'], CAL['J4CalDir'], CAL['J5CalDir'], CAL['J6CalDir'], CAL['J7CalDir'], CAL['J8CalDir'], CAL['J9CalDir'],

    CAL['J1CalSwitch'], CAL['J2CalSwitch'], CAL['J3CalSwitch'], CAL['J4CalSwitch'], CAL['J5CalSwitch'], CAL['J6CalSwitch'], CAL['J7CalSwitch'], CAL['J8CalSwitch'], CAL['J9CalSwitch'],

    CAL['J1PosLim'], CAL['J1NegLim'],
    CAL['J2PosLim'], CAL['J2NegLim'],
    CAL['J3PosLim'], CAL['J3NegLim'],
    CAL['J4PosLim'], CAL['J4NegLim'],
    CAL['J5PosLim'], CAL['J5NegLim'],
    CAL['J6PosLim'], CAL['J6NegLim'],

    CAL['J1StepDeg'], CAL['J2StepDeg'], CAL['J3StepDeg'], CAL['J4StepDeg'], CAL['J5StepDeg'], CAL['J6StepDeg'],

    J1EncMult, J2EncMult, J3EncMult, J4EncMult, J5EncMult, J6EncMult,

    CAL['J1ΘDHpar'], CAL['J2ΘDHpar'], CAL['J3ΘDHpar'], CAL['J4ΘDHpar'], CAL['J5ΘDHpar'], CAL['J6ΘDHpar'],
    CAL['J1αDHpar'], CAL['J2αDHpar'], CAL['J3αDHpar'], CAL['J4αDHpar'], CAL['J5αDHpar'], CAL['J6αDHpar'],
    CAL['J1dDHpar'], CAL['J2dDHpar'], CAL['J3dDHpar'], CAL['J4dDHpar'], CAL['J5dDHpar'], CAL['J6dDHpar'],
    CAL['J1aDHpar'], CAL['J2aDHpar'], CAL['J3aDHpar'], CAL['J4aDHpar'], CAL['J5aDHpar'], CAL['J6aDHpar'],
    ]

    payload = [str(x).replace(",", "") for x in payload]

    command = (
        '{"cmd":"update_params",'
        '"params":[' + ",".join(f'"{x}"' for x in payload) + ']'
        '}\n'
    )

    try:
        response = RUN['robot'].command(command)
        if is_error_response(response):
          ErrorHandler(response)
    except Exception as e:
        logger.error(f"Serial error: {e}")



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

  J7jogslide.config(from_=-J7NegLim, to=CAL['J7PosLim'], length=125, orient=HORIZONTAL, command=J7sliderUpdate)
  J8jogslide.config(from_=-J8NegLim, to=J8PosLim, length=125, orient=HORIZONTAL, command=J8sliderUpdate)
  J9jogslide.config(from_=-J9NegLim, to=J9PosLim, length=125, orient=HORIZONTAL, command=J9sliderUpdate)

  command = (
      '{"cmd":"config_ext_axis",'
      f'"j7":[{CAL["J7PosLim"]},{CAL["J7rotation"]},{CAL["J7steps"]}],'
      f'"j8":[{J8PosLim},{CAL["J8rotation"]},{CAL["J8steps"]}],'
      f'"j9":[{J9PosLim},{CAL["J9rotation"]},{CAL["J9steps"]}]'
      '}\n'
  )
  response = RUN['robot'].command(command)


def zeroAxis7():
  command = '{"cmd":"zero_j7"}\n'
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0, command)
  response = RUN['robot'].command(command)
  almStatusLab.config(text="J7 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J7 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J7 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  logger.warning(message)
  value = tab8.ElogView.get(0, END)
  pickle.dump(value, open("ErrorLog", "wb"))
  displayPosition(response)


def zeroAxis8():
  command = '{"cmd":"zero_j8"}\n'
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0, command)
  response = RUN['robot'].command(command)
  almStatusLab.config(text="J8 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J8 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J8 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  logger.warning(message)
  value = tab8.ElogView.get(0, END)
  pickle.dump(value, open("ErrorLog", "wb"))
  displayPosition(response)


def zeroAxis9():
  command = '{"cmd":"zero_j9"}\n'
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0, command)
  response = RUN['robot'].command(command)
  almStatusLab.config(text="J9 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J9 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J9 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  logger.warning(message)
  value = tab8.ElogView.get(0, END)
  pickle.dump(value, open("ErrorLog", "wb"))
  displayPosition(response)  


def sendPos():
  command = (
      '{"cmd":"set_position",'
      f'"j":[{CAL["J1AngCur"]},{CAL["J2AngCur"]},{CAL["J3AngCur"]},{CAL["J4AngCur"]},{CAL["J5AngCur"]},{CAL["J6AngCur"]},{CAL["J7PosCur"]},{CAL["J8PosCur"]},{CAL["J9PosCur"]}]'
      '}\n'
  )
  response = RUN['robot'].command(command)

def CalZeroPos():
  command = '{"cmd":"set_position","j":[0,0,0,0,45,0,0,0,0]}\n'
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0, command)
  response = RUN['robot'].command(command)
  displayPosition(response)

  almStatusLab.config(text="Calibration Forced to Home", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Home", style="Warn.TLabel")

  message = "Calibration Forced to Home - this is for commissioning and testing - be careful!"
  logger.warning(message)
  value = tab8.ElogView.get(0, END)
  pickle.dump(value, open("ErrorLog", "wb"))

  RUN['VR_angles'] = [
    float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']),
    float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])
  ]
  setStepMonitorsVR()  

def CalRestPos():
  command = '{"cmd":"set_position","j":[0,0,-89,0,0,0,0,0,0]}\n'
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0, command)
  response = RUN['robot'].command(command)
  displayPosition(response)

  almStatusLab.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")

  message = "Calibration Forced to Vertical - this is for commissioning and testing - be careful!"
  logger.warning(message)
  value = tab8.ElogView.get(0, END)
  pickle.dump(value, open("ErrorLog", "wb"))

  RUN['VR_angles'] = [
    float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']),
    float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])
  ]
  setStepMonitorsVR()


def _set_status_ready():
  """Show SYSTEM READY. Called when the robot (or the virtual robot) has actually completed
  something, never from a button press: clicking a jog button used to overwrite an active
  alarm with "SYSTEM READY" before anything had moved."""
  almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
  almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")


def displayPosition(response):
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0, response)

  try:
    data = json.loads(response)
  except Exception as e:
    manEntryField.delete(0, 'end')
    manEntryField.insert(0, f"JSON position parse error: {e}")
    return

  # Queued-move acknowledgement (streaming): nothing to display
  if data.get("msg") == "ack":
    return

  if data.get("msg") != "robot_pos":
    manEntryField.delete(0, 'end')
    manEntryField.insert(0, "Unexpected response type")
    return

  j = data.get("j", [])
  pose = data.get("pose", [])

  if len(j) < 6 or len(pose) < 9:
    manEntryField.delete(0, 'end')
    manEntryField.insert(0, "Invalid robot_pos JSON length")
    return

  CAL['J1AngCur'] = str(j[0])
  CAL['J2AngCur'] = str(j[1])
  CAL['J3AngCur'] = str(j[2])
  CAL['J4AngCur'] = str(j[3])
  CAL['J5AngCur'] = str(j[4])
  CAL['J6AngCur'] = str(j[5])

  if CAL['J5AngCur'].strip() != '' and float(CAL['J5AngCur']) > 0:
    RUN['WC'] = "F"
  else:
    RUN['WC'] = "N"

  CAL['XcurPos'] = str(pose[0])
  CAL['YcurPos'] = str(pose[1])
  CAL['ZcurPos'] = str(pose[2])
  CAL['RzcurPos'] = str(pose[3])
  CAL['RycurPos'] = str(pose[4])
  CAL['RxcurPos'] = str(pose[5])
  CAL['J7PosCur'] = str(pose[6])
  CAL['J8PosCur'] = str(pose[7])
  CAL['J9PosCur'] = str(pose[8])

  SpeedVioation = str(data.get("speed_violation", 0))
  Debug = str(data.get("debug", ""))
  Flag = str(data.get("flag", ""))

  if not Flag.startswith("E"):
    _set_status_ready()        # the robot answered with a position: it is fine

  J1curAngEntryField.delete(0, 'end')
  J1curAngEntryField.insert(0, CAL['J1AngCur'])
  J2curAngEntryField.delete(0, 'end')
  J2curAngEntryField.insert(0, CAL['J2AngCur'])
  J3curAngEntryField.delete(0, 'end')
  J3curAngEntryField.insert(0, CAL['J3AngCur'])
  J4curAngEntryField.delete(0, 'end')
  J4curAngEntryField.insert(0, CAL['J4AngCur'])
  J5curAngEntryField.delete(0, 'end')
  J5curAngEntryField.insert(0, CAL['J5AngCur'])
  J6curAngEntryField.delete(0, 'end')
  J6curAngEntryField.insert(0, CAL['J6AngCur'])

  XcurEntryField.delete(0, 'end')
  XcurEntryField.insert(0, CAL['XcurPos'])
  YcurEntryField.delete(0, 'end')
  YcurEntryField.insert(0, CAL['YcurPos'])
  ZcurEntryField.delete(0, 'end')
  ZcurEntryField.insert(0, CAL['ZcurPos'])
  RzcurEntryField.delete(0, 'end')
  RzcurEntryField.insert(0, CAL['RzcurPos'])
  RycurEntryField.delete(0, 'end')
  RycurEntryField.insert(0, CAL['RycurPos'])
  RxcurEntryField.delete(0, 'end')
  RxcurEntryField.insert(0, CAL['RxcurPos'])

  J7curAngEntryField.delete(0, 'end')
  J7curAngEntryField.insert(0, CAL['J7PosCur'])
  J8curAngEntryField.delete(0, 'end')
  J8curAngEntryField.insert(0, CAL['J8PosCur'])
  J9curAngEntryField.delete(0, 'end')
  J9curAngEntryField.insert(0, CAL['J9PosCur'])

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
  manEntryField.insert(0, Debug)

  save_calibration(CAL)

  if Flag != "":
    ErrorHandler(Flag)

  if SpeedVioation == '1':
    message = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"
    logger.warning(message)
    value = tab8.ElogView.get(0, END)
    pickle.dump(value, open("ErrorLog", "wb"))
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
  J1CalHighLowEntryField.delete(0,'end')
  J2CalHighLowEntryField.delete(0,'end')
  J3CalHighLowEntryField.delete(0,'end')
  J4CalHighLowEntryField.delete(0,'end')
  J5CalHighLowEntryField.delete(0,'end')
  J6CalHighLowEntryField.delete(0,'end')
  J7CalHighLowEntryField.delete(0,'end')
  J8CalHighLowEntryField.delete(0,'end')
  J9CalHighLowEntryField.delete(0,'end')

def LoadAR4Mk5default():
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
  J1PosLimEntryField.insert(0,str(160))
  J1NegLimEntryField.insert(0,str(160))
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
  J1CalHighLowEntryField.insert(0,"LOW") 
  J2CalHighLowEntryField.insert(0,"LOW")
  J3CalHighLowEntryField.insert(0,"LOW")
  J4CalHighLowEntryField.insert(0,"HIGH")
  J5CalHighLowEntryField.insert(0,"HIGH")
  J6CalHighLowEntryField.insert(0,"HIGH")
  J7CalHighLowEntryField.insert(0,"HIGH")
  J8CalHighLowEntryField.insert(0,"HIGH")
  J9CalHighLowEntryField.insert(0,"HIGH") 


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
  J1CalHighLowEntryField.insert(0,"HIGH") 
  J2CalHighLowEntryField.insert(0,"HIGH")
  J3CalHighLowEntryField.insert(0,"HIGH")
  J4CalHighLowEntryField.insert(0,"HIGH")
  J5CalHighLowEntryField.insert(0,"HIGH")
  J6CalHighLowEntryField.insert(0,"HIGH")
  J7CalHighLowEntryField.insert(0,"HIGH")
  J8CalHighLowEntryField.insert(0,"HIGH")
  J9CalHighLowEntryField.insert(0,"HIGH")  

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
  J1CalHighLowEntryField.insert(0,"HIGH") 
  J2CalHighLowEntryField.insert(0,"HIGH")
  J3CalHighLowEntryField.insert(0,"HIGH")
  J4CalHighLowEntryField.insert(0,"HIGH")
  J5CalHighLowEntryField.insert(0,"HIGH")
  J6CalHighLowEntryField.insert(0,"HIGH")
  J7CalHighLowEntryField.insert(0,"HIGH")
  J8CalHighLowEntryField.insert(0,"HIGH")
  J9CalHighLowEntryField.insert(0,"HIGH")   


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
  J1CalHighLowEntryField.insert(0,"HIGH") 
  J2CalHighLowEntryField.insert(0,"HIGH")
  J3CalHighLowEntryField.insert(0,"HIGH")
  J4CalHighLowEntryField.insert(0,"HIGH")
  J5CalHighLowEntryField.insert(0,"HIGH")
  J6CalHighLowEntryField.insert(0,"HIGH")
  J7CalHighLowEntryField.insert(0,"HIGH")
  J8CalHighLowEntryField.insert(0,"HIGH")
  J9CalHighLowEntryField.insert(0,"HIGH")   

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
  J1CalHighLowEntryField.insert(0,"HIGH") 
  J2CalHighLowEntryField.insert(0,"HIGH")
  J3CalHighLowEntryField.insert(0,"HIGH")
  J4CalHighLowEntryField.insert(0,"HIGH")
  J5CalHighLowEntryField.insert(0,"HIGH")
  J6CalHighLowEntryField.insert(0,"HIGH")
  J7CalHighLowEntryField.insert(0,"HIGH")
  J8CalHighLowEntryField.insert(0,"HIGH")
  J9CalHighLowEntryField.insert(0,"HIGH")   

def LoadAR4Mk4default():
  # MK4 currently shares MK3 defaults — placeholder for future MK4-specific values
  LoadAR4Mk3default()

def LoadTSTMk1default():
  ClearKinTabFields()
  # Motor directions (from TST reference sketch)
  J1MotDirEntryField.insert(0,str(0))
  J2MotDirEntryField.insert(0,str(1))
  J3MotDirEntryField.insert(0,str(1))
  J4MotDirEntryField.insert(0,str(1))
  J5MotDirEntryField.insert(0,str(1))
  J6MotDirEntryField.insert(0,str(1))
  J7MotDirEntryField.insert(0,str(1))
  J8MotDirEntryField.insert(0,str(1))
  J9MotDirEntryField.insert(0,str(1))
  # Calibration directions (from TST reference sketch)
  J1CalDirEntryField.insert(0,str(1))
  J2CalDirEntryField.insert(0,str(0))
  J3CalDirEntryField.insert(0,str(1))
  J4CalDirEntryField.insert(0,str(0))
  J5CalDirEntryField.insert(0,str(0))
  J6CalDirEntryField.insert(0,str(1))
  J7CalDirEntryField.insert(0,str(0))
  J8CalDirEntryField.insert(0,str(0))
  J9CalDirEntryField.insert(0,str(0))
  # Axis limits (from TST reference sketch)
  J1PosLimEntryField.insert(0,str(160))
  J1NegLimEntryField.insert(0,str(160))
  J2PosLimEntryField.insert(0,str(90))
  J2NegLimEntryField.insert(0,str(45))
  J3PosLimEntryField.insert(0,str(52))
  J3NegLimEntryField.insert(0,str(89))
  J4PosLimEntryField.insert(0,str(165))
  J4NegLimEntryField.insert(0,str(165))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(155))
  J6NegLimEntryField.insert(0,str(155))
  # Steps per degree (from TST reference sketch)
  J1StepDegEntryField.insert(0,str(222.222))
  J2StepDegEntryField.insert(0,str(111.111))
  J3StepDegEntryField.insert(0,str(55.5555))
  J4StepDegEntryField.insert(0,str(49.7777))
  J5StepDegEntryField.insert(0,str(21.8602))
  J6StepDegEntryField.insert(0,str(22.2222))
  # Drive microsteps — not used by TST (drives handle internally), set to 1 for safe math
  J1DriveMSEntryField.insert(0,str(1))
  J2DriveMSEntryField.insert(0,str(1))
  J3DriveMSEntryField.insert(0,str(1))
  J4DriveMSEntryField.insert(0,str(1))
  J5DriveMSEntryField.insert(0,str(1))
  J6DriveMSEntryField.insert(0,str(1))
  # Encoder CPR — not used by TST, set to 1 for safe math
  J1EncCPREntryField.insert(0,str(1))
  J2EncCPREntryField.insert(0,str(1))
  J3EncCPREntryField.insert(0,str(1))
  J4EncCPREntryField.insert(0,str(1))
  J5EncCPREntryField.insert(0,str(1))
  J6EncCPREntryField.insert(0,str(1))
  # DH parameters — using AR4-MK5 values until TST DH params are determined
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
  # Calibration pin trigger levels — TST uses hall effect sensors on all joints (HIGH trigger, INPUT_PULLUP)
  J1CalHighLowEntryField.insert(0,"HIGH")
  J2CalHighLowEntryField.insert(0,"HIGH")
  J3CalHighLowEntryField.insert(0,"HIGH")
  J4CalHighLowEntryField.insert(0,"HIGH")
  J5CalHighLowEntryField.insert(0,"HIGH")
  J6CalHighLowEntryField.insert(0,"HIGH")
  J7CalHighLowEntryField.insert(0,"HIGH")
  J8CalHighLowEntryField.insert(0,"HIGH")
  J9CalHighLowEntryField.insert(0,"HIGH")

# Dispatcher for model combo box
MODEL_DEFAULTS_MAP = {
  "AR3":      LoadAR3default,
  "AR4-MK1":  LoadAR4default,
  "AR4-MK2":  LoadAR4Mk2default,
  "AR4-MK3":  LoadAR4Mk3default,
  "AR4-MK4":  LoadAR4Mk4default,
  "AR4-MK5":  LoadAR4Mk5default,
  "TST-MK1":  LoadTSTMk1default,
}

def LoadSelectedModelDefaults():
  selected = modelSelectVar.get()
  func = MODEL_DEFAULTS_MAP.get(selected)
  if func:
    func()
  else:
    messagebox.showwarning("Model Error", "Unknown model: " + selected)

def sendRobotModelToTeensy():
  """Send the selected robot model to the Teensy to store in EEPROM.

  Returns:
    "rebooting" - model changed; Teensy is rebooting to apply the new pin map
    "ok"        - model unchanged; no reboot
    "error"     - write failed or timed out
  """
  selected = modelSelectVar.get()
  # Split combo value into model and version (e.g. "AR4-MK5" -> "AR4", "MK5")
  if "-" in selected:
    model, version = selected.split("-", 1)
  else:
    model = selected
    version = ""

  command = {
    "cmd": "set_robot_id",
    "model": model,
    "version": version,
    "board": "NA",
    "serial": "NA",
    "asset": "NA"
  }

  try:
    # Send command and read lines until we get "Done" or an error
    with RUN['robot'].command_lock:
      RUN['robot'].send_json(command)
      start_time = time.time()
      while time.time() - start_time < 5:
        line = RUN['robot'].serial_connection.readline()
        if not line:
          continue
        line = line.decode(errors="replace").strip()
        if not line:
          continue
        if line == "Done:Rebooting":
          logger.info(f"Robot model '{selected}' written to Teensy EEPROM - Teensy rebooting to apply pin map")
          return "rebooting"
        elif line == "Done":
          logger.info(f"Robot model '{selected}' written to Teensy EEPROM (unchanged, no reboot)")
          return "ok"
        elif line.startswith("Error"):
          logger.error(f"Teensy returned error for set_robot_id: {line}")
          messagebox.showwarning("EEPROM Error", "Failed to write robot model to Teensy: " + line)
          return "error"
        else:
          # Debug output from Teensy — skip it
          logger.debug(f"set_robot_id debug: {line}")
      logger.error("Timed out waiting for set_robot_id response")
      return "error"
  except Exception as e:
    logger.error(f"Serial error sending set_robot_id: {e}")
    return "error"

def reconnectAfterTeensyReboot(timeout=15):
  """After a model change, the Teensy software-resets and drops USB serial.
  Close the port, wait for re-enumeration, reconnect, and re-run hello.

  Returns True on successful reconnect, False otherwise."""
  almStatusLab.config(text="Teensy rebooting - reconnecting...", style="Alarm.TLabel")
  almStatusLab2.config(text="Teensy rebooting - reconnecting...", style="Alarm.TLabel")
  root.update_idletasks()

  try:
    if 'robot' in RUN and RUN['robot']:
      RUN['robot'].disconnect()
  except Exception:
    pass

  # Give the Teensy time to reset and the OS time to drop the old port
  time.sleep(2.0)

  start_time = time.time()
  while time.time() - start_time < timeout:
    try:
      RUN['robot'] = ARRobot(port=CAL['comPort'])
      RUN['robot'].connect()
      time.sleep(0.2)
      logger.info("Reconnected to Teensy on %s after model-change reboot", CAL['comPort'])
      # Drain any boot output (debug prints, etc.) so it can't be mistaken
      # for a command response and desync the serial protocol. The port is
      # opened with timeout=None (blocking), so set a short timeout for the
      # drain and restore it after.
      try:
        ser = RUN['robot'].serial_connection
        old_timeout = ser.timeout
        try:
          ser.timeout = 0.3
          drain_deadline = time.time() + 1.5
          while time.time() < drain_deadline:
            bootline = ser.readline()
            if not bootline:
              break  # buffer quiet
            logger.debug("Teensy boot output: %s", bootline.decode(errors="replace").strip())
        finally:
          ser.timeout = old_timeout
      except Exception:
        pass
      almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
      almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
      # Post-reboot hello: skip mismatch check (we just wrote the model)
      sendHelloToTeensy(checkMismatch=False)
      return True
    except Exception:
      try:
        if 'robot' in RUN and RUN['robot']:
          RUN['robot'].disconnect()
      except Exception:
        pass
      time.sleep(0.5)

  logger.error("Failed to reconnect to Teensy after model-change reboot")
  almStatusLab.config(text="TEENSY REBOOTED - RECONNECT FAILED - CHECK COM PORT", style="Alarm.TLabel")
  almStatusLab2.config(text="TEENSY REBOOTED - RECONNECT FAILED - CHECK COM PORT", style="Alarm.TLabel")
  messagebox.showwarning("Reconnect Failed",
    "The Teensy rebooted to apply the new robot model but the HMI could not reconnect.\n"
    "Check the COM port and press Set Com to reconnect.")
  return False

def LoadMaxdefault():
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
  J1StepDegEntryField.insert(0,str(1422.222))
  J2StepDegEntryField.insert(0,str(1777.777)) 
  J3StepDegEntryField.insert(0,str(1777.777)) 
  J4StepDegEntryField.insert(0,str(1592.888)) 
  J5StepDegEntryField.insert(0,str(349.763)) 
  J6StepDegEntryField.insert(0,str(711.111))
  J1DriveMSEntryField.insert(0,str(12800))
  J2DriveMSEntryField.insert(0,str(12800))  
  J3DriveMSEntryField.insert(0,str(12800))  
  J4DriveMSEntryField.insert(0,str(12800))  
  J5DriveMSEntryField.insert(0,str(12800))  
  J6DriveMSEntryField.insert(0,str(12800))
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
  J1CalHighLowEntryField.insert(0,"HIGH") 
  J2CalHighLowEntryField.insert(0,"HIGH")
  J3CalHighLowEntryField.insert(0,"HIGH")
  J4CalHighLowEntryField.insert(0,"HIGH")
  J5CalHighLowEntryField.insert(0,"HIGH")
  J6CalHighLowEntryField.insert(0,"HIGH")
  J7CalHighLowEntryField.insert(0,"HIGH")
  J8CalHighLowEntryField.insert(0,"HIGH")
  J9CalHighLowEntryField.insert(0,"HIGH")     
  
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
  J1CalHighLowEntryField.insert(0,str(CAL['J1CalSwitch']))
  J2CalHighLowEntryField.insert(0,str(CAL['J2CalSwitch']))
  J3CalHighLowEntryField.insert(0,str(CAL['J3CalSwitch']))
  J4CalHighLowEntryField.insert(0,str(CAL['J4CalSwitch']))
  J5CalHighLowEntryField.insert(0,str(CAL['J5CalSwitch']))
  J6CalHighLowEntryField.insert(0,str(CAL['J6CalSwitch']))
  J7CalHighLowEntryField.insert(0,str(CAL['J7CalSwitch']))
  J8CalHighLowEntryField.insert(0,str(CAL['J8CalSwitch']))
  J9CalHighLowEntryField.insert(0,str(CAL['J9CalSwitch']))



  # Add Encoder control checkboxes
  # Add auto-calibration settings

  updateParams()

def sync_fields_to_calibration():
  ''' synchronize the running CAL from field values '''
  CAL['robotModel'] = modelSelectVar.get()
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

  CAL['VisBrightVal']   = float(VisBrightSlide.get())
  CAL['VisContVal']     = float(VisContrastSlide.get())
  CAL['VisBacColor']    = str(VisBacColorEntryField.get()) 
  CAL['VisScore']       = float(VisScoreEntryField.get())
  CAL['VisX1Val']       = int(VisX1PixEntryField.get())
  CAL['VisY1Val']       = int(VisY1PixEntryField.get())
  CAL['VisX2Val']       = int(VisX2PixEntryField.get())
  CAL['VisY2Val']       = int(VisY2PixEntryField.get())
  CAL['VisRobX1Val']    = float(VisX1RobEntryField.get())
  CAL['VisRobY1Val']    = float(VisY1RobEntryField.get())
  CAL['VisRobX2Val']    = float(VisX2RobEntryField.get())
  CAL['VisRobY2Val']    = float(VisY2RobEntryField.get())
  CAL['zoom']           = float(VisZoomSlide.get())
  CAL['pick180Val']     = int(RUN['pick180'].get()) 
  CAL['pickClosestVal'] = int(RUN['pickClosest'].get())
  CAL['curCam']         = str(visoptions.get())
  CAL['fullRotVal']     = int(RUN['fullRot'].get())
  CAL['autoBGVal']      = int(RUN['autoBG'].get())
  CAL['GC_ST_E1']       = float(GC_ST_E1_EntryField.get())
  CAL['GC_ST_E2']       = float(GC_ST_E2_EntryField.get())
  CAL['GC_ST_E3']       = float(GC_ST_E3_EntryField.get())
  CAL['GC_ST_E4']       = float(GC_ST_E4_EntryField.get())
  CAL['GC_ST_E5']       = float(GC_ST_E5_EntryField.get())
  CAL['GC_ST_E6']       = float(GC_ST_E6_EntryField.get())
  CAL['GC_ST_WC']       = str(GC_ST_WC_EntryField.get())





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

def update_defaults_from_arconfig():
    try:
        if os.path.exists("ARconfig.json"):
            with open("ARconfig.json", "r") as src:
                data = json.load(src)

            with open("defaults.json", "w") as dst:
                json.dump(data, dst, indent=4)

            logger.info("defaults.json updated from ARconfig.json")
    except Exception as e:
        logger.error(f"Failed to update defaults.json from ARconfig.json: {e}")

def SaveAndApplyCalibration():
  try:
    sync_fields_to_calibration()
    prevTeensyModel = RUN.get('teensyModel', "")
    newModel = modelSelectVar.get()
    updateParams()
    time.sleep(.1)
    calExtAxis()
    time.sleep(.1)
    result = sendRobotModelToTeensy()
    time.sleep(.1)
    if result == "rebooting":
      # Teensy reset to apply the new pin map — reconnect, then run the same
      # full re-init as a fresh connection (params + position sync), since the
      # reboot wiped all runtime state including step counters.
      if reconnectAfterTeensyReboot():
        updateParams()
        time.sleep(.1)
        calExtAxis()
        time.sleep(.1)
        sendPos()
        time.sleep(.1)
        requestPos()
        time.sleep(.1)
        updateVisOp()
    else:
      sendHelloToTeensy(checkMismatch=False)
    if result in ("rebooting", "ok") and prevTeensyModel and prevTeensyModel != newModel:
      messagebox.showinfo(
        "Robot Model Changed",
        f"The robot model was changed from {prevTeensyModel} to {newModel}.\n\n"
        "A recalibration is strongly recommended before running the robot —\n"
        "stored positions and calibration data from the previous model may not be valid."
      )
  except:
    logger.error("no serial connection with Teensy board")  
  save_calibration(CAL)
  update_defaults_from_arconfig()


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



def is_error_response(response):
  try:
    data = json.loads(response)
    return data.get("msg") == "error"
  except (json.JSONDecodeError, TypeError, AttributeError):
    return False


def ErrorHandler(response):
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0, response)

  messages = []
  alarm_message = "Unknown Error"

  try:
    data = json.loads(response)
  except Exception:
    data = None

  if data and data.get("msg") == "error":

    code = data.get("code", "")
    axes = data.get("axes", [])
    axis = data.get("axis", None)

    ## AXIS LIMIT ERROR
    if code == "L":
      for i in range(9):
        if i < len(axes) and str(axes[i]) == "1":
          messages.append(f"J{i+1} Axis Limit")

      alarm_message = "Axis Limit Error - See Log"
      # The firmware has dropped the queued moves; continuing would only queue rejections
      if RUN['streamActive']:
        stopProg()

    ## NO ROBOT CONNECTED (online mode, but setCom never connected or the port was lost)
    elif code == "N":
      if RUN['progRunning'] or RUN['streamActive']:
        stopProg()
      message = "Robot not connected - select the COM port or use offline mode"
      messages.append(message)
      alarm_message = message

    ## STREAM ABORTED (queued move arrived after a stop or fault)
    elif code == "Q":
      stopProg()
      message = "Motion queue aborted"
      messages.append(message)
      alarm_message = message

    ## COLLISION ERROR
    elif code == "C":
      for i in range(6):
        if i < len(axes) and str(axes[i]) == "1":
          messages.append(f"J{i+1} Collision or Motor Error")

      correctPos()
      stopProg()
      alarm_message = "Collision or Motor Error - See Log"
      almStatusLab.config(text=alarm_message, style="Alarm.TLabel")
      almStatusLab2.config(text=alarm_message, style="Alarm.TLabel")

    ## REACH ERROR
    elif code == "R":
      RUN['posOutreach'] = TRUE
      stopProg()
      message = "Position Out of Reach"
      messages.append(message)
      alarm_message = message

    ## SPLINE ERROR
    elif code == "S":
      stopProg()
      message = "Spline Can Only Have Move L Types"
      messages.append(message)
      alarm_message = message

    ## GCODE ERROR
    elif code == "G":
      stopProg()
      message = "Gcode file not found"
      messages.append(message)
      alarm_message = message

    ## ESTOP BUTTON
    elif code == "B":
      RUN['estopActive'] = TRUE
      stopProg()
      message = "Estop Button was Pressed"
      messages.append(message)
      alarm_message = message

    ## UPDATE PARAMS ERROR
    elif code == "U":
      RUN['estopActive'] = TRUE
      stopProg()
      message = "Error Updating Firmware Parameters"
      messages.append(message)
      alarm_message = message

    ## CALIBRATION ERROR
    elif code == "A":
      if len(axes) > 0:
        for i in range(9):
          if i < len(axes) and str(axes[i]) == "1":
            messages.append(f"J{i+1} CALIBRATION ERROR")

      elif axis is not None:
        messages.append(f"J{axis} CALIBRATION ERROR")

      else:
        messages.append("Calibration Error")

      alarm_message = "Calibration Error - See Log"
      stopProg()

    ## MODBUS ERROR
    elif code == "modbus":
      stopProg()
      message = "Modbus Error"
      messages.append(message)
      alarm_message = message
      

    else:
      stopProg()
      message = "Unknown Error"
      messages.append(message)
      alarm_message = message
      

  else:
    # Legacy fallback for old E-style responses
    if response == 'Modbus Error':
      stopProg()
      message = "Modbus Error"
      messages.append(message)
      alarm_message = message

    elif response[:1] == 'E':

      if response[1:2] == 'L':
        for i in range(9):
          if response[i+2:i+3] == '1':
            messages.append(f"J{i+1} Axis Limit")
        alarm_message = "Axis Limit Error - See Log"

      elif response[1:2] == 'C':
        for i in range(6):
          if response[i+2:i+3] == '1':
            messages.append(f"J{i+1} Collision or Motor Error")
        correctPos()
        stopProg()
        alarm_message = "Collision or Motor Error - See Log"

      elif response[1:2] == 'R':
        RUN['posOutreach'] = TRUE
        stopProg()
        message = "Position Out of Reach"
        messages.append(message)
        alarm_message = message

      elif response[1:2] == 'S':
        stopProg()
        message = "Spline Can Only Have Move L Types"
        messages.append(message)
        alarm_message = message

      elif response[1:2] == 'G':
        stopProg()
        message = "Gcode file not found"
        messages.append(message)
        alarm_message = message

      elif response[1:2] == 'B':
        RUN['estopActive'] = TRUE
        stopProg()
        message = "Estop Button was Pressed"
        messages.append(message)
        alarm_message = message

      elif response[1:2] == 'U':
        RUN['estopActive'] = TRUE
        stopProg()
        message = "Error Updating Firmware Parameters"
        messages.append(message)
        alarm_message = message

      elif response[1:2] == 'A':
        cal_axis = response[2:3]

        if cal_axis in ["1", "2", "3", "4", "5", "6", "7", "8", "9"]:
          messages.append(f"J{cal_axis} CALIBRATION ERROR")
        else:
          messages.append("Calibration Error")

        alarm_message = "Calibration Error - See Log"
        stopProg()

      else:
        stopProg()
        message = "Unknown Error"
        messages.append(message)
        alarm_message = message

    else:
      stopProg()
      message = "Unknown Error"
      messages.append(message)
      alarm_message = message

  for msg in messages:
    logger.error(msg)

  value = tab8.ElogView.get(0, END)
  pickle.dump(value, open("ErrorLog", "wb"))

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
    curVisStringSel = visoptions.get()
    for i in range(len(camList)):
      if curVisStringSel == camList[i]:
          RUN['selectedCam'] = i
          break
    #RUN['cap'] = cv2.VideoCapture(RUN['selectedCam']) 
    RUN['selectedCam'] = selected_camera_source()
    RUN['cap'] = open_camera(RUN['selectedCam'])
    for _ in range(5):
      RUN['cap'].read()
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

  try:
    if(RUN['cam_on']):
      ret, frame = RUN['cap'].read()
    else:
      curVisStingSel = visoptions.get()
      l = len(camList)
      for i in range(l):
        if (visoptions.get() == camList[i]):
          RUN['selectedCam'] = i
      RUN['selectedCam'] = selected_camera_source()
      RUN['cap'] = open_camera(RUN['selectedCam'])
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
      x1 = int(float(VisX1PixEntryField.get()))
      y1 = int(float(VisY1PixEntryField.get()))
      x2 = int(float(VisX2PixEntryField.get()))
      y2 = int(float(VisY1PixEntryField.get()))
      x3 = int(float(VisX1PixEntryField.get()))
      y3 = int(float(VisY2PixEntryField.get()))
      BG1 = cv2image[x1][y1]
      BG2 = cv2image[x2][y2]
      BG3 = cv2image[x3][y3]
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
  except:
    print("camera failed")

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
    RUN['selectedCam'] = selected_camera_source()
    RUN['cap'] = open_camera(RUN['selectedCam'])
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
  global Visoptmenu
  try:
    Visoptmenu.destroy()   # rebuild instead of stacking a new combobox each call
  except Exception:
    pass
  Visoptmenu = ttk.Combobox(visObjFrame, textvariable=RUN['selectedTemplate'], values=filelist, state='readonly', width=18)
  Visoptmenu.grid(row=0, column=1, sticky="ew", pady=3)
  Visoptmenu.bind("<<ComboboxSelected>>", VisOpUpdate)




def VisOpUpdate(event):
    file = RUN['selectedTemplate'].get()
    logger.info(file)

    # Load image
    img = cv2.imread(file, cv2.IMREAD_COLOR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # --- Square preview settings ---
    TARGET_SIZE = 150   # final image will be 150x150

    h, w = img.shape[:2]

    # Scale so the longest side fits TARGET_SIZE
    scale = TARGET_SIZE / max(w, h)
    new_w = int(w * scale)
    new_h = int(h * scale)

    img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Create square canvas (background color can be changed)
    square = np.zeros((TARGET_SIZE, TARGET_SIZE, 3), dtype=np.uint8)
    # Example gray background instead of black:
    # square[:] = (40, 40, 40)

    # Center the resized image
    x = (TARGET_SIZE - new_w) // 2
    y = (TARGET_SIZE - new_h) // 2
    square[y:y+new_h, x:x+new_w] = img

    # Convert to Tk image
    img = Image.fromarray(square)
    imgtk = ImageTk.PhotoImage(image=img)

    template_lbl.imgtk = imgtk
    template_lbl.configure(image=imgtk, anchor='center')



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

def VisUpdateBriCon(event):
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
  folder = os.path.dirname(sys.executable) if getattr(sys, 'frozen', False) else os.path.dirname(os.path.realpath(__file__))
  filename = ask_open_file(title='Open files',initialdir=folder,filetypes=filetypes)
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
  command = (
    f'{{"cmd":"move_j",'
    f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal},{J7Val},{J8Val},{J9Val}],'
    f'"spd":{Speed},'
    f'"spd_type":"{map_spd_type(speedPrefix)}",'
    f'"acc":{ACCspd},'
    f'"dec":{DECspd},'
    f'"ramp":{ACCramp},'
    f'"w":"{RUN["WC"]}",'
    f'"loop_mode":"{LoopMode}"'
    f'}}\n'
  )
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  if is_error_response(response):
    ErrorHandler(response)   
  else:
    displayPosition(response) 
  



def GCstepFwd():
    GCalmStatusLab.config(text="GCODE READY",  style="OK.TLabel")
    GCexecuteRow() 
    GCselRow = tab7.gcodeView.curselection()[0]
    last = tab7.gcodeView.index('end')
    for row in range (0,GCselRow):
      tab7.gcodeView.itemconfig(row, {'fg': ui.C['info']})
    tab7.gcodeView.itemconfig(GCselRow, {'fg': ui.C['accent']})
    for row in range (GCselRow+1,last):
      tab7.gcodeView.itemconfig(row, {'fg': ui.C['muted']})
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
    command = (
      f'{{"cmd":"delete_SD_program",'
      f'"filename":"{Filename}"'
      f'}}\n'
    )
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    response = RUN['robot'].command(command)
    if is_error_response(response):
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
  command = '{"cmd":"read_SD_files"}\n'
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = RUN['robot'].command(command)
  if is_error_response(response):
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
    Fn = Filename + ".txt"
    command = (
      f'{{"cmd":"play_gcode_file",'
      f'"filename":"{Fn}"'
      f'}}\n'
    )
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    response = RUN['robot'].command(command)
    if is_error_response(response):
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
    # snapshot current position before conversion
    savedJ1 = CAL['J1AngCur']
    savedJ2 = CAL['J2AngCur']
    savedJ3 = CAL['J3AngCur']
    savedJ4 = CAL['J4AngCur']
    savedJ5 = CAL['J5AngCur']
    savedJ6 = CAL['J6AngCur']
    savedJ7 = CAL['J7PosCur']
    savedJ8 = CAL['J8PosCur']
    savedJ9 = CAL['J9PosCur']

    def restorePos():
      CAL['J1AngCur'] = savedJ1
      CAL['J2AngCur'] = savedJ2
      CAL['J3AngCur'] = savedJ3
      CAL['J4AngCur'] = savedJ4
      CAL['J5AngCur'] = savedJ5
      CAL['J6AngCur'] = savedJ6
      CAL['J7PosCur'] = savedJ7
      CAL['J8PosCur'] = savedJ8
      CAL['J9PosCur'] = savedJ9
      sendPos()

    Filename = GcodeFilenameField.get() + ".txt"
    command = (
      f'{{"cmd":"delete_SD_program",'
      f'"filename":"{Filename}"'
      f'}}\n'
    )
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    response = RUN['robot'].command(command)
    last = tab7.gcodeView.index('end')
    for row in range (0,last):
      tab7.gcodeView.itemconfig(row, {'fg': ui.C['fg']})

    def GCthreadProg():
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
        tab7.gcodeView.itemconfig(GCselRow, {'fg': ui.C['accent']})
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
          tab7.GCrunTrue = 0
          GCalmStatusLab.config(text="GCODE CONVERSION COMPLETE",  style="OK.TLabel")
      # restore position on any exit from the loop
      restorePos()

    GCt = threading.Thread(target=GCthreadProg)
    GCt.start() 

     


def GCstopProg():
    lastProg = ""
    tab7.GCrunTrue = 0
    GCalmStatusLab.config(text="GCODE CONVERSION STOPPED",  style="Alarm.TLabel")
    if(RUN['splineActive'] == "1"):
      RUN['splineActive'] = "0"
      if(RUN['stopQueue'] == "1"):
        RUN['stopQueue'] = "0"
        stop()
      if (RUN['moveInProc'] == 1):
        RUN['moveInProc'] == 2
      command = '{"cmd":"spline_end"}\n'
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)
      response = RUN['robot'].command(command)
      if is_error_response(response):
        ErrorHandler(response)   
      else:
        displayPosition(response)         

def GCexecuteRow():
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
      Rounding = "0"
      RUN['WC'] = GC_ST_WC_EntryField.get()
      LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
      Filename = GcodeFilenameField.get() + ".txt"
      command = (
        f'{{"cmd":"write_gcode_move",'
        f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal},{J7Val},{J8Val},{J9Val}],'
        f'"spd":{Speed},'
        f'"spd_type":"{map_spd_type(speedPrefix)}",'
        f'"acc":{ACCspd},'
        f'"dec":{DECspd},'
        f'"ramp":{ACCramp},'
        f'"rnd":{Rounding},'
        f'"w":"{RUN["WC"]}",'
        f'"loop_mode":"{LoopMode}",'
        f'"filename":"{Filename}"'
        f'}}\n'
      )
      cmdSentEntryField.delete(0, 'end') 
      
      print(str(command))

      cmdSentEntryField.insert(0,command)
      response = RUN['robot'].command(command)
      print(str(response))
      if is_error_response(response):
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
        RUN['xVal'] =str(round(float(RUN['xVal']),3))
      else:
        RUN['xVal'] =""  
      if("Y" in command):
        ytemp=command[command.find("Y")+1:]     
        RUN['yVal'] =ytemp[:ytemp.find(" ")]
        RUN['yVal'] =str(round(float(RUN['yVal']),3))
      else:
        RUN['yVal'] =""
      if("Z" in command):
        ztemp=command[command.find("Z")+1:]     
        RUN['zVal'] =ztemp[:ztemp.find(" ")]
        RUN['zVal'] =str(round(float(RUN['zVal']),3))
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
        RUN['xVal'] = str(round((float(GC_ST_E1_EntryField.get())+float(RUN['xVal'])),3))
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
        RUN['yVal'] = str(round((float(GC_ST_E2_EntryField.get())+float(RUN['yVal'])),3))
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
        RUN['zVal'] = str(round((float(GC_ST_E3_EntryField.get())+float(RUN['zVal'])),3))
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

      ACCspd = "0.1"
      DECspd = "0.1"
      ACCramp = "100"
      Rounding = "0"

      RUN['WC'] = GC_ST_WC_EntryField.get()
      #LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
      LoopMode ="111111"
      #DisWrist = str(CAL['DisableWristRotVal'].get())
      Filename = GcodeFilenameField.get() + ".txt"
      command = (
        f'{{"cmd":"write_gcode_move",'
        f'"pose":[{RUN["xVal"]},{RUN["yVal"]},{RUN["zVal"]},{rzVal},{ryVal},{rxVal},{J7Val},{J8Val},{J9Val}],'
        f'"spd":{Speed},'
        f'"spd_type":"{map_spd_type(speedPrefix)}",'
        f'"acc":{ACCspd},'
        f'"dec":{DECspd},'
        f'"ramp":{ACCramp},'
        f'"rnd":{Rounding},'
        f'"w":"{RUN["WC"]}",'
        f'"loop_mode":"{LoopMode}",'
        f'"filename":"{Filename}"'
        f'}}\n'
      )
      RUN['prevxVal'] = RUN['xVal']
      RUN['prevyVal'] = RUN['yVal']
      RUN['prevzVal'] = RUN['zVal']
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)

      #tab8.ElogView.insert(END, command)
      #value=tab8.ElogView.get(0,END)
      #pickle.dump(value,open("ErrorLog","wb"))
      
      response = RUN['robot'].command(command)
      if is_error_response(response):
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
tab1.grid_columnconfigure(1, weight=1, minsize=420)  # Center panel - expands, shrinks first
tab1.grid_columnconfigure(2, weight=0, minsize=800)  # Right panel - fixed minimum width

# ============================================================================
# LEFT PANEL - Program Controls
# ============================================================================
leftPanel = ui.scroll_frame(tab1)   # scrolls when the window is short
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


print("LabelFrame =", LabelFrame)
print("LabelFrame module =", LabelFrame.__module__)
print("ttk_bootstrap.LabelFrame =", ttk_bootstrap.LabelFrame)
print("ttk.LabelFrame =", ttk.LabelFrame)



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

clearNodesBut = ttk.Button(virtualFrame, text="Clear Nodes", command=clear_nodes)
clearNodesBut.grid(row=2, column=0, sticky="ew", padx=2, pady=2)

# Row 7: Position Commands frame
posFrame = LabelFrame(leftPanel, text="Position Commands", padding=5)
posFrame.grid(row=7, column=0, columnspan=2, sticky="new", padx=5, pady=(2, 5))

posFrame.grid_columnconfigure(0, weight=1)

moveSelMenu = OptionMenu(posFrame, options, "Move J", "Move J", "Move L", "Move R", "OFF J", "Move PR", "OFF PR ", "Teach PR", "Move A Mid", "Move A End", "Move C Center", "Move C Start", "Move C Plane", "Move Vis", "Virtual Pick", "Virtual Place", command=posRegFieldVisible)
moveSelMenu.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

# Position Register Entry Field (hidden by default, shown when PR moves selected)
SavePosEntryField = Entry(posFrame, width=4, justify="center")
SavePosEntryField.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
SavePosEntryField.grid_remove()  # Hidden by default, shown by posRegFieldVisible()

teachPosBut = ttk.Button(posFrame, text="Teach / Insert Command", command=teachInsertBelSelected)
teachPosBut.grid(row=2, column=0, sticky="ew", padx=2, pady=2)

modPosBut = ttk.Button(posFrame, text="Modify Command", command=teachReplaceSelected)
modPosBut.grid(row=3, column=0, sticky="ew", padx=2, pady=2)

deleteBut = ttk.Button(posFrame, text="Delete", command=deleteitem)
deleteBut.grid(row=4, column=0, sticky="ew", padx=2, pady=2)

autoCalBut = ttk.Button(posFrame, text="Auto Calibrate CMD", command=insCalibrate)
CalibrateBut = autoCalBut  # Alias for compatibility
autoCalBut.grid(row=5, column=0, sticky="ew", padx=2, pady=2)

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
Label(visionFrame, text="Pass Tab", font=ui.F(8)).grid(row=2, column=0, sticky="e", padx=(2, 1))
Label(visionFrame, text="Fail Tab", font=ui.F(8)).grid(row=2, column=1, sticky="w", padx=(1, 2))

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
waitContainer = LabelFrame(leftPanel, text="Wait - Stop", padding=5)
waitContainer.grid(row=9, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

waitContainer.grid_columnconfigure(0, weight=1)
waitContainer.grid_columnconfigure(1, weight=0)

waitSecBut = ttk.Button(waitContainer, text="Wait Time (seconds)", command=waitTime)
waitSecBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

waitSecField = Entry(waitContainer, width=8, justify="center")
waitSecField.grid(row=0, column=1, sticky="w", padx=2, pady=2)

stopBut = ttk.Button(waitContainer, text="Stop", command=insertStop)
stopBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

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
runStatusLab = Label(centerPanel, text="SYSTEM STARTING - PLEASE WAIT", font=ui.F(10, "bold"), style="OK.TLabel")
runStatusLab.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

# Create alias for compatibility with existing code
almStatusLab = runStatusLab

# Row 0.5: Play controls
playFrame = Frame(centerPanel)
playFrame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

for i in range(4):
    playFrame.grid_columnconfigure(i, weight=1)

# Program controls: Run is the only solid orange control (it moves the robot)
playBut = Button(playFrame, text="▶  Run", style="Run.TButton", command=runProg)
playBut.grid(row=0, column=0, sticky="nsew", padx=2, pady=0)

revBut = Button(playFrame, text="Step back", style="Step.TButton", command=stepRev)
revBut.grid(row=0, column=1, sticky="nsew", padx=2, pady=0)

fwdBut = Button(playFrame, text="Step forward", style="Step.TButton", command=stepFwd)
fwdBut.grid(row=0, column=2, sticky="nsew", padx=2, pady=0)

stopBut = Button(playFrame, text="■  Stop", style="Stop.TButton", command=stopProg)
stopBut.grid(row=0, column=3, sticky="nsew", padx=2, pady=0)

# Row 2: Program view with scrollbar
progframe = Frame(centerPanel, relief="sunken", borderwidth=1)
progframe.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)

scrollbar = Scrollbar(progframe)
scrollbar.pack(side=RIGHT, fill=Y)

tab1.progView = Listbox(progframe, exportselection=0, width=70, height=28, yscrollcommand=scrollbar.set)
tab1.progView.bind('<<ListboxSelect>>', progViewselect)


# ---- program editing: copy / cut / paste / delete / duplicate --------------------------
# Ctrl/Shift+click selects several rows. Ctrl+C copy, Ctrl+X cut, Ctrl+V paste below the
# selection, Delete removes, Ctrl+D duplicates; the same actions are on the right-click menu.
# Copied rows also go to the system clipboard as text, and text copied from elsewhere (one
# program line per row) can be pasted in. Every edit saves the program file, like Delete.
tab1.progView.config(selectmode="extended")
RUN['prog_clipboard'] = []


def _prog_selected_rows():
  return sorted(int(i) for i in tab1.progView.curselection())


def _prog_row_text(i):
  item = tab1.progView.get(i)
  if isinstance(item, bytes):
    item = item.decode('utf-8', 'replace')
  return str(item).rstrip('\r\n')


def _prog_save():
  if RUN.get('progRunning') and getattr(tab1, 'runTrue', 0) == 1:
    return
  name = ProgEntryField.get().strip()
  if not name:
    return
  try:
    with open(path.relpath(name), 'w', encoding='utf-8') as f:
      for i in range(tab1.progView.size()):
        f.write(_prog_row_text(i).strip())
        f.write('\n')
  except Exception as e:
    logger.error(f"Could not save program {name}: {e}")


def _prog_select(first, count):
  tab1.progView.selection_clear(0, END)
  if count > 0:
    tab1.progView.selection_set(first, first + count - 1)
    tab1.progView.see(first + count - 1)
    tab1.progView.activate(first)
    curRowEntryField.delete(0, 'end')
    curRowEntryField.insert(0, first)


def _prog_editing_blocked():
  if getattr(tab1, 'runTrue', 0) == 1:
    _status("Stop the program before editing it", "Warn.TLabel")
    return True
  return False


def prog_copy(event=None):
  rows = _prog_selected_rows()
  if not rows:
    return "break"
  RUN['prog_clipboard'] = [_prog_row_text(i) for i in rows]
  try:
    root.clipboard_clear()
    root.clipboard_append('\n'.join(RUN['prog_clipboard']))
  except Exception:
    pass
  _status(f"Copied {len(rows)} row{'s' if len(rows) != 1 else ''}")
  return "break"


def prog_delete(event=None):
  if _prog_editing_blocked():
    return "break"
  rows = _prog_selected_rows()
  if not rows:
    return "break"
  for i in reversed(rows):
    tab1.progView.delete(i)
  _prog_select(min(rows[0], max(tab1.progView.size() - 1, 0)), 1 if tab1.progView.size() else 0)
  _prog_save()
  _status(f"Deleted {len(rows)} row{'s' if len(rows) != 1 else ''}")
  return "break"


def prog_cut(event=None):
  if _prog_editing_blocked():
    return "break"
  prog_copy()
  return prog_delete()


def prog_paste(event=None):
  if _prog_editing_blocked():
    return "break"
  lines = list(RUN['prog_clipboard'])
  try:   # prefer the system clipboard when it holds something else (e.g. copied from a text file)
    text = root.clipboard_get()
    if text and text.split('\n') != lines:
      lines = [l.rstrip('\r') for l in text.split('\n') if l.strip()]
  except Exception:
    pass
  if not lines:
    return "break"
  rows = _prog_selected_rows()
  at = rows[-1] + 1 if rows else tab1.progView.size()
  for k, line in enumerate(lines):
    tab1.progView.insert(at + k, bytes(line + '\n', 'utf-8'))
  _prog_select(at, len(lines))
  _prog_save()
  _status(f"Pasted {len(lines)} row{'s' if len(lines) != 1 else ''}")
  return "break"


def prog_duplicate(event=None):
  if _prog_editing_blocked():
    return "break"
  rows = _prog_selected_rows()
  if not rows:
    return "break"
  lines = [_prog_row_text(i) for i in rows]
  at = rows[-1] + 1
  for k, line in enumerate(lines):
    tab1.progView.insert(at + k, bytes(line + '\n', 'utf-8'))
  _prog_select(at, len(lines))
  _prog_save()
  return "break"


_prog_menu = tk.Menu(root, tearoff=0)
_prog_menu.add_command(label="Copy", accelerator="Ctrl+C", command=prog_copy)
_prog_menu.add_command(label="Cut", accelerator="Ctrl+X", command=prog_cut)
_prog_menu.add_command(label="Paste below", accelerator="Ctrl+V", command=prog_paste)
_prog_menu.add_command(label="Duplicate", accelerator="Ctrl+D", command=prog_duplicate)
_prog_menu.add_separator()
_prog_menu.add_command(label="Delete", accelerator="Del", command=prog_delete)


def _prog_context_menu(event):
  i = tab1.progView.nearest(event.y)
  if i >= 0 and i not in _prog_selected_rows():
    tab1.progView.selection_clear(0, END)
    tab1.progView.selection_set(i)
  tab1.progView.focus_set()
  try:
    _prog_menu.tk_popup(event.x_root, event.y_root)
  finally:
    _prog_menu.grab_release()
  return "break"


for _seq, _fn in (("<Control-c>", prog_copy), ("<Control-C>", prog_copy),
                  ("<Control-x>", prog_cut), ("<Control-X>", prog_cut),
                  ("<Control-v>", prog_paste), ("<Control-V>", prog_paste),
                  ("<Control-d>", prog_duplicate), ("<Control-D>", prog_duplicate),
                  ("<Delete>", prog_delete), ("<BackSpace>", prog_delete),
                  ("<Button-3>", _prog_context_menu)):
  tab1.progView.bind(_seq, _fn)
if sys.platform == "darwin":
  tab1.progView.bind("<Button-2>", _prog_context_menu)
tab1.progView.bind("<Button-1>", lambda e: tab1.progView.focus_set(), add="+")
tab1.progView.pack(side=LEFT, fill=BOTH, expand=True)
ui.style_listbox(tab1.progView)

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
getSelBut = ttk.Button(manEntryFrame, text="Copy", command=getSel)
getSelBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

insertBut = ttk.Button(manEntryFrame, text="Paste", command=manInsItem)
insertBut.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

replaceBut = ttk.Button(manEntryFrame, text="Replace", command=manReplItem)
replaceBut.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

openTextBut = ttk.Button(manEntryFrame, text="Open .txt File", command=openText)
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
rightPanel = ui.scroll_frame(tab1)   # scrolls when the window is short
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
    lab = Label(frame, font=ui.F(14), text=joint_name)
    lab.grid(row=0, column=0, padx=2)
    
    # Current angle entry
    entry = Entry(frame, width=6, justify="center")
    entry.grid(row=0, column=1, padx=2)
    
    # Negative jog button
    neg_but = Button(frame, text="\u2212", width=3, style="Motion.TButton")
    neg_but.grid(row=0, column=2, padx=2)
    
    # Slider
    slider = Scale(frame, from_=-170, to=170, orient=HORIZONTAL, length=150, bootstyle="warning")
    slider.grid(row=0, column=3, sticky="ew", padx=2)
    
    # Positive jog button
    pos_but = Button(frame, text="+", width=3, style="Motion.TButton")
    pos_but.grid(row=0, column=4, padx=2)
    
    # Limit labels (second row)
    neg_lim_lab = Label(frame, font=ui.F(8), text="-170", style="Jointlim.TLabel")
    neg_lim_lab.grid(row=1, column=2, sticky="w")
    
    pos_lim_lab = Label(frame, font=ui.F(8), text="170", style="Jointlim.TLabel")
    pos_lim_lab.grid(row=1, column=4, sticky="e")
    
    slide_label = Label(frame, font=ui.F(8))
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
    LiveJointJog(1, -1)
J1jogNegBut.bind("<ButtonPress-1>",SelJ1jogNeg)
J1jogNegBut.bind("<ButtonRelease-1>",StopJog)

def SelJ1jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J1jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(1, 1)
J1jogPosBut.bind("<ButtonPress-1>",SelJ1jogPos)
J1jogPosBut.bind("<ButtonRelease-1>",StopJog)

def J1sliderUpdate(event):
  J1slidelabel.config(text=round(float(J1jogslide.get()),2))

def J1sliderExecute(event):
  J1delta = float(J1jogslide.get()) - float(J1curAngEntryField.get())
  RUN['forceDisplayPosition'] = True
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
    LiveJointJog(2, -1)
J2jogNegBut.bind("<ButtonPress-1>",SelJ2jogNeg)
J2jogNegBut.bind("<ButtonRelease-1>",StopJog)

def SelJ2jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J2jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(2, 1)
J2jogPosBut.bind("<ButtonPress-1>",SelJ2jogPos)
J2jogPosBut.bind("<ButtonRelease-1>",StopJog)

def J2sliderUpdate(event):
  J2slidelabel.config(text=round(float(J2jogslide.get()),2))
def J2sliderExecute(event):
  J2delta = float(J2jogslide.get()) - float(J2curAngEntryField.get())
  RUN['forceDisplayPosition'] = True
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
    LiveJointJog(3, -1)
J3jogNegBut.bind("<ButtonPress-1>",SelJ3jogNeg)
J3jogNegBut.bind("<ButtonRelease-1>",StopJog)

def SelJ3jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J3jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(3, 1)
J3jogPosBut.bind("<ButtonPress-1>",SelJ3jogPos)
J3jogPosBut.bind("<ButtonRelease-1>",StopJog)

def J3sliderUpdate(event):
  J3slidelabel.config(text=round(float(J3jogslide.get()),2))
def J3sliderExecute(event):
  J3delta = float(J3jogslide.get()) - float(J3curAngEntryField.get())
  RUN['forceDisplayPosition'] = True
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
    LiveJointJog(4, -1)
J4jogNegBut.bind("<ButtonPress-1>",SelJ4jogNeg)
J4jogNegBut.bind("<ButtonRelease-1>",StopJog)

def SelJ4jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J4jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(4, 1)
J4jogPosBut.bind("<ButtonPress-1>",SelJ4jogPos)
J4jogPosBut.bind("<ButtonRelease-1>",StopJog)

def J4sliderUpdate(event):
  J4slidelabel.config(text=round(float(J4jogslide.get()),2))
def J4sliderExecute(event):
  J4delta = float(J4jogslide.get()) - float(J4curAngEntryField.get())
  RUN['forceDisplayPosition'] = True
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
    LiveJointJog(5, -1)
J5jogNegBut.bind("<ButtonPress-1>",SelJ5jogNeg)
J5jogNegBut.bind("<ButtonRelease-1>",StopJog)

def SelJ5jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J5jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(5, 1)
J5jogPosBut.bind("<ButtonPress-1>",SelJ5jogPos)
J5jogPosBut.bind("<ButtonRelease-1>",StopJog)

def J5sliderUpdate(event):
  J5slidelabel.config(text=round(float(J5jogslide.get()),2))
def J5sliderExecute(event):
  J5delta = float(J5jogslide.get()) - float(J5curAngEntryField.get())
  RUN['forceDisplayPosition'] = True
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
    LiveJointJog(6, -1)
J6jogNegBut.bind("<ButtonPress-1>",SelJ6jogNeg)
J6jogNegBut.bind("<ButtonRelease-1>",StopJog)

def SelJ6jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J6jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(6, 1)
J6jogPosBut.bind("<ButtonPress-1>",SelJ6jogPos)
J6jogPosBut.bind("<ButtonRelease-1>",StopJog)

def J6sliderUpdate(event):
  J6slidelabel.config(text=round(float(J6jogslide.get()),2))
def J6sliderExecute(event):
  J6delta = float(J6jogslide.get()) - float(J6curAngEntryField.get())
  RUN['forceDisplayPosition'] = True
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
    Label(parent, font=ui.F(14), text=label_text).grid(row=row, column=col, pady=2)
    
    # Create frame for horizontal button layout
    button_frame = Frame(parent)
    button_frame.grid(row=row+1, column=col, pady=2)
    
    neg_but = Button(button_frame, text="\u2212", width=3, style="Motion.TButton")
    neg_but.grid(row=0, column=0, padx=1)
    
    pos_but = Button(button_frame, text="+", width=3, style="Motion.TButton")
    pos_but.grid(row=0, column=1, padx=1)
    
    return neg_but, pos_but

# Helper function for cartesian controls
def create_cart_control(parent, row, col, label_text):
    """Create a cartesian jog control column with horizontal buttons"""
    Label(parent, font=ui.F(14), text=label_text).grid(row=row, column=col, pady=2)
    
    entry = Entry(parent, width=6, justify="center")
    entry.grid(row=row+1, column=col, pady=2)
    
    # Create frame for horizontal button layout
    button_frame = Frame(parent)
    button_frame.grid(row=row+2, column=col, pady=2)
    
    neg_but = Button(button_frame, text="\u2212", width=3, style="Motion.TButton")
    neg_but.grid(row=0, column=0, padx=1)
    
    pos_but = Button(button_frame, text="+", width=3, style="Motion.TButton")
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

def handle_cart_jog(axis, direction, inc_func):
    def _handler(event=None):
        IncJogStatVal = int(RUN['IncJogStat'].get())
        if IncJogStatVal == 1:
            inc_func(float(incrementEntryField.get()))
        else:
            LiveCarJog(axis, direction)
    return _handler


XjogNegBut.bind("<ButtonPress-1>",handle_cart_jog("X", "neg", lambda inc: XjogNeg(inc)))
XjogNegBut.bind("<ButtonRelease-1>",StopJog)

XjogPosBut.bind("<ButtonPress-1>",handle_cart_jog("X", "pos", lambda inc: XjogPos(inc)))
XjogPosBut.bind("<ButtonRelease-1>",StopJog)

YjogNegBut.bind("<ButtonPress-1>",handle_cart_jog("Y", "neg", lambda inc: YjogNeg(inc)))
YjogNegBut.bind("<ButtonRelease-1>",StopJog)

YjogPosBut.bind("<ButtonPress-1>",handle_cart_jog("Y", "pos", lambda inc: YjogPos(inc)))
YjogPosBut.bind("<ButtonRelease-1>",StopJog)

ZjogNegBut.bind("<ButtonPress-1>",handle_cart_jog("Z", "neg", lambda inc: ZjogNeg(inc)))
ZjogNegBut.bind("<ButtonRelease-1>",StopJog)

ZjogPosBut.bind("<ButtonPress-1>",handle_cart_jog("Z", "pos", lambda inc: ZjogPos(inc)))
ZjogPosBut.bind("<ButtonRelease-1>",StopJog)

RzjogNegBut.bind("<ButtonPress-1>",handle_cart_jog("Rz", "neg", lambda inc: RzjogNeg(inc)))
RzjogNegBut.bind("<ButtonRelease-1>",StopJog)

RzjogPosBut.bind("<ButtonPress-1>",handle_cart_jog("Rz", "pos", lambda inc: RzjogPos(inc)))
RzjogPosBut.bind("<ButtonRelease-1>",StopJog)

RyjogNegBut.bind("<ButtonPress-1>",handle_cart_jog("Ry", "neg", lambda inc: RyjogNeg(inc)))
RyjogNegBut.bind("<ButtonRelease-1>",StopJog)

RyjogPosBut.bind("<ButtonPress-1>",handle_cart_jog("Ry", "pos", lambda inc: RyjogPos(inc)))
RyjogPosBut.bind("<ButtonRelease-1>",StopJog)

RxjogNegBut.bind("<ButtonPress-1>",handle_cart_jog("Rx", "neg", lambda inc: RxjogNeg(inc)))
RxjogNegBut.bind("<ButtonRelease-1>",StopJog)

RxjogPosBut.bind("<ButtonPress-1>",handle_cart_jog("Rx", "pos", lambda inc: RxjogPos(inc)))
RxjogPosBut.bind("<ButtonRelease-1>",StopJog)

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
    Label(parent, font=ui.F(14), text=label_text).grid(row=0, column=col, pady=2)
    
    # Create frame for horizontal button layout
    button_frame = Frame(parent)
    button_frame.grid(row=1, column=col, pady=2)
    
    neg_but = Button(button_frame, text="\u2212", width=3, style="Motion.TButton")
    neg_but.grid(row=0, column=0, padx=1)
    
    pos_but = Button(button_frame, text="+", width=3, style="Motion.TButton")
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

def handle_tool_jog(axis, direction, inc_func):
    def _handler(event=None):
        IncJogStatVal = int(RUN['IncJogStat'].get())
        if IncJogStatVal == 1:
            inc_func(float(incrementEntryField.get()))
        else:
            LiveToolJog(axis, direction)
    return _handler


TXjogNegBut.bind("<ButtonPress-1>",handle_tool_jog("X", "neg", lambda inc: TXjogNeg(inc)))
TXjogNegBut.bind("<ButtonRelease-1>",StopJog)

TXjogPosBut.bind("<ButtonPress-1>",handle_tool_jog("X", "pos", lambda inc: TXjogPos(inc)))
TXjogPosBut.bind("<ButtonRelease-1>",StopJog)


TYjogNegBut.bind("<ButtonPress-1>",handle_tool_jog("Y", "neg", lambda inc: TYjogNeg(inc)))
TYjogNegBut.bind("<ButtonRelease-1>",StopJog)

TYjogPosBut.bind("<ButtonPress-1>",handle_tool_jog("Y", "pos", lambda inc: TYjogPos(inc)))
TYjogPosBut.bind("<ButtonRelease-1>",StopJog)


TZjogNegBut.bind("<ButtonPress-1>",handle_tool_jog("Z", "neg", lambda inc: TZjogNeg(inc)))
TZjogNegBut.bind("<ButtonRelease-1>",StopJog)

TZjogPosBut.bind("<ButtonPress-1>",handle_tool_jog("Z", "pos", lambda inc: TZjogPos(inc)))
TZjogPosBut.bind("<ButtonRelease-1>",StopJog)


TRzjogNegBut.bind("<ButtonPress-1>",handle_tool_jog("Rz", "neg", lambda inc: TRzjogNeg(inc)))
TRzjogNegBut.bind("<ButtonRelease-1>",StopJog)

TRzjogPosBut.bind("<ButtonPress-1>",handle_tool_jog("Rz", "pos", lambda inc: TRzjogPos(inc)))
TRzjogPosBut.bind("<ButtonRelease-1>",StopJog)


TRyjogNegBut.bind("<ButtonPress-1>",handle_tool_jog("Ry", "neg", lambda inc: TRyjogNeg(inc)))
TRyjogNegBut.bind("<ButtonRelease-1>",StopJog)

TRyjogPosBut.bind("<ButtonPress-1>",handle_tool_jog("Ry", "pos", lambda inc: TRyjogPos(inc)))
TRyjogPosBut.bind("<ButtonRelease-1>",StopJog)


TRxjogNegBut.bind("<ButtonPress-1>",handle_tool_jog("Rx", "neg", lambda inc: TRxjogNeg(inc)))
TRxjogNegBut.bind("<ButtonRelease-1>",StopJog)

TRxjogPosBut.bind("<ButtonPress-1>",handle_tool_jog("Rx", "pos", lambda inc: TRxjogPos(inc)))
TRxjogPosBut.bind("<ButtonRelease-1>",StopJog)


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

J7Lab = Label(J7jogFrame, font=ui.F(12), text="7th Axis")
J7Lab.grid(row=0, column=0, columnspan=3, pady=2)

J7negLimLab = Label(J7jogFrame, font=ui.F(8), text="0.00", style="Jointlim.TLabel")
J7negLimLab.grid(row=1, column=0, sticky="w", padx=2)
J7curAngEntryField = Entry(J7jogFrame, width=8, justify="center")
J7curAngEntryField.grid(row=1, column=1, padx=5)
J7posLimLab = Label(J7jogFrame, font=ui.F(8), text="0", style="Jointlim.TLabel")
J7posLimLab.grid(row=1, column=2, sticky="e", padx=2)

J7jogslide = Scale(J7jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120, bootstyle="warning")
J7jogslide.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

# Create frame for button layout (- entry +)
J7buttonFrame = Frame(J7jogFrame)
J7buttonFrame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

J7buttonFrame.grid_columnconfigure(0, weight=1)
J7buttonFrame.grid_columnconfigure(1, weight=0)
J7buttonFrame.grid_columnconfigure(2, weight=1)

J7jogNegBut = Button(J7buttonFrame, text="\u2212", width=3, style="Motion.TButton")
J7jogNegBut.grid(row=0, column=0, sticky="w", padx=(0, 2))

J7slideLimLab = Entry(J7buttonFrame, width=8, justify="center", state="readonly")
J7slideLimLab.grid(row=0, column=1, padx=2)

J7jogPosBut = Button(J7buttonFrame, text="+", width=3, style="Motion.TButton")
J7jogPosBut.grid(row=0, column=2, sticky="e", padx=(2, 0))

# Bind J7 events
def SelJ7jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J7jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(7, -1)
J7jogNegBut.bind("<ButtonPress-1>",SelJ7jogNeg)
J7jogNegBut.bind("<ButtonRelease-1>",StopJog)

def SelJ7jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J7jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(7, 1)
J7jogPosBut.bind("<ButtonPress-1>",SelJ7jogPos)
J7jogPosBut.bind("<ButtonRelease-1>",StopJog)

def J7sliderUpdate(event):
  J7slideLimLab.config(state="normal")
  J7slideLimLab.delete(0, END)
  J7slideLimLab.insert(0, round(float(J7jogslide.get()), 2))
  J7slideLimLab.config(state="readonly")
def J7sliderExecute(event):
  J7delta = float(J7jogslide.get()) - float(J7curAngEntryField.get())
  RUN['forceDisplayPosition'] = True
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

J8Lab = Label(J8jogFrame, font=ui.F(12), text="8th Axis")
J8Lab.grid(row=0, column=0, columnspan=3, pady=2)

J8negLimLab = Label(J8jogFrame, font=ui.F(8), text="0.00", style="Jointlim.TLabel")
J8negLimLab.grid(row=1, column=0, sticky="w", padx=2)
J8curAngEntryField = Entry(J8jogFrame, width=8, justify="center")
J8curAngEntryField.grid(row=1, column=1, padx=5)
J8posLimLab = Label(J8jogFrame, font=ui.F(8), text="0", style="Jointlim.TLabel")
J8posLimLab.grid(row=1, column=2, sticky="e", padx=2)

J8jogslide = Scale(J8jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120, bootstyle="warning")
J8jogslide.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

# Create frame for button layout (- entry +)
J8buttonFrame = Frame(J8jogFrame)
J8buttonFrame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

J8buttonFrame.grid_columnconfigure(0, weight=1)
J8buttonFrame.grid_columnconfigure(1, weight=0)
J8buttonFrame.grid_columnconfigure(2, weight=1)

J8jogNegBut = Button(J8buttonFrame, text="\u2212", width=3, style="Motion.TButton")
J8jogNegBut.grid(row=0, column=0, sticky="w", padx=(0, 2))

J8slideLimLab = Entry(J8buttonFrame, width=8, justify="center", state="readonly")
J8slideLimLab.grid(row=0, column=1, padx=2)

J8jogPosBut = Button(J8buttonFrame, text="+", width=3, style="Motion.TButton")
J8jogPosBut.grid(row=0, column=2, sticky="e", padx=(2, 0))

# Bind J8 events
def SelJ8jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J8jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(8, -1)
J8jogNegBut.bind("<ButtonPress-1>",SelJ8jogNeg)
J8jogNegBut.bind("<ButtonRelease-1>",StopJog)

def SelJ8jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J8jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(8, 1)
J8jogPosBut.bind("<ButtonPress-1>",SelJ8jogPos)
J8jogPosBut.bind("<ButtonRelease-1>",StopJog)

def J8sliderUpdate(event):
  J8slideLimLab.config(state="normal")
  J8slideLimLab.delete(0, END)
  J8slideLimLab.insert(0, round(float(J8jogslide.get()), 2))
  J8slideLimLab.config(state="readonly")
def J8sliderExecute(event):
  J8delta = float(J8jogslide.get()) - float(J8curAngEntryField.get())
  RUN['forceDisplayPosition'] = True
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

J9Lab = Label(J9jogFrame, font=ui.F(12), text="9th Axis")
J9Lab.grid(row=0, column=0, columnspan=3, pady=2)

J9negLimLab = Label(J9jogFrame, font=ui.F(8), text="0.00", style="Jointlim.TLabel")
J9negLimLab.grid(row=1, column=0, sticky="w", padx=2)
J9curAngEntryField = Entry(J9jogFrame, width=8, justify="center")
J9curAngEntryField.grid(row=1, column=1, padx=5)
J9posLimLab = Label(J9jogFrame, font=ui.F(8), text="0", style="Jointlim.TLabel")
J9posLimLab.grid(row=1, column=2, sticky="e", padx=2)

J9jogslide = Scale(J9jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120, bootstyle="warning")
J9jogslide.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

# Create frame for button layout (- entry +)
J9buttonFrame = Frame(J9jogFrame)
J9buttonFrame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

J9buttonFrame.grid_columnconfigure(0, weight=1)
J9buttonFrame.grid_columnconfigure(1, weight=0)
J9buttonFrame.grid_columnconfigure(2, weight=1)

J9jogNegBut = Button(J9buttonFrame, text="\u2212", width=3, style="Motion.TButton")
J9jogNegBut.grid(row=0, column=0, sticky="w", padx=(0, 2))

J9slideLimLab = Entry(J9buttonFrame, width=8, justify="center", state="readonly")
J9slideLimLab.grid(row=0, column=1, padx=2)

J9jogPosBut = Button(J9buttonFrame, text="+", width=3, style="Motion.TButton")
J9jogPosBut.grid(row=0, column=2, sticky="e", padx=(2, 0))

# Bind J9 events

def SelJ9jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J9jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(9, -1)
J9jogNegBut.bind("<ButtonPress-1>",SelJ9jogNeg)
J9jogNegBut.bind("<ButtonRelease-1>",StopJog)

def SelJ9jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J9jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(9, 1)
J9jogPosBut.bind("<ButtonPress-1>",SelJ9jogPos)
J9jogPosBut.bind("<ButtonRelease-1>",StopJog)

def J9sliderUpdate(event):
  J9slideLimLab.config(state="normal")
  J9slideLimLab.delete(0, END)
  J9slideLimLab.insert(0, round(float(J9jogslide.get()), 2))
  J9slideLimLab.config(state="readonly")
def J9sliderExecute(event):
  J9delta = float(J9jogslide.get()) - float(J9curAngEntryField.get())
  RUN['forceDisplayPosition'] = True
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
Label(cmdFrame, text="IF", font=ui.F(10, "bold")).grid(row=0, column=0, sticky="w", padx=(2, 5), pady=2)

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
Label(cmdFrame, text="SET", font=ui.F(10, "bold")).grid(row=1, column=0, sticky="w", padx=(2, 5), pady=2)

setmenu = OptionMenu(cmdFrame, setoption, "5v Output", "5v Output", "MB Coil", "MB Hold Reg")
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
Label(cmdFrame, text="WAIT", font=ui.F(10, "bold")).grid(row=2, column=0, sticky="w", padx=(2, 5), pady=2)

waitmenu = OptionMenu(cmdFrame, waitoption, "5v Input", "5v Input", "MB Coil", "MB Hold Reg")
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
Label(regFrame, text="Register", font=ui.F(8)).grid(row=0, column=1, sticky="w", padx=2)
Label(regFrame, text="(++/--)", font=ui.F(8)).grid(row=0, column=2, sticky="w", padx=2)
Label(regFrame, text="Pos Reg", font=ui.F(8)).grid(row=0, column=4, sticky="w", padx=2)
Label(regFrame, text="Element", font=ui.F(8)).grid(row=0, column=5, sticky="w", padx=2)
Label(regFrame, text="(++/--)", font=ui.F(8)).grid(row=0, column=6, sticky="w", padx=2)

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
Label(devFrame, text="Number", font=ui.F(8)).grid(row=0, column=1, sticky="w", padx=2)
Label(devFrame, text="Position", font=ui.F(8)).grid(row=0, column=2, sticky="w", padx=2)
Label(devFrame, text="Port", font=ui.F(8)).grid(row=0, column=4, sticky="w", padx=2)
Label(devFrame, text="Char", font=ui.F(8)).grid(row=0, column=5, sticky="w", padx=2)

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
tab2.grid_columnconfigure(2, weight=0, minsize=180)  # Calibration Offsets
tab2.grid_columnconfigure(3, weight=0, minsize=180)  # Encoder Control
tab2.grid_columnconfigure(4, weight=0, minsize=180)  # Theme
tab2.grid_columnconfigure(5, weight=0, minsize=180)  # Virtual Import
tab2.grid_columnconfigure(6, weight=0, minsize=180)  # Save
tab2.grid_columnconfigure(7, weight=1)  # Spacer (expands)

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
def _usable_serial_ports():
  """Serial ports worth offering. On Linux pyserial also lists the legacy /dev/ttyS0..31
  UART nodes, which exist whether or not anything is wired to them, so the menu showed
  30+ dead entries. Keep ports that belong to a real device (USB VID/PID, /dev/ttyUSB*,
  /dev/ttyACM*, /dev/serial/by-id) and put the Teensy (VID 0x16C0) first."""
  from serial.tools import list_ports
  ports = list(list_ports.comports())
  if not IS_WINDOWS:
    def real(p):
      dev = p.device or ""
      if p.vid is not None:
        return True
      if "/ttyUSB" in dev or "/ttyACM" in dev or "/serial/by-id/" in dev:
        return True
      return not re.search(r"/ttyS\d+$", dev)    # anything else that is not a legacy UART
    ports = [p for p in ports if real(p)]
  ports.sort(key=lambda p: (0 if p.vid == 0x16C0 else 1, p.device or ""))
  return ports


def detect_ports():
  ports = _usable_serial_ports()
  choices = [p.device for p in ports]
  choices.insert(0, "None")
  if globals().get('CAL', {}).get('comPort') not in ("", None, "None"):
    port1_default = CAL['comPort']
  else:
    port1_default = "None"
  
  if globals().get('CAL', {}).get('com2Port')not in ("", None, "None"):
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
themeFrame.grid(row=1, column=4, sticky="nsew", padx=5, pady=5)

themeFrame.grid_columnconfigure(0, weight=1)
themeFrame.grid_columnconfigure(1, weight=1)

# Theme buttons
lightBut = Button(themeFrame, text="  Light  ", command=lightTheme)
lightBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

darkBut = Button(themeFrame, text="  Dark   ", command=darkTheme)
darkBut.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

# Robot Color label and dropdown
robotColorLab = Label(themeFrame, text="Robot Color", font=ui.F(10, "bold"))
robotColorLab.grid(row=1, column=0, columnspan=2, sticky="w", padx=2, pady=(10, 2))

main_color_dropdown = ttk.OptionMenu(themeFrame, main_color_var, main_color_var.get(), *color_options, command=update_main_color)
main_color_dropdown.grid(row=2, column=0, columnspan=2, sticky="ew", padx=2, pady=2)







# ============================================================================
# ROW 1, COLUMN 6: Virtual Import Frame
# ============================================================================
virtualImportFrame = LabelFrame(tab2, text="Virtual Import", padding=10)
virtualImportFrame.grid(row=1, column=5, sticky="nsew", padx=5, pady=5)

virtualImportFrame.grid_columnconfigure(0, weight=1)

# Import 3D button
importSTLBut = ttk.Button(virtualImportFrame, text="Import 3D File", command=lambda: import_3d_file(False))
importSTLBut.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

# Imported CAD List
fileNameLab = Label(virtualImportFrame, text="Imported CAD")
fileNameLab.grid(row=1, column=0, sticky="w", padx=5, pady=(10, 2))

importedCadList = Listbox(virtualImportFrame, height=6, exportselection=False)
ui.style_listbox(importedCadList, mono=False)
importedCadList.grid(row=2, column=0, sticky="nsew", padx=5, pady=2)
importedCadList.bind("<<ListboxSelect>>", on_imported_cad_selected)

# X Position
xPosLab = Label(virtualImportFrame, text="X Position")
xPosLab.grid(row=3, column=0, sticky="w", padx=5, pady=(5, 1))
x_entry = Entry(virtualImportFrame, textvariable=x_var, width=10)
x_entry.grid(row=4, column=0, sticky="w", padx=5, pady=(0, 1))

# Y Position
yPosLab = Label(virtualImportFrame, text="Y Position")
yPosLab.grid(row=5, column=0, sticky="w", padx=5, pady=(5, 1))
y_entry = Entry(virtualImportFrame, textvariable=y_var, width=10)
y_entry.grid(row=6, column=0, sticky="w", padx=5, pady=(0, 1))

# Z Position
zPosLab = Label(virtualImportFrame, text="Z Position")
zPosLab.grid(row=7, column=0, sticky="w", padx=5, pady=(5, 1))
z_entry = Entry(virtualImportFrame, textvariable=z_var, width=10)
z_entry.grid(row=8, column=0, sticky="w", padx=5, pady=(0, 1))

# X Rotation
xRotLab = Label(virtualImportFrame, text="X Rotation")
xRotLab.grid(row=9, column=0, sticky="w", padx=5, pady=(5, 1))
rx_entry = Entry(virtualImportFrame, textvariable=rx_var, width=10)
rx_entry.grid(row=10, column=0, sticky="w", padx=5, pady=(0, 1))

# Y Rotation
yRotLab = Label(virtualImportFrame, text="Y Rotation")
yRotLab.grid(row=11, column=0, sticky="w", padx=5, pady=(5, 1))
ry_entry = Entry(virtualImportFrame, textvariable=ry_var, width=10)
ry_entry.grid(row=12, column=0, sticky="w", padx=5, pady=(0, 1))

# Z Rotation
zRotLab = Label(virtualImportFrame, text="Z Rotation")
zRotLab.grid(row=13, column=0, sticky="w", padx=5, pady=(5, 1))
rz_entry = Entry(virtualImportFrame, textvariable=rz_var, width=10)
rz_entry.grid(row=14, column=0, sticky="w", padx=5, pady=(0, 1))

# Update Position button
updatePosBut = ttk.Button(virtualImportFrame, text="Update Position", command=update_stl_transform)
updatePosBut.grid(row=15, column=0, sticky="ew", padx=5, pady=(5, 4))

# Delete CAD button
delete_cad_btn = ttk.Button(virtualImportFrame, text="Delete CAD", command=delete_selected_imported_cad)
delete_cad_btn.grid(row=16, column=0, sticky="ew", padx=5, pady=(0, 4))

# Attach button
attachBtn = ttk.Button(
    virtualImportFrame,
    text="Attach",
    command=lambda: attachtogripper(importedCadList.get(importedCadList.curselection()[0])) if importedCadList.curselection() else logger.warning("No CAD object selected.")
)
attachBtn.grid(row=17, column=0, sticky="ew", padx=5, pady=(0, 4))

# Detach button
detachBtn = ttk.Button(
    virtualImportFrame,
    text="Detach",
    command=lambda: detachfromgripper(importedCadList.get(importedCadList.curselection()[0])) if importedCadList.curselection() else logger.warning("No CAD object selected.")
)
detachBtn.grid(row=18, column=0, sticky="ew", padx=5, pady=(0, 4))




# ============================================================================
# ROW 1, COLUMN 7: Tool Import Frame
# ============================================================================

toolImportFrame = LabelFrame(tab2, text="Tool Import", padding=10)
toolImportFrame.grid(row=1, column=6, sticky="nsew", padx=5, pady=5)

Tool_importSTLBut = ttk.Button(
    toolImportFrame,
    text="Import Tool",
    command=import_eoat_file
)
Tool_importSTLBut.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

toolNameLab = Label(toolImportFrame, text="Current Tool")
toolNameLab.grid(row=1, column=0, sticky="w", padx=5, pady=(10, 2))

tool_name_entry = Entry(
    toolImportFrame,
    textvariable=tool_name_var,
    width=20,
    state="readonly"
)
tool_name_entry.grid(row=2, column=0, sticky="ew", padx=5, pady=2)

toolXLab = Label(toolImportFrame, text="X Offset")
toolXLab.grid(row=3, column=0, sticky="w", padx=5, pady=(10, 2))
tool_x_entry = Entry(toolImportFrame, textvariable=tool_x_var, width=10)
tool_x_entry.grid(row=4, column=0, sticky="w", padx=5, pady=2)

toolYLab = Label(toolImportFrame, text="Y Offset")
toolYLab.grid(row=5, column=0, sticky="w", padx=5, pady=(10, 2))
tool_y_entry = Entry(toolImportFrame, textvariable=tool_y_var, width=10)
tool_y_entry.grid(row=6, column=0, sticky="w", padx=5, pady=2)

toolZLab = Label(toolImportFrame, text="Z Offset")
toolZLab.grid(row=7, column=0, sticky="w", padx=5, pady=(10, 2))
tool_z_entry = Entry(toolImportFrame, textvariable=tool_z_var, width=10)
tool_z_entry.grid(row=8, column=0, sticky="w", padx=5, pady=2)

toolRXLab = Label(toolImportFrame, text="X Rotation")
toolRXLab.grid(row=9, column=0, sticky="w", padx=5, pady=(10, 2))
tool_rx_entry = Entry(toolImportFrame, textvariable=tool_rx_var, width=10)
tool_rx_entry.grid(row=10, column=0, sticky="w", padx=5, pady=2)

toolRYLab = Label(toolImportFrame, text="Y Rotation")
toolRYLab.grid(row=11, column=0, sticky="w", padx=5, pady=(10, 2))
tool_ry_entry = Entry(toolImportFrame, textvariable=tool_ry_var, width=10)
tool_ry_entry.grid(row=12, column=0, sticky="w", padx=5, pady=2)

toolRZLab = Label(toolImportFrame, text="Z Rotation")
toolRZLab.grid(row=13, column=0, sticky="w", padx=5, pady=(10, 2))
tool_rz_entry = Entry(toolImportFrame, textvariable=tool_rz_var, width=10)
tool_rz_entry.grid(row=14, column=0, sticky="w", padx=5, pady=2)

def update_tool_transform():
    actor = RUN.get('tool_actor')

    if actor is None:
        messagebox.showwarning("No Tool Loaded", "Please import a tool first.")
        return

    try:
        x = float(tool_x_var.get())
        y = float(tool_y_var.get())
        z = float(tool_z_var.get())
        rx = float(tool_rx_var.get())
        ry = float(tool_ry_var.get())
        rz = float(tool_rz_var.get())
    except ValueError:
        messagebox.showerror("Invalid Entry", "Tool position values must be numbers.")
        return

    actor.SetPosition(
        x,
        y,
        z + RUN['EOAT_DEFAULT_Z_OFFSET']
    )

    actor.SetOrientation(
        rx + RUN['EOAT_DEFAULT_RX_OFFSET'],
        ry + RUN['EOAT_DEFAULT_RY_OFFSET'],
        rz + RUN['EOAT_DEFAULT_RZ_OFFSET']
    )

    RUN['render_window'].Render()
    save_eoat_scene()

updateToolBut = ttk.Button(
    toolImportFrame,
    text="Update Tool Position",
    command=update_tool_transform
)
updateToolBut.grid(row=15, column=0, sticky="ew", padx=5, pady=(10, 5))


def remove_tool_cad():
    actor = RUN.get('tool_actor')

    if actor is None:
        return

    assembly = RUN['assemblies']["Link 6-1.STL"]
    assembly.RemovePart(actor)

    RUN['tool_actor'] = None
    RUN['tool_filename'] = ""

    tool_name_var.set("")
    tool_x_var.set("0")
    tool_y_var.set("0")
    tool_z_var.set("0")
    tool_rx_var.set("0")
    tool_ry_var.set("0")
    tool_rz_var.set("0")

    RUN['render_window'].Render()
    save_eoat_scene()

removeToolBut = ttk.Button(
    toolImportFrame,
    text="Remove Tool",
    command=remove_tool_cad
)
removeToolBut.grid(row=16, column=0, sticky="ew", padx=5, pady=5)



# ============================================================================
# ROW 2, COLUMN 5: Save Frame (below and right of Commands)
# ============================================================================
saveFrame = LabelFrame(tab2, text="Save All", padding=10)
saveFrame.grid(row=2, column=5, rowspan=1, sticky="nsew", padx=5, pady=5)

saveFrame.grid_columnconfigure(0, weight=1)

# Save button
saveCalBut = Button(saveFrame, text="SAVE", width=26, command=SaveAndApplyCalibration)
saveCalBut.grid(row=0, column=0, padx=5, pady=5)




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



##############################################################################
####TAB 3
##############################################################################

# ============================================================================
# Tab 3 Grid Layout Configuration
# ============================================================================
tab3.grid_rowconfigure(0, weight=1)
tab3.grid_rowconfigure(1, weight=1)
tab3.grid_columnconfigure(0, weight=0, minsize=180)  # Motor Dir, Cal Dir
tab3.grid_columnconfigure(1, weight=0, minsize=180)  # Pos Limits, Cal High Low
tab3.grid_columnconfigure(2, weight=0, minsize=180)  # Drive MS, Steps Deg
tab3.grid_columnconfigure(3, weight=0, minsize=220)  # DH Parameters, Tool Frame
tab3.grid_columnconfigure(4, weight=0, minsize=280)  # External Axis Config
tab3.grid_columnconfigure(5, weight=0, minsize=280)  # Defaults
tab3.grid_columnconfigure(6, weight=0, minsize=280)  # Save
tab3.grid_columnconfigure(7, weight=1)  # Remaining .place() widgets

# ============================================================================
# Motor Direction Frame (Row 0, Column 0)
# ============================================================================
motorDirFrame = LabelFrame(tab3, text="Motor Direction", padding=10)
motorDirFrame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
motorDirFrame.grid_columnconfigure(0, weight=0)
motorDirFrame.grid_columnconfigure(1, weight=1)

J1MotDirLab_grid = Label(motorDirFrame, font=ui.F(8), text="J1 Motor Direction")
J1MotDirLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J1MotDirEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2MotDirLab_grid = Label(motorDirFrame, font=ui.F(8), text="J2 Motor Direction")
J2MotDirLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J2MotDirEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3MotDirLab_grid = Label(motorDirFrame, font=ui.F(8), text="J3 Motor Direction")
J3MotDirLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J3MotDirEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4MotDirLab_grid = Label(motorDirFrame, font=ui.F(8), text="J4 Motor Direction")
J4MotDirLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J4MotDirEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5MotDirLab_grid = Label(motorDirFrame, font=ui.F(8), text="J5 Motor Direction")
J5MotDirLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J5MotDirEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6MotDirLab_grid = Label(motorDirFrame, font=ui.F(8), text="J6 Motor Direction")
J6MotDirLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J6MotDirEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J7MotDirLab_grid = Label(motorDirFrame, font=ui.F(8), text="J7 Motor Direction")
J7MotDirLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J7MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J7MotDirEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J8MotDirLab_grid = Label(motorDirFrame, font=ui.F(8), text="J8 Motor Direction")
J8MotDirLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J8MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J8MotDirEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J9MotDirLab_grid = Label(motorDirFrame, font=ui.F(8), text="J9 Motor Direction")
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

J1CalDirLab_grid = Label(calDirFrame, font=ui.F(8), text="J1 Calibration Dir.")
J1CalDirLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J1CalDirEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2CalDirLab_grid = Label(calDirFrame, font=ui.F(8), text="J2 Calibration Dir.")
J2CalDirLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J2CalDirEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3CalDirLab_grid = Label(calDirFrame, font=ui.F(8), text="J3 Calibration Dir.")
J3CalDirLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J3CalDirEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4CalDirLab_grid = Label(calDirFrame, font=ui.F(8), text="J4 Calibration Dir.")
J4CalDirLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J4CalDirEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5CalDirLab_grid = Label(calDirFrame, font=ui.F(8), text="J5 Calibration Dir.")
J5CalDirLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J5CalDirEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6CalDirLab_grid = Label(calDirFrame, font=ui.F(8), text="J6 Calibration Dir.")
J6CalDirLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J6CalDirEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J7CalDirLab_grid = Label(calDirFrame, font=ui.F(8), text="J7 Calibration Dir.")
J7CalDirLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J7CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J7CalDirEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J8CalDirLab_grid = Label(calDirFrame, font=ui.F(8), text="J8 Calibration Dir.")
J8CalDirLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J8CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J8CalDirEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J9CalDirLab_grid = Label(calDirFrame, font=ui.F(8), text="J9 Calibration Dir.")
J9CalDirLab_grid.grid(row=8, column=0, sticky="w", padx=5, pady=2)
J9CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J9CalDirEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)


# ============================================================================
# Calibration Switch High | Low (Row 1, Column 1)
# ============================================================================
calHLFrame = LabelFrame(tab3, text="Calibration Switch - HIGH/LOW", padding=10)
calHLFrame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
calHLFrame.grid_columnconfigure(0, weight=0)
calHLFrame.grid_columnconfigure(1, weight=1)

J1CalHighLowLab_grid = Label(calHLFrame, font=ui.F(8), text="J1 Calibration Switch.")
J1CalHighLowLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1CalHighLowEntryField = Entry(calHLFrame, width=6, justify="center")
J1CalHighLowEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2CalHighLowLab_grid = Label(calHLFrame, font=ui.F(8), text="J2 Calibration Switch")
J2CalHighLowLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2CalHighLowEntryField = Entry(calHLFrame, width=6, justify="center")
J2CalHighLowEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3CalHighLowLab_grid = Label(calHLFrame, font=ui.F(8), text="J3 Calibration Switch")
J3CalHighLowLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3CalHighLowEntryField = Entry(calHLFrame, width=6, justify="center")
J3CalHighLowEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4CalHighLowLab_grid = Label(calHLFrame, font=ui.F(8), text="J4 Calibration Switch")
J4CalHighLowLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4CalHighLowEntryField = Entry(calHLFrame, width=6, justify="center")
J4CalHighLowEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5CalHighLowLab_grid = Label(calHLFrame, font=ui.F(8), text="J5 Calibration Switch")
J5CalHighLowLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5CalHighLowEntryField = Entry(calHLFrame, width=6, justify="center")
J5CalHighLowEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6CalHighLowLab_grid = Label(calHLFrame, font=ui.F(8), text="J6 Calibration Switch")
J6CalHighLowLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6CalHighLowEntryField = Entry(calHLFrame, width=6, justify="center")
J6CalHighLowEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J7CalHighLowLab_grid = Label(calHLFrame, font=ui.F(8), text="J7 Calibration Switch")
J7CalHighLowLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J7CalHighLowEntryField = Entry(calHLFrame, width=6, justify="center")
J7CalHighLowEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J8CalHighLowLab_grid = Label(calHLFrame, font=ui.F(8), text="J8 Calibration Switch")
J8CalHighLowLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J8CalHighLowEntryField = Entry(calHLFrame, width=6, justify="center")
J8CalHighLowEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J9CalHighLowLab_grid = Label(calHLFrame, font=ui.F(8), text="J9 Calibration Switch")
J9CalHighLowLab_grid.grid(row=8, column=0, sticky="w", padx=5, pady=2)
J9CalHighLowEntryField = Entry(calHLFrame, width=6, justify="center")
J9CalHighLowEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)



# ============================================================================
# Position Limits Frame (Row 0, Column 1)
# ============================================================================
posLimFrame = LabelFrame(tab3, text="Position Limits", padding=10)
posLimFrame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
posLimFrame.grid_columnconfigure(0, weight=0)
posLimFrame.grid_columnconfigure(1, weight=1)

J1PosLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J1 Pos Limit")
J1PosLimLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J1PosLimEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J1NegLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J1 Neg Limit")
J1NegLimLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J1NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J1NegLimEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J2PosLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J2 Pos Limit")
J2PosLimLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J2PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J2PosLimEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J2NegLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J2 Neg Limit")
J2NegLimLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J2NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J2NegLimEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J3PosLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J3 Pos Limit")
J3PosLimLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J3PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J3PosLimEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J3NegLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J3 Neg Limit")
J3NegLimLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J3NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J3NegLimEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J4PosLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J4 Pos Limit")
J4PosLimLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J4PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J4PosLimEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J4NegLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J4 Neg Limit")
J4NegLimLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J4NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J4NegLimEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J5PosLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J5 Pos Limit")
J5PosLimLab_grid.grid(row=8, column=0, sticky="w", padx=5, pady=2)
J5PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J5PosLimEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)

J5NegLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J5 Neg Limit")
J5NegLimLab_grid.grid(row=9, column=0, sticky="w", padx=5, pady=2)
J5NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J5NegLimEntryField.grid(row=9, column=1, sticky="w", padx=5, pady=2)

J6PosLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J6 Pos Limit")
J6PosLimLab_grid.grid(row=10, column=0, sticky="w", padx=5, pady=2)
J6PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J6PosLimEntryField.grid(row=10, column=1, sticky="w", padx=5, pady=2)

J6NegLimLab_grid = Label(posLimFrame, font=ui.F(8), text="J6 Neg Limit")
J6NegLimLab_grid.grid(row=11, column=0, sticky="w", padx=5, pady=2)
J6NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J6NegLimEntryField.grid(row=11, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Steps per Degree Frame (Row 1, Column 2)
# ============================================================================
stepDegFrame = LabelFrame(tab3, text="Steps per Degree", padding=10)
stepDegFrame.grid(row=1, column=2, sticky="nsew", padx=5, pady=5)
stepDegFrame.grid_columnconfigure(0, weight=0)
stepDegFrame.grid_columnconfigure(1, weight=1)

J1StepDegLab_grid = Label(stepDegFrame, font=ui.F(8), text="J1 Step/Deg")
J1StepDegLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J1StepDegEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2StepDegLab_grid = Label(stepDegFrame, font=ui.F(8), text="J2 Step/Deg")
J2StepDegLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J2StepDegEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3StepDegLab_grid = Label(stepDegFrame, font=ui.F(8), text="J3 Step/Deg")
J3StepDegLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J3StepDegEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4StepDegLab_grid = Label(stepDegFrame, font=ui.F(8), text="J4 Step/Deg")
J4StepDegLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J4StepDegEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5StepDegLab_grid = Label(stepDegFrame, font=ui.F(8), text="J5 Step/Deg")
J5StepDegLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J5StepDegEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6StepDegLab_grid = Label(stepDegFrame, font=ui.F(8), text="J6 Step/Deg")
J6StepDegLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J6StepDegEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Drive Microsteps Frame (Row 0, Column 2)
# ============================================================================
driveMSFrame = LabelFrame(tab3, text="Drive Microsteps", padding=10)
driveMSFrame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
driveMSFrame.grid_columnconfigure(0, weight=0)
driveMSFrame.grid_columnconfigure(1, weight=1)

J1DriveMSLab_grid = Label(driveMSFrame, font=ui.F(8), text="J1 Drive Microstep")
J1DriveMSLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J1DriveMSEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2DriveMSLab_grid = Label(driveMSFrame, font=ui.F(8), text="J2 Drive Microstep")
J2DriveMSLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J2DriveMSEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3DriveMSLab_grid = Label(driveMSFrame, font=ui.F(8), text="J3 Drive Microstep")
J3DriveMSLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J3DriveMSEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4DriveMSLab_grid = Label(driveMSFrame, font=ui.F(8), text="J4 Drive Microstep")
J4DriveMSLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J4DriveMSEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5DriveMSLab_grid = Label(driveMSFrame, font=ui.F(8), text="J5 Drive Microstep")
J5DriveMSLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J5DriveMSEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6DriveMSLab_grid = Label(driveMSFrame, font=ui.F(8), text="J6 Drive Microstep")
J6DriveMSLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J6DriveMSEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Encoder CPR Frame (Row 1, Column 3)
# ============================================================================
encCPRFrame = LabelFrame(tab3, text="Encoder CPR", padding=10)
encCPRFrame.grid(row=1, column=3, sticky="nsew", padx=5, pady=5)
encCPRFrame.grid_columnconfigure(0, weight=0)
encCPRFrame.grid_columnconfigure(1, weight=1)

J1EncCPRLab_grid = Label(encCPRFrame, font=ui.F(8), text="J1 Encoder CPR")
J1EncCPRLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J1EncCPREntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2EncCPRLab_grid = Label(encCPRFrame, font=ui.F(8), text="J2 Encoder CPR")
J2EncCPRLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J2EncCPREntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3EncCPRLab_grid = Label(encCPRFrame, font=ui.F(8), text="J3 Encoder CPR")
J3EncCPRLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J3EncCPREntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4EncCPRLab_grid = Label(encCPRFrame, font=ui.F(8), text="J4 Encoder CPR")
J4EncCPRLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J4EncCPREntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5EncCPRLab_grid = Label(encCPRFrame, font=ui.F(8), text="J5 Encoder CPR")
J5EncCPRLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J5EncCPREntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6EncCPRLab_grid = Label(encCPRFrame, font=ui.F(8), text="J6 Encoder CPR")
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
Label(dhParamsFrame, font=ui.F(8), text="").grid(row=0, column=0)
Label(dhParamsFrame, font=ui.F(8), text="DH-Θ").grid(row=0, column=1)
Label(dhParamsFrame, font=ui.F(8), text="DH-α").grid(row=0, column=2)
Label(dhParamsFrame, font=ui.F(8), text="DH-d").grid(row=0, column=3)
Label(dhParamsFrame, font=ui.F(8), text="DH-a").grid(row=0, column=4)

# J1 row
Label(dhParamsFrame, font=ui.F(8), text="J1").grid(row=1, column=0, sticky="w", padx=5, pady=2)
J1ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1ΘEntryField.grid(row=1, column=1, padx=2, pady=2)
J1αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1αEntryField.grid(row=1, column=2, padx=2, pady=1)
J1dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1dEntryField.grid(row=1, column=3, padx=2, pady=2)
J1aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1aEntryField.grid(row=1, column=4, padx=2, pady=2)

# J2 row
Label(dhParamsFrame, font=ui.F(8), text="J2").grid(row=2, column=0, sticky="w", padx=5, pady=2)
J2ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2ΘEntryField.grid(row=2, column=1, padx=2, pady=2)
J2αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2αEntryField.grid(row=2, column=2, padx=2, pady=1)
J2dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2dEntryField.grid(row=2, column=3, padx=2, pady=2)
J2aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2aEntryField.grid(row=2, column=4, padx=2, pady=2)

# J3 row
Label(dhParamsFrame, font=ui.F(8), text="J3").grid(row=3, column=0, sticky="w", padx=5, pady=2)
J3ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3ΘEntryField.grid(row=3, column=1, padx=2, pady=2)
J3αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3αEntryField.grid(row=3, column=2, padx=2, pady=1)
J3dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3dEntryField.grid(row=3, column=3, padx=2, pady=2)
J3aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3aEntryField.grid(row=3, column=4, padx=2, pady=2)

# J4 row
Label(dhParamsFrame, font=ui.F(8), text="J4").grid(row=4, column=0, sticky="w", padx=5, pady=2)
J4ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4ΘEntryField.grid(row=4, column=1, padx=2, pady=2)
J4αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4αEntryField.grid(row=4, column=2, padx=2, pady=1)
J4dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4dEntryField.grid(row=4, column=3, padx=2, pady=2)
J4aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4aEntryField.grid(row=4, column=4, padx=2, pady=2)

# J5 row
Label(dhParamsFrame, font=ui.F(8), text="J5").grid(row=5, column=0, sticky="w", padx=5, pady=2)
J5ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5ΘEntryField.grid(row=5, column=1, padx=2, pady=2)
J5αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5αEntryField.grid(row=5, column=2, padx=2, pady=1)
J5dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5dEntryField.grid(row=5, column=3, padx=2, pady=2)
J5aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5aEntryField.grid(row=5, column=4, padx=2, pady=2)

# J6 row
Label(dhParamsFrame, font=ui.F(8), text="J6").grid(row=6, column=0, sticky="w", padx=5, pady=2)
J6ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6ΘEntryField.grid(row=6, column=1, padx=2, pady=2)
J6αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6αEntryField.grid(row=6, column=2, padx=2, pady=1)
J6dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6dEntryField.grid(row=6, column=3, padx=2, pady=2)
J6aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6aEntryField.grid(row=6, column=4, padx=2, pady=2)



# ============================================================================
# ROW 1, COLUMN 4: External Axes Frame
# ============================================================================
externalAxesFrame = LabelFrame(tab3, text="External Axes", padding=10)
externalAxesFrame.grid(row=0, column=4, sticky="nsew", padx=5, pady=5)

externalAxesFrame.grid_columnconfigure(0, weight=0)  # Labels
externalAxesFrame.grid_columnconfigure(1, weight=1)  # Entry fields

# --- 7th Axis Calibration ---
axis7Lab = Label(externalAxesFrame, font=ui.F(10, "bold"), text="7th Axis Calibration")
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

axis7pinsetLab = Label(externalAxesFrame, font=ui.F(8), text="StepPin = 12 / DirPin = 13 / CalPin = 36")
axis7pinsetLab.grid(row=6, column=0, columnspan=2, sticky="w", padx=5, pady=(2, 15))

# --- 8th Axis Calibration ---
axis8Lab = Label(externalAxesFrame, font=ui.F(10, "bold"), text="8th Axis Calibration")
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

axis8pinsetLab = Label(externalAxesFrame, font=ui.F(8), text="StepPin = 32 / DirPin = 33 / CalPin = 37")
axis8pinsetLab.grid(row=13, column=0, columnspan=2, sticky="w", padx=5, pady=(2, 15))

# --- 9th Axis Calibration ---
axis9Lab = Label(externalAxesFrame, font=ui.F(10, "bold"), text="9th Axis Calibration")
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

axis9pinsetLab = Label(externalAxesFrame, font=ui.F(8), text="StepPin = 34 / DirPin = 35 / CalPin = 38")
axis9pinsetLab.grid(row=20, column=0, columnspan=2, sticky="w", padx=5, pady=(2, 5))


# ============================================================================
# Tool Frame Offset Frame (Row 1, Column 4)
# ============================================================================
toolFrameFrame = LabelFrame(tab3, text="Tool Frame Offset", padding=10)
toolFrameFrame.grid(row=1, column=4, sticky="nsew", padx=5, pady=5)

toolFrameFrame.grid_columnconfigure(0, weight=1)
toolFrameFrame.grid_columnconfigure(1, weight=1)
toolFrameFrame.grid_columnconfigure(2, weight=1)
toolFrameFrame.grid_columnconfigure(3, weight=1)
toolFrameFrame.grid_columnconfigure(4, weight=1)
toolFrameFrame.grid_columnconfigure(5, weight=1)

# Header row
Label(toolFrameFrame, font=ui.F(11), text="X").grid(row=0, column=0, padx=2, pady=2)
Label(toolFrameFrame, font=ui.F(11), text="Y").grid(row=0, column=1, padx=2, pady=2)
Label(toolFrameFrame, font=ui.F(11), text="Z").grid(row=0, column=2, padx=2, pady=1)
Label(toolFrameFrame, font=ui.F(11), text="Rz").grid(row=0, column=3, padx=2, pady=2)
Label(toolFrameFrame, font=ui.F(11), text="Ry").grid(row=0, column=4, padx=2, pady=2)
Label(toolFrameFrame, font=ui.F(11), text="Rx").grid(row=0, column=5, padx=2, pady=2)

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
# Save Frame (Row 1, Column 5)
# ============================================================================
SaveAllFrame = LabelFrame(tab3, text="Save All", padding=10)
SaveAllFrame.grid(row=1, column=5, rowspan=1, sticky="nsew", padx=5, pady=5)

SaveAllFrame.grid_columnconfigure(0, weight=1)

saveCalBut = Button(SaveAllFrame, text="SAVE", width=26, command=SaveAndApplyCalibration)
saveCalBut.grid(row=0, column=0, padx=5, pady=5) # 10 pixels above, 30 below
















# ============================================================================
# Defaults Frame (Row 0-1, Column 4)
# ============================================================================
defaultsFrame = LabelFrame(tab3, text="Defaults", padding=10)
defaultsFrame.grid(row=0, column=5, rowspan=1, sticky="nsew", padx=5, pady=5)

defaultsFrame.grid_columnconfigure(0, weight=1)

modelSelectLabel = Label(defaultsFrame, text="Robot Model:")
modelSelectLabel.grid(row=0, column=0, padx=5, pady=(5,0), sticky="w")

modelSelectVar = tk.StringVar(value="AR4-MK5")
modelSelectMenu = tk.OptionMenu(defaultsFrame, modelSelectVar, *VALID_MODELS)
modelSelectMenu.config(width=22)
modelSelectMenu.grid(row=1, column=0, padx=5, pady=5)

modelSelectStartupDone = False  # guard: don't load defaults during startup restore
def onModelSelectChanged(*args):
  if modelSelectStartupDone:
    LoadSelectedModelDefaults()
modelSelectVar.trace_add("write", onModelSelectChanged)




loadCustomBut = Button(defaultsFrame, text="Load Custom Calibration", width=26, command=load_custom_calibration)
loadCustomBut .grid(row=7, column=0, padx=5, pady=(30, 5))

saveCustomBut = Button(defaultsFrame, text="Save Custom Calibration", width=26, command=save_custom_calibration)
saveCustomBut.grid(row=8, column=0, padx=5, pady=5)

loadMaxStepBut = Button(defaultsFrame, text="Load Max Microsteps", width=26, command=LoadMaxdefault)
#loadMaxStepBut.grid(row=9, column=0, padx=5, pady=5)



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

MBwriteRegBut = Button(modbusFrame, text="Write Holding Register", width=30, command=MBwriteReg)
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

link2 = Label(infoFrame, font=ui.F(9), text="https://youtu.be/76F6dS4ar8Y?si=Z6NstZy1zNeHgtCF", style="Link.TLabel", cursor="hand2")
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
# IOboardLab = Label(tab4, font=ui.F(10, "bold"), text = "5v IO BOARD")
# IOboardLab.place(x=95, y=10)
# 
# AuxComLab = Label(tab4, font=ui.F(10, "bold"), text = "AUX COM DEVICE")
# AuxComLab.place(x=400, y=10)
# 
# ModbusLab = Label(tab4, font=ui.F(10, "bold"), text = "MODBUS DEVICE")
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
# link2 = Label(tab4, font=ui.F(8), text="https://youtu.be/76F6dS4ar8Y?si=Z6NstZy1zNeHgtCF", foreground="blue", cursor="hand2")
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




### 6 LAYOUT ################################################################
# Two columns: left = controls strip + camera view, right = values, object
# settings and the live feed. Every group sits in its own LabelFrame, laid out
# with grid like the other tabs.
#############################################################################

tab6.grid_columnconfigure(0, weight=0)
tab6.grid_columnconfigure(1, weight=0)
tab6.grid_columnconfigure(2, weight=1)   # spare width goes to the right edge
tab6.grid_rowconfigure(1, weight=1)

visLeft = Frame(tab6)
visLeft.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=(10, 5), pady=10)
visRight = Frame(tab6)
visRight.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=(5, 10), pady=10)

# ---------------------------------------------------------------------------
# Left / top: Camera + Image Adjust
# ---------------------------------------------------------------------------
visTopStrip = Frame(visLeft)
visTopStrip.grid(row=0, column=0, sticky="ew")
visTopStrip.grid_columnconfigure(1, weight=1)

camFrame = LabelFrame(visTopStrip, text="Camera", padding=10)
camFrame.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 5))
camFrame.grid_columnconfigure((0, 1, 2), weight=1, uniform="cam")

adjFrame = LabelFrame(visTopStrip, text="Image Adjust", padding=10)
adjFrame.grid(row=0, column=1, sticky="nsew", padx=(5, 0), pady=(0, 5))
adjFrame.grid_columnconfigure(1, weight=1)

# ---------------------------------------------------------------------------
# Left / bottom: Camera View (backdrop with the video drawn on top of it)
# ---------------------------------------------------------------------------
viewFrame = LabelFrame(visLeft, text="Camera View", padding=8)
viewFrame.grid(row=1, column=0, sticky="nw", pady=(5, 0))

viewHolder = Frame(viewFrame, width=690, height=530)
viewHolder.grid(row=0, column=0)
viewHolder.grid_propagate(False)

VisBackdropImg = Image.open("VisBackdrop.png")
VisBackdropTk = ImageTk.PhotoImage(VisBackdropImg)
VisBackdromLbl = Label(viewHolder, image = VisBackdropTk, borderwidth=0, padding=0)
VisBackdromLbl.place(x=0, y=0)

# video sits 35px in from the backdrop's origin, as before
video_frame = Frame(viewHolder,width=640,height=480)
video_frame.place(x=35, y=35)

vid_lbl = Label(video_frame)
vid_lbl.place(x=0, y=0)
vid_lbl.bind('<Button-1>', motion)

# ---------------------------------------------------------------------------
# Right column groups
# ---------------------------------------------------------------------------
visValuesRow = Frame(visRight)
visValuesRow.grid(row=0, column=0, sticky="ew")
visValuesRow.grid_columnconfigure((0, 1), weight=1)

foundFrame = LabelFrame(visValuesRow, text="Found Values", padding=10)
foundFrame.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(0, 5))
foundFrame.grid_columnconfigure(0, weight=1)

calVisFrame = LabelFrame(visValuesRow, text="Calibration Values", padding=10)
calVisFrame.grid(row=0, column=1, sticky="nsew", padx=(5, 0), pady=(0, 5))
calVisFrame.grid_columnconfigure((0, 2), weight=1)

visObjFrame = LabelFrame(visRight, text="Object & Search", padding=10)
visObjFrame.grid(row=1, column=0, sticky="ew", pady=5)
visObjFrame.grid_columnconfigure(1, weight=1)

liveFrame = LabelFrame(visRight, text="Live Video Feed", padding=8)
liveFrame.grid(row=2, column=0, sticky="nw", pady=(5, 0))

liveCanvas = Canvas(liveFrame, width=490, height=330, highlightthickness=0)
liveCanvas.grid(row=0, column=0)
live_frame = Frame(liveFrame,width=480,height=320)
live_frame.place(in_=liveCanvas, x=5, y=5)
live_lbl = Label(live_frame)
live_lbl.place(x=0, y=0)

# kept for compatibility with any code that references the old header labels
LiveLab = liveFrame
FoundValuesLab = foundFrame
CalValuesLab = calVisFrame


### 6 CAMERA ################################################################
#############################################################################

match CE['Platform']['OS']:

  case "Windows":
    try:
      graph = FilterGraph()
      allcams = graph.get_input_devices()
    except Exception:
      allcams = ["Select a Camera"]

    # --- filter out virtual/fake cameras (case-insensitive) ---
    _ban = ("iriun", "droidcam", "obs", "virtual", "manycam", "snap camera", "ndi", "xsplit", "mmhmm")
    camList = [n for n in allcams if not any(b in n.lower() for b in _ban)]
    if not camList:
      camList = allcams[:]  # fallback if everything got filtered
    # Index in the unfiltered list is the DirectShow device number OpenCV expects
    CAMERA_LABEL_TO_ID.update({name: i for i, name in enumerate(allcams)})
    # ----------------------------------------------------------

    visoptions = StringVar(tab6)
    visoptions.set("Select a Camera")

    try:
      # If we have real cams, preselect the first real one; otherwise keep the placeholder
      if camList and camList[0] != "Select a Camera":
        vismenu = OptionMenu(camFrame, visoptions, camList[0], *camList)
        visoptions.set(camList[0])  # ensures the real cam (e.g., Logi C270) is selected
      else:
        vismenu = OptionMenu(camFrame, visoptions, "Select a Camera")
      vismenu.config(width=20)
      vismenu.grid(row=0, column=0, columnspan=3, sticky="ew", padx=3, pady=(0, 6))
    except Exception:
      logger.error("no camera")

  case _:
    # Linux (and any other non-Windows OS): camera list from AR4config, else probe /dev/video*
    label_to_id = {}
    try:
      label_to_id = {c.label: c.id for c in CE['Cameras']['Enum']}
    except Exception as e:
      logger.warning(f"Camera enumeration from config failed: {e}")
    if not label_to_id:
      for _i in range(10):
        _dev = f"/dev/video{_i}"
        if os.path.exists(_dev):
          label_to_id[_dev] = _dev
    CAMERA_LABEL_TO_ID.update(label_to_id)
    camList = list(label_to_id.keys()) or ["Select a Camera"]

    selected_label = tk.StringVar(value=camList[0])

    visoptions = StringVar(tab6)
    visoptions.set(camList[0])

    def on_camera_select(chosen_label):
      cam_id = label_to_id.get(chosen_label, "None")
      logger.debug("User picked camera %s -> using id %s", chosen_label, cam_id)

    try:
      vismenu = OptionMenu(camFrame, visoptions, selected_label.get(), *camList, command=on_camera_select)
      vismenu.config(width=20)
      vismenu.grid(row=0, column=0, columnspan=3, sticky="ew", padx=3, pady=(0, 6))
    except Exception:
      logger.error("no camera")


StartCamBut = Button(camFrame,  text="Start Camera", command = start_vid)
StartCamBut.grid(row=1, column=0, columnspan=1, sticky="ew", padx=3, pady=3)

StopCamBut = Button(camFrame,  text="Stop Camera", command = stop_vid)
StopCamBut.grid(row=1, column=1, columnspan=2, sticky="ew", padx=3, pady=3)

CapImgBut = Button(camFrame,  text="Snap Image", command = take_pic)
CapImgBut.grid(row=2, column=0, sticky="ew", padx=3, pady=3)

TeachImgBut = Button(camFrame,  text="Teach Object", command = selectTemplate)
TeachImgBut.grid(row=2, column=1, sticky="ew", padx=3, pady=3)

FindVisBut = Button(camFrame,  text="Snap & Find", command = snapFind)
FindVisBut.grid(row=2, column=2, sticky="ew", padx=3, pady=3)


### 6 IMAGE ADJUST ##########################################################
#############################################################################

VisZoomLab = Label(adjFrame, text = "Zoom")
VisZoomLab.grid(row=0, column=0, sticky="w", padx=(3, 8), pady=4)
VisZoomSlide = Scale(adjFrame, from_=50, to=1,  length=180, orient=HORIZONTAL)
VisZoomSlide.bind("<ButtonRelease-1>", VisUpdateBriCon)
VisZoomSlide.grid(row=0, column=1, sticky="ew", pady=4)
VisZoomSlide.set(50)

VisBrightLab = Label(adjFrame, text = "Brightness")
VisBrightLab.grid(row=1, column=0, sticky="w", padx=(3, 8), pady=4)
VisBrightSlide = Scale(adjFrame, from_=-127, to=127,  length=180, orient=HORIZONTAL)
VisBrightSlide.bind("<ButtonRelease-1>", VisUpdateBriCon)
VisBrightSlide.grid(row=1, column=1, sticky="ew", pady=4)

VisContrastLab = Label(adjFrame, text = "Contrast")
VisContrastLab.grid(row=2, column=0, sticky="w", padx=(3, 8), pady=4)
VisContrastSlide = Scale(adjFrame, from_=-127, to=127,  length=180, orient=HORIZONTAL)
VisContrastSlide.bind("<ButtonRelease-1>", VisUpdateBriCon)
VisContrastSlide.grid(row=2, column=1, sticky="ew", pady=4)

adjBtnRow = Frame(adjFrame)
adjBtnRow.grid(row=0, column=2, rowspan=3, sticky="ns", padx=(10, 0))

ZeroBrCnBut = Button(adjBtnRow, text="Zero",  width=6, command = zeroBrCn)
ZeroBrCnBut.pack(fill=X, pady=(4, 3))

maskBut = Button(adjBtnRow, text="Mask",  width=6, command = selectMask)
maskBut.pack(fill=X, pady=3)


### 6 OBJECT & SEARCH #######################################################
#############################################################################

VisSelObjLab = Label(visObjFrame, text = "Select Object")
VisSelObjLab.grid(row=0, column=0, sticky="w", padx=(3, 8), pady=3)
# the object combobox (Visoptmenu) is created by updateVisOp() at row 0, column 1

VisBacColorLab = Label(visObjFrame, text = "Background Color")
VisBacColorLab.grid(row=1, column=0, sticky="w", padx=(3, 8), pady=3)
bgColorRow = Frame(visObjFrame)
bgColorRow.grid(row=1, column=1, sticky="w", pady=3)
VisBacColorEntryField = Entry(bgColorRow,width=10,justify="center")
VisBacColorEntryField.pack(side=LEFT)
bgAutoCbut = Checkbutton(bgColorRow, command=checkAutoBG, text="Auto",variable = RUN['autoBG'])
bgAutoCbut.pack(side=LEFT, padx=(8, 0))

VisScoreLab = Label(visObjFrame, text = "Score Threshold")
VisScoreLab.grid(row=2, column=0, sticky="w", padx=(3, 8), pady=3)
VisScoreEntryField = Entry(visObjFrame,width=10,justify="center")
VisScoreEntryField.grid(row=2, column=1, sticky="w", pady=3)

fullRotCbut = Checkbutton(visObjFrame, text="Full Rotation Search",variable = RUN['fullRot'])
fullRotCbut.grid(row=3, column=0, columnspan=2, sticky="w", padx=3, pady=(8, 2))

pick180Cbut = Checkbutton(visObjFrame, text="Pick Closest 180°",variable = RUN['pick180'])
pick180Cbut.grid(row=4, column=0, columnspan=2, sticky="w", padx=3, pady=2)

pickClosestCbut = Checkbutton(visObjFrame, text="Try Closest When Out of Range",variable = RUN['pickClosest'])
pickClosestCbut.grid(row=5, column=0, columnspan=2, sticky="w", padx=3, pady=2)

# taught-object preview
template_frame = Frame(visObjFrame,width=150,height=150)
template_frame.grid(row=0, column=2, rowspan=6, sticky="ne", padx=(10, 0))
template_frame.grid_propagate(False)
template_lbl = Label(template_frame)
template_lbl.place(x=0, y=0)


### 6 FOUND VALUES ##########################################################
#############################################################################

def _vis_value_row(parent, row, text, col=0):
  lab = Label(parent, text=text)
  lab.grid(row=row, column=col, sticky="w", padx=(3, 8), pady=3)
  ent = Entry(parent, width=9, justify="center")
  ent.grid(row=row, column=col + 1, sticky="e", padx=(0, 3), pady=3)
  return ent, lab

VisRetScoreEntryField, VisRetScoreLab = _vis_value_row(foundFrame, 0, "Scored Value")
VisRetAngleEntryField, VisRetAngleLab = _vis_value_row(foundFrame, 1, "Found Angle")
VisRetXpixEntryField,  VisRetXpixLab  = _vis_value_row(foundFrame, 2, "Pixel X Position")
VisRetYpixEntryField,  VisRetYpixLab  = _vis_value_row(foundFrame, 3, "Pixel Y Position")
VisRetXrobEntryField,  VisRetXrobLab  = _vis_value_row(foundFrame, 4, "Robot X Position")
VisRetYrobEntryField,  VisRetYrobLab  = _vis_value_row(foundFrame, 5, "Robot Y Position")


### 6 CALIBRATION VALUES ####################################################
#############################################################################

Label(calVisFrame, text="PIXEL").grid(row=0, column=0, columnspan=2, sticky="w", padx=3)
Label(calVisFrame, text="ROBOT").grid(row=0, column=2, columnspan=2, sticky="w", padx=(12, 3))

VisX1PixEntryField, VisX1PixLab = _vis_value_row(calVisFrame, 1, "X1")
VisY1PixEntryField, VisY1PixLab = _vis_value_row(calVisFrame, 2, "Y1")
VisX2PixEntryField, VisX2PixLab = _vis_value_row(calVisFrame, 3, "X2")
VisY2PixEntryField, VisY2PixLab = _vis_value_row(calVisFrame, 4, "Y2")

VisX1RobEntryField, VisX1RobLab = _vis_value_row(calVisFrame, 1, "X1", col=2)
VisY1RobEntryField, VisY1RobLab = _vis_value_row(calVisFrame, 2, "Y1", col=2)
VisX2RobEntryField, VisX2RobLab = _vis_value_row(calVisFrame, 3, "X2", col=2)
VisY2RobEntryField, VisY2RobLab = _vis_value_row(calVisFrame, 4, "Y2", col=2)
for _lab in (VisX1RobLab, VisY1RobLab, VisX2RobLab, VisY2RobLab):
  _lab.grid_configure(padx=(12, 8))

saveCalBut = Button(calVisFrame,  text="SAVE VISION DATA", command = SaveAndApplyCalibration)
saveCalBut.grid(row=5, column=0, columnspan=4, sticky="ew", padx=3, pady=(10, 0))





####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 7
# Left column = grouped controls in LabelFrames, right = program viewer that
# fills the remaining space. Laid out with grid like the other tabs.

tab7.grid_columnconfigure(0, weight=0)
tab7.grid_columnconfigure(1, weight=1)
tab7.grid_rowconfigure(0, weight=1)

gcLeft = Frame(tab7)
gcLeft.grid(row=0, column=0, sticky="nsew", padx=(10, 5), pady=10)
gcLeft.grid_columnconfigure(0, weight=1)

# ---------------------------------------------------------------------------
# Program
# ---------------------------------------------------------------------------
gcProgFrame = LabelFrame(gcLeft, text="Program", padding=10)
gcProgFrame.grid(row=0, column=0, sticky="ew", pady=(0, 5))
gcProgFrame.grid_columnconfigure(0, weight=1)

LoadGcodeBut = Button(gcProgFrame,  text="Load Program", command = loadGcodeProg)
LoadGcodeBut.grid(row=0, column=0, sticky="ew", padx=3, pady=(0, 6))

GcodeProgEntryField = Entry(gcProgFrame,width=40,justify="center")
GcodeProgEntryField.grid(row=1, column=0, sticky="ew", padx=3, pady=3)

# ---------------------------------------------------------------------------
# Start Position + Offset
# ---------------------------------------------------------------------------
gcStartFrame = LabelFrame(gcLeft, text="Start Position", padding=10)
gcStartFrame.grid(row=1, column=0, sticky="ew", pady=5)
for _c in range(7):
  gcStartFrame.grid_columnconfigure(_c, weight=1, uniform="gcst")

for _c, _t in enumerate(("X", "Y", "Z", "Rz", "Ry", "Rx", "WC")):
  Label(gcStartFrame, text=_t, anchor="center").grid(row=0, column=_c, sticky="ew", padx=2)

GC_ST_E1_EntryField = Entry(gcStartFrame,width=6,justify="center")
GC_ST_E1_EntryField.grid(row=1, column=0, sticky="ew", padx=2, pady=(2, 6))
GC_ST_E2_EntryField = Entry(gcStartFrame,width=6,justify="center")
GC_ST_E2_EntryField.grid(row=1, column=1, sticky="ew", padx=2, pady=(2, 6))
GC_ST_E3_EntryField = Entry(gcStartFrame,width=6,justify="center")
GC_ST_E3_EntryField.grid(row=1, column=2, sticky="ew", padx=2, pady=(2, 6))
GC_ST_E4_EntryField = Entry(gcStartFrame,width=6,justify="center")
GC_ST_E4_EntryField.grid(row=1, column=3, sticky="ew", padx=2, pady=(2, 6))
GC_ST_E5_EntryField = Entry(gcStartFrame,width=6,justify="center")
GC_ST_E5_EntryField.grid(row=1, column=4, sticky="ew", padx=2, pady=(2, 6))
GC_ST_E6_EntryField = Entry(gcStartFrame,width=6,justify="center")
GC_ST_E6_EntryField.grid(row=1, column=5, sticky="ew", padx=2, pady=(2, 6))
GC_ST_WC_EntryField = Entry(gcStartFrame,width=4,justify="center")
GC_ST_WC_EntryField.grid(row=1, column=6, sticky="ew", padx=2, pady=(2, 6))

GcodeStartPosBut = Button(gcStartFrame,  text="Set Start Position", command = SetGcodeStartPos)
GcodeStartPosBut.grid(row=2, column=0, columnspan=7, sticky="ew", padx=2, pady=(0, 2))

gcOffFrame = LabelFrame(gcLeft, text="Start Position Offset", padding=10)
gcOffFrame.grid(row=2, column=0, sticky="ew", pady=5)
for _c in range(7):
  gcOffFrame.grid_columnconfigure(_c, weight=1, uniform="gcst")

for _c, _t in enumerate(("X", "Y", "Z", "Rz", "Ry", "Rx")):
  Label(gcOffFrame, text=_t, anchor="center").grid(row=0, column=_c, sticky="ew", padx=2)

GC_SToff_E1_EntryField = Entry(gcOffFrame,width=6,justify="center")
GC_SToff_E1_EntryField.grid(row=1, column=0, sticky="ew", padx=2, pady=(2, 6))
GC_SToff_E2_EntryField = Entry(gcOffFrame,width=6,justify="center")
GC_SToff_E2_EntryField.grid(row=1, column=1, sticky="ew", padx=2, pady=(2, 6))
GC_SToff_E3_EntryField = Entry(gcOffFrame,width=6,justify="center")
GC_SToff_E3_EntryField.grid(row=1, column=2, sticky="ew", padx=2, pady=(2, 6))
GC_SToff_E4_EntryField = Entry(gcOffFrame,width=6,justify="center")
GC_SToff_E4_EntryField.grid(row=1, column=3, sticky="ew", padx=2, pady=(2, 6))
GC_SToff_E5_EntryField = Entry(gcOffFrame,width=6,justify="center")
GC_SToff_E5_EntryField.grid(row=1, column=4, sticky="ew", padx=2, pady=(2, 6))
GC_SToff_E6_EntryField = Entry(gcOffFrame,width=6,justify="center")
GC_SToff_E6_EntryField.grid(row=1, column=5, sticky="ew", padx=2, pady=(2, 6))

GcodeMoveStartPosBut = Button(gcOffFrame,  text="Move to Start Offset", command = MoveGcodeStartPos)
GcodeMoveStartPosBut.grid(row=2, column=0, columnspan=7, sticky="ew", padx=2, pady=(0, 2))

# ---------------------------------------------------------------------------
# SD Card
# ---------------------------------------------------------------------------
gcSDFrame = LabelFrame(gcLeft, text="SD Card", padding=10)
gcSDFrame.grid(row=3, column=0, sticky="ew", pady=5)
gcSDFrame.grid_columnconfigure((0, 1), weight=1, uniform="gcsd")

gcodeFilenameLab = Label(gcSDFrame, text = "Filename:")
gcodeFilenameLab.grid(row=0, column=0, columnspan=2, sticky="w", padx=3)
GcodeFilenameField = Entry(gcSDFrame,width=40,justify="center")
GcodeFilenameField.grid(row=1, column=0, columnspan=2, sticky="ew", padx=3, pady=(2, 8))

runGcodeBut = Button(gcSDFrame, text="Convert & Upload to SD", command = GCconvertProg)
runGcodeBut.grid(row=2, column=0, sticky="ew", padx=3, pady=3)

stopGcodeBut = Button(gcSDFrame, text="Stop Conversion & Upload", command = GCstopProg)
stopGcodeBut.grid(row=2, column=1, sticky="ew", padx=3, pady=3)

delGcodeBut = Button(gcSDFrame, text="Delete File from SD", command = GCdelete)
delGcodeBut.grid(row=3, column=0, sticky="ew", padx=3, pady=3)

readGcodeBut = Button(gcSDFrame, text="Read Files from SD", command = partial(GCread, "yes"))
readGcodeBut.grid(row=3, column=1, sticky="ew", padx=3, pady=3)

playGPhoto=PhotoImage(file="play-icon.png")
GCplayFileBut = Button(gcSDFrame, text="Play Gcode File", command = GCplay, image = playGPhoto, compound=LEFT)
GCplayFileBut.grid(row=4, column=0, columnspan=2, sticky="ew", padx=3, pady=(8, 0))

#revGcodeBut = Button(tab7,  text="REV ",  command = stepRev)
#fwdGcodeBut = Button(tab7,  text="FWD", command = GCstepFwd)

saveGCBut = Button(gcLeft,  text="SAVE DATA", command = SaveAndApplyCalibration)
saveGCBut.grid(row=4, column=0, sticky="ew", pady=(10, 0))

# ---------------------------------------------------------------------------
# Program viewer (right, fills the tab)
# ---------------------------------------------------------------------------
gcViewFrame = LabelFrame(tab7, text="Program View", padding=10)
gcViewFrame.grid(row=0, column=1, sticky="nsew", padx=(5, 10), pady=10)
gcViewFrame.grid_columnconfigure(0, weight=1)
gcViewFrame.grid_rowconfigure(1, weight=1)

gcViewHeader = Frame(gcViewFrame)
gcViewHeader.grid(row=0, column=0, sticky="ew", pady=(0, 6))
gcViewHeader.grid_columnconfigure(0, weight=1)

GCalmStatusLab = Label(gcViewHeader, text = "GCODE IDLE", style="OK.TLabel")
GCalmStatusLab.grid(row=0, column=0, sticky="w")

gcodeCurRowLab = Label(gcViewHeader, text = "Current Row:")
gcodeCurRowLab.grid(row=0, column=1, sticky="e", padx=(0, 6))
GcodCurRowEntryField = Entry(gcViewHeader,width=10,justify="center")
GcodCurRowEntryField.grid(row=0, column=2, sticky="e")

gcodeframe=Frame(gcViewFrame)
gcodeframe.grid(row=1, column=0, sticky="nsew")
gcodescrollbar = Scrollbar(gcodeframe)
gcodescrollbar.pack(side=RIGHT, fill=Y)
tab7.gcodeView = Listbox(gcodeframe,exportselection=0,width=80,height=30, yscrollcommand=gcodescrollbar.set)
ui.style_listbox(tab7.gcodeView)
tab7.gcodeView.bind('<<ListboxSelect>>', gcodeViewselect)
tab7.gcodeView.pack(side=LEFT, fill=BOTH, expand=True)
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







####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 8

Elogframe=Frame(tab8)
Elogframe.pack(side=TOP, fill=BOTH, expand=True, padx=16, pady=(14, 8))
scrollbar = Scrollbar(Elogframe) 
scrollbar.pack(side=RIGHT, fill=Y)
tab8.ElogView = Listbox(Elogframe,width=230,height=40, yscrollcommand=scrollbar.set)
ui.style_listbox(tab8.ElogView, mono=False)
try:
  Elog = pickle.load(open("ErrorLog","rb"))
except:
  Elog = ['##BEGINNING OF LOG##']
  pickle.dump(Elog,open("ErrorLog","wb"))
time.sleep(.1)
for item in Elog:
  tab8.ElogView.insert(END,item) 
tab8.ElogView.pack(side=LEFT, fill=BOTH, expand=True)
scrollbar.config(command=tab8.ElogView.yview)

def clearLog():
 tab8.ElogView.delete(1,END)
 value=tab8.ElogView.get(0,END)
 pickle.dump(value,open("ErrorLog","wb"))

clearLogBut = Button(tab8,  text="Clear Log",  width=26, command = clearLog)
clearLogBut.pack(side=BOTTOM, anchor='w', padx=16, pady=(0, 14), before=Elogframe)




####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 9

link = Label(tab9, font=ui.F(12), text="https://www.anninrobotics.com/tutorials",  cursor="hand2")
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


splash.step("Loading calibration")
loaded_calibration = load_calibration()
apply_calibration(loaded_calibration, CAL)

if loaded_calibration.get("_loaded_defaults_due_to_config_error"):
    messagebox.showwarning(
        "Configuration Error",
        "ARconfig.json had missing or invalid required startup values.\n\n"
        "Default settings were loaded instead.\n\n"
        "Please recalibrate the robot before running."
    )

logger.debug(f"Comport 1 restored value is: {CAL['comPort']}")
logger.debug(f"Comport 1 restored value is: {CAL['com2Port']}")

if CAL['comPort'] in port_choices:
  com1SelectedValue.set(CAL['comPort'])

if CAL['com2Port'] in port_choices:
  com2SelectedValue.set(CAL['com2Port'])

# Restore saved robot model into combo box (without triggering defaults load)
savedModel = CAL.get('robotModel', 'AR4-MK5')
if savedModel in VALID_MODELS:
  modelSelectVar.set(savedModel)
modelSelectStartupDone = True  # enable combo box trace from now on


incrementEntryField.insert(0,"10")
speedEntryField.insert(0,"25")
ACCspeedField.insert(0,"15")
DECspeedField.insert(0,"15")
ACCrampField.insert(0,"50")
roundEntryField.insert(0,"15")
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
J1CalHighLowEntryField.insert(0,str(CAL['J1CalSwitch']))
J2CalHighLowEntryField.insert(0,str(CAL['J2CalSwitch']))
J3CalHighLowEntryField.insert(0,str(CAL['J3CalSwitch']))
J4CalHighLowEntryField.insert(0,str(CAL['J4CalSwitch']))
J5CalHighLowEntryField.insert(0,str(CAL['J5CalSwitch']))
J6CalHighLowEntryField.insert(0,str(CAL['J6CalSwitch']))
J7CalHighLowEntryField.insert(0,str(CAL['J7CalSwitch']))
J8CalHighLowEntryField.insert(0,str(CAL['J8CalSwitch']))
J9CalHighLowEntryField.insert(0,str(CAL['J9CalSwitch']))
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


def clear_cad_selection_on_outside_click(event):
    """Deselect the imported CAD when the user clicks the empty background of the virtual
    robot's import panel. This used to fire for every click anywhere in the program: typing
    into a field on another tab first wiped the CAD selection and blanked its X/Y/Z boxes."""
    w = event.widget
    if not isinstance(w, (tk.Widget, ttk.Widget)):
        return
    if selected_imported_filename is None:
        return
    # only clicks inside the import panel count ...
    inside = False
    parent = w
    while parent is not None:
        if parent is virtualImportFrame:
            inside = True
            break
        try:
            parent = parent.master
        except AttributeError:
            break
    if not inside:
        return
    # ... and not on something that takes input or acts on the selection
    cls = w.winfo_class()
    if cls in ("Entry", "TEntry", "Listbox", "Button", "TButton", "Spinbox", "TSpinbox",
               "TCombobox", "Menubutton", "TMenubutton", "Scale", "TScale", "Text"):
        return
    deselect_imported_cad()

root.bind_all("<Button-1>", clear_cad_selection_on_outside_click, add="+")




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

tab1.lastProg = ""
ui.run_launch(root, setCom)   # plays the splash, connects, then shows the main window

#tab1.mainloop()
root.mainloop()



#manEntryField.delete(0, 'end')
#manEntryField.insert(0,value)
