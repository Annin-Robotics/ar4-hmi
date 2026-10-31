#!/usr/bin/env python3
"""
make_crossplatform.py - patch AR4.py (v8.0) so it runs on both Windows and Linux.

Usage (from the folder that contains AR4.py):
    python make_crossplatform.py            # patch AR4.py in place (backup saved first)
    python make_crossplatform.py --check    # report what would change, write nothing
    python make_crossplatform.py path/to/AR4.py

All-or-nothing: if any expected code block isn't found, or the patched file would not
compile, nothing is written. Line endings (CRLF/LF) and encoding are preserved.
"""

import os
import re
import shutil
import sys
import time

SENTINEL = "# CROSS-PLATFORM PATCH v1"


def tolerant_pattern(old):
    """Regex for `old` that ignores trailing whitespace on each line."""
    return "\n".join(re.escape(line.rstrip()) + r"[ \t]*" for line in old.split("\n"))


# Each edit: (name, kind, old, new, expected)
#   kind     "text"  -> literal block (trailing whitespace tolerant)
#            "regex" -> raw regex (re.M), `new` may use \1 backrefs
#   expected int = exact match count, "+" = one or more
EDITS = []


def edit(name, old, new, expected=1, kind="text"):
    EDITS.append((name, kind, old, new, expected))


# ---------------------------------------------------------------------------
# 1. Resolve the app folder and chdir BEFORE anything loads relative files.
#    (CAD folders were built from os.getcwd() before the old chdir, so launching
#    from another directory on Linux put them in the wrong place.)
# ---------------------------------------------------------------------------
edit("early chdir + IS_WINDOWS",
"""import sys
import os
from datetime import datetime
from ARrobots.ARrobot import ARRobot, ARNano""",
"""import sys
import os
import pathlib
from datetime import datetime

""" + SENTINEL + """
# Resolve the app folder and chdir before anything opens a relative path (STLs, icons,
# config, CAD folders), so the program works no matter where it is launched from.
if getattr(sys, "frozen", False):
    DIR = pathlib.Path(sys.executable).resolve().parent
else:
    DIR = pathlib.Path(__file__).resolve().parent
os.chdir(DIR)
IS_WINDOWS = sys.platform.startswith("win")

from ARrobots.ARrobot import ARRobot, ARNano""")

# ---------------------------------------------------------------------------
# 2. Unused Tk-VTK widget import: needs vtkRenderingTk, missing from many Linux VTK builds
# ---------------------------------------------------------------------------
edit("optional vtkTkRenderWindowInteractor",
"""from vtkmodules.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor""",
"""try:
  from vtkmodules.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor  # unused, optional
except Exception:
  vtkTkRenderWindowInteractor = None""")

# ---------------------------------------------------------------------------
# 3. pygrabber optional + shared cross-platform helpers
# ---------------------------------------------------------------------------
edit("pygrabber optional + helpers",
"""if CE['Platform']['IS_WINDOWS']:
  from pygrabber.dshow_graph import FilterGraph""",
"""FilterGraph = None
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
  \"\"\"Turn a port entry into a real device name for this OS.
  '3' -> COM3 on Windows, /dev/ttyUSB3 on Linux; 'ttyACM0' -> /dev/ttyACM0 on Linux;
  full names (COM5, /dev/ttyACM0, /dev/serial/by-id/...) are used as-is.\"\"\"
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


def open_camera(source):
  \"\"\"Open a camera with the best backend for this OS (DirectShow on Windows, V4L2 on
  Linux), falling back to OpenCV's default backend.\"\"\"
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
  \"\"\"OpenCV source for the camera currently picked in the Vision tab menu.\"\"\"
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
  return str(x).replace("\\r", "").replace("\\n", "").strip()


def find_prog_row(text):
  \"\"\"Index of a program row, ignoring CRLF vs LF and bytes vs str.
  Program files saved on Linux end lines with \\\\n only, so exact matches against
  '...\\\\r\\\\n' failed there (and on any OS right after inserting a new row).\"\"\"
  target = _norm_prog_row(text)
  for i, item in enumerate(tab1.progView.get(0, "end")):
    if _norm_prog_row(item) == target:
      return i
  raise ValueError(f"Program row not found: {target!r}")


def resolve_imported_name(name):
  \"\"\"Find an imported CAD by name regardless of extension case (part / part.stl / part.STL).
  Linux filenames are case-sensitive, so blindly appending '.STL' missed 'part.stl'.\"\"\"
  if name in imported_actors:
    return name
  low = name.lower()
  base = os.path.splitext(low)[0] if low.endswith((".stl", ".step", ".stp")) else low
  for key in imported_actors:
    k = key.lower()
    if k == low or os.path.splitext(k)[0] == base:
      return key
  return name if low.endswith(".stl") else name + '.STL'""")

# ---------------------------------------------------------------------------
# 4. Virtual pick/place: case-insensitive CAD name lookup
# ---------------------------------------------------------------------------
edit("attachtogripper name lookup",
'''    # Allow callers to omit the .stl extension
    if not actorname.lower().endswith(".stl"):
        actorname += ".STL"''',
"""    # Allow callers to omit the extension; matches .stl/.STL either way (Linux is case-sensitive)
    actorname = resolve_imported_name(actorname)""")

edit("detachfromgripper name lookup",
'''    # Allow callers to omit the .STL extension
    if not actorname.lower().endswith(".stl"):
        actorname += ".STL"''',
"""    # Allow callers to omit the extension; matches .stl/.STL either way (Linux is case-sensitive)
    actorname = resolve_imported_name(actorname)""")

# ---------------------------------------------------------------------------
# 5. Virtual robot window
# ---------------------------------------------------------------------------
edit("VTK topmost thread is Windows-only",
"""def set_vtk_topmost_delayed(render_window, root_widget):
    '''Continuously keep VTK window on top using window title'''""",
"""def set_vtk_topmost_delayed(render_window, root_widget):
    '''Continuously keep VTK window on top using window title'''
    if not IS_WINDOWS:
        return  # uses win32gui; on Linux the window manager handles stacking""")

edit("pump VTK events on Linux",
"""def update_vtk(render_window, root_widget):
    if RUN['vtk_running']:
        process_vr_events()""",
"""def update_vtk(render_window, root_widget):
    if RUN['vtk_running'] and not IS_WINDOWS and RUN.get('interactor') is not None:
        # On X11 Tk's event loop doesn't service VTK's own display connection, so the
        # viewer would never see mouse/keyboard/close events. Pump them here.
        # (On Windows they arrive through the shared message loop already.)
        try:
            RUN['interactor'].ProcessEvents()
        except Exception as e:
            logger.debug(f"VTK ProcessEvents failed: {e}")
    if RUN['vtk_running']:
        process_vr_events()""")

# ---------------------------------------------------------------------------
# 6. Serial ports
# ---------------------------------------------------------------------------
edit("IO board COM default typo (com2ort)",
""".get('com2ort')""",
""".get('com2Port')""")

edit("Read COM program row port name",
"""port = "COM" + comNum""",
"""port = resolve_serial_port(comNum)""")

edit("Test Aux COM port name",
"""port = "COM" + com3PortEntryField.get()""",
"""port = resolve_serial_port(com3PortEntryField.get())""")

# ---------------------------------------------------------------------------
# 7. Cameras
# ---------------------------------------------------------------------------
edit("start_vid backend",
"""    RUN['cap'] = cv2.VideoCapture(RUN['selectedCam'], cv2.CAP_DSHOW)""",
"""    RUN['selectedCam'] = selected_camera_source()
    RUN['cap'] = open_camera(RUN['selectedCam'])""")

edit("take_pic / mask_pic camera source",
r"""^([ \t]*)RUN\['cap'\] = cv2\.VideoCapture\(RUN\['selectedCam'\]\)[ \t]*$""",
r"""\1RUN['selectedCam'] = selected_camera_source()
\1RUN['cap'] = open_camera(RUN['selectedCam'])""",
expected=2, kind="regex")

edit("Windows camera list without pygrabber",
"""    graph = FilterGraph()
    try:
      allcams = graph.get_input_devices()""",
"""    try:
      graph = FilterGraph()
      allcams = graph.get_input_devices()""")

edit("Windows camera index = DirectShow index",
"""    if not camList:
      camList = allcams[:]  # fallback if everything got filtered""",
"""    if not camList:
      camList = allcams[:]  # fallback if everything got filtered
    # Index in the unfiltered list is the DirectShow device number OpenCV expects
    CAMERA_LABEL_TO_ID.update({name: i for i, name in enumerate(allcams)})""")

edit("Linux / other camera enumeration",
"""  case "Linux":
    # Build label->id map
    label_to_id = {c.label: c.id for c in CE['Cameras']['Enum']}
    camList = list(label_to_id.keys())

    selected_label = tk.StringVar(value=(camList[0] if camList else "Select a Camera"))

    visoptions = StringVar(tab6)
    visoptions.set("Select a Camera")

    def on_camera_select(chosen_label):
      cam_id = label_to_id.get(chosen_label, "None")
      logger.debug("Debug - User picked:", chosen_label, " -> using id:", cam_id)""",
"""  case _:
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
      logger.debug("User picked camera %s -> using id %s", chosen_label, cam_id)""")

# ---------------------------------------------------------------------------
# 8. Program rows: CRLF vs LF tolerant lookups (Jump/Call/Vis Find/program loop)
# ---------------------------------------------------------------------------
edit("tab lookups ignore line endings",
"""index = tab1.progView.get(0, "end").index(tabNum)""",
"""index = find_prog_row(tabNum)""",
expected="+")

edit("program-loop lookup ignores line endings",
"""index = tab1.progView.get(0, "end").index(progLoop)""",
"""index = find_prog_row(progLoop)""",
expected="+")

edit("Load dialog 'all files' filter",
"""filetypes = (('robot program', '*.ar4'),("all files", "*.*"))""",
"""filetypes = (('robot program', '*.ar4 *.AR4'),("all files", "*"))""")

# ---------------------------------------------------------------------------
# 9. Jog buttons: left mouse button only. On Linux/X11 the scroll wheel sends
#    ButtonPress 4/5, so scrolling over a jog button started a jog.
# ---------------------------------------------------------------------------
edit("jog press = left button only",
""".bind("<ButtonPress>",""",
""".bind("<ButtonPress-1>",""",
expected="+")

edit("jog release = left button only",
""".bind("<ButtonRelease>",""",
""".bind("<ButtonRelease-1>",""",
expected="+")

# ---------------------------------------------------------------------------
# 10. Xbox controller: missing driver/library disables the feature instead of crashing
# ---------------------------------------------------------------------------
edit("XInput missing is not fatal",
"""  if _xinput is None:
      raise OSError("XInput DLL not found")""",
"""  if _xinput is None:
      logger.warning("XInput DLL not found - Xbox controller support disabled")""")

edit("XInput stub when DLL missing",
"""  XInputGetState = _xinput.XInputGetState
  XInputGetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_STATE)]
  XInputGetState.restype  = ctypes.c_uint""",
"""  if _xinput is not None:
      XInputGetState = _xinput.XInputGetState
      XInputGetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_STATE)]
      XInputGetState.restype  = ctypes.c_uint
  else:
      def XInputGetState(_idx, _state):
          return 1167  # ERROR_DEVICE_NOT_CONNECTED -> shows 'No XInput controller detected'""")

edit("Linux 'inputs' package optional",
"""else:
  from inputs import get_gamepad
  def xbox():
    def threadxbox():""",
"""else:
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
    def threadxbox():""")


# ---------------------------------------------------------------------------
def apply(text):
    report, missing = [], []
    for name, kind, old, new, expected in EDITS:
        if kind == "regex":
            pattern = re.compile(old, re.M)
        else:
            pattern = re.compile(tolerant_pattern(old))
        hits = len(pattern.findall(text))
        ok = hits >= 1 if expected == "+" else hits == expected
        if not ok:
            missing.append(f"  - {name}: expected {expected}, found {hits}")
            continue
        if kind == "regex":
            text = pattern.sub(new, text)
        else:
            text = pattern.sub(lambda m, n=new: n, text)
        report.append(f"  ok  {name} ({hits}x)")
    return text, report, missing


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    check_only = "--check" in sys.argv
    target = args[0] if args else "AR4.py"

    if not os.path.isfile(target):
        sys.exit(f"Can't find {target} - run this from the folder that contains AR4.py "
                 f"or pass its path.")

    with open(target, "rb") as f:
        raw = f.read()
    bom = raw.startswith(b"\xef\xbb\xbf")
    src = raw.decode("utf-8-sig")
    newline = "\r\n" if "\r\n" in src else "\n"
    src = src.replace("\r\n", "\n")

    if SENTINEL in src:
        sys.exit(f"{target} is already patched (found '{SENTINEL}'). Nothing to do.")

    patched, report, missing = apply(src)
    print("\n".join(report))

    if missing:
        print("\nThese blocks were not found, so NOTHING was written:")
        print("\n".join(missing))
        print("\nYour AR4.py probably differs from the v8.0 file this patch was written for.")
        sys.exit(1)

    try:
        compile(patched, target, "exec")
    except SyntaxError as e:
        sys.exit(f"\nPatched file would not compile ({e}); NOTHING was written.")

    if check_only:
        print(f"\n--check: all {len(EDITS)} changes apply cleanly. Run without --check to write.")
        return

    backup = f"{target}.{time.strftime('%Y%m%d-%H%M%S')}.bak"
    shutil.copy2(target, backup)
    out = patched.replace("\n", newline).encode("utf-8")
    with open(target, "wb") as f:
        f.write((b"\xef\xbb\xbf" if bom else b"") + out)
    print(f"\nPatched {target} ({len(EDITS)} changes). Original saved as {backup}")


if __name__ == "__main__":
    main()
