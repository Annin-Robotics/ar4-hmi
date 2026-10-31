#!/usr/bin/env python3
"""
make_modern_ui.py - apply the modern UI (ar4_ui.py + assets/) to AR4.py 8.0.

Run from the folder that contains AR4.py, ar4_ui.py and assets/:
    python make_modern_ui.py --check     # report only
    python make_modern_ui.py             # patch AR4.py (timestamped backup kept)

Expects the cross-platform patch to be applied already. All-or-nothing: nothing is
written unless every change matches and the result compiles.
"""
import os
import re
import shutil
import sys
import time

SENTINEL = "# MODERN UI PATCH v1"
REQUIRES = "# CROSS-PLATFORM PATCH v1"


def tolerant_pattern(old):
    return "\n".join(re.escape(line.rstrip()) + r"[ \t]*" for line in old.split("\n"))


EDITS = []


def edit(name, old, new, expected=1, kind="text"):
    EDITS.append((name, kind, old, new, expected))


# 1 ---------------------------------------------------------------- import the UI module
edit("import ar4_ui",
"""LabelFrame = ttk.LabelFrame""",
"""LabelFrame = ttk.LabelFrame

""" + SENTINEL + """
import ar4_ui as ui   # themes, fonts, header bar, splash (see ar4_ui.py / assets/)""")

# 2 ---------------------------------------------------------------- window, icon, splash
edit("root window + splash",
"""root = Tk()
root.wm_title("AR4 Software Ver 8.0")
root.iconphoto(True, tk.PhotoImage(file="AR.png"))""",
"""root = Tk(className="AR4")
root.wm_title("AR4 Robot Control 8.0")
ui.init(root)                            # fonts, icon, themes + styles: before any widget
splash = ui.show_launch_splash(root)     # main window stays hidden until startup is done""")

# 3 ---------------------------------------------------------------- header bar + tabs
edit("header bar above the notebook",
"""nb = ttk_bootstrap.Notebook(root)

# Configure root window for resizing
root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=1)
nb.grid(row=0, column=0, sticky='nsew')""",
"""def _header_status():
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
nb.grid(row=1, column=0, sticky='nsew')""")

for old, new in ((" Main Controls ", "Main Controls"), ("  System Setup  ", "System Setup"),
                 ("   Robot Parameters    ", "Robot Parameters"), (" Inputs Outputs ", "Inputs & Outputs"),
                 ("   Registers    ", "Registers"), ("   Vision    ", "Vision"),
                 ("    G-Code     ", "G-Code"), ("      Log      ", "Log")):
    edit(f"tab label {new}", f"text='{old}')", f"text='{new}')")

# Pages and side panels scroll when the window is shorter than their content
edit("settings pages scroll when the window is short",
     r"^(tab[2345]) = ttk_bootstrap\.Frame\(nb\)\nnb\.add\(\1,",
     r"\1 = ui.scroll_frame(nb)\nnb.add(ui.page(\1),", expected=4, kind="regex")
edit("Main Controls left panel scrolls",
"""leftPanel = Frame(tab1, relief="raised", borderwidth=1)""",
"""leftPanel = ui.scroll_frame(tab1)   # scrolls when the window is short""")
edit("program list gives way before the jog panel is clipped",
"""tab1.grid_columnconfigure(1, weight=1, minsize=550)  # Center panel - expands""",
"""tab1.grid_columnconfigure(1, weight=1, minsize=420)  # Center panel - expands, shrinks first""")
edit("Main Controls right panel scrolls",
"""rightPanel = Frame(tab1)""",
"""rightPanel = ui.scroll_frame(tab1)   # scrolls when the window is short""")

edit("header shows the pages as tabs",
"""nb.add(tab8, text='Log')""",
"""nb.add(tab8, text='Log')
header.attach(nb)""")

# 4 ---------------------------------------------------------------- startup progress
edit("startup progress uses the splash",
"""def startup_with_spinner(root, timeout=10.0):
    spinner, pb = startup_spinner(root, "Please Wait.. System Starting")
    q = Queue()""",
"""def startup_with_spinner(root, timeout=10.0):
    try:
      port = CAL['comPort']
    except Exception:
      port = ""
    progress = ui.busy(root, f"Connecting to Teensy on {port}" if port else "Connecting to Teensy")
    q = Queue()""")

edit("close startup progress",
"""    # close spinner (success or timeout)
    try: pb.stop()
    except: pass
    try: spinner.grab_release()
    except: pass
    spinner.destroy()""",
"""    # close progress (success or timeout); the launch splash closes itself
    progress.close()""")

edit("old spinner window: new icon",
"""    win.iconphoto(True, tk.PhotoImage(file="AR.png"))""",
"""    ui.set_window_icon(win, default=False)""")

# 5 ---------------------------------------------------------------- themes
edit("dark/light theme functions",
r"""(?ms)^def darkTheme\(\):.*?(?=^#{40,})""",
"""def darkTheme():
  CAL['curTheme'] = 0
  ui.set_theme(root, "dark")      # graphite


def lightTheme():
  CAL['curTheme'] = 1
  ui.set_theme(root, "light")     # aluminium



""", kind="regex")

# 6 ---------------------------------------------------------------- fonts
edit("fonts: (\"Arial\", N, \"bold\")", r'\("Arial",\s*(\d+),\s*"bold"\)', r'ui.F(\1, "bold")',
     expected="+", kind="regex")
edit("fonts: (\"Arial\", N)", r'\("Arial",\s*(\d+)\)', r'ui.F(\1)', expected="+", kind="regex")
edit("fonts: (\"Arial N bold\")", r'\("Arial (\d+) bold"\)', r'ui.F(\1, "bold")', expected="+", kind="regex")
edit("fonts: font='12'", r"font='(\d+)'", r"font=ui.F(\1)", expected="+", kind="regex")

# 7 ---------------------------------------------------------------- program controls
edit("run / step / stop buttons",
r"""(?ms)^# Load play/stop button images\n.*?^stopBut\.grid\(row=0, column=3, sticky="nsew", padx=2, pady=0\)[ \t]*$""",
"""# Program controls: Run is the only solid orange control (it moves the robot)
playBut = Button(playFrame, text="▶  Run", style="Run.TButton", command=runProg)
playBut.grid(row=0, column=0, sticky="nsew", padx=2, pady=0)

revBut = Button(playFrame, text="Step back", style="Step.TButton", command=stepRev)
revBut.grid(row=0, column=1, sticky="nsew", padx=2, pady=0)

fwdBut = Button(playFrame, text="Step forward", style="Step.TButton", command=stepFwd)
fwdBut.grid(row=0, column=2, sticky="nsew", padx=2, pady=0)

stopBut = Button(playFrame, text="■  Stop", style="Stop.TButton", command=stopProg)
stopBut.grid(row=0, column=3, sticky="nsew", padx=2, pady=0)""", kind="regex")

# 8 ---------------------------------------------------------------- jog controls
edit("jog - buttons", r'Button\((\w+), text="-", width=3\)',
     r'Button(\1, text="\\u2212", width=3, style="Motion.TButton")', expected=7, kind="regex")
edit("jog + buttons", r'Button\((\w+), text="\+", width=3\)',
     r'Button(\1, text="+", width=3, style="Motion.TButton")', expected=7, kind="regex")
edit("joint sliders are motion controls",
     r'(slider = Scale\(frame, from_=-170, to=170, orient=HORIZONTAL, length=150)\)',
     r'\1, bootstyle="warning")', kind="regex")
edit("J7-J9 sliders are motion controls",
     r'(J[789]jogslide = Scale\(J[789]jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120)\)',
     r'\1, bootstyle="warning")', expected=3, kind="regex")

# 9 ---------------------------------------------------------------- listboxes
edit("program listing style",
"""tab1.progView.pack(side=LEFT, fill=BOTH, expand=True)""",
"""tab1.progView.pack(side=LEFT, fill=BOTH, expand=True)
ui.style_listbox(tab1.progView)""")
edit("gcode listing style",
"""tab7.gcodeView = Listbox(gcodeframe,exportselection=0,width=105,height=43, yscrollcommand=gcodescrollbar.set)""",
"""tab7.gcodeView = Listbox(gcodeframe,exportselection=0,width=105,height=43, yscrollcommand=gcodescrollbar.set)
ui.style_listbox(tab7.gcodeView)""")
edit("log listing style",
"""tab8.ElogView = Listbox(Elogframe,width=230,height=40, yscrollcommand=scrollbar.set)""",
"""tab8.ElogView = Listbox(Elogframe,width=230,height=40, yscrollcommand=scrollbar.set)
ui.style_listbox(tab8.ElogView, mono=False)""")
edit("imported CAD list style",
"""importedCadList = Listbox(virtualImportFrame, height=6, exportselection=False)""",
"""importedCadList = Listbox(virtualImportFrame, height=6, exportselection=False)
ui.style_listbox(importedCadList, mono=False)""")

# Row colours follow the theme (white rows used to vanish in the light theme)
ROW_COLOURS = {"#FFFFFF": "fg", "#000000": "fg", "#1E90FF": "info", "#0561BD": "accent",
               "#0057A6": "accent", "#9E9E9E": "muted", "#959697": "muted",
               "#FF0000": "danger", "#EE5C42": "danger", "#8264B8": "edited"}
edit("program row colours follow the theme",
     r"""(itemconfig\([^\n]*?\{'fg': )"(#[0-9A-Fa-f]{6})"\}""",
     lambda m: f"{m.group(1)}ui.C['{ROW_COLOURS[m.group(2).upper()]}']}}",
     expected="+", kind="regex")

# Log page: list fills the page, Clear button underneath instead of on top of it
edit("log page layout",
"""Elogframe=Frame(tab8)
Elogframe.place(x=40,y=15)""",
"""Elogframe=Frame(tab8)
Elogframe.pack(side=TOP, fill=BOTH, expand=True, padx=16, pady=(14, 8))""")
edit("log list fills its frame",
"""tab8.ElogView.pack()""",
"""tab8.ElogView.pack(side=LEFT, fill=BOTH, expand=True)""")
edit("clear-log button below the list",
"""clearLogBut.place(x=40, y=690)""",
"""clearLogBut.pack(side=BOTTOM, anchor='w', padx=16, pady=(0, 14), before=Elogframe)""")
# Tracebacks: one row per line instead of one row full of newline boxes
edit("log entries keep their line breaks",
"""          tab8.ElogView.after(0, lambda: tab8.ElogView.insert(END, f"{Curtime} - {message}"))""",
"""          lines = str(message).splitlines() or [""]
          rows = [f"{Curtime} - {lines[0]}"] + ["      " + l for l in lines[1:]]
          tab8.ElogView.after(0, lambda: (tab8.ElogView.insert(END, *rows), tab8.ElogView.see(END)))""")
edit("links readable in both themes",
     r'(link2 = Label\(infoFrame, font=)ui\.F\(8\)(, text="[^"]*"), foreground="blue"',
     r'\1ui.F(9)\2, style="Link.TLabel"', kind="regex")

# 10 --------------------------------------------------------------- virtual robot viewer
edit("viewer background follows the theme",
"""    RUN['renderer'].SetBackground(vtk.vtkNamedColors().GetColor3d("LightSlateGray"))""",
"""    _vc = ui.vtk_colors()
    RUN['renderer'].SetBackground(*_vc['bottom'])
    RUN['renderer'].SetBackground2(*_vc['top'])
    RUN['renderer'].GradientBackgroundOn()""")
edit("viewer floor grid colour",
"""    actor.GetProperty().SetColor(0.7, 0.7, 0.7)  # Light gray""",
"""    actor.GetProperty().SetColor(*ui.vtk_colors()['grid'])""")

# 11 --------------------------------------------------------------- launch sequence
edit("splash progress before calibration load",
"""loaded_calibration = load_calibration()
apply_calibration(loaded_calibration, CAL)""",
"""splash.step("Loading calibration")
loaded_calibration = load_calibration()
apply_calibration(loaded_calibration, CAL)""")
edit("launch: connect, then reveal the window",
"""tab1.after(100, setCom)""",
"""ui.run_launch(root, setCom)   # plays the splash, connects, then shows the main window""")


# ---------------------------------------------------------------------------- engine
def apply(text):
    report, missing = [], []
    for name, kind, old, new, expected in EDITS:
        pattern = re.compile(old, re.M) if kind == "regex" else re.compile(tolerant_pattern(old))
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
        sys.exit(f"Can't find {target} - run this from the folder that contains AR4.py.")
    for need in ("ar4_ui.py", "assets"):
        if not os.path.exists(os.path.join(os.path.dirname(os.path.abspath(target)), need)):
            print(f"Warning: {need} is not next to {target}; copy it there before running AR4.")

    with open(target, "rb") as f:
        raw = f.read()
    bom = raw.startswith(b"\xef\xbb\xbf")
    src = raw.decode("utf-8-sig")
    newline = "\r\n" if "\r\n" in src else "\n"
    src = src.replace("\r\n", "\n")

    if SENTINEL in src:
        sys.exit(f"{target} already has the modern UI patch. Nothing to do.")
    if REQUIRES not in src:
        sys.exit(f"{target} needs the cross-platform patch first (make_crossplatform.py).")

    patched, report, missing = apply(src)
    print("\n".join(report))
    if missing:
        print("\nThese blocks were not found, so NOTHING was written:")
        print("\n".join(missing))
        sys.exit(1)
    leftover = re.findall(r"""font\s*=\s*\(?["'][^"']*Arial""", patched)
    if leftover:
        print(f"\nNote: {len(leftover)} Arial font spec(s) left as-is: {leftover[:3]}")
    try:
        compile(patched, target, "exec")
    except SyntaxError as e:
        sys.exit(f"\nPatched file would not compile ({e}); NOTHING was written.")
    if check_only:
        print(f"\n--check: all {len(EDITS)} changes apply cleanly. Run without --check to write.")
        return
    backup = f"{target}.{time.strftime('%Y%m%d-%H%M%S')}.bak"
    shutil.copy2(target, backup)
    with open(target, "wb") as f:
        f.write((b"\xef\xbb\xbf" if bom else b"") + patched.replace("\n", newline).encode("utf-8"))
    print(f"\nPatched {target} ({len(EDITS)} changes). Original saved as {backup}")


if __name__ == "__main__":
    main()
