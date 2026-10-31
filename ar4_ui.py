"""
ar4_ui.py - look and feel for the AR4 HMI.

Everything visual lives here so AR4.py only has to call a handful of functions:

    import ar4_ui as ui
    root = Tk(className="AR4")
    ui.init(root)                 # fonts, window icon, themes
    splash = ui.Splash(root)      # animated launch splash
    header = ui.Header(root, status_fn)
    ui.set_theme(root, "dark")    # or "light"
    ui.F(10, "bold")              # font tuple in the UI typeface

Colour language (the same in both themes):
    accent / orange   things that MOVE the robot: jog, run, sliders
    danger / red      stop, alarms, faults
    success / green   connected, OK
    caution / amber   warnings, offline mode
    info / steel      focus, selection, informational readouts
"""

import ctypes
import ctypes.util
import glob
import logging
import os
import subprocess
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

from PIL import Image, ImageTk

logger = logging.getLogger("ARrobots.ui")

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")
IS_WINDOWS = sys.platform.startswith("win")

APP_NAME = "AR4 Robot Control"
APP_VERSION = "8.1"

# --------------------------------------------------------------------------- palette
PALETTES = {
    # Machined graphite: blue-grey neutrals, recessed (darker) inputs, raised slabs.
    "dark": {
        "bg": "#1a1e24", "chrome": "#13161b", "surface": "#0f1216",
        "raised": "#2b313a", "raised_hover": "#363d48", "raised_pressed": "#232830",
        "border": "#2e343d", "border_strong": "#424a56",
        "fg": "#e7eaee", "muted": "#8a94a2", "faint": "#5b6470",
        "accent": "#ff7a1a", "accent_hover": "#ff9142", "accent_pressed": "#e5660b",
        "on_accent": "#1a1e24",
        "danger": "#ff5a5f", "danger_hover": "#ff7478", "danger_pressed": "#e54449",
        "on_danger": "#1a1e24",
        "success": "#34c77b", "caution": "#f2c94c", "info": "#5aa9e6",
        "edited": "#b39dfa",
        "select": "#2d4560", "select_fg": "#ffffff",
        "vtk_top": "#2a3039", "vtk_bottom": "#0f1216", "vtk_grid": "#3a424d",
    },
    # Brushed aluminium: cool light greys, white inputs, same accents darkened for contrast.
    "light": {
        "bg": "#e8ebef", "chrome": "#dadfe5", "surface": "#fbfcfd",
        "raised": "#f6f7f9", "raised_hover": "#ffffff", "raised_pressed": "#dde2e8",
        "border": "#c3cad3", "border_strong": "#a7b0bc",
        "fg": "#1a1e24", "muted": "#5d6773", "faint": "#8a94a2",
        "accent": "#f47316", "accent_hover": "#ff8833", "accent_pressed": "#d9620c",
        "on_accent": "#1a1e24",
        "danger": "#d1343a", "danger_hover": "#e0474d", "danger_pressed": "#b82a30",
        "on_danger": "#ffffff",
        "success": "#13894f", "caution": "#9a6700", "info": "#1f6fb5",
        "edited": "#6d4fd1",
        "select": "#c9dcef", "select_fg": "#1a1e24",
        "vtk_top": "#f7f8fa", "vtk_bottom": "#cfd5dc", "vtk_grid": "#aab2bc",
    },
}

#: Live palette for the current theme (mutated in place, so `from ar4_ui import C` stays valid)
C = dict(PALETTES["dark"])
MODE = {"name": "dark"}

# ----------------------------------------------------------------------------- fonts
FONT = {"family": "TkDefaultFont", "bold_family": None, "mono": "TkFixedFont", "offset": 0}
UI_FAMILIES = ("Barlow Semi Condensed", "Segoe UI", "Noto Sans", "Cantarell",
               "Ubuntu", "DejaVu Sans", "Helvetica", "Arial")
MONO_FAMILIES = ("Cascadia Mono", "Consolas", "JetBrains Mono", "Noto Sans Mono",
                 "DejaVu Sans Mono", "Liberation Mono", "Menlo", "Courier New")


def _register_bundled_fonts():
    """Make assets/fonts/*.ttf available to Tk for this process only (nothing installed)."""
    files = glob.glob(os.path.join(ASSETS, "fonts", "*.ttf"))
    if not files:
        return
    try:
        if IS_WINDOWS:
            add = ctypes.windll.gdi32.AddFontResourceExW
            for f in files:
                add(f, 0x10, 0)                   # FR_PRIVATE
        elif sys.platform.startswith("linux") or "bsd" in sys.platform:
            lib = ctypes.util.find_library("fontconfig")
            if lib:
                fc = ctypes.CDLL(lib)
                fc.FcConfigAppFontAddFile.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
                for f in files:
                    fc.FcConfigAppFontAddFile(None, f.encode())
    except Exception as e:                        # fall back to system fonts
        logger.debug(f"Bundled font registration failed: {e}")


_register_bundled_fonts()   # must run before Tk opens its first font


def F(size=10, weight="normal", mono=False):
    """Font tuple in the UI typeface. 'bold' uses the SemiBold cut when available."""
    size = int(size) + FONT["offset"]
    if mono:
        return (FONT["mono"], size - 1, weight)
    if weight == "bold" and FONT["bold_family"]:
        return (FONT["bold_family"], size)
    return (FONT["family"], size, weight)


def _pick(families, wanted):
    for name in wanted:
        if name in families:
            return name
    return None


def _setup_fonts(root):
    fams = set(tkfont.families(root))
    fam = _pick(fams, UI_FAMILIES) or "TkDefaultFont"
    FONT["family"] = fam
    # Bundled Barlow ships a SemiBold cut; fontconfig/GDI both resolve this family name
    FONT["bold_family"] = (fam + " SemiBold") if fam.startswith("Barlow") else None
    FONT["mono"] = _pick(fams, MONO_FAMILIES) or "TkFixedFont"
    FONT["offset"] = 1 if fam.startswith("Barlow") else 0
    for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont",
                 "TkCaptionFont", "TkSmallCaptionFont", "TkIconFont", "TkTooltipFont"):
        try:
            tkfont.nametofont(name, root).configure(family=fam, size=10 + FONT["offset"])
        except tk.TclError:
            pass
    try:
        tkfont.nametofont("TkFixedFont", root).configure(family=FONT["mono"], size=10)
    except tk.TclError:
        pass
    logger.debug(f"UI font: {fam} (bold: {FONT['bold_family']}), mono: {FONT['mono']}")


# ------------------------------------------------------------------------------ init
_IMAGES = {}   # keep PhotoImage references alive


def _img(name, master=None):
    key = (name, str(master))
    if key not in _IMAGES:
        _IMAGES[key] = tk.PhotoImage(master=master, file=os.path.join(ASSETS, name))
    return _IMAGES[key]


def init(root, mode="dark"):
    """Fonts, window/taskbar icon, themes and every custom style. Call right after Tk(),
    before any widget exists: ttkbootstrap quietly replaces a style= name that is not
    defined yet with the plain base style (e.g. 'Motion.TButton' -> 'TButton')."""
    if IS_WINDOWS:
        try:   # own taskbar group + icon instead of python.exe's
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("AnninRobotics.AR4")
        except Exception:
            pass
    _setup_fonts(root)
    set_window_icon(root)
    _bind_wheel(root)
    _register_themes()
    set_theme(root, mode)


def set_window_icon(win, default=True):
    try:
        icons = [_img(f"icon-{s}.png", win) for s in (256, 128, 64, 48, 32, 16)]
        win.iconphoto(default, *icons)
    except Exception as e:
        logger.debug(f"Window icon not set: {e}")


# ---------------------------------------------------------------------------- themes
THEME_NAMES = {"dark": "ar4graphite", "light": "ar4aluminum"}


def _register_themes():
    from ttkbootstrap import Style
    from ttkbootstrap.style import ThemeDefinition
    style = Style()
    for mode, name in THEME_NAMES.items():
        if name in style.theme_names():
            continue
        p = PALETTES[mode]
        style.register_theme(ThemeDefinition(name=name, themetype=mode, colors={
            # primary = interaction colour (focus rings, checks, selection)
            "primary": p["info"], "secondary": p["raised"],
            "success": p["success"], "info": p["info"],
            # 'warning' is repurposed as the motion accent so bootstyle="warning"
            # gives orange sliders/handles; amber cautions use C['caution'] directly.
            "warning": p["accent"], "danger": p["danger"],
            "light": p["border_strong"], "dark": p["surface"],   # light = slider trough
            "bg": p["bg"], "fg": p["fg"], "selectbg": p["select"], "selectfg": p["select_fg"],
            "border": p["border"], "inputfg": p["fg"], "inputbg": p["surface"],
            "active": p["raised_hover"],
        }))


def _button(style, name, bg, fg, hover, pressed, border=None, font=None, padding=(10, 3)):
    border = border or bg
    style.configure(name, foreground=fg, background=bg, bordercolor=border,
                    darkcolor=bg, lightcolor=bg, relief="raised", focusthickness=1,
                    focuscolor=fg, padding=padding, anchor="center", font=font or F(10))
    dis_bg = C["chrome"]
    style.map(name,
              foreground=[("disabled", C["faint"])],
              background=[("disabled", dis_bg), ("pressed !disabled", pressed), ("hover !disabled", hover)],
              bordercolor=[("disabled", dis_bg), ("focus !disabled", C["info"])],
              darkcolor=[("disabled", dis_bg), ("pressed !disabled", pressed), ("hover !disabled", hover)],
              lightcolor=[("disabled", dis_bg), ("pressed !disabled", pressed), ("hover !disabled", hover)])


_LISTBOXES = []
_THEME_CALLBACKS = []


def on_theme_change(fn):
    """Register fn(mode) to run after every theme switch."""
    _THEME_CALLBACKS.append(fn)
    return fn


def style_listbox(lb, mono=True):
    """Recessed, flat listbox; restyled automatically on theme switch."""
    if all(lb is not w for w, _ in _LISTBOXES):
        _LISTBOXES.append((lb, mono))
    try:
        lb.configure(bg=C["surface"], fg=C["fg"], selectbackground=C["select"],
                     selectforeground=C["select_fg"], highlightthickness=1,
                     highlightbackground=C["border"], highlightcolor=C["info"],
                     borderwidth=0, relief="flat", activestyle="none",
                     font=F(10, mono=True) if mono else F(10))
    except tk.TclError:
        pass


def set_theme(root, mode="dark"):
    """Switch to the graphite ('dark') or aluminium ('light') theme. Returns the Style."""
    from ttkbootstrap import Style
    mode = "light" if mode == "light" else "dark"
    MODE["name"] = mode
    C.clear()
    C.update(PALETTES[mode])
    style = getattr(root, "style", None) or Style()
    root.style = style
    style.theme_use(THEME_NAMES[mode])
    _apply_styles(style)
    root.configure(background=C["bg"])
    for lb, mono in list(_LISTBOXES):
        style_listbox(lb, mono=mono)
    restyle_tree(root)
    for fn in list(_THEME_CALLBACKS):
        try:
            fn(mode)
        except Exception as e:
            logger.debug(f"theme callback failed: {e}")
    _windows_titlebar(root, mode == "dark")
    return style


def _apply_styles(s):
    c = C
    s.configure(".", font=F(10))

    # Buttons: neutral slabs by default; colour is reserved for meaning.
    _button(s, "TButton", c["raised"], c["fg"], c["raised_hover"], c["raised_pressed"], border=c["border"])
    # Jog buttons: orange outline at rest, solid orange while pressed/hovered.
    # Run is then the only solid orange block on screen.
    s.configure("Motion.TButton", foreground=c["accent"], background=c["raised"],
                bordercolor=c["accent"], darkcolor=c["raised"], lightcolor=c["raised"],
                relief="raised", focusthickness=1, focuscolor=c["accent"],
                padding=(6, 1), anchor="center", font=F(12, "bold"))
    s.map("Motion.TButton",
          foreground=[("disabled", c["faint"]), ("pressed !disabled", c["on_accent"]),
                      ("hover !disabled", c["on_accent"])],
          background=[("disabled", c["chrome"]), ("pressed !disabled", c["accent_pressed"]),
                      ("hover !disabled", c["accent"])],
          bordercolor=[("disabled", c["border"]), ("pressed !disabled", c["accent_pressed"])],
          darkcolor=[("pressed !disabled", c["accent_pressed"]), ("hover !disabled", c["accent"])],
          lightcolor=[("pressed !disabled", c["accent_pressed"]), ("hover !disabled", c["accent"])])
    _button(s, "Run.TButton", c["accent"], c["on_accent"], c["accent_hover"], c["accent_pressed"],
            font=F(11, "bold"), padding=(14, 7))
    _button(s, "Stop.TButton", c["danger"], c["on_danger"], c["danger_hover"], c["danger_pressed"],
            font=F(11, "bold"), padding=(14, 7))
    _button(s, "Step.TButton", c["raised"], c["fg"], c["raised_hover"], c["raised_pressed"],
            border=c["border"], font=F(11, "bold"), padding=(14, 7))
    # Online / offline toggle: amber outline while offline so it is impossible to miss
    _button(s, "Online.TButton", c["raised"], c["fg"], c["raised_hover"], c["raised_pressed"], border=c["border"])
    _button(s, "Offline.TButton", c["raised"], c["caution"], c["raised_hover"], c["raised_pressed"],
            border=c["caution"], font=F(10, "bold"))
    _button(s, "AlarmBut.TButton", c["raised"], c["danger"], c["raised_hover"], c["raised_pressed"], border=c["danger"])

    s.configure("TMenubutton", background=c["raised"], foreground=c["fg"], bordercolor=c["border"],
                darkcolor=c["raised"], lightcolor=c["raised"], arrowcolor=c["muted"],
                padding=(8, 2), font=F(10), relief="raised")
    s.map("TMenubutton",
          background=[("pressed", c["raised_pressed"]), ("hover", c["raised_hover"])],
          darkcolor=[("pressed", c["raised_pressed"]), ("hover", c["raised_hover"])],
          lightcolor=[("pressed", c["raised_pressed"]), ("hover", c["raised_hover"])],
          arrowcolor=[("hover", c["fg"])])

    # Inputs: recessed pockets
    s.configure("TEntry", fieldbackground=c["surface"], foreground=c["fg"], bordercolor=c["border"],
                lightcolor=c["surface"], darkcolor=c["surface"], insertcolor=c["fg"],
                padding=(4, 1), font=F(10))
    s.map("TEntry",
          bordercolor=[("focus", c["info"]), ("hover", c["border_strong"])],
          lightcolor=[("focus", c["surface"])], darkcolor=[("focus", c["surface"])],
          fieldbackground=[("readonly", c["bg"]), ("disabled", c["bg"])],
          foreground=[("disabled", c["faint"])])
    s.configure("TCombobox", fieldbackground=c["surface"], foreground=c["fg"], bordercolor=c["border"],
                arrowcolor=c["muted"], padding=(4, 2))

    # Containers
    s.configure("TFrame", background=c["bg"])
    s.configure("TLabel", background=c["bg"], foreground=c["fg"], font=F(10))
    s.configure("TLabelframe", background=c["bg"], bordercolor=c["border"],
                lightcolor=c["bg"], darkcolor=c["bg"], relief="solid", borderwidth=1)
    s.configure("TLabelframe.Label", background=c["bg"], foreground=c["muted"], font=F(10, "bold"))
    s.configure("TCheckbutton", background=c["bg"], foreground=c["fg"], font=F(10))
    s.configure("TRadiobutton", background=c["bg"], foreground=c["fg"], font=F(10))

    # Notebook: tabs sit on the chrome strip; the selected tab joins the page
    s.configure("TNotebook", background=c["chrome"], bordercolor=c["border"],
                lightcolor=c["chrome"], darkcolor=c["chrome"], tabmargins=(8, 6, 8, 0))
    s.configure("TNotebook.Tab", background=c["chrome"], foreground=c["muted"],
                bordercolor=c["chrome"], lightcolor=c["chrome"], darkcolor=c["chrome"],
                padding=(16, 7), font=F(10, "bold"))
    s.map("TNotebook.Tab",
          background=[("selected", c["bg"]), ("hover", c["raised_pressed"])],
          foreground=[("selected", c["fg"]), ("hover", c["fg"])],
          bordercolor=[("selected", c["border"])],
          lightcolor=[("selected", c["bg"])], darkcolor=[("selected", c["bg"])],
          expand=[("selected", (0, 0, 0, 0))])

    # Main notebook: tabs live in the header bar, so the notebook draws none of its own
    s.layout("Bare.TNotebook.Tab", [])
    s.configure("Bare.TNotebook", background=c["bg"], bordercolor=c["bg"], lightcolor=c["bg"],
                darkcolor=c["bg"], borderwidth=0, tabmargins=(0, 0, 0, 0), padding=0)

    s.configure("Link.TLabel", background=c["bg"], foreground=c["info"], font=F(9))

    # Scrollbars: quiet thumbs that only firm up under the pointer
    for orient in ("Vertical", "Horizontal"):
        name = f"{orient}.TScrollbar"
        s.configure(name, background=c["raised"], troughcolor=c["bg"], bordercolor=c["bg"],
                    lightcolor=c["raised"], darkcolor=c["raised"], arrowcolor=c["muted"],
                    gripcount=0, arrowsize=11, relief="flat")
        s.map(name,
              background=[("pressed", c["border_strong"]), ("active", c["raised_hover"])],
              lightcolor=[("pressed", c["border_strong"]), ("active", c["raised_hover"])],
              darkcolor=[("pressed", c["border_strong"]), ("active", c["raised_hover"])],
              arrowcolor=[("active", c["fg"])])

    # Status text used throughout AR4.py
    s.configure("OK.TLabel", foreground=c["success"], font=F(10, "bold"))
    s.configure("Warn.TLabel", foreground=c["caution"], font=F(10, "bold"))
    s.configure("Alarm.TLabel", foreground=c["danger"], font=F(10, "bold"))
    s.configure("Jointlim.TLabel", foreground=c["muted"], font=F(8))
    s.configure("Frame1.TFrame", background=c["fg"])

    # Header / brand bar
    s.configure("Header.TFrame", background=c["chrome"])
    s.configure("HeaderRule.TFrame", background=c["border"])
    s.configure("HeaderTitle.TLabel", background=c["chrome"], foreground=c["fg"], font=F(13, "bold"))
    s.configure("HeaderMuted.TLabel", background=c["chrome"], foreground=c["muted"], font=F(10))
    s.configure("HeaderKey.TLabel", background=c["chrome"], foreground=c["fg"], font=F(10, "bold"))
    s.configure("Chip.TFrame", background=c["chrome"])


def _windows_titlebar(root, dark):
    """Match the Windows 10/11 title bar to the theme."""
    if not IS_WINDOWS:
        return
    try:
        root.update_idletasks()
        hwnd = ctypes.windll.user32.GetParent(root.winfo_id())
        val = ctypes.c_int(1 if dark else 0)
        for attr in (20, 19):   # DWMWA_USE_IMMERSIVE_DARK_MODE (new, old builds)
            if ctypes.windll.dwmapi.DwmSetWindowAttribute(hwnd, attr, ctypes.byref(val), 4) == 0:
                break
    except Exception:
        pass


def vtk_colors():
    """Background gradient and floor-grid colours for the virtual robot viewer (0..1 RGB)."""
    def rgb(h):
        h = h.lstrip("#")
        return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))
    return {"top": rgb(C["vtk_top"]), "bottom": rgb(C["vtk_bottom"]), "grid": rgb(C["vtk_grid"])}


# ---------------------------------------------------------------------------- header
class Header(ttk.Frame):
    """One bar for the whole window: logo, the page tabs, live connection status.

    The notebook's own tab strip is hidden (style "Bare.TNotebook"); attach(nb) builds
    the tabs here instead, so branding costs no extra height.

    status_fn() -> dict(teensy=(connected, port), io=(connected, port),
                        offline=bool, running=bool)
    """

    POLL_MS = 600

    def __init__(self, master, status_fn=None, **kw):
        super().__init__(master, style="Header.TFrame", **kw)
        skip_restyle(self)
        self.status_fn = status_fn
        self.nb = None
        self.tabs = []                     # (tab widget id, _NavTab)
        self.columnconfigure(0, weight=1)

        bar = self.bar = ttk.Frame(self, style="Header.TFrame", padding=(16, 0, 16, 0))
        bar.grid(row=0, column=0, sticky="ew")
        bar.columnconfigure(3, weight=1)

        brand = ttk.Frame(bar, style="Header.TFrame")
        brand.grid(row=0, column=0, sticky="w", pady=7)
        self.logo = ttk.Label(brand, style="HeaderTitle.TLabel")
        self.logo.grid(row=0, column=0, padx=(0, 10))
        ttk.Label(brand, text=APP_NAME, style="HeaderTitle.TLabel").grid(row=0, column=1, sticky="w")
        ttk.Label(brand, text=APP_VERSION, style="HeaderMuted.TLabel").grid(
            row=0, column=2, sticky="w", padx=(6, 0), pady=(3, 0))

        self.sep = _tkw(tk.Frame, bar, width=1, height=22)
        self.sep.grid(row=0, column=1, padx=(18, 10))
        self.nav = _tkw(tk.Frame, bar)
        self.nav.grid(row=0, column=2, sticky="nsw")
        self.nav.rowconfigure(0, weight=1)

        chips = ttk.Frame(bar, style="Header.TFrame")
        chips.grid(row=0, column=4, sticky="e")
        self.mode_chip = _Chip(chips, "")
        self.mode_chip.grid(row=0, column=0, padx=(0, 20))
        self.teensy = _Chip(chips, "Teensy")
        self.teensy.grid(row=0, column=1, padx=(0, 20))
        self.io = _Chip(chips, "IO board")
        self.io.grid(row=0, column=2)

        ttk.Frame(self, style="HeaderRule.TFrame", height=1).grid(row=1, column=0, sticky="ew")

        on_theme_change(self._retheme)
        self._retheme(MODE["name"])
        self.after(300, self._poll)

    # -- tabs
    def attach(self, nb):
        """Mirror nb's pages as header tabs (call after all pages are added)."""
        self.nb = nb
        for child in self.nav.winfo_children():
            child.destroy()
        self.tabs = []
        for i, tab_id in enumerate(nb.tabs()):
            item = _NavTab(self.nav, nb.tab(tab_id, "text").strip(),
                           command=lambda t=tab_id: self._select(t))
            item.grid(row=0, column=i, sticky="ns")
            self.tabs.append((tab_id, item))
        nb.bind("<<NotebookTabChanged>>", lambda e: self._sync(), add="+")
        try:
            nb.enable_traversal()            # Ctrl+Tab / Ctrl+Shift+Tab still switch pages
        except tk.TclError:
            pass
        self._sync()

    def _select(self, tab_id):
        if self.nb is not None:
            self.nb.select(tab_id)

    def _sync(self):
        if self.nb is None:
            return
        try:
            cur = self.nb.select()
        except tk.TclError:
            return
        for tab_id, item in self.tabs:
            item.set_selected(str(tab_id) == str(cur))

    def _retheme(self, mode):
        self.logo.configure(image=_img("logo-light.png" if mode == "dark" else "logo-dark.png", self))
        self.sep.configure(bg=C["border_strong"])
        self.nav.configure(bg=C["chrome"])
        for _, item in self.tabs:
            item.retheme()
        for chip in (self.mode_chip, self.teensy, self.io):
            chip.retheme()

    def _poll(self):
        try:
            st = self.status_fn() if self.status_fn else {}
            ok, port = st.get("teensy", (False, ""))
            self.teensy.set("success" if ok else "danger", port if ok and port else "Not connected")
            ok, port = st.get("io", (False, ""))
            self.io.set("success" if ok else "faint", port if ok and port else "Not connected")
            if st.get("running"):
                self.mode_chip.set("accent", "Program running", key="")
                self.mode_chip.grid()
            elif st.get("offline"):
                self.mode_chip.set("caution", "Offline mode", key="")
                self.mode_chip.grid()
            else:
                self.mode_chip.grid_remove()
        except Exception as e:
            logger.debug(f"header poll: {e}")
        self.after(self.POLL_MS, self._poll)


class _Chip(ttk.Frame):
    """Status dot + bold key + muted value."""

    def __init__(self, master, key):
        super().__init__(master, style="Chip.TFrame")
        self.dot = _tkw(tk.Canvas, self, width=10, height=10, highlightthickness=0, bd=0)
        self.dot.grid(row=0, column=0, padx=(0, 7), pady=(2, 0))
        self.key = ttk.Label(self, text=key, style="HeaderKey.TLabel")
        self.key.grid(row=0, column=1)
        self.val = ttk.Label(self, text="", style="HeaderMuted.TLabel")
        self.val.grid(row=0, column=2, padx=(6, 0))
        self._color = "faint"
        self._key = key

    def set(self, color, value, key=None):
        if key is not None and key != self._key:
            self._key = key
            self.key.configure(text=key)
            self.key.grid() if key else self.key.grid_remove()
        self.val.configure(text=value, style="HeaderKey.TLabel" if not self._key else "HeaderMuted.TLabel")
        if color != self._color:
            self._color = color
            self._draw()

    def retheme(self):
        self.dot.configure(bg=C["chrome"])
        self._draw()

    def _draw(self):
        self.dot.delete("all")
        self.dot.create_oval(1, 1, 9, 9, fill=C[self._color], outline="")


class _NavTab(tk.Frame):
    """A page tab in the header: label with a 3px marker underneath when selected."""

    def __init__(self, master, text, command):
        if _supports_autostyle():
            super().__init__(master, autostyle=False)
        else:
            super().__init__(master)
        self.command = command
        self.selected = False
        self.hover = False
        self.label = _tkw(tk.Label, self, text=text, padx=12, pady=0, cursor="hand2", bd=0)
        self.label.pack(side="top", fill="both", expand=True)
        self.mark = _tkw(tk.Frame, self, height=3)
        self.mark.pack(side="bottom", fill="x", padx=8)
        for w in (self, self.label):
            w.bind("<Button-1>", self._click)
            w.bind("<Enter>", lambda e: self._hover(True))
            w.bind("<Leave>", lambda e: self._hover(False))
        self.retheme()

    def _click(self, _e=None):
        self.command()

    def _hover(self, on):
        self.hover = on
        self.retheme()

    def set_selected(self, on):
        if on != self.selected:
            self.selected = on
            self.retheme()

    def retheme(self):
        bg = C["chrome"]
        fg = C["fg"] if (self.selected or self.hover) else C["muted"]
        self.configure(bg=bg)
        self.label.configure(bg=bg, fg=fg, activebackground=bg, font=F(10, "bold"))
        self.mark.configure(bg=C["accent"] if self.selected else
                            (C["border_strong"] if self.hover else bg))


# --------------------------------------------------------------- classic Tk widgets
# OptionMenu, Menu, Text and Canvas are plain Tk (not ttk), so ttk styles miss them and
# ttkbootstrap paints them in its primary colour. Recolour the whole tree on each switch.
_SKIP = set()   # widgets that manage their own colours (header, splash)


def skip_restyle(*widgets):
    for w in widgets:
        _SKIP.add(str(w))


_CHEVRONS = {}
_MB_CHARS = {}   # OptionMenu -> its original width in characters


def _chevron(color, master):
    """Small down-chevron image for classic OptionMenus (matches the ttk dropdown arrow)."""
    key = (color, str(master.winfo_toplevel()))
    if key not in _CHEVRONS:
        from PIL import ImageDraw
        ss, w, h = 4, 9, 5
        im = Image.new("RGBA", (w * ss, h * ss), (0, 0, 0, 0))
        rgb = tuple(int(color.lstrip("#")[i:i + 2], 16) for i in (0, 2, 4))
        ImageDraw.Draw(im).polygon([(0, 0), (w * ss, 0), (w * ss / 2, h * ss)], fill=rgb + (255,))
        _CHEVRONS[key] = ImageTk.PhotoImage(im.resize((w, h), Image.LANCZOS),
                                            master=master.winfo_toplevel())
    return _CHEVRONS[key]


def _restyle_one(w, cls):
    c = C
    if cls == "Menubutton":
        w.configure(bg=c["raised"], fg=c["fg"], activebackground=c["raised_hover"],
                    activeforeground=c["fg"], highlightthickness=1,
                    highlightbackground=c["border"], highlightcolor=c["info"],
                    relief="flat", bd=0, padx=8, pady=3, font=F(10))
        try:   # chevron instead of Tk's old raised-bar indicator
            key = str(w)
            if key not in _MB_CHARS:     # width counts characters until an image is set
                _MB_CHARS[key] = int(w.cget("width") or 0)
            chars = _MB_CHARS[key]
            px = 0
            if chars > 0:                # ...and pixels afterwards: keep the same size
                px = tkfont.Font(font=F(10)).measure("0") * chars + 9 + 6
            w.configure(indicatoron=0, image=_chevron(c["muted"], w), compound="right",
                        width=px)
        except tk.TclError:
            pass
    elif cls == "Menu":
        w.configure(bg=c["raised"], fg=c["fg"], activebackground=c["select"],
                    activeforeground=c["select_fg"], selectcolor=c["accent"],
                    relief="flat", bd=1, activeborderwidth=0, font=F(10))
    elif cls == "Text":
        w.configure(bg=c["surface"], fg=c["fg"], insertbackground=c["fg"],
                    selectbackground=c["select"], selectforeground=c["select_fg"],
                    highlightthickness=1, highlightbackground=c["border"],
                    highlightcolor=c["info"], relief="flat", bd=0)
    elif cls == "Canvas":
        w.configure(bg=c["bg"], highlightthickness=0)
    elif cls == "Scrollbar":       # classic tk scrollbars (rare)
        w.configure(bg=c["raised"], troughcolor=c["surface"], activebackground=c["raised_hover"],
                    highlightthickness=0, bd=0)


def restyle_tree(widget):
    """Apply the current palette to every classic Tk widget under `widget`."""
    stack = [widget]
    while stack:
        w = stack.pop()
        try:
            name = str(w)
            if name not in _SKIP:
                cls = w.winfo_class()
                if cls in ("Menubutton", "Menu", "Text", "Canvas", "Scrollbar"):
                    try:
                        _restyle_one(w, cls)
                    except tk.TclError:
                        pass
                    if cls == "Menubutton":
                        try:
                            stack.append(w.nametowidget(w.cget("menu")))
                        except (tk.TclError, KeyError):
                            pass
            else:
                continue
            stack.extend(w.winfo_children())
        except tk.TclError:
            continue


# ------------------------------------------------------------------ scrollable panels
_SCROLLERS = {}   # str(canvas) -> (canvas, scrollbar)


class _ScrollInner(ttk.Frame):
    """The frame callers fill with widgets. Geometry calls on it (grid/pack/place) are
    forwarded to the outer container, so existing layout code works unchanged."""

    def grid(self, *a, **k):
        return self._outer.grid(*a, **k)

    grid_configure = grid

    def pack(self, *a, **k):
        return self._outer.pack(*a, **k)

    pack_configure = pack

    def place(self, *a, **k):
        return self._outer.place(*a, **k)

    place_configure = place

    def grid_remove(self):
        return self._outer.grid_remove()

    def grid_forget(self):
        return self._outer.grid_forget()

    def pack_forget(self):
        return self._outer.pack_forget()


def scroll_frame(parent, **frame_kw):
    """A frame that grows a vertical scrollbar when the window is too short for it.

    Returns the inner frame: put widgets in it and grid/pack it as usual. To add it to a
    notebook use nb.add(ui.page(frame), ...). Width follows the content; height follows
    the space the parent gives it, and the content is stretched to fill when it fits
    (so row/column weights inside still behave).
    """
    outer = ttk.Frame(parent)
    outer.rowconfigure(0, weight=1)
    outer.columnconfigure(0, weight=1)
    canvas = _tkw(tk.Canvas, outer, highlightthickness=0, bd=0, height=40,
                  bg=C["bg"], yscrollincrement=24)
    canvas.grid(row=0, column=0, sticky="nsew")
    vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
    canvas.configure(yscrollcommand=vsb.set)

    inner = _ScrollInner(canvas, **frame_kw)
    inner._outer = outer
    win = canvas.create_window(0, 0, window=inner, anchor="nw")
    state = {"busy": False}

    def relayout(_e=None):
        if state["busy"]:
            return
        state["busy"] = True
        try:
            rw, rh = inner.winfo_reqwidth(), inner.winfo_reqheight()
            cw, ch = canvas.winfo_width(), canvas.winfo_height()
            if int(canvas.cget("width")) != rw:
                canvas.configure(width=rw)          # the column keeps the content's width
            w, h = max(cw, rw), max(ch, rh)
            canvas.itemconfigure(win, width=w, height=h)
            canvas.configure(scrollregion=(0, 0, w, h))
            if ch > 1 and rh > ch + 1:
                vsb.grid(row=0, column=1, sticky="ns")
            else:
                vsb.grid_remove()
                canvas.yview_moveto(0)
        finally:
            state["busy"] = False

    inner.bind("<Configure>", relayout, add="+")
    canvas.bind("<Configure>", relayout, add="+")
    _SCROLLERS[str(canvas)] = (canvas, vsb)
    return inner


def page(frame):
    """The widget to hand to Notebook.add() for a scroll_frame (or any plain frame)."""
    return getattr(frame, "_outer", frame)


_NO_WHEEL = {"Listbox", "Text", "TCombobox", "Treeview", "Scale", "TScale", "Canvas"}


def _on_wheel(event):
    try:
        w = event.widget.winfo_containing(event.x_root, event.y_root)
    except (tk.TclError, AttributeError, KeyError):
        return
    first = True
    while w is not None:
        key = str(w)
        if key in _SCROLLERS:
            canvas, vsb = _SCROLLERS[key]
            if not vsb.winfo_ismapped():
                return
            if getattr(event, "num", None) == 4:
                step = -1
            elif getattr(event, "num", None) == 5:
                step = 1
            else:
                d = event.delta
                step = -int(d / 120) if abs(d) >= 120 else (-1 if d > 0 else 1)
            canvas.yview_scroll(step, "units")
            return "break"
        if first and w.winfo_class() in _NO_WHEEL:
            return          # widgets that scroll themselves keep the wheel
        first = False
        w = w.master


def _bind_wheel(root):
    for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
        try:
            root.bind_all(seq, _on_wheel, add="+")
        except tk.TclError:
            pass


_AUTOSTYLE = {}


def _supports_autostyle():
    """ttkbootstrap restyles plain tk widgets; autostyle=False opts out where supported."""
    if "ok" not in _AUTOSTYLE:
        try:
            import inspect
            import ttkbootstrap  # noqa: F401  (patches tk widget constructors)
            _AUTOSTYLE["ok"] = "autostyle" in inspect.getsource(tk.Canvas.__init__)
        except Exception:
            _AUTOSTYLE["ok"] = False
    return _AUTOSTYLE["ok"]


def _tkw(cls, master, **kw):
    """Create a plain tk widget that keeps our explicit colours under ttkbootstrap."""
    if _supports_autostyle():
        kw["autostyle"] = False
    return cls(master, **kw)


# ---------------------------------------------------------------------------- splash
#
# Splash(root, message, modal=False) returns one of two implementations with the same
# interface (step / set_message / start / sweep_done / skipped / close / closed):
#
#   _ProcSplash  the window lives in its own process (ar4_splash.py) with its own Tk loop,
#                so the robot animation keeps moving while this process is busy building
#                the interface, waiting on the Teensy or running a calibration. Default.
#   _TkSplash    the old in-process Toplevel. Used when the child process cannot be
#                started, or with AR4_SPLASH_INPROCESS=1.


def _splash_child_command():
    """Command line that runs ar4_splash.py with the same interpreter as this program."""
    if getattr(sys, "frozen", False):
        return [sys.executable, "--splash-child"]          # AR4.py handles this flag at the top
    return [sys.executable, os.path.join(HERE, "ar4_splash.py")]


class _ProcSplash:
    """Splash window in a separate process; see ar4_splash.py for the protocol."""

    FPS = 30

    def __init__(self, root, message="Starting", modal=False):
        self.root = root
        self.modal = modal
        self.closed = False
        self.skipped = False
        self._started = None
        self._lock = threading.Lock()
        try:
            self.nframes = getattr(Image.open(os.path.join(ASSETS, "splash.gif")), "n_frames", 1)
        except Exception:
            self.nframes = 1

        args = _splash_child_command() + ["--message", message]
        if modal and root.winfo_viewable():
            cx = root.winfo_rootx() + root.winfo_width() // 2
            cy = root.winfo_rooty() + root.winfo_height() // 2
            args += ["--center", f"{cx},{cy}"]
        kw = {}
        if IS_WINDOWS:
            kw["creationflags"] = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.proc = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, text=True, bufsize=1,
                                     cwd=HERE, **kw)
        self._ready = threading.Event()
        threading.Thread(target=self._reader, daemon=True).start()
        # Give the child a moment to map its window so nothing flashes before it; it is
        # already animating on its own by then.
        if not self._ready.wait(3.0):
            if self.proc.poll() is not None:
                raise RuntimeError("splash process exited early")

    def _reader(self):
        try:
            for line in self.proc.stdout:
                line = line.strip()
                if line == "ready":
                    self._ready.set()
                elif line == "skip":
                    self.skipped = True
        except Exception:
            pass
        self._ready.set()

    def _send(self, text):
        if self.closed:
            return
        with self._lock:
            try:
                self.proc.stdin.write(text + "\n")
                self.proc.stdin.flush()
            except Exception as e:
                logger.debug(f"splash process not reachable: {e}")

    def step(self, message=None):
        if message:
            self._send("msg " + message.replace("\n", " "))

    def set_message(self, message):
        self.step(message)

    def start(self):
        if self._started is None:
            self._started = time.monotonic()

    @property
    def sweep_done(self):
        if self._started is None:
            return False
        return (time.monotonic() - self._started) >= self.nframes / self.FPS

    def close(self):
        if self.closed:
            return
        self._send("close")
        self.closed = True
        try:
            self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.wait(timeout=1.0)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass


def Splash(root, message="Starting", modal=False):
    """Animated splash that keeps running while the program is busy (see above)."""
    if os.environ.get("AR4_SPLASH_INPROCESS", "").lower() not in ("1", "true", "yes"):
        try:
            return _ProcSplash(root, message, modal)
        except Exception as e:
            logger.debug(f"splash process unavailable, using in-process splash: {e}")
    return _TkSplash(root, message, modal)


class _TkSplash:
    """Borderless launch window: the AR logo with the robot arm sweeping across it.

    In-process fallback: the animation holds its first frame while the interface is
    built, plays once the event loop runs (start()), and finish() closes it no earlier
    than the end of one sweep. Clicking the splash skips the wait.
    """

    FPS = 30
    BG = PALETTES["dark"]["chrome"]

    def __init__(self, root, message="Starting", modal=False):
        self.root = root
        self.modal = modal
        self.closed = False
        self.skipped = False
        self._started = None
        self._after = None
        self._t0 = time.monotonic()

        bg, fg, muted = self.BG, PALETTES["dark"]["fg"], PALETTES["dark"]["muted"]
        w = self.win = _tkw(tk.Toplevel, root, bg=bg, highlightthickness=1,
                            highlightbackground=PALETTES["dark"]["border"])
        skip_restyle(w)
        w.withdraw()
        w.overrideredirect(True)
        try:
            w.attributes("-topmost", not modal)
        except tk.TclError:
            pass
        set_window_icon(w, default=False)

        try:
            self.gif = Image.open(os.path.join(ASSETS, "splash.gif"))
            self.nframes = getattr(self.gif, "n_frames", 1)
            self.photo = ImageTk.PhotoImage(self.gif.convert("RGB"), master=w)
        except Exception as e:
            logger.debug(f"splash animation unavailable: {e}")
            self.gif, self.nframes = None, 1
            self.photo = _img("logo-light-2x.png", w)
        self.frame = 0

        pad = 40
        self.anim = _tkw(tk.Label, w, image=self.photo, bg=bg, bd=0)
        self.anim.pack(padx=pad, pady=(pad, 26))

        row = _tkw(tk.Frame, w, bg=bg)
        row.pack(fill="x", padx=pad)
        _tkw(tk.Label, row, text=APP_NAME, bg=bg, fg=fg, font=F(16, "bold")).pack(side="left")
        _tkw(tk.Label, row, text=f"Software {APP_VERSION}", bg=bg, fg=muted, font=F(11)).pack(
            side="right", pady=(4, 0))

        self.msg = _tkw(tk.Label, w, text=message, bg=bg, fg=muted, font=F(11), anchor="w")
        self.msg.pack(fill="x", padx=pad, pady=(4, 14))

        self.bar_w = self.photo.width()
        self.bar = _tkw(tk.Canvas, w, width=self.bar_w, height=3, bg=PALETTES["dark"]["border"],
                        highlightthickness=0, bd=0)
        self.bar.pack(padx=pad, pady=(0, pad))
        self.seg = self.bar.create_rectangle(0, 0, 0, 3, fill=PALETTES["dark"]["accent"], width=0)

        for wid in (w, self.anim, self.msg):
            wid.bind("<Button-1>", self._skip)

        w.update_idletasks()
        sw, sh = w.winfo_reqwidth(), w.winfo_reqheight()
        if modal and root.winfo_viewable():
            x = root.winfo_rootx() + (root.winfo_width() - sw) // 2
            y = root.winfo_rooty() + (root.winfo_height() - sh) // 2
        else:
            x = (w.winfo_screenwidth() - sw) // 2
            y = (w.winfo_screenheight() - sh) // 2
        w.geometry(f"+{max(0, x)}+{max(0, y)}")
        w.deiconify()
        if modal:
            try:
                w.transient(root)
                w.grab_set()
            except tk.TclError:
                pass
        w.update()

    # -- progress while the UI is built (event loop not running yet)
    def step(self, message=None):
        if self.closed:
            return
        if message:
            self.msg.configure(text=message)
        self._draw_bar(time.monotonic() - self._t0)
        self.win.update_idletasks()

    def set_message(self, message):
        if not self.closed:
            self.msg.configure(text=message)

    # -- animation
    def start(self):
        if self._started is None:
            self._started = time.monotonic()
            self._tick()

    def _tick(self):
        if self.closed:
            return
        t = time.monotonic() - self._started
        target = int(t * self.FPS)
        if self.gif is not None and target != self.frame:
            target = min(target, self.nframes - 1)   # hold the final pose
            try:
                if target < self.frame:
                    self.gif.seek(0)
                    self.frame = 0
                while self.frame < target:
                    self.frame += 1
                    self.gif.seek(self.frame)
                self.photo.paste(self.gif.convert("RGB"))
            except EOFError:
                pass
        self._draw_bar(time.monotonic() - self._t0)
        self._after = self.win.after(16, self._tick)

    def _draw_bar(self, t):
        # an orange segment sliding with ease-in-out, 1.6 s per pass
        period = 1.6
        p = (t % period) / period
        e = p * p * (3 - 2 * p)
        seg = self.bar_w * 0.28
        x = -seg + e * (self.bar_w + seg)
        self.bar.coords(self.seg, x, 0, x + seg, 3)

    @property
    def sweep_done(self):
        if self._started is None:
            return False
        return (time.monotonic() - self._started) >= self.nframes / self.FPS

    def _skip(self, _evt=None):
        self.skipped = True

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self._after:
            try:
                self.win.after_cancel(self._after)
            except tk.TclError:
                pass
        try:
            self.win.grab_release()
        except tk.TclError:
            pass
        try:
            self.win.destroy()
        except tk.TclError:
            pass


class _Launch:
    """Tracks the launch splash so other code can reuse or dismiss it."""
    splash = None
    shown = False


def show_launch_splash(root):
    """Withdraw the main window and show the animated splash while AR4 builds its UI."""
    root.withdraw()
    _Launch.splash = Splash(root, "Building the interface")
    _guard_messageboxes(root)
    return _Launch.splash


def reveal_main_window(root):
    """Close the launch splash (if any) and show the main window."""
    sp = _Launch.splash
    _Launch.splash = None
    if sp:
        sp.close()
    if not _Launch.shown:
        _Launch.shown = True
        restyle_tree(root)
        root.deiconify()
        maximize(root)
        root.lift()
        try:
            root.focus_force()
        except tk.TclError:
            pass


def maximize(root):
    """Open full-size: every tab has room for all of its panels. AR4_WINDOWED=1 opts out."""
    if os.environ.get("AR4_WINDOWED", "").lower() in ("1", "true", "yes"):
        return
    try:
        if IS_WINDOWS or sys.platform == "darwin":
            root.state("zoomed")
        else:
            root.attributes("-zoomed", True)
    except tk.TclError:
        pass


def run_launch(root, connect_fn, message="Connecting to the robot controller"):
    """Play the splash, run connect_fn() (e.g. AR4's setCom), then reveal the window
    once the connection attempt is over and one robot sweep has played (or on click)."""
    sp = _Launch.splash

    def finish():
        s = _Launch.splash
        if s is None or s.sweep_done or s.skipped:
            reveal_main_window(root)
        else:
            root.after(40, finish)

    def go():
        if sp:
            sp.start()
            sp.set_message(message)
        try:
            connect_fn()
        except Exception:
            logger.exception("Startup connection failed")
        finally:
            finish()

    root.after(30, go)


class busy:
    """Progress window for long operations (Teensy startup handshake, auto calibration).

    Reuses the launch splash while it is still up; otherwise shows a copy over the main
    window. The window animates in its own process, so it keeps moving even while this
    program blocks on the serial port.

        progress = ui.busy(root, "Calibrating J1")
        try: ...
        finally: progress.close()
    """

    def __init__(self, root, message):
        self.own = _Launch.splash is None
        if self.own:
            self.splash = Splash(root, message, modal=True)
            self.splash.start()
        else:
            self.splash = _Launch.splash
            self.splash.set_message(message)

    def set_message(self, message):
        self.splash.set_message(message)

    def close(self):
        if self.own:
            self.splash.close()


def _guard_messageboxes(root):
    """Dialogs must never hide behind the launch splash: reveal the window first."""
    from tkinter import messagebox
    if getattr(messagebox, "_ar4_guarded", False):
        return
    for name in ("showinfo", "showwarning", "showerror", "askquestion", "askokcancel",
                 "askyesno", "askyesnocancel", "askretrycancel"):
        orig = getattr(messagebox, name, None)
        if orig is None:
            continue

        def wrapper(*a, _orig=orig, **k):
            if _Launch.splash is not None:
                reveal_main_window(root)
            return _orig(*a, **k)
        setattr(messagebox, name, wrapper)
    messagebox._ar4_guarded = True
