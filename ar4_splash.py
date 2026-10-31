"""
ar4_splash.py - the AR4 splash window, run as its own process.

ar4_ui.Splash() starts this with the interpreter that runs AR4.py:

    python ar4_splash.py --message "Building the interface" [--center X,Y]
    AR4.exe --splash-child --message "..."            (frozen build, see top of AR4.py)

Why a separate process: building the interface, the Teensy handshake and the auto
calibration all block the main program's Tk loop, so a splash drawn by that loop froze
mid-frame. This process has its own Tk loop and keeps the robot animation and progress
bar moving no matter how busy the main program is.

Protocol (one command per line):
    stdin  <- "msg <text>"       change the message under the animation
              "center <x> <y>"   move the window so its centre is at that screen point
              "close"            exit
              (stdin closing also exits: the parent is gone)
    stdout -> "ready"            window is on screen
              "skip"             the user clicked the splash
Nothing else in the program depends on this module.
"""

import os
import queue
import sys
import threading
import time
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "assets")

# Same values as ar4_ui.PALETTES["dark"] (kept local so this process imports nothing heavy)
BG = "#13161b"
FG = "#e7eaee"
MUTED = "#8a94a2"
BORDER = "#2e343d"
ACCENT = "#ff7a1a"
APP_NAME = "AR4 Robot Control"
APP_VERSION = "8.1"
FPS = 30


def _font(size, weight="normal"):
    for fam in ("Barlow Semi Condensed", "Segoe UI", "Noto Sans", "Cantarell", "DejaVu Sans", "Helvetica"):
        try:
            import tkinter.font as tkfont
            if fam in tkfont.families():
                return (fam, size, weight)
        except Exception:
            break
    return ("TkDefaultFont", size, weight)


class SplashWindow:
    def __init__(self, message, center=None):
        self.root = tk.Tk(className="AR4Splash")
        w = self.root
        w.withdraw()
        w.overrideredirect(True)
        w.configure(bg=BG, highlightthickness=1, highlightbackground=BORDER)
        try:
            w.attributes("-topmost", True)
        except tk.TclError:
            pass
        try:
            ico = os.path.join(ASSETS, "icon-64.png")
            if os.path.isfile(ico):
                self._icon = tk.PhotoImage(file=ico, master=w)
                w.iconphoto(True, self._icon)
        except Exception:
            pass

        self.gif = None
        self.nframes = 1
        self.frame = 0
        try:
            from PIL import Image, ImageTk
            self.gif = Image.open(os.path.join(ASSETS, "splash.gif"))
            self.nframes = getattr(self.gif, "n_frames", 1)
            self.photo = ImageTk.PhotoImage(self.gif.convert("RGB"), master=w)
        except Exception:
            self.gif = None
            try:
                self.photo = tk.PhotoImage(file=os.path.join(ASSETS, "logo-light-2x.png"), master=w)
            except Exception:
                self.photo = tk.PhotoImage(width=600, height=200, master=w)

        pad = 40
        self.anim = tk.Label(w, image=self.photo, bg=BG, bd=0)
        self.anim.pack(padx=pad, pady=(pad, 26))

        row = tk.Frame(w, bg=BG)
        row.pack(fill="x", padx=pad)
        tk.Label(row, text=APP_NAME, bg=BG, fg=FG, font=_font(16, "bold")).pack(side="left")
        tk.Label(row, text=f"Software {APP_VERSION}", bg=BG, fg=MUTED, font=_font(11)).pack(side="right", pady=(4, 0))

        self.msg = tk.Label(w, text=message, bg=BG, fg=MUTED, font=_font(11), anchor="w")
        self.msg.pack(fill="x", padx=pad, pady=(4, 14))

        self.bar_w = self.photo.width()
        self.bar = tk.Canvas(w, width=self.bar_w, height=3, bg=BORDER, highlightthickness=0, bd=0)
        self.bar.pack(padx=pad, pady=(0, pad))
        self.seg = self.bar.create_rectangle(0, 0, 0, 3, fill=ACCENT, width=0)

        for wid in (w, self.anim, self.msg):
            wid.bind("<Button-1>", self._skip)

        w.update_idletasks()
        self.place(center)
        w.deiconify()
        w.lift()

        self.t0 = time.monotonic()
        self.cmds = queue.Queue()
        threading.Thread(target=self._read_stdin, daemon=True).start()
        self._tick()
        self._poll()
        _say("ready")

    def place(self, center):
        w = self.root
        sw, sh = w.winfo_reqwidth(), w.winfo_reqheight()
        if center:
            x, y = int(center[0] - sw / 2), int(center[1] - sh / 2)
        else:
            x = (w.winfo_screenwidth() - sw) // 2
            y = (w.winfo_screenheight() - sh) // 2
        w.geometry(f"+{max(0, x)}+{max(0, y)}")

    # -- stdin commands -------------------------------------------------------------------
    def _read_stdin(self):
        try:
            for line in sys.stdin:
                self.cmds.put(line.rstrip("\r\n"))
        except Exception:
            pass
        self.cmds.put("close")

    def _poll(self):
        try:
            while True:
                line = self.cmds.get_nowait()
                if line == "close":
                    self.root.destroy()
                    return
                if line.startswith("msg "):
                    self.msg.configure(text=line[4:])
                elif line.startswith("center "):
                    try:
                        _, x, y = line.split()
                        self.place((float(x), float(y)))
                    except ValueError:
                        pass
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    # -- animation --------------------------------------------------------------------------
    def _tick(self):
        t = time.monotonic() - self.t0
        if self.gif is not None:
            target = int(t * FPS) % self.nframes          # loops: long waits keep moving
            if target != self.frame:
                try:
                    if target < self.frame:
                        self.gif.seek(0)
                        self.frame = 0
                    while self.frame < target:
                        self.frame += 1
                        self.gif.seek(self.frame)
                    self.photo.paste(self.gif.convert("RGB"))
                except Exception:
                    pass
        period = 1.6
        p = (t % period) / period
        e = p * p * (3 - 2 * p)
        seg = self.bar_w * 0.28
        x = -seg + e * (self.bar_w + seg)
        self.bar.coords(self.seg, x, 0, x + seg, 3)
        self.root.after(16, self._tick)

    def _skip(self, _evt=None):
        _say("skip")


def _say(text):
    try:
        sys.stdout.write(text + "\n")
        sys.stdout.flush()
    except Exception:
        pass


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    message = "Starting"
    center = None
    i = 0
    while i < len(argv):
        if argv[i] == "--message" and i + 1 < len(argv):
            message = argv[i + 1]
            i += 2
        elif argv[i] == "--center" and i + 1 < len(argv):
            try:
                center = tuple(float(v) for v in argv[i + 1].split(",")[:2])
            except ValueError:
                center = None
            i += 2
        else:
            i += 1
    try:
        win = SplashWindow(message, center)
        win.root.mainloop()
    except tk.TclError:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
