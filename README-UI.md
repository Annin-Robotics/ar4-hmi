# AR4 modern UI

A new look for the AR4 HMI (software 8.0): graphite (dark) and aluminium (light)
themes, the new AR logo, an animated launch splash built from the robot animation,
and layout fixes so every panel stays reachable on any screen size.

## Install

Copy these into your `ar4-hmi-dev` folder (next to the existing `AR4.py`):

```
AR4.py            already patched (cross-platform + modern UI)
ar4_ui.py         themes, fonts, header, splash, scrolling panels
assets/           logo, icons, splash animation, Barlow font
tools/            install_launcher.py, build_assets.py
make_modern_ui.py the patch on its own (see below)
```

Then run it the same way as before:

```bash
.venv/bin/python AR4.py
```

No new packages are needed (Pillow and ttkbootstrap are already installed). Keep
ttkbootstrap on 1.x: `pip install "ttkbootstrap<2"`.

**If your AR4.py has your own changes**, don't overwrite it. Copy everything except
`AR4.py`, then apply just the UI changes to your file:

```bash
.venv/bin/python make_modern_ui.py --check   # dry run
.venv/bin/python make_modern_ui.py           # patch, keeps a timestamped backup
```

It only writes if every change matches and the result compiles.

## Add it to your app menu (optional)

```bash
.venv/bin/python tools/install_launcher.py          # Linux: KDE/GNOME menu entry + icon
.venv/bin/python tools/install_launcher.py --remove
```

On Windows the same command creates a Start menu shortcut using `pythonw.exe`, so no
console window opens.

## What changed

- **Header bar**: logo, page tabs and live status in one strip. The status shows
  whether the Teensy and the IO board are connected (with their port), plus
  "Offline mode" or "Program running" while those are active.
- **Colour means something**: orange is only used on controls that move the robot
  (jog buttons, sliders, Run); red is Stop and alarms; green is connected/OK; amber
  is warnings and offline mode. Everything else is neutral.
- **Run / Step back / Step forward / Stop** are large labelled buttons.
- **Scrolling**: the side panels on Main Controls and the System Setup, Robot
  Parameters, IO and Registers pages scroll with the mouse wheel when the window
  is shorter than their content, instead of cutting off the bottom rows.
- **Starts maximized**. Set `AR4_WINDOWED=1` to keep the old 1600x900 window.
- **Splash**: the black robot arm sweeps across the AR logo while the interface
  loads and the Teensy connects. Click it to skip. Error dialogs are never hidden
  behind it.
- **Light/Dark** buttons (System Setup > Theme) switch everything live, including
  the program listing, dropdowns, log and the virtual robot background.
- **Icon**: new window, taskbar and Alt-Tab icon on Windows and Linux.
- **Log page**: tracebacks show one line per row, and the list fills the page.
- **Font**: Barlow Semi Condensed is bundled (SIL Open Font License, see
  `assets/fonts/OFL.txt`) and loaded for this program only; nothing is installed.

## Rebuilding the assets

If you change the logo or the animation:

```bash
.venv/bin/python tools/build_assets.py --svg new-logo.svg --video new-animation.mov --out assets
```

Needs ffmpeg on your PATH for the video. It regenerates the logo PNGs, the icon set
(`icon-*.png`, `ar4.ico`) and `splash.gif`.
