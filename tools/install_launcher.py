#!/usr/bin/env python3
"""
install_launcher.py - add AR4 Robot Control to your app menu, with its icon.

    python tools/install_launcher.py              # install
    python tools/install_launcher.py --remove     # uninstall

Linux:   writes ~/.local/share/applications/ar4-robot-control.desktop and the icon set
         under ~/.local/share/icons. It runs AR4.py with the project's .venv if there
         is one, so it works the same as `.venv/bin/python AR4.py`.
Windows: creates a Start menu shortcut (AR4 Robot Control.lnk) that uses pythonw.exe
         from .venv if present, so no console window opens.
Nothing outside your user profile is touched; no admin rights needed.
"""
import os
import shutil
import subprocess
import sys

APP_ID = "ar4-robot-control"
APP_NAME = "AR4 Robot Control"
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # folder with AR4.py
ASSETS = os.path.join(ROOT, "assets")


def find_python(windowed=False):
    if sys.platform.startswith("win"):
        exe = "pythonw.exe" if windowed else "python.exe"
        cand = os.path.join(ROOT, ".venv", "Scripts", exe)
        if os.path.isfile(cand):
            return cand
        base = os.path.join(os.path.dirname(sys.executable), exe)
        return base if os.path.isfile(base) else sys.executable
    cand = os.path.join(ROOT, ".venv", "bin", "python")
    return cand if os.path.exists(cand) else (shutil.which("python3") or sys.executable)


# ------------------------------------------------------------------------- Linux
def linux_paths():
    data = os.environ.get("XDG_DATA_HOME") or os.path.expanduser("~/.local/share")
    return data, os.path.join(data, "applications", f"{APP_ID}.desktop")


def linux_install():
    data, desktop = linux_paths()
    for px in (16, 24, 32, 48, 64, 128, 256):
        src = os.path.join(ASSETS, f"icon-{px}.png")
        if os.path.isfile(src):
            dst = os.path.join(data, "icons", "hicolor", f"{px}x{px}", "apps", f"{APP_ID}.png")
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
    py = find_python()
    os.makedirs(os.path.dirname(desktop), exist_ok=True)
    with open(desktop, "w") as f:
        f.write(
            "[Desktop Entry]\n"
            "Type=Application\n"
            f"Name={APP_NAME}\n"
            "GenericName=Robot Arm Control\n"
            "Comment=Program, jog and calibrate AR robot arms\n"
            f'Exec="{py}" "{os.path.join(ROOT, "AR4.py")}"\n'
            f'Path={ROOT}\n'
            f"Icon={APP_ID}\n"
            "Terminal=false\n"
            "Categories=Development;Engineering;Electronics;\n"
            "Keywords=robot;arm;AR4;Annin;CNC;\n"
            # Tk(className="AR4") gives the window WM_CLASS ("aR4", "Ar4"): matching it
            # groups the running window under this launcher and shows its icon
            "StartupWMClass=Ar4\n"
            "StartupNotify=true\n")
    os.chmod(desktop, 0o755)
    for cmd in (["update-desktop-database", os.path.dirname(desktop)],
                ["gtk-update-icon-cache", "-q", "-t", os.path.join(data, "icons", "hicolor")],
                ["kbuildsycoca6"], ["kbuildsycoca5"]):
        if shutil.which(cmd[0]):
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"Installed {desktop}")
    print(f"  runs: {py} AR4.py   (in {ROOT})")
    if ".venv" not in py:
        print("  note: no .venv found next to AR4.py; using the system Python")
    print("Look for 'AR4 Robot Control' in your application menu.")


def linux_remove():
    data, desktop = linux_paths()
    removed = []
    if os.path.exists(desktop):
        os.remove(desktop)
        removed.append(desktop)
    for px in (16, 24, 32, 48, 64, 128, 256):
        p = os.path.join(data, "icons", "hicolor", f"{px}x{px}", "apps", f"{APP_ID}.png")
        if os.path.exists(p):
            os.remove(p)
            removed.append(p)
    print("Removed:\n  " + "\n  ".join(removed) if removed else "Nothing to remove.")


# ----------------------------------------------------------------------- Windows
def windows_lnk():
    appdata = os.environ.get("APPDATA", os.path.expanduser("~"))
    return os.path.join(appdata, "Microsoft", "Windows", "Start Menu", "Programs", f"{APP_NAME}.lnk")


def windows_install():
    lnk = windows_lnk()
    py = find_python(windowed=True)
    ico = os.path.join(ASSETS, "ar4.ico")

    def q(s):
        return s.replace("'", "''")
    ps = (
        "$s=(New-Object -ComObject WScript.Shell).CreateShortcut('" + q(lnk) + "');"
        "$s.TargetPath='" + q(py) + "';"
        "$s.Arguments='\"" + q(os.path.join(ROOT, "AR4.py")) + "\"';"
        "$s.WorkingDirectory='" + q(ROOT) + "';"
        "$s.IconLocation='" + q(ico) + "';"
        "$s.Description='" + APP_NAME + "';"
        "$s.Save()")
    subprocess.run(["powershell", "-NoProfile", "-Command", ps], check=True)
    print(f"Created {lnk}\n  runs: {py} AR4.py")
    print("Find 'AR4 Robot Control' in the Start menu (right-click it to pin to the taskbar).")


def windows_remove():
    lnk = windows_lnk()
    if os.path.exists(lnk):
        os.remove(lnk)
        print(f"Removed {lnk}")
    else:
        print("Nothing to remove.")


def main():
    if not os.path.isfile(os.path.join(ROOT, "AR4.py")):
        sys.exit(f"AR4.py not found in {ROOT} - keep this script in the 'tools' folder next to it.")
    remove = "--remove" in sys.argv
    if sys.platform.startswith("win"):
        windows_remove() if remove else windows_install()
    elif sys.platform == "darwin":
        sys.exit("macOS is not supported by this script.")
    else:
        linux_remove() if remove else linux_install()


if __name__ == "__main__":
    main()
