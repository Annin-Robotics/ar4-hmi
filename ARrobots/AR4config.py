import os
import platform
import shutil
import subprocess
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.NOTSET) # Inherit level from Parent

@dataclass
class CameraEntry:
    kind: str       # "CSI" or "USB"
    label: str      # human-readable label
    id: str         # logical identifier, e.g. "rpicam:0" or "/dev/video0"

    def __str__(self) -> str:
        return f"{self.kind} | {self.label} -> {self.id}"

class AR4_Configuration(object):
    def __init__(self):
        super().__init__()
        self.Environment = {}                                   # Operating Enviroment related config variables
        self.Environment['Platform'] = {}                       # Platform Related Config Variables
        self.Environment['Platform']['OS'] = None
        self.Environment['Platform']['IS_WINDOWS'] = False
        self.Environment['Platform']['IS_LINUX'] = False
        self.Environment['Platform']['IS_DARWIN'] = False
        self.Environment['Platform']['IS_RPI'] = False
        self.Environment['Platform']['IS_SUPPORTED'] = False
        self.Environment['Platform']['IS_HEADLESS'] = False

        self.Environment['Cameras'] = {}                        # Camera Realted Variables
        self.Environment['Cameras']['HAS_CSI_CAMERA'] = False
        self.Environment['Cameras']['HAS_USB_CAMERA'] = False
        self.Environment['Cameras']['Enum'] = []                # List of enumerated cameras for selection

        self.Calibration = {}                                   # Calibration Related Variables
        self.RuntimeState = {}                                  # Runtime Related Variables

        self._detect_platform()
        self._enumerate_cameras()

        logger.debug(f"OS Platform is: {self.Environment['Platform']['OS']}")
        logger.debug(f"Raspberry Pi Detected!") if self.Environment['Platform']['IS_RPI'] else None
        logger.debug(f"CSI Camera Detected!") if self.Environment['Cameras']['HAS_CSI_CAMERA'] else None
        logger.debug(f"USB Camera Detected!") if self.Environment['Cameras']['HAS_USB_CAMERA'] else None
        logger.debug(f"Detected Cameras: {self.Environment['Cameras']['Enum']}")
        logger.debug(f"Supported Platform: {str(self.Environment['Platform']['IS_SUPPORTED'])}")
        logger.debug(f"Headless Platform: {str(self.Environment['Platform']['IS_HEADLESS'])}")

    def _detect_platform(self) -> None:
        self.Environment['Platform']['OS'] = platform.system()
        match self.Environment['Platform']['OS']:
            case "Linux":
                if self._is_raspberry_pi():
                    self.Environment['Platform']['IS_RPI'] = True
                    self.Environment['Platform']['IS_HEADLESS'] = self._rpi_is_headless()
                    if self._supported_debian_version():
                        self.Environment['Platform']['IS_SUPPORTED'] = True
                        if self._has_csi_camera():
                            self.Environment['Cameras']['HAS_CSI_CAMERA'] = True
                else:
                    #!! Add check for various supported Linux configs
                    self.Environment['Platform']['IS_SUPPORTED'] = True
                
                if self._has_usb_camera():
                    self.Environment['Cameras']['HAS_USB_CAMERA'] = True
                
            case "Windows":
                #!! Add check for various supported Windows configs
                self.Environment['Platform']['IS_WINDOWS'] = True
                self.Environment['Platform']['IS_SUPPORTED'] = True
            case "Darwin":
                self.Environment['Platform']['IS_DARWIN'] = True
                self.Environment['Platform']['IS_SUPPORTED'] = False
            case _:
                pass

    def _has_csi_camera(self) -> bool:
        if not shutil.which("rpicam-hello"):
            logger.debug("Debug - rpicam-hello not found, please install the rpicam-utils package")
        try:
            out = subprocess.check_output(
                ["rpicam-hello", "--list-cameras"], 
                stderr=subprocess.STDOUT,
                text=True
            )
            if "Modes:" in out:
                return True
            else:
                return False
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False
    
    def _has_usb_camera(self) -> bool:
        # Need to check platform first
        match self.Environment['Platform']['OS']:
            case "Linux":
                if not shutil.which("v4l2-ctl"):
                    logger.debug("Debug - v4l2-ctl not found, please install the v4l-utils package")
                try:
                    out = subprocess.check_output(
                        ["v4l2-ctl", "--list-devices"], 
                        stderr=subprocess.STDOUT,
                        text=True
                    )
                    if "usb" in out.lower():
                        return True
                except Exception as e:
                    return False
                
            case "Windows":
                return False
            case "Darwin":
                return False
            case _:
                return False
            
    def _supported_debian_version(self) -> bool:
        try:
            info = platform.freedesktop_os_release()
            version = int(info.get("VERSION_ID", "0"))
            if version >= 12:
                return True
            else:
                logger.debug(f"Debug - Unsupported Debian version: {version}")
                return False
        except Exception as e:
            return False
        
    def _is_raspberry_pi(self) -> bool:
        try:
            model = Path("/sys/firmware/devicetree/base/model").read_text()
            if "Raspberry" in model:
                return True
            else:
                return False
        except Exception as e:
            return False

    def _enumerate_cameras(self):
        if self.Environment['Cameras']['HAS_CSI_CAMERA']:
            self.Environment['Cameras']['Enum'] += self._list_csi_rpicam()
        if self.Environment['Cameras']['HAS_USB_CAMERA']:
            self.Environment['Cameras']['Enum'] += self._list_usb_v4l2()

        # de-duplicate entries by id
        seen = set()
        all_list: List[CameraEntry] = []
        for e in self.Environment['Cameras']['Enum']:
            if e.id not in seen:
                seen.add(e.id)
                all_list.append(e)
        self.Environment['Cameras']['Enum'] = all_list
        
    def _list_csi_rpicam(self) -> List[CameraEntry]:
        """
        Enumerate CSI cameras using rpicam-hello --list-cameras.
        Returns a list of CameraEntry(kind="CSI", ...).
        Assumes rpicam-hello is present (per your requirement).
        """
        if not shutil.which("rpicam-hello"):
            logger.debug("Debug - rpicam-hello not found, please install the rpicam-utils package")
            return []

        try:
            out = subprocess.check_output(
                ["rpicam-hello", "--list-cameras"],
                stderr=subprocess.STDOUT, text=True, timeout=3
            )
        except Exception:
            return []

        entries: List[CameraEntry] = []
        for line in out.splitlines():
            line = line.strip()
            # Expected lines like: "0 : imx219 [3280x2464] (/base/...)"
            if not line or ":" not in line or not line[0].isdigit():
                continue
            idx_str, rest = line.split(":", 1)
            try:
                idx = int(idx_str.strip())
            except ValueError:
                continue
            # model is the token before the first '[' if present
            model = rest.split("[", 1)[0].strip()
            model = model or "CSI Camera"
            entries.append(CameraEntry(
                kind="CSI",
                label=f"CSI: {model} (#{idx})",
                id=f"rpicam:{idx}"
            ))
        return entries

    def _list_usb_v4l2(self) -> List[CameraEntry]:
        """
        Enumerate USB webcams via v4l2-ctl --list-devices (if available),
        else fallback to scanning /dev/video* (less precise).
        """
        entries: List[CameraEntry] = []

        if not shutil.which("v4l2-ctl"):
            logger.debug("Debug - v4l2-ctl not found, please install the v4l-utils package")
            return []
        else:
            try:
                out = subprocess.check_output(
                    ["v4l2-ctl", "--list-devices"],
                    stderr=subprocess.STDOUT, text=True, timeout=3
                )
                # Parse blocks like:
                #   HD Webcam (usb-0000:00:14.0-5):
                #       /dev/video0
                block = []
                for line in out.splitlines():
                    if line.strip() and not line.startswith("\t") and ":" in line:
                        # new device block
                        if block:
                            entries.extend(self._parse_v4l2_block(block))
                            block = []
                        block.append(line.rstrip())
                    elif line.startswith("\t"):
                        block.append(line.rstrip())
                if block:
                    entries.extend(self._parse_v4l2_block(block))

                return entries or []
            
            except Exception:
                return []
    
    def _parse_v4l2_block(self, block_lines: list[str]) -> list[CameraEntry]:
        """
        Parse one `v4l2-ctl --list-devices` block and return ONLY the first
        /dev/video* path for that device (skip codec/ISP platform nodes).
        """
        header = block_lines[0].strip()

        # Keep only USB-backed devices; codec/ISP nodes are typically "platform:*"
        is_usb = "(usb-" in header.lower() or "usb-" in header.lower() or " usb " in header.lower()
        if not is_usb:
            return []

        model = header.split("(", 1)[0].strip() or "USB Camera"

        # Find the first /dev/video* line after the header
        first_dev = None
        for ln in block_lines[1:]:
            ln = ln.strip()
            if ln.startswith("/dev/video"):
                first_dev = ln
                break

        if not first_dev:
            return []

        return [
            CameraEntry(
                kind="USB",
                label=f"{model[:18]}",
                id=first_dev,
            )
        ]
    
    def _rpi_is_headless(self) -> bool:
        """
        Check all DRM HDMI interfaces under /sys/class/drm.
        Returns True if *no* HDMI connectors are reported as 'connected'.
        Returns False if *any* HDMI connector is 'connected'.
        """
        try:
            drm_root = "/sys/class/drm"
            if not os.path.isdir(drm_root):
                return True  # No DRM subsystem → treat as headless

            for entry in os.listdir(drm_root):
                if "HDMI" not in entry:
                    continue

                status_path = os.path.join(drm_root, entry, "status")
                if not os.path.isfile(status_path):
                    continue

                try:
                    with open(status_path) as f:
                        status = f.read().strip()
                    if status == "connected":
                        return False  # At least one HDMI connected
                except OSError:
                    # Ignore unreadable entries
                    continue
            logger.debug("No connected HDMI interfaces found - Device is Headless.")
            return True  # No connected HDMI found / Running headless
        except Exception as e:
            logger.error(f"Error checking HDMI status: {e}")
            return False  # On error, assume not headless






