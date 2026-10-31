#!/usr/bin/env python3
"""
build_assets.py - regenerate the AR4 UI brand assets.

    python tools/build_assets.py --svg logo.svg --video ar3.mov --out assets

Produces (in --out):
    ar4-logo.svg              clean glyph-only logo, transparent, tight viewBox
    logo-light.png/-2x.png    light glyph for dark theme header
    logo-dark.png/-2x.png     dark glyph for light theme header
    icon-<size>.png, ar4.ico  window / taskbar icon set
    splash.gif                robot animation, recoloured to the splash palette
Needs Pillow + numpy; the splash also needs ffmpeg on PATH (ProRes 4444 source).
"""
import argparse
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw

# Palette shared with ar4_ui.py (graphite theme)
SPLASH_BG = (19, 22, 27)        # #13161b  graphite chrome
SPLASH_FG = (242, 244, 246)     # #f2f4f6
GLYPH_LIGHT = (231, 234, 238)   # #e7eaee  on dark surfaces
GLYPH_DARK = (26, 30, 36)       # #1a1e24  on light surfaces
ORANGE = (255, 122, 26)         # #ff7a1a  motion / brand accent
GRAPHITE = (26, 30, 36)


# --------------------------------------------------------------------------- SVG
def _parse_path(d):
    """Return list of subpaths, each a list of (x, y); supports M/L/C/Z (abs)."""
    toks = re.findall(r"[MLCZmlcz]|-?\d*\.?\d+(?:e-?\d+)?", d)
    subs, cur, i, cmd, pos = [], [], 0, None, (0.0, 0.0)
    while i < len(toks):
        t = toks[i]
        if t.isalpha():
            cmd = t
            i += 1
            if cmd in "Zz":
                if cur:
                    subs.append(cur)
                cur = []
            continue
        if cmd == "M":
            pos = (float(toks[i]), float(toks[i + 1])); i += 2
            if cur:
                subs.append(cur)
            cur = [pos]
            cmd = "L"
        elif cmd == "L":
            pos = (float(toks[i]), float(toks[i + 1])); i += 2
            cur.append(pos)
        elif cmd == "C":
            p0 = pos
            p1 = (float(toks[i]), float(toks[i + 1]))
            p2 = (float(toks[i + 2]), float(toks[i + 3]))
            p3 = (float(toks[i + 4]), float(toks[i + 5]))
            i += 6
            for s in np.linspace(0, 1, 24)[1:]:
                u = 1 - s
                cur.append((u**3 * p0[0] + 3 * u * u * s * p1[0] + 3 * u * s * s * p2[0] + s**3 * p3[0],
                            u**3 * p0[1] + 3 * u * u * s * p1[1] + 3 * u * s * s * p2[1] + s**3 * p3[1]))
            pos = p3
        else:
            raise ValueError(f"unsupported path command {cmd}")
    if cur:
        subs.append(cur)
    return subs


def _matrix(transform):
    m = np.eye(3)
    for name, args in re.findall(r"(\w+)\(([^)]*)\)", transform or ""):
        v = [float(a) for a in re.split(r"[ ,]+", args.strip()) if a]
        t = np.eye(3)
        if name == "translate":
            t[0, 2], t[1, 2] = v[0], (v[1] if len(v) > 1 else 0)
        elif name == "scale":
            t[0, 0], t[1, 1] = v[0], (v[1] if len(v) > 1 else v[0])
        elif name == "rotate":
            a = np.radians(v[0]); t[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
        elif name in ("skewX", "skewY"):
            a = np.tan(np.radians(v[0]))
            t[0 if name == "skewX" else 1, 1 if name == "skewX" else 0] = a
        elif name == "matrix":
            t = np.array([[v[0], v[2], v[4]], [v[1], v[3], v[5]], [0, 0, 1]])
        m = m @ t
    return m


def load_glyph(svg_path):
    """All filled paths (excluding full-canvas background rects) in user space."""
    ns = "{http://www.w3.org/2000/svg}"
    polys = []

    def walk(el, m):
        m = m @ _matrix(el.get("transform"))
        if el.tag == ns + "path" and el.get("fill", "none") != "none":
            for sub in _parse_path(el.get("d")):
                pts = np.array([(x, y, 1.0) for x, y in sub]) @ m.T
                polys.append(pts[:, :2])
        for ch in el:
            walk(ch, m)

    walk(ET.parse(svg_path).getroot(), np.eye(3))
    allpts = np.vstack(polys)
    x0, y0 = allpts.min(0)
    x1, y1 = allpts.max(0)
    return [p - (x0, y0) for p in polys], (x1 - x0, y1 - y0)


def render_glyph(polys, size, height, color, pad=0.0, ss=8):
    """Glyph scaled to `height` px, transparent background, supersampled."""
    w, h = size
    scale = height * (1 - 2 * pad) / h
    W = int(round(w * scale + 2 * pad * height))
    H = height
    big = Image.new("L", (W * ss, H * ss), 0)
    d = ImageDraw.Draw(big)
    off = pad * height
    for p in polys:
        d.polygon([((x * scale + off) * ss, (y * scale + off) * ss) for x, y in p], fill=255)
    mask = big.resize((W, H), Image.LANCZOS)
    out = Image.new("RGBA", (W, H), color + (0,))
    out.putalpha(mask)
    return out


def write_clean_svg(polys, size, path):
    w, h = size
    ds = []
    for p in polys:
        ds.append("M" + " L".join(f"{x:.2f} {y:.2f}" for x, y in p) + " Z")
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.2f} {h:.2f}" '
           f'width="{w:.0f}" height="{h:.0f}">\n'
           f'  <title>AR4</title>\n'
           f'  <path fill="currentColor" d="{" ".join(ds)}"/>\n</svg>\n')
    with open(path, "w") as f:
        f.write(svg)


def make_icon(polys, size, px):
    """Orange rounded tile, graphite glyph: reads at 16px in any taskbar."""
    ss = 4
    S = px * ss
    tile = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    d = ImageDraw.Draw(tile)
    d.rounded_rectangle([0, 0, S - 1, S - 1], radius=int(S * 0.22), fill=ORANGE + (255,))
    w, h = size
    gw = S * (0.74 if px > 32 else 0.86)
    scale = gw / w
    gh = h * scale
    ox, oy = (S - gw) / 2, (S - gh) / 2 + S * 0.01
    m = Image.new("L", (S, S), 0)
    md = ImageDraw.Draw(m)
    for p in polys:
        md.polygon([(x * scale + ox, y * scale + oy) for x, y in p], fill=255)
    glyph = Image.new("RGBA", (S, S), GRAPHITE + (255,))
    tile.paste(glyph, (0, 0), m)
    return tile.resize((px, px), Image.LANCZOS)


# ------------------------------------------------------------------------- video
def build_splash(video, out_gif, width=600, motion_end=None):
    tmp = tempfile.mkdtemp()
    try:
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", video,
                        "-vf", "scale=960:540:flags=lanczos", "-pix_fmt", "gray",
                        os.path.join(tmp, "f_%03d.png")], check=True)
        files = sorted(os.listdir(tmp))
        frames = [np.asarray(Image.open(os.path.join(tmp, f)), dtype=np.float32) for f in files]

        # union bbox of everything that isn't background
        x0 = y0 = 10**9; x1 = y1 = 0
        for a in frames:
            ys, xs = np.where(a > 24)
            x0, y0 = min(x0, xs.min()), min(y0, ys.min())
            x1, y1 = max(x1, xs.max()), max(y1, ys.max())
        m = 6
        box = (max(0, x0 - m), max(0, y0 - m), min(960, x1 + m), min(540, y1 + m))

        # trim the static hold at the end: keep frames until motion stops
        if motion_end is None:
            diffs = [np.abs(frames[i] - frames[i - 1]).mean() for i in range(1, len(frames))]
            moving = [i for i, d in enumerate(diffs, 1) if d > 0.05]
            motion_end = min(len(frames), (moving[-1] + 2) if moving else len(frames))

        # palette: 64 steps from splash bg to splash fg (exact theme colours)
        steps = 64
        pal = []
        for k in range(steps):
            t = k / (steps - 1)
            pal += [round(SPLASH_BG[c] + (SPLASH_FG[c] - SPLASH_BG[c]) * t) for c in range(3)]
        pal += [0] * (768 - len(pal))

        out = []
        for a in frames[:motion_end]:
            crop = Image.fromarray(a.astype(np.uint8)).crop(box)
            hgt = round(width * crop.height / crop.width)
            crop = crop.resize((width, hgt), Image.LANCZOS)
            idx = np.clip(np.round(np.asarray(crop, np.float32) / 255 * (steps - 1)), 0, steps - 1)
            im = Image.fromarray(idx.astype(np.uint8), "P")
            im.putpalette(pal)
            out.append(im)
        out[0].save(out_gif, save_all=True, append_images=out[1:], duration=33, loop=0,
                    optimize=False, disposal=1)
        return len(out), out[0].size
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--svg", required=True)
    ap.add_argument("--video")
    ap.add_argument("--out", default="assets")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    polys, size = load_glyph(a.svg)
    print(f"glyph {size[0]:.1f} x {size[1]:.1f} units, {len(polys)} shapes")
    write_clean_svg(polys, size, os.path.join(a.out, "ar4-logo.svg"))

    for name, col in (("logo-light", GLYPH_LIGHT), ("logo-dark", GLYPH_DARK)):
        render_glyph(polys, size, 26, col).save(os.path.join(a.out, f"{name}.png"))
        render_glyph(polys, size, 52, col).save(os.path.join(a.out, f"{name}-2x.png"))

    sizes = [16, 24, 32, 48, 64, 128, 256]
    icons = {px: make_icon(polys, size, px) for px in sizes}
    for px, im in icons.items():
        im.save(os.path.join(a.out, f"icon-{px}.png"))
    icons[256].save(os.path.join(a.out, "ar4.ico"), sizes=[(s, s) for s in sizes])

    if a.video:
        n, dims = build_splash(a.video, os.path.join(a.out, "splash.gif"))
        print(f"splash.gif: {n} frames, {dims[0]}x{dims[1]}")


if __name__ == "__main__":
    main()
