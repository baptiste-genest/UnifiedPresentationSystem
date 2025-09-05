#!/usr/bin/env python3
import os, sys, io, json, shutil, argparse, subprocess, math
from pathlib import Path
from typing import Dict, Tuple, List
from lxml import etree
from PIL import Image
import cairosvg

# ---------------------- SVG transform utilities ----------------------
class Mat3:
    """2D affine transform stored as a 3x3 (homogeneous) matrix:
       [a c e; b d f; 0 0 1]. This is *not* 3D, just 2D with translation.
    """
    def __init__(self, a=1,b=0,c=0,d=1,e=0,f=0):
        self.a, self.b, self.c, self.d, self.e, self.f = a,b,c,d,e,f
    def __matmul__(self, o: 'Mat3') -> 'Mat3':
        return Mat3(
            a=self.a*o.a + self.c*o.b,
            b=self.b*o.a + self.d*o.b,
            c=self.a*o.c + self.c*o.d,
            d=self.b*o.c + self.d*o.d,
            e=self.a*o.e + self.c*o.f + self.e,
            f=self.b*o.e + self.d*o.f + self.f,
        )
    def apply(self, x: float, y: float) -> Tuple[float,float]:
        return (self.a*x + self.c*y + self.e, self.b*x + self.d*y + self.f)
    @staticmethod
    def identity():
        return Mat3()
    @staticmethod
    def translate(tx: float, ty: float) -> 'Mat3':
        return Mat3(1,0,0,1,tx,ty)
    @staticmethod
    def scale(sx: float, sy: float=None) -> 'Mat3':
        if sy is None:
            sy = sx
        return Mat3(sx,0,0,sy,0,0)
    @staticmethod
    def matrix(a,b,c,d,e,f) -> 'Mat3':
        return Mat3(a,b,c,d,e,f)

def parse_transform(txt: str) -> Mat3:
    if not txt:
        return Mat3.identity()
    i = 0
    m = Mat3.identity()
    while i < len(txt):
        while i < len(txt) and txt[i].isspace():
            i += 1
        if i >= len(txt):
            break
        # read function name
        j = i
        while j < len(txt) and txt[j].isalpha():
            j += 1
        name = txt[i:j]
        # read args inside (...)
        while j < len(txt) and txt[j] != '(':
            j += 1
        if j >= len(txt):
            break
        k = j+1
        depth = 1
        while k < len(txt) and depth>0:
            if txt[k] == '(':
                depth += 1
            elif txt[k] == ')':
                depth -= 1
            k += 1
        args_str = txt[j+1:k-1]
        # split args by comma/space
        args = []
        cur = ''
        for ch in args_str:
            if ch in ',\t\n\r ':
                if cur:
                    args.append(float(cur))
                    cur = ''
            else:
                cur += ch
        if cur:
            args.append(float(cur))
        comp = Mat3.identity()
        if name == 'translate':
            tx = args[0] if len(args)>0 else 0.0
            ty = args[1] if len(args)>1 else 0.0
            comp = Mat3.translate(tx,ty)
        elif name == 'scale':
            sx = args[0] if len(args)>0 else 1.0
            sy = args[1] if len(args)>1 else sx
            comp = Mat3.scale(sx,sy)
        elif name == 'matrix':
            if len(args) >= 6:
                comp = Mat3.matrix(args[0],args[1],args[2],args[3],args[4],args[5])
        # Note: dvisvgm mainly uses translate/scale/matrix; rotate/skew are rare here.
        m = m @ comp
        i = k
    return m

# ---------------------- LaTeX → SVG pipeline ----------------------
def run_cmd(cmd: List[str], cwd: Path) -> subprocess.CompletedProcess:
    print(f"[CMD] {' '.join(cmd)} (cwd={cwd})")
    return subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True)

def check_tool(name: str) -> None:
    if shutil.which(name) is None:
        print(f"[FATAL] Required tool '{name}' not found on PATH.")
        sys.exit(1)

def write_tex_file(tex_path: Path, latex_body: str) -> None:
    tex_content = r"""\documentclass{standalone}
\usepackage{amsmath}
\begin{document}
""" + latex_body + r"""
\end{document}
"""
    tex_path.write_text(tex_content, encoding="utf-8")
    print(f"[INFO] Wrote LaTeX file: {tex_path}")

def compile_to_svg(workdir: Path, basename: str) -> Path:
    tex = workdir / f"{basename}.tex"
    dvi = workdir / f"{basename}.dvi"
    svg = workdir / f"{basename}.svg"

    cp = run_cmd(["latex", "-interaction=nonstopmode", tex.name], cwd=workdir)
    print(cp.stdout)
    if cp.returncode != 0 or not dvi.exists():
        print(cp.stderr)
        raise RuntimeError("LaTeX compilation failed or .dvi not produced")

    cp = run_cmd(["dvisvgm", "--no-fonts", "--exact", "-o", svg.name, dvi.name], cwd=workdir)
    print(cp.stdout)
    if cp.returncode != 0 or not svg.exists():
        print(cp.stderr)
        raise RuntimeError("dvisvgm failed or .svg not produced")

    print(f"[INFO] SVG created: {svg}")
    return svg

# ---------------------- SVG parsing & glyph PNGs ----------------------

def parse_svg(svg_path: Path) -> Tuple[Dict[str, dict], List[dict], Dict[str, float]]:
    print(f"[INFO] Parsing SVG: {svg_path}")
    tree = etree.parse(str(svg_path))
    root = tree.getroot()
    ns = {"svg": "http://www.w3.org/2000/svg", "xlink": "http://www.w3.org/1999/xlink"}

    vb = root.get("viewBox", None)
    vb_nums = [float(x) for x in vb.split()] if vb else [0,0,0,0]

    defs = root.find("svg:defs", namespaces=ns)
    if defs is None:
        raise RuntimeError("<defs> not found — unexpected dvisvgm output.")

    # Collect path geometry *and* any transform attached to that path
    path_lookup: Dict[str, dict] = {}
    for p in defs.findall("svg:path", namespaces=ns):
        pid = p.get("id")
        d   = p.get("d")
        if not pid or not d:
            continue
        tr  = p.get("transform", "") or ""
        path_lookup[pid] = {"d": d, "transform": tr}
    print(f"[INFO] Collected {len(path_lookup)} glyph path definitions.")

    uses = root.findall(".//svg:use", namespaces=ns)
    print(f"[INFO] Found {len(uses)} placed glyphs (<use> elements).")

    placements = []
    for idx, u in enumerate(uses):
        href = u.get("{http://www.w3.org/1999/xlink}href")
        if not href or not href.startswith("#"):
            print(f"[WARN] <use> {idx} missing/invalid href, skipping")
            continue
        gid = href[1:]
        M = Mat3.identity()
        # Ancestors from root→...→parent (correct order)
        ancs = list(u.iterancestors())
        for anc in reversed(ancs):
            t = anc.get("transform")
            if t:
                M = M @ parse_transform(t)
        # <use>'s own transform
        t = u.get("transform")
        if t:
            M = M @ parse_transform(t)
        x = float(u.get("x", "0"))
        y = float(u.get("y", "0"))
        X, Y = M.apply(x, y)
        placements.append({"instance_id": idx, "glyph_id": gid, "x": X, "y": Y})

    meta = {"viewBox": vb_nums, "width": root.get("width"), "height": root.get("height")}
    return path_lookup, placements, meta


def render_glyph_png(gid: str, path_d: str, path_transform: str,
                     out_png: Path, raster_ppu: float, viewbox_units: float) -> Tuple[int,int,int,int]:
    """Render one glyph path into a large canvas, then tightly crop to content.
       Returns (crop_left_px, crop_top_px, width_px, height_px).
       *raster_ppu* is pixels per SVG unit used for rasterization.
       *viewbox_units* defines the width & height of the square viewBox, centered at the origin.
    """
    half = viewbox_units * 0.5
    t_attr = f' transform="{path_transform}"' if path_transform else ""
    mini_svg = f'''<svg xmlns="http://www.w3.org/2000/svg"
         xmlns:xlink="http://www.w3.org/1999/xlink"
         width="{int(viewbox_units)}" height="{int(viewbox_units)}" viewBox="{-half} {-half} {viewbox_units} {viewbox_units}">
  <defs><path id="{gid}" d="{path_d}" fill="black"{t_attr}/></defs>
  <use xlink:href="#{gid}" x="0" y="0"/>
</svg>'''
    png_bytes = cairosvg.svg2png(bytestring=mini_svg.encode("utf-8"), scale=raster_ppu)
    im = Image.open(io.BytesIO(png_bytes)).convert("RGBA")
    bbox = im.getbbox()
    if bbox is None:
        out_png.parent.mkdir(parents=True, exist_ok=True)
        Image.new("RGBA", (1,1), (0,0,0,0)).save(out_png)
        return (0,0,1,1)
    l, t, r, b = bbox
    cropped = im.crop(bbox)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    cropped.save(out_png)
    w, h = cropped.size
    return (l, t, w, h)


def extract_glyphs(svg_path: Path, out_dir: Path, manifest_path: Path, raster_ppu: float, viewbox_units: float) -> None:
    path_lookup, placements, meta = parse_svg(svg_path)

    glyph_png_cache: Dict[str, dict] = {}
    entries: List[dict] = []

    min_x = math.inf
    min_y = math.inf
    max_x = -math.inf
    max_y = -math.inf

    for pl in placements:
        gid = pl["glyph_id"]
        X, Y = pl["x"], pl["y"]
        if gid not in path_lookup:
            print(f"[WARN] No path found for glyph id '{gid}', skipping placement #{pl['instance_id']}")
            continue
        if gid not in glyph_png_cache:
            rec = path_lookup[gid]
            out_png = out_dir / f"{gid}.png"
            l, t, w, h = render_glyph_png(gid, rec["d"], rec["transform"],
                                          out_png, raster_ppu, viewbox_units)
            glyph_png_cache[gid] = {
                "file": out_png.name,
                "crop_left_px": l, "crop_top_px": t, "w_px": w, "h_px": h,
                "crop_left_units": l / raster_ppu,
                "crop_top_units":  t / raster_ppu,
                "w_units": w / raster_ppu,
                "h_units": h / raster_ppu,
            }
            print(f"[INFO] Rendered {gid} -> {out_png} ({w}x{h}px, crop=({l},{t})px)")

        info = glyph_png_cache[gid]
        gx0 = X - info["crop_left_units"]
        gy0 = Y - info["crop_top_units"]
        gx1 = gx0 + info["w_units"]
        gy1 = gy0 + info["h_units"]
        min_x = min(min_x, gx0)
        min_y = min(min_y, gy0)
        max_x = max(max_x, gx1)
        max_y = max(max_y, gy1)

        entries.append({
            "instance_id": pl["instance_id"],
            "glyph_id": gid,
            "file": info["file"],
            "pos_units": [X, Y],
            "anchor_units": [info["crop_left_units"], info["crop_top_units"]],
            "size_units": [info["w_units"], info["h_units"]],
            "size_px":    [info["w_px"], info["h_px"]]
        })

    bounds_units = [float(min_x), float(min_y), float(max_x-min_x), float(max_y-min_y)] if entries else [0,0,0,0]

    manifest = {
        "source_svg": svg_path.name,
        "svg_meta": meta,
        "ppu": raster_ppu,
        "glyph_viewbox_units": viewbox_units,
        "bounds_units": bounds_units,
        "out_dir": out_dir.name,
        "glyphs": entries
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"[INFO] Wrote manifest: {manifest_path}")
    print(f"[INFO] Total placed glyph entries: {len(entries)}")


def main():
    parser = argparse.ArgumentParser(description="LaTeX → DVI → SVG → glyph PNGs + JSON manifest (dvisvgm-based), high-res + correct anchors")
    parser.add_argument("--latex", help="LaTeX body to render (e.g. '$E=mc^2$'). If omitted, --texfile is required.")
    parser.add_argument("--texfile", help="Path to a .tex file (full document). Ignored if --latex is provided.")
    parser.add_argument("--basename", default="equation", help="Base name for intermediate files (default: equation)")
    parser.add_argument("--workdir", default=".", help="Working directory (intermediates + SVG).")
    parser.add_argument("--outdir", default="glyphs", help="Output directory for glyph PNGs (default: glyphs)")
    parser.add_argument("--manifest", default="equation_glyphs.json", help="Manifest JSON path (default: equation_glyphs.json)")
    parser.add_argument("--raster-ppu", type=float, default=8.0, help="Pixels-per-SVG-unit for PNGs (increase for higher PNG resolution without changing on-screen size)")
    parser.add_argument("--glyph-viewbox", type=float, default=512.0, help="Square viewBox size (units) used when rasterizing each glyph (default: 512)")
    parser.add_argument("--keep-intermediates", action="store_true", help="Keep .aux/.log/.dvi etc. (default: delete some)")
    args = parser.parse_args()

    check_tool("latex")
    check_tool("dvisvgm")

    workdir = Path(args.workdir).resolve()
    out_dir = Path(args.outdir).resolve()
    manifest_path = Path(args.manifest).resolve()

    workdir.mkdir(parents=True, exist_ok=True)
    out_dir.mkdir(parents=True, exist_ok=True)

    tex_path = workdir / f"{args.basename}.tex"

    if args.latex:
        write_tex_file(tex_path, args.latex)
    else:
        if not args.texfile:
            print("[FATAL] Provide either --latex '<code>' or --texfile path.")
            sys.exit(1)
        src = Path(args.texfile).resolve()
        if not src.exists():
            print(f"[FATAL] texfile not found: {src}")
            sys.exit(1)
        shutil.copy(src, tex_path)
        print(f"[INFO] Copied {src} → {tex_path}")

    try:
        svg_path = compile_to_svg(workdir, args.basename)
        extract_glyphs(svg_path, out_dir, manifest_path, args.raster_ppu, args.glyph_viewbox)
    finally:
        if not args.keep_intermediates:
            for ext in (".aux", ".log", ".dvi", ".eps"):
                p = workdir / f"{args.basename}{ext}"
                if p.exists():
                    p.unlink()
                    print(f"[INFO] Removed intermediate: {p}")

if __name__ == "__main__":
    main()
