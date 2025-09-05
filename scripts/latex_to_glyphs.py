#!/usr/bin/env python3
import os, sys, io, json, shutil, argparse, subprocess
from pathlib import Path
from typing import Dict, Tuple, List
from lxml import etree
from PIL import Image
import cairosvg

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

    # latex -> dvi
    cp = run_cmd(["latex", "-interaction=nonstopmode", tex.name], cwd=workdir)
    print(cp.stdout)
    if cp.returncode != 0 or not dvi.exists():
        print(cp.stderr)
        raise RuntimeError("LaTeX compilation failed or .dvi not produced")

    # dvisvgm -> svg
    cp = run_cmd(["dvisvgm", "--no-fonts", "--exact", "-o", svg.name, dvi.name], cwd=workdir)
    print(cp.stdout)
    if cp.returncode != 0 or not svg.exists():
        print(cp.stderr)
        raise RuntimeError("dvisvgm failed or .svg not produced")

    print(f"[INFO] SVG created: {svg}")
    return svg

def parse_svg(svg_path: Path) -> Tuple[Dict[str, str], List[dict]]:
    print(f"[INFO] Parsing SVG: {svg_path}")
    tree = etree.parse(str(svg_path))
    root = tree.getroot()
    ns = {"svg": "http://www.w3.org/2000/svg", "xlink": "http://www.w3.org/1999/xlink"}

    defs = root.find("svg:defs", namespaces=ns)
    if defs is None:
        raise RuntimeError("<defs> not found — unexpected dvisvgm output.")

    # glyph id -> 'd'
    path_lookup = {p.get("id"): p.get("d") for p in defs.findall("svg:path", namespaces=ns) if p.get("id") and p.get("d")}
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
        x = float(u.get("x", "0"))
        y = float(u.get("y", "0"))
        placements.append({"instance_id": idx, "glyph_id": gid, "x": x, "y": y})
    return path_lookup, placements

def render_glyph_png(gid: str, path_d: str, out_png: Path) -> Tuple[int,int,int,int]:
    """Render one glyph 'd' into a large canvas, then tightly crop to content.
       Returns (crop_left, crop_top, width, height) in pixels.
    """
    # Large safe viewBox; cairo needs explicit size
    mini_svg = f'''<svg xmlns="http://www.w3.org/2000/svg"
         xmlns:xlink="http://www.w3.org/1999/xlink"
         width="512" height="512" viewBox="-256 -256 512 512">
  <defs><path id="{gid}" d="{path_d}" fill="black"/></defs>
  <use xlink:href="#{gid}" x="0" y="0"/>
</svg>'''
    png_bytes = cairosvg.svg2png(bytestring=mini_svg.encode("utf-8"),scale=2.0)
    im = Image.open(io.BytesIO(png_bytes)).convert("RGBA")
    bbox = im.getbbox()  # (l, t, r, b) of non-transparent
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

def extract_glyphs(svg_path: Path, out_dir: Path, manifest_path: Path) -> None:
    path_lookup, placements = parse_svg(svg_path)

    glyph_png_cache: Dict[str, dict] = {}
    entries: List[dict] = []

    for pl in placements:
        gid = pl["glyph_id"]
        x, y = pl["x"], pl["y"]

        if gid not in path_lookup:
            print(f"[WARN] No path found for glyph id '{gid}', skipping placement #{pl['instance_id']}")
            continue

        if gid not in glyph_png_cache:
            out_png = out_dir / f"{gid}.png"
            l, t, w, h = render_glyph_png(gid, path_lookup[gid], out_png)
            glyph_png_cache[gid] = {
                "file": out_png.name, "crop_left": l, "crop_top": t, "w": w, "h": h
            }
            print(f"[INFO] Rendered {gid} -> {out_png} ({w}x{h}, crop_off=({l},{t}))")

        info = glyph_png_cache[gid]
        entries.append({
            "instance_id": pl["instance_id"],
            "glyph_id": gid,
            "file": info["file"],
            "position": [x, y],
            "anchor_crop_offset": [info["crop_left"], info["crop_top"]],
            "size_px": [info["w"], info["h"]]
        })

    manifest = {
        "source_svg": svg_path.name,
        "out_dir": out_dir.name,
        "glyphs": entries
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"[INFO] Wrote manifest: {manifest_path}")
    print(f"[INFO] Total placed glyph entries: {len(entries)}")
    if out_dir.exists():
        print("[INFO] Output glyph PNGs:")
        for name in sorted(p.name for p in out_dir.glob("*.png")):
            print(" -", name)

def main():
    parser = argparse.ArgumentParser(description="LaTeX -> DVI -> SVG -> glyph PNGs + JSON manifest (dvisvgm-based).")
    parser.add_argument("--latex", help="LaTeX body to render (e.g. '$E=mc^2$'). If omitted, --texfile is required.")
    parser.add_argument("--texfile", help="Path to a .tex file (full document). Ignored if --latex is provided.")
    parser.add_argument("--basename", default="equation", help="Base name for intermediate files (default: equation)")
    parser.add_argument("--workdir", default=".", help="Working directory (intermediates + SVG).")
    parser.add_argument("--outdir", default="glyphs", help="Output directory for glyph PNGs (default: glyphs)")
    parser.add_argument("--manifest", default="equation_glyphs.json", help="Manifest JSON path (default: equation_glyphs.json)")
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
        print(f"[INFO] Copied {src} -> {tex_path}")

    try:
        svg_path = compile_to_svg(workdir, args.basename)
        extract_glyphs(svg_path, out_dir, manifest_path)
    finally:
        if not args.keep_intermediates:
            for ext in (".aux", ".log", ".dvi", ".eps"):
                p = workdir / f"{args.basename}{ext}"
                if p.exists():
                    p.unlink()
                    print(f"[INFO] Removed intermediate: {p}")

if __name__ == "__main__":
    main()
