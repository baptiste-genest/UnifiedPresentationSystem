import os, json, io
from lxml import etree
from PIL import Image
import cairosvg

def extract_glyphs_from_dvisvgm(svg_path, out_dir, manifest_path):
    os.makedirs(out_dir, exist_ok=True)
    tree = etree.parse(svg_path)
    ns = {"svg": "http://www.w3.org/2000/svg", "xlink": "http://www.w3.org/1999/xlink"}

    defs = tree.getroot().find("svg:defs", namespaces=ns)
    if defs is None:
        raise RuntimeError("<defs> not found — unexpected dvisvgm output.")

    # Map glyph id -> path data
    path_lookup = {p.get("id"): p.get("d") for p in defs.findall("svg:path", namespaces=ns) if p.get("id") and p.get("d")}
    uses = tree.findall(".//svg:use", namespaces=ns)

    def render_glyph_id(gid, png_path):
        d = path_lookup.get(gid)
        if not d:
            raise ValueError(f"Path data not found for {gid}")
        # Big safe viewBox; then crop with PIL
        mini_svg = f'''<svg xmlns="http://www.w3.org/2000/svg"
             xmlns:xlink="http://www.w3.org/1999/xlink"
             width="512" height="512" viewBox="-256 -256 512 512">
  <defs><path id="{gid}" d="{d}" fill="black"/></defs>
  <use xlink:href="#{gid}" x="0" y="0"/>
</svg>'''
        png_bytes = cairosvg.svg2png(bytestring=mini_svg.encode("utf-8"))
        im = Image.open(io.BytesIO(png_bytes)).convert("RGBA")
        bbox = im.getbbox() or (0,0,1,1)
        cropped = im.crop(bbox)
        cropped.save(png_path)
        left, top, right, bottom = bbox
        return (left, top, cropped.size[0], cropped.size[1])

    glyph_png_cache = {}
    entries = []
    for idx, u in enumerate(uses):
        href = u.get("{http://www.w3.org/1999/xlink}href")
        if not href or not href.startswith("#"):
            continue
        gid = href[1:]
        x = float(u.get("x", "0"))
        y = float(u.get("y", "0"))
        if gid not in glyph_png_cache:
            png_path = os.path.join(out_dir, f"{gid}.png")
            crop_left, crop_top, w, h = render_glyph_id(gid, png_path)
            glyph_png_cache[gid] = {
                "file": os.path.basename(png_path),
                "crop_left": crop_left, "crop_top": crop_top,
                "w": w, "h": h
            }
        info = glyph_png_cache[gid]
        entries.append({
            "instance_id": idx,
            "glyph_id": gid,
            "file": info["file"],
            "position": [x, y],
            "anchor_crop_offset": [info["crop_left"], info["crop_top"]],
            "size_px": [info["w"], info["h"]],
        })

    manifest = {"source_svg": os.path.basename(svg_path),
                "out_dir": os.path.basename(out_dir),
                "glyphs": entries}
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

if __name__ == "__main__":
    extract_glyphs_from_dvisvgm(
        svg_path="equation.svg",
        out_dir="glyphs2",
        manifest_path="equation_glyphs2.json"
    )

