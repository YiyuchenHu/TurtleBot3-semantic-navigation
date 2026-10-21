#!/usr/bin/env python3
"""Generate colored-shirt variants of the person_standing Gazebo model.

Runs INSIDE the sim container (needs /opt/gazebo_models and PIL):

    docker compose -f docker/compose.yaml -f docker/compose.local.yaml exec sim \
        python3 /home/ubuntu/ws/scripts/make_person_variants.py

For each color it copies /opt/gazebo_models/person_standing to
src/tb3_frontier_exploration/models/person_<color>_shirt/, renames the model
(model.config name, model.sdf model name and mesh URIs), and multiplies the
t-shirt diffuse texture (tshirt02_texture.png) by the color. All other
textures and the COLLADA mesh are byte-identical copies; the .dae references
textures via ../materials/textures/ so no mesh edit is needed.

The generated model dirs are ~28 MB each and are .gitignore'd; this script is
the source of truth. The launch file appends the models/ dir to
GAZEBO_MODEL_PATH so gzserver can resolve model://person_<color>_shirt.
"""
import argparse
import shutil
import sys
from pathlib import Path

from PIL import Image

SRC_MODEL = Path("/opt/gazebo_models/person_standing")
DEFAULT_DST = Path("/home/ubuntu/ws/src/tb3_frontier_exploration/models")
SHIRT_TEXTURE = "materials/textures/tshirt02_texture.png"

# Multiply factors for the light-gray shirt base (~230): keeps shading, gives
# a clearly nameable color.
COLORS = {
    "green": (40, 190, 40),
    "red": (210, 40, 40),
    "blue": (40, 80, 220),
}


def tint(png_path: Path, rgb):
    im = Image.open(png_path).convert("RGBA")
    r, g, b, a = im.split()
    r = r.point(lambda v: v * rgb[0] // 255)
    g = g.point(lambda v: v * rgb[1] // 255)
    b = b.point(lambda v: v * rgb[2] // 255)
    Image.merge("RGBA", (r, g, b, a)).save(png_path)


def make_variant(color: str, dst_root: Path):
    name = f"person_{color}_shirt"
    dst = dst_root / name
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(SRC_MODEL, dst)

    cfg = dst / "model.config"
    cfg.write_text(
        cfg.read_text().replace(
            "<name>Standing person</name>",
            f"<name>Standing person ({color} shirt)</name>",
        )
    )

    sdf = dst / "model.sdf"
    sdf.write_text(
        sdf.read_text()
        .replace('name="person_standing"', f'name="{name}"')
        .replace("model://person_standing", f"model://{name}")
    )

    tint(dst / SHIRT_TEXTURE, COLORS[color])
    print(f"OK {dst}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dst", type=Path, default=DEFAULT_DST)
    ap.add_argument("--colors", nargs="+", default=list(COLORS), choices=list(COLORS))
    args = ap.parse_args()

    if not SRC_MODEL.is_dir():
        sys.exit(f"source model not found: {SRC_MODEL} (run inside the sim container)")
    args.dst.mkdir(parents=True, exist_ok=True)
    for color in args.colors:
        make_variant(color, args.dst)


if __name__ == "__main__":
    main()
