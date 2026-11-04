"""Generate a tileable pegboard texture for the shelving back wall.

Real grocery / hardware-store pegboard typically has:
    - 1 inch (~25 mm) hole pitch on a regular square grid
    - ~6 mm hole diameter
    - Off-white painted steel substrate

Output: 512x512 PNG that, paired with `texuniform="true" texrepeat="2 2"` in
the material, gives 0.5 m tiles -> ~25 mm hole pitch matching the real-world
spec. Holes drawn as small dark dots with a faint shadow ring so they read as
recessed at any rendering scale.

Run: python3 generate_shelf_pegboard.py
"""

from pathlib import Path
from PIL import Image, ImageDraw, ImageFilter

HERE = Path(__file__).resolve().parent
DST = HERE / "shelf_pegboard.png"

SIZE = 512
BG = (228, 222, 210)            # cream-painted steel
BG_DARK = (210, 204, 190)       # subtle vertical seam shading
HOLE_COLOR = (40, 36, 32)       # nearly black (the dark space behind a hole)
HOLE_SHADOW = (155, 148, 138)   # ring around the hole for recess shading
GRID = 20                       # holes per dimension
HOLE_RADIUS = 4                 # in pixels at SIZE=512
SHADOW_RADIUS = 6


def main() -> None:
    img = Image.new("RGB", (SIZE, SIZE), BG)
    draw = ImageDraw.Draw(img)

    # Faint vertical seams every 1/4 of the tile, suggesting the steel-panel
    # joints where adjacent pegboard sheets butt together.
    for i in range(1, 4):
        x = i * SIZE // 4
        draw.line([(x, 0), (x, SIZE - 1)], fill=BG_DARK, width=1)

    pitch = SIZE / GRID
    for ix in range(GRID):
        for iy in range(GRID):
            cx = (ix + 0.5) * pitch
            cy = (iy + 0.5) * pitch
            # outer "shadow" ring (recess illusion)
            draw.ellipse(
                (cx - SHADOW_RADIUS, cy - SHADOW_RADIUS,
                 cx + SHADOW_RADIUS, cy + SHADOW_RADIUS),
                fill=HOLE_SHADOW,
            )
            # the hole itself
            draw.ellipse(
                (cx - HOLE_RADIUS, cy - HOLE_RADIUS,
                 cx + HOLE_RADIUS, cy + HOLE_RADIUS),
                fill=HOLE_COLOR,
            )

    # Light blur smooths circle edges to "drilled" rather than "drawn".
    img = img.filter(ImageFilter.GaussianBlur(radius=0.4))
    img.save(DST, format="PNG", optimize=True)
    print(f"wrote {DST}  ({img.size[0]}x{img.size[1]})")


if __name__ == "__main__":
    main()
