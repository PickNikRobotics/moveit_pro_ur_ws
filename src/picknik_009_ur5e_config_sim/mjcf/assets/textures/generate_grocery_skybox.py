"""Generate a grocery-store-feel cube-unfold skybox texture.

The skybox replaces MuJoCo's default `builtin="gradient"` background with
something that suggests "you are in the FAR background of a real grocery
store interior" — not the immediate workspace, but the distant fixtures and
displays you'd glimpse looking past the working shelf.

The previous design read as tall warm-tan walls because the shelving band
filled half the cell with a single near-uniform color and the ceiling was a
modest fluorescent strip on cream. That looked like an aisle endcap, not
like distance. This rewrite breaks each side cell into five conceptual
zones, top to bottom:

    - **Ceiling** : bright cool-white industrial ceiling crossed by a few
      darker truss / I-beam lines, with several small fluorescent fixtures
      spaced along each row and a couple of suspended-fixture droppers.
      Dominant — a real grocery-store ceiling fills more of the visual field
      than people expect.
    - **Banner zone** : a narrow strip of hanging banner-style signage in
      mixed grocery-aisle colors (red / dark / green / gold), each banner
      drawn with a thin suspension cable up into the ceiling so it reads as
      hanging rather than floating.
    - **Product wall** : the busy mid-distance mix of distant aisles —
      vertical bands of mixed produce-cart and packaged-goods colors over a
      warm tan ground, broken up by a single far-shelf horizontal line and
      a couple of dark end-cap blobs.
    - **Lower shelving** : a band of dark-wood display bases with thin
      vertical separators.
    - **Floor** : a cool gray strip with a single horizon-suggestion line.

The top cell is the same ceiling drawn in plan view — bright cool-white with
a 4x4 truss grid, one fluorescent rectangle per cell, and a few scattered
suspended-fixture dots. Bottom cell is solid floor gray; the actual floor
plane covers it almost everywhere.

A final Gaussian blur softens the geometry so the cells read as out-of-focus
distant fixtures rather than crisp UI shapes. Per-side seeds vary the
banner / product-band layout so the four sides aren't identical.

Output: 1024 x 768 PNG with a 4 col x 3 row cube unfold (".U..LFRB.D..").
MuJoCo loads it with:

    <texture type="skybox" file="textures/grocery_skybox.png"
             gridsize="3 4" gridlayout=".U..LFRB.D.." />

Run: python3 generate_grocery_skybox.py
"""

from pathlib import Path
import random

from PIL import Image, ImageDraw, ImageFilter

HERE = Path(__file__).resolve().parent
DST = HERE / "grocery_skybox.png"

CELL = 256
COLS, ROWS = 4, 3
W, H = CELL * COLS, CELL * ROWS

# Ceiling palette — bright cool-white industrial, a hair off pure white so
# the ceiling has a faint blue cast typical of fluorescent / LED fixtures.
CEIL_WHITE = (235, 240, 248)
TRUSS_GRAY = (180, 185, 195)
FLUOR = (255, 255, 250)
FLUOR_HALO = (245, 245, 235)
DUCT_DARK = (110, 115, 120)

# Banner / signage palette
BANNER_RED = (170, 60, 50)
BANNER_DARK = (75, 65, 60)
BANNER_GREEN = (90, 130, 75)
BANNER_GOLD = (200, 165, 70)

# Product-wall palette: warm produce + packaged-goods colors over tan
PROD_RED = (200, 70, 60)
PROD_YEL = (235, 200, 90)
PROD_GRN = (130, 175, 100)
PROD_ORG = (220, 145, 75)
PROD_BRN = (130, 90, 60)
PROD_TAN = (200, 175, 130)

SHELF_DARK = (100, 80, 65)
FLOOR_GRAY = (175, 175, 170)
EMPTY = (60, 56, 50)  # unused cells of the cube unfold


def ceiling_cell() -> Image.Image:
    """Top face: bright cool-white industrial ceiling seen in plan view."""
    img = Image.new("RGB", (CELL, CELL), CEIL_WHITE)
    draw = ImageDraw.Draw(img)
    rng = random.Random(7)

    # 4x4 grid of darker truss / cross-beams (3 verticals + 3 horizontals).
    step = CELL // 4
    beam_w = 6
    for i in range(1, 4):
        x = i * step
        draw.rectangle(
            [(x - beam_w // 2, 0), (x + beam_w // 2, CELL - 1)], fill=TRUSS_GRAY
        )
        y = i * step
        draw.rectangle(
            [(0, y - beam_w // 2), (CELL - 1, y + beam_w // 2)], fill=TRUSS_GRAY
        )

    # One fluorescent fixture per grid cell, oriented horizontal/vertical at
    # random. Halo first, then warmer core.
    for cy_idx in range(4):
        for cx_idx in range(4):
            cx = cx_idx * step + step // 2
            cy = cy_idx * step + step // 2
            horizontal = rng.random() < 0.5
            if horizontal:
                length, half_w = step // 2, 4
                halo_box = (cx - length, cy - half_w - 2, cx + length, cy + half_w + 2)
                core_box = (cx - length, cy - half_w, cx + length, cy + half_w)
            else:
                length, half_w = step // 2, 4
                halo_box = (cx - half_w - 2, cy - length, cx + half_w + 2, cy + length)
                core_box = (cx - half_w, cy - length, cx + half_w, cy + length)
            draw.rectangle(halo_box, fill=FLUOR_HALO)
            draw.rectangle(core_box, fill=FLUOR)

    # Half a dozen scattered dark dots — sprinklers / suspended-light brackets.
    for _ in range(6):
        bx = rng.randint(8, CELL - 8)
        by = rng.randint(8, CELL - 8)
        bw = rng.randint(2, 4)
        bh = rng.randint(2, 4)
        draw.rectangle((bx, by, bx + bw, by + bh), fill=DUCT_DARK)

    return img.filter(ImageFilter.GaussianBlur(radius=1.2))


def side_cell(seed: int) -> Image.Image:
    """L/F/R/B face: ceiling -> banner zone -> product wall -> lower shelving
    -> floor. Each zone gets a realistic share of the vertical extent so the
    overall image reads as a distant grocery-store interior rather than as a
    near wall."""
    rng = random.Random(seed)
    img = Image.new("RGB", (CELL, CELL), CEIL_WHITE)
    draw = ImageDraw.Draw(img)

    ceiling_h = int(CELL * 0.30)         # 0..30%
    banner_h = int(CELL * 0.42) - ceiling_h   # 30..42%
    product_h = int(CELL * 0.78) - (ceiling_h + banner_h)  # 42..78%
    lower_h = int(CELL * 0.90) - (ceiling_h + banner_h + product_h)  # 78..90%
    # floor zone fills 90..100%

    y_ceiling_top = 0
    y_banner_top = ceiling_h
    y_product_top = ceiling_h + banner_h
    y_lower_top = y_product_top + product_h
    y_floor_top = y_lower_top + lower_h

    # --- Ceiling band ----------------------------------------------------
    draw.rectangle(
        [(0, y_ceiling_top), (CELL, y_banner_top)], fill=CEIL_WHITE
    )
    # 2-3 horizontal truss / I-beam lines.
    n_beams = rng.randint(2, 3)
    for i in range(n_beams):
        by = y_ceiling_top + int((i + 1) * ceiling_h / (n_beams + 1))
        draw.rectangle([(0, by - 1), (CELL, by + 1)], fill=TRUSS_GRAY)
        # Per-row fluorescent fixtures: 3-4 small rects with halo.
        n_fluor = rng.randint(3, 4)
        for j in range(n_fluor):
            fx_center = int((j + 0.5) * CELL / n_fluor) + rng.randint(-6, 6)
            half_len = rng.randint(8, 14)
            fx0, fx1 = fx_center - half_len, fx_center + half_len
            fy = by - rng.randint(4, 8)
            draw.rectangle((fx0, fy - 3, fx1, fy + 3), fill=FLUOR_HALO)
            draw.rectangle((fx0, fy - 1, fx1, fy + 1), fill=FLUOR)
    # 2 thin vertical droppers — suspended-fixture cables.
    for _ in range(2):
        dx = rng.randint(20, CELL - 20)
        dy_top = rng.randint(0, ceiling_h // 4)
        dy_bot = rng.randint(ceiling_h - 6, ceiling_h - 2)
        draw.rectangle((dx, dy_top, dx + 1, dy_bot), fill=DUCT_DARK)

    # --- Banner zone -----------------------------------------------------
    # Background of the banner strip is the same cool-white as the ceiling
    # so the banners read as hanging IN the ceiling space, not on a wall.
    draw.rectangle(
        [(0, y_banner_top), (CELL, y_product_top)], fill=CEIL_WHITE
    )
    n_banners = rng.randint(1, 3)
    banner_palette = (BANNER_RED, BANNER_DARK, BANNER_GREEN, BANNER_GOLD)
    for _ in range(n_banners):
        bw = rng.randint(CELL // 3, CELL // 2)
        bx0 = rng.randint(8, CELL - bw - 8)
        bx1 = bx0 + bw
        by0 = y_banner_top + rng.randint(2, 6)
        by1 = y_product_top - rng.randint(2, 5)
        color = rng.choice(banner_palette)
        draw.rectangle((bx0, by0, bx1, by1), fill=color)
        # Suspension cable up into the ceiling band (left + right edge).
        cable_x_left = bx0 + rng.randint(2, 6)
        cable_x_right = bx1 - rng.randint(2, 6)
        cable_top = max(0, y_banner_top - rng.randint(8, 14))
        draw.rectangle(
            (cable_x_left, cable_top, cable_x_left + 1, by0), fill=DUCT_DARK
        )
        draw.rectangle(
            (cable_x_right, cable_top, cable_x_right + 1, by0), fill=DUCT_DARK
        )

    # --- Product wall ----------------------------------------------------
    draw.rectangle(
        [(0, y_product_top), (CELL, y_lower_top)], fill=PROD_TAN
    )
    # 4-6 vertical bands of mixed product colors.
    n_bands = rng.randint(4, 6)
    band_palette = (PROD_RED, PROD_YEL, PROD_GRN, PROD_ORG, PROD_BRN)
    cursor = rng.randint(0, 12)
    for _ in range(n_bands):
        if cursor >= CELL:
            break
        bw = rng.randint(12, 25)
        color = rng.choice(band_palette)
        draw.rectangle(
            (cursor, y_product_top, cursor + bw, y_lower_top), fill=color
        )
        cursor += bw + rng.randint(6, 22)
    # Far-shelf horizontal line ~2/3 of the way down the band.
    sy = y_product_top + int(product_h * 0.66)
    draw.rectangle((0, sy - 1, CELL, sy + 1), fill=SHELF_DARK)
    # 2-3 small dark blobs — distant end-cap displays.
    for _ in range(rng.randint(2, 3)):
        ex = rng.randint(8, CELL - 24)
        ey = y_product_top + rng.randint(8, product_h - 16)
        ew = rng.randint(8, 18)
        eh = rng.randint(8, 14)
        draw.rectangle((ex, ey, ex + ew, ey + eh), fill=SHELF_DARK)

    # --- Lower shelving --------------------------------------------------
    draw.rectangle(
        [(0, y_lower_top), (CELL, y_floor_top)], fill=PROD_BRN
    )
    n_seps = rng.randint(4, 6)
    for j in range(n_seps):
        sx = int((j + 0.5) * CELL / n_seps) + rng.randint(-4, 4)
        draw.rectangle(
            (sx, y_lower_top, sx + 1, y_floor_top), fill=SHELF_DARK
        )

    # --- Floor band ------------------------------------------------------
    draw.rectangle([(0, y_floor_top), (CELL, CELL)], fill=FLOOR_GRAY)
    floor_mid = (y_floor_top + CELL) // 2
    draw.rectangle(
        (0, floor_mid, CELL, floor_mid + 1),
        fill=(FLOOR_GRAY[0] - 20, FLOOR_GRAY[1] - 20, FLOOR_GRAY[2] - 20),
    )

    return img.filter(ImageFilter.GaussianBlur(radius=1.5))


def floor_cell() -> Image.Image:
    """Bottom face: solid floor gray. The floor plane covers it in practice."""
    return Image.new("RGB", (CELL, CELL), FLOOR_GRAY)


def main() -> None:
    out = Image.new("RGB", (W, H), EMPTY)
    out.paste(ceiling_cell(), (1 * CELL, 0 * CELL))            # U
    out.paste(side_cell(seed=10), (0 * CELL, 1 * CELL))        # L
    out.paste(side_cell(seed=20), (1 * CELL, 1 * CELL))        # F
    out.paste(side_cell(seed=30), (2 * CELL, 1 * CELL))        # R
    out.paste(side_cell(seed=40), (3 * CELL, 1 * CELL))        # B
    out.paste(floor_cell(),       (1 * CELL, 2 * CELL))        # D
    out.save(DST, format="PNG", optimize=True)
    print(f"wrote {DST}  ({out.size[0]}x{out.size[1]})")


if __name__ == "__main__":
    main()
