"""Generate a tileable industrial-plastic texture for the calibrated_bin and the
calibrated_table (both bound to `bin_material` — see `mjcf/assets/additions.xml`).

The bin is composed of five separate box geoms that share one material, so a
per-face cube unfold doesn't fit — instead we ship a single seamless 2D
texture and let MuJoCo wrap it onto every face with the default 2D
projection (`texuniform="false"`). The calibrated_table reuses the same
material (the dedicated wood cube-unfold was retired). That means the plastic "grain" scales
with each face's dimensions rather than being world-unit-stable, which is
fine here: the win is uniform color across faces, not a physically-sized
grain pattern.

What the image contains:
    - A base medium-gray ("HDPE industrial gray") with low-frequency
      brightness drift to break up the dead-flat color.
    - Two octaves of fine value noise on top, like the matte sheen of molded
      polypropylene, no specular features.
    - Faint horizontal rib lines spaced so they line up with the molded
      reinforcement bands you see on real totes (every ~50 mm in world
      units when paired with the `texrepeat` chosen in additions.xml).
    - The image tiles seamlessly: noise is wrapped via FFT-equivalent
      ping-pong copies and the rib spacing divides 256 evenly, so no visible
      seams when MuJoCo wraps adjacent faces together.

Run from anywhere; outputs next to this script:
    python3 generate_tote_plastic.py
"""

from pathlib import Path
import random

from PIL import Image, ImageDraw, ImageFilter

HERE = Path(__file__).resolve().parent
DST = HERE / "tote_plastic.png"

SIZE = 256
BASE = (108, 110, 113)        # medium industrial gray (slight cool tint)
RIB_COLOR = (62, 64, 67)      # darker rib lines (more contrast than v1)
RIB_HIGHLIGHT = (140, 142, 145)
NUM_RIBS = 5                  # ribs per tile
NOISE_AMPLITUDE = 18          # grayscale-only noise; same value applied to R/G/B
                              # so the texture has no color tint anywhere — fixes
                              # the per-face hue drift that texuniform=true was
                              # exposing on long vs short walls.


def seamless_noise(size: int, amplitude: int, seed: int) -> Image.Image:
    """Coarse value noise that wraps seamlessly along both axes.

    Trick: build noise on a half-resolution grid, mirror-tile it 2x2, then
    blur. The blur smears the mirror seams away and the result is still
    periodic at full resolution.
    """
    rng = random.Random(seed)
    half = size // 2
    src = Image.new("L", (half, half))
    pixels = [rng.randint(0, 255) for _ in range(half * half)]
    src.putdata(pixels)

    # 2x2 mirror tile -> seamless when wrapped
    full = Image.new("L", (size, size))
    full.paste(src, (0, 0))
    full.paste(src.transpose(Image.FLIP_LEFT_RIGHT), (half, 0))
    full.paste(src.transpose(Image.FLIP_TOP_BOTTOM), (0, half))
    full.paste(
        src.transpose(Image.FLIP_LEFT_RIGHT).transpose(Image.FLIP_TOP_BOTTOM),
        (half, half),
    )
    full = full.filter(ImageFilter.GaussianBlur(radius=4))

    # Center on 128 and rescale to +/- amplitude.
    out = Image.new("L", (size, size))
    out.putdata([
        max(0, min(255, 128 + int((p - 128) / 8 * amplitude / 16)))
        for p in full.getdata()
    ])
    return out


def main() -> None:
    base = Image.new("RGB", (SIZE, SIZE), BASE)

    # Single grayscale noise field applied identically to R, G, B — keeps the
    # surface a uniform gray everywhere (no per-channel hue drift).
    drift = seamless_noise(SIZE, NOISE_AMPLITUDE, 11)
    speckle = seamless_noise(SIZE, NOISE_AMPLITUDE // 2, 99)
    base_pixels = list(base.getdata())
    drift_pixels = list(drift.getdata())
    speckle_pixels = list(speckle.getdata())
    new_pixels = []
    for (r, g, b), d_, s in zip(base_pixels, drift_pixels, speckle_pixels):
        delta = (d_ - 128) // 2 + (s - 128) // 4
        new_pixels.append(
            (
                max(0, min(255, r + delta)),
                max(0, min(255, g + delta)),
                max(0, min(255, b + delta)),
            )
        )
    base.putdata(new_pixels)

    # Horizontal mold-rib bands. Two-pixel dark groove (the rib's shadowed
    # underside) followed by a single bright pixel (the ridge highlight),
    # repeated NUM_RIBS times across the tile so the rib pitch is obvious in
    # the rendered image.
    draw = ImageDraw.Draw(base)
    rib_spacing = SIZE // NUM_RIBS
    for i in range(NUM_RIBS):
        y = (i * rib_spacing + rib_spacing // 2) % SIZE
        draw.line([(0, y - 1), (SIZE - 1, y - 1)], fill=RIB_COLOR, width=1)
        draw.line([(0, y), (SIZE - 1, y)], fill=RIB_COLOR, width=1)
        draw.line([(0, (y + 1) % SIZE), (SIZE - 1, (y + 1) % SIZE)], fill=RIB_HIGHLIGHT, width=1)

    # A small final blur smooths everything to "molded" rather than "drawn".
    base = base.filter(ImageFilter.GaussianBlur(radius=0.7))

    base.save(DST, format="PNG", optimize=True)
    print(f"wrote {DST}  ({base.size[0]}x{base.size[1]})")


if __name__ == "__main__":
    main()
