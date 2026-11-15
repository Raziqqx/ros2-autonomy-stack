#!/usr/bin/env python3
"""Generates the simulator's warehouse map (PGM + map_server YAML).

The map is described as axis-aligned boxes in metres so it is easy to read and
regenerate; run from the repo root:  python3 scripts/make_map.py
"""
from pathlib import Path

RESOLUTION = 0.05  # m per cell
WIDTH_M, HEIGHT_M = 12.0, 9.0
WALL = 0.1

# (x0, y0, x1, y1) in metres, map origin at the lower-left corner.
OBSTACLES = [
    (0.0, 0.0, WIDTH_M, WALL),  # outer walls
    (0.0, HEIGHT_M - WALL, WIDTH_M, HEIGHT_M),
    (0.0, 0.0, WALL, HEIGHT_M),
    (WIDTH_M - WALL, 0.0, WIDTH_M, HEIGHT_M),
    (3.5, 4.5, 7.5, 5.0),  # long shelf
    (9.0, 2.0, 9.5, 6.0),  # shelf along y
    (6.0, 1.8, 6.4, 2.2),  # pillar
    (1.5, 6.5, 2.5, 7.5),  # crate
    (5.0, 7.0, 5.3, HEIGHT_M),  # wall stub
    (10.5, 7.2, 11.0, 7.7),  # pillar
]


def main() -> None:
    out = Path(__file__).resolve().parent.parent / "src/autonomy_bringup/maps"
    w, h = round(WIDTH_M / RESOLUTION), round(HEIGHT_M / RESOLUTION)
    pixels = bytearray([254]) * (w * h)  # free
    for x0, y0, x1, y1 in OBSTACLES:
        for cy in range(round(y0 / RESOLUTION), round(y1 / RESOLUTION)):
            for cx in range(round(x0 / RESOLUTION), round(x1 / RESOLUTION)):
                row = h - 1 - cy  # PGM rows run top to bottom
                pixels[row * w + cx] = 0  # occupied
    (out / "warehouse.pgm").write_bytes(f"P5\n{w} {h}\n255\n".encode() + bytes(pixels))
    (out / "warehouse.yaml").write_text(
        "image: warehouse.pgm\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION}\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )


if __name__ == "__main__":
    main()
