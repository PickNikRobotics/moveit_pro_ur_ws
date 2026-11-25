# Agent comments

Detail that is too long for an inline comment, organised by file and then by code location. Keep entries short.

## objectives/calibration/board_offsets.yaml

### `wall_corner_in_wall_board` (line 5)
NRF 2027 cell, 2026-11-25. A new cell built to mimic the old one, with the board remounted.
- a, b tape-measured from the wall's left and top edges to the pattern's black edge: 443 mm, 543 mm.
- z (t) is NOT measured. It is the old cell's fitted value, kept by choice. On the old cell it was fitted to AprilTag poses and is far above the print thickness, so it may hide an error in the board solve or the measurement. It moves the whole shelf in depth: check the depth first in the overlay.
- The old cell's measurement and fit history is in git on `shelf-stocking-pro10`.

### `top_rack_from_wall_corner` (line 29), `bottom_rack_from_wall_corner` (line 34)
NRF 2027 cell: not measured. Each is the old cell's value shifted by the middle rack's change, so its spacing from the middle rack is unchanged. Measure them if the arm comes near them.

### `middle_rack_from_wall_corner` (line 39)
NRF 2027 cell, tape-measured. L = wall left edge to rack left end (29 mm), D = wall face to rack front face (413 mm), H = wall top edge to rack top surface at the front (840 mm).
- x = L + half the mesh length; y = -D + the mesh's lip depth (its front face sits in front of the origin); z = -H.
- Mesh length and lip depth come from the bounding box of `description/chunky_shelf_only.stl`.
