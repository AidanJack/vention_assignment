"""Palletizing Grid Calculations.

============================

Calculate place positions for boxes in an N×M grid pattern.
"""


def calculate_place_positions(
    rows: int,
    cols: int,
    box_size_mm: tuple[float, float, float],
    pallet_origin_mm: tuple[float, float, float],
    spacing_mm: float = 10.0,
) -> list[tuple[float, float, float]]:
    """Calculate TCP positions for placing boxes in a grid pattern.

    Args:
        rows: Number of rows (N)
        cols: Number of columns (M)
        box_size_mm: (width, depth, height) of each box in mm
        pallet_origin_mm: (x, y, z) position of the first box placement
        spacing_mm: Gap between adjacent boxes (default 10mm)

    Returns:
        List of (x, y, z) TCP target positions, ordered for row-by-row filling.
    """
    box_width_mm, box_depth_mm, _ = box_size_mm
    origin_x_mm, origin_y_mm, origin_z_mm = pallet_origin_mm

    x_step_mm = box_width_mm + spacing_mm
    y_step_mm = box_depth_mm + spacing_mm

    positions: list[tuple[float, float, float]] = []

    for row in range(rows):
        for col in range(cols):
            x_mm = origin_x_mm + (col * x_step_mm)
            y_mm = origin_y_mm + (row * y_step_mm)
            positions.append((x_mm, y_mm, origin_z_mm))

    return positions
