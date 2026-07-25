from dataclasses import dataclass
import math


@dataclass
class BoxSpec:
    name: str
    length_mm: float
    width_mm: float
    height_mm: float
    eof_open_mm: float
    eof_close_mm: float


@dataclass
class SlotPose:
    slot_index: int
    layer: int
    row: int
    col: int
    x_mm: float
    y_mm: float
    z_top_mm: float


class BasePlanner:
    def __init__(
        self,
        origin_x_mm: float,
        origin_y_mm: float,
        floor_z_mm: float,
        inner_width_mm: float,
        inner_length_mm: float,
        inner_height_mm: float,
        wall_clearance_mm: float,
        box_gap_mm: float,
    ):
        self.origin_x_mm = origin_x_mm
        self.origin_y_mm = origin_y_mm
        self.floor_z_mm = floor_z_mm
        self.inner_width_mm = inner_width_mm
        self.inner_length_mm = inner_length_mm
        self.inner_height_mm = inner_height_mm
        self.wall_clearance_mm = wall_clearance_mm
        self.box_gap_mm = box_gap_mm

    def capacity_for(self, box: BoxSpec) -> dict:
        usable_w = self.inner_width_mm - 2 * self.wall_clearance_mm
        usable_l = self.inner_length_mm - 2 * self.wall_clearance_mm
        usable_h = self.inner_height_mm

        pitch_x = box.width_mm + self.box_gap_mm
        pitch_y = box.length_mm + self.box_gap_mm
        pitch_z = box.height_mm

        cols = max(0, math.floor((usable_w + self.box_gap_mm) / pitch_x))
        rows = max(0, math.floor((usable_l + self.box_gap_mm) / pitch_y))
        layers = max(0, math.floor(usable_h / pitch_z))

        return {
            "cols": cols,
            "rows": rows,
            "layers": layers,
            "total": cols * rows * layers,
        }

    def next_slot(self, box: BoxSpec, occupied_count: int) -> SlotPose | None:
        cap = self.capacity_for(box)
        cols = cap["cols"]
        rows = cap["rows"]
        layers = cap["layers"]

        if cols == 0 or rows == 0 or layers == 0:
            return None
        if occupied_count >= cap["total"]:
            return None

        per_layer = cols * rows
        layer = occupied_count // per_layer
        rem = occupied_count % per_layer
        row = rem // cols
        col = rem % cols

        x_mm = (
            self.origin_x_mm
            + self.wall_clearance_mm
            + (box.width_mm / 2.0)
            + col * (box.width_mm + self.box_gap_mm)
        )
        y_mm = (
            self.origin_y_mm
            + self.wall_clearance_mm
            + (box.length_mm / 2.0)
            + row * (box.length_mm + self.box_gap_mm)
        )
        z_top_mm = self.floor_z_mm + (layer + 1) * box.height_mm

        return SlotPose(
            slot_index=occupied_count,
            layer=layer,
            row=row,
            col=col,
            x_mm=x_mm,
            y_mm=y_mm,
            z_top_mm=z_top_mm,
        )
