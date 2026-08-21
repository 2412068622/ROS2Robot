#!/usr/bin/env python3
"""Generate texture-free Gazebo text labels as lightweight OBJ meshes."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
WORLD_TEMPLATE = ROOT / "src" / "robot_simulation" / "worlds" / "indoor_room.sdf.xacro"
MESH_DIR = ROOT / "src" / "robot_simulation" / "media" / "text_meshes"

FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
]


@dataclass(frozen=True)
class LabelSpec:
    model: str
    text: str
    pose: str
    height_m: float
    color: tuple[float, float, float]

    @property
    def mesh_name(self) -> str:
        return f"{self.model}.obj"


BLACK = (0.0, 0.0, 0.0)


OBJECT_LABELS = [
    LabelSpec("label_robot_1", "主车", "0.00 0.00 1.70 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_robot_2", "副车", "-3.90 -3.10 1.60 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_dock", "充电桩", "-3.90 -3.80 1.65 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_receiving", "收货口", "-3.70 0.65 1.55 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_door", "门禁", "-4.15 0.00 1.70 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_elevator", "电梯", "3.65 2.85 1.80 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_floor_2", "二楼", "3.65 3.85 1.65 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_storage_a", "货架A", "1.50 1.50 1.60 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_storage_b", "货架B", "-2.00 2.00 1.60 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_packing", "打包台", "2.00 -2.00 1.60 0 0 0.00", 0.28, BLACK),
    LabelSpec("label_traffic", "路口", "0.00 0.00 1.35 0 0 0.00", 0.26, BLACK),
    LabelSpec("label_slow_zone", "限速区", "1.20 -0.25 1.25 0 0 0.00", 0.26, BLACK),
    LabelSpec("label_confirm", "确认点", "-3.32 1.03 1.35 0 0 0.00", 0.26, BLACK),
    LabelSpec("label_payload", "货物", "-3.35 -0.45 1.20 0 0 0.00", 0.26, BLACK),
]


STATUS_LABELS = [
    LabelSpec("label_charging_status", "充电中", "-4.30 -3.80 1.10 0 0 0.00", 0.26, BLACK),
    LabelSpec("label_docking_retry_status", "对桩重试", "-4.30 -3.42 -2 0 0 0.00", 0.26, BLACK),
    LabelSpec("label_opportunity_charge_status", "机会充电", "-3.55 -3.42 -2 0 0 0.00", 0.26, BLACK),
    LabelSpec("label_top_module_status", "上装动作", "-3.35 -0.90 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_localization_status", "定位恢复", "-3.35 -3.55 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_manual_takeover_status", "人工接管", "-4.35 3.65 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_operator_confirmation_status", "等待确认", "0.00 0.00 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_traffic_block_status", "路段封锁", "0.00 0.00 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_traffic_deadlock_status", "交通冲突", "0.00 0.00 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_traffic_reopen_status", "路段恢复", "0.00 0.00 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_speed_limit_status", "动态限速", "1.20 -0.25 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_speed_restored_status", "速度恢复", "1.20 -0.25 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_obstacle_status", "障碍阻塞", "0.00 0.00 -2 0 0 0.00", 0.25, BLACK),
    LabelSpec("label_obstacle_clear_status", "清障完成", "0.65 3.65 -2 0 0 0.00", 0.25, BLACK),
]


def load_font(size: int) -> ImageFont.FreeTypeFont:
    for candidate in FONT_CANDIDATES:
        path = Path(candidate)
        if path.exists():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def char_mask(char: str, pixel_height: int = 36) -> Image.Image:
    source_font = load_font(96)
    draw = ImageDraw.Draw(Image.new("L", (1, 1)))
    bbox = draw.textbbox((0, 0), char, font=source_font, stroke_width=1)
    width = bbox[2] - bbox[0] + 12
    height = bbox[3] - bbox[1] + 12
    image = Image.new("L", (width, height), 0)
    draw = ImageDraw.Draw(image)
    draw.text((6 - bbox[0], 6 - bbox[1]), char, font=source_font, fill=255, stroke_width=1, stroke_fill=255)
    scale = pixel_height / image.height
    resized = image.resize((max(1, int(image.width * scale)), pixel_height), Image.Resampling.LANCZOS)
    return resized.point(lambda value: 255 if value > 72 else 0)


def mask_for_vertical_text(text: str, char_pixel_height: int = 36, gap_px: int = 4) -> Image.Image:
    masks = [char_mask(char, char_pixel_height) for char in text]
    width = max(mask.width for mask in masks)
    height = sum(mask.height for mask in masks) + gap_px * max(0, len(masks) - 1)
    image = Image.new("L", (width, height), 0)
    y = 0
    for mask in masks:
        x = (width - mask.width) // 2
        image.paste(mask, (x, y))
        y += mask.height + gap_px
    return image


def row_runs(mask: Image.Image) -> Iterable[tuple[int, int, int]]:
    for y in range(mask.height):
        start = None
        for x in range(mask.width):
            on = mask.getpixel((x, y)) > 0
            if on and start is None:
                start = x
            elif not on and start is not None:
                yield y, start, x - 1
                start = None
        if start is not None:
            yield y, start, mask.width - 1


def write_obj(spec: LabelSpec) -> None:
    mask = mask_for_vertical_text(spec.text)
    target_height = max(spec.height_m, 0.22 * len(spec.text))
    cell = target_height / mask.height
    text_width = mask.width * cell
    text_height = mask.height * cell
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    for y_px, x0, x1 in row_runs(mask):
        left = x0 * cell - text_width / 2
        right = (x1 + 1) * cell - text_width / 2
        top = text_height / 2 - y_px * cell
        bottom = text_height / 2 - (y_px + 1) * cell
        base = len(vertices) + 1
        vertices.extend([(left, bottom, 0.0), (right, bottom, 0.0), (right, top, 0.0), (left, top, 0.0)])
        faces.append((base, base + 1, base + 2))
        faces.append((base, base + 2, base + 3))

    material_file = "label_text_black.mtl"
    lines = ["# Generated AMR text label mesh", f"mtllib {material_file}", "o " + spec.model, "usemtl label_text_black"]
    lines.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices)
    lines.extend(f"f {a} {b} {c}" for a, b, c in faces)
    (MESH_DIR / spec.mesh_name).write_text("\n".join(lines) + "\n", encoding="utf-8")


def model_xml(spec: LabelSpec) -> str:
    r, g, b = spec.color
    return f"""    <model name="{spec.model}">
      <static>false</static>
      <pose>{spec.pose}</pose>
      <link name="link">
        <gravity>false</gravity>
        <visual name="text_mesh_visual">
          <cast_shadows>false</cast_shadows>
          <geometry><mesh><uri>file://${{simulation_media_path}}/text_meshes/{spec.mesh_name}</uri></mesh></geometry>
          <material>
            <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
            <emissive>0 0 0 1</emissive>
            <double_sided>true</double_sided>
          </material>
        </visual>
      </link>
    </model>"""


def generated_section() -> str:
    parts = ["    <!-- BEGIN AMR_TEXT_LABEL_MODELS -->"]
    parts.extend(model_xml(spec) for spec in OBJECT_LABELS)
    parts.append("    <!-- END AMR_TEXT_LABEL_MODELS -->")
    parts.append("")
    parts.append("    <!-- BEGIN AMR_STATUS_LABEL_MODELS -->")
    parts.extend(model_xml(spec) for spec in STATUS_LABELS)
    parts.append("    <!-- END AMR_STATUS_LABEL_MODELS -->")
    return "\n".join(parts)


def main() -> None:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for stale in MESH_DIR.glob("label_*.obj"):
        stale.unlink()
    (MESH_DIR / "label_text_black.mtl").write_text(
        "\n".join(
            [
                "newmtl label_text_black",
                "Ka 0.000 0.000 0.000",
                "Kd 0.000 0.000 0.000",
                "Ks 0.000 0.000 0.000",
                "Ke 0.000 0.000 0.000",
                "d 1.0",
                "illum 1",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    for spec in [*OBJECT_LABELS, *STATUS_LABELS]:
        write_obj(spec)

    text = WORLD_TEMPLATE.read_text(encoding="utf-8")
    start = text.index("    <!-- BEGIN AMR_TEXT_LABEL_MODELS -->")
    end_marker = "    <!-- END AMR_STATUS_LABEL_MODELS -->"
    end = text.index(end_marker, start) + len(end_marker)
    WORLD_TEMPLATE.write_text(text[:start] + generated_section() + text[end:], encoding="utf-8")


if __name__ == "__main__":
    main()
