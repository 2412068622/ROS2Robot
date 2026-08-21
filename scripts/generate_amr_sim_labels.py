#!/usr/bin/env python3
"""Generate high-contrast Gazebo label textures for the AMR showcase."""

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "src" / "robot_simulation" / "media" / "labels"

FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
]

OBJECT_LABELS = {
    "robot_1": ("主车", "#2f80ed"),
    "robot_2": ("副车", "#27ae60"),
    "dock": ("充电桩", "#2ecc71"),
    "receiving": ("收货口", "#16a085"),
    "door": ("门禁", "#e67e22"),
    "elevator": ("电梯", "#8e44ad"),
    "floor_2": ("二楼", "#7f8c8d"),
    "storage_a": ("货架A", "#f39c12"),
    "storage_b": ("货架B", "#f1c40f"),
    "packing": ("打包台", "#3498db"),
    "traffic": ("路口", "#d35400"),
    "slow_zone": ("限速区", "#2980b9"),
    "confirm": ("确认点", "#c0392b"),
    "payload": ("货物", "#f39c12"),
}

STATUS_LABELS = {
    "charging_status": ("充电中", "#2ecc71"),
    "dock_retry": ("对桩重试", "#f39c12"),
    "opportunity_charge": ("机会充电", "#27ae60"),
    "top_module": ("上装动作", "#1abc9c"),
    "localization": ("定位恢复", "#3498db"),
    "manual_takeover": ("人工接管", "#e74c3c"),
    "confirm_wait": ("等待确认", "#f1c40f"),
    "route_block": ("路段封锁", "#e74c3c"),
    "traffic_deadlock": ("交通冲突", "#c0392b"),
    "route_open": ("路段恢复", "#2ecc71"),
    "speed_limit_active": ("动态限速", "#3498db"),
    "speed_ok": ("速度恢复", "#2ecc71"),
    "obstacle": ("障碍阻塞", "#e74c3c"),
    "obstacle_clear": ("清障完成", "#2ecc71"),
}


def load_font(size: int) -> ImageFont.FreeTypeFont:
    for candidate in FONT_CANDIDATES:
        path = Path(candidate)
        if path.exists():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def fit_font(text: str, max_width: int, max_height: int) -> ImageFont.FreeTypeFont:
    for size in range(126, 34, -4):
        font = load_font(size)
        bbox = ImageDraw.Draw(Image.new("RGBA", (1, 1))).textbbox((0, 0), text, font=font)
        if bbox[2] - bbox[0] <= max_width and bbox[3] - bbox[1] <= max_height:
            return font
    return load_font(34)


def draw_label(filename: str, text: str, accent: str, status: bool) -> None:
    width, height = 1024, 256
    image = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)

    margin = 26
    radius = 42
    bg = (18, 24, 32, 232) if not status else (28, 18, 18, 238)
    border = accent
    draw.rounded_rectangle(
        (margin, margin, width - margin, height - margin),
        radius=radius,
        fill=bg,
        outline=border,
        width=8,
    )
    draw.rounded_rectangle(
        (margin + 18, margin + 24, margin + 56, height - margin - 24),
        radius=18,
        fill=accent,
    )

    font = fit_font(text, width - 190, height - 88)
    bbox = draw.textbbox((0, 0), text, font=font, stroke_width=2)
    text_w = bbox[2] - bbox[0]
    text_h = bbox[3] - bbox[1]
    x = (width + 62 - text_w) / 2
    y = (height - text_h) / 2 - 8
    draw.text(
        (x, y),
        text,
        font=font,
        fill=(255, 255, 255, 255),
        stroke_width=3,
        stroke_fill=(0, 0, 0, 185),
    )

    # Add a subtle lower highlight so the sign remains readable on gray floors.
    draw.line((margin + 78, height - margin - 18, width - margin - 34, height - margin - 18),
              fill=(255, 255, 255, 60), width=3)
    image.save(OUT_DIR / filename)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for stem, (text, accent) in OBJECT_LABELS.items():
        draw_label(f"{stem}.png", text, accent, status=False)
    for stem, (text, accent) in STATUS_LABELS.items():
        draw_label(f"{stem}.png", text, accent, status=True)


if __name__ == "__main__":
    main()
