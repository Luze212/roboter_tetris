from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


HERE = Path(__file__).parent
FONT_PATH = Path(r"C:\Windows\Fonts\arial.ttf")
FONT_BOLD_PATH = Path(r"C:\Windows\Fonts\arialbd.ttf")
if not FONT_PATH.exists():  # macOS
    FONT_PATH = Path("/System/Library/Fonts/Supplemental/Arial.ttf")
    FONT_BOLD_PATH = Path("/System/Library/Fonts/Supplemental/Arial Bold.ttf")


def font(size: int, bold: bool = False):
    path = FONT_BOLD_PATH if bold else FONT_PATH
    return ImageFont.truetype(path, size)


TITLE = font(36, True)
HEADING = font(27, True)
LABEL = font(23)
SMALL = font(20)
NOTE = font(18)

INK = "#17212b"
MUTED = "#4b5563"
BLUE = "#1d70b8"
BELT = "#e7eef4"
BELT_EDGE = "#305b78"
ORANGE = "#e87520"
CAMERA = "#d6dce1"
ROBOT = "#dbe8d6"
CRATE = "#f5dfbd"


def text_box(draw, xy, text, used_font, fill=INK, padding=8):
    x, y = xy
    left, top, right, bottom = draw.textbbox((x, y), text, font=used_font)
    draw.rounded_rectangle(
        (left - padding, top - padding, right + padding, bottom + padding),
        radius=5,
        fill="white",
    )
    draw.text((x, y), text, font=used_font, fill=fill)


def arrow(draw, start, end, fill, width=5, head=16):
    draw.line((start, end), fill=fill, width=width)
    x1, y1 = start
    x2, y2 = end
    if abs(x2 - x1) >= abs(y2 - y1):
        points = [(x2, y2), (x2 - head, y2 - head // 2), (x2 - head, y2 + head // 2)]
    else:
        points = [(x2, y2), (x2 - head // 2, y2 + head), (x2 + head // 2, y2 + head)]
        if y2 < y1:
            points = [(x2, y2), (x2 - head // 2, y2 + head), (x2 + head // 2, y2 + head)]
    draw.polygon(points, fill=fill)


def dashed_rect(draw, box, fill, width=3, dash=14, gap=10):
    x0, y0, x1, y1 = box
    for a, b, fixed, horizontal in ((x0, x1, y0, True), (x0, x1, y1, True),
                                    (y0, y1, x0, False), (y0, y1, x1, False)):
        pos = a
        while pos < b:
            stop = min(pos + dash, b)
            if horizontal:
                draw.line((pos, fixed, stop, fixed), fill=fill, width=width)
            else:
                draw.line((fixed, pos, fixed, stop), fill=fill, width=width)
            pos += dash + gap


def plan_view():
    image = Image.new("RGB", (1600, 1100), "white")
    draw = ImageDraw.Draw(image)
    draw.text((70, 52), "Draufsicht des Versuchsaufbaus", font=TITLE, fill=INK)
    draw.text((70, 102), "Globale Koordinaten im Bezugssystem world", font=NOTE, fill=MUTED)

    x_min, x_max = -1.45, 0.35
    y_min, y_max = -0.55, 1.30
    x_scale, y_scale = 600, 470
    x_offset, y_offset = 190, 120

    def point(x, y):
        return x_offset + (x - x_min) * x_scale, y_offset + (y_max - y) * y_scale

    # Förderband und Arbeitsbereich
    band_left, band_top = point(-1.275, 1.140)
    band_right, band_bottom = point(-0.480, -0.367)
    draw.rounded_rectangle((band_left, band_top, band_right, band_bottom), radius=8,
                           fill=BELT, outline=BELT_EDGE, width=4)
    text_box(draw, (band_left + 145, band_top + 28), "Förderband", HEADING)
    work_left, work_top = point(-1.000, 0.445)
    work_right, work_bottom = point(-0.530, -0.320)
    dashed_rect(draw, (work_left, work_top, work_right, work_bottom), BLUE)

    # Bandlaufrichtung
    center_x = (band_left + band_right) / 2
    arrow(draw, (center_x, band_top + 120), (center_x, band_bottom - 110), ORANGE, 7, 24)
    text_box(draw, (band_right + 55, (band_top + band_bottom) / 2 - 30),
             "Bandlaufrichtung", LABEL, ORANGE)

    # Basiskamera
    cam_x, cam_y = point(-0.880, 1.140)
    draw.rounded_rectangle((cam_x - 42, cam_y - 52, cam_x + 42, cam_y - 20), radius=4,
                           fill=CAMERA, outline=MUTED, width=3)
    draw.line((cam_x, cam_y - 20, cam_x, cam_y), fill=MUTED, width=3)
    text_box(draw, (90, 160), "Basiskamera", HEADING)
    draw.line((260, 180, cam_x - 48, cam_y - 36), fill=MUTED, width=2)
    text_box(draw, (90, 199), "mittig am Bandanfang, ca. 0,850 m über Band", NOTE, MUTED)

    # Ablagekiste und Roboterbasis
    crate_x, crate_y = point(-0.316, 0.476)
    draw.rounded_rectangle((crate_x - 82, crate_y - 50, crate_x + 82, crate_y + 50), radius=4,
                           fill=CRATE, outline="#9b6622", width=3)
    text_box(draw, (crate_x - 62, crate_y - 15), "Ablagekiste", SMALL)
    base_x, base_y = point(0.000, 0.000)
    draw.ellipse((base_x - 72, base_y - 72, base_x + 72, base_y + 72),
                 fill=ROBOT, outline="#46734a", width=4)
    text_box(draw, (base_x - 38, base_y - 15), "UR10e", SMALL)

    # conveyor_frame am Bandanfang
    conveyor_x, conveyor_y = point(-0.880, 1.140)
    draw.ellipse((conveyor_x - 5, conveyor_y - 5, conveyor_x + 5, conveyor_y + 5), fill=BLUE)
    arrow(draw, (conveyor_x, conveyor_y), (conveyor_x - 90, conveyor_y), BLUE, 4, 14)
    arrow(draw, (conveyor_x, conveyor_y), (conveyor_x, conveyor_y + 90), BLUE, 4, 14)
    text_box(draw, (conveyor_x - 160, conveyor_y + 102), "conveyor_frame", NOTE, BLUE)
    text_box(draw, (conveyor_x - 112, conveyor_y - 30), "X+", NOTE, BLUE)
    text_box(draw, (conveyor_x + 18, conveyor_y + 82), "Y+", NOTE, BLUE)

    # world an der Roboterbasis
    draw.ellipse((base_x - 5, base_y - 5, base_x + 5, base_y + 5), fill=BLUE)
    arrow(draw, (base_x, base_y), (base_x + 110, base_y), BLUE, 4, 14)
    arrow(draw, (base_x, base_y), (base_x, base_y - 100), BLUE, 4, 14)
    text_box(draw, (base_x + 125, base_y + 14), "world", NOTE, BLUE)
    text_box(draw, (base_x + 92, base_y - 34), "X+", NOTE, BLUE)
    text_box(draw, (base_x + 18, base_y - 124), "Y+", NOTE, BLUE)

    # Nicht überlappende Maß- und Arbeitsbereichshinweise
    draw.line((work_right, work_top, 950, 340), fill=BLUE, width=2)
    text_box(draw, (970, 305), "Greifbarer Arbeitsbereich", HEADING, BLUE)
    text_box(draw, (970, 346), "x = -1,000 bis -0,530 m", LABEL, BLUE)
    text_box(draw, (970, 383), "y = -0,320 bis 0,445 m", LABEL, BLUE)
    text_box(draw, (970, 466), "Ablagepose", HEADING)
    text_box(draw, (970, 507), "x = -0,316 m", LABEL)
    text_box(draw, (970, 544), "y = 0,476 m", LABEL)
    text_box(draw, (970, 581), "z = 0,420 m", LABEL)
    text_box(draw, (970, 825), "Bandkanten: x = -1,275 bis -0,480 m", LABEL, MUTED)
    text_box(draw, (970, 865), "Nutzbare gerade Förderstrecke: ca. 1,507 m", LABEL, MUTED)
    draw.text((70, 1030), "Z+ zeigt von der Bandebene nach oben und ist in der Draufsicht nicht dargestellt.",
              font=NOTE, fill=MUTED)
    image.save(HERE / "systemaufbau_draufsicht.png")


def side_view():
    image = Image.new("RGB", (1600, 900), "white")
    draw = ImageDraw.Draw(image)
    draw.text((70, 52), "Seitenansicht der relevanten Höhen", font=TITLE, fill=INK)
    draw.text((70, 102), "Flanschhöhen im globalen Bezugssystem world", font=NOTE, fill=MUTED)

    z_scale, z_base = 620, 780

    def y_of_z(z):
        return z_base - z * z_scale

    band_surface = 0.054
    x_left, x_right = 230, 900
    camera_z = band_surface + 0.850
    camera_y = y_of_z(camera_z)
    belt_y = y_of_z(band_surface)

    # einfacher Kamerarahmen und Förderband
    draw.line((280, camera_y - 48, 280, belt_y), fill="#697782", width=8)
    draw.line((850, camera_y - 48, 850, belt_y), fill="#697782", width=8)
    draw.line((280, camera_y - 48, 850, camera_y - 48), fill="#697782", width=8)
    draw.rounded_rectangle((520, camera_y - 32, 610, camera_y + 6), radius=4,
                           fill=CAMERA, outline=MUTED, width=3)
    draw.rounded_rectangle((x_left, belt_y, x_right, belt_y + 52), radius=5,
                           fill=BELT, outline=BELT_EDGE, width=4)
    text_box(draw, (490, belt_y + 12), "Förderband", HEADING)

    # Höhenlinien; Beschriftungen beginnen immer rechts neben dem Linienende
    levels = [
        (0.600, "obere Arbeitsraumgrenze: z = 0,600 m", BLUE, 3, None, 392),
        (0.490, "Transferhöhe: z = 0,490 m", "#6d9bc1", 2, (10, 8), 454),
        (0.450, "Folge- und Beobachtungshöhe: z = 0,450 m", "#6d9bc1", 2, (10, 8), 525),
        (0.299, "untere Arbeitsraumgrenze: z = 0,299 m", BLUE, 3, None, 570),
    ]
    for z, label, color, width, dash, label_y in levels:
        y = y_of_z(z)
        if dash:
            x = 300
            while x < 880:
                draw.line((x, y, min(x + dash[0], 880), y), fill=color, width=width)
                x += dash[0] + dash[1]
        else:
            draw.line((300, y, 880, y), fill=color, width=width)
        draw.line((880, y, 900, y, 915, label_y + 16), fill=color, width=2)
        text_box(draw, (930, label_y), label, LABEL, color)

    text_box(draw, (660, camera_y - 6), "Basiskamera", HEADING)
    text_box(draw, (660, camera_y + 32), "ca. 0,850 m über Band", LABEL, MUTED)
    arrow(draw, (145, belt_y), (145, camera_y), ORANGE, 4, 14)
    text_box(draw, (65, (belt_y + camera_y) / 2 - 15), "ca. 0,850 m", LABEL, ORANGE)
    arrow(draw, (1180, belt_y), (1180, y_of_z(0.720)), BLUE, 4, 14)
    text_box(draw, (1200, y_of_z(0.720) - 14), "Z+", LABEL, BLUE)
    text_box(draw, (1140, belt_y + 30), "world", NOTE, BLUE)
    draw.text((70, 845), "Die Höhen betreffen den Flansch ur_tool0. Der Greifer ist nicht dargestellt.",
              font=NOTE, fill=MUTED)
    image.save(HERE / "systemaufbau_seitenansicht.png")


if __name__ == "__main__":
    plan_view()
    side_view()
