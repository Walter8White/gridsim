"""OpenMV H7 surface classifier for adaptive KEYENCE presets.

Copy this file to the OpenMV IDE and run it, or save it as main.py on the
OpenMV disk. It prints one serial line per frame:

OPENMV_SURFACE ... presets=NORMAL,DARK,SHINY
"""

import sensor
import time


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240, practical limit on the current H7 setup.
sensor.skip_frames(time=2000)

# Keep image statistics comparable over time. If the scene is extremely dark or
# bright, re-enable auto exposure for a short calibration run, then lock it again.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

IMG_W = 320
IMG_H = 240
ROI = (40, 30, 240, 180)
ROI_AREA = ROI[2] * ROI[3]

# OpenMV LAB thresholds: (L_min, L_max, A_min, A_max, B_min, B_max).
DARK_THRESHOLD = (0, 35, -128, 127, -128, 127)
BRIGHT_THRESHOLD = (82, 100, -128, 127, -128, 127)


def blob_pixel_ratio(img, threshold, color):
    blobs = img.find_blobs(
        [threshold],
        roi=ROI,
        pixels_threshold=350,
        area_threshold=350,
        merge=True,
        margin=8,
    )
    pixels = 0
    for blob in blobs:
        pixels += blob.pixels()
        img.draw_rectangle(blob.rect(), color=color)
        img.draw_cross(blob.cx(), blob.cy(), color=color)
    return pixels / ROI_AREA


def select_classes(l_mean, l_stdev, a_stdev, b_stdev, dark_ratio, bright_ratio):
    classes = []

    # Dark surfaces need high exposure. Use both blob area and global luminance
    # because black paint may appear as one large region or several dark blobs.
    if dark_ratio > 0.10 or l_mean < 38:
        classes.append("DARK")

    # Bright/specular zones often need a lower exposure scan.
    if bright_ratio > 0.025:
        classes.append("SHINY")

    # Keep a normal scan when there is enough non-extreme area, or when the
    # decision is uncertain.
    extreme_ratio = dark_ratio + bright_ratio
    color_variation = a_stdev + b_stdev
    if extreme_ratio < 0.75 or (l_stdev > 18 and color_variation > 8):
        classes.append("NORMAL")

    if not classes:
        classes.append("NORMAL")

    # Stable order makes laptop-side behavior deterministic.
    ordered = []
    for name in ("NORMAL", "DARK", "SHINY"):
        if name in classes:
            ordered.append(name)
    return ordered


while True:
    clock.tick()
    img = sensor.snapshot()
    stats = img.get_statistics(roi=ROI)

    dark_ratio = blob_pixel_ratio(img, DARK_THRESHOLD, (255, 0, 0))
    bright_ratio = blob_pixel_ratio(img, BRIGHT_THRESHOLD, (255, 255, 0))

    classes = select_classes(
        stats.l_mean(),
        stats.l_stdev(),
        stats.a_stdev(),
        stats.b_stdev(),
        dark_ratio,
        bright_ratio,
    )

    img.draw_rectangle(ROI, color=(255, 255, 255))
    img.draw_string(4, 4, ",".join(classes), color=(255, 255, 255), scale=1)

    print(
        "OPENMV_SURFACE v=1 "
        "fps=%.2f "
        "l_mean=%d l_std=%d a_std=%d b_std=%d "
        "dark_ratio=%.3f bright_ratio=%.3f "
        "classes=%s presets=%s"
        % (
            clock.fps(),
            stats.l_mean(),
            stats.l_stdev(),
            stats.a_stdev(),
            stats.b_stdev(),
            dark_ratio,
            bright_ratio,
            ",".join(classes),
            ",".join(classes),
        )
    )

    time.sleep_ms(200)
