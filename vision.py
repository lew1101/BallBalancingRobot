import cv2
import numpy as np

from .constants import *

COLORS = ["red", "green", "blue"]

COLOR_RANGES = {
    "red": (((0, 145, 85), (10, 255, 255)), ((170, 135, 80), (180, 255, 255))),
    "green": ((40, 75, 75), (85, 255, 255)),
    "blue": ((100, 150, 0), (140, 255, 255)),
    # You may need to fine-tune these based on lighting
}

# Color draw map
COLOR_BGR = {
    "red": (0.0, 0.0, 255.0),
    "green": (0.0, 255.0, 0.0),
    "blue": (255.0, 0.0, 0.0),
}


def getMask(color, hsv):
    bounds = COLOR_RANGES[color]
    match color:
        case "red":
            (lower1, upper1), (lower2, upper2) = bounds
            mask1 = cv2.inRange(hsv, np.array(lower1), np.array(upper1))
            mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper2))
            return cv2.bitwise_or(mask1, mask2)
        case _:
            lower, upper = bounds
            return cv2.inRange(hsv, np.array(lower), np.array(upper))


def findBestBlob(mask, min_area=AREA_THRESHOLD, min_circularity=CIRCULARITY_THRESHOLD):

    def contourScore(contour):
        area = cv2.contourArea(contour)
        if area < min_area:
            return -1  # too small
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * np.pi * area / (perimeter * perimeter + 1e-6)
        return circularity if circularity > min_circularity else -1

    # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, None, mask
    # Use the largest contour
    best_contour = max(contours, key=contourScore, default=None)

    if contourScore(best_contour) < 0:
        return None, None, mask

    (x, y), radius = cv2.minEnclosingCircle(best_contour)

    return (int(x), int(y)), int(radius), mask


def getBallPos(frame,
               *,
               min_area=AREA_THRESHOLD,
               min_circularity=CIRCULARITY_THRESHOLD,
               debug=False):
    ball_color = None
    ball_centre = None
    ball_radius = 0

    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)

    for color in COLORS:
        mask = getMask(color, hsv)

        centre, radius, clean_mask = findBestBlob(mask, min_area, min_circularity)

        if debug:
            cv2.imshow(f"{color.capitalize()} Mask", clean_mask)

        if radius and radius > ball_radius:
            ball_color = color
            ball_centre = centre
            ball_radius = radius

    return ball_color, ball_centre, ball_radius
