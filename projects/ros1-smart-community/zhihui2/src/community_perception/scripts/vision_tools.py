#!/usr/bin/env python3
import cv2
import numpy as np


def _find_colored_regions(image, lower_bgr, upper_bgr, min_area=300):
    lower = np.array(lower_bgr, dtype=np.uint8)
    upper = np.array(upper_bgr, dtype=np.uint8)
    mask = cv2.inRange(image, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        boxes.append((x, y, w, h, area))
    return boxes


def analyze_people(image):
    blue_boxes = _find_colored_regions(image, (80, 20, 0), (255, 140, 80), min_area=250)
    red_boxes = _find_colored_regions(image, (0, 0, 120), (90, 90, 255), min_area=250)
    annotated = image.copy()
    for x, y, w, h, _ in blue_boxes:
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (255, 80, 20), 2)
        cv2.putText(annotated, "resident", (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 80, 20), 2)
    for x, y, w, h, _ in red_boxes:
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 0, 255), 3)
        cv2.putText(annotated, "outsider", (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return {
        "type": "people",
        "resident_count": len(blue_boxes),
        "outsider_count": len(red_boxes),
        "summary": "residents=%d outsiders=%d" % (len(blue_boxes), len(red_boxes)),
        "annotated": annotated,
    }


def analyze_plate(image):
    blue_boxes = _find_colored_regions(image, (90, 20, 0), (255, 140, 90), min_area=180)
    annotated = image.copy()
    candidates = []
    for idx, (x, y, w, h, area) in enumerate(blue_boxes, 1):
        aspect = float(w) / float(h) if h else 0.0
        if aspect < 1.4:
            continue
        candidates.append((x, y, w, h, area))
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(annotated, "plate_%d" % idx, (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)
    return {
        "type": "plate",
        "plate_count": len(candidates),
        "summary": "plate_candidates=%d" % len(candidates),
        "annotated": annotated,
    }


def analyze_image(label, image):
    if label.startswith("people"):
        return analyze_people(image)
    if label.startswith("plate"):
        return analyze_plate(image)
    return {
        "type": "unknown",
        "summary": "no analyzer for label=%s" % label,
        "annotated": image.copy(),
    }
