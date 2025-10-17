import cv2
import numpy as np

def detect_red_regions(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    lower1 = np.array([0,120,70]); upper1 = np.array([10,255,255])
    lower2 = np.array([170,120,70]); upper2 = np.array([180,255,255])
    mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = [cv2.boundingRect(c) for c in cnts if cv2.contourArea(c) > 200]
    return boxes
