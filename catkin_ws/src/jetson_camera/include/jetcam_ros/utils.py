import cv2


def bgr8_to_jpeg(value, quality=75):
    return cv2.imencode('.jpg', value)[1]