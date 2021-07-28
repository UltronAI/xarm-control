
import numpy as np

try:
    import cv2
except:
    import sys
    sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
    import cv2

from imutils import contours

def detect(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv_image.shape, dtype=np.uint8)

    # Color threshold to find the squares
    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))

    COLOR_RANGE = {
        "orange": ([0, 110, 90], [17, 255, 255])
    }
    for color, (lower, upper) in COLOR_RANGE.items():
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        color_mask = cv2.inRange(hsv_image, lower, upper)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, close_kernel, iterations=1)
        color_mask = cv2.merge([color_mask, color_mask, color_mask])

        mask = cv2.bitwise_or(mask, color_mask)

    gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

    cnts = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    if len(cnts) == 0:
        return None
    (cnts, _) = contours.sort_contours(cnts, method="top-to-bottom")

    # bounding_boxes = [cv2.boundingRect(c) for c in cnts]
    max_bbox = 0
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        if (w * h) > max_bbox:
            bbox = (x, y, w, h)
            max_bbox = w * h
    return [bbox]

def draw_bbox(image, bboxes):
    show_image = image.copy()
    number = 0
    for bbox in bboxes:
        x, y, w, h = bbox
        cv2.rectangle(show_image, (x, y), (x + w, y + h), (36,255,12), 2)
        cv2.putText(show_image, "#{}".format(number + 1), (x,y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        number += 1

    return show_image



if __name__ == "__main__":
    img_dir = "assets/"
    TEST_IMAGE = img_dir + "test_Color.png"
    COLOR_RANGE = {
        "orange": ([0, 110, 105], [17, 255, 255])
    }

    image = cv2.imread(TEST_IMAGE, cv2.IMREAD_COLOR)
    original = image.copy()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(image.shape, dtype=np.uint8)

    # Color threshold to find the squares
    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    for color, (lower, upper) in COLOR_RANGE.items():
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        color_mask = cv2.inRange(image, lower, upper)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, close_kernel, iterations=1)
        color_mask = cv2.merge([color_mask, color_mask, color_mask])

        mask = cv2.bitwise_or(mask, color_mask)

    gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

    cnts = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    (cnts, _) = contours.sort_contours(cnts, method="top-to-bottom")

    print(len(cnts))

    # Draw text
    number = 0
    for c in cnts:
        x,y,w,h = cv2.boundingRect(c)
        cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)

        cv2.putText(original, "#{}".format(number + 1), (x,y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        number += 1

    cv2.imwrite(img_dir + "mask.png", mask)
    cv2.imwrite(img_dir + "masked_color.png", original)