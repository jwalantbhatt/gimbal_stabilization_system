# imports
import cv2
import numpy as np
import math
import time


# variables
result_window = "Display Window"
#clear = lambda: os.system('clear')
start_time = 0

# functions
def drawdotted(img, pt1, pt2, color, thickness=1):

    dist = ((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2) ** .5
    pts = []

    for i in np.arange(0, dist, 5):
        r = i / dist
        x = int((pt1[0] * (1 - r) + pt2[0] * r) + .5)
        y = int((pt1[1] * (1 - r) + pt2[1] * r) + .5)
        p = (x, y)
        pts.append(p)

    for p in pts:
        cv2.circle(img, p, thickness, color, -1)


def compute_ptr(bb, img, w, h):

    # handle
    if len(bb) < 2:
        print ("No Markers Detected")
        time.sleep(1)
        return 1

    # preliminary computations
    w = float(w)
    h = float(h)
    height, width, channels = img.shape  # width and height of image
    # centers
    c0 = width / 2, height / 2  # center of image
    c1 = ((bb[0][0][0] + bb[0][1][0]) / 2), ((bb[0][0][1] + bb[0][1][1]) / 2)  # center of one marker
    c2 = ((bb[-1][0][0] + bb[-1][1][0]) / 2), ((bb[-1][0][1] + bb[-1][1][1]) / 2)  # center of other marker
    # midpoint
    m = (c1[0] + c2[0]) / 2, (c1[1] + c2[1]) / 2  # midpoint of line connecting markers
    # radius for circles
    radius1 = int((math.sqrt(w * w + h * h)) / float(2))  # radius for marker circles
    radius2 = int(radius1 / 2)

    # centers
    # for center
    color = (0, 0, 255)
    cv2.circle(img, c0, int(radius2), color, thickness=2, lineType=8, shift=0)
    cv2.line(img, (c0[0], c0[1] - radius1), (c0[0], c0[1] + radius1), color, 2)
    cv2.line(img, (c0[0] - radius1, c0[1]), (c0[0] + radius1, c0[1]), color, 2)
    # for markers midpoint and lines
    color = (255, 0, 0)
    cv2.circle(img, m, radius2 / 2, color, thickness=2, lineType=8, shift=0)
    cv2.circle(img, c1, radius1, color, thickness=2, lineType=8, shift=0)
    cv2.circle(img, c2, radius1, color, thickness=2, lineType=8, shift=0)
    drawdotted(img, c1, c2, color, thickness=1)

    # ptr
    color = (0, 200, 0)
    # pan
    x1 = c0[0]
    x2 = m[0]
    pan = x1 - x2
    ori = ''
    print("")
    if pan >= 0:
        cv2.arrowedLine(img, (x1, c0[1]), (x2, c0[1]), color, thickness=2, tipLength=0.2)
        ori = "towards right."
    elif pan < 0:
        cv2.arrowedLine(img, (x1, c0[1]), (x2, c0[1]), color, thickness=2, tipLength=0.2)
        ori = "towards left."
    print ("Pan: %d pixels %s" % (abs(pan), ori))

    # tilt
    y1 = c0[1]
    y2 = m[1]
    tilt = y1 - y2
    if tilt >= 0:
        cv2.arrowedLine(img, (m[0], y1), (m[0], y2), color, thickness=2, tipLength=0.1)
        ori = "downwards."

    elif tilt < 0:
        cv2.arrowedLine(img, (m[0], y1), (m[0], y2), color, thickness=2, tipLength=0.1)
        ori = "upwards."
    print ("Tilt: %d pixels %s" % (abs(tilt), ori))

    # roll
    if (c2[0] - c1[0]) == 0:
        print("Only one marker detected!")
        time.sleep(1)
        return 1
    slope = float(c2[1] - c1[1]) / (c2[0] - c1[0])
    angle = np.rad2deg(np.arctan(float(slope)))
    if -45 < angle < 45:
        if pan >= 0:
            if angle >= 0:
                ori1 = "horizontal"
                ori2 = "clockwise"
            else:
                ori1 = "horizontal"
                ori2 = "anti-clockwise"
            cv2.line(img, (m[0], m[1]), (m[0] - abs(m[0] - c1[0]), m[1]), (0, 0, 255), 2)
            cv2.ellipse(img, (m[0], m[1]), (abs(m[0] - c2[0]), abs(m[0] - c2[0])), 0, 180, 180 + angle, (0, 200, 0), 2)
        else:
            if angle >= 0:
                ori1 = "horizontal"
                ori2 = "anti-clockwise"
            else:
                ori1 = "horizontal"
                ori2 = "clockwise"
            cv2.line(img, (m[0], m[1]), (m[0] + abs(m[0] - c2[0]), m[1]), (0, 0, 255), 2)
            cv2.ellipse(img, (m[0], m[1]), (abs(m[0] - c2[0]), abs(m[0] - c2[0])), 0, 360 + angle, 360, (0, 200, 0), 2)
        print ("Roll: %d degrees from %s axis in %s direction." % (abs(angle), ori1, ori2))
    else:
        if tilt >= 0:
            if angle >= 0:
                j = (90 - angle)
                ori1 = "vertical"
                ori2 = "anti-clockwise"
            else:
                j = -(90 + angle)
                ori1 = "vertical"
                ori2 = "clockwise"
            cv2.line(img, (m[0], m[1]), (m[0], m[1] - abs(m[1] - c2[1])), (0, 0, 255), 2)
            cv2.ellipse(img, (m[0], m[1]), (abs(m[1] - c2[1]), abs(m[1] - c2[1])), 0, 270, 270 - j, (0, 200, 0), 2)

        else:
            if angle >= 0:
                j = (90 - angle)
                ori1 = "vertical2"
                ori2 = "anti-lockwise"
            else:
                j = -(90 + angle)
                ori1 = "vertical2"
                ori2 = "clockwise"
            cv2.line(img, (m[0], m[1]), (m[0], m[1] + abs(m[1] - c2[1])), (0, 0, 255), 2)
            cv2.ellipse(img, (m[0], m[1]), (abs(m[1] - c2[1]), abs(m[1] - c2[1])), 0, 90 - j, 90, (0, 200, 0), 2)
        print ("Roll: %d degrees from %s axis in %s direction." %((90 - abs(angle)), ori1, ori2))
    print("--- %s seconds ---" % (time.time() - start_time))
    cv2.imshow(result_window, img)
    cv2.waitKey(1)


def read_frames():
    # read from camera
    camera = cv2.VideoCapture(1)
    for i in range(10):
        return_value, image = camera.read()
        cv2.imwrite('opencv'+str(i)+'.png', image)


def template_matching(img_rgbs, str1):

    # template matching
    # read images
    img_rgb = img_rgbs
    templatec = cv2.imread(str1, cv2.IMREAD_COLOR)
    templateg = cv2.imread(str1, 0)

    # create windows
    cv2.namedWindow(result_window, cv2.WINDOW_AUTOSIZE)

    # width and height
    w, h = templateg.shape[::-1]

    res = cv2.matchTemplate(img_rgb, templatec, cv2.TM_CCOEFF_NORMED)
    threshold = 0.6
    loc = []
    bb = []
    loc = np.where(res >= threshold)
    for pt in zip(*loc[::-1]):
        # cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (255, 0, 0), 2)
        bb.append((pt, (pt[0] + w, pt[1] + h)))

    compute_ptr(bb, img_rgb, w, h)


def main():

    print("Gimbal Stabilization System")
    camera = cv2.VideoCapture(0)
    while True:
        #clear()
        k = cv2.waitKey(1) & 0xFF
        # press 'q' to exit
        if k == ord('q'):
            break
        start_time = time.time()
        return_value, image = camera.read()
        template_matching(image, "template_red (2).png")


if __name__ == '__main__':
    main()


