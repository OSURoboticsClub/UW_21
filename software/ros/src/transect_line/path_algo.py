import cv2
import numpy as np

def isolate_color(lower, upper, hsvFrame, frame):
    """
    @param
        - lower : lower bound of the color you want to isolate in HSV
        - upper : upper bound of the color you want to isolate in HSV
        - hsvFrame : frame of image in HSV
        - frame : regular frame of image
    @return
        - mask : the color mask
        - ret : image after mask
    """
    kernel = np.ones((5,5), "uint8")
    mask = cv2.inRange(hsvFrame, lower, upper)
    mask = cv2.dilate(mask, kernel)
    ret = cv2.bitwise_and(frame, frame, mask=mask)
    return mask, ret

def get_line(frame, mask, margin=(0,1)):
    """
    @param
        - frame : frame of image
        - mask : color mask
        - margin : default full frame, change to look only at certain region of frame
    @in function
        - draw the longest line in the color mask
    @return
        - attr : (x_start, y_start, height) of the line
    """
    left, right = int(frame.shape[1] * margin[0]), int(frame.shape[1] * margin[1])
    contours, hierarchy = cv2.findContours(mask[:,left:right], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    mx_h, attr = -1, None
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if h > mx_h:
            mx_h = h
            attr = (x, y, h)
    x, y, h = attr
    cv2.line(img=frame, pt1=(x,y), pt2=(x,y+h), color=(255,0,0), thickness=3)
    return attr

def draw_grid(frame, split_size, iterations, start_x):
    """
    @param
        - frame : frame of image
        - split_size : size of each splited region
        - iterations : number of time to split
        - start_x : starting x coordinate
    @in function
        - draw the lines of split
    @return
        - left splited region and right splited region
    """
    head = (start_x, 0)
    tail = (start_x, frame.shape[1])
    left_bound = (start_x, start_x+split_size)
    for _ in range(iterations):
        cv2.line(img=frame, pt1=head, pt2=tail, color=(255, 255, 255), thickness=2)
        head = (head[0]+split_size, head[1])
        tail = (tail[0]+split_size, tail[1])
    right_bound = (head[0]-split_size, head[0])
    return [left_bound, right_bound]

def check_bounds(frame, bounds, esize):
    """
    @param
        - frame : frame of image
        - bounds : the left and right bounds for draw_grid function
        - esize : margin of error size for determining left or right
    @return
        - returns left, right, or stay depending where the blue line is
    """
    def get_mid(left, right):
        return int(left + (right-left)//2)

    center = get_mid(*bounds[0])
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue_mask, _ = isolate_color(np.array([94, 80, 2], np.uint8), np.array([120, 255, 255], np.uint8), hsvFrame, frame)
    x, y, h = get_line(frame, blue_mask, margin=(0,0.5))

    if (x < center - esize):
        return (1, "right")
    elif (x > center + esize):
        return (-1, "left")
    else:
        return (0, "stay")
