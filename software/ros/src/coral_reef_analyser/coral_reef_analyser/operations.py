import numpy as np
import cv2 as cv

def getIMG(file, size=(700,700), toFloat=False):
    img = cv.imread(file, cv.IMREAD_COLOR)
    img = cv.resize(img, size, interpolation = cv.INTER_AREA)
    if toFloat:
        img = np.float32(img)
    return img

def detectMatch(img1, img2):
    MIN_MATCHES = 50

    orb = cv.ORB_create(nfeatures=500)
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    index_params = dict(algorithm=6,
                        table_number=6,
                        key_size=12,
                        multi_probe_level=2)
    search_params = {}
    flann = cv.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1, des2, k=2)

    # As per Lowe's ratio test to filter good matches
    good_match = [m for m, n in matches if m.distance < n.distance*0.8]
    return (kp1, des1), (kp2, des2), good_match

def warpImage(img1, img2, match_req=10):
    (kp1, des1), (kp2, des2), good_matches = detectMatch(img1, img2)
    if len(good_matches) < match_req:
        print("Not Enought Matches")
        return None
    src_points = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_points = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    m, mask = cv.findHomography(src_points, dst_points, cv.RANSAC, 5.0)
    warped_img = cv.warpPerspective(img1, m, (img2.shape[1], img2.shape[0]))
    return warped_img

def filterColor(img):
    lab= cv.cvtColor(img, cv.COLOR_BGR2LAB)
    l, a, b = cv.split(lab)
    ret1, a = cv.threshold(a, 150, 255, cv.THRESH_BINARY)
    a = cv.cvtColor(a, cv.COLOR_GRAY2BGR)
    a[np.where((a==[255,255,255]).all(axis=2))] = [0,0,255]
    a[np.where((a==[0,0,0]).all(axis=2))] = [255,255,255]
    a = cv.GaussianBlur(a, (5, 5), 0)

    ret, b = cv.threshold(b,123,255,cv.THRESH_BINARY_INV)
    b = cv.cvtColor(b, cv.COLOR_GRAY2BGR)

    res = cv.bitwise_and(a, b)
    res = cv.medianBlur(res, 13)
    return res

def getDiff_weak(img1, img2, mark_img):
    mark_img = mark_img.copy()
    dif = cv.bitwise_xor(img1, img2)

    kernel = np.ones((13, 13), np.uint8)
    dif_cleaned = cv.morphologyEx(dif, cv.MORPH_OPEN, kernel)

    dif2 = dif_cleaned.copy()
    b, g, r = cv.split(dif2)
    cont_b ,hierarchy_b = cv.findContours(b, 1, 2)
    cont_r ,hierarchy_r = cv.findContours(r, 1, 2)

    contArr, color = [cont_b, cont_r], [(255,0,0), (0,0,255)]
    for i, cnt in enumerate(contArr):
        for pts in cnt:
            x, y, w, h = cv.boundingRect(pts)
            cv.rectangle(mark_img, (x, y), (x+w, y+h), color[i], 2)
    return mark_img
