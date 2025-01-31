import cv2
import numpy as np

def onChange(x):
    pass

def setting_bar():
    cv2.namedWindow('HSV_settings')

    cv2.createTrackbar('H_MAX', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('H_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('H_MIN', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('H_MIN', 'HSV_settings', 0)
    cv2.createTrackbar('S_MAX', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('S_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('S_MIN', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('S_MIN', 'HSV_settings', 0)
    cv2.createTrackbar('V_MAX', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('V_MAX', 'HSV_settings', 255)
    cv2.createTrackbar('V_MIN', 'HSV_settings', 0, 255, onChange)
    cv2.setTrackbarPos('V_MIN', 'HSV_settings', 0)

setting_bar()


def showcam():
    try:
        cap = cv2.VideoCapture(0)
        print('open cam')
    except:
        print ('Not working')
        return
    # cap.set(3, 300)
    # cap.set(4, 300)

    while True:
        ret, frame = cap.read()
        H_MAX = cv2.getTrackbarPos('H_MAX', 'HSV_settings')
        H_MIN = cv2.getTrackbarPos('H_MIN', 'HSV_settings')
        S_MAX = cv2.getTrackbarPos('S_MAX', 'HSV_settings')
        S_MIN = cv2.getTrackbarPos('S_MIN', 'HSV_settings')
        V_MAX = cv2.getTrackbarPos('V_MAX', 'HSV_settings')
        V_MIN = cv2.getTrackbarPos('V_MIN', 'HSV_settings')
        lower = np.array([H_MIN, S_MIN, V_MIN])
        higher = np.array([H_MAX, S_MAX, V_MAX])
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        Gmask = cv2.inRange(hsv, lower, higher)
        G = cv2.bitwise_and(frame, frame, mask = Gmask)
        if not ret:
            print('error')
            break
        cv2.imshow('cam_load',frame)
        cv2.imshow('G',G)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

showcam()

