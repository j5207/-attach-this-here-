import cv2
while True:
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    #cap.release()
    cv2.imshow("dd ", frame)
    if cv2.waitKey(2000) != -1:
        break