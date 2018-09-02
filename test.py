import cv2
import numpy as np

def draw_x(img, x, y, line):
    cv2.line(img, (x + int(line/2), y + int(line/2)),(x - int(line/2), y - int(line/2)), (255, 255, 255), 10)
    cv2.line(img, (x - int(line / 2), y + int(line / 2)), (x + int(line / 2), y - int(line / 2)), (255, 255, 255), 10)
    return img

def draw_arrow(img, x1, y1, x2, y2, line):
    cv2.arrowedLine(img, (int(0.6*x1 + 0.4*x2), int(0.6*y1 + 0.4*y2)), (int(0.4*x1 + 0.6*x2), int(0.4*y1+0.6*y2)),(255, 255, 255), line)
    return img


img = np.zeros((900, 1600, 3), np.uint8) * 255
cv2.circle(img, (int(1600*500/640),int(250*900/480)), 40, (255, 255, 255), -1)
img = draw_x(img,int(325*1600/640),int(350*900/480), 40)
img = draw_arrow(img, int(1600*500/640), int(250*900/480), int(1600*325/640),int(350*900/480),5)
cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.imshow("projector", img)
cv2.waitKey(0)


img = np.zeros((900, 1600, 3), np.uint8) * 255
cv2.circle(img, (int(1600*200/640),int(250*900/480)), 40, (255, 255, 255), -1)
cv2.circle(img, (int(1600*200/640+100),int(250*900/480)), 40, (255, 255, 255), -1)
img = draw_x(img,int(325*1600/640),int(350*900/480), 40)
img = draw_arrow(img, int(1600*200/640+30), int(250*900/480+30), int(1600*325/640),int(350*900/480),5)
cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.imshow("projector", img)
cv2.waitKey(0)

img = np.zeros((900, 1600, 3), np.uint8) * 255
cv2.circle(img, (int(1600*150/640 +50),int(150*900/480)), 120, (255, 255, 255), 5)
img = draw_x(img,int(325*1600/640),int(350*900/480), 40)
img = draw_arrow(img, int(1600*150/640 + 50), int(150*900/480 + 20), int(1600*325/640),int(350*900/480),5)
cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.imshow("projector", img)
cv2.waitKey(0)

img = np.zeros((900, 1600, 3), np.uint8) * 255
cv2.circle(img, (int(1600*400/640 + 40),int(300*900/480)), 120, (255, 255, 255), 5)
img = draw_x(img,int(325*1600/640),int(350*900/480), 40)
img = draw_arrow(img, int(1600*400/640 + 40), int(300*900/480), int(1600*325/640),int(350*900/480),5)
cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.imshow("projector", img)
cv2.waitKey(0)

img = np.zeros((900, 1600, 3), np.uint8) * 255
cv2.circle(img, (int(1600*275/640),int(190*900/480)), 40, (255, 255, 255), -1)
cv2.circle(img, (int(1600*275/640+80),int(190*900/480)), 40, (255, 255, 255), -1)
img = draw_x(img,int(325*1600/640),int(350*900/480), 40)
img = draw_arrow(img, int(1600*275/640+40), int(190*900/480), int(1600*325/640),int(350*900/480),5)
cv2.namedWindow("projector", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("projector", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
cv2.imshow("projector", img)
cv2.waitKey(0)