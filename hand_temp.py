import cv2
import numpy as np


center = (255, 255)
surface = np.zeros((600, 600, 3))
cv2.circle(surface, center, 50, (255, 255, 255), -1)
cv2.imshow('hh', surface)
cv2.waitKey(0)